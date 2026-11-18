// VL53L7CH driver: "STM32duino VL53L7CH" library (ST VL53LMZ ULD) with CNH
// histogram output. Only this file may include the CH library headers.
#include <math.h>
#include <vl53l7ch.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "tof_driver.h"

#define STEP(call, label)     \
  do {                        \
    uint8_t _st = (call);     \
    if (_st) {                \
      failed_step_ = (label); \
      return _st;             \
    }                         \
  } while (0)

// The VL53LMZ ULD keeps the output-block list in file-scope globals
// (g_output_config), so two ports both on this driver must not configure at
// the same time. Once sent, the config lives in the sensor.
static SemaphoreHandle_t lmz_mutex() {
  static SemaphoreHandle_t m = xSemaphoreCreateMutex();
  return m;
}

class ChDriver : public TofDriver {
 public:
  ChDriver(TwoWire* wire, int lpn_pin) : dev_(wire, lpn_pin) {}

  uint8_t init() override {
    dev_.begin();  // toggles LPn if wired
    // Accepts any VL53L5/L7/L8 die; an L7 reads device 0xF0 / revision 0x02.
    STEP(dev_.init(), "init");
    return 0;
  }

  uint8_t configure(const PortConfig& c) override {
    info_ = CnhInfo();
    STEP(dev_.set_resolution(c.zones), "set_resolution");
    STEP(dev_.set_ranging_frequency_hz(c.hz), "set_ranging_frequency_hz");
    STEP(dev_.set_ranging_mode(c.mode), "set_ranging_mode");
    if (c.mode == VL53LMZ_RANGING_MODE_AUTONOMOUS)
      STEP(dev_.set_integration_time_ms(c.integration_ms), "set_integration_time_ms");
    STEP(dev_.set_sharpener_percent(c.sharpener), "set_sharpener_percent");
    STEP(dev_.set_target_order(c.order), "set_target_order");

    uint8_t st;
    if (!c.cnh) {
      xSemaphoreTake(lmz_mutex(), portMAX_DELAY);
      st = dev_.start_ranging();  // also uses the output-config globals
      xSemaphoreGive(lmz_mutex());
      if (st) failed_step_ = "start_ranging";
      return st;
    }

    // CNH setup, following ST's VL53L8CH_CNHData example (same VL53LMZ ULD).
    int side = (c.zones == 64) ? 8 : 4;
    int merge = c.cnh_merge;
    if (merge < 1 || merge > side || side % merge) {
      failed_step_ = "cnh merge must divide the grid side";
      return VL53LMZ_STATUS_INVALID_PARAM;
    }
    int cols = side / merge;
    STEP(dev_.cnh_init_config(&cfg_, c.cnh_start_bin, c.cnh_bins, c.cnh_sub), "cnh_init_config");
    STEP(dev_.cnh_create_agg_map(&cfg_, c.zones, 0, 0, merge, merge, cols, cols), "cnh_create_agg_map");
    uint32_t bytes = 0;
    st = dev_.cnh_calc_required_memory(&cfg_, &bytes);
    if (st || bytes > VL53LMZ_CNH_MAX_DATA_BYTES) {
      // ~ aggregates * (5 * bins + 5) + 28 bytes must fit in 6160.
      failed_step_ = "cnh_calc_required_memory (too many aggregates x bins; use fewer bins or merge 2)";
      return st ? st : VL53LMZ_STATUS_INVALID_PARAM;
    }
    STEP(dev_.cnh_send_config(&cfg_), "cnh_send_config");

    // Do not use VL53LMZ_CNH_DATA_BH: it shifts the 6160-byte size into the
    // 12-bit size field, overflowing into the index. ST's example builds the
    // header by hand: type 4 (32-bit words), size in words.
    union Block_header bh;
    bh.idx = VL53LMZ_CNH_DATA_IDX;
    bh.type = 4;
    bh.size = bytes / 4;

    xSemaphoreTake(lmz_mutex(), portMAX_DELAY);
    st = dev_.create_output_config();
    if (st) failed_step_ = "create_output_config";
    if (!st && (st = dev_.add_output_block(bh.bytes))) failed_step_ = "add_output_block";
    if (!st && (st = dev_.send_output_config_and_start())) failed_step_ = "send_output_config_and_start";
    xSemaphoreGive(lmz_mutex());
    if (st) return st;

    info_.bytes = bytes;
    info_.aggregates = cfg_.nb_of_aggregates;
    info_.bins = cfg_.feature_length;
    info_.cols = (uint8_t)cols;
    return 0;
  }

  uint8_t stop() override { return dev_.stop_ranging(); }
  uint8_t check_ready(uint8_t* ready) override { return dev_.check_data_ready(ready); }

  uint8_t read() override {
    cnh_valid_ = false;
    uint8_t st = dev_.get_ranging_data(&res_);
    if (!st && info_.bytes)
      cnh_valid_ = dev_.results_extract_block(VL53LMZ_CNH_DATA_IDX, (uint8_t*)buf_, (uint16_t)info_.bytes) == 0;
    return st;
  }

  void range_view(RangeView* v) override {
    v->temp_c = res_.silicon_temp_degc;
    v->targets = VL53LMZ_NB_TARGET_PER_ZONE;
    v->nb_target_detected = res_.nb_target_detected;
    v->ambient_per_spad = res_.ambient_per_spad;
    v->nb_spads_enabled = res_.nb_spads_enabled;
    v->distance_mm = res_.distance_mm;
    v->range_sigma_mm = res_.range_sigma_mm;
    v->signal_per_spad = res_.signal_per_spad;
    v->reflectance = res_.reflectance;
    v->target_status = res_.target_status;
  }

  uint8_t targets_per_zone() const override { return VL53LMZ_NB_TARGET_PER_ZONE; }
  const char* uld_version() const override { return VL53LMZ_API_REVISION; }

  CnhInfo cnh_info() const override { return info_; }
  bool cnh_valid() const override { return cnh_valid_; }
  uint32_t cnh_ref_residual() override { return dev_.cnh_get_ref_residual(buf_); }

  // Scaled with ST's own block-address helper, so the host never has to
  // mirror the CNH memory layout.
  void cnh_aggregate(int agg, float* ambient, float* bins) override {
    int32_t *hist, *amb;
    int8_t *hist_sc, *amb_sc;
    dev_.cnh_get_block_addresses(&cfg_, agg, buf_, &hist, &hist_sc, &amb, &amb_sc);
    *ambient = ldexpf((float)*amb, -*amb_sc);
    for (int b = 0; b < info_.bins; b++) bins[b] = ldexpf((float)hist[b], -hist_sc[b]);
  }

 private:
  VL53L7CH dev_;
  VL53LMZ_ResultsData res_;
  VL53LMZ_Motion_Configuration cfg_;
  cnh_data_buffer_t buf_;
  CnhInfo info_;
  bool cnh_valid_ = false;
};

TofDriver* make_ch_driver(TwoWire* wire, int lpn_pin) { return new ChDriver(wire, lpn_pin); }
