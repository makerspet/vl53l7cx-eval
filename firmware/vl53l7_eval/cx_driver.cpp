// VL53L7CX driver: "STM32duino VL53L7CX" library (ST ULD for VL53L7CX).
// Only this file may include the CX library headers (see tof_driver.h).
#include <vl53l7cx_class.h>

#include "tof_driver.h"

#define STEP(call, label)     \
  do {                        \
    uint8_t _st = (call);     \
    if (_st) {                \
      failed_step_ = (label); \
      return _st;             \
    }                         \
  } while (0)

class CxDriver : public TofDriver {
 public:
  CxDriver(TwoWire* wire, int lpn_pin) : dev_(wire, lpn_pin) {}

  uint8_t init() override {
    dev_.begin();
    // Toggles LPn, checks device 0xF0 / revision 0x02, downloads firmware.
    STEP((uint8_t)dev_.init_sensor(), "init_sensor");
    return 0;
  }

  uint8_t configure(const PortConfig& c) override {
    STEP(dev_.vl53l7cx_set_resolution(c.zones), "set_resolution");
    STEP(dev_.vl53l7cx_set_ranging_frequency_hz(c.hz), "set_ranging_frequency_hz");
    STEP(dev_.vl53l7cx_set_ranging_mode(c.mode), "set_ranging_mode");
    if (c.mode == VL53L7CX_RANGING_MODE_AUTONOMOUS)
      STEP(dev_.vl53l7cx_set_integration_time_ms(c.integration_ms), "set_integration_time_ms");
    STEP(dev_.vl53l7cx_set_sharpener_percent(c.sharpener), "set_sharpener_percent");
    STEP(dev_.vl53l7cx_set_target_order(c.order), "set_target_order");
    STEP(dev_.vl53l7cx_start_ranging(), "start_ranging");
    return 0;
  }

  uint8_t stop() override { return dev_.vl53l7cx_stop_ranging(); }
  uint8_t check_ready(uint8_t* ready) override { return dev_.vl53l7cx_check_data_ready(ready); }
  uint8_t read() override { return dev_.vl53l7cx_get_ranging_data(&res_); }

  void range_view(RangeView* v) override {
    v->temp_c = res_.silicon_temp_degc;
    v->targets = VL53L7CX_NB_TARGET_PER_ZONE;
    v->nb_target_detected = res_.nb_target_detected;
    v->ambient_per_spad = res_.ambient_per_spad;
    v->nb_spads_enabled = res_.nb_spads_enabled;
    v->distance_mm = res_.distance_mm;
    v->range_sigma_mm = res_.range_sigma_mm;
    v->signal_per_spad = res_.signal_per_spad;
    v->reflectance = res_.reflectance;
    v->target_status = res_.target_status;
  }

  uint8_t targets_per_zone() const override { return VL53L7CX_NB_TARGET_PER_ZONE; }
  const char* uld_version() const override { return VL53L7CX_API_REVISION; }

 private:
  VL53L7CX dev_;
  VL53L7CX_ResultsData res_;
};

TofDriver* make_cx_driver(TwoWire* wire, int lpn_pin) { return new CxDriver(wire, lpn_pin); }
