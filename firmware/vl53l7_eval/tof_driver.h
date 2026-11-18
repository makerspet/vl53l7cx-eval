// Neutral interface over the two ST libraries.
//
// The ST headers cannot share a translation unit: vl53l7cx_api.h and
// vl53lmz_api.h both define `union Block_header`, and vl53l7ch.h's
// platform.h also hides its typedefs behind a DEFAULT_I2C_BUFFER_LEN guard
// that vl53l7cx_class.h defines. So each library is wrapped in its own .cpp
// (cx_driver.cpp, ch_driver.cpp) and nothing else includes ST headers.
#pragma once
#include <Arduino.h>
#include <Wire.h>

enum Driver : uint8_t { DRV_OFF = 0, DRV_CX = 1, DRV_CH = 2 };
const char* driver_name(Driver d);

struct PortConfig {
  Driver driver = DRV_OFF;
  bool enabled = true;           // ranging on/off without unloading the firmware
  uint8_t zones = 64;            // 16 (4x4) or 64 (8x8)
  uint8_t hz = 10;               // 8x8: 1-15 Hz, 4x4: 1-60 Hz
  uint8_t mode = 1;              // 1 continuous, 3 autonomous
  uint16_t integration_ms = 20;  // autonomous mode only
  uint8_t sharpener = 5;         // percent
  uint8_t order = 2;             // 1 closest, 2 strongest
  // CNH histograms (CH driver only).
  bool cnh = true;
  int16_t cnh_start_bin = 0;  // first device histogram bin (37.5 mm each)
  int16_t cnh_bins = 18;      // output bins; 8x8 with one aggregate per zone allows up to 18
  int16_t cnh_sub = 2;        // device bins summed per output bin
  uint8_t cnh_merge = 1;      // 1: one aggregate per zone, 2: 2x2 zones, 4: 4x4
};

// Pointers into the driver's last results (ST struct-of-arrays layout; the
// per-target arrays are indexed [zone * targets + target]).
struct RangeView {
  int8_t temp_c;
  uint8_t targets;
  const uint8_t* nb_target_detected;
  const uint32_t* ambient_per_spad;
  const uint32_t* nb_spads_enabled;
  const int16_t* distance_mm;
  const uint16_t* range_sigma_mm;
  const uint32_t* signal_per_spad;
  const uint8_t* reflectance;
  const uint8_t* target_status;
};

struct CnhInfo {
  uint32_t bytes = 0;  // 0 = CNH not active
  uint8_t aggregates = 0;
  uint8_t bins = 0;
  uint8_t cols = 0;  // aggregate grid is cols x cols
};

class TofDriver {
 public:
  virtual ~TofDriver() {}
  // Toggle LPn (if wired), check the chip, download firmware. ~1 s at 1 MHz.
  virtual uint8_t init() = 0;
  // Apply settings and start ranging.
  virtual uint8_t configure(const PortConfig& c) = 0;
  virtual uint8_t stop() = 0;
  virtual uint8_t check_ready(uint8_t* ready) = 0;
  // Read one frame (and the CNH block if active) into the driver.
  virtual uint8_t read() = 0;
  virtual void range_view(RangeView* v) = 0;
  virtual uint8_t targets_per_zone() const = 0;
  virtual const char* uld_version() const = 0;
  // Name of the ST call that failed in the last init()/configure().
  const char* failed_step() const { return failed_step_; }

  // CNH (only the CH driver implements these).
  virtual CnhInfo cnh_info() const { return CnhInfo(); }
  virtual bool cnh_valid() const { return false; }  // last read() got a CNH block
  virtual uint32_t cnh_ref_residual() { return 0; }
  // Ambient and `bins` histogram values for one aggregate, scaled to float.
  virtual void cnh_aggregate(int agg, float* ambient, float* bins) {}

 protected:
  const char* failed_step_ = "";
};

TofDriver* make_cx_driver(TwoWire* wire, int lpn_pin);
TofDriver* make_ch_driver(TwoWire* wire, int lpn_pin);
