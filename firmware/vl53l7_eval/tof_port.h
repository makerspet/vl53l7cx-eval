// One VL53L7 sensor on its own I2C bus, driven by its own FreeRTOS task.
//
// Each port can run either ST driver against whatever board is wired to it:
//   cx - "STM32duino VL53L7CX" library (ST ULD for VL53L7CX)
//   ch - "STM32duino VL53L7CH" library (ST VL53LMZ ULD, adds CNH histograms)
// Running a driver on the other part's board is the substitution test.
#pragma once
#include <Arduino.h>
#include <Wire.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "tof_driver.h"

class TofPort {
 public:
  TofPort(char name, uint8_t id, TwoWire* wire, int sda, int scl, int int_pin, int lpn_pin);

  // Starts the bus and the task. The task loads the driver and starts ranging.
  void begin(uint32_t i2c_hz, const PortConfig& cfg, BaseType_t core);

  // Thread-safe: queue a new config; the task applies it. `reinit` forces a
  // full sensor init (firmware download) even if the driver did not change.
  void set_config(const PortConfig& cfg, bool reinit);
  PortConfig get_config();
  void set_i2c_hz(uint32_t hz);
  void request_status();
  void set_text_interval(uint32_t ms) { text_ms_ = ms; }

 private:
  static void task_entry(void* arg);
  static void IRAM_ATTR isr(void* arg);
  void task_loop();

  void apply(const PortConfig& c, bool reinit);
  void stop_ranging();
  bool probe();
  void read_chip_ids();
  void read_and_emit();
  void emit_range(uint32_t read_us, int64_t t_us);
  void emit_cnh(int64_t t_us);
  void emit_text(uint32_t read_us);
  void emit_info(const char* event);
  void fail(const char* what, uint8_t status);

  const char name_;
  const uint8_t id_;
  TwoWire* wire_;
  const int sda_, scl_, int_pin_, lpn_pin_;
  uint32_t i2c_hz_ = 0;

  TofDriver* drivers_[3] = {nullptr, nullptr, nullptr};  // indexed by Driver
  TofDriver* drv_ = nullptr;                             // loaded driver, or null

  TaskHandle_t task_ = nullptr;
  SemaphoreHandle_t mtx_ = nullptr;

  // Written by other tasks under mtx_.
  PortConfig pending_;
  bool pending_flag_ = false;
  bool pending_reinit_ = false;
  uint32_t pending_hz_ = 0;
  bool status_req_ = false;

  // Owned by the port task (cfg_ is also read under mtx_).
  PortConfig cfg_;
  Driver loaded_ = DRV_OFF;  // driver whose firmware is in the sensor
  bool present_ = false;
  bool running_ = false;
  uint8_t dev_id_ = 0, rev_id_ = 0;
  uint32_t init_ms_ = 0;
  uint32_t seq_ = 0;
  uint32_t errors_ = 0;
  uint32_t cnh_errors_ = 0;
  uint32_t last_frame_ms_ = 0;
  uint32_t last_text_ms_ = 0;
  uint32_t stall_retries_ = 0;
  char last_error_[96] = "";
  volatile uint32_t int_count_ = 0;
  volatile uint32_t text_ms_ = 500;

  // Largest frame: CNH 64 aggregates x (18 bins + ambient) x 4 B (~4.9 KB),
  // or range data for 64 zones x 4 targets (~3.2 KB).
  uint8_t tx_[8192];
  float bins_[128];
};
