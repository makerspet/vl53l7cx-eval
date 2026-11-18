#include "tof_port.h"

#include "config.h"
#include "esp_timer.h"
#include "output.h"
#include "protocol.h"

static const uint8_t I2C_ADDR7 = 0x29;  // 0x52 in ST's 8-bit notation
static const uint32_t POLL_MS = 3;      // data-ready poll interval when INT is quiet
static const uint32_t STALL_MS = 3000;  // no frame this long -> reinit

const char* driver_name(Driver d) {
  switch (d) {
    case DRV_CX: return "cx";
    case DRV_CH: return "ch";
    default: return "off";
  }
}

TofPort::TofPort(char name, uint8_t id, TwoWire* wire, int sda, int scl, int int_pin, int lpn_pin)
    : name_(name), id_(id), wire_(wire), sda_(sda), scl_(scl), int_pin_(int_pin), lpn_pin_(lpn_pin) {}

void TofPort::begin(uint32_t i2c_hz, const PortConfig& cfg, BaseType_t core) {
  drivers_[DRV_CX] = make_cx_driver(wire_, lpn_pin_);
  drivers_[DRV_CH] = make_ch_driver(wire_, lpn_pin_);
  mtx_ = xSemaphoreCreateMutex();
  i2c_hz_ = i2c_hz;
  wire_->begin(sda_, scl_, i2c_hz);
  pending_ = cfg;
  pending_flag_ = true;
  pending_reinit_ = true;
  char tname[8] = {'t', 'o', 'f', name_, 0};
  xTaskCreatePinnedToCore(task_entry, tname, 8192, this, 2, &task_, core);
  if (int_pin_ >= 0) {
    pinMode(int_pin_, INPUT_PULLUP);
    attachInterruptArg(int_pin_, isr, this, FALLING);
  }
}

void TofPort::set_config(const PortConfig& cfg, bool reinit) {
  xSemaphoreTake(mtx_, portMAX_DELAY);
  pending_ = cfg;
  pending_flag_ = true;
  pending_reinit_ = pending_reinit_ || reinit;
  xSemaphoreGive(mtx_);
  xTaskNotifyGive(task_);
}

PortConfig TofPort::get_config() {
  xSemaphoreTake(mtx_, portMAX_DELAY);
  PortConfig c = pending_flag_ ? pending_ : cfg_;
  xSemaphoreGive(mtx_);
  return c;
}

void TofPort::set_i2c_hz(uint32_t hz) {
  xSemaphoreTake(mtx_, portMAX_DELAY);
  pending_hz_ = hz;
  xSemaphoreGive(mtx_);
  xTaskNotifyGive(task_);
}

void TofPort::request_status() {
  xSemaphoreTake(mtx_, portMAX_DELAY);
  status_req_ = true;
  xSemaphoreGive(mtx_);
  xTaskNotifyGive(task_);
}

void TofPort::task_entry(void* arg) { static_cast<TofPort*>(arg)->task_loop(); }

void IRAM_ATTR TofPort::isr(void* arg) {
  TofPort* p = static_cast<TofPort*>(arg);
  p->int_count_ = p->int_count_ + 1;
  BaseType_t woken = pdFALSE;
  if (p->task_) vTaskNotifyGiveFromISR(p->task_, &woken);
  if (woken) portYIELD_FROM_ISR();
}

void TofPort::task_loop() {
  for (;;) {
    PortConfig c;
    bool have = false, reinit = false, status = false;
    xSemaphoreTake(mtx_, portMAX_DELAY);
    if (pending_flag_) {
      c = pending_;
      reinit = pending_reinit_;
      have = true;
      pending_flag_ = pending_reinit_ = false;
    }
    uint32_t hz = pending_hz_;
    pending_hz_ = 0;
    status = status_req_;
    status_req_ = false;
    xSemaphoreGive(mtx_);

    if (hz) {
      stop_ranging();
      i2c_hz_ = hz;
      wire_->setClock(hz);
      if (!have) c = cfg_;
      have = reinit = true;  // re-download the firmware: a good link test at the new clock
    }
    if (have) apply(c, reinit);
    if (status) emit_info("status");

    if (!running_) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
      continue;
    }

    // Woken early by INT (or a command); otherwise poll.
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(POLL_MS));
    uint8_t ready = 0;
    uint8_t st = drv_->check_ready(&ready);
    if (st) {
      fail("check_data_ready", st);
      vTaskDelay(pdMS_TO_TICKS(10));
    } else if (ready) {
      read_and_emit();
    }

    if (running_ && millis() - last_frame_ms_ > STALL_MS && stall_retries_ < 5) {
      stall_retries_++;
      out_log("port %c: no frame for %lu ms, reinit (try %lu of 5)", name_, (unsigned long)STALL_MS,
              (unsigned long)stall_retries_);
      apply(cfg_, true);
    }
  }
}

// --- sensor bring-up --------------------------------------------------------

bool TofPort::probe() {
  // ST's read routine retries forever on NACK, so never let it talk to an
  // empty bus. A bare address probe returns instead.
  wire_->beginTransmission(I2C_ADDR7);
  return wire_->endTransmission() == 0;
}

// The same register sequence as the drivers' is_alive(), done here so the
// IDs get logged for both boards whichever driver runs.
void TofPort::read_chip_ids() {
  auto wr = [&](uint16_t reg, uint8_t v) {
    wire_->beginTransmission(I2C_ADDR7);
    wire_->write((uint8_t)(reg >> 8));
    wire_->write((uint8_t)(reg & 0xFF));
    wire_->write(v);
    return wire_->endTransmission();
  };
  dev_id_ = rev_id_ = 0;
  if (wr(0x7FFF, 0x00)) return;
  wire_->beginTransmission(I2C_ADDR7);
  wire_->write((uint8_t)0x00);
  wire_->write((uint8_t)0x00);
  if (wire_->endTransmission(false) == 0 && wire_->requestFrom((uint8_t)I2C_ADDR7, (uint8_t)2) == 2) {
    dev_id_ = wire_->read();
    rev_id_ = wire_->read();
  }
  wr(0x7FFF, 0x02);
}

void TofPort::stop_ranging() {
  if (running_ && drv_) drv_->stop();
  running_ = false;
}

void TofPort::fail(const char* what, uint8_t status) {
  errors_++;
  snprintf(last_error_, sizeof last_error_, "%s: status %u", what, status);
  out_log("port %c: %s", name_, last_error_);
}

void TofPort::apply(const PortConfig& c, bool reinit) {
  stop_ranging();
  xSemaphoreTake(mtx_, portMAX_DELAY);
  cfg_ = c;
  xSemaphoreGive(mtx_);
  last_error_[0] = 0;

  if (c.driver == DRV_OFF) {
    loaded_ = DRV_OFF;
    drv_ = nullptr;
    emit_info("off");
    return;
  }

  if (reinit || c.driver != loaded_) {
    loaded_ = DRV_OFF;
    drv_ = nullptr;
    // The CX driver leaves LPn low after begin(); release it so the probe can see the sensor.
    if (lpn_pin_ >= 0) {
      pinMode(lpn_pin_, OUTPUT);
      digitalWrite(lpn_pin_, HIGH);
      delay(10);
    }
    present_ = probe();
    if (!present_) {
      snprintf(last_error_, sizeof last_error_, "no ACK at 0x29 (check wiring, power, pull-ups)");
      out_log("port %c: %s", name_, last_error_);
      emit_info("absent");
      return;
    }
    read_chip_ids();
    out_log("port %c: device_id 0x%02X revision_id 0x%02X, loading %s driver firmware...", name_, dev_id_,
            rev_id_, driver_name(c.driver));
    TofDriver* d = drivers_[c.driver];
    uint32_t t0 = millis();
    uint8_t st = d->init();
    init_ms_ = millis() - t0;
    if (st) {
      fail(d->failed_step(), st);
      emit_info("init_failed");
      return;
    }
    out_log("port %c: %s driver loaded in %lu ms", name_, driver_name(c.driver), (unsigned long)init_ms_);
    loaded_ = c.driver;
    drv_ = d;
  }

  if (!c.enabled) {
    emit_info("stopped");
    return;
  }

  uint8_t st = drv_->configure(c);
  if (st) {
    fail(drv_->failed_step(), st);
    emit_info("config_failed");
    return;
  }
  running_ = true;
  last_frame_ms_ = millis();
  emit_info("started");
}

// --- data path --------------------------------------------------------------

void TofPort::read_and_emit() {
  int64_t t_us = esp_timer_get_time();
  uint32_t t0 = micros();
  uint8_t st = drv_->read();
  uint32_t read_us = micros() - t0;
  if (st) {
    fail("get_ranging_data", st);
    return;
  }
  if (drv_->cnh_info().bytes && !drv_->cnh_valid()) cnh_errors_++;
  seq_++;
  last_frame_ms_ = millis();
  stall_retries_ = 0;

  if (out_mode() == OUT_BIN) {
    emit_range(read_us, t_us);
    if (drv_->cnh_valid()) emit_cnh(t_us);
  } else if (millis() - last_text_ms_ >= text_ms_) {
    last_text_ms_ = millis();
    emit_text(read_us);
  }
}

void TofPort::emit_range(uint32_t read_us, int64_t t_us) {
  RangeView v;
  drv_->range_view(&v);
  const uint8_t nz = cfg_.zones, nt = v.targets;
  FrameWriter w(tx_, sizeof tx_);
  w.begin(FT_RANGE);
  w.put<uint8_t>(PROTO_VERSION);
  w.put<uint8_t>(id_);
  w.put<uint8_t>(loaded_);
  w.put<uint8_t>(nz);
  w.put<uint8_t>(nt);
  w.put<int8_t>(v.temp_c);
  w.put<uint16_t>(0);
  w.put<uint32_t>(seq_);
  w.put<int64_t>(t_us);
  w.put<uint32_t>(read_us);
  // Struct-of-arrays, in ST's layout.
  w.putn(v.nb_target_detected, nz);
  w.putn(v.ambient_per_spad, nz * 4);
  w.putn(v.nb_spads_enabled, nz * 4);
  w.putn(v.distance_mm, nz * nt * 2);
  w.putn(v.range_sigma_mm, nz * nt * 2);
  w.putn(v.signal_per_spad, nz * nt * 4);
  w.putn(v.reflectance, nz * nt);
  w.putn(v.target_status, nz * nt);
  out_frame(tx_, w.finish());
}

void TofPort::emit_cnh(int64_t t_us) {
  const CnhInfo ci = drv_->cnh_info();
  FrameWriter w(tx_, sizeof tx_);
  w.begin(FT_CNH);
  w.put<uint8_t>(PROTO_VERSION);
  w.put<uint8_t>(id_);
  w.put<uint8_t>(cfg_.zones);
  w.put<uint8_t>(ci.aggregates);
  w.put<uint8_t>(ci.bins);
  w.put<uint8_t>((uint8_t)cfg_.cnh_sub);
  w.put<uint8_t>(cfg_.cnh_merge);
  w.put<uint8_t>(ci.cols);
  w.put<int16_t>(cfg_.cnh_start_bin);
  w.put<uint16_t>(0);
  w.put<uint32_t>(seq_);
  w.put<int64_t>(t_us);
  w.put<uint32_t>(drv_->cnh_ref_residual());
  // Per aggregate: ambient, then its bins.
  for (int a = 0; a < ci.aggregates; a++) {
    float amb;
    drv_->cnh_aggregate(a, &amb, bins_);
    w.put<float>(amb);
    w.putn(bins_, ci.bins * sizeof(float));
  }
  out_frame(tx_, w.finish());
}

// Serial Monitor view: distance grid ('.' = no valid target) and the
// histogram of the centre aggregate.
void TofPort::emit_text(uint32_t read_us) {
  RangeView v;
  drv_->range_view(&v);
  const uint8_t nz = cfg_.zones, side = (nz == 64) ? 8 : 4, nt = v.targets;
  char* s = (char*)tx_;
  const size_t cap = sizeof tx_;
  size_t n = 0;
  n += snprintf(s + n, cap - n, "\n[%c %s %ux%u] frame %lu  read %lu us  %d C\n", name_, driver_name(loaded_), side,
                side, (unsigned long)seq_, (unsigned long)read_us, v.temp_c);
  for (int z = 0; z < nz; z++) {
    uint8_t stat = v.target_status[z * nt];
    if (v.nb_target_detected[z] && (stat == 5 || stat == 9))
      n += snprintf(s + n, cap - n, "%6d", v.distance_mm[z * nt]);
    else
      n += snprintf(s + n, cap - n, "     .");
    if (z % side == side - 1) n += snprintf(s + n, cap - n, "\n");
  }
  const CnhInfo ci = drv_->cnh_info();
  if (drv_->cnh_valid() && ci.cols) {
    int a = (ci.aggregates / ci.cols / 2) * ci.cols + ci.cols / 2;
    float amb;
    drv_->cnh_aggregate(a, &amb, bins_);
    n += snprintf(s + n, cap - n, "CNH aggregate %d, ambient %.1f, bins:", a, amb);
    for (int b = 0; b < ci.bins && n < cap - 16; b++) n += snprintf(s + n, cap - n, " %.0f", bins_[b]);
    n += snprintf(s + n, cap - n, "\n");
  }
  out_text(s);
}

void TofPort::emit_info(const char* event) {
  const PortConfig& c = cfg_;
  const CnhInfo ci = drv_ ? drv_->cnh_info() : CnhInfo();
  char j[768];
  snprintf(j, sizeof j,
           "{\"proto\":%d,\"fw\":\"%s\",\"port\":\"%c\",\"port_id\":%u,\"event\":\"%s\",\"driver\":\"%s\","
           "\"loaded\":\"%s\",\"uld\":\"%s\",\"present\":%s,\"device_id\":%u,\"revision_id\":%u,"
           "\"running\":%s,\"enabled\":%s,\"zones\":%u,\"hz\":%u,\"mode\":\"%s\",\"integration_ms\":%u,"
           "\"sharpener\":%u,\"order\":\"%s\",\"targets_per_zone\":%u,"
           "\"cnh\":{\"requested\":%s,\"active\":%s,\"start_bin\":%d,\"bins\":%d,\"sub\":%d,\"merge\":%u,"
           "\"aggregates\":%u,\"bytes\":%lu,\"bin_mm\":37.5348},"
           "\"i2c_hz\":%lu,\"init_ms\":%lu,\"frames\":%lu,\"errors\":%lu,\"cnh_errors\":%lu,"
           "\"int_count\":%lu,\"ms_since_frame\":%lu,\"last_error\":\"%s\"}",
           PROTO_VERSION, FW_VERSION, name_, id_, event, driver_name(c.driver), driver_name(loaded_),
           drv_ ? drv_->uld_version() : "", present_ ? "true" : "false", dev_id_, rev_id_,
           running_ ? "true" : "false", c.enabled ? "true" : "false", c.zones, c.hz,
           c.mode == 3 ? "autonomous" : "continuous", c.integration_ms, c.sharpener,
           c.order == 1 ? "closest" : "strongest", drv_ ? drv_->targets_per_zone() : 0, c.cnh ? "true" : "false",
           ci.bytes ? "true" : "false", c.cnh_start_bin, c.cnh_bins, c.cnh_sub, c.cnh_merge, ci.aggregates,
           (unsigned long)ci.bytes, (unsigned long)i2c_hz_, (unsigned long)init_ms_, (unsigned long)seq_,
           (unsigned long)errors_, (unsigned long)cnh_errors_, (unsigned long)int_count_,
           (unsigned long)(millis() - last_frame_ms_), last_error_);
  out_info(j);
}
