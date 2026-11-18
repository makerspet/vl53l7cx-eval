#include "output.h"

#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "protocol.h"

static SemaphoreHandle_t s_mtx;
static volatile OutMode s_mode = OUT_TEXT;

void out_init() { s_mtx = xSemaphoreCreateMutex(); }
OutMode out_mode() { return s_mode; }
void out_set_mode(OutMode m) { s_mode = m; }

static void write_locked(const uint8_t* d, size_t n) {
  xSemaphoreTake(s_mtx, portMAX_DELAY);
  Serial.write(d, n);
  xSemaphoreGive(s_mtx);
}

void out_frame(const uint8_t* data, size_t n) {
  if (s_mode == OUT_BIN && n) write_locked(data, n);
}

void out_text(const char* s) {
  if (s_mode == OUT_TEXT) write_locked((const uint8_t*)s, strlen(s));
}

static void text_or_frame(uint8_t type, const char* prefix, const char* s) {
  if (s_mode == OUT_TEXT) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    Serial.print(prefix);
    Serial.println(s);
    xSemaphoreGive(s_mtx);
    return;
  }
  uint8_t buf[1024 + FRAME_HDR_BYTES + FRAME_CRC_BYTES];
  size_t len = strlen(s);
  if (len > 1024) len = 1024;
  FrameWriter w(buf, sizeof buf);
  w.begin(type);
  w.putn(s, len);
  write_locked(buf, w.finish());
}

void out_log(const char* fmt, ...) {
  char line[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof line, fmt, ap);
  va_end(ap);
  text_or_frame(FT_LOG, "# ", line);
}

void out_log_str(const char* s) { text_or_frame(FT_LOG, "# ", s); }

void out_info(const char* json) { text_or_frame(FT_INFO, "# INFO ", json); }
