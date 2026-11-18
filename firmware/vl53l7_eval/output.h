// Serial output shared by the main loop and both sensor tasks.
// Text mode is for the Arduino Serial Monitor; binary mode is for tools/capture.py.
#pragma once
#include <Arduino.h>

enum OutMode : uint8_t { OUT_TEXT = 0, OUT_BIN = 1 };

void out_init();
OutMode out_mode();
void out_set_mode(OutMode m);

// Binary frame; dropped in text mode.
void out_frame(const uint8_t* data, size_t n);
// Multi-line human-readable block; dropped in binary mode.
void out_text(const char* s);
// "# ..." line in text mode, FT_LOG frame in binary mode.
void out_log(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
// Same as out_log for a preformatted string of up to 1 KB.
void out_log_str(const char* s);
// "# INFO {...}" line in text mode, FT_INFO frame in binary mode.
void out_info(const char* json);
