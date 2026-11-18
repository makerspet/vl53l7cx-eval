// Binary frame format shared with tools/vlog.py.
//
//   0xA5 0x5A | type u8 | len u16 | payload[len] | crc16 u16
//
// All integers little-endian. crc16 is CRC-16/CCITT-FALSE (poly 0x1021,
// init 0xFFFF) over type, len and payload. Keep in sync with tools/vlog.py.
#pragma once
#include <stdint.h>
#include <string.h>
#include <stddef.h>

#define PROTO_VERSION 1

#define FRAME_SYNC0 0xA5
#define FRAME_SYNC1 0x5A
#define FRAME_HDR_BYTES 5
#define FRAME_CRC_BYTES 2

enum FrameType : uint8_t {
  FT_RANGE = 0x01,  // per-zone ranging results (both drivers)
  FT_CNH = 0x02,    // compact normalised histograms (CH driver only)
  FT_INFO = 0x03,   // JSON: port config + state, sent after every (re)config
  FT_LOG = 0x04,    // free text
};

static inline uint16_t crc16_ccitt(const uint8_t* p, size_t n, uint16_t crc = 0xFFFF) {
  while (n--) {
    crc ^= (uint16_t)(*p++) << 8;
    for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// Builds one frame in a caller-owned buffer.
class FrameWriter {
 public:
  FrameWriter(uint8_t* buf, size_t cap) : buf_(buf), cap_(cap) {}

  void begin(uint8_t type) {
    n_ = 0;
    overflow_ = false;
    put<uint8_t>(FRAME_SYNC0);
    put<uint8_t>(FRAME_SYNC1);
    put<uint8_t>(type);
    put<uint16_t>(0);  // length, patched in finish()
  }

  void putn(const void* p, size_t k) {
    if (n_ + k > cap_) {
      overflow_ = true;
      return;
    }
    memcpy(buf_ + n_, p, k);
    n_ += k;
  }

  template <typename T>
  void put(T v) { putn(&v, sizeof v); }

  // Returns the total frame size, or 0 if the payload did not fit.
  size_t finish() {
    if (overflow_ || n_ + FRAME_CRC_BYTES > cap_ || n_ - FRAME_HDR_BYTES > 0xFFFF) return 0;
    uint16_t len = (uint16_t)(n_ - FRAME_HDR_BYTES);
    buf_[3] = len & 0xFF;
    buf_[4] = len >> 8;
    uint16_t crc = crc16_ccitt(buf_ + 2, n_ - 2);
    buf_[n_++] = crc & 0xFF;
    buf_[n_++] = crc >> 8;
    return n_;
  }

 private:
  uint8_t* buf_;
  size_t cap_;
  size_t n_ = 0;
  bool overflow_ = false;
};
