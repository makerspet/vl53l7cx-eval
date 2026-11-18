// VL53L7CX vs VL53L7CH evaluation firmware for a 30-pin ESP32 DevKit.
//
// Reads both sensors at once, each on its own I2C bus, and streams ranging
// results (and CNH histograms from the CH driver) over USB serial.
//   - Arduino Serial Monitor at 1000000 baud: text view + commands (type `help`)
//   - tools/capture.py: switches to binary mode and records everything
//
// Needs the "STM32duino VL53L7CX" and "STM32duino VL53L7CH" libraries
// (Arduino Library Manager) and the esp32 board package. See README.md.

#include "config.h"
#include "output.h"
#include "tof_port.h"

static TofPort portA('A', 0, &Wire, PORT_A_SDA, PORT_A_SCL, PORT_A_INT, PORT_A_LPN);
static TofPort portB('B', 1, &Wire1, PORT_B_SDA, PORT_B_SCL, PORT_B_INT, PORT_B_LPN);
static TofPort* const ports[2] = {&portA, &portB};

static const char HELP[] =
    "Commands (an optional 'a' or 'b' prefix targets one port, otherwise both):\n"
    "#   help | status | out text|bin | text_ms <ms> | i2c <hz>\n"
    "#   drv <A> <B>            set both drivers, each cx|ch|off  (e.g. 'drv ch cx' = swap)\n"
    "#   [a|b] drv cx|ch|off    driver for one port\n"
    "#   [a|b] res 4|8          4x4 or 8x8 zones\n"
    "#   [a|b] hz <n>           ranging rate (8x8 <= 15, 4x4 <= 60)\n"
    "#   [a|b] mode cont|auto   continuous or autonomous\n"
    "#   [a|b] it <ms>          integration time (autonomous only)\n"
    "#   [a|b] sharp <0..99>    sharpener percent\n"
    "#   [a|b] order closest|strongest\n"
    "#   [a|b] cnh off | cnh <start_bin> <bins> <sub_sample> [merge 1|2|4]   (ch driver)\n"
    "#   [a|b] start | stop | reinit";

static bool parse_driver(const char* s, Driver* d) {
  if (!strcmp(s, "cx")) *d = DRV_CX;
  else if (!strcmp(s, "ch")) *d = DRV_CH;
  else if (!strcmp(s, "off")) *d = DRV_OFF;
  else return false;
  return true;
}

static bool parse_int(const char* s, long lo, long hi, long* out) {
  if (!s) return false;
  char* end;
  long v = strtol(s, &end, 10);
  if (*end || v < lo || v > hi) return false;
  *out = v;
  return true;
}

// Applies one port-level command to port p. Returns an error string or nullptr.
static const char* port_command(TofPort* p, char** tok, int ntok) {
  const char* cmd = tok[0];
  PortConfig c = p->get_config();
  bool reinit = false;
  long v;

  if (!strcmp(cmd, "drv")) {
    if (ntok != 2 || !parse_driver(tok[1], &c.driver)) return "usage: drv cx|ch|off";
  } else if (!strcmp(cmd, "res")) {
    if (ntok != 2 || !parse_int(tok[1], 4, 8, &v) || (v != 4 && v != 8)) return "usage: res 4|8";
    c.zones = (uint8_t)(v * v);
    if (c.zones == 64 && c.hz > 15) c.hz = 15;
  } else if (!strcmp(cmd, "hz")) {
    if (ntok != 2 || !parse_int(tok[1], 1, 60, &v)) return "usage: hz 1..60";
    c.hz = (uint8_t)v;
  } else if (!strcmp(cmd, "mode")) {
    if (ntok == 2 && !strcmp(tok[1], "cont")) c.mode = 1;
    else if (ntok == 2 && !strcmp(tok[1], "auto")) c.mode = 3;
    else return "usage: mode cont|auto";
  } else if (!strcmp(cmd, "it")) {
    if (ntok != 2 || !parse_int(tok[1], 2, 1000, &v)) return "usage: it 2..1000";
    c.integration_ms = (uint16_t)v;
  } else if (!strcmp(cmd, "sharp")) {
    if (ntok != 2 || !parse_int(tok[1], 0, 99, &v)) return "usage: sharp 0..99";
    c.sharpener = (uint8_t)v;
  } else if (!strcmp(cmd, "order")) {
    if (ntok == 2 && !strcmp(tok[1], "closest")) c.order = 1;
    else if (ntok == 2 && !strcmp(tok[1], "strongest")) c.order = 2;
    else return "usage: order closest|strongest";
  } else if (!strcmp(cmd, "cnh")) {
    if (ntok == 2 && !strcmp(tok[1], "off")) {
      c.cnh = false;
    } else {
      long sb, nb, ss, mg = 1;
      if (ntok < 4 || ntok > 5 || !parse_int(tok[1], 0, 127, &sb) || !parse_int(tok[2], 1, 128, &nb) ||
          !parse_int(tok[3], 1, 16, &ss) || (ntok == 5 && !parse_int(tok[4], 1, 4, &mg)))
        return "usage: cnh off | cnh <start_bin 0..127> <bins 1..128> <sub 1..16> [merge 1|2|4]";
      c.cnh = true;
      c.cnh_start_bin = (int16_t)sb;
      c.cnh_bins = (int16_t)nb;
      c.cnh_sub = (int16_t)ss;
      c.cnh_merge = (uint8_t)mg;
    }
  } else if (!strcmp(cmd, "start")) {
    c.enabled = true;
  } else if (!strcmp(cmd, "stop")) {
    c.enabled = false;
  } else if (!strcmp(cmd, "reinit")) {
    reinit = true;
  } else {
    return "unknown command (try 'help')";
  }
  p->set_config(c, reinit);
  return nullptr;
}

static void handle_line(char* line) {
  char* tok[8];
  int ntok = 0;
  char* save;
  for (char* t = strtok_r(line, " \t", &save); t && ntok < 8; t = strtok_r(nullptr, " \t", &save)) tok[ntok++] = t;
  if (!ntok) return;

  // Global commands.
  if (!strcmp(tok[0], "help")) {
    out_log_str(HELP);
    return;
  }
  if (!strcmp(tok[0], "status")) {
    for (TofPort* p : ports) p->request_status();
    return;
  }
  if (!strcmp(tok[0], "out")) {
    if (ntok == 2 && !strcmp(tok[1], "bin")) out_set_mode(OUT_BIN);
    else if (ntok == 2 && !strcmp(tok[1], "text")) out_set_mode(OUT_TEXT);
    else out_log("err: usage: out text|bin");
    out_log("ok out %s", out_mode() == OUT_BIN ? "bin" : "text");
    return;
  }
  if (!strcmp(tok[0], "text_ms")) {
    long v;
    if (ntok == 2 && parse_int(tok[1], 0, 60000, &v)) {
      for (TofPort* p : ports) p->set_text_interval((uint32_t)v);
      out_log("ok text_ms %ld", v);
    } else {
      out_log("err: usage: text_ms <ms>");
    }
    return;
  }
  if (!strcmp(tok[0], "i2c")) {
    long v;
    if (ntok == 2 && parse_int(tok[1], 10000, 1000000, &v)) {
      for (TofPort* p : ports) p->set_i2c_hz((uint32_t)v);
      out_log("ok i2c %ld (sensors will re-init)", v);
    } else {
      out_log("err: usage: i2c <10000..1000000>");
    }
    return;
  }
  if (!strcmp(tok[0], "drv") && ntok == 3) {  // drv <A> <B>
    Driver da, db;
    if (!parse_driver(tok[1], &da) || !parse_driver(tok[2], &db)) {
      out_log("err: usage: drv <cx|ch|off> <cx|ch|off>");
      return;
    }
    PortConfig ca = portA.get_config(), cb = portB.get_config();
    ca.driver = da;
    cb.driver = db;
    portA.set_config(ca, false);
    portB.set_config(cb, false);
    out_log("ok drv %s %s", tok[1], tok[2]);
    return;
  }

  // Port commands, optionally prefixed with a|b.
  TofPort* targets[2] = {&portA, &portB};
  int ntargets = 2;
  if (!strcmp(tok[0], "a") || !strcmp(tok[0], "b")) {
    targets[0] = (tok[0][0] == 'a') ? &portA : &portB;
    ntargets = 1;
    for (int i = 1; i < ntok; i++) tok[i - 1] = tok[i];
    if (--ntok == 0) return;
  }
  for (int i = 0; i < ntargets; i++) {
    const char* err = port_command(targets[i], tok, ntok);
    if (err) {
      out_log("err: %s", err);
      return;
    }
  }
  out_log("ok %s", tok[0]);
}

void setup() {
  Serial.setTxBufferSize(16384);  // must precede begin()
  Serial.setRxBufferSize(1024);
  Serial.begin(SERIAL_BAUD);
  out_init();
  delay(300);
  out_log("vl53l7-eval %s ready; type 'help'", FW_VERSION);

  PortConfig a, b;
  a.driver = DRV_CX;
  b.driver = DRV_CH;
  portA.set_text_interval(TEXT_MS_DEFAULT);
  portB.set_text_interval(TEXT_MS_DEFAULT);
  // One task per bus so the two I2C transfers overlap.
  portA.begin(I2C_HZ_DEFAULT, a, 0);
  portB.begin(I2C_HZ_DEFAULT, b, 1);
}

void loop() {
  static char line[128];
  static size_t n = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (n) {
        line[n] = 0;
        handle_line(line);
        n = 0;
      }
    } else if (n < sizeof line - 1) {
      line[n++] = ch;
    }
  }
  delay(5);
}
