# VL53L7CX vs VL53L7CH evaluation

Firmware and tools to compare ST's VL53L7CX (8×8 multizone ToF, depth output) with the VL53L7CH (same, plus histogram output) for robot vacuum obstacle avoidance. Two questions:

1. **Does the CH's histogram output (CNH) detect obstacles the CX's depth output misses?**
2. **Can one part substitute for the other** when sourcing?

An ESP32 reads both sensors at once, each on its own I2C bus, and streams everything over USB. A Python tool records captures. A second script measures noise and validity per zone, and checks whether the histograms carry returns that the depth output dropped.

## Hardware

- 30-pin ESP32 DevKit (ESP32-WROOM-32E)
- VL53L7CX breakout (pins VIN, GND, SCL, SDA, INT, L_Pn) on **port A**
- VL53L7CH breakout (pins INT, SDA, SCL, VCC, GND) on **port B**

| Sensor pin | CX board → ESP32 | CH board → ESP32 |
|---|---|---|
| VIN / VCC | 3V3 (see power note) | 3V3 (see power note) |
| GND | GND | GND |
| SDA | GPIO21 | GPIO25 |
| SCL | GPIO22 | GPIO26 |
| INT | GPIO4 | GPIO27 |
| L_Pn | GPIO23 (optional) | n/a |

Pins are set in [firmware/vl53l7_eval/config.h](firmware/vl53l7_eval/config.h). Each sensor has its own bus, so both keep the default address 0x29 and no LPn address sequence is needed. INT is optional; without it the firmware polls.

**Power.** Check each board before connecting. If it has its own 3.3 V regulator, feed VIN/VCC from 5 V. Otherwise use 3V3: the sensor's AVDD wants 3.3 V and a regulator fed from 3.3 V drops below spec. Either way, SDA and SCL must idle at 3.3 V or less, because the ESP32 is not 5 V tolerant. With power on, measure them before wiring to the ESP32.

**I2C speed.** The firmware runs at 1 MHz: one CH frame with histograms is about 7.5 KB, and at 400 kHz the CH drops to about 4 Hz. Keep jumper wires short. If init fails or you see errors, send `i2c 400000`. The breakouts need pull-ups on SDA/SCL, which most have; if yours don't, add 2.2 kΩ to 3.3 V.

**Placement.** Mount both side by side at robot height, facing the same way, 1–2 cm apart. Both flood the scene with 940 nm light, so one sensor's emitter adds noise to the other; compare runs with one port stopped (`a stop` / `b stop`) to measure this.

## Firmware

1. Arduino IDE → Boards Manager: **esp32** by Espressif. Built against 2.0.17; 3.x should work but is untested.
2. Library Manager: **STM32duino VL53L7CX** (1.0.3) and **STM32duino VL53L7CH** (1.0.0).
3. Open [firmware/vl53l7_eval/vl53l7_eval.ino](firmware/vl53l7_eval/vl53l7_eval.ino), board **ESP32 Dev Module**, upload.
4. Serial Monitor at **1000000** baud shows a distance grid per sensor (`.` = no valid target) and the CH's centre-zone histogram. Type `help` for commands.

Each port can run either ST driver against whichever board is attached:

| Command | Effect |
|---|---|
| `drv cx ch` | default: CX driver on the CX board, CH driver on the CH board |
| `drv ch cx` | swap: the substitution test |
| `drv ch ch` | CH driver on both: does the CX chip produce histograms? |
| `[a\|b] res 4\|8`, `hz <n>` | resolution; rate (8×8 up to 15 Hz, 4×4 up to 60) |
| `[a\|b] cnh <start_bin> <bins> <sub> [merge]` | histogram layout (CH driver), see below |
| `[a\|b] sharp`, `order`, `mode`, `it` | sharpener %, closest/strongest target first, continuous/autonomous, integration time |
| `[a\|b] start\|stop\|reinit`, `status`, `i2c <hz>` | |

Commands without an `a`/`b` prefix apply to both ports.

**CNH layout.** The device histogram has 37.5 mm bins. CNH keeps `bins` output bins, each summing `sub` device bins, starting at `start_bin`. Each aggregate is one zone, or `merge`×`merge` zones. Everything must fit in 6160 bytes, roughly `aggregates × (5·bins + 5) + 28`:

- default `cnh 0 18 2 1`: every zone of 8×8, 18 bins × 75 mm, about 0–1.35 m
- `cnh 0 32 1 2`: 4×4 aggregates of 2×2 zones, 32 bins × 37.5 mm, about 0–1.2 m
- `res 4` then `cnh 0 64 1`: 4×4 zones, 64 bins × 37.5 mm, about 0–2.4 m

Bin-to-distance is nominal (`(start + (i + 0.5)·sub) × 37.5 mm`); `analyze.py` reports the measured offset against the CH's own ranging.

**Multiple targets per zone.** Both libraries report 1 target per zone by default, but the chip can report up to 4. For a fair CX-vs-CH comparison, also run with 4: in the installed libraries' `src` folders, add `vl53l7cx_platform_config_custom.h` (CX) and `platform_config_custom.h` (CH), containing:

```c
#define VL53L7CX_NB_TARGET_PER_ZONE 4U   // CX file
#define VL53LMZ_NB_TARGET_PER_ZONE 4U    // CH file
```

Then rebuild. The tools handle any target count.

## Capture and analysis

```
py -m pip install -r tools/requirements.txt
py tools/capture.py --port COM5 --view                          # live view; click a zone
py tools/capture.py --port COM5 --label wall_500 --seconds 20   # record
py tools/capture.py --port COM5 --drivers ch,cx --label swap --seconds 20
py tools/analyze.py captures/<file>.vlog                        # per-zone tables
py tools/analyze.py captures/<file>.vlog --plot --zone 3,4      # maps + histogram waterfall
```

Captures go to `captures/` (git-ignored) as raw frames (`.vlog`) plus a `.json` sidecar with the config and chip IDs. For each port, `analyze.py` prints frame rate, I2C read time, and per-zone valid %, median distance and temporal noise. For CNH captures it adds:

- the bin→mm offset against the CH's ranged distance;
- **the key metric**: the share of frames where a zone's histogram has a second peak but the ranging output reported a single target. These are returns the depth output drops.

The waterfall plot (`--plot --zone r,c`) shows the histogram over time, with the ranged distance overlaid. Use it to check any number the peak heuristic produces.

**Serial Monitor logs.** A text log saved from the Serial Monitor can be turned into a video:

```
py tools/textlog_video.py data/<log>.txt          # writes data/<log>.mp4 (or --out x.gif)
```

Each frame shows the depth map, every zone's height above a floor plane fitted to that frame, a side view and a top view, plus the fitted sensor height, tilt and roll ([tools/floor.py](tools/floor.py)). The sensor reports depth along its optical axis, and the near rows fit a plane to a few mm. Heights of far zones are coarse: each zone is 7.5° wide, and the plane is extrapolated from near rows.

`py -m pytest tools` runs offline tests of the frame format, floor fit and analysis.

## Test plan

**1. Substitution.** For each driver setting, run `status` and record:

| Driver on CX board / CH board | CX board init? | CH board init? | CX board histograms? |
|---|---|---|---|
| `drv cx ch` (native) | | | n/a |
| `drv ch cx` (swapped) | | | |
| `drv ch ch` (CH driver on both) | | | |

What the code shows: both drivers accept the same silicon ID (device 0xF0, revision 0x02), and each downloads its own firmware into the sensor at boot. So the remaining question is whether the CX chip accepts CNH configuration; the firmware logs the IDs for both boards. If the CX driver runs the CH board normally, a CH can fill a CX socket. If the CH driver runs the CX chip (histograms aside), building robot firmware on the CH (VL53LMZ) driver lets either part be fitted.

**2. Baseline.** A white wall and a black target (cloth or card), square to the sensors, at 100, 200, 500 and 1000 mm, 20 s each. Compare valid % and temporal std between ports.

**3. Vacuum scenes.** Sensors at robot height, near level. For each scene, capture `drv cx ch`, then 4×4 with `cnh 0 64 1`:

- chair leg / table leg in front of a far wall (thin near object, strong far return)
- cable or sock on the floor; dark shoe; black furniture skirt
- glass door, mirror, glossy tile (specular)
- sofa gap at about robot height (overhang)
- bright sunlight through a window (ambient)

For each scene: did the CX's depth output report the obstacle? Does the CH histogram show it (second peak, or a peak in zones where ranging said "no target")? Does the CH's own depth output differ from the CX's?

## Repo layout

```
firmware/vl53l7_eval/   Arduino sketch (ESP32)
  cx_driver.cpp         ST VL53L7CX library wrapper
  ch_driver.cpp         ST VL53L7CH library wrapper (CNH histograms)
  tof_port.*            one sensor per I2C bus, one FreeRTOS task each
  protocol.h            binary frame format
tools/
  capture.py            configure, record, live view
  analyze.py            per-zone stats, histogram second-peak analysis
  vlog.py               frame parser, capture loader, Serial Monitor log parser
  floor.py              floor-plane fit, sensor pose, heights above floor
  textlog_video.py      Serial Monitor log -> video
```

### Notes on the ST libraries

These are worked around in the firmware; they matter if you reuse the libraries:

- The two libraries can't be included in one source file: both define `union Block_header`, and the CH library's `platform.h` hides its typedefs behind a guard that the CX header defines. Each lives in its own `.cpp` here.
- `VL53LMZ_CNH_DATA_BH` is wrong: the 6160-byte size overflows its 12-bit field into the block index. Build the header by hand (type 4, size in 32-bit words), as ST's VL53L8CH CNH example does.
- The libraries' read routine retries forever on a NACK, so a missing sensor hangs the caller. The firmware probes the address before handing the bus to a driver.
- ST's CI builds these libraries only for STM32; this firmware was compile- and link-checked against the esp32 2.0.17 core.
