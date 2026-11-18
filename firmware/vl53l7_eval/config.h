// Build-time settings: pin map, serial speed, defaults.
#pragma once

#define FW_VERSION "0.1.0"

// ---------------------------------------------------------------------------
// Pin map for a 30-pin ESP32 (ESP32-WROOM-32E) DevKit.
//
// Each sensor gets its own I2C bus, so both can stay at the default address
// (0x29) and no LPn address-change sequence is needed.
//   Port A = Wire  (I2C0) - wire the VL53L7CX board here
//   Port B = Wire1 (I2C1) - wire the VL53L7CH board here
// Set an INT or LPn pin to -1 if it is not wired.
// ---------------------------------------------------------------------------
#define PORT_A_SDA 21
#define PORT_A_SCL 22
#define PORT_A_INT 4
#define PORT_A_LPN 23  // CX board L_Pn (optional; the board pulls it up)

#define PORT_B_SDA 25
#define PORT_B_SCL 26
#define PORT_B_INT 27
#define PORT_B_LPN -1  // the CH board has no LPn pin

// The VL53L7 supports 1 MHz I2C. A CNH histogram frame is ~7.5 KB, so at
// 400 kHz the CH tops out around 4-5 Hz. Drop to 400000 (command `i2c
// 400000`) if init fails or frames come back corrupted with long jumper wires.
#define I2C_HZ_DEFAULT 1000000

// CP2102 and CH340 USB-UART bridges both handle 921600.
#define SERIAL_BAUD 921600

// In text mode, print at most one frame per port every TEXT_MS_DEFAULT ms.
#define TEXT_MS_DEFAULT 500
