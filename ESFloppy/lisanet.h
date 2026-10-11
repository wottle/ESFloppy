// ---------------------------------------------------------------------------------------------
// LisaNet support for the LisaFPGA board -- NOT part of upstream ESFloppy. Written with an AI assistant.
// See the LisaFPGA fork: docs/superpowers/specs/2026-10-10-esp32-wifi-mode-design.md and docs/lisanet_link_protocol.md.
//
// The FPGA's on-screen menu has an ESP32 MODE row (FLOPPY / WIFI) and drives the MODE line (FPGA_3, GPIO37) high for
// WIFI. The board's own ESP32s have no antenna, so WIFI mode uses an external WiFi board on the FPGA's J21 header, and
// this ESP32 becomes its SD-card server: in that mode the FPGA wires FPGA_4 (GPIO38) and FPGA_5 (GPIO39) through to
// the WiFi board. At boot ESFloppy reads MODE and runs either as usual or as the SD server; each personality restarts
// the ESP32 when MODE changes.
// ---------------------------------------------------------------------------------------------
#pragma once
#include <Arduino.h>

#define LISANET_MODE_PIN    37   // FPGA_3: MODE, 1 = WIFI
#define LISANET_LINK_RX_PIN 38   // FPGA_4: from the WiFi board (through the FPGA)
#define LISANET_LINK_TX_PIN 39   // FPGA_5: to the WiFi board (the FPGA never drives this line)
#define LISANET_LINK_BAUD   2000000

// MODE must hold steady this long before either personality acts on it. While the FPGA is still configuring at
// power-up its pins float or are weakly pulled up, so a single read could see WIFI that isn't really selected.
#define LISANET_MODE_STABLE_MS 300

// True if MODE is high and stays high for LISANET_MODE_STABLE_MS. Call before ESFloppy configures anything.
bool lisanetModeRequestedAtBoot();

// True once MODE has been at `level` continuously for LISANET_MODE_STABLE_MS. Polled by the watchers.
bool lisanetModeStableAt(bool level);

// The SD-server personality (sdServer.cpp). Never returns: restarts the ESP32 when MODE goes back to FLOPPY.
[[noreturn]] void sdServerMain();
