#pragma once

#include <Arduino.h>

// Public RaceBox telemetry exposed to the rest of the firmware.
struct RaceBoxTelemetry {
  bool connected = false;
  bool liveValid = false;
  uint32_t livePackets = 0;
  float speedKmh = 0.0f;
  double lat = 0.0;
  double lon = 0.0;
  uint32_t towMs = 0;
  uint8_t fix = 0;
  uint8_t sats = 0;
  float leanDeg = 0.0f;
  bool leanValid = false;
};

// First stage of the modularisation. Implementation will be moved from
// main.cpp into racebox.cpp while preserving the existing behaviour.
namespace RaceBox {
  const RaceBoxTelemetry &telemetry();
}
