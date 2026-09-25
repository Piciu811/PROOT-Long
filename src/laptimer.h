#pragma once

#include <Arduino.h>
#include "racebox.h"

struct LapTimerState {
  uint32_t lapStartTow = 0;
  uint32_t lastLapMs = 0;
  uint32_t bestLapMs = 0;
  int32_t deltaMs = 0;
  bool deltaValid = false;
  uint16_t lapCount = 0;
  bool running = false;
  bool stopped = false;
  bool customLine = false;
  bool factoryTrack = false;
  int16_t factoryTrackIndex = -1;
  uint32_t flashStarted = 0;
};

namespace LapTimer {
  void reset();
  void update(const RaceBoxTelemetry &gps);
  void setCustomStartFinish(const RaceBoxTelemetry &gps);
  void stop();
  const LapTimerState &state();
}
