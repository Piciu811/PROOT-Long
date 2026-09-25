#include "laptimer.h"

namespace {
LapTimerState timerState;
}

namespace LapTimer {
void reset() {
  timerState = LapTimerState{};
}

void update(const RaceBoxTelemetry &gps) {
  // Timing/crossing logic will be moved here from main.cpp in the next step.
  // Keeping the telemetry argument explicit prevents RaceBox globals leaking
  // into the timing module.
  (void)gps;
}

void setCustomStartFinish(const RaceBoxTelemetry &gps) {
  if (gps.fix < 2) return;
  timerState.customLine = true;
  timerState.factoryTrack = false;
  timerState.factoryTrackIndex = -1;
  timerState.lapStartTow = gps.towMs;
  timerState.running = true;
  timerState.stopped = false;
  timerState.lapCount = 0;
  timerState.lastLapMs = 0;
  timerState.bestLapMs = 0;
  timerState.deltaMs = 0;
  timerState.deltaValid = false;
}

void stop() {
  timerState.running = false;
  timerState.stopped = true;
  timerState.deltaValid = false;
}

const LapTimerState &state() {
  return timerState;
}
}
