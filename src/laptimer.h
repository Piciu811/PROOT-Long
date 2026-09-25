#pragma once

#include <Arduino.h>
#include "racebox.h"

struct LapTimerState {
  uint32_t lapStartTow=0,lastLapMs=0,bestLapMs=0;
  int32_t deltaMs=0;
  bool deltaValid=false;
  uint16_t lapCount=0;
  bool running=false,stopped=false;
  bool customLine=false,factoryTrack=false;
  int16_t factoryTrackIndex=-1;
  uint32_t flashStarted=0;
  uint8_t historyCount=0,historyPage=0;
};

namespace LapTimer {
  void reset();
  void update(const RaceBoxTelemetry &gps);
  void setCustomStartFinish(const RaceBoxTelemetry &gps);
  void stop();
  void resume();
  void historyPageUp();
  void historyPageDown();
  uint32_t history(uint8_t index);
  const LapTimerState &state();
}
