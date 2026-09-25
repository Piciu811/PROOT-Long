#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>

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

namespace RaceBox {
  void reset();
  void setConnection(NimBLEClient *client, NimBLERemoteCharacteristic *rx, bool connected);
  void notify(NimBLERemoteCharacteristic *characteristic, uint8_t *data, size_t len, bool isNotify);
  void process();
  bool sendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t plen);
  void requestRecording(bool start);
  bool recordingEnabled();
  const RaceBoxTelemetry &telemetry();
}
