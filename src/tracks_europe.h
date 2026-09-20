#pragma once
#include <stdint.h>
struct FactoryTrack { int32_t lat1,lon1,lat2,lon2; uint16_t id; uint8_t enabled; };
static const FactoryTrack FACTORY_TRACKS[] PROGMEM = {
};
static const uint16_t FACTORY_TRACK_COUNT=0;
