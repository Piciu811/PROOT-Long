#include "racebox.h"

namespace {
RaceBoxTelemetry state;
}

namespace RaceBox {
const RaceBoxTelemetry &telemetry() {
  return state;
}
}
