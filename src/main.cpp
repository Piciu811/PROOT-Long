#include <Arduino.h>
#include "AXS15231B.h"
#include "esp_heap_caps.h"

extern uint32_t transfer_num;
extern size_t lcd_PushColors_len;

static uint16_t *nativeFrame = nullptr;
static uint16_t *landscapeFrame = nullptr;

static inline uint16_t rgb565(uint16_t c) {
  return (uint16_t)((c << 8) | (c >> 8));
}

// Logical screen is 640x180 landscape.
// Native panel memory is 180x640 portrait.
// Rotate logical pixels into the known-good native framebuffer path.
static void presentLandscape() {
  for (int y = 0; y < 180; ++y) {
    for (int x = 0; x < 640; ++x) {
      // 90 degree rotation: logical (x,y) -> native (nx,ny)
      const int nx = 179 - y;
      const int ny = x;
      nativeFrame[(size_t)ny * 180 + nx] = landscapeFrame[(size_t)y * 640 + x];
    }
  }
  lcd_PushColors(0, 0, 180, 640, nativeFrame);
}

static void fillRect(int x, int y, int w, int h, uint16_t c) {
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x + w > 640) w = 640 - x;
  if (y + h > 180) h = 180 - y;
  if (w <= 0 || h <= 0) return;
  for (int yy = y; yy < y + h; ++yy)
    for (int xx = x; xx < x + w; ++xx)
      landscapeFrame[(size_t)yy * 640 + xx] = c;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("PROOT LONG - 640x180 LANDSCAPE TEST");

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  axs15231_init();

  const size_t pixels = 180u * 640u;
  nativeFrame = (uint16_t *)heap_caps_malloc(pixels * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  landscapeFrame = (uint16_t *)heap_caps_malloc(pixels * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!nativeFrame) nativeFrame = (uint16_t *)heap_caps_malloc(pixels * 2, MALLOC_CAP_8BIT);
  if (!landscapeFrame) landscapeFrame = (uint16_t *)heap_caps_malloc(pixels * 2, MALLOC_CAP_8BIT);
  if (!nativeFrame || !landscapeFrame) {
    Serial.println("FRAME ALLOC FAILED");
    return;
  }

  const uint16_t BLACK = rgb565(0x0000);
  const uint16_t RED   = rgb565(0xF800);
  const uint16_t GREEN = rgb565(0x07E0);
  const uint16_t BLUE  = rgb565(0x001F);
  const uint16_t WHITE = rgb565(0xFFFF);

  for (size_t i = 0; i < pixels; ++i) landscapeFrame[i] = BLACK;

  // Landscape orientation marker: five vertical bars across the 640px width.
  fillRect(0,   0, 128, 180, RED);
  fillRect(128, 0, 128, 180, GREEN);
  fillRect(256, 0, 128, 180, BLUE);
  fillRect(384, 0, 128, 180, WHITE);
  fillRect(512, 0, 128, 180, BLACK);

  // White corner markers make rotation obvious.
  fillRect(8, 8, 28, 12, WHITE);
  fillRect(8, 8, 12, 28, WHITE);

  presentLandscape();
  Serial.println("LANDSCAPE TEST SENT");
}

void loop() {
  if (transfer_num <= 1 && lcd_PushColors_len > 0)
    lcd_PushColors(0, 0, 0, 0, NULL);
  delay(1);
}
