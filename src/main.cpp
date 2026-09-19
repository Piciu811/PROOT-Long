#include <Arduino.h>
#include "AXS15231B.h"
#include "esp_heap_caps.h"

extern uint32_t transfer_num;
extern size_t lcd_PushColors_len;

static uint16_t *frame = nullptr;

static inline uint16_t swap16(uint16_t v) {
  return (uint16_t)((v << 8) | (v >> 8));
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("PROOT LONG - RGB565 BYTE ORDER TEST");

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  axs15231_init();

  const int W = 180;
  const int H = 640;
  const size_t pixels = (size_t)W * H;

  frame = (uint16_t *)heap_caps_malloc(pixels * sizeof(uint16_t),
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!frame) frame = (uint16_t *)heap_caps_malloc(pixels * sizeof(uint16_t), MALLOC_CAP_8BIT);
  if (!frame) {
    Serial.println("FRAME ALLOC FAILED");
    return;
  }

  // AXS15231B expects RGB565 bytes MSB first. ESP32 uint16_t memory is little-endian,
  // so pre-swap each test color before DMA.
  const uint16_t colors[5] = {
    swap16(0xF800), // red
    swap16(0x07E0), // green
    swap16(0x001F), // blue
    swap16(0xFFFF), // white
    swap16(0x0000)  // black
  };

  for (int y = 0; y < H; ++y) {
    int band = (y * 5) / H;
    for (int x = 0; x < W; ++x) {
      frame[(size_t)y * W + x] = colors[band];
    }
  }

  lcd_PushColors(0, 0, W, H, frame);
  Serial.println("RGB565 TEST SENT");
}

void loop() {
  if (transfer_num <= 1 && lcd_PushColors_len > 0) {
    lcd_PushColors(0, 0, 0, 0, NULL);
  }
  delay(1);
}
