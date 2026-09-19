#include <Arduino.h>
#include "AXS15231B.h"
#include "esp_heap_caps.h"

extern uint32_t transfer_num;
extern size_t lcd_PushColors_len;

static uint16_t *frame = nullptr;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("PROOT LONG - FULL FRAME FACTORY PATH");

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  axs15231_init();

  const size_t pixels = 180u * 640u;
  frame = (uint16_t *)heap_caps_malloc(pixels * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!frame) frame = (uint16_t *)heap_caps_malloc(pixels * sizeof(uint16_t), MALLOC_CAP_8BIT);
  if (!frame) {
    Serial.println("FRAME ALLOC FAILED");
    return;
  }

  for (size_t i = 0; i < pixels; ++i) frame[i] = 0x07E0; // green
  lcd_PushColors(0, 0, 180, 640, frame);
}

void loop() {
  if (transfer_num <= 1 && lcd_PushColors_len > 0) {
    lcd_PushColors(0, 0, 0, 0, NULL);
  }
  delay(1);
}
