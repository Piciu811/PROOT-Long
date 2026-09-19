#include <Arduino.h>
#include "AXS15231B.h"

extern uint32_t transfer_num;
extern size_t lcd_PushColors_len;

static uint16_t linebuf[180 * 16];

static void fill_screen(uint16_t color) {
  for (size_t i = 0; i < sizeof(linebuf)/sizeof(linebuf[0]); ++i) linebuf[i] = color;
  for (int y = 0; y < 640; y += 16) {
    lcd_PushColors(0, y, 180, 16, linebuf);
    while (transfer_num > 0 || lcd_PushColors_len > 0) {
      lcd_PushColors(0, 0, 0, 0, NULL);
      delay(1);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("PROOT LONG - LILYGO FACTORY DRIVER TEST");

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  axs15231_init();
  fill_screen(0x001F);
  Serial.println("DISPLAY TEST OK");
}

void loop() {
  delay(1000);
}
