#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include "board_config.h"

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  TFT_QSPI_CS, TFT_QSPI_SCK,
  TFT_QSPI_D0, TFT_QSPI_D1, TFT_QSPI_D2, TFT_QSPI_D3);

Arduino_GFX *gfx = new Arduino_AXS15231(
  bus, TFT_QSPI_RST, 0, false, LCD_NATIVE_W, LCD_NATIVE_H);

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println("PROOT LONG DISPLAY TEST");

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, LOW);

  Serial.println("gfx begin...");
  gfx->begin();
  Serial.println("gfx begin OK");

  gfx->fillScreen(0xFFFF);
  delay(300);
  gfx->fillScreen(0x0000);

  gfx->setTextColor(0xFFFF);
  gfx->setTextSize(2);
  gfx->setCursor(18, 280);
  gfx->println("PROOT LONG");
  gfx->setCursor(18, 310);
  gfx->println("DISPLAY OK");

  digitalWrite(TFT_BL, HIGH);
  Serial.println("backlight ON");
}

void loop() {
  delay(1000);
}
