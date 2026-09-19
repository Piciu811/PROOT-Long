#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include "board_config.h"

static constexpr uint16_t BLACK=0x0000, WHITE=0xFFFF, GREEN=0x07E0, YELLOW=0xFFE0, DARKGREY=0x7BEF;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(TFT_QSPI_CS, TFT_QSPI_SCK, TFT_QSPI_D0, TFT_QSPI_D1, TFT_QSPI_D2, TFT_QSPI_D3);
Arduino_GFX *gfx = new Arduino_AXS15231B(bus, TFT_QSPI_RST, 1, false, LCD_NATIVE_W, LCD_NATIVE_H);

static const uint8_t TOUCH_READ_CMD[] = {0xB5,0xAB,0xA5,0x5A,0x00,0x00,0x00,0x08,0x00,0x00,0x00};
volatile bool touchIRQ=false;
uint32_t nextTouchRead=0;
struct TouchPoint { bool pressed=false; uint16_t x=0,y=0; };
void IRAM_ATTR touchISR(){ touchIRQ=true; }

bool writeC8D8(uint8_t address,uint8_t reg,uint8_t data){
  Wire.beginTransmission(address); Wire.write(reg); Wire.write(data);
  return Wire.endTransmission()==0;
}

TouchPoint readTouch(){
  TouchPoint p; uint8_t b[8]={0};
  if(!touchIRQ || millis()<nextTouchRead) return p;
  touchIRQ=false; nextTouchRead=millis()+20;
  Wire.beginTransmission(0x3B);
  Wire.write(TOUCH_READ_CMD, sizeof(TOUCH_READ_CMD));
  if(Wire.endTransmission(false)!=0) return p;
  size_t got=Wire.requestFrom((uint8_t)0x3B,(uint8_t)sizeof(b));
  if(got!=sizeof(b)) { while(Wire.available()) Wire.read(); return p; }
  for(size_t i=0;i<sizeof(b);i++) b[i]=Wire.read();
  uint8_t fingers=b[1], event=b[2]>>4;
  if(fingers==1 && event==0x08){
    uint16_t nativeX=((uint16_t)(b[4]&0x0F)<<8)|b[5];
    uint16_t nativeY=LCD_NATIVE_H-(((uint16_t)(b[2]&0x0F)<<8)|b[3]);
    p.x=constrain(nativeY,0,LCD_W-1);
    p.y=constrain(LCD_NATIVE_W-1-nativeX,0,LCD_H-1);
    p.pressed=true;
  }
  return p;
}

static BLEUUID RB_SERVICE("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID RB_RX("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID RB_TX("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
BLEClient *rbClient=nullptr; BLERemoteCharacteristic *rbRx=nullptr,*rbTx=nullptr;
volatile uint32_t rbPackets=0, rbBytes=0; bool rbConnected=false; String rbDeviceName="";
uint32_t lastScan=0,lastDraw=0;

void rbNotify(BLERemoteCharacteristic*,uint8_t *data,size_t len,bool){
  rbPackets++; rbBytes+=len;
  Serial.printf("[RaceBox] %u bytes:",(unsigned)len);
  for(size_t i=0;i<len && i<16;i++) Serial.printf(" %02X",data[i]);
  Serial.println();
}

bool connectRaceBox(BLEAdvertisedDevice &dev){
  rbClient=BLEDevice::createClient();
  if(!rbClient->connect(&dev)) return false;
  auto *svc=rbClient->getService(RB_SERVICE);
  if(!svc){ rbClient->disconnect(); return false; }
  rbRx=svc->getCharacteristic(RB_RX); rbTx=svc->getCharacteristic(RB_TX);
  if(!rbTx || !rbTx->canNotify()){ rbClient->disconnect(); return false; }
  rbTx->registerForNotify(rbNotify); rbConnected=true;
  rbDeviceName=dev.haveName()?dev.getName().c_str():"RaceBox";
  Serial.printf("Connected to %s\n",rbDeviceName.c_str());
  return true;
}

void scanRaceBox(){
  if(rbConnected) return;
  BLEScan *scan=BLEDevice::getScan();
  scan->setActiveScan(true); scan->setInterval(100); scan->setWindow(99);
  BLEScanResults *results=scan->start(4,false);
  for(int i=0;i<results->getCount();i++){
    BLEAdvertisedDevice dev=results->getDevice(i);
    if(dev.haveServiceUUID() && dev.isAdvertisingService(RB_SERVICE)){
      if(connectRaceBox(dev)) break;
    }
  }
  scan->clearResults();
}

void centerText(const String &s,int16_t cx,int16_t y,uint8_t size,uint16_t color){
  gfx->setTextSize(size); gfx->setTextColor(color);
  int16_t x=cx-(int16_t)(s.length()*6*size)/2;
  gfx->setCursor(max<int16_t>(0,x),y); gfx->print(s);
}
void drawUI(){
  gfx->fillScreen(BLACK);
  gfx->setTextColor(WHITE); gfx->setTextSize(2); gfx->setCursor(12,9); gfx->print("PROOT LONG");
  gfx->setTextSize(1); gfx->setCursor(170,13); gfx->print(rbConnected?"RACEBOX CONNECTED":"RACEBOX SCANNING...");
  gfx->setCursor(500,13); gfx->printf("PKT %lu",(unsigned long)rbPackets);
  gfx->drawFastHLine(8,34,624,DARKGREY);
  centerText("1:46.02",120,55,4,WHITE);
  centerText("-0.21 s",330,61,3,GREEN);
  centerText("LAP 3",535,61,3,WHITE);
  gfx->setTextSize(1); gfx->setTextColor(WHITE);
  gfx->setCursor(18,116); gfx->print("LAST  1:46.02");
  gfx->setCursor(166,116); gfx->print("BEST  1:45.81");
  gfx->setCursor(318,116); gfx->print("SAT 14  FIX 3D");
  gfx->setCursor(480,116); gfx->print("128 km/h");
  gfx->drawRoundRect(10,143,145,28,5,WHITE);
  gfx->drawRoundRect(165,143,145,28,5,WHITE);
  gfx->drawRoundRect(320,143,145,28,5,WHITE);
  gfx->drawRoundRect(475,143,155,28,5,rbConnected?GREEN:WHITE);
  centerText("DELTA",82,152,1,WHITE);
  centerText("LAPS",237,152,1,WHITE);
  centerText("RACEBOX",392,152,1,WHITE);
  centerText(rbConnected?"CONNECTED":"SCAN",552,152,1,rbConnected?GREEN:WHITE);
}
void markTouch(uint16_t x,uint16_t y){
  gfx->fillCircle(x,y,4,YELLOW);
  Serial.printf("[Touch] x=%u y=%u\n",x,y);
  if(y>=140 && x>=475 && !rbConnected) lastScan=0;
}
void setup(){
  Serial.begin(115200); delay(300);
  pinMode(TFT_BL,OUTPUT); digitalWrite(TFT_BL,LOW);
  pinMode(TOUCH_RES,OUTPUT); pinMode(TOUCH_INT,INPUT_PULLUP);
  digitalWrite(TOUCH_RES,HIGH); delay(2); digitalWrite(TOUCH_RES,LOW); delay(100); digitalWrite(TOUCH_RES,HIGH); delay(2);
  Wire.begin(TOUCH_IICSDA, TOUCH_IICSCL, 400000);
  writeC8D8(0x6A,0x00,0b00111111); writeC8D8(0x6A,0x09,0b01100100);
  attachInterrupt(TOUCH_INT,touchISR,FALLING);
  if(!gfx->begin()){ Serial.println("DISPLAY INIT FAILED"); while(true) delay(1000); }
  digitalWrite(TFT_BL,HIGH);
  drawUI();
  BLEDevice::init("PROOT-Long");
  scanRaceBox();
  drawUI();
}
void loop(){
  TouchPoint p=readTouch();
  if(p.pressed) markTouch(p.x,p.y);
  if(rbConnected && rbClient && !rbClient->isConnected()){ rbConnected=false; rbDeviceName=""; drawUI(); }
  if(!rbConnected && millis()-lastScan>6000){ lastScan=millis(); scanRaceBox(); drawUI(); }
  if(millis()-lastDraw>1000){
    lastDraw=millis();
    gfx->fillRect(495,4,140,26,BLACK); gfx->setTextColor(WHITE); gfx->setTextSize(1); gfx->setCursor(500,13);
    gfx->printf("PKT %lu",(unsigned long)rbPackets);
  }
  delay(2);
}
