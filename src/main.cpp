#include <Arduino.h>
#include "AXS15231B.h"
#include "esp_heap_caps.h"
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <Wire.h>

extern uint32_t transfer_num;
extern size_t lcd_PushColors_len;

static uint16_t *nativeFrame=nullptr,*screen=nullptr;
static String raceboxes[8];
static String raceboxAddr[8];
static uint8_t raceboxAddrType[8]={0};
static int raceboxCount=0, selectedRacebox=0;
static int listPage=0;
static int bleSeen=0;
static bool touchDown=false;
static bool rbConnected=false;
static String connectedAddr="";
static Preferences prefs;
static String savedRbAddr="";
#define TOUCH_ADDR 0x3B
#define TOUCH_SCL 10
#define TOUCH_SDA 15
#define TOUCH_INT 11
#define TOUCH_RST 16
static inline uint16_t C(uint16_t v){ return (uint16_t)((v<<8)|(v>>8)); }

static void rect(int x,int y,int w,int h,uint16_t c){
  if(x<0){w+=x;x=0;} if(y<0){h+=y;y=0;}
  if(x+w>640)w=640-x; if(y+h>180)h=180-y;
  if(w<=0||h<=0)return;
  for(int yy=y;yy<y+h;yy++) for(int xx=x;xx<x+w;xx++) screen[(size_t)yy*640+xx]=c;
}
static void present(){
  for(int y=0;y<180;y++) for(int x=0;x<640;x++)
    nativeFrame[(size_t)x*180+(179-y)]=screen[(size_t)y*640+x];
  lcd_PushColors(0,0,180,640,nativeFrame);
}
// Tiny 5x7 font, enough for the hardware UI prototype.
static const uint8_t DIG[12][5]={
 {0x3E,0x51,0x49,0x45,0x3E},{0,0x42,0x7F,0x40,0},{0x42,0x61,0x51,0x49,0x46},
 {0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
 {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},{0x36,0x49,0x49,0x49,0x36},
 {0x06,0x49,0x49,0x29,0x1E},{0,0x36,0x36,0,0},{0,0x60,0x60,0,0}};
static void glyph(int x,int y,int id,int s,uint16_t col){
  for(int cx=0;cx<5;cx++) for(int cy=0;cy<7;cy++) if(DIG[id][cx]&(1<<cy)) rect(x+cx*s,y+cy*s,s,s,col);
}
static void num(int x,int y,const char*t,int s,uint16_t col){
  while(*t){ int id=-1; if(*t>='0'&&*t<='9')id=*t-'0'; else if(*t==':')id=10; else if(*t=='.')id=11;
    if(id>=0)glyph(x,y,id,s,col); x+=6*s; t++; }
}
static void scanRaceBoxes(){
  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEScan *scan=NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
  NimBLEScanResults found=scan->getResults(10000,false);
  raceboxCount=0;
  bleSeen=found.getCount();
  NimBLEUUID rbService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");

  int used[8]; for(int i=0;i<8;i++) used[i]=-1;
  for(int slot=0;slot<8;slot++){
    int best=-1,bestRssi=-999;
    for(int i=0;i<found.getCount();i++){
      bool already=false; for(int k=0;k<slot;k++) if(used[k]==i) already=true;
      if(already) continue;
      const NimBLEAdvertisedDevice *d=found.getDevice(i);
      if(d && d->getRSSI()>bestRssi){best=i;bestRssi=d->getRSSI();}
    }
    if(best<0) break;
    used[slot]=best;
    const NimBLEAdvertisedDevice *d=found.getDevice(best);
    String name=d->haveName()?String(d->getName().c_str()):String("BLE");
    bool serviceMatch=d->isAdvertisingService(rbService);
    raceboxes[raceboxCount]=name;
    raceboxAddr[raceboxCount]=String(d->getAddress().toString().c_str());
    raceboxAddrType[raceboxCount]=d->getAddress().getType();
    Serial.printf("BLE candidate %d name='%s' addr=%s type=%u RSSI=%d RBsvc=%d\\n",
      raceboxCount+1,name.c_str(),raceboxAddr[raceboxCount].c_str(),
      raceboxAddrType[raceboxCount],d->getRSSI(),serviceMatch);
    raceboxCount++;
  }
}
static void saveRaceBox(const String &addr){
  prefs.begin("proot",false); prefs.putString("rbAddr",addr); prefs.end();
  savedRbAddr=addr;
}
static NimBLEClient *rbClient=nullptr;
static portMUX_TYPE rbDataMux=portMUX_INITIALIZER_UNLOCKED;
static volatile bool rbLiveValid=false;
static volatile uint32_t rbLivePackets=0;
static float rbSpeedKmh=0;
static uint8_t rbFix=0,rbSats=0;
static uint8_t rbStream[512];
static size_t rbStreamLen=0;
static NimBLERemoteCharacteristic *rbTx=nullptr,*rbRx=nullptr;
enum ConnectState : uint8_t { CONN_IDLE, CONN_WORKING, CONN_OK, CONN_FAIL };
static volatile ConnectState connState=CONN_IDLE;
static volatile int connIndex=-1;
static ConnectState drawnConnState=CONN_IDLE;

static void rbNotify(NimBLERemoteCharacteristic*, uint8_t*, size_t len, bool){
  // Isolation build: do not parse/copy RaceBox stream yet. Prove the BLE link
  // remains stable while 25 Hz notifications are arriving.
  rbLivePackets++;
  rbLiveValid = len > 0;
}

static void processRaceBoxStream(){
  // intentionally empty in isolation build
}

static bool probeRaceBoxIndex(int idx){
  if(idx<0 || idx>=raceboxCount) return false;
  const String addr=raceboxAddr[idx];
  NimBLEAddress target(std::string(addr.c_str()),raceboxAddrType[idx]);
  Serial.printf("NIMBLE PROBE %s type=%u...\\n",addr.c_str(),raceboxAddrType[idx]);

  NimBLEClient *stale=NimBLEDevice::getClientByPeerAddress(target);
  if(stale) NimBLEDevice::deleteClient(stale);
  NimBLEClient *client=NimBLEDevice::createClient();
  if(!client){ Serial.println("PROBE FAIL: createClient"); return false; }

  if(!client->connect(target)){
    Serial.println("PROBE FAIL: connect");
    NimBLEDevice::deleteClient(client);
    return false;
  }

  NimBLERemoteService *svc=client->getService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
  if(!svc){
    Serial.println("PROBE FAIL: no RaceBox UART service");
    client->disconnect(); NimBLEDevice::deleteClient(client); return false;
  }
  rbRx=svc->getCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
  rbTx=svc->getCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
  if(!rbRx || !rbTx || !rbTx->canNotify()){
    Serial.println("PROBE FAIL: RaceBox UART characteristics");
    client->disconnect(); NimBLEDevice::deleteClient(client); rbRx=nullptr; rbTx=nullptr; return false;
  }
  if(!rbTx->subscribe(true,rbNotify)){
    Serial.println("PROBE FAIL: TX subscribe");
    client->disconnect(); NimBLEDevice::deleteClient(client); rbRx=nullptr; rbTx=nullptr; return false;
  }

  rbClient=client;
  rbConnected=true; connectedAddr=addr; saveRaceBox(addr);
  Serial.printf("RACEBOX CONNECTED %s\\n",addr.c_str());
  return true;
}
static bool probeRaceBoxAddress(const String &addr){
  for(int i=0;i<raceboxCount;i++) if(raceboxAddr[i].equalsIgnoreCase(addr)) return probeRaceBoxIndex(i);
  return false;
}
static bool connectSelectedRaceBox(){
  if(selectedRacebox<0 || selectedRacebox>=raceboxCount) return false;
  return probeRaceBoxAddress(raceboxAddr[selectedRacebox]);
}
static void raceBoxConnectTask(void *){
  int idx=connIndex;
  bool ok=false;
  if(idx>=0 && idx<raceboxCount) ok=probeRaceBoxAddress(raceboxAddr[idx]);
  connState=ok?CONN_OK:CONN_FAIL;
  vTaskDelete(NULL);
}
static void startRaceBoxConnect(int idx){
  if(connState==CONN_WORKING) return;
  connIndex=idx;
  connState=CONN_WORKING;
  xTaskCreatePinnedToCore(raceBoxConnectTask,"rb-connect",8192,nullptr,1,nullptr,0);
}
static int autoFindRaceBox(){
  // First try the address remembered by the original-style rbAddr preference.
  if(savedRbAddr.length()){
    for(int i=0;i<raceboxCount;i++){
      if(raceboxAddr[i].equalsIgnoreCase(savedRbAddr) && probeRaceBoxAddress(raceboxAddr[i])) return i;
    }
  }
  // RaceBox does not have to advertise its UART UUID. Connect to nearby BLE candidates
  // one by one and identify it by the service that exists after connection.
  for(int i=0;i<raceboxCount;i++){
    if(savedRbAddr.length() && raceboxAddr[i].equalsIgnoreCase(savedRbAddr)) continue;
    if(probeRaceBoxAddress(raceboxAddr[i])) return i;
  }
  return -1;
}
static void drawRaceBoxLive(){
  uint16_t black=C(0x0000),white=C(0xFFFF),green=C(0x07E0),gray=C(0x4208),red=C(0xF800);
  float speed; uint8_t fix,sats; uint32_t packets; bool valid;
  portENTER_CRITICAL(&rbDataMux);
  speed=rbSpeedKmh; fix=rbFix; sats=rbSats; packets=rbLivePackets; valid=rbLiveValid;
  portEXIT_CRITICAL(&rbDataMux);
  for(size_t i=0;i<180u*640u;i++)screen[i]=black;
  rect(0,0,640,4,green);
  rect(12,12,120,22,rbConnected?green:red);
  rect(148,12,120,22,valid?green:gray);
  char sp[16]; snprintf(sp,sizeof(sp),"%03d",(int)(speed+0.5f));
  num(22,58,sp,10,white);
  num(360,58,String(sats).c_str(),7,green);
  num(500,58,String(fix).c_str(),7,fix>=2?green:red);
  num(500,130,String(packets%10000).c_str(),3,gray);
  present();
}
static bool readTouch(int &lx,int &ly){
  uint8_t cmd[8]={0xb5,0xab,0xa5,0x5a,0,0,0,8},b[14]={0};
  Wire.beginTransmission(TOUCH_ADDR); Wire.write(cmd,8);
  if(Wire.endTransmission()!=0) return false;
  if(Wire.requestFrom(TOUCH_ADDR,14)!=(size_t)14) return false;
  Wire.readBytes(b,14);
  if(!b[1] || b[0]) return false;
  int nx=((b[2]&0x0F)<<8)|b[3], ny=((b[4]&0x0F)<<8)|b[5];
  // Touch controller axes are opposite to the framebuffer's native axes on this module:
  // raw X spans the long 640px axis, raw Y spans the short 180px axis.
  // Map to our landscape UI and mirror X to match the physical display orientation.
  lx=639-nx;
  ly=179-ny;
  lx=constrain(lx,0,639); ly=constrain(ly,0,179);
  return true;
}
static void drawRaceBoxList(){
  uint16_t black=C(0x0000),white=C(0xFFFF),green=C(0x07E0),gray=C(0x4208),red=C(0xF800);
  for(size_t i=0;i<180u*640u;i++)screen[i]=black;
  rect(0,0,640,4,green);
  // Header/status blocks: green=scan complete, red=no devices.
  rect(12,12,180,22,bleSeen?green:red);
  num(210,10,String(raceboxCount).c_str(),4,white);
  num(270,10,String(listPage+1).c_str(),4,white);
  num(306,10,String((raceboxCount+3)/4).c_str(),4,white);
  // Total BLE advertisements seen, for hardware diagnostics even when no RaceBox matches.
  num(330,10,String(bleSeen).c_str(),4,white);
  // One visible row per discovered RaceBox (up to 4 on 180px screen).
  int first=listPage*4;
  for(int row=0;row<4;row++){
    int i=first+row;
    if(i>=raceboxCount) break;
    int y=46+row*31;
    uint16_t rowColor=gray;
    if(i==selectedRacebox){
      if(connState==CONN_WORKING) rowColor=C(0xFFE0);
      else if(connState==CONN_OK && rbConnected) rowColor=green;
      else if(connState==CONN_FAIL) rowColor=red;
    }
    rect(12,y,616,25,rowColor);
    num(22,y+3,String(i+1).c_str(),3,black);
    // RaceBox advertises a human-visible serial in its BLE name on supported models.
    // Show all decimal digits from the advertised name; fall back to BLE address suffix.
    String id="";
    for(int k=0;k<raceboxes[i].length();k++) if(isDigit(raceboxes[i][k])) id+=raceboxes[i][k];
    if(!id.length()){
      String a=raceboxAddr[i];
      for(int k=0;k<a.length();k++) if(isHexadecimalDigit(a[k])) id+=a[k];
      if(id.length()>4) id=id.substring(id.length()-4);
    }
    if(id.length()>10) id=id.substring(id.length()-10);
    num(92,y+3,id.c_str(),3,white);
  }
  // Visible paging controls at the bottom corners when more than four devices exist.
  if(raceboxCount>4){
    rect(4,150,72,26,gray);
    rect(564,150,72,26,green);
    num(26,152,String(listPage+1).c_str(),3,white);
    num(586,152,String((listPage+1)%((raceboxCount+3)/4)+1).c_str(),3,black);
  }
  present();
}
void setup(){
  Serial.begin(115200); delay(200);
  pinMode(TFT_BL,OUTPUT); digitalWrite(TFT_BL,HIGH);
  // Official LilyGO touch example resets the shared AXS15231B/touch controller first.
  pinMode(TOUCH_RST,OUTPUT);
  digitalWrite(TOUCH_RST,HIGH); delay(2);
  digitalWrite(TOUCH_RST,LOW); delay(10);
  digitalWrite(TOUCH_RST,HIGH); delay(2);
  Wire.begin(TOUCH_SDA,TOUCH_SCL);
  pinMode(TOUCH_INT,INPUT);
  axs15231_init();
  const size_t n=180u*640u;
  nativeFrame=(uint16_t*)heap_caps_malloc(n*2,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
  screen=(uint16_t*)heap_caps_malloc(n*2,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
  if(!nativeFrame||!screen){Serial.println("FRAME ALLOC FAILED");return;}
  uint16_t black=C(0x0000),white=C(0xFFFF),green=C(0x07E0),gray=C(0x4208),red=C(0xF800);
  for(size_t i=0;i<n;i++)screen[i]=black;

  // Show an intentional scan screen while the 10 s BLE scan is running.
  // This replaces the unexplained blank area seen during startup.
  rect(0,0,640,180,black);
  rect(0,0,640,4,green);
  rect(12,68,616,44,gray);
  while(transfer_num>1){ lcd_PushColors(0,0,0,0,NULL); delay(1); }
  present();
  while(transfer_num>0 && lcd_PushColors_len>0){ lcd_PushColors(0,0,0,0,NULL); delay(1); }
  Serial.println("BLE SCANNING");
  prefs.begin("proot",true); savedRbAddr=prefs.getString("rbAddr",""); prefs.end();
  scanRaceBoxes();
  // PRÖÖT-style behavior: prefer the previously selected RaceBox address.
  if(savedRbAddr.length()){
    for(int i=0;i<raceboxCount;i++) if(raceboxAddr[i].equalsIgnoreCase(savedRbAddr)){
      selectedRacebox=i;
      Serial.printf("Saved RaceBox found at row %d: %s\\n",i+1,savedRbAddr.c_str());
      break;
    }
  }
  // Do not connect/probe here: BLEClient::connect() can block setup for seconds per candidate.
  // Show the list immediately; connection is attempted only after the user selects a row.
  Serial.printf("RaceBox found: %d\\n",raceboxCount);
  for(int i=0;i<raceboxCount;i++) Serial.printf("%c %d: %s %s\\n",i==selectedRacebox?'>':' ',i+1,raceboxes[i].c_str(),raceboxAddr[i].c_str());
  while(transfer_num>1){ lcd_PushColors(0,0,0,0,NULL); delay(1); }
  drawRaceBoxList();
}
void loop(){
  if(transfer_num<=1&&lcd_PushColors_len>0)lcd_PushColors(0,0,0,0,NULL);
  processRaceBoxStream();
  if(connState!=drawnConnState){
    drawnConnState=connState;
    if(connState==CONN_OK && connIndex>=0){
      selectedRacebox=connIndex;
      saveRaceBox(raceboxAddr[selectedRacebox]);
    }
    while(transfer_num>1){ lcd_PushColors(0,0,0,0,NULL); delay(1); }
    drawRaceBoxList();
  }
  int x,y; bool down=readTouch(x,y);
  static uint32_t lastDiag=0;
  if(millis()-lastDiag>1000){
    lastDiag=millis();
    Wire.beginTransmission(TOUCH_ADDR);
    int err=Wire.endTransmission();
    Serial.printf("TOUCH I2C=%d INT=%d\\n",err,digitalRead(TOUCH_INT));
  }
  if(down&&!touchDown) Serial.printf("TOUCH DOWN x=%d y=%d\\n",x,y);
  if(down&&!touchDown&&y>=150 && raceboxCount>4){
    listPage=(listPage+1)%((raceboxCount+3)/4);
    while(transfer_num>1){ lcd_PushColors(0,0,0,0,NULL); delay(1); }
    drawRaceBoxList();
  } else if(down&&!touchDown&&x>=12&&x<628&&y>=46&&y<145){
    int row=(y-46)/31;
    int idx=listPage*4+row;
    if(idx>=0&&idx<raceboxCount){
      selectedRacebox=idx;
      Serial.printf("Selected BLE %d: %s %s\\n",idx+1,raceboxes[idx].c_str(),raceboxAddr[idx].c_str());
      // Connection runs on a separate FreeRTOS task, so touch/display stay responsive.
      rbConnected=false;
      startRaceBoxConnect(idx);
      // Do not draw here. loop() redraws exactly once when connState changes.
    }
  }
  touchDown=down;
  delay(20);
}
