#include <Arduino.h>
#include "AXS15231B.h"
#include "esp_heap_caps.h"
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <Wire.h>
#include <math.h>

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
static const uint8_t ALPHA[26][5]={
 {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},
 {0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},
 {0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
 {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},{0x7F,0x09,0x09,0x09,0x06},
 {0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},
 {0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x7F,0x20,0x18,0x20,0x7F},{0x63,0x14,0x08,0x14,0x63},
 {0x03,0x04,0x78,0x04,0x03},{0x61,0x51,0x49,0x45,0x43}};
static void text5(int x,int y,const char*t,int s,uint16_t col){
  while(*t){ char ch=*t++; if(ch==' '){x+=4*s;continue;} int id=-1;
    if(ch>='A'&&ch<='Z')id=ch-'A'; else if(ch>='a'&&ch<='z')id=ch-'a';
    if(id>=0){ for(int cx=0;cx<5;cx++) for(int cy=0;cy<7;cy++) if(ALPHA[id][cx]&(1<<cy)) rect(x+cx*s,y+cy*s,s,s,col); }
    x+=6*s;
  }
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
static double rbLat=0,rbLon=0;
static uint32_t rbTowMs=0;
static uint8_t rbFix=0,rbSats=0;
static uint32_t lapStartTow=0, lapLastMs=0, lapBestMs=0;
static int32_t lapDeltaMs=0;
static bool lapDeltaValid=false;
static uint16_t lapCount=0;
static bool lapClockRunning=false;
// Custom start/finish: long-press dashboard to arm a line at the current GNSS point. Build trigger 2026-09-20.
// The line is perpendicular to the vehicle heading estimated from consecutive GNSS fixes.
static bool customLineValid=false, customLineArmed=false, customDirectionPending=false, havePrevFix=false;
static double customLat=0,customLon=0,customDirX=0,customDirY=0,prevLat=0,prevLon=0;
static uint32_t customSavedAt=0,lastCrossTow=0;
struct LapPoint { float x,y; uint32_t t; };
static const uint16_t LAP_TRACE_MAX=2400;
static LapPoint refTrace[LAP_TRACE_MAX], curTrace[LAP_TRACE_MAX];
static uint16_t refTraceN=0,curTraceN=0,refCursor=0;
static uint32_t lastTraceTow=0;
static uint8_t rbStream[512];
static size_t rbStreamLen=0;
static NimBLERemoteCharacteristic *rbTx=nullptr,*rbRx=nullptr;
enum ConnectState : uint8_t { CONN_IDLE, CONN_WORKING, CONN_OK, CONN_FAIL };
static volatile ConnectState connState=CONN_IDLE;
static volatile int connIndex=-1;
static ConnectState drawnConnState=CONN_IDLE;

static void rbNotify(NimBLERemoteCharacteristic*, uint8_t *data, size_t len, bool){
  // Copy only; no parsing and no LCD work in NimBLE callback.
  portENTER_CRITICAL(&rbDataMux);
  size_t freeBytes=sizeof(rbStream)-rbStreamLen;
  size_t take=len<freeBytes?len:freeBytes;
  if(take){ memcpy(rbStream+rbStreamLen,data,take); rbStreamLen+=take; }
  portEXIT_CRITICAL(&rbDataMux);
}

static void processRaceBoxStream(){
  static uint8_t fifo[1024];
  static size_t fifoLen=0;
  uint8_t local[512]; size_t n=0;
  portENTER_CRITICAL(&rbDataMux);
  n=rbStreamLen;
  if(n){ memcpy(local,rbStream,n); rbStreamLen=0; }
  portEXIT_CRITICAL(&rbDataMux);
  if(n){
    if(n>sizeof(fifo)-fifoLen) fifoLen=0;
    if(n<=sizeof(fifo)-fifoLen){ memcpy(fifo+fifoLen,local,n); fifoLen+=n; }
  }
  while(fifoLen>=8){
    size_t s=0;
    while(s+1<fifoLen && !(fifo[s]==0xB5 && fifo[s+1]==0x62)) s++;
    if(s){ memmove(fifo,fifo+s,fifoLen-s); fifoLen-=s; if(fifoLen<8) break; }
    uint16_t plen=(uint16_t)fifo[4]|((uint16_t)fifo[5]<<8);
    size_t fl=(size_t)plen+8;
    if(fl>sizeof(fifo)){ fifoLen=0; break; }
    if(fifoLen<fl) break;
    uint8_t a=0,b=0;
    for(size_t i=2;i<6u+plen;i++){ a=(uint8_t)(a+fifo[i]); b=(uint8_t)(b+a); }
    if(a==fifo[6+plen] && b==fifo[7+plen] && fifo[2]==0xFF && fifo[3]==0x01 && plen>=80){
      const uint8_t *p=fifo+6;
      uint32_t speedMm=0,tow=0; int32_t lonRaw=0,latRaw=0;
      memcpy(&tow,p+0,4); memcpy(&lonRaw,p+24,4); memcpy(&latRaw,p+28,4); memcpy(&speedMm,p+48,4);
      portENTER_CRITICAL(&rbDataMux);
      rbTowMs=tow; rbLon=(double)lonRaw/10000000.0; rbLat=(double)latRaw/10000000.0;
      rbFix=p[20]; rbSats=p[23]; rbSpeedKmh=(float)speedMm*0.0036f;
      rbLivePackets++; rbLiveValid=true;
      portEXIT_CRITICAL(&rbDataMux);
    }
    memmove(fifo,fifo+fl,fifoLen-fl); fifoLen-=fl;
  }
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
static void fmtLap(uint32_t ms,char *out,size_t n){
  uint32_t min=ms/60000u; ms%=60000u;
  uint32_t sec=ms/1000u, millisec=ms%1000u;
  snprintf(out,n,"%02lu:%02lu.%03lu",(unsigned long)min,(unsigned long)sec,(unsigned long)millisec);
}
static void fmtDelta(int32_t ms,char *out,size_t n){
  char sign=ms<=0?'-':'+';
  uint32_t a=(uint32_t)(ms<0?-ms:ms);
  snprintf(out,n,"%c%lu.%03lu",sign,(unsigned long)(a/1000u),(unsigned long)(a%1000u));
}
static double localX(double lon,double lat0){ return lon*111320.0*cos(lat0*0.017453292519943295); }
static double localY(double lat){ return lat*110540.0; }
static void saveCustomLine(){
  double lat,lon; uint8_t fix; float speed;
  portENTER_CRITICAL(&rbDataMux); lat=rbLat; lon=rbLon; fix=rbFix; speed=rbSpeedKmh; portEXIT_CRITICAL(&rbDataMux);
  if(fix<2) return;
  customLat=lat; customLon=lon; customLineValid=true; customLineArmed=false;
  // If moving, derive direction immediately. If stationary, learn it after moving ~3 m.
  customDirectionPending=true;
  if(havePrevFix && speed>=3.0f){
    double dx=localX(lon,lat)-localX(prevLon,lat), dy=localY(lat)-localY(prevLat);
    double n=sqrt(dx*dx+dy*dy);
    if(n>=0.20){ customDirX=dx/n; customDirY=dy/n; customDirectionPending=false; }
  }
  lapStartTow=rbTowMs; lapClockRunning=true; lapCount=0; lapLastMs=lapBestMs=0; lapDeltaValid=false; lastCrossTow=0;
  refTraceN=curTraceN=refCursor=0; lastTraceTow=0; customSavedAt=millis();
  prefs.begin("proot",false); prefs.putDouble("sfLat",customLat); prefs.putDouble("sfLon",customLon);
  prefs.putDouble("sfDx",customDirX); prefs.putDouble("sfDy",customDirY); prefs.putBool("sfOk",true); prefs.end();
  Serial.printf("CUSTOM SF SET %.7f %.7f pending=%d\\n",customLat,customLon,customDirectionPending);
}
static void updateLapClock(){
  double lat,lon; uint32_t tow; uint8_t fix; float speed;
  portENTER_CRITICAL(&rbDataMux); tow=rbTowMs; fix=rbFix; lat=rbLat; lon=rbLon; speed=rbSpeedKmh; portEXIT_CRITICAL(&rbDataMux);
  if(fix<2) return;
  if(customLineValid && customDirectionPending){
    double dx=localX(lon,customLat)-localX(customLon,customLat), dy=localY(lat)-localY(customLat);
    double n=sqrt(dx*dx+dy*dy);
    if(n>=3.0 && speed>=3.0f){
      customDirX=dx/n; customDirY=dy/n; customDirectionPending=false;
      prefs.begin("proot",false); prefs.putDouble("sfDx",customDirX); prefs.putDouble("sfDy",customDirY); prefs.end();
      Serial.printf("CUSTOM SF DIRECTION %.3f %.3f\\n",customDirX,customDirY);
    }
  }
  if(customLineValid && !customDirectionPending && havePrevFix){
    double x0=localX(customLon,customLat), y0=localY(customLat);
    double px=localX(prevLon,customLat)-x0, py=localY(prevLat)-y0;
    double cx=localX(lon,customLat)-x0, cy=localY(lat)-y0;
    // Signed distance along travel direction; crossing zero means crossing the perpendicular S/F line.
    double prevAlong=px*customDirX+py*customDirY, curAlong=cx*customDirX+cy*customDirY;
    double lateral=fabs(cx*(-customDirY)+cy*customDirX);
    if(curAlong < -8.0) customLineArmed=true;
    if(customLineArmed && prevAlong<=0.0 && curAlong>0.0 && lateral<25.0 && speed>5.0f && (lastCrossTow==0 || tow-lastCrossTow>10000u)){
      lastCrossTow=tow; customLineArmed=false;
      if(lapClockRunning){
        uint32_t lap=tow-lapStartTow;
        if(lap>10000u){
          lapLastMs=lap; lapCount++;
          // The fastest completed lap becomes the spatial reference for live delta.
          if(!lapBestMs || lap<lapBestMs){
            lapBestMs=lap;
            refTraceN=curTraceN;
            for(uint16_t i=0;i<refTraceN;i++) refTrace[i]=curTrace[i];
          }
        }
      }
      lapStartTow=tow; lapClockRunning=true; curTraceN=0; refCursor=0; lastTraceTow=0; lapDeltaValid=false;
      Serial.printf("LAP CROSS #%u last=%lu best=%lu\\n",lapCount,(unsigned long)lapLastMs,(unsigned long)lapBestMs);
    }
    if(lapClockRunning){
      uint32_t elapsed=tow-lapStartTow;
      double x0=localX(customLon,customLat), y0=localY(customLat);
      float tx=(float)(localX(lon,customLat)-x0), ty=(float)(localY(lat)-y0);
      // Store current lap at 10 Hz: up to four minutes per reference lap.
      if(curTraceN<LAP_TRACE_MAX && (!lastTraceTow || tow-lastTraceTow>=100u)){
        curTrace[curTraceN++]={tx,ty,elapsed}; lastTraceTow=tow;
      }
      // Live delta is current elapsed time minus reference time at the same track position.
      if(refTraceN>2){
        uint16_t lo=refCursor>12?refCursor-12:0;
        uint16_t hi=(uint16_t)min((int)refTraceN-1,(int)refCursor+40);
        float bestD=1e30f; uint16_t bi=refCursor;
        for(uint16_t i=lo;i<=hi;i++){
          float dx=tx-refTrace[i].x, dy=ty-refTrace[i].y, d=dx*dx+dy*dy;
          if(d<bestD){ bestD=d; bi=i; }
        }
        refCursor=bi;
        if(bestD<2500.0f){
          lapDeltaMs=(int32_t)elapsed-(int32_t)refTrace[bi].t;
          lapDeltaValid=true;
        }
      }
    }
  }
  prevLat=lat; prevLon=lon; havePrevFix=true;
}

static void drawRaceBoxLive(){
  uint16_t black=C(0x0000),white=C(0xFFFF),green=C(0x07E0),gray=C(0x4208),red=C(0xF800);
  float speed; uint8_t fix,sats; uint32_t packets,tow; bool valid;
  portENTER_CRITICAL(&rbDataMux);
  speed=rbSpeedKmh; fix=rbFix; sats=rbSats; packets=rbLivePackets; valid=rbLiveValid; tow=rbTowMs;
  portEXIT_CRITICAL(&rbDataMux);
  for(size_t i=0;i<180u*640u;i++)screen[i]=black;
  rect(0,0,640,4,green);
  rect(12,12,120,22,rbConnected?green:red);
  rect(148,12,120,22,valid?green:gray);
  char lap[16]; uint32_t elapsed=(lapClockRunning && tow>=lapStartTow)?tow-lapStartTow:0;
  fmtLap(elapsed,lap,sizeof(lap));
  num(22,58,lap,6,white);
  // SAT is only a status lamp: green when at least one satellite is visible.
  text5(12,12,"SAT",3,sats>0?green:red);
  // Explicit START/META button: red before setting, blue after successful setting.
  uint16_t blue=C(0x001F);
  rect(500,8,128,34,customLineValid?blue:red);
  text5(510,15,"START",2,white);
  text5(574,15,"META",2,white);
  // Right column: LAST, BEST and continuously updated spatial DELTA.
  text5(390,50,"LAST",2,white);
  if(lapLastMs){ char t[16]; fmtLap(lapLastMs,t,sizeof(t)); num(470,48,t,2,white); }
  text5(390,86,"BEST",2,green);
  if(lapBestMs){ char t[16]; fmtLap(lapBestMs,t,sizeof(t)); num(470,84,t,2,green); }
  text5(390,122,"DELTA",2,white);
  if(lapDeltaValid){ char dt[16]; fmtDelta(lapDeltaMs,dt,sizeof(dt)); num(500,120,dt,2,lapDeltaMs<=0?green:red); }
  // Packet counter kept internally; do not show it on the normal dashboard.
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

  // Startup splash shown while the 10 s BLE scan is running.
  rect(0,0,640,180,black);
  rect(0,0,640,4,green);
  // Left aligned AEP, with Racing / Development to its right.
  text5(24,48,"AEP",10,white);
  text5(230,48,"Racing",4,white);
  text5(230,88,"Development",3,gray);
  while(transfer_num>1){ lcd_PushColors(0,0,0,0,NULL); delay(1); }
  present();
  while(transfer_num>0 && lcd_PushColors_len>0){ lcd_PushColors(0,0,0,0,NULL); delay(1); }
  Serial.println("BLE SCANNING");
  prefs.begin("proot",true); savedRbAddr=prefs.getString("rbAddr","");
  customLineValid=prefs.getBool("sfOk",false);
  if(customLineValid){ customLat=prefs.getDouble("sfLat",0); customLon=prefs.getDouble("sfLon",0); customDirX=prefs.getDouble("sfDx",0); customDirY=prefs.getDouble("sfDy",0); }
  prefs.end();
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
  updateLapClock();
  if(connState!=drawnConnState){
    drawnConnState=connState;
    if(connState==CONN_OK && connIndex>=0){
      selectedRacebox=connIndex;
      saveRaceBox(raceboxAddr[selectedRacebox]);
    }
    while(transfer_num>1){ lcd_PushColors(0,0,0,0,NULL); delay(1); }
    drawRaceBoxList();
  }
  // After a confirmed RaceBox connection, leave the selector and show the timing
  // screen exactly once. BLE notification callback remains a no-op in this build.
  static bool timingShown=false;
  static uint32_t connOkSince=0;
  if(connState==CONN_OK && rbConnected){
    if(!connOkSince) connOkSince=millis();
    if(!timingShown && millis()-connOkSince>=750){
      // Fully drain any previous LCD DMA transfer before replacing the framebuffer.
      while(lcd_PushColors_len>0){ lcd_PushColors(0,0,0,0,NULL); delay(1); }
      drawRaceBoxLive();
      timingShown=true;
    }
  } else {
    connOkSince=0;
    timingShown=false;
  }

  static uint32_t lastDataDraw=0;
  if(connState==CONN_OK && rbConnected && timingShown && rbLiveValid && millis()-lastDataDraw>=1000){
    lastDataDraw=millis();
    while(lcd_PushColors_len>0){ lcd_PushColors(0,0,0,0,NULL); delay(1); }
    drawRaceBoxLive();
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
  // On the live dashboard a normal tap must not disconnect RaceBox.
  // Navigation/settings will get explicit touch zones later.
  if(connState==CONN_OK && down&&!touchDown){
    // Only the red/blue START META button is active on the dashboard.
    if(x>=500 && x<640 && y>=0 && y<52){
      saveCustomLine();
      while(lcd_PushColors_len>0){ lcd_PushColors(0,0,0,0,NULL); delay(1); }
      drawRaceBoxLive();
    }
  } else if(connState!=CONN_OK && down&&!touchDown&&y>=150 && raceboxCount>4){
    listPage=(listPage+1)%((raceboxCount+3)/4);
    while(transfer_num>1){ lcd_PushColors(0,0,0,0,NULL); delay(1); }
    drawRaceBoxList();
  } else if(connState!=CONN_OK && down&&!touchDown&&x>=12&&x<628&&y>=46&&y<145){
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
