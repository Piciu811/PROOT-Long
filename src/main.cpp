#include <Arduino.h>
#include "AXS15231B.h"
#include "esp_heap_caps.h"
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <Preferences.h>
#include <Wire.h>

extern uint32_t transfer_num;
extern size_t lcd_PushColors_len;

static uint16_t *nativeFrame=nullptr,*screen=nullptr;
static String raceboxes[8];
static String raceboxAddr[8];
static int raceboxCount=0, selectedRacebox=0;
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
  BLEDevice::init("");
  BLEScan *scan=BLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
  BLEScanResults found=scan->start(10,false);
  raceboxCount=0;
  bleSeen=found.getCount();
  BLEUUID rbService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");

  // Do not discard devices just because RaceBox does not advertise its UART service/name.
  // Show the strongest BLE devices and let the user select the nearby unit.
  int used[8]; for(int i=0;i<8;i++) used[i]=-1;
  for(int slot=0;slot<8;slot++){
    int best=-1,bestRssi=-999;
    for(int i=0;i<found.getCount();i++){
      bool already=false; for(int k=0;k<slot;k++) if(used[k]==i) already=true;
      if(already) continue;
      BLEAdvertisedDevice d=found.getDevice(i);
      if(d.getRSSI()>bestRssi){best=i;bestRssi=d.getRSSI();}
    }
    if(best<0) break;
    used[slot]=best;
    BLEAdvertisedDevice d=found.getDevice(best);
    String name=d.haveName()?String(d.getName().c_str()):String("BLE");
    bool serviceMatch=d.haveServiceUUID() && d.isAdvertisingService(rbService);
    raceboxes[raceboxCount]=name;
    raceboxAddr[raceboxCount]=String(d.getAddress().toString().c_str());
    Serial.printf("BLE candidate %d name='%s' addr=%s RSSI=%d RBsvc=%d\\n",
      raceboxCount+1,name.c_str(),raceboxAddr[raceboxCount].c_str(),d.getRSSI(),serviceMatch);
    raceboxCount++;
  }
  scan->clearResults();
}
static void saveRaceBox(const String &addr){
  prefs.begin("proot",false); prefs.putString("rbAddr",addr); prefs.end();
  savedRbAddr=addr;
}
static BLEClient *rbClient=nullptr;
static bool probeRaceBoxAddress(const String &addr){
  BLEClient *client=BLEDevice::createClient();
  Serial.printf("PROBE %s...\\n",addr.c_str());
  if(!client->connect(BLEAddress(addr.c_str()))){
    Serial.println("PROBE connect failed"); delete client; return false;
  }
  BLEUUID svc("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
  BLERemoteService *s=client->getService(svc);
  if(!s){
    Serial.println("PROBE not RaceBox");
    client->disconnect(); delete client; return false;
  }
  rbClient=client;
  rbConnected=true; connectedAddr=addr; saveRaceBox(addr);
  Serial.printf("RACEBOX CONFIRMED %s\\n",addr.c_str());
  return true;
}
static bool connectSelectedRaceBox(){
  if(selectedRacebox<0 || selectedRacebox>=raceboxCount) return false;
  return probeRaceBoxAddress(raceboxAddr[selectedRacebox]);
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
  // Total BLE advertisements seen, for hardware diagnostics even when no RaceBox matches.
  num(330,10,String(bleSeen).c_str(),4,white);
  // One visible row per discovered RaceBox (up to 4 on 180px screen).
  for(int i=0;i<raceboxCount && i<4;i++){
    int y=46+i*31;
    rect(12,y,616,25,(rbConnected&&i==selectedRacebox)?green:gray);
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

  // PRÖÖT 640x180 dashboard skeleton.
  rect(0,0,640,3,green);
  rect(0,142,640,2,gray);
  rect(420,3,2,139,gray);
  rect(10,12,395,116,C(0x0841));
  num(34,28,"123.4",10,white);          // speed placeholder
  num(438,18,"0:00.000",4,green);       // delta/current placeholder
  num(438,66,"1:23.456",3,white);       // last
  num(438,102,"1:22.987",3,green);      // best
  rect(12,151,90,18,green);             // RaceBox status
  rect(112,151,90,18,gray);             // GPS status
  rect(212,151,90,18,red);              // REC/status
  num(535,149,"12",3,white);             // lap/sats placeholder

  present();
  Serial.println("PROOT UI SKELETON SENT");
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
  int x,y; bool down=readTouch(x,y);
  static uint32_t lastDiag=0;
  if(millis()-lastDiag>1000){
    lastDiag=millis();
    Wire.beginTransmission(TOUCH_ADDR);
    int err=Wire.endTransmission();
    Serial.printf("TOUCH I2C=%d INT=%d\\n",err,digitalRead(TOUCH_INT));
  }
  if(down&&!touchDown){
    Serial.printf("TOUCH DOWN x=%d y=%d\\n",x,y);
    rect(x-12,y-12,24,24,C(0xFFFF));
    while(transfer_num>1){ lcd_PushColors(0,0,0,0,NULL); delay(1); }
    present();
  }
  if(down&&!touchDown&&x>=12&&x<628&&y>=46){
    int idx=(y-46)/31;
    if(idx>=0&&idx<raceboxCount&&idx<4){
      selectedRacebox=idx;
      Serial.printf("Selected BLE %d: %s %s\\n",idx+1,raceboxes[idx].c_str(),raceboxAddr[idx].c_str());
      bool ok=connectSelectedRaceBox();
      // Immediate visible result: green row = confirmed RaceBox, red marker = not RaceBox/connect failed.
      if(ok) rect(12,46+idx*31,616,25,C(0x07E0));
      else rect(600,46+idx*31,28,25,C(0xF800));
      // Do not overwrite framebuffer while its previous DMA transfer is still queued.
      while(transfer_num>1){ lcd_PushColors(0,0,0,0,NULL); delay(1); }
      drawRaceBoxList();
    }
  }
  touchDown=down;
  delay(20);
}
