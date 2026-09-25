#include "racebox.h"
#include <math.h>

namespace {
portMUX_TYPE dataMux=portMUX_INITIALIZER_UNLOCKED;
RaceBoxTelemetry state;
uint8_t stream[512];
size_t streamLen=0;
NimBLEClient *client=nullptr;
NimBLERemoteCharacteristic *rx=nullptr;
uint32_t leanTow=0;
enum RecordPending : uint8_t { REC_NONE, REC_START, REC_STOP };
RecordPending recordPending=REC_NONE;
bool recordingOn=true;
constexpr uint32_t SECURITY_CODE=123456u;

void sendRecordingConfig(bool enable){
  uint8_t p[12]={0};
  if(enable){p[0]=1;p[1]=0;p[2]=0x01;}
  RaceBox::sendUbx(0xFF,0x25,p,sizeof(p));
}
}

namespace RaceBox {
void reset(){
  portENTER_CRITICAL(&dataMux);
  state=RaceBoxTelemetry{};
  streamLen=0;
  leanTow=0;
  portEXIT_CRITICAL(&dataMux);
  client=nullptr; rx=nullptr; recordPending=REC_NONE; recordingOn=true;

  // Bring the BLE controller up explicitly before main.cpp starts discovery.
  // This keeps BLE startup deterministic after the RaceBox code was moved
  // out of main.cpp during the modularisation refactor.
  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  delay(50);
}

void setConnection(NimBLEClient *c,NimBLERemoteCharacteristic *r,bool connected){
  client=c;rx=r;
  portENTER_CRITICAL(&dataMux);state.connected=connected;portEXIT_CRITICAL(&dataMux);
}

void notify(NimBLERemoteCharacteristic*,uint8_t *data,size_t len,bool){
  portENTER_CRITICAL(&dataMux);
  size_t freeBytes=sizeof(stream)-streamLen;
  size_t take=len<freeBytes?len:freeBytes;
  if(take){memcpy(stream+streamLen,data,take);streamLen+=take;}
  portEXIT_CRITICAL(&dataMux);
}

bool sendUbx(uint8_t cls,uint8_t id,const uint8_t *payload,uint16_t plen){
  if(!state.connected||!rx)return false;
  uint8_t pkt[32];if((size_t)plen+8u>sizeof(pkt))return false;
  pkt[0]=0xB5;pkt[1]=0x62;pkt[2]=cls;pkt[3]=id;pkt[4]=(uint8_t)(plen&0xFF);pkt[5]=(uint8_t)(plen>>8);
  if(plen&&payload)memcpy(pkt+6,payload,plen);
  uint8_t a=0,b=0;for(size_t i=2;i<6u+plen;i++){a=(uint8_t)(a+pkt[i]);b=(uint8_t)(b+a);}
  pkt[6+plen]=a;pkt[7+plen]=b;
  bool ok=rx->writeValue(pkt,(size_t)plen+8u,true);
  Serial.printf("RB CMD %02X/%02X %s\n",cls,id,ok?"sent":"failed");return ok;
}

void requestRecording(bool start){
  recordingOn=start;recordPending=start?REC_START:REC_STOP;
  uint8_t p[4]={(uint8_t)(SECURITY_CODE&0xFF),(uint8_t)((SECURITY_CODE>>8)&0xFF),(uint8_t)((SECURITY_CODE>>16)&0xFF),(uint8_t)((SECURITY_CODE>>24)&0xFF)};
  if(!sendUbx(0xFF,0x30,p,sizeof(p)))recordPending=REC_NONE;
}

bool recordingEnabled(){return recordingOn;}

void process(){
  static uint8_t fifo[1024];static size_t fifoLen=0;uint8_t local[512];size_t n=0;
  portENTER_CRITICAL(&dataMux);n=streamLen;if(n){memcpy(local,stream,n);streamLen=0;}portEXIT_CRITICAL(&dataMux);
  if(n){if(n>sizeof(fifo)-fifoLen)fifoLen=0;if(n<=sizeof(fifo)-fifoLen){memcpy(fifo+fifoLen,local,n);fifoLen+=n;}}
  while(fifoLen>=8){
    size_t s=0;while(s+1<fifoLen&&!(fifo[s]==0xB5&&fifo[s+1]==0x62))s++;
    if(s){memmove(fifo,fifo+s,fifoLen-s);fifoLen-=s;if(fifoLen<8)break;}
    uint16_t plen=(uint16_t)fifo[4]|((uint16_t)fifo[5]<<8);size_t fl=(size_t)plen+8;
    if(fl>sizeof(fifo)){fifoLen=0;break;}if(fifoLen<fl)break;
    uint8_t a=0,b=0;for(size_t i=2;i<6u+plen;i++){a=(uint8_t)(a+fifo[i]);b=(uint8_t)(b+a);}
    if(a==fifo[6+plen]&&b==fifo[7+plen]&&fifo[2]==0xFF&&(fifo[3]==0x02||fifo[3]==0x03)&&plen>=2){
      bool ack=fifo[3]==0x02;uint8_t ackCls=fifo[6],ackId=fifo[7];Serial.printf("RB %s %02X/%02X\n",ack?"ACK":"NACK",ackCls,ackId);
      if(ackCls==0xFF&&ackId==0x30&&recordPending!=REC_NONE){RecordPending cmd=recordPending;recordPending=REC_NONE;if(ack)sendRecordingConfig(cmd==REC_START);}
    } else if(a==fifo[6+plen]&&b==fifo[7+plen]&&fifo[2]==0xFF&&fifo[3]==0x01&&plen>=80){
      const uint8_t *p=fifo+6;uint32_t speedMm=0,tow=0;int32_t lonRaw=0,latRaw=0;int16_t gY=0,gZ=0,gyroX=0;
      memcpy(&tow,p+0,4);memcpy(&lonRaw,p+24,4);memcpy(&latRaw,p+28,4);memcpy(&speedMm,p+48,4);memcpy(&gY,p+70,2);memcpy(&gZ,p+72,2);memcpy(&gyroX,p+74,2);
      float speed=(float)speedMm*0.0036f;float accelRoll=atan2f((float)gY,(float)gZ)*57.2957795f;float lean=state.leanDeg;
      if(!state.leanValid){lean=accelRoll;}else{uint32_t dms=tow-leanTow;if(dms>0&&dms<250u){float dt=dms*0.001f;lean+=(float)gyroX*0.01f*dt;if(speed<8.0f)lean=0.96f*lean+0.04f*accelRoll;}}
      if(lean>89.9f)lean=89.9f;if(lean<-89.9f)lean=-89.9f;leanTow=tow;
      portENTER_CRITICAL(&dataMux);state.towMs=tow;state.lon=(double)lonRaw/10000000.0;state.lat=(double)latRaw/10000000.0;state.fix=p[20];state.sats=p[23];state.speedKmh=speed;state.leanDeg=lean;state.leanValid=true;state.livePackets++;state.liveValid=true;portEXIT_CRITICAL(&dataMux);
    }
    memmove(fifo,fifo+fl,fifoLen-fl);fifoLen-=fl;
  }
}

const RaceBoxTelemetry &telemetry(){return state;}
}
