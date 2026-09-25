#include "laptimer.h"
#include "tracks_europe.h"
#include <math.h>
#include <string.h>

namespace {
LapTimerState s;
constexpr uint8_t HISTORY_MAX=99;
uint32_t historyBuf[HISTORY_MAX]={0};
struct LapPoint { float x,y; uint32_t t; };
constexpr uint16_t TRACE_MAX=2400;
LapPoint refTrace[TRACE_MAX],curTrace[TRACE_MAX];
uint16_t refN=0,curN=0,refCursor=0;
uint32_t lastTraceTow=0,lastCrossTow=0,lastTrackDetectMs=0;
bool lineValid=false,lineArmed=false,directionPending=false,havePrevFix=false;
double lineLat=0,lineLon=0,dirX=0,dirY=0,prevLat=0,prevLon=0;
inline double localX(double lon,double lat0){return lon*111320.0*cos(lat0*0.017453292519943295);}
inline double localY(double lat){return lat*110540.0;}
void clearLapData(){s.lapStartTow=s.lastLapMs=s.bestLapMs=0;s.deltaMs=0;s.deltaValid=false;s.lapCount=0;s.flashStarted=0;s.historyCount=s.historyPage=0;memset(historyBuf,0,sizeof(historyBuf));refN=curN=refCursor=0;lastTraceTow=lastCrossTow=0;}
void detectFactoryTrack(double lat,double lon){if(lineValid||s.factoryTrack||millis()-lastTrackDetectMs<2000u)return;lastTrackDetectMs=millis();double bestD2=10000.0*10000.0;int best=-1;for(size_t i=0;i<PROOT_EUROPE_TRACK_COUNT;i++){ProotTrackLine t;memcpy_P(&t,&PROOT_EUROPE_TRACKS[i],sizeof(t));if(!t.enabled)continue;double aLat=t.lat1*1e-7,aLon=t.lon1*1e-7,bLat=t.lat2*1e-7,bLon=t.lon2*1e-7,mLat=(aLat+bLat)*0.5,mLon=(aLon+bLon)*0.5;double dx=(lon-mLon)*111320.0*cos(lat*0.017453292519943295),dy=(lat-mLat)*110540.0,d2=dx*dx+dy*dy;if(d2<bestD2){bestD2=d2;best=(int)i;}}if(best<0)return;ProotTrackLine t;memcpy_P(&t,&PROOT_EUROPE_TRACKS[best],sizeof(t));double aLat=t.lat1*1e-7,aLon=t.lon1*1e-7,bLat=t.lat2*1e-7,bLon=t.lon2*1e-7;lineLat=(aLat+bLat)*0.5;lineLon=(aLon+bLon)*0.5;double sx=localX(bLon,lineLat)-localX(aLon,lineLat),sy=localY(bLat)-localY(aLat),nn=sqrt(sx*sx+sy*sy);if(nn<0.5)return;dirX=-sy/nn;dirY=sx/nn;if(havePrevFix){double vx=localX(lon,lat)-localX(prevLon,lat),vy=localY(lat)-localY(prevLat);if(vx*dirX+vy*dirY<0){dirX=-dirX;dirY=-dirY;}}lineValid=true;directionPending=false;lineArmed=false;s.factoryTrack=true;s.factoryTrackIndex=best;s.customLine=false;lastCrossTow=0;}
}
namespace LapTimer {
void reset(){s=LapTimerState{};memset(historyBuf,0,sizeof(historyBuf));lineValid=lineArmed=directionPending=havePrevFix=false;lineLat=lineLon=dirX=dirY=prevLat=prevLon=0;refN=curN=refCursor=0;lastTraceTow=lastCrossTow=lastTrackDetectMs=0;}
void setCustomStartFinish(const RaceBoxTelemetry &gps){if(gps.fix<2)return;s.factoryTrack=false;s.factoryTrackIndex=-1;s.customLine=true;lineLat=gps.lat;lineLon=gps.lon;lineValid=true;lineArmed=false;directionPending=true;if(havePrevFix&&gps.speedKmh>=3.0f){double dx=localX(gps.lon,gps.lat)-localX(prevLon,gps.lat),dy=localY(gps.lat)-localY(prevLat),nn=sqrt(dx*dx+dy*dy);if(nn>=0.20){dirX=dx/nn;dirY=dy/nn;directionPending=false;}}clearLapData();s.lapStartTow=gps.towMs;s.running=true;s.stopped=false;}
void restoreCustomStartFinish(double lat,double lon,double dx,double dy){lineLat=lat;lineLon=lon;dirX=dx;dirY=dy;lineValid=true;lineArmed=false;directionPending=(fabs(dx)+fabs(dy)<0.01);s.customLine=true;s.factoryTrack=false;s.factoryTrackIndex=-1;}
LapTimerLine line(){LapTimerLine r;r.valid=lineValid;r.lat=lineLat;r.lon=lineLon;r.dirX=dirX;r.dirY=dirY;return r;}
void update(const RaceBoxTelemetry &gps){if(gps.fix<2)return;double lat=gps.lat,lon=gps.lon;uint32_t tow=gps.towMs;float speed=gps.speedKmh;detectFactoryTrack(lat,lon);if(lineValid&&directionPending){double dx=localX(lon,lineLat)-localX(lineLon,lineLat),dy=localY(lat)-localY(lineLat),nn=sqrt(dx*dx+dy*dy);if(nn>=3.0&&speed>=3.0f){dirX=dx/nn;dirY=dy/nn;directionPending=false;}}if(lineValid&&!directionPending&&havePrevFix&&!s.stopped){double x0=localX(lineLon,lineLat),y0=localY(lineLat),px=localX(prevLon,lineLat)-x0,py=localY(prevLat)-y0,cx=localX(lon,lineLat)-x0,cy=localY(lat)-y0,prevAlong=px*dirX+py*dirY,curAlong=cx*dirX+cy*dirY,lateral=fabs(cx*(-dirY)+cy*dirX);if(curAlong<-5.0)lineArmed=true;if(lineArmed&&prevAlong<=0.0&&curAlong>0.0&&lateral<50.0&&speed>5.0f&&(lastCrossTow==0||tow-lastCrossTow>10000u)){lastCrossTow=tow;lineArmed=false;if(s.running){uint32_t lap=tow-s.lapStartTow;if(lap>10000u){s.lastLapMs=lap;s.lapCount++;if(s.historyCount<HISTORY_MAX)historyBuf[s.historyCount++]=lap;s.historyPage=s.historyCount?(s.historyCount-1)/3:0;s.flashStarted=millis();if(!s.bestLapMs||lap<s.bestLapMs){s.bestLapMs=lap;refN=curN;for(uint16_t i=0;i<refN;i++)refTrace[i]=curTrace[i];}}}s.lapStartTow=tow;s.running=true;curN=0;refCursor=0;lastTraceTow=0;s.deltaValid=false;}if(s.running){uint32_t elapsed=tow-s.lapStartTow;float tx=(float)(localX(lon,lineLat)-x0),ty=(float)(localY(lat)-y0);if(curN<TRACE_MAX&&(!lastTraceTow||tow-lastTraceTow>=100u)){curTrace[curN++]={tx,ty,elapsed};lastTraceTow=tow;}if(refN>2){uint16_t lo=refCursor>12?refCursor-12:0,hi=(uint16_t)min((int)refN-1,(int)refCursor+40);float bestD=1e30f;uint16_t bi=refCursor;for(uint16_t i=lo;i<=hi;i++){float dx=tx-refTrace[i].x,dy=ty-refTrace[i].y,d=dx*dx+dy*dy;if(d<bestD){bestD=d;bi=i;}}refCursor=bi;if(bestD<2500.0f){s.deltaMs=(int32_t)elapsed-(int32_t)refTrace[bi].t;s.deltaValid=true;}}}}prevLat=lat;prevLon=lon;havePrevFix=true;}
void stop(){s.running=false;s.stopped=true;s.deltaValid=false;}
void resume(){s.stopped=false;}
void historyPageUp(){if(s.historyPage>0)s.historyPage--;}
void historyPageDown(){uint8_t maxPage=s.historyCount?(s.historyCount-1)/3:0;if(s.historyPage<maxPage)s.historyPage++;}
uint32_t history(uint8_t index){return index<s.historyCount?historyBuf[index]:0;}
const LapTimerState &state(){return s;}
}
