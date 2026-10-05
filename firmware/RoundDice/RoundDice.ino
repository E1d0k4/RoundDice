#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "Arduino_GFX_Library.h"
#include "TouchDrvCSTXXX.hpp"
#include "SensorQMI8658.hpp"
#include "pin_config.h"
#include "esp_heap_caps.h"

namespace {
constexpr uint8_t kTouchAddress = CST92XX_SLAVE_ADDRESS;
constexpr uint8_t kMaxTouchPoints = 2;
constexpr int16_t kSwipeThreshold = 70;
constexpr uint32_t kTouchPollMs = 8;
constexpr uint32_t kImuPollMs = 25;
constexpr uint32_t kRollCooldownMs = 500;
constexpr float kShakeThreshold = 2.20f;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS,LCD_SCLK,LCD_SDIO0,LCD_SDIO1,LCD_SDIO2,LCD_SDIO3);
Arduino_CO5300 *gfx = new Arduino_CO5300(bus,LCD_RESET,0,LCD_WIDTH,LCD_HEIGHT,6,0,0,0);
TouchDrvCST92xx touch;
SensorQMI8658 qmi;

int16_t touchX[kMaxTouchPoints]={},touchY[kMaxTouchPoints]={};
uint32_t lastTouchPollMs=0,lastImuPollMs=0,lastRollMs=0;
uint8_t lastDice=6;
bool menuOpen=false,touchWasDown=false,trackingSwipe=false;
int16_t swipeStartY=0,swipeMinY=0,swipeMaxY=0;
float lastAccelX=0,lastAccelY=0,lastAccelZ=1;
bool imuReady=false;

uint16_t *frame=nullptr;
constexpr int16_t W=LCD_WIDTH,H=LCD_HEIGHT;
constexpr float CUBE=170.0f,FOCAL=470.0f,CX=233.0f,CY=238.0f;
float rotX=-0.42f,rotY=0.58f,rotZ=0.08f;

const uint16_t BG=RGB565_BLACK;
const uint16_t WHITE=RGB565_WHITE;
const uint16_t DICE_BASE=0xD69A;
const uint16_t DICE_BEVEL=0xF7BE;
const uint16_t PIP_DARK=RGB565_BLACK;
const uint16_t PIP_SHADOW=0x0861;


struct V2{float x,y;};
struct V3{float x,y,z;};
struct Face{uint8_t a,b,c,d,value;float depth;};

inline void px(int x,int y,uint16_t c){if((unsigned)x<W&&(unsigned)y<H)frame[y*W+x]=c;}
void clearFrame(){memset(frame,0,W*H*sizeof(uint16_t));}

V3 rotate(V3 p){
  float sx=sinf(rotX),cx=cosf(rotX),sy=sinf(rotY),cy=cosf(rotY),sz=sinf(rotZ),cz=cosf(rotZ);
  float y=p.y*cx-p.z*sx,z=p.y*sx+p.z*cx;p.y=y;p.z=z;
  float x=p.x*cy+p.z*sy;z=-p.x*sy+p.z*cy;p.x=x;p.z=z;
  x=p.x*cz-p.y*sz;y=p.x*sz+p.y*cz;p.x=x;p.y=y;return p;
}
V2 project(V3 p){float s=FOCAL/(FOCAL+p.z);return {CX+p.x*s,CY+p.y*s};}

void fillTriangle(V2 a,V2 b,V2 c,uint16_t col){
  int minX=max(0,(int)floorf(min(a.x,min(b.x,c.x)))),maxX=min(W-1,(int)ceilf(max(a.x,max(b.x,c.x))));
  int minY=max(0,(int)floorf(min(a.y,min(b.y,c.y)))),maxY=min(H-1,(int)ceilf(max(a.y,max(b.y,c.y))));
  float area=(b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);if(fabsf(area)<0.01f)return;
  for(int y=minY;y<=maxY;y++)for(int x=minX;x<=maxX;x++){
    float px0=x+0.5f,py0=y+0.5f;
    float w0=(b.x-a.x)*(py0-a.y)-(b.y-a.y)*(px0-a.x);
    float w1=(c.x-b.x)*(py0-b.y)-(c.y-b.y)*(px0-b.x);
    float w2=(a.x-c.x)*(py0-c.y)-(a.y-c.y)*(px0-c.x);
    if((w0>=0&&w1>=0&&w2>=0)||(w0<=0&&w1<=0&&w2<=0))px(x,y,col);
  }
}
void drawFilledCircle(V2 center,float radius,uint16_t col){
  int minX=max(0,(int)floorf(center.x-radius)),maxX=min(W-1,(int)ceilf(center.x+radius));
  int minY=max(0,(int)floorf(center.y-radius)),maxY=min(H-1,(int)ceilf(center.y+radius));
  float r2=radius*radius;
  for(int y=minY;y<=maxY;y++)for(int x=minX;x<=maxX;x++){
    float dx=(x+0.5f)-center.x,dy=(y+0.5f)-center.y;if(dx*dx+dy*dy<=r2)px(x,y,col);
  }
}
V3 facePoint(const V3&a,const V3&b,const V3&c,const V3&d,float u,float v){
  float s=(u+1.0f)*0.5f,t=(v+1.0f)*0.5f;
  return {(1-s)*(1-t)*a.x+s*(1-t)*b.x+s*t*c.x+(1-s)*t*d.x,
          (1-s)*(1-t)*a.y+s*(1-t)*b.y+s*t*c.y+(1-s)*t*d.y,
          (1-s)*(1-t)*a.z+s*(1-t)*b.z+s*t*c.z+(1-s)*t*d.z};
}
V3 faceNormal(const Face&q,const V3 v[]){
  V3 a=v[q.a],b=v[q.b],c=v[q.c];
  V3 ab{b.x-a.x,b.y-a.y,b.z-a.z},ac{c.x-a.x,c.y-a.y,c.z-a.z};
  return {ab.y*ac.z-ab.z*ac.y,ab.z*ac.x-ab.x*ac.z,ab.x*ac.y-ab.y*ac.x};
}
void fillRoundedFace(const Face&q,const V3 v[],uint16_t col){
  // Full painted face. The bevel is added separately and uses almost the same tone,
  // so the die reads as one solid piece instead of a wireframe model.
  V2 p0=project(v[q.a]),p1=project(v[q.b]),p2=project(v[q.c]),p3=project(v[q.d]);
  fillTriangle(p0,p1,p2,col);fillTriangle(p0,p2,p3,col);
}

void drawSoftBevel(const Face&q,const V3 v[]){
  // No inset contour: the die stays visually flat-painted. Rounded geometry
  // is produced by a soft edge highlight rather than a rectangular line.
  (void)q;(void)v;
}
void drawRecessedPips(const Face&q,const V3 v[]){
  static const float pips[6][7][2]={
    {{0,0}},{{-.52f,-.52f},{.52f,.52f}},{{-.52f,-.52f},{0,0},{.52f,.52f}},
    {{-.52f,-.52f},{.52f,-.52f},{-.52f,.52f},{.52f,.52f}},
    {{-.52f,-.52f},{.52f,-.52f},{0,0},{-.52f,.52f},{.52f,.52f}},
    {{-.52f,-.58f},{-.52f,0},{-.52f,.58f},{.52f,-.58f},{.52f,0},{.52f,.58f}}
  };
  V3 A=v[q.a],B=v[q.b],D=v[q.d];
  V3 u{B.x-A.x,B.y-A.y,B.z-A.z},vv{D.x-A.x,D.y-A.y,D.z-A.z};
  float ul=sqrtf(u.x*u.x+u.y*u.y+u.z*u.z),vl=sqrtf(vv.x*vv.x+vv.y*vv.y+vv.z*vv.z);
  if(ul<1||vl<1)return;
  u.x/=ul;u.y/=ul;u.z/=ul;vv.x/=vl;vv.y/=vl;vv.z/=vl;
  V3 n{u.y*vv.z-u.z*vv.y,u.z*vv.x-u.x*vv.z,u.x*vv.y-u.y*vv.x};
  float nl=sqrtf(n.x*n.x+n.y*n.y+n.z*n.z);if(nl<0.01f)return;
  n.x/=nl;n.y/=nl;n.z/=nl;
  for(uint8_t i=0;i<q.value;i++){
    V3 p=facePoint(v[q.a],v[q.b],v[q.c],v[q.d],pips[q.value-1][i][0],pips[q.value-1][i][1]);
    // Push the centre of the pip visibly into the face. A broad dark cavity
    // plus a smaller offset shadow removes the impression of a raised nub.
    p.x-=n.x*5.0f;p.y-=n.y*5.0f;p.z-=n.z*5.0f;
    V2 s=project(p);float scale=FOCAL/(FOCAL+p.z),r=11.0f*scale;
    drawFilledCircle(s,r,PIP_DARK);
    V2 shadow{s.x+r*0.16f,s.y+r*0.16f};
    drawFilledCircle(shadow,r*0.48f,PIP_SHADOW);
    drawFilledCircle(s,r*0.72f,PIP_DARK);
  }
}}