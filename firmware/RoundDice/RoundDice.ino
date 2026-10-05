#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "Arduino_GFX_Library.h"
#include "TouchDrvCSTXXX.hpp"
#include "SensorQMI8658.hpp"
#include "pin_config.h"

namespace {
constexpr uint8_t kTouchAddress = CST92XX_SLAVE_ADDRESS;
constexpr uint8_t kMaxTouchPoints = 2;
constexpr int16_t kSwipeThreshold = 70;
constexpr uint32_t kTouchPollMs = 8;
constexpr uint32_t kImuPollMs = 20;
constexpr uint32_t kRollCooldownMs = 500;
constexpr float kShakeThreshold = 2.20f;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_CO5300 *gfx = new Arduino_CO5300(
    bus, LCD_RESET, 0, LCD_WIDTH, LCD_HEIGHT, 6, 0, 0, 0);
TouchDrvCST92xx touch;
SensorQMI8658 qmi;

int16_t touchX[kMaxTouchPoints] = {}, touchY[kMaxTouchPoints] = {};
uint8_t lastDice = 6;
uint32_t lastTouchPollMs = 0, lastImuPollMs = 0, lastRollMs = 0;

bool menuOpen = false, touchWasDown = false, trackingSwipe = false;
int16_t swipeStartY = 0, swipeMinY = 0, swipeMaxY = 0;
float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 1;
bool imuReady = false;

// A small local RGB565 framebuffer. Only this rectangle is redrawn.
constexpr int16_t FB_X = 88, FB_Y = 88, FB_W = 290, FB_H = 290;
uint16_t fb[FB_W * FB_H];

constexpr float CUBE = 170.0f;
constexpr float FOCAL = 390.0f;
constexpr float CX = 145.0f;
constexpr float CY = 143.0f;
float rotX = -0.30f, rotY = 0.48f, rotZ = 0.0f;
float velX = 0.0f, velY = 0.0f, velZ = 0.0f;

const uint16_t BG = RGB565_BLACK;
const uint16_t FRONT = 0x21C7;
const uint16_t TOP = 0x4A69;
const uint16_t RIGHT = 0x1164;
const uint16_t LEFT = 0x1A05;
const uint16_t EDGE = RGB565_CYAN;
const uint16_t HILITE = 0xDFFF;
const uint16_t SHADOW = 0x1082;
const uint16_t PIP = RGB565_WHITE;

struct V2 { float x, y; };
struct V3 { float x, y, z; };
struct Face { uint8_t a, b, c, d, value; float depth; };

void putPixel(int x, int y, uint16_t c) {
  if (x >= 0 && x < FB_W && y >= 0 && y < FB_H) fb[y * FB_W + x] = c;
}

void clearFb() {
  for (size_t i = 0; i < sizeof(fb) / sizeof(fb[0]); ++i) fb[i] = BG;
}

void presentFb() {
  gfx->draw16bitRGBBitmap(FB_X, FB_Y, fb, FB_W, FB_H);
}

V3 rotate(V3 p) {
  const float sx = sinf(rotX), cx = cosf(rotX);
  const float sy = sinf(rotY), cy = cosf(rotY);
  const float sz = sinf(rotZ), cz = cosf(rotZ);

  float y = p.y * cx - p.z * sx;
  float z = p.y * sx + p.z * cx;
  p.y = y; p.z = z;

  float x = p.x * cy + p.z * sy;
  z = -p.x * sy + p.z * cy;
  p.x = x; p.z = z;

  x = p.x * cz - p.y * sz;
  y = p.x * sz + p.y * cz;
  p.x = x; p.y = y;
  return p;
}

V2 project(V3 p) {
  const float scale = FOCAL / (FOCAL + p.z);
  return {CX + p.x * scale, CY + p.y * scale};
}

void fillTriangle(V2 a, V2 b, V2 c, uint16_t color) {
  int minX = max(0, (int)floorf(min(a.x, min(b.x, c.x))));
  int maxX = min(FB_W - 1, (int)ceilf(max(a.x, max(b.x, c.x))));
  int minY = max(0, (int)floorf(min(a.y, min(b.y, c.y))));
  int maxY = min(FB_H - 1, (int)ceilf(max(a.y, max(b.y, c.y))));
  const float area = (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x);
  if (fabsf(area) < 0.01f) return;
  for (int y=minY; y<=maxY; ++y) for (int x=minX; x<=maxX; ++x) {
    V2 p={(float)x+0.5f,(float)y+0.5f};
    float w0=(b.x-a.x)*(p.y-a.y)-(b.y-a.y)*(p.x-a.x);
    float w1=(c.x-b.x)*(p.y-b.y)-(c.y-b.y)*(p.x-b.x);
    float w2=(a.x-c.x)*(p.y-c.y)-(a.y-c.y)*(p.x-c.x);
    if ((w0>=0 && w1>=0 && w2>=0)||(w0<=0 && w1<=0 && w2<=0)) putPixel(x,y,color);
  }
}

void line(V2 a, V2 b, uint16_t c) {
  int x0=(int)lroundf(a.x), y0=(int)lroundf(a.y), x1=(int)lroundf(b.x), y1=(int)lroundf(b.y);
  int dx=abs(x1-x0), sx=x0<x1?1:-1, dy=-abs(y1-y0), sy=y0<y1?1:-1, err=dx+dy;
  while(true){ putPixel(x0,y0,c); if(x0==x1&&y0==y1)break; int e2=2*err; if(e2>=dy){err+=dy;x0+=sx;} if(e2<=dx){err+=dx;y0+=sy;} }
}

void pip(V2 p, int r=8) {
  for(int y=-r-2;y<=r+2;++y) for(int x=-r-2;x<=r+2;++x)
    if(x*x+y*y <= (r+2)*(r+2)) putPixel((int)p.x+x,(int)p.y+y,0x0000);
  for(int y=-r;y<=r;++y) for(int x=-r;x<=r;++x)
    if(x*x+y*y <= r*r) putPixel((int)p.x+x,(int)p.y+y,PIP);
}

void drawFacePips(uint8_t value, const V2 &a,const V2 &b,const V2 &c,const V2 &d) {
  // Bilinear interpolation keeps the pips attached to the perspective face.
  auto point=[&](float u,float v)->V2 {
    return {a.x*(1-u)*(1-v)+b.x*u*(1-v)+d.x*(1-u)*v+c.x*u*v,
            a.y*(1-u)*(1-v)+b.y*u*(1-v)+d.y*(1-u)*v+c.y*u*v};
  };
  const float q=0.25f, m=0.5f, t=0.75f;
  V2 p[9]={point(q,q),point(m,q),point(t,q),point(q,m),point(m,m),point(t,m),point(q,t),point(m,t),point(t,t)};
  static const uint8_t map[7][6]={{},{4,0,0,0,0,0},{0,8,0,0,0,0},{0,4,8,0,0,0},{0,2,6,0,0,0},{0,2,4,6,8,0},{0,2,3,5,7,9}};
  for(uint8_t i=0;i<value;++i){ uint8_t idx=map[value][i]; if(idx) pip(p[idx-1]); }
}

void renderCube(bool moving=false) {
  clearFb();

  const float h=CUBE/2.0f;
  V3 v[8]={{-h,-h,-h},{h,-h,-h},{h,h,-h},{-h,h,-h},{-h,-h,h},{h,-h,h},{h,h,h},{-h,h,h}};
  for(auto &p:v) p=rotate(p);

  V2 s[8]; for(int i=0;i<8;++i) s[i]=project(v[i]);

  Face faces[6]={
    {0,1,2,3,1,0},{4,7,6,5,6,0},{0,4,5,1,2,0},
    {3,2,6,7,4,0},{0,3,7,4,5,0},{1,5,6,2,6,0}
  };
  for(auto &f:faces) f.depth=(v[f.a].z+v[f.b].z+v[f.c].z+v[f.d].z)*0.25f;

  // Painter's algorithm: farthest face first.
  for(int i=0;i<6;++i) for(int j=i+1;j<6;++j) if(faces[i].depth < faces[j].depth){ Face t=faces[i];faces[i]=faces[j];faces[j]=t; }

  for(const Face &f:faces) {
    // Only faces facing the viewer are visible.
    const V3 A=v[f.a], B=v[f.b], C=v[f.c];
    const float nx=(B.y-A.y)*(C.z-A.z)-(B.z-A.z)*(C.y-A.y);
    const float ny=(B.z-A.z)*(C.x-A.x)-(B.x-A.x)*(C.z-A.z);
    const float nz=(B.x-A.x)*(C.y-A.y)-(B.y-A.y)*(C.x-A.x);
    if(nz <= 0.0f) continue;

    uint16_t col = f.value==1 ? FRONT : (f.value==2 ? RIGHT : (f.value==6 ? TOP : LEFT));
    fillTriangle(s[f.a],s[f.b],s[f.c],col);
    fillTriangle(s[f.a],s[f.c],s[f.d],col);
    line(s[f.a],s[f.b],EDGE); line(s[f.b],s[f.c],EDGE);
    line(s[f.c],s[f.d],EDGE); line(s[f.d],s[f.a],EDGE);

    if(f.value==1) drawFacePips(lastDice,s[f.a],s[f.b],s[f.c],s[f.d]);
  }

  // Subtle local shadow, no full-screen erase.
  if(!moving) {
    // A small highlight on the projected top silhouette.
    line(s[0],s[1],HILITE);
  }
  presentFb();
}

void drawTextAt(const char *text,int16_t x,int16_t y,uint8_t size,uint16_t color){
  gfx->setTextSize(size); gfx->setTextColor(color); gfx->setCursor(x,y); gfx->print(text);
}

void drawStaticScreen(){
  gfx->fillScreen(BG);
  drawTextAt("ROUND DICE",145,25,3,RGB565_WHITE);
  drawTextAt("TRUE 3D",188,62,2,RGB565_CYAN);
  drawTextAt("TOUCH  •  SHAKE  •  TILT",130,447,1,RGB565_CYAN);
  char r[16]; snprintf(r,sizeof(r),"D6  %u",(unsigned)lastDice); drawTextAt(r,210,425,2,RGB565_YELLOW);
}

void drawResult(){ gfx->fillRect(195,418,80,28,BG); char r[16]; snprintf(r,sizeof(r),"D6  %u",(unsigned)lastDice); drawTextAt(r,210,425,2,RGB565_YELLOW); }

void drawMenu(){
  gfx->fillRoundRect(18,8,430,285,28,RGB565_DARKGREY);
  gfx->drawRoundRect(18,8,430,285,28,RGB565_CYAN);
  drawTextAt("MENU",190,30,3,RGB565_WHITE);
  drawTextAt("Einstellungen",105,88,2,RGB565_WHITE);
  drawTextAt("Spiele",170,138,2,RGB565_WHITE);
  drawTextAt("Soundboard",150,188,2,RGB565_WHITE);
  drawTextAt("Info",198,238,2,RGB565_WHITE);
  drawTextAt("Nach oben wischen = schliessen",90,268,1,RGB565_CYAN);
}

void animateDice(uint8_t to){
  const float ox=rotX, oy=rotY, oz=rotZ;
  for(int i=0;i<24;++i){
    float t=(float)i/23.0f;
    rotX=ox+sinf(t*PI*2.0f)*0.35f;
    rotY=oy+t*PI*3.0f;
    rotZ=oz+sinf(t*PI*4.0f)*0.25f;
    lastDice=(i==23)?to:(uint8_t)random(1,7);
    renderCube(true);
    delay(22);
  }
  lastDice=to; rotX=ox; rotY=oy; rotZ=oz; renderCube(false); drawResult();
}

void rollDice(const char *reason){
  uint32_t now=millis(); if(now-lastRollMs<kRollCooldownMs)return;
  lastRollMs=now; uint8_t result=(uint8_t)random(1,7);
  animateDice(result);
  Serial.printf("D6 roll (%s): %u\n",reason,(unsigned)result);
}

void updateMotion(){
  float ax,ay,az; if(!qmi.getAccelerometer(ax,ay,az))return;
  if(!imuReady){lastAccelX=ax;lastAccelY=ay;lastAccelZ=az;imuReady=true;return;}
  float dx=ax-lastAccelX,dy=ay-lastAccelY,dz=az-lastAccelZ;
  float motion=sqrtf(dx*dx+dy*dy+dz*dz);
  lastAccelX=ax;lastAccelY=ay;lastAccelZ=az;
  if(!menuOpen && motion>=kShakeThreshold){rollDice("shake");return;}
  if(menuOpen)return;

  // IMU changes the actual 3D orientation, not just screen position.
  const float targetX=constrain(-ay*0.75f,-0.85f,0.85f);
  const float targetY=constrain(ax*0.75f,-0.85f,0.85f);
  rotX += (targetX-rotX)*0.10f;
  rotY += (targetY-rotY)*0.10f;
  rotZ += (-az*0.05f-rotZ)*0.04f;
  renderCube(true);
}

void processTouch(){
  uint8_t supported=touch.getSupportTouchPoint();
  uint8_t limit=supported<kMaxTouchPoints?supported:kMaxTouchPoints;
  uint8_t points=touch.getPoint(touchX,touchY,limit);
  bool down=points>0;
  if(!down){
    if(touchWasDown){
      int16_t downDist=swipeMaxY-swipeStartY, upDist=swipeStartY-swipeMinY;
      if(trackingSwipe){
        if(!menuOpen && downDist>=kSwipeThreshold){menuOpen=true;drawMenu();}
        else if(menuOpen && upDist>=kSwipeThreshold){menuOpen=false;drawStaticScreen();renderCube(false);drawResult();}
      }
      touchWasDown=false;trackingSwipe=false;
    }
    return;
  }
  int16_t x=touchX[0],y=touchY[0];
  if(!touchWasDown){
    touchWasDown=true;trackingSwipe=true;swipeStartY=y;swipeMinY=y;swipeMaxY=y;
    // Cube hit region is deliberately generous around the projected cube.
    if(!menuOpen && x>95 && x<375 && y>90 && y<380){trackingSwipe=false;rollDice("touch");}
    return;
  }
  if(trackingSwipe){if(y<swipeMinY)swipeMinY=y;if(y>swipeMaxY)swipeMaxY=y;}
}
} // namespace

void setup(){
  Serial.begin(115200); delay(1000);
  Wire.begin(IIC_SDA,IIC_SCL);
  if(!gfx->begin())while(true)delay(1000);
  gfx->fillScreen(BG);gfx->setBrightness(180);

  touch.setPins(TP_RESET,TP_INT);
  if(!touch.begin(Wire,kTouchAddress,IIC_SDA,IIC_SCL)){drawTextAt("TOUCH ERROR",150,220,3,RGB565_RED);while(true)delay(1000);}
  touch.setMaxCoordinates(LCD_WIDTH,LCD_HEIGHT);touch.setMirrorXY(true,true);

  if(!qmi.begin(Wire,QMI8658_L_SLAVE_ADDRESS,IIC_SDA,IIC_SCL)){drawTextAt("IMU ERROR",155,220,3,RGB565_RED);while(true)delay(1000);}
  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G,SensorQMI8658::ACC_ODR_1000Hz,SensorQMI8658::LPF_MODE_0);
  qmi.enableAccelerometer();

  randomSeed((unsigned long)micros());
  drawStaticScreen(); renderCube(false);
  Serial.println("Round Dice true 3D renderer ready.");
}

void loop(){
  uint32_t now=millis();
  if(now-lastTouchPollMs>=kTouchPollMs){lastTouchPollMs=now;processTouch();}
  if(now-lastImuPollMs>=kImuPollMs){lastImuPollMs=now;updateMotion();}
  delay(1);
}
