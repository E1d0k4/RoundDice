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

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_CO5300 *gfx = new Arduino_CO5300(
    bus, LCD_RESET, 0, LCD_WIDTH, LCD_HEIGHT, 6, 0, 0, 0);
TouchDrvCST92xx touch;
SensorQMI8658 qmi;

int16_t touchX[kMaxTouchPoints] = {}, touchY[kMaxTouchPoints] = {};
uint32_t lastTouchPollMs = 0, lastImuPollMs = 0, lastRollMs = 0;
uint8_t lastDice = 6;
bool menuOpen = false, touchWasDown = false, trackingSwipe = false;
int16_t swipeStartY = 0, swipeMinY = 0, swipeMaxY = 0;
float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 1;
bool imuReady = false;

// Full-screen PSRAM framebuffer: render the entire scene first, then transfer it once.
// This deliberately avoids erasing/redrawing the cube directly on the AMOLED.
uint16_t *frame = nullptr;
constexpr int16_t W = LCD_WIDTH;
constexpr int16_t H = LCD_HEIGHT;

constexpr float CUBE = 170.0f;
constexpr float FOCAL = 470.0f;
constexpr float CX = 233.0f;
constexpr float CY = 238.0f;
float rotX = -0.42f, rotY = 0.58f, rotZ = 0.08f;

const uint16_t BG = RGB565_BLACK;
const uint16_t EDGE = RGB565_CYAN;
const uint16_t WHITE = RGB565_WHITE;
const uint16_t FACE[6] = {0x29A7,0x4A69,0x19A5,0x39C8,0x21C6,0x1164};

struct V2 { float x,y; };
struct V3 { float x,y,z; };
struct Face { uint8_t a,b,c,d,value; float depth; };

inline void px(int x,int y,uint16_t c){
  if((unsigned)x < W && (unsigned)y < H) frame[y*W+x]=c;
}
void clearFrame(){ memset(frame,0,W*H*sizeof(uint16_t)); }

V3 rotate(V3 p){
  float sx=sinf(rotX),cx=cosf(rotX),sy=sinf(rotY),cy=cosf(rotY),sz=sinf(rotZ),cz=cosf(rotZ);
  float y=p.y*cx-p.z*sx, z=p.y*sx+p.z*cx; p.y=y;p.z=z;
  float x=p.x*cy+p.z*sy; z=-p.x*sy+p.z*cy; p.x=x;p.z=z;
  x=p.x*cz-p.y*sz; y=p.x*sz+p.y*cz; p.x=x;p.y=y;
  return p;
}
V2 project(V3 p){
  float s=FOCAL/(FOCAL+p.z);
  return {CX+p.x*s,CY+p.y*s};
}
void line(V2 a,V2 b,uint16_t c){
  int x0=lroundf(a.x),y0=lroundf(a.y),x1=lroundf(b.x),y1=lroundf(b.y);
  int dx=abs(x1-x0),sx=x0<x1?1:-1,dy=-abs(y1-y0),sy=y0<y1?1:-1,err=dx+dy;
  while(true){px(x0,y0,c);if(x0==x1&&y0==y1)break;int e=2*err;if(e>=dy){err+=dy;x0+=sx;}if(e<=dx){err+=dx;y0+=sy;}}
}
void fillTriangle(V2 a,V2 b,V2 c,uint16_t col){
  int minX=max(0,(int)floorf(min(a.x,min(b.x,c.x)))),maxX=min(W-1,(int)ceilf(max(a.x,max(b.x,c.x))));
  int minY=max(0,(int)floorf(min(a.y,min(b.y,c.y)))),maxY=min(H-1,(int)ceilf(max(a.y,max(b.y,c.y))));
  float area=(b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x); if(fabsf(area)<0.01f)return;
  for(int y=minY;y<=maxY;y++)for(int x=minX;x<=maxX;x++){
    float px0=x+0.5f,py0=y+0.5f;
    float w0=(b.x-a.x)*(py0-a.y)-(b.y-a.y)*(px0-a.x);
    float w1=(c.x-b.x)*(py0-b.y)-(c.y-b.y)*(px0-b.x);
    float w2=(a.x-c.x)*(py0-c.y)-(a.y-c.y)*(px0-c.x);
    if((w0>=0&&w1>=0&&w2>=0)||(w0<=0&&w1<=0&&w2<=0))px(x,y,col);
  }
}
void draw3DTestCube(){
  clearFrame();
  float h=CUBE/2.0f;
  V3 v[8]={{-h,-h,-h},{h,-h,-h},{h,h,-h},{-h,h,-h},{-h,-h,h},{h,-h,h},{h,h,h},{-h,h,h}};
  for(auto &p:v)p=rotate(p);
  V2 s[8];for(int i=0;i<8;i++)s[i]=project(v[i]);

  Face f[6]={{0,1,2,3,1,0},{4,7,6,5,2,0},{0,4,5,1,3,0},{3,2,6,7,4,0},{0,3,7,4,5,0},{1,5,6,2,6,0}};
  for(auto &q:f)q.depth=(v[q.a].z+v[q.b].z+v[q.c].z+v[q.d].z)*0.25f;
  for(int i=0;i<6;i++)for(int j=i+1;j<6;j++)if(f[i].depth<f[j].depth){Face t=f[i];f[i]=f[j];f[j]=t;}

  for(const auto &q:f){
    V3 A=v[q.a],B=v[q.b],C=v[q.c];
    float nz=(B.x-A.x)*(C.y-A.y)-(B.y-A.y)*(C.x-A.x);
    if(nz<=0)continue;
    fillTriangle(s[q.a],s[q.b],s[q.c],FACE[q.value-1]);
    fillTriangle(s[q.a],s[q.c],s[q.d],FACE[q.value-1]);
  }
  // Strong, unmistakable 3D wireframe: all 12 cube edges.
  const uint8_t e[12][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
  for(auto &a:e)line(s[a[0]],s[a[1]],EDGE);
  // Center crosshair makes rotation/depth visually obvious.
  line({CX-7,CY},{CX+7,CY},WHITE); line({CX,CY-7},{CX,CY+7},WHITE);

  gfx->draw16bitRGBBitmap(0,0,frame,W,H);
}

void drawTextAt(const char *text,int16_t x,int16_t y,uint8_t size,uint16_t color){
  gfx->setTextSize(size);gfx->setTextColor(color);gfx->setCursor(x,y);gfx->print(text);
}
void drawStaticScreen(){
  gfx->fillScreen(BG);
  drawTextAt("ROUND DICE",145,25,3,WHITE);
  drawTextAt("3D HARDWARE TEST",142,62,2,EDGE);
  drawTextAt("TOUCH  •  SHAKE  •  TILT",130,447,1,EDGE);
  char r[16];snprintf(r,sizeof(r),"D6  %u",(unsigned)lastDice);drawTextAt(r,210,425,2,RGB565_YELLOW);
}
void drawResult(){gfx->fillRect(195,418,80,28,BG);char r[16];snprintf(r,sizeof(r),"D6  %u",(unsigned)lastDice);drawTextAt(r,210,425,2,RGB565_YELLOW);}
void drawMenu(){
  gfx->fillRoundRect(18,8,430,285,28,RGB565_DARKGREY);gfx->drawRoundRect(18,8,430,285,28,EDGE);
  drawTextAt("MENU",190,30,3,WHITE);drawTextAt("Einstellungen",105,88,2,WHITE);
  drawTextAt("Spiele",170,138,2,WHITE);drawTextAt("Soundboard",150,188,2,WHITE);drawTextAt("Info",198,238,2,WHITE);
  drawTextAt("Nach oben wischen = schliessen",90,268,1,EDGE);
}
void animateDice(uint8_t to){
  float ox=rotX,oy=rotY,oz=rotZ;
  for(int i=0;i<30;i++){
    float t=(float)i/29.0f;
    rotX=ox+sinf(t*PI*2.0f)*0.55f;
    rotY=oy+t*PI*4.0f;
    rotZ=oz+sinf(t*PI*3.0f)*0.45f;
    lastDice=(i==29)?to:(uint8_t)random(1,7);
    draw3DTestCube();delay(20);
  }
  lastDice=to;rotX=ox;rotY=oy;rotZ=oz;draw3DTestCube();drawResult();
}
void rollDice(const char *reason){
  uint32_t now=millis();if(now-lastRollMs<kRollCooldownMs)return;lastRollMs=now;
  uint8_t result=(uint8_t)random(1,7);animateDice(result);Serial.printf("D6 roll (%s): %u\n",reason,(unsigned)result);
}
void updateMotion(){
  float ax,ay,az;if(!qmi.getAccelerometer(ax,ay,az))return;
  if(!imuReady){lastAccelX=ax;lastAccelY=ay;lastAccelZ=az;imuReady=true;return;}
  float dx=ax-lastAccelX,dy=ay-lastAccelY,dz=az-lastAccelZ;
  float motion=sqrtf(dx*dx+dy*dy+dz*dz);lastAccelX=ax;lastAccelY=ay;lastAccelZ=az;
  if(!menuOpen&&motion>=kShakeThreshold){rollDice("shake");return;}
  if(menuOpen)return;
  float tx=constrain(-ay*0.95f,-1.05f,1.05f),ty=constrain(ax*0.95f,-1.05f,1.05f);
  rotX+=(tx-rotX)*0.08f;rotY+=(ty-rotY)*0.08f;
  rotZ+=(-az*0.12f-rotZ)*0.05f;
  draw3DTestCube();
}
void processTouch(){
  uint8_t supported=touch.getSupportTouchPoint(),limit=supported<kMaxTouchPoints?supported:kMaxTouchPoints;
  uint8_t points=touch.getPoint(touchX,touchY,limit);bool down=points>0;
  if(!down){
    if(touchWasDown){
      int16_t downDist=swipeMaxY-swipeStartY,upDist=swipeStartY-swipeMinY;
      if(trackingSwipe){
        if(!menuOpen&&downDist>=kSwipeThreshold){menuOpen=true;drawMenu();}
        else if(menuOpen&&upDist>=kSwipeThreshold){menuOpen=false;drawStaticScreen();draw3DTestCube();drawResult();}
      }
      touchWasDown=false;trackingSwipe=false;
    }return;
  }
  int16_t x=touchX[0],y=touchY[0];
  if(!touchWasDown){
    touchWasDown=true;trackingSwipe=true;swipeStartY=y;swipeMinY=y;swipeMaxY=y;
    if(!menuOpen&&x>85&&x<390&&y>85&&y<390){trackingSwipe=false;rollDice("touch");}
    return;
  }
  if(trackingSwipe){swipeMinY=min(swipeMinY,y);swipeMaxY=max(swipeMaxY,y);}
}
} // namespace

void setup(){
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("Round Dice boot");

  // Initialize the display BEFORE allocating the large framebuffer.
  // Allocation failures are therefore visible instead of looking like a dead device.
  Wire.begin(IIC_SDA,IIC_SCL);
  if(!gfx->begin()){
    Serial.println("DISPLAY ERROR");
    while(true) delay(1000);
  }
  gfx->fillScreen(BG);
  gfx->setBrightness(180);
  drawTextAt("ROUND DICE",145,25,3,WHITE);
  drawTextAt("DISPLAY OK",175,205,2,EDGE);
  drawTextAt("STARTING 3D...",150,240,2,WHITE);

  Serial.printf("PSRAM found: %s, size: %u bytes\n",
                psramFound() ? "YES" : "NO",
                (unsigned)ESP.getPsramSize());

  const size_t frameBytes=(size_t)W*(size_t)H*sizeof(uint16_t);
  frame=(uint16_t*)heap_caps_malloc(frameBytes,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
  if(!frame){
    Serial.println("PSRAM FRAMEBUFFER ALLOCATION FAILED");
    gfx->fillScreen(BG);
    drawTextAt("PSRAM ERROR",145,190,3,RGB565_RED);
    drawTextAt("NO FRAMEBUFFER",125,235,2,WHITE);
    while(true) delay(1000);
  }
  Serial.printf("Framebuffer allocated: %u bytes\n",(unsigned)frameBytes);

  touch.setPins(TP_RESET,TP_INT);
  if(!touch.begin(Wire,kTouchAddress,IIC_SDA,IIC_SCL)){
    Serial.println("TOUCH ERROR");
    gfx->fillScreen(BG);
    drawTextAt("TOUCH ERROR",150,220,3,RGB565_RED);
    while(true) delay(1000);
  }
  touch.setMaxCoordinates(LCD_WIDTH,LCD_HEIGHT);
  touch.setMirrorXY(true,true);

  if(!qmi.begin(Wire,QMI8658_L_SLAVE_ADDRESS,IIC_SDA,IIC_SCL)){
    Serial.println("IMU ERROR");
    gfx->fillScreen(BG);
    drawTextAt("IMU ERROR",155,220,3,RGB565_RED);
    while(true) delay(1000);
  }
  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G,SensorQMI8658::ACC_ODR_1000Hz,SensorQMI8658::LPF_MODE_0);
  qmi.enableAccelerometer();

  randomSeed((unsigned long)micros());
  drawStaticScreen();
  draw3DTestCube();
  Serial.println("Round Dice true 3D renderer ready.");
}

void loop(){
  uint32_t now=millis();
  if(now-lastTouchPollMs>=kTouchPollMs){lastTouchPollMs=now;processTouch();}
  if(now-lastImuPollMs>=kImuPollMs){lastImuPollMs=now;updateMotion();}
  delay(1);
}
