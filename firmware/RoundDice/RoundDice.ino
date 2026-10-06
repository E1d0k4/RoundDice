#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <vector>
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
TouchDrvCST92xx touch; SensorQMI8658 qmi;
int16_t touchX[kMaxTouchPoints]={},touchY[kMaxTouchPoints]={}; uint32_t lastTouchPollMs=0,lastImuPollMs=0,lastRollMs=0;
bool menuOpen=false,touchWasDown=false,trackingSwipe=false; int16_t swipeStartY=0,swipeMinY=0,swipeMaxY=0;
float lastAccelX=0,lastAccelY=0,lastAccelZ=1; bool imuReady=false; uint16_t *frame=nullptr;
constexpr int16_t W=LCD_WIDTH,H=LCD_HEIGHT; constexpr float CUBE=178.0f,FOCAL=500.0f,CX=233.0f,CY=232.0f;
enum DiceType { D4=4,D6=6,D8=8,D10=10,D12=12,D20=20,D100=100 };
struct Dice3D {
  DiceType type;
  int value;
  bool isHeld;
  float x,y,z;
  float vx,vy,vz;
  float rotX,rotY,rotZ;
  float rotVelX,rotVelY,rotVelZ;
  bool isSettled;
};
std::vector<Dice3D> activeDice;
int totalSum=0;
bool diceRolling=false;
uint32_t lastPhysicsMs=0;
const uint16_t BG=RGB565_BLACK,WHITE=RGB565_WHITE,DICE_BASE=RGB565_WHITE,DICE_BEVEL=RGB565_WHITE,PIP_DARK=RGB565_BLACK,PIP_SHADOW=0x0861;
struct V2{float x,y;}; struct V3{float x,y,z;}; struct Face{uint8_t a,b,c,d,value;float depth;};
inline void px(int x,int y,uint16_t c){if((unsigned)x<W&&(unsigned)y<H)frame[y*W+x]=c;} void clearFrame(){memset(frame,0,W*H*sizeof(uint16_t));}
V3 rotate(V3 p,const Dice3D& d){
  float sx=sinf(d.rotX),cx=cosf(d.rotX),sy=sinf(d.rotY),cy=cosf(d.rotY),sz=sinf(d.rotZ),cz=cosf(d.rotZ);
  float y=p.y*cx-p.z*sx,z=p.y*sx+p.z*cx;p.y=y;p.z=z;
  float x=p.x*cy+p.z*sy;z=-p.x*sy+p.z*cy;p.x=x;p.z=z;
  x=p.x*cz-p.y*sz;y=p.x*sz+p.y*cz;p.x=x;p.y=y;return p;
}
V2 project(V3 p){float s=FOCAL/(FOCAL+p.z);return {CX+p.x*s,CY+p.y*s};}
void fillTriangle(V2 a,V2 b,V2 c,uint16_t col){int minX=max(0,(int)floorf(min(a.x,min(b.x,c.x)))),maxX=min(W-1,(int)ceilf(max(a.x,max(b.x,c.x))));int minY=max(0,(int)floorf(min(a.y,min(b.y,c.y)))),maxY=min(H-1,(int)ceilf(max(a.y,max(b.y,c.y))));float area=(b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);if(fabsf(area)<0.01f)return;for(int y=minY;y<=maxY;y++)for(int x=minX;x<=maxX;x++){float px0=x+0.5f,py0=y+0.5f;float w0=(b.x-a.x)*(py0-a.y)-(b.y-a.y)*(px0-a.x);float w1=(c.x-b.x)*(py0-b.y)-(c.y-b.y)*(px0-b.x);float w2=(a.x-c.x)*(py0-c.y)-(a.y-c.y)*(px0-c.x);if((w0>=0&&w1>=0&&w2>=0)||(w0<=0&&w1<=0&&w2<=0))px(x,y,col);}}
void drawFilledCircle(V2 center,float radius,uint16_t col){int minX=max(0,(int)floorf(center.x-radius)),maxX=min(W-1,(int)ceilf(center.x+radius));int minY=max(0,(int)floorf(center.y-radius)),maxY=min(H-1,(int)ceilf(center.y+radius));float r2=radius*radius;for(int y=minY;y<=maxY;y++)for(int x=minX;x<=maxX;x++){float dx=(x+0.5f)-center.x,dy=(y+0.5f)-center.y;if(dx*dx+dy*dy<=r2)px(x,y,col);}}
V3 facePoint(const V3&a,const V3&b,const V3&c,const V3&d,float u,float v){float s=(u+1)*.5f,t=(v+1)*.5f;return {(1-s)*(1-t)*a.x+s*(1-t)*b.x+s*t*c.x+(1-s)*t*d.x,(1-s)*(1-t)*a.y+s*(1-t)*b.y+s*t*c.y+(1-s)*t*d.y,(1-s)*(1-t)*a.z+s*(1-t)*b.z+s*t*c.z+(1-s)*t*d.z};}
V3 faceNormal(const Face&q,const V3 v[]){V3 a=v[q.a],b=v[q.b],c=v[q.c],ab{b.x-a.x,b.y-a.y,b.z-a.z},ac{c.x-a.x,c.y-a.y,c.z-a.z};return {ab.y*ac.z-ab.z*ac.y,ab.z*ac.x-ab.x*ac.z,ab.x*ac.y-ab.y*ac.x};}
void fillRoundedFace(const Face&q,const V3 v[],uint16_t col){
  // A real bevelled face: the four corners are cut back instead of drawing
  // the old full square.  This gives the die a softer, manufactured edge.
  const float b=.16f;
  V3 A=v[q.a],B=v[q.b],C=v[q.c],D=v[q.d];
  auto lerp3=[](const V3&a,const V3&b,float t)->V3{return {a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,a.z+(b.z-a.z)*t};};
  V3 ab=lerp3(A,B,b), ba=lerp3(B,A,b), bc=lerp3(B,C,b), cb=lerp3(C,B,b);
  V3 cd=lerp3(C,D,b), dc=lerp3(D,C,b), da=lerp3(D,A,b), ad=lerp3(A,D,b);
  V2 pab=project(ab),pbc=project(bc),pcd=project(cd),pda=project(da);
  V2 pba=project(ba),pcb=project(cb),pdc=project(dc),pad=project(ad);
  fillTriangle(pab,pba,pbc,col); fillTriangle(pab,pbc,project(lerp3(ab,bc,.5f)),col);
  fillTriangle(pbc,pcb,pcd,col); fillTriangle(pbc,pcd,project(lerp3(bc,cd,.5f)),col);
  fillTriangle(pcd,pdc,pda,col); fillTriangle(pcd,pda,project(lerp3(cd,da,.5f)),col);
  fillTriangle(pda,pad,pab,col); fillTriangle(pda,pab,project(lerp3(da,ab,.5f)),col);
  fillTriangle(pba,pcb,pdc,col); fillTriangle(pba,pdc,pad,col);
  // The centre is deliberately inset, so the black pips never become bumps.
  fillTriangle(project(lerp3(ab,ad,.5f)),project(lerp3(ab,bc,.5f)),
               project(lerp3(cd,bc,.5f)),col);
  fillTriangle(project(lerp3(ab,ad,.5f)),project(lerp3(cd,bc,.5f)),
               project(lerp3(cd,ad,.5f)),col);
}
void drawSoftBevel(const Face&q,const V3 v[]){(void)q;(void)v;}
void drawRecessedPip(V3 center,V3 u,V3 vv,V3 n,float radius){constexpr int SEG=20,RINGS=5;const uint16_t shade[RINGS]={0x3186,0x20E4,0x18C3,0x1062,RGB565_BLACK};for(int r=0;r<RINGS-1;r++){float t=(float)r/(RINGS-1),rr=radius*(1-.76f*t),depth=7*t,t2=(float)(r+1)/(RINGS-1),rr2=radius*(1-.76f*t2),depth2=7*t2;V3 ringA[SEG],ringB[SEG];for(int i=0;i<SEG;i++){float a=2*PI*i/SEG,ca=cosf(a),sa=sinf(a);ringA[i]={center.x+u.x*(ca*rr)+vv.x*(sa*rr)-n.x*depth,center.y+u.y*(ca*rr)+vv.y*(sa*rr)-n.y*depth,center.z+u.z*(ca*rr)+vv.z*(sa*rr)-n.z*depth};ringB[i]={center.x+u.x*(ca*rr2)+vv.x*(sa*rr2)-n.x*depth2,center.y+u.y*(ca*rr2)+vv.y*(sa*rr2)-n.y*depth2,center.z+u.z*(ca*rr2)+vv.z*(sa*rr2)-n.z*depth2};}for(int i=0;i<SEG;i++){int j=(i+1)%SEG;fillTriangle(project(ringA[i]),project(ringA[j]),project(ringB[j]),shade[r]);fillTriangle(project(ringA[i]),project(ringB[j]),project(ringB[i]),shade[r]);}}V3 bottom{center.x-n.x*7,center.y-n.y*7,center.z-n.z*7};float scale=FOCAL/(FOCAL+bottom.z);drawFilledCircle(project(bottom),radius*.24f*scale,RGB565_BLACK);}
void drawRecessedPips(const Face&q,const V3 v[]){static const float pips[6][7][2]={{{0,0}},{{-.50f,-.50f},{.50f,.50f}},{{-.50f,-.50f},{0,0},{.50f,.50f}},{{-.50f,-.50f},{.50f,-.50f},{-.50f,.50f},{.50f,.50f}},{{-.50f,-.50f},{.50f,-.50f},{0,0},{-.50f,.50f},{.50f,.50f}},{{-.50f,-.56f},{-.50f,0},{-.50f,.56f},{.50f,-.56f},{.50f,0},{.50f,.56f}}};V3 A=v[q.a],B=v[q.b],D=v[q.d],u{B.x-A.x,B.y-A.y,B.z-A.z},vv{D.x-A.x,D.y-A.y,D.z-A.z};float ul=sqrtf(u.x*u.x+u.y*u.y+u.z*u.z),vl=sqrtf(vv.x*vv.x+vv.y*vv.y+vv.z*vv.z);if(ul<1||vl<1)return;u.x/=ul;u.y/=ul;u.z/=ul;vv.x/=vl;vv.y/=vl;vv.z/=vl;V3 n{u.y*vv.z-u.z*vv.y,u.z*vv.x-u.x*vv.z,u.x*vv.y-u.y*vv.x};float nl=sqrtf(n.x*n.x+n.y*n.y+n.z*n.z);if(nl<.01f)return;n.x/=nl;n.y/=nl;n.z/=nl;for(uint8_t i=0;i<q.value;i++){V3 p=facePoint(v[q.a],v[q.b],v[q.c],v[q.d],pips[q.value-1][i][0],pips[q.value-1][i][1]);drawRecessedPip(p,u,vv,n,20);}}
void drawTableShadow(const Dice3D& d){
  float h=constrain(d.z,0.0f,300.0f),sc=constrain(1.0f+h/180.0f,1.0f,2.0f);
  V2 center{CX+d.x,CY+d.y+112};
  for(int i=10;i>=1;i--){float rr=(82.0f*sc)*i/10.0f;uint16_t shade=(uint16_t)(0x0841+(10-i)*0x0180);drawFilledCircle({center.x,center.y+6},rr,shade);}
}
void drawSingleDie(const Dice3D& d){
  const float h=CUBE*.5f;
  // Rounded/chamfered solid die.  It is intentionally not the old sharp
  // eight-vertex box: each visible face is inset and its corners are cut.
  V3 v[8]={
    {-h,-h,-h},{h,-h,-h},{h,h,-h},{-h,h,-h},
    {-h,-h,h},{h,-h,h},{h,h,h},{-h,h,h}
  };
  for(auto&p:v){
    p=rotate(p,d);
    p.x+=d.x;
    p.y+=d.y-d.z;
  }
  Face f[6]={
    {0,1,2,3,1,0},{4,7,6,5,2,0},{0,4,5,1,3,0},
    {3,2,6,7,4,0},{0,3,7,4,5,0},{1,5,6,2,6,0}
  };
  for(auto&q:f)q.depth=(v[q.a].z+v[q.b].z+v[q.c].z+v[q.d].z)*.25f;
  for(int i=0;i<6;i++)for(int j=i+1;j<6;j++)
    if(f[i].depth<f[j].depth){Face t=f[i];f[i]=f[j];f[j]=t;}
  for(const auto&q:f){
    V3 n=faceNormal(q,v);
    if(n.z<=0)continue;
    fillRoundedFace(q,v,DICE_BASE);
    if(q.value>=1&&q.value<=6)drawRecessedPips(q,v);
  }
}
void draw3DTestCube(){
  clearFrame();
  for(const auto&d:activeDice)drawTableShadow(d);
  for(const auto&d:activeDice)drawSingleDie(d);
  gfx->draw16bitRGBBitmap(0,0,frame,W,H);
}
void calculateTotal(){
  totalSum=0;
  for(const auto&d:activeDice)totalSum+=d.value;
}
void rollDice(){
  diceRolling=true;
  totalSum=0;
  for(auto&d:activeDice){
    if(d.isHeld)continue;
    d.x=random(80,380)-230;
    d.y=random(80,200)-140;
    d.z=random(150,301);
    d.vx=random(-15,16);
    d.vy=random(-10,21);
    d.vz=-random(5,13);
    d.rotVelX=random(15,46);
    d.rotVelY=random(15,46);
    d.rotVelZ=random(15,46);
    d.isSettled=false;
    d.value=random(1,(int)d.type+1);
  }
  lastPhysicsMs=millis();
}
void updatePhysics(){
  if(!diceRolling)return;
  uint32_t now=millis();
  if(now-lastPhysicsMs<16)return;
  lastPhysicsMs=now;
  bool allSettled=true;
  const float gravity=-0.8f,bounceDamping=0.55f,friction=0.92f;
  for(size_t i=0;i<activeDice.size();i++){
    auto&d=activeDice[i];
    if(d.isSettled||d.isHeld)continue;
    allSettled=false;
    d.vz+=gravity;d.x+=d.vx;d.y+=d.vy;d.z+=d.vz;
    d.rotX+=d.rotVelX*(PI/180.0f);d.rotY+=d.rotVelY*(PI/180.0f);d.rotZ+=d.rotVelZ*(PI/180.0f);
    if(d.z<=0){
      d.z=0;d.vz=-d.vz*bounceDamping;d.vx*=friction;d.vy*=friction;
      d.rotVelX*=friction;d.rotVelY*=friction;d.rotVelZ*=friction;
      if(fabsf(d.vz)<1.0f&&fabsf(d.vx)<0.5f&&fabsf(d.vy)<0.5f){d.isSettled=true;d.vz=d.vx=d.vy=0;}
    }
    for(size_t j=i+1;j<activeDice.size();j++){
      auto&d2=activeDice[j];
      float dx=d2.x-d.x,dy=d2.y-d.y,dist=sqrtf(dx*dx+dy*dy);
      const float minDist=55.0f;
      if(dist<minDist&&dist>0){
        float overlap=minDist-dist,nx=dx/dist,ny=dy/dist;
        d.x-=nx*overlap*.5f;d.y-=ny*overlap*.5f;
        d2.x+=nx*overlap*.5f;d2.y+=ny*overlap*.5f;
        float tvx=d.vx,tvy=d.vy;d.vx=d2.vx;d.vy=d2.vy;d2.vx=tvx;d2.vy=tvy;
      }
    }
  }
  if(allSettled){diceRolling=false;calculateTotal();}
  draw3DTestCube();
}
void triggerRoll(const char* reason){
  uint32_t now=millis();
  if(now-lastRollMs<kRollCooldownMs||diceRolling)return;
  lastRollMs=now;
  Serial.printf("D6 physical roll (%s): %u dice\\n",reason,(unsigned)activeDice.size());
  rollDice();
}
void drawTextAt(const char*text,int16_t x,int16_t y,uint8_t size,uint16_t color){gfx->setTextSize(size);gfx->setTextColor(color);gfx->setCursor(x,y);gfx->print(text);}
void drawStaticScreen(){gfx->fillScreen(BG);drawTextAt("ROUND DICE",145,25,3,WHITE);drawTextAt("3D HARDWARE TEST",142,62,2,0x29A7);drawTextAt("TOUCH  •  SHAKE  •  TILT",130,447,1,0x29A7);char r[16];snprintf(r,sizeof(r),"D6  %u",(unsigned)(activeDice.empty()?6:activeDice[0].value));drawTextAt(r,210,425,2,RGB565_YELLOW);}
void drawResult(){gfx->fillRect(195,418,80,28,BG);char r[16];snprintf(r,sizeof(r),"D6  %u",(unsigned)(activeDice.empty()?6:activeDice[0].value));drawTextAt(r,210,425,2,RGB565_YELLOW);}
void drawMenu(){gfx->fillRoundRect(18,8,430,285,28,RGB565_DARKGREY);gfx->drawRoundRect(18,8,430,285,28,0x29A7);drawTextAt("MENU",190,30,3,WHITE);drawTextAt("Einstellungen",105,88,2,WHITE);drawTextAt("Spiele",170,138,2,WHITE);drawTextAt("Soundboard",150,188,2,WHITE);drawTextAt("Info",198,238,2,WHITE);drawTextAt("Nach oben wischen = schliessen",90,268,1,0x29A7);}
void updateMotion(){
  float ax,ay,az;
  if(!qmi.getAccelerometer(ax,ay,az))return;
  if(!imuReady){lastAccelX=ax;lastAccelY=ay;lastAccelZ=az;imuReady=true;return;}
  float dx=ax-lastAccelX,dy=ay-lastAccelY,dz=az-lastAccelZ;
  float motion=sqrtf(dx*dx+dy*dy+dz*dz);
  lastAccelX=ax;lastAccelY=ay;lastAccelZ=az;
  if(!menuOpen&&motion>=kShakeThreshold){triggerRoll("shake");return;}
  if(menuOpen||diceRolling||activeDice.empty())return;
  float tx=constrain(-ay*.95f,-1.05f,1.05f),ty=constrain(ax*.95f,-1.05f,1.05f);
  activeDice[0].rotX+=(tx-activeDice[0].rotX)*.08f;
  activeDice[0].rotY+=(ty-activeDice[0].rotY)*.08f;
  activeDice[0].rotZ+=(-az*.12f-activeDice[0].rotZ)*.05f;
  draw3DTestCube();
}
void processTouch(){uint8_t supported=touch.getSupportTouchPoint(),limit=supported<kMaxTouchPoints?supported:kMaxTouchPoints;uint8_t points=touch.getPoint(touchX,touchY,limit);bool down=points>0;if(!down){if(touchWasDown){int16_t downDist=swipeMaxY-swipeStartY,upDist=swipeStartY-swipeMinY;if(trackingSwipe){if(!menuOpen&&downDist>=kSwipeThreshold){menuOpen=true;drawMenu();}else if(menuOpen&&upDist>=kSwipeThreshold){menuOpen=false;drawStaticScreen();draw3DTestCube();drawResult();}}touchWasDown=false;trackingSwipe=false;}return;}int16_t x=touchX[0],y=touchY[0];if(!touchWasDown){touchWasDown=true;trackingSwipe=true;swipeStartY=y;swipeMinY=y;swipeMaxY=y;if(!menuOpen&&x>85&&x<390&&y>85&&y<390){trackingSwipe=false;triggerRoll("touch");}return;}if(trackingSwipe){swipeMinY=min(swipeMinY,y);swipeMaxY=max(swipeMaxY,y);}}
}
void setup(){Serial.begin(115200);delay(1000);Serial.println();Serial.println("Round Dice boot");Wire.begin(IIC_SDA,IIC_SCL);if(!gfx->begin()){Serial.println("DISPLAY ERROR");while(true)delay(1000);}gfx->fillScreen(BG);gfx->setBrightness(180);drawTextAt("ROUND DICE",145,25,3,WHITE);drawTextAt("DISPLAY OK",175,205,2,0x29A7);drawTextAt("STARTING 3D...",150,240,2,WHITE);Serial.printf("PSRAM found: %s, size: %u bytes\n",psramFound()?"YES":"NO",(unsigned)ESP.getPsramSize());const size_t frameBytes=(size_t)W*(size_t)H*sizeof(uint16_t);frame=(uint16_t*)heap_caps_malloc(frameBytes,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);if(!frame){Serial.println("PSRAM FRAMEBUFFER ALLOCATION FAILED");gfx->fillScreen(BG);drawTextAt("PSRAM ERROR",145,190,3,RGB565_RED);drawTextAt("NO FRAMEBUFFER",125,235,2,WHITE);while(true)delay(1000);}Serial.printf("Framebuffer allocated: %u bytes\n",(unsigned)frameBytes);touch.setPins(TP_RESET,TP_INT);if(!touch.begin(Wire,kTouchAddress,IIC_SDA,IIC_SCL)){Serial.println("TOUCH ERROR");gfx->fillScreen(BG);drawTextAt("TOUCH ERROR",150,220,3,RGB565_RED);while(true)delay(1000);}touch.setMaxCoordinates(LCD_WIDTH,LCD_HEIGHT);touch.setMirrorXY(true,true);if(!qmi.begin(Wire,QMI8658_L_SLAVE_ADDRESS,IIC_SDA,IIC_SCL)){Serial.println("IMU ERROR");gfx->fillScreen(BG);drawTextAt("IMU ERROR",155,220,3,RGB565_RED);while(true)delay(1000);}qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G,SensorQMI8658::ACC_ODR_1000Hz,SensorQMI8658::LPF_MODE_0);qmi.enableAccelerometer();randomSeed((unsigned long)micros());
activeDice.clear();
activeDice.push_back({D6,6,false,-70,-40,0,0,0,0,0,0,0,0,0,0,true});
activeDice.push_back({D6,4,false,70,-40,0,0,0,0,0,0,0,0,0,0,true});
activeDice.push_back({D6,3,false,0,80,0,0,0,0,0,0,0,0,0,0,true});
calculateTotal();drawStaticScreen();draw3DTestCube();Serial.println("Round Dice true 3D renderer ready.");}
void loop(){uint32_t now=millis();if(now-lastTouchPollMs>=kTouchPollMs){lastTouchPollMs=now;processTouch();}if(now-lastImuPollMs>=kImuPollMs){lastImuPollMs=now;updateMotion();}if(diceRolling)updatePhysics();delay(1);}