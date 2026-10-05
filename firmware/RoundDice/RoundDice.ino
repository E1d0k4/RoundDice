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

int16_t touchX[kMaxTouchPoints] = {};
int16_t touchY[kMaxTouchPoints] = {};
uint8_t lastDice = 6;

uint32_t lastTouchPollMs = 0;
uint32_t lastImuPollMs = 0;
uint32_t lastRollMs = 0;

bool menuOpen = false;
bool touchWasDown = false;
bool trackingSwipe = false;
int16_t swipeStartY = 0;
int16_t swipeMinY = 0;
int16_t swipeMaxY = 0;

float lastAccelX = 0.0f;
float lastAccelY = 0.0f;
float lastAccelZ = 1.0f;
bool imuReady = false;

constexpr int16_t kCubeW = 184;
constexpr int16_t kCubeH = 184;
constexpr int16_t kCubeDepth = 42;
constexpr int16_t kCubeHomeX = 141;
constexpr int16_t kCubeHomeY = 116;

int16_t cubeX = kCubeHomeX;
int16_t cubeY = kCubeHomeY;

const uint16_t kBackground = RGB565_BLACK;
const uint16_t kFront = 0x18E3;
const uint16_t kFrontLight = 0x2D26;
const uint16_t kTop = 0x3D55;
const uint16_t kRight = 0x10A3;
const uint16_t kEdge = RGB565_CYAN;
const uint16_t kEdgeSoft = 0x5E9F;
const uint16_t kHighlight = 0xAFFF;
const uint16_t kShadow = 0x0841;
const uint16_t kPip = RGB565_WHITE;

void drawTextAt(const char *text, int16_t x, int16_t y, uint8_t size, uint16_t color) {
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  gfx->setCursor(x, y);
  gfx->print(text);
}

void drawStaticScreen() {
  gfx->fillScreen(kBackground);
  drawTextAt("ROUND DICE", 145, 25, 3, RGB565_WHITE);
  drawTextAt("3D MOTION", 162, 62, 2, RGB565_CYAN);
  drawTextAt("TOUCH  •  SHAKE  •  TILT", 130, 447, 1, RGB565_CYAN);

  char result[20];
  snprintf(result, sizeof(result), "D6  %u", static_cast<unsigned>(lastDice));
  drawTextAt(result, 210, 425, 2, RGB565_YELLOW);
}

void clearCubeArea() {
  // The cube and its motion envelope stay clear of the title and footer.
  gfx->fillRect(95, 88, 280, 285, kBackground);
}

void drawResult(uint8_t value) {
  gfx->fillRect(195, 418, 80, 28, kBackground);
  char result[20];
  snprintf(result, sizeof(result), "D6  %u", static_cast<unsigned>(value));
  drawTextAt(result, 210, 425, 2, RGB565_YELLOW);
}

void drawPip(int16_t x, int16_t y, int16_t radius = 11) {
  gfx->fillCircle(x, y, radius + 2, 0x0000);
  gfx->fillCircle(x, y, radius, kPip);
}

void drawFrontPips(uint8_t value, int16_t x, int16_t y, int16_t size) {
  const int16_t left = x + size / 4;
  const int16_t mid = x + size / 2;
  const int16_t right = x + (size * 3) / 4;
  const int16_t top = y + size / 4;
  const int16_t center = y + size / 2;
  const int16_t bottom = y + (size * 3) / 4;

  switch (value) {
    case 1:
      drawPip(mid, center);
      break;
    case 2:
      drawPip(left, top);
      drawPip(right, bottom);
      break;
    case 3:
      drawPip(left, top);
      drawPip(mid, center);
      drawPip(right, bottom);
      break;
    case 4:
      drawPip(left, top);
      drawPip(right, top);
      drawPip(left, bottom);
      drawPip(right, bottom);
      break;
    case 5:
      drawPip(left, top);
      drawPip(right, top);
      drawPip(mid, center);
      drawPip(left, bottom);
      drawPip(right, bottom);
      break;
    case 6:
      drawPip(left, top);
      drawPip(right, top);
      drawPip(left, center);
      drawPip(right, center);
      drawPip(left, bottom);
      drawPip(right, bottom);
      break;
  }
}

void drawCube(uint8_t value, float tiltX = 0.0f, float tiltY = 0.0f) {
  const int16_t x = cubeX + static_cast<int16_t>(tiltX);
  const int16_t y = cubeY + static_cast<int16_t>(tiltY);
  const int16_t f = kCubeW;
  const int16_t d = kCubeDepth;

  // Three visible faces: top, right and front.
  int16_t fx0 = x;
  int16_t fy0 = y + d;
  int16_t fx1 = x + f;
  int16_t fy1 = y + d;
  int16_t fx2 = x + f;
  int16_t fy2 = y + d + f;
  int16_t fx3 = x;
  int16_t fy3 = y + d + f;

  int16_t tx0 = x;
  int16_t ty0 = y + d;
  int16_t tx1 = x + d;
  int16_t ty1 = y;
  int16_t tx2 = x + f + d;
  int16_t ty2 = y;
  int16_t tx3 = x + f;
  int16_t ty3 = y + d;

  int16_t rx0 = x + f;
  int16_t ry0 = y + d;
  int16_t rx1 = x + f + d;
  int16_t ry1 = y;
  int16_t rx2 = x + f + d;
  int16_t ry2 = y + f;
  int16_t rx3 = x + f;
  int16_t ry3 = y + d + f;

  // Soft shadow, then faces.
  gfx->fillRoundRect(x + 10, y + d + 10, f + d + 8, f + 8, 18, kShadow);

  gfx->fillTriangle(tx0, ty0, tx1, ty1, tx2, ty2, kTop);
  gfx->fillTriangle(tx0, ty0, tx2, ty2, tx3, ty3, kTop);

  gfx->fillTriangle(rx0, ry0, rx1, ry1, rx2, ry2, kRight);
  gfx->fillTriangle(rx0, ry0, rx2, ry2, rx3, ry3, kRight);

  gfx->fillRect(fx0, fy0, f, f, kFront);
  gfx->fillRect(fx0 + 7, fy0 + 7, f - 14, f - 14, kFrontLight);

  // Strong outer edges make the 3D geometry readable on AMOLED.
  gfx->drawLine(tx0, ty0, tx1, ty1, kEdge);
  gfx->drawLine(tx1, ty1, tx2, ty2, kEdgeSoft);
  gfx->drawLine(tx2, ty2, tx3, ty3, kEdge);
  gfx->drawLine(rx1, ry1, rx2, ry2, kEdgeSoft);
  gfx->drawLine(rx2, ry2, rx3, ry3, kEdge);
  gfx->drawRect(fx0, fy0, f, f, kEdge);
  gfx->drawRect(fx0 + 4, fy0 + 4, f - 8, f - 8, kHighlight);

  // Top-face highlight.
  gfx->drawLine(tx0 + 8, ty0 - 2, tx1 + 8, ty1 + 2, kHighlight);
  gfx->drawLine(tx1 + 8, ty1 + 2, tx2 - 8, ty2 + 2, kHighlight);

  drawFrontPips(value, fx0, fy0, f);
}

void drawScene() {
  clearCubeArea();
  drawCube(lastDice);
}

void drawMenu() {
  gfx->fillRoundRect(18, 8, 430, 285, 28, RGB565_DARKGREY);
  gfx->drawRoundRect(18, 8, 430, 285, 28, RGB565_CYAN);
  drawTextAt("MENU", 190, 30, 3, RGB565_WHITE);
  drawTextAt("Einstellungen", 105, 88, 2, RGB565_WHITE);
  drawTextAt("Spiele", 170, 138, 2, RGB565_WHITE);
  drawTextAt("Soundboard", 150, 188, 2, RGB565_WHITE);
  drawTextAt("Info", 198, 238, 2, RGB565_WHITE);
  drawTextAt("Nach oben wischen = schliessen", 90, 268, 1, RGB565_CYAN);
}

void animateDice(uint8_t to) {
  const int16_t homeX = cubeX;
  const int16_t homeY = cubeY;

  for (uint8_t i = 0; i < 18; ++i) {
    const uint8_t next = (i == 17) ? to : static_cast<uint8_t>(random(1, 7));
    const float phase = static_cast<float>(i) * 0.55f;
    const float travel = 18.0f - static_cast<float>(i) * 0.8f;

    cubeX = homeX + static_cast<int16_t>(sinf(phase) * travel);
    cubeY = homeY + static_cast<int16_t>(cosf(phase * 1.15f) * travel * 0.55f);

    clearCubeArea();
    drawCube(next, sinf(phase) * 7.0f, cosf(phase) * 5.0f);
    delay(28);
  }

  cubeX = homeX;
  cubeY = homeY;
  drawScene();
}

void rollDice(const char *reason) {
  const uint32_t now = millis();
  if (now - lastRollMs < kRollCooldownMs) return;

  lastRollMs = now;
  const uint8_t result = static_cast<uint8_t>(random(1, 7));
  animateDice(result);
  lastDice = result;
  drawScene();
  drawResult(lastDice);
  Serial.printf("D6 roll (%s): %u\n", reason, static_cast<unsigned>(lastDice));
}

void updateMotion() {
  float ax, ay, az;
  if (!qmi.getAccelerometer(ax, ay, az)) return;

  if (!imuReady) {
    lastAccelX = ax;
    lastAccelY = ay;
    lastAccelZ = az;
    imuReady = true;
    return;
  }

  const float deltaX = ax - lastAccelX;
  const float deltaY = ay - lastAccelY;
  const float deltaZ = az - lastAccelZ;
  const float motion = sqrtf(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);

  lastAccelX = ax;
  lastAccelY = ay;
  lastAccelZ = az;

  if (!menuOpen && motion >= kShakeThreshold) {
    rollDice("shake");
    return;
  }

  if (menuOpen) return;

  // Tilt controls the cube's position, with strong damping.
  const int16_t targetX = kCubeHomeX -
      static_cast<int16_t>(constrain(ay * 15.0f, -22.0f, 22.0f));
  const int16_t targetY = kCubeHomeY +
      static_cast<int16_t>(constrain(ax * 15.0f, -18.0f, 18.0f));

  if (abs(targetX - cubeX) > 1 || abs(targetY - cubeY) > 1) {
    cubeX += (targetX - cubeX) / 4;
    cubeY += (targetY - cubeY) / 4;
    drawScene();
  }
}

void processTouch() {
  const uint8_t supportedPoints = touch.getSupportTouchPoint();
  const uint8_t pointLimit =
      supportedPoints < kMaxTouchPoints ? supportedPoints : kMaxTouchPoints;
  const uint8_t touchedPoints = touch.getPoint(touchX, touchY, pointLimit);
  const bool isDown = touchedPoints > 0;

  if (!isDown) {
    if (touchWasDown) {
      const int16_t totalDown = swipeMaxY - swipeStartY;
      const int16_t totalUp = swipeStartY - swipeMinY;

      if (trackingSwipe) {
        if (!menuOpen && totalDown >= kSwipeThreshold) {
          menuOpen = true;
          drawMenu();
          Serial.println("Swipe down: menu opened");
        } else if (menuOpen && totalUp >= kSwipeThreshold) {
          menuOpen = false;
          cubeX = kCubeHomeX;
          cubeY = kCubeHomeY;
          drawStaticScreen();
          drawScene();
          Serial.println("Swipe up: menu closed");
        }
      }

      touchWasDown = false;
      trackingSwipe = false;
    }
    return;
  }

  const int16_t x = touchX[0];
  const int16_t y = touchY[0];

  if (!touchWasDown) {
    touchWasDown = true;
    trackingSwipe = true;
    swipeStartY = y;
    swipeMinY = y;
    swipeMaxY = y;

    Serial.printf("Touch start: X=%d Y=%d\n", x, y);

    // A touch on the visible front face rolls the die.
    const int16_t frontY = cubeY + kCubeDepth;
    if (!menuOpen &&
        x >= cubeX && x <= cubeX + kCubeW + kCubeDepth &&
        y >= frontY && y <= frontY + kCubeH) {
      trackingSwipe = false;
      rollDice("touch");
    }
    return;
  }

  if (!trackingSwipe) return;

  if (y < swipeMinY) swipeMinY = y;
  if (y > swipeMaxY) swipeMaxY = y;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== Round Dice 3D motion test ===");
  Serial.println("Waveshare ESP32-S3-Touch-AMOLED-1.75");

  Wire.begin(IIC_SDA, IIC_SCL);

  Serial.println("Initializing CO5300 display...");
  if (!gfx->begin()) {
    Serial.println("ERROR: gfx->begin() failed!");
    while (true) delay(1000);
  }

  gfx->fillScreen(kBackground);
  gfx->setBrightness(180);

  Serial.println("Initializing CST9217 touch...");
  touch.setPins(TP_RESET, TP_INT);
  if (!touch.begin(Wire, kTouchAddress, IIC_SDA, IIC_SCL)) {
    Serial.println("ERROR: CST9217 initialization failed!");
    drawTextAt("TOUCH ERROR", 150, 220, 3, RGB565_RED);
    while (true) delay(1000);
  }

  touch.setMaxCoordinates(LCD_WIDTH, LCD_HEIGHT);
  touch.setMirrorXY(true, true);

  Serial.println("Initializing QMI8658 motion sensor...");
  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    Serial.println("ERROR: QMI8658 initialization failed!");
    drawTextAt("IMU ERROR", 155, 220, 3, RGB565_RED);
    while (true) delay(1000);
  }

  qmi.configAccelerometer(
      SensorQMI8658::ACC_RANGE_4G,
      SensorQMI8658::ACC_ODR_1000Hz,
      SensorQMI8658::LPF_MODE_0);
  qmi.enableAccelerometer();

  randomSeed(static_cast<unsigned long>(micros()));
  drawStaticScreen();
  drawScene();

  Serial.printf("Touch controller: %s\n", touch.getModelName());
  Serial.println("QMI8658 motion control ready.");
  Serial.println("Touch the die or shake the device to roll.");
}
void loop() {
  const uint32_t now = millis();

  if (now - lastTouchPollMs >= kTouchPollMs) {
    lastTouchPollMs = now;
    processTouch();
  }

  if (now - lastImuPollMs >= kImuPollMs) {
    lastImuPollMs = now;
    updateMotion();
  }

  delay(1);
}
