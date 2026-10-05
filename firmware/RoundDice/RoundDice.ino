#include <Arduino.h>
#include <Wire.h>
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
constexpr float kShakeThreshold = 1.65f;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_CO5300 *gfx = new Arduino_CO5300(
    bus, LCD_RESET, 0, LCD_WIDTH, LCD_HEIGHT, 6, 0, 0, 0);

TouchDrvCST92xx touch;
SensorQMI8658 qmi;

int16_t touchX[kMaxTouchPoints] = {};
int16_t touchY[kMaxTouchPoints] = {};
uint8_t lastDice = 6;

uint32_t lastTouchActionMs = 0;
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

constexpr int16_t kDieSize = 220;
constexpr int16_t kDieHomeX = 123;
constexpr int16_t kDieHomeY = 112;
int16_t dieX = kDieHomeX;
int16_t dieY = kDieHomeY;

const uint16_t kDieFace = 0x18E3;
const uint16_t kDieFaceLight = 0x2D26;
const uint16_t kDieEdge = RGB565_CYAN;
const uint16_t kDieShadow = 0x0841;
const uint16_t kDieHighlight = 0x7DFF;
const uint16_t kPipColor = RGB565_WHITE;

void drawTextAt(const char *text, int16_t x, int16_t y, uint8_t size, uint16_t color) {
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  gfx->setCursor(x, y);
  gfx->print(text);
}

void drawStaticScreen() {
  gfx->fillScreen(RGB565_BLACK);
  drawTextAt("ROUND DICE", 145, 25, 3, RGB565_WHITE);
  drawTextAt("MOTION DICE", 151, 62, 2, RGB565_CYAN);

  drawTextAt("ANTIPPEN  •  SCHÜTTELN", 112, 447, 1, RGB565_CYAN);
  char result[20];
  snprintf(result, sizeof(result), "D6  %u", static_cast<unsigned>(lastDice));
  drawTextAt(result, 210, 425, 2, RGB565_YELLOW);
}

void drawPip(int col, int row, uint16_t color) {
  static const int16_t dx[3] = {55, 110, 165};
  static const int16_t dy[3] = {55, 110, 165};
  gfx->fillCircle(dieX + dx[col], dieY + dy[row], 13, color);
}

void drawDieBase() {
  gfx->fillRoundRect(dieX + 8, dieY + 10, kDieSize, kDieSize, 34, kDieShadow);
  gfx->fillRoundRect(dieX, dieY, kDieSize, kDieSize, 34, kDieFace);
  gfx->fillRoundRect(dieX + 8, dieY + 8, kDieSize - 16, kDieSize - 16, 27, kDieFaceLight);
  gfx->drawRoundRect(dieX, dieY, kDieSize, kDieSize, 34, kDieEdge);
  gfx->drawRoundRect(dieX + 4, dieY + 4, kDieSize - 8, kDieSize - 8, 30, kDieHighlight);

  gfx->fillRoundRect(dieX + 18, dieY + 16, 72, 10, 5, kDieHighlight);
}

void drawDiePips(uint8_t value, uint16_t color) {
  auto pip = [&](int col, int row) { drawPip(col, row, color); };

  switch (value) {
    case 1: pip(1, 1); break;
    case 2: pip(0, 0); pip(2, 2); break;
    case 3: pip(0, 0); pip(1, 1); pip(2, 2); break;
    case 4: pip(0, 0); pip(2, 0); pip(0, 2); pip(2, 2); break;
    case 5: pip(0, 0); pip(2, 0); pip(1, 1); pip(0, 2); pip(2, 2); break;
    case 6:
      pip(0, 0); pip(2, 0); pip(0, 1); pip(2, 1); pip(0, 2); pip(2, 2);
      break;
  }
}

void drawDieFace(uint8_t value) {
  drawStaticScreen();
  drawDieBase();
  drawDiePips(value, kPipColor);
}

void drawResult(uint8_t value) {
  gfx->fillRect(195, 418, 80, 28, RGB565_BLACK);
  char result[20];
  snprintf(result, sizeof(result), "D6  %u", static_cast<unsigned>(value));
  drawTextAt(result, 210, 425, 2, RGB565_YELLOW);
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
  const int16_t homeX = dieX;
  const int16_t homeY = dieY;

  for (uint8_t i = 0; i < 14; ++i) {
    const uint8_t next = (i == 13) ? to : static_cast<uint8_t>(random(1, 7));
    const float phase = static_cast<float>(i) * 0.65f;
    dieX = homeX + static_cast<int16_t>(sinf(phase) * (18.0f - i));
    dieY = homeY + static_cast<int16_t>(cosf(phase * 1.15f) * (12.0f - i / 2));

    gfx->fillScreen(RGB565_BLACK);
    drawStaticScreen();
    drawDieBase();
    drawDiePips(next, kPipColor);
    delay(35);
  }

  dieX = homeX;
  dieY = homeY;
  drawDieFace(to);
}

void rollDice(const char *reason) {
  const uint32_t now = millis();
  if (now - lastRollMs < kRollCooldownMs) return;

  lastRollMs = now;
  const uint8_t result = static_cast<uint8_t>(random(1, 7));
  animateDice(result);
  lastDice = result;
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

  // Gentle physical movement while the device is tilted.
  const int16_t targetX = kDieHomeX - static_cast<int16_t>(constrain(ay * 10.0f, -16.0f, 16.0f));
  const int16_t targetY = kDieHomeY + static_cast<int16_t>(constrain(ax * 10.0f, -16.0f, 16.0f));

  if (abs(targetX - dieX) > 1 || abs(targetY - dieY) > 1) {
    dieX += (targetX - dieX) / 3;
    dieY += (targetY - dieY) / 3;

    if (!menuOpen) {
      gfx->fillScreen(RGB565_BLACK);
      drawStaticScreen();
      drawDieBase();
      drawDiePips(lastDice, kPipColor);
    }
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
          dieX = kDieHomeX;
          dieY = kDieHomeY;
          drawDieFace(lastDice);
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

    if (!menuOpen &&
        x >= dieX && x <= dieX + kDieSize &&
        y >= dieY && y <= dieY + kDieSize) {
      trackingSwipe = false;
      rollDice("touch");
    }
    return;
  }

  if (!trackingSwipe) return;

  if (y < swipeMinY) swipeMinY = y;
  if (y > swipeMaxY) swipeMaxY = y;

  if (menuOpen) return;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== Round Dice motion dice test ===");
  Serial.println("Waveshare ESP32-S3-Touch-AMOLED-1.75");

  Wire.begin(IIC_SDA, IIC_SCL);

  Serial.println("Initializing CO5300 display...");
  if (!gfx->begin()) {
    Serial.println("ERROR: gfx->begin() failed!");
    while (true) delay(1000);
  }

  gfx->fillScreen(RGB565_BLACK);
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
  drawDieFace(lastDice);

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
