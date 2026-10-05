#include <Arduino.h>
#include <Wire.h>
#include "Arduino_GFX_Library.h"
#include "TouchDrvCSTXXX.hpp"
#include "pin_config.h"

namespace {
constexpr uint8_t kTouchAddress = CST92XX_SLAVE_ADDRESS;
constexpr uint8_t kMaxTouchPoints = 2;
constexpr int16_t kSwipeThreshold = 70;
constexpr int16_t kTopSwipeStart = 120;
constexpr uint32_t kTouchPollMs = 8;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_CO5300 *gfx = new Arduino_CO5300(
    bus, LCD_RESET, 0, LCD_WIDTH, LCD_HEIGHT, 6, 0, 0, 0);
TouchDrvCST92xx touch;

int16_t touchX[kMaxTouchPoints] = {};
int16_t touchY[kMaxTouchPoints] = {};
uint8_t lastDice = 6;
uint32_t lastTouchActionMs = 0;
uint32_t lastTouchPollMs = 0;
bool menuOpen = false;
bool touchWasDown = false;
bool trackingSwipe = false;
int16_t swipeStartY = 0;

void drawTextAt(const char *text, int16_t x, int16_t y, uint8_t size, uint16_t color) {
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  gfx->setCursor(x, y);
  gfx->print(text);
}

void drawStaticScreen() {
  gfx->fillScreen(RGB565_BLACK);
  drawTextAt("ROUND DICE", 145, 28, 3, RGB565_WHITE);
  drawTextAt("WAVESHARE TEST", 142, 65, 2, RGB565_CYAN);

  gfx->fillRoundRect(112, 365, 242, 58, 20, RGB565_BLUE);
  gfx->drawRoundRect(112, 365, 242, 58, 20, RGB565_WHITE);
  drawTextAt("WURFELN", 173, 384, 2, RGB565_WHITE);

  char result[32];
  snprintf(result, sizeof(result), "D6 = %u", static_cast<unsigned>(lastDice));
  drawTextAt(result, 196, 445, 2, RGB565_YELLOW);
}

const int16_t kDieX = 123;
const int16_t kDieY = 118;
const int16_t kDieSize = 220;
const int16_t kPipX[3] = {178, 233, 288};
const int16_t kPipY[3] = {173, 228, 283};
const int16_t kPipRadius = 13;

void drawPip(int col, int row, uint16_t color) {
  gfx->fillCircle(kPipX[col], kPipY[row], kPipRadius, color);
}

void drawDieBase() {
  gfx->fillRoundRect(kDieX, kDieY, kDieSize, kDieSize, 30, RGB565_WHITE);
  gfx->drawRoundRect(kDieX, kDieY, kDieSize, kDieSize, 30, RGB565_CYAN);
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
  drawDiePips(value, RGB565_BLACK);
}

void drawResult(uint8_t value) {
  gfx->fillRect(185, 440, 100, 26, RGB565_BLACK);
  char result[32];
  snprintf(result, sizeof(result), "D6 = %u", static_cast<unsigned>(value));
  drawTextAt(result, 196, 445, 2, RGB565_YELLOW);
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

void rollDice() {
  drawDieBase();
  drawDiePips(lastDice, RGB565_BLACK);

  uint8_t previous = lastDice;
  for (uint8_t i = 0; i < 10; ++i) {
    drawDiePips(previous, RGB565_WHITE);
    uint8_t next = static_cast<uint8_t>(random(1, 7));
    drawDiePips(next, RGB565_BLACK);
    previous = next;
    delay(55);
  }

  lastDice = previous;
  drawResult(lastDice);
  Serial.printf("D6 roll: %u\n", static_cast<unsigned>(lastDice));
}

void processTouch() {
  const uint8_t supportedPoints = touch.getSupportTouchPoint();
  const uint8_t pointLimit =
      supportedPoints < kMaxTouchPoints ? supportedPoints : kMaxTouchPoints;
  const uint8_t touchedPoints = touch.getPoint(touchX, touchY, pointLimit);
  const bool isDown = touchedPoints > 0;

  if (!isDown) {
    if (touchWasDown) {
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
    Serial.printf("Touch start: X=%d Y=%d\n", x, y);
    return;
  }

  if (!trackingSwipe) return;

  // Swipe down: poll the controller continuously while the finger is held.
  if (!menuOpen && swipeStartY <= kTopSwipeStart &&
      y - swipeStartY >= kSwipeThreshold) {
    menuOpen = true;
    trackingSwipe = false;
    drawMenu();
    Serial.println("Swipe down: menu opened");
    return;
  }

  if (menuOpen && swipeStartY >= 220 &&
      swipeStartY - y >= kSwipeThreshold) {
    menuOpen = false;
    trackingSwipe = false;
    drawDieFace(lastDice);
    Serial.println("Swipe up: menu closed");
    return;
  }

  if (menuOpen) return;

  const uint32_t now = millis();
  if (now - lastTouchActionMs < 300) return;

  if (x >= 112 && x <= 354 && y >= 365 && y <= 423) {
    lastTouchActionMs = now;
    trackingSwipe = false;
    rollDice();
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== Round Dice hardware test ===");
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

  randomSeed(static_cast<unsigned long>(micros()));
  drawDieFace(lastDice);

  Serial.printf("Touch controller: %s\n", touch.getModelName());
  Serial.printf("Supported touch points: %u\n",
                static_cast<unsigned>(touch.getSupportTouchPoint()));
  Serial.println("Hardware test ready.");
}

void loop() {
  const uint32_t now = millis();
  if (now - lastTouchPollMs >= kTouchPollMs) {
    lastTouchPollMs = now;
    processTouch();
  }
  delay(1);
}
