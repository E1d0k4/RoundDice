#include <Arduino.h>
#include <Wire.h>
#include "Arduino_GFX_Library.h"
#include "TouchDrvCSTXXX.hpp"
#include "pin_config.h"

namespace {
constexpr uint8_t kTouchAddress = CST92XX_SLAVE_ADDRESS;
constexpr uint8_t kMaxTouchPoints = 2;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_CO5300 *gfx = new Arduino_CO5300(
    bus, LCD_RESET, 0, LCD_WIDTH, LCD_HEIGHT, 6, 0, 0, 0);
TouchDrvCST92xx touch;

int16_t touchX[kMaxTouchPoints] = {};
int16_t touchY[kMaxTouchPoints] = {};
volatile bool touchPending = false;
uint8_t lastDice = 6;
uint32_t lastTouchMs = 0;

void IRAM_ATTR onTouchInterrupt() { touchPending = true; }

bool takeTouchInterrupt() {
  noInterrupts();
  const bool pending = touchPending;
  touchPending = false;
  interrupts();
  return pending;
}

void drawCenteredText(const char *text, int16_t y, uint8_t size, uint16_t color) {
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  const int16_t width = gfx->textWidth(text);
  gfx->setCursor((LCD_WIDTH - width) / 2, y);
  gfx->print(text);
}

void drawDieFace(uint8_t value) {
  gfx->fillScreen(RGB565_BLACK);
  drawCenteredText("ROUND DICE", 28, 3, RGB565_WHITE);
  drawCenteredText("WAVESHARE TEST", 65, 2, RGB565_CYAN);

  gfx->fillRoundRect(123, 118, 220, 220, 30, RGB565_WHITE);
  gfx->drawRoundRect(123, 118, 220, 220, 30, RGB565_CYAN);

  const int16_t cx[3] = {178, 233, 288};
  const int16_t cy[3] = {173, 228, 283};

  auto pip = [&](int col, int row) {
    gfx->fillCircle(cx[col], cy[row], 13, RGB565_BLACK);
  };

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

  gfx->fillRoundRect(112, 365, 242, 58, 20, RGB565_BLUE);
  gfx->drawRoundRect(112, 365, 242, 58, 20, RGB565_WHITE);
  drawCenteredText("WURFELN", 384, 2, RGB565_WHITE);

  char result[32];
  snprintf(result, sizeof(result), "D6 = %u", static_cast<unsigned>(value));
  drawCenteredText(result, 445, 2, RGB565_YELLOW);
}

void showTouch(int16_t x, int16_t y) {
  gfx->fillRoundRect(48, 8, 370, 34, 12, RGB565_DARKGREY);
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565_WHITE);
  char buffer[48];
  snprintf(buffer, sizeof(buffer), "Touch X:%d Y:%d", x, y);
  const int16_t width = gfx->textWidth(buffer);
  gfx->setCursor((LCD_WIDTH - width) / 2, 17);
  gfx->print(buffer);
}

void rollDice() {
  for (uint8_t i = 0; i < 8; ++i) {
    drawDieFace(static_cast<uint8_t>(random(1, 7)));
    delay(55);
  }
  lastDice = static_cast<uint8_t>(random(1, 7));
  drawDieFace(lastDice);
  Serial.printf("D6 roll: %u\n", static_cast<unsigned>(lastDice));
}

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
    drawCenteredText("TOUCH ERROR", 220, 3, RGB565_RED);
    while (true) delay(1000);
  }

  touch.setMaxCoordinates(LCD_WIDTH, LCD_HEIGHT);
  touch.setMirrorXY(true, true);

  pinMode(TP_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TP_INT), onTouchInterrupt, FALLING);

  randomSeed(static_cast<unsigned long>(micros()));
  drawDieFace(lastDice);

  Serial.printf("Touch controller: %s\n", touch.getModelName());
  Serial.printf("Supported touch points: %u\n",
                static_cast<unsigned>(touch.getSupportTouchPoint()));
  Serial.println("Hardware test ready.");
}

void loop() {
  if (!takeTouchInterrupt()) {
    delay(1);
    return;
  }

  const uint8_t supportedPoints = touch.getSupportTouchPoint();
  const uint8_t pointLimit =
      supportedPoints < kMaxTouchPoints ? supportedPoints : kMaxTouchPoints;
  const uint8_t touchedPoints = touch.getPoint(touchX, touchY, pointLimit);
  if (touchedPoints == 0) return;

  const int16_t x = touchX[0];
  const int16_t y = touchY[0];

  Serial.printf("Touch: X=%d Y=%d points=%u\n",
                x, y, static_cast<unsigned>(touchedPoints));

  const uint32_t now = millis();
  if (now - lastTouchMs < 250) return;
  lastTouchMs = now;

  if (x >= 112 && x <= 354 && y >= 365 && y <= 423) {
    rollDice();
  } else {
    showTouch(x, y);
    delay(500);
    drawDieFace(lastDice);
  }
}
