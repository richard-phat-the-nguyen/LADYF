#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define SDP610_ADDR   0x40
#define SCALE_FACTOR  240.0f

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

unsigned long INTERVAL_UPDATE = 250;
unsigned long tic = 0;
float P = 0;

void setup() {
  Wire.begin();

  Wire.beginTransmission(SDP610_ADDR);
  Wire.write(0xFE);
  Wire.endTransmission();
  delay(100);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();
}

void loop() {
  if (millis() - tic > INTERVAL_UPDATE) {
    P = LecturePression();
    Impression_pression();
    tic = millis();
  }
}

float LecturePression() {
  Wire.beginTransmission(SDP610_ADDR);
  Wire.write(0xF1);
  Wire.endTransmission();

  Wire.requestFrom(SDP610_ADDR, 2);

  if (Wire.available() < 2) return 0.0f;

  int16_t raw = (Wire.read() << 8) | Wire.read();
  return raw / SCALE_FACTOR;
}

void Impression_pression() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Pression");
  display.println("[Pa]");
  display.setTextSize(3);
  display.setCursor(0, 40);
  display.println(P, 1);
  display.display();
}