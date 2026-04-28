//Code co-conçu avec Copilot IA et modification et correction majeur

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Définition de la taille d'écran OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// =========================
// Adjustable update interval
// =========================
unsigned long INTERVAL_UPDATE = 1000;   // milliseconds

unsigned long tic = 0;
const int PIN_SDP = A0;
float P;

void setup() {
  Wire.begin();
  randomSeed(analogRead(A0));
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  Serial.begin(9600);
}

void loop() {
  unsigned long toc = millis() - tic;

  if (toc > INTERVAL_UPDATE) {
    Impression_pression();
  }
  Serial.println(P);
}


void Impression_pression() {
  tic = millis();                     // reset tic

  //int valeur = random(0, 126);        // 0 to 125
  P=LecturePression();
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("Pression");
  display.println("[Pa]");

  display.setTextSize(3);
  display.setCursor(0, 40);
  //display.println(valeur);
  display.println(P);
  display.display();
}


float LecturePression() {
  int N = analogRead(PIN_SDP);          // 0..1023
  float dp = 750.0f * (float)N / 1023.0f - 150.0f;
  return dp;                            // Pascals
}