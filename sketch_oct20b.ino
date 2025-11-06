#include <LiquidCrystal.h>

#define LDR_PIN    A0    // ambient light sensor
#define PIR_PIN    2     // motion detector
#define DHT_PIN    3     // DHT22 data pin
#define RED_PIN    9     // R-channel via NPN
#define GREEN_PIN 10     // G-channel
#define BLUE_PIN  11     // B-channel

LiquidCrystal lcd(12, 8, 4, 5, 6, 7);

uint8_t dhtData[5];
bool readDHT22(float &T, float &H) {
  unsigned long t;
  pinMode(DHT_PIN, OUTPUT);
  digitalWrite(DHT_PIN, LOW);
  delay(1);              
  digitalWrite(DHT_PIN, HIGH);
  delayMicroseconds(30);
  pinMode(DHT_PIN, INPUT_PULLUP);

  t = micros();
  while (digitalRead(DHT_PIN) == HIGH)
    if (micros() - t > 100) return false;
  t = micros();
  while (digitalRead(DHT_PIN) == LOW)
    if (micros() - t > 100) return false;
  t = micros();
  while (digitalRead(DHT_PIN) == HIGH)
    if (micros() - t > 100) return false;

  memset(dhtData, 0, 5);
  for (int bit = 0; bit < 40; bit++) {
    t = micros();
    while (digitalRead(DHT_PIN) == LOW)
      if (micros() - t > 100) return false;
    t = micros();
    while (digitalRead(DHT_PIN) == HIGH)
      if (micros() - t > 100) return false;
    if (micros() - t > 40) {
      dhtData[bit/8] |= 1 << (7 - (bit%8));
    }
  }

  uint8_t sum = dhtData[0] + dhtData[1] + dhtData[2] + dhtData[3];
  if (sum != dhtData[4]) return false;

  uint16_t rawH = (dhtData[0] << 8) | dhtData[1];
  uint16_t rawT = (dhtData[2] << 8) | dhtData[3];
  H = rawH * 0.1;
  if (rawT & 0x8000) {
    rawT &= 0x7FFF;
    T = -rawT * 0.1;
  } else {
    T = rawT * 0.1;
  }
  return true;
}

float activityScore = 1.0;
const float alphaUp   = 0.05, alphaDown = 0.02;

unsigned long lastLCD   = 0;
const unsigned long LCD_INTERVAL = 1000; 

void setup() {
  Serial.begin(9600);

  pinMode(PIR_PIN, INPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(DHT_PIN, INPUT_PULLUP);

  lcd.begin(16, 2);
  lcd.print("EmotionLamp v1");
  delay(1500);
  lcd.clear();
}

void loop() {
  unsigned long now = millis();
  int  ldrVal = analogRead(LDR_PIN);         
  bool motion = (digitalRead(PIR_PIN) == HIGH);

  if (motion)
    activityScore += alphaUp * (1.0 - activityScore);
  else
    activityScore -= alphaDown * activityScore;
  activityScore = constrain(activityScore, 0.0, 1.0);

  static float tempC = 25, hum = 50;
  float t, h;
  if (readDHT22(t, h)) {
    tempC = t;
    hum   = h;
  }
  int hourOfDay = (millis() / 1000) % 24;
  int cct = (hourOfDay < 7  ? 2700 :
             hourOfDay < 11 ? 6000 :
             hourOfDay < 16 ? 4800 :
             hourOfDay < 20 ? 3500 :
             hourOfDay < 23 ? 2800 : 2700);

  float tNorm = constrain((tempC - 20.0) / 10.0, 0.0, 1.0);
  float hNorm = constrain(hum / 100.0, 0.0, 1.0);
  if ((tNorm + hNorm) * 0.5 > 0.7) {
    cct = min(cct + 300, 6500);
  }

  int bright = map(ldrVal, 0, 1023, 255, 127);
  bright = constrain(bright, 127, 255);

  byte rTbl, gTbl, bTbl;
  if      (cct <= 2700) { rTbl = 255; gTbl = 147; bTbl =  44; }
  else if (cct <= 3200) { rTbl = 255; gTbl = 180; bTbl = 107; }
  else if (cct <= 4000) { rTbl = 255; gTbl = 209; bTbl = 163; }
  else if (cct <= 5000) { rTbl = 255; gTbl = 228; bTbl = 206; }
  else                  { rTbl = 255; gTbl = 249; bTbl = 253; }

  int rPWM = (rTbl * bright) / 255;
  int gPWM = (gTbl * bright) / 255;
  int bPWM = (bTbl * bright) / 255;

  analogWrite(RED_PIN,   rPWM);
  analogWrite(GREEN_PIN, gPWM);
  analogWrite(BLUE_PIN,  bPWM);

  if (now - lastLCD > LCD_INTERVAL) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(tempC, 1);
    lcd.print("C H:");
    lcd.print(hum, 0);
    lcd.print("%");

    lcd.setCursor(0, 1);
    lcd.print("CCT:");
    lcd.print(cct);
    lcd.print(" B:");
    lcd.print(bright * 100 / 255);
    lcd.print("%");

    Serial.print("LDR=");     Serial.print(ldrVal);
    Serial.print(" PIR=");     Serial.print(motion ? 1 : 0);
    Serial.print(" T=");       Serial.print(tempC); Serial.print("C");
    Serial.print(" H=");       Serial.print(hum);    Serial.print("%");
    Serial.print(" Act=");     Serial.print(activityScore, 2);
    Serial.print(" CCT=");     Serial.print(cct);
    Serial.print(" Bright=");  Serial.println(bright);

    lastLCD = now;
  }

  if (activityScore < 0.1) {
    for (int i = 0; i < 3; i++) {
      analogWrite(RED_PIN,   200);
      analogWrite(GREEN_PIN,  80);
      analogWrite(BLUE_PIN,    0);
      delay(300);
      analogWrite(RED_PIN,     0);
      analogWrite(GREEN_PIN,   0);
      analogWrite(BLUE_PIN,     0);
      delay(300);
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Take a short");
    lcd.setCursor(0, 1);
    lcd.print("break + hydrate");
    delay(2000);
  }

  delay(50);
}
