// this is the c code for RoadSense , the components are Arduino Uno R3, MPU6050 and 0.96 Oled Display. it will classify the readings into normal ,rougn and pothole and also count the number of potholes encountered //
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------- OLED ----------------
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3C   // Wokwi uses 0x3C by default

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------------- MPU6050 ----------------
#define MPU_ADDR 0x68
const float ACCEL_SCALE = 8192.0;   

// ---------------- Sampling / window ----------------
const int SAMPLE_INTERVAL_MS = 20;    
const int WINDOW_SIZE        = 10;    
const int WINDOW_INTERVAL_MS = 200;   

float devBuffer[WINDOW_SIZE];
int   bufIndex = 0;

unsigned long lastSampleTime = 0;
unsigned long lastWindowTime = 0;

// ---------------- Thresholds ----------------
float POTHOLE_THRESHOLD = 0.60;   
float ROUGH_THRESHOLD   = 0.15;   

// ---------------- Calibration ----------------
float baselineG = 1.0;   

// ---------------- State ----------------
enum RoadStatus { ROAD_NORMAL, ROAD_ROUGH, ROAD_POTHOLE };
RoadStatus currentStatus = ROAD_NORMAL;

unsigned long potholeHoldUntil = 0;
const unsigned long POTHOLE_HOLD_MS = 1000;  

long  potholeCount = 0;
float lastPeak = 0;
float lastAvg  = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);   

  mpuInit();
  calibrateBaseline();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("OLED not found - check wiring/I2C address"));
    while (true) {}
  }
  
  showSplash();
}

void loop() {
  unsigned long now = millis();

  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;
    sampleAndBuffer();
  }

  if (now - lastWindowTime >= WINDOW_INTERVAL_MS) {
    lastWindowTime = now;
    classifyWindow();
    updateDisplay();
    logSerial();
  }
}

// ================= MPU6050 =================
void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // PWR_MGMT_1
  Wire.write(0x00);   // wake the sensor up
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);   // ACCEL_CONFIG
  Wire.write(0x08);   // +/-4g range
  Wire.endTransmission(true);
}

void readAccelG(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);   
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  uint8_t axH = Wire.read(); uint8_t axL = Wire.read();
  uint8_t ayH = Wire.read(); uint8_t ayL = Wire.read();
  uint8_t azH = Wire.read(); uint8_t azL = Wire.read();

  int16_t axRaw = (axH << 8) | axL;
  int16_t ayRaw = (ayH << 8) | ayL;
  int16_t azRaw = (azH << 8) | azL;

  ax = axRaw / ACCEL_SCALE;
  ay = ayRaw / ACCEL_SCALE;
  az = azRaw / ACCEL_SCALE;
}

void calibrateBaseline() {
  Serial.println(F("Calibrating - keep the sensor still..."));
  float sum = 0;
  const int N = 100;
  for (int i = 0; i < N; i++) {
    float ax, ay, az;
    readAccelG(ax, ay, az);
    sum += sqrt(ax * ax + ay * ay + az * az);
    delay(10);
  }
  baselineG = sum / N;
  Serial.print(F("Baseline (g): "));
  Serial.println(baselineG, 3);
}

// ================= Sampling / classification =================
void sampleAndBuffer() {
  float ax, ay, az;
  readAccelG(ax, ay, az);

  float magnitude = sqrt(ax * ax + ay * ay + az * az);
  float deviation = fabs(magnitude - baselineG);

  devBuffer[bufIndex] = deviation;
  bufIndex = (bufIndex + 1) % WINDOW_SIZE;

  if (deviation >= POTHOLE_THRESHOLD) {
    if (currentStatus != ROAD_POTHOLE) potholeCount++;
    currentStatus = ROAD_POTHOLE;
    potholeHoldUntil = millis() + POTHOLE_HOLD_MS;
  }
}

void classifyWindow() {
  float peak = 0, sum = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    if (devBuffer[i] > peak) peak = devBuffer[i];
    sum += devBuffer[i];
  }
  float avg = sum / WINDOW_SIZE;
  lastPeak = peak;
  lastAvg  = avg;

  if (millis() < potholeHoldUntil) {
    currentStatus = ROAD_POTHOLE;         
  } else if (avg >= ROUGH_THRESHOLD) {
    currentStatus = ROAD_ROUGH;
  } else {
    currentStatus = ROAD_NORMAL;
  }
}

const char* statusText(RoadStatus s) {
  switch (s) {
    case ROAD_NORMAL:  return "NORMAL";
    case ROAD_ROUGH:   return "ROUGH";
    case ROAD_POTHOLE: return "POTHOLE";
  }
  return "";
}

// ================= Display =================
void showSplash() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);   
  
  display.setCursor(10, 0);
  display.println(F("BHEEEM"));
  
  display.setCursor(10, 12);
  display.println(F("MADE BY -"));
  
  display.setCursor(10, 26);
  display.println(F("ADITYA JOSHI,"));
  
  display.setCursor(10, 38);
  display.println(F("LAKSHENDRA Mishra,"));
  
  display.setCursor(10, 50);
  display.println(F("NIKHIL REDDY"));
  
  display.display();
  delay(2000);  

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10, 24);
  display.println(F("RoadSense"));
  display.display();
  delay(1200);  
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("RoadSense"));
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(10, 22);
  display.println(statusText(currentStatus));

  display.setTextSize(1);
  display.setCursor(0, 50);
  display.print(F("Potholes: "));
  display.println(potholeCount);

  display.display();
}

void logSerial() {
  Serial.print(F("peak:"));
  Serial.print(lastPeak, 3);
  Serial.print(F(" avg:"));
  Serial.print(lastAvg, 3);
  Serial.print(F(" -> "));
  Serial.print(statusText(currentStatus));
  Serial.print(F(" (potholes: "));
  Serial.print(potholeCount);
  Serial.println(F(")"));
}
