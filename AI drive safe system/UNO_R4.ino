#include <WiFiS3.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Engeach_O2.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;

float predOxygen;
float predTemperature;

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 38400;

// GPS
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// MPU6050
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz, rawTemp;
bool blinkState;
bool mpuAvailable = false;
float tempC = 0;
float accelX, accelY, accelZ;
float angularVelX, angularVelY, angularVelZ;
float velocityX = 0, velocityY = 0, velocityZ = 0;
unsigned long lastTime = 0;
float dt;

// OLED
#define SCREEN_ADDRESS 0x3C  // OLED I2C address
#define SCREEN_WIDTH 128     // OLED display width
#define SCREEN_HEIGHT 64     // OLED display height
#define OLED_RESET -1        // Reset pin for OLED (not used in QT-PY / XIAO)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Oxygen Sensor
const uint8_t analogPin = A0;
const float offsetVoltage = 0.016;
const float sensitivity = 4.6;
Engeach_O2 o2Sensor(analogPin, offsetVoltage, sensitivity);
float percentage = 0.0;
float voltage = 0.0;
uint16_t rawValue = 0;

// Wi-Fi
const char* ssid = "UFI-521718";
const char* password = "12345678";
WiFiServer server(80);

void setup() {
  Serial.begin(38400);
  ss.begin(GPSBaud);

  // Wi-Fi connection
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // OLED setup
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);  // Hang if OLED setup fails
  }
  display.clearDisplay();

  // MPU6050 setup
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  mpu.initialize();
  
  Serial.println("Testing MPU6050 connection...");
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
    mpuAvailable = true;
  } else {
    Serial.println("MPU6050 not found, skipping...");
    mpuAvailable = false;
  }

  if (mpuAvailable) {
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
  }

  pinMode(LED_BUILTIN, OUTPUT);
  server.begin();

}

void loop() {
  WiFiClient client = server.available();
  // client_oxygen(client, 0, 0, 0, 0, 0);
  oxygen_value();
  if (mpuAvailable) {
    MPU6050_value();
  }
  OLED();  // Update OLED display
  GPS_index();  // Read GPS data

  // เก็บข้อมูลเวลา ออกซิเจน และอุณหภูมิ
  static vector<float> timeData;
  static vector<float> oxygenData;
  static vector<float> temperatureData;
  static unsigned long startTime = millis();

  float elapsedTime = (millis() - startTime) / 60000.0;  // แปลงเป็นนาที

  timeData.push_back(elapsedTime);
  oxygenData.push_back(percentage);
  temperatureData.push_back(tempC);

  // เก็บข้อมูลแค่ 20 จุดล่าสุด
  if (timeData.size() > 20) {
    timeData.erase(timeData.begin());
    oxygenData.erase(oxygenData.begin());
    temperatureData.erase(temperatureData.begin());
  }

  // ทำนายค่าออกซิเจนและอุณหภูมิในอีก 10 นาที
  float futureTime = elapsedTime + 100;
  predictOxygenAndTemperature(futureTime, timeData, oxygenData, temperatureData);

  // ถ้าออกซิเจนต่ำกว่า 14.5% ส่งข้อมูลไปยัง client
  if (predOxygen < 14.5) {
    if (client) {
      client_oxygen(client, percentage, oxygenData.back(), futureTime, tempC, temperatureData.back());
    }
  }
}

void OLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("O2: ");
  display.print(percentage);
  display.println("%");
  display.setCursor(0, 10);
  display.print("IP: ");
  display.println(WiFi.localIP());
  display.setCursor(0, 20);
  display.print("predOxygen: ");
  display.println(predOxygen);
  display.setCursor(0, 30);
  display.print("predTemperature: ");
  display.println(predTemperature);

  if (mpuAvailable) {
    tempC = rawTemp / 340.0 + 36.53;  // Convert raw temperature to Celsius
    display.setCursor(0, 40);
    display.print("Temp: ");
    display.print(tempC, 2);
    display.println(" C");
  } else {
    display.setCursor(0, 50);
    display.print("MPU6050 Not Found");
  }

  display.display();
}

void client_oxygen(WiFiClient &client, float oxygen, float predicOxygen, float time, float temperature, float predicTemperature) {
  if (client) {
    Serial.println("Client connected");
    String request = client.readStringUntil('\r');
    Serial.println(request);
    client.flush();

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: keep-alive");
    client.println();

    client.print("{\"oxygen\":");
    client.print(oxygen);
    client.print(", \"predicted_oxygen\":");
    client.print(predicOxygen);
    client.print(", \"predicted_time\":");
    client.print(time);
    client.print(", \"temperature\":");
    client.print(temperature);
    client.print(", \"predicted_temperature\":");
    client.print(predicTemperature);
    client.println("}");

    delay(100);
    client.stop();
    Serial.println("Client disconnected");
  } else {
    Serial.println("Client not connected");
  }
}

void oxygen_value() {
  voltage = o2Sensor.readVout();
  percentage = o2Sensor.getO2_percentage();
  rawValue = o2Sensor.readValue();
}

void MPU6050_value() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  rawTemp = mpu.getTemperature();  // Get temperature from MPU6050
  tempC = rawTemp / 340.0 + 36.53;  // Convert to Celsius

  // Convert accelerometer data to m/s² (assuming ±2g scale)
  accelX = ax / 16384.0 * 9.81;  // Convert to m/s²
  accelY = ay / 16384.0 * 9.81;
  accelZ = az / 16384.0 * 9.81;

  // Integrate acceleration to get velocity (simple approximation)
  velocityX += accelX * dt;
  velocityY += accelY * dt;
  velocityZ += accelZ * dt;

  // Angular velocity from gyroscope data (assuming ±250 degrees/second scale)
  angularVelX = gx / 131.0;  // degrees per second
  angularVelY = gy / 131.0;
  angularVelZ = gz / 131.0;

  // Print sensor data
  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.print(gz); Serial.print("\t");
  Serial.print("Temp: ");
  Serial.print(tempC, 2);
  Serial.println(" C");

  Serial.print("Linear Velocity (m/s):\t");
  Serial.print(velocityX, 2); Serial.print("\t");
  Serial.print(velocityY, 2); Serial.print("\t");
  Serial.print(velocityZ, 2); Serial.println();

  Serial.print("Angular Velocity (deg/s):\t");
  Serial.print(angularVelX, 2); Serial.print("\t");
  Serial.print(angularVelY, 2); Serial.print("\t");
  Serial.print(angularVelZ, 2); Serial.println();

  blinkState = !blinkState;
  digitalWrite(LED_BUILTIN, blinkState);
}

void GPS_index() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {
      Serial.print("Latitude= ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude= ");
      Serial.println(gps.location.lng(), 6);
    }
  }
}

// ฟังก์ชัน Linear Regression
vector<float> linearRegression(vector<float> x, vector<float> y) {
    int n = x.size();
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

    for (int i = 0; i < n; i++) {
        sumX += x[i];
        sumY += y[i];
        sumXY += x[i] * y[i];
        sumX2 += x[i] * x[i];
    }

    // คำนวณ slope (a) และ intercept (b)
    float a = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    float b = (sumY - a * sumX) / n;

    return {a, b}; // คืนค่า slope และ intercept
}

// ฟังก์ชันสำหรับทำนายค่า O2 และอุณหภูมิ
void predictOxygenAndTemperature(float newTime, vector<float> time, vector<float> oxygen, vector<float> temperature) {
    if (time.size() < 50) { // ต้องมีข้อมูลอย่างน้อย 2 จุดเพื่อทำนาย
        predOxygen = oxygen.back();
        predTemperature = temperature.back();
        return;
    }

    vector<float> oxygenParams = linearRegression(time, oxygen);
    vector<float> tempParams = linearRegression(time, temperature);

    predOxygen = oxygenParams[0] * newTime + oxygenParams[1]; // Linear prediction for oxygen
    predTemperature = tempParams[0] * newTime + tempParams[1]; // Linear prediction for temperature

    // ป้องกันค่าที่เกินจริง
    predOxygen = max(0.0f, min(1000.0f, predOxygen)); // Oxygen should be between 0 and 100%
    predTemperature = max(-50.0f, min(1000.0f, predTemperature)); // Temperature should be reasonable
}
