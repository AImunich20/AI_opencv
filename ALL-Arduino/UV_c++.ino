#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <WiFiS3.h>
#include "DHT.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// GND:GND VCC:5V SCL:D13 SDA:D11 RST:D8 DC:D9 CS:D10 BLK:3.3V
#define TFT_CS 10
#define TFT_RST 8
#define TFT_DC 9
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// Light Detection Light Sensing GND:GND VCC:3.3V
#define sen1 A0
#define sen2 A1

// UV Index GND:GND VCC:3.3V
const double uvSensorPin1 = A2;
const double uvSensorPin2 = A3;

// WIFI
const char ssid[] = "MaybeYouOK";  // change your network SSID (name)
const char pass[] = "natthanathron";   // change your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;
WiFiServer server(80);

// DHT22
#define DHTPIN 7
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// GPS Module GY-NEO6MV2 Ublox
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 115200;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// LED
// #define green 0
// #define yellow 1
// #define orange 2
// #define red 5
// #define violet 6

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST77XX_BLACK);
  text(1, 0, 0, "UV Index Start");
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION)
    text(1, 0, 10, "upgrade the firmware");
  while (status != WL_CONNECTED) {
    text(1, 0, 20, "connect to SSID: ");
    text(1, 0, 30, ssid);
    status = WiFi.begin(ssid, pass);
    delay(1000);
  }
  server.begin();
  text(1,0,60,"DHT22 test!");
  printWifiStatus();
  delay(2000);
  // tft.fillScreen(ST77XX_BLACK);
}

// WIFI IP
void printWifiStatus() {
  text(1, 0, 40, "IP Address: ");
  String IP = WiFi.localIP().toString();
  text(1, 0, 50, IP);
}

// led
void led(int color) {
  if (color == 1) {
    digitalWrite(0 , 1);
    digitalWrite(1 , 0);
    digitalWrite(2 , 0);
    digitalWrite(5 , 0);
    digitalWrite(6 , 0);
  }
  else if (color == 2) {
    digitalWrite(0 , 0);
    digitalWrite(1 , 1);
    digitalWrite(2 , 0);
    digitalWrite(5 , 0);
    digitalWrite(6 , 0);
  }
  else if (color == 3) {
    digitalWrite(0 , 0);
    digitalWrite(1 , 0);
    digitalWrite(2 , 1);
    digitalWrite(5 , 0);
    digitalWrite(6 , 0);
  }
  else if (color == 4) {
    digitalWrite(0 , 0);
    digitalWrite(1 , 0);
    digitalWrite(2 , 0);
    digitalWrite(5 , 1);
    digitalWrite(6 , 0);
  }
  else if (color == 5) {
    digitalWrite(0 , 0);
    digitalWrite(1 , 0);
    digitalWrite(2 , 0);
    digitalWrite(5 , 0);
    digitalWrite(6 , 1);
  }
}
// OLED Text
void text(int size, int x, int y, String texts) {
  tft.setTextSize(size);
  tft.setCursor(x, y);
  tft.println(texts);
}

// calculate UV Index
float calculateUVIndex(int sensorPin) {
  int uvValue = analogRead(sensorPin);
  float voltage = uvValue * (5.0 / 1024.0);
  float uvIntensity = voltage / 0.1;
  float uvIndex = uvIntensity / 250.0;
  return uvIndex;
}

// UV Index
void UV_Index(float uvIndex1 , float uvIndex2 , float uvIndexAvg) {
  text(1, 0, 0, "UV Index 1:");
  text(1, 70, 0, String(uvIndex1));
  text(1, 0, 10, "UV Index 2:");
  text(1, 70, 10, String(uvIndex2));
  text(1, 0, 20, "Avg UV Index:");
  text(1, 80, 20, String(uvIndexAvg));
}

// UV Zone
void UV_Zone(float uvIndex) {
  if (uvIndex >= 0 && uvIndex <= 2.9) {
    text(1, 0, 30, "<green zone>");
    led(1);
  }
  else if (uvIndex >= 3 && uvIndex <= 5.9) {
    text(1, 0, 30, "<yellow zone>");
    led(2);
  }
  else if (uvIndex >= 6 && uvIndex <= 7.9) {
    text(1, 0, 30, "<orange zone>");
    led(3);
  }
  else if (uvIndex >= 8 && uvIndex <= 10.9) {
    text(1, 0, 30, "<red zone>");
    led(4);
  }
  else {
    text(1, 0, 30, "<violet zone>");
    led(5);
  }
}

// GPS 
void GPS() {
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0) {
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {
      text(1, 0, 40, "Latitude = " + String(gps.location.lat(), 6));
      text(1, 0, 50, "Raw Lat = " + String(gps.location.rawLat().negative ? "-" : "+") + String(gps.location.rawLat().deg) + " " + String(gps.location.rawLat().billionths));
      text(1, 0, 70, "Raw Lng = " + String(gps.location.rawLng().negative ? "-" : "+") + String(gps.location.rawLng().deg) + " " + String(gps.location.rawLng().billionths));
      text(1, 0, 90, "Date(D/M/Y) = " + String(gps.date.value()));
    }
  }
}

// DHT22
void DHT22_index() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  
  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);
  // Text
  text(1, 0, 100, "Humidity: " + String(h));
  text(1, 0, 110, "% Temperature: " + String(t) + " C");
}

void loop() {  
  int light = (analogRead(sen1) + analogRead(sen2))/2;
  tft.fillScreen(ST77XX_BLACK);
  if (light >= 0 ) {
    tft.fillScreen(ST77XX_BLACK);
    float uvIndex1 = calculateUVIndex(uvSensorPin1);
    float uvIndex2 = calculateUVIndex(uvSensorPin2);
    float uvIndexAvg = (uvIndex1 + uvIndex2) / 2.0;
    UV_Index(uvIndex1 , uvIndex2 , uvIndexAvg);
    UV_Zone(uvIndexAvg);
    GPS();
    DHT22_index();
  }
  else tft.fillScreen(ST77XX_BLACK);
  delay(500);
}
