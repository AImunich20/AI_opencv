// Include required libraries
//#define BLYNK_TEMPLATE_ID "TMPL6_q-9ZFan"
//#define BLYNK_TEMPLATE_NAME "Smart Farm"
//#define BLYNK_AUTH_TOKEN "6KSwG9MSWUQmn-Ew_jjjw14uqc_rlQ-M"
// #define BLYNK_TEMPLATE_ID "TMPL6uxt4DXqY"
// #define BLYNK_TEMPLATE_NAME "MushCool Pro"
// #define BLYNK_AUTH_TOKEN "JTismPDuVAX4pW8-sDHi8MOV9rSn0nSl"

// #define BLYNK_TEMPLATE_ID "TMPL6K_cy04fa"
// #define BLYNK_TEMPLATE_NAME "smart"
// #define BLYNK_AUTH_TOKEN "5wNVB1Vl7dRAjNgTYv3N2jNtUTYMEpgE"

// #define BLYNK_TEMPLATE_ID "TMPL6bqELNP-m"
// #define BLYNK_TEMPLATE_NAME "smart"
// #define BLYNK_AUTH_TOKEN "qoWv0EsgCjTq5RfDcALutyCuCoscQ_Hc"

#define BLYNK_TEMPLATE_ID "TMPL6uVLkVrAc"
#define BLYNK_TEMPLATE_NAME "smart"
#define BLYNK_AUTH_TOKEN "CPubwSTcAwuMauv7Uqyng0S-_pYWYUsj"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Arduino.h>
#include <Wire.h>
#include <ArtronShop_SHT45.h>
#include <ArtronShop_BH1750.h>

// WiFi credentials
const char* ssid = "MaybeYouOK";
const char* pass = "natthanathron";

// Sensor objects
ArtronShop_BH1750 bh1750(0x23, &Wire); // Non-Jump ADDR: 0x23
ArtronShop_SHT45 sht45(&Wire, 0x44);   // SHT45-AD1B => 0x44

// Define the pump and fan pins
const int pump = 4;  
const int fan = 16; 

// Variables for temperature and humidity
double tem;
double hum;

// Blynk virtual pin handlers
BLYNK_WRITE(V5) {
  int A = param.asInt();
  if (A == 1) {
    if (tem > 28) {
      digitalWrite(fan, HIGH);
      digitalWrite(pump, HIGH);
    } else if (tem < 28 && tem > 26) {
      digitalWrite(fan, LOW);
      digitalWrite(pump, LOW);
    }
  }
}

BLYNK_WRITE(V6) {
  int B = param.asInt();
  if (B == 1) {
    if (tem > 27) {
      digitalWrite(fan, LOW);
      digitalWrite(pump, LOW);
    } else if (tem < 27 && tem > 25) {
      digitalWrite(fan, HIGH);
      digitalWrite(pump, HIGH);
    }
  }
}

BLYNK_WRITE(V7) {
  int C = param.asInt();
  if (C == 1) {
    if (tem > 27) {
      digitalWrite(fan, LOW);
      digitalWrite(pump, LOW);
    } else if (tem < 27 && tem > 25) {
      digitalWrite(fan, HIGH);
      digitalWrite(pump, HIGH);
    }
  }
}

BLYNK_WRITE(V3) {
  int pinValue = param.asInt();
  digitalWrite(pump, pinValue);
}

BLYNK_WRITE(V4) {
  int pinValue = param.asInt();
  digitalWrite(fan, pinValue);
}

BLYNK_WRITE(V8) {
  int pinValue = param.asInt();
  if (pinValue == 1) {
    digitalWrite(fan, LOW);
    digitalWrite(pump, LOW);
    Blynk.virtualWrite(V3, 0); 
    Blynk.virtualWrite(V4, 0); 
    Blynk.virtualWrite(V5, 0); 
    Blynk.virtualWrite(V6, 0);  
    Blynk.virtualWrite(V7, 0);  
  }
}

void setup() {
  // Debug console
  Serial.begin(115200);

  // Initialize pins
  pinMode(pump, OUTPUT);
  pinMode(fan, OUTPUT);

  // Initialize sensors
  Wire.begin();
  while (!sht45.begin()) {
    Serial.println("SHT45 not found!");
    delay(1000);
  }
  while (!bh1750.begin()) {
    Serial.println("BH1750 not found!");
    delay(1000);
  }

  // Connect to Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop() {
  // Measure temperature and humidity
  if (sht45.measure()) {
    tem = sht45.temperature();
    hum = sht45.humidity();

    Serial.print("Temperature: ");
    Serial.print(tem, 1);
    Serial.print(" *C\tHumidity: ");
    Serial.print(hum, 1);
    Serial.println(" %RH");

    // Send sensor data to Blynk
    Blynk.virtualWrite(V0, tem);
    Blynk.virtualWrite(V1, hum);
    Blynk.virtualWrite(V2, bh1750.light());
  } else {
    Serial.println("SHT45 read error");
  }

  // Run Blynk
  Blynk.run();

  // Delay for stability
  delay(5000);
}
