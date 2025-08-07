#include <WiFiS3.h>
#include "arduino_secrets.h"
#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "lcd_display.h"
#include "WifiManager.h"
#include <ArduinoHttpClient.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "TemperatureControl.h"

/* ======================
   MAIN ARDUINO SKETCH
   ======================
   This sketch implements an automated temperature and humidity control system
   with heating/cooling, fan management, LCD display, LED matrix, and Wi-Fi.
   Interacts with Arduino Nano via I2C for fan control.
*/

/* --- Pin and constant definitions ---
   NANO_ADDR - I2C address of Arduino Nano
   FAN_CHANNEL - fan channel (used for control via Nano)
   DHTPIN - pin for DHT11 sensor (temperature/humidity)
   FAN_PIN - PWM pin for coolers
*/

/* --- Global objects ---
   matrix - LED matrix
   dht - temperature and humidity sensor
   lcd - LCD display via I2C
   wifiManager - Wi-Fi management
   httpClient - HTTP client for data sending
   tc - temperature and actuator control
*/

/* --- setup() ---
   Initializes I2C, turns off fans via Nano, sets up temperature controller,
   starts Serial, LCD, Wi-Fi, DHT11 sensor, LED matrix. Sends first data to server.
*/

/* --- loop() ---
   Checks Wi-Fi, updates data on display and LED matrix, sends data to server,
   updates temperature controller state, 2-second delay.
*/

/* --- lcdLog ---
   Displays the last two log lines on the LCD display.
*/

/* --- printSensorData ---
   Reads temperature and humidity, displays on LCD and LED matrix.
*/

/* --- sendSensorData ---
   Sends temperature, humidity, and system state data to the server via HTTP POST.
*/

// --- Pin and constant definitions ---
// NANO_ADDR - I2C address of Arduino Nano
#define NANO_ADDR 0x10
// FAN_CHANNEL - fan channel (used for control via Nano)
// Minimum PWM value.
#define PWM_MIN 0
 // Maximum PWM value.
#define PWM_MAX 4095


// DHTPIN - pin for DHT11 sensor (temperature/humidity)
#define DHTPIN 2
// DHTTYPE - type of DHT sensor (DHT11, DHT22, DHT21)
#define DHTTYPE DHT11
// API_ENDPOINT - endpoint for API data sending
#define API_ENDPOINT "/api/sensors"
// Wi-Fi timeout in milliseconds.
#define WIFI_TIMEOUT 20000
// Interval for sending data (5 minutes).
#define SEND_INTERVAL 300000
 // PWM pin for coolers.
#define FAN_PIN 3

// --- Global objects ---
// matrix - LED matrix
ArduinoLEDMatrix matrix;
// dht - temperature and humidity sensor
DHT dht(DHTPIN, DHTTYPE);
// lcd - LCD display via I2C
LCD_Display lcd(0x27, 16, 2);
// wifiManager - Wi-Fi management
WifiManager wifiManager(SECRET_SSID, SECRET_PASS);
// httpClient - HTTP client for data sending
WiFiClient wifiClient;
HttpClient httpClient(wifiClient, SECRET_SRV_ADDR, 80);
// tc - temperature and actuator control
TemperatureControl tc(
  // RPWM - cooling control
  /*coolPWM*/ 9, 
  // LPWM - heating control
  /*heatPWM*/ 10,
  // Nano I2C address
  NANO_ADDR
);

// --- setup() ---
// Initializes I2C, turns off fans via Nano, sets up temperature controller,
// starts Serial, LCD, Wi-Fi, DHT11 sensor, LED matrix. Sends first data to server.
String lcdLine1 = "";
String lcdLine2 = "";
unsigned long lastSendTime = 0;

void setup() {
  Wire.begin();
  delay(500);

  Wire.beginTransmission(NANO_ADDR);
  // Fan ID 0 - air intake fan.
  Wire.write(0);
  // PWM 0 - off.
  Wire.write(0);
  Wire.endTransmission();

  Wire.beginTransmission(NANO_ADDR);
  // Fan ID 1 - radiator fans.
  Wire.write(1);
  // PWM 0 - off.
  Wire.write(0);
  Wire.endTransmission();

  // Temperature setup.
  tc.begin();
  // Ensure all systems are off at startup.
  tc.offAll();
  tc.setTargetTemperature(24);
  tc.setNanoFanSpeed(255);


  // Everything else.
  Serial.begin(9600);
  delay(2000);
  Serial.println("Booting...");

  lcd.init();
  lcdLog("Booting...");

  pinMode(FAN_PIN, OUTPUT);
  // digitalWrite(FAN_PIN, LOW);

  Serial.println("Initializing system...");
  lcdLog("Init system...");
  lcdLog("Checking Wi-Fi...");
  wifiManager.begin();

  unsigned long startAttemptTime = millis();
  while (!wifiManager.isConnected() && millis() - startAttemptTime < WIFI_TIMEOUT) {
    delay(1000);
  }

  if (wifiManager.isConnected()) {
    lcdLog("Wi-Fi: Connected");
  } else {
    lcdLog("Wi-Fi: Offline");
  }

  lcdLog("Init DHT11...");
  Serial.println("Initializing DHT11 sensor...");
  dht.begin();

  lcdLog("Init LED Matrix...");
  Serial.println("Initializing LED Matrix...");
  matrix.begin();

  lcdLog("System ready!");
  delay(2000);
  lcd.clear();
  lcdLog("Sending data!");
  sendSensorData();
}

/**
 * Main loop.
 * Checks Wi-Fi, updates data on display and LED matrix, sends data to server, 
 * updates temperature controller state, 2-second delay.
 */
void loop() {
  if (!wifiManager.isConnected()) {
    lcdLog("Wi-Fi lost!");
    lcdLog("Reconnecting...");
    wifiManager.reconnect();

    if (wifiManager.isConnected()) {
      lcdLog("Wi-Fi: Connected");
    } else {
      lcdLog("Wi-Fi: Offline");
    }
  }

  printSensorData();

  // Send data to server every 5 minutes.
  if (millis() - lastSendTime >= SEND_INTERVAL) {
    sendSensorData();
    lastSendTime = millis();
  }

  float currentTemp = dht.readTemperature();
  if (!isnan(currentTemp)) {
    tc.setCurrentTemperature(currentTemp);
    tc.handle();
  }

  // tc.forceCooling();
  delay(2000);
}

/**
 * Displays the last two log lines on the LCD display.
 */
void lcdLog(const String& message) {
  lcdLine1 = lcdLine2;
  lcdLine2 = message;
  lcd.clear();
  lcd.printText(lcdLine1.c_str(), 0, 0);
  lcd.printText(lcdLine2.c_str(), 0, 1);
}

/**
 * Reads and prints sensor data to LCD and updates LED matrix.
 */
void printSensorData() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  String text = String(temperature) + " " + String(humidity) + "%";

  lcd.clear();
  lcd.printText(("Temp.: " + String(temperature) + "C").c_str(), 0, 0);
  lcd.printText(("Hum.: " + String(humidity) + "%").c_str(), 0, 1);

  if (wifiManager.isConnected()) {
    Serial.println("Updating LED Matrix...");
    matrix.beginDraw();
    matrix.stroke(0xFFFFFFFF);
    matrix.textScrollSpeed(100);
    matrix.textFont(Font_5x7);
    matrix.beginText(0, 1, 0xFFFFFF);
    matrix.println(text);
    matrix.endText(SCROLL_LEFT);
    matrix.endDraw();
  } else {
    Serial.println("Offline mode: LED Matrix update skipped.");
  }
}

/**
 * Sends temperature, humidity, and system state data to the server via HTTP POST.
 */
void sendSensorData() {
  Serial.println(">>> sendSensorData");
  if (!wifiManager.isConnected()) {
    Serial.println("❌ Wifi not connected!");
    return;
  }

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("❌ Error reading data fron sensor DHT11");
    return;
  }

  bool isHeating = tc.isHeating();
  bool isCooling = tc.isCooling();
  bool isIdle = tc.isIdle();

  Serial.println("📡 Sending data to server");

  WiFiClient client;
  if (!client.connect(SECRET_SRV_ADDR, 80)) {
    Serial.println("❌ Error connecting to server!");
    return;
  }

  String postData = "{\"username\": \"arduino\", \"password\": \"encore\", ";
  postData += "\"temperature\": " + String(temperature) + ", ";
  postData += "\"humidity\": " + String(humidity) + ", ";
  postData += "\"heating\": " + String(isHeating ? "true" : "false") + ", ";
  postData += "\"cooling\": " + String(isCooling ? "true" : "false") + ", ";
  postData += "\"idle\": " + String(isIdle ? "true" : "false") + "}";

  Serial.print("📡 Request: ");
  Serial.println(postData);

  client.println("POST " + String(API_ENDPOINT) + " HTTP/1.1");
  client.println("Host: " + String(SECRET_SRV_ADDR));
  client.println("Content-Type: application/json");
  client.println("Accept: application/json");
  client.print("Content-Length: ");
  client.println(postData.length());
  client.println();
  client.println(postData);

  delay(500);

  Serial.println("📩 Server response:");
  while (client.available()) {
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
  Serial.println();
  client.stop();
}
