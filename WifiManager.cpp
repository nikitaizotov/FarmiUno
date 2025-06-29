#include "WifiManager.h"
#include <Arduino.h>
#include "arduino_secrets.h"

WifiManager::WifiManager(const char* ssid, const char* password)
  : _ssid(ssid), _password(password), _status(WL_IDLE_STATUS), _connected(false)
{
}

void WifiManager::begin() {
  Serial.println("Checking Wi-Fi module...");
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Error: Wi-Fi module not found!");
    return;
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware.");
  }

  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(_ssid);

  unsigned long startAttemptTime = millis();
  while (_status != WL_CONNECTED && millis() - startAttemptTime < 30000) {  // 30 сек
    _status = WiFi.begin(_ssid, _password);
    Serial.print(".");
    delay(5000);
  }

  if (_status == WL_CONNECTED) {
    _connected = true;
    Serial.println("\nConnected to Wi-Fi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    printCurrentNet();
    printWifiData();
  } else {
    _connected = false;
    Serial.println("\nWi-Fi connection failed. Running in offline mode.");
  }
}

void WifiManager::reconnect() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi lost, attempting reconnection...");
    _connected = false;
    
    unsigned long startAttemptTime = millis();
    while (_status != WL_CONNECTED && millis() - startAttemptTime < 30000) {  // 30 сек
      _status = WiFi.begin(_ssid, _password);
      Serial.print(".");
      delay(5000);
    }

    if (_status == WL_CONNECTED) {
      _connected = true;
      Serial.println("\nReconnected to Wi-Fi!");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("\nReconnection failed.");
    }
  }
}

bool WifiManager::isConnected() {
  return _connected;
}

void WifiManager::printCurrentNet() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  long rssi = WiFi.RSSI();
  Serial.print("Signal Strength (RSSI): ");
  Serial.println(rssi);

  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type: ");
  Serial.println(encryption, HEX);
  Serial.println();
}

void WifiManager::printWifiData() {
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC Address: ");
  printMacAddress(mac);
}

void WifiManager::printMacAddress(byte mac[]) {
  for (int i = 0; i < 6; i++) {
    if (i > 0) {
      Serial.print(":");
    }
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
  }
  Serial.println();
}
