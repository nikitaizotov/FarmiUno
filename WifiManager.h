#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFiS3.h>

class WifiManager {
public:
  WifiManager(const char* ssid, const char* password);
  void begin();
  void reconnect();
  bool isConnected();  

private:
  const char* _ssid;
  const char* _password;
  int _status;
  bool _connected;
  void printCurrentNet();
  void printWifiData();
  void printMacAddress(byte mac[]);
};

#endif // WIFI_MANAGER_H
