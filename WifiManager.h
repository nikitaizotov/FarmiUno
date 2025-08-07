#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFiS3.h>

/**
 * Class for managing Wi-Fi connection.
 *
 * Allows you to connect to a network, reconnect, check status, and print network info.
 */
class WifiManager {
public:
    /**
     * Constructor for WifiManager.
     *
     * @param ssid Wi-Fi network name
     * @param password Wi-Fi password
     */
    WifiManager(const char* ssid, const char* password);

    /**
     * Initializes Wi-Fi connection.
     *
     * Call this in setup() to connect to the specified Wi-Fi network.
     */
    void begin();

    /**
     * Reconnects to Wi-Fi if connection is lost.
     *
     * Use this in loop() to maintain a stable connection.
     */
    void reconnect();

    /**
     * Checks connection status.
     *
     * @return true if connected, false otherwise.
     */
    bool isConnected();  

private:
    const char* _ssid;      /**< Wi-Fi network name */
    const char* _password;  /**< Wi-Fi password */
    int _status;            /**< Connection status */
    bool _connected;        /**< Connection flag */
    /**
     * Prints current network info to Serial.
     */
    void printCurrentNet();
    /**
     * Prints Wi-Fi module info to Serial.
     */
    void printWifiData();
    /**
     * Prints MAC address to Serial.
     */
    void printMacAddress(byte mac[]);
};

#endif // WIFI_MANAGER_H
