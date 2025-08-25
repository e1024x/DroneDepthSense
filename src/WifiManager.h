// WiFiManager.h
#pragma once

#include <WiFi.h>

class WiFiManager {
private:
    const char* ssid;
    const char* password;
    uint16_t port;
    WiFiServer server;
    WiFiClient client;
    
public:
    WiFiManager(const char* ssid, const char* password, uint16_t port);
    
    bool initialize();
    void handleClient();
    WiFiClient getClient();
    
private:
    void setupAccessPoint();
    void startServer();
};