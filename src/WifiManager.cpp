// WiFiManager.cpp
#include "WiFiManager.h"
#include <Arduino.h>

WiFiManager::WiFiManager(const char* ssid, const char* password, uint16_t port)
    : ssid(ssid), password(password), port(port), server(port) {
}

bool WiFiManager::initialize() {
    setupAccessPoint();
    startServer();
    return true;
}

void WiFiManager::setupAccessPoint() {
    Serial.println("Configuring wifi AP");
    WiFi.softAP(ssid, password);
    
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
}

void WiFiManager::startServer() {
    server.begin();
    Serial.println("TCP server started");
}

void WiFiManager::handleClient() {
    if (!client || !client.connected()) {
        client = server.available();
        if (client) {
            Serial.println("New client connected");
        }
    }
}

WiFiClient WiFiManager::getClient() {
    return client;
}