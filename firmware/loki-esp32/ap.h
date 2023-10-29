#ifndef KAIA_AP_H_
#define KAIA_AP_H_

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebSrv.h>
#include <AsyncTCP.h>
#include "SPIFFS.h"

#define SSID_AP "KAIAAI-WIFI"

// Search for parameter in HTTP POST request
#define PARAM_INPUT_SSID "ssid"
#define PARAM_INPUT_PASS "pass"
#define PARAM_INPUT_DEST_IP "dest_ip"
#define PARAM_INPUT_DEST_PORT "dest_port"

void initSPIFFS();
void ObtainWiFiCreds(void (*callback)());
String getSSID();
String getPassw();
String getDestIP();
String getDestPort();

//String readFile(fs::FS &fs, const char * path);
//void writeFile(fs::FS &fs, const char * path, const char * message);
void resetWiFiSettings();

#endif
