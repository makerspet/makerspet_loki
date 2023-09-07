#ifndef KAIA_AP_H_
#define KAIA_AP_H_

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebSrv.h>
#include <AsyncTCP.h>
#include "SPIFFS.h"

#define SSID_AP "KAIA-WIFI"

void initSPIFFS();
void ObtainWiFiCreds();
String getSSID();
String getPassw();
String getIP();
String getGateway();

#endif
