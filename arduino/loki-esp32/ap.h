// Based on:
//   TODO Arduino library
//
// Copyright 2023-2024 REMAKE.AI, KAIA.AI, MAKERSPET.COM
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebSrv.h>
#include <AsyncTCP.h>
#include "SPIFFS.h"

// Search for parameter in HTTP POST request
#define PARAM_INPUT_SSID "ssid"
#define PARAM_INPUT_PASS "pass"
#define PARAM_INPUT_DEST_IP "dest_ip"
#define PARAM_INPUT_DEST_PORT "dest_port"

bool initSPIFFS();
void ObtainWiFiCreds(void (*callback)(), const char * SSID_AP);
String getSSID();
String getPassw();
String getDestIP();
String getDestPort();

//String readFile(fs::FS &fs, const char * path);
//void writeFile(fs::FS &fs, const char * path, const char * message);
void resetWiFiSettings();
