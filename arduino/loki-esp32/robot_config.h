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

// Choose your LDS
//#define LDS_YDLIDAR_X4
#define LDS_LDS02RR

// Motors config
#define WHEEL_DIA (2*33.5e-3)      // meters
#define WHEEL_BASE (159.063*1e-3)  // wheel base, meters
#define MAX_WHEEL_ACCEL 2.0        // wheel vs floor m2/sec

#define LDS_MOTOR_SPEED_DEFAULT -1 // tristate YDLidar X4 SCTP pin for default motor speed
#define LDS_MOTOR_PWM_CHANNEL    2 // ESP32 PWM channel for LDS motor speed control

class CONFIG {
public:
  // ESP32 pin assignment
  static const uint8_t LED_PIN = 2; // ESP32 on-board LED
  static const uint8_t LDS_MOTOR_PWM_PIN = 15; // LDS motor speed control using PWM
  static const uint8_t LDS_MOTOR_EN_PIN = 19; // LDS motor enable pin (was 12)
  static const uint8_t BAT_ADC_PIN = 36;

  static const uint8_t RESET_SETTINGS_HOLD_SEC = 10; // Hold BOOT button to reset WiFi
};

// Micro-ROS config
#define UROS_CLIENT_KEY 0xCA1AA100
#define UROS_TELEM_TOPIC_NAME "telemetry"
#define UROS_LOG_TOPIC_NAME "rosout"
#define UROS_CMD_VEL_TOPIC_NAME "cmd_vel"
#define UROS_ROBOT_MODEL "MAKERSPET_LOKI"
#define UROS_NODE_NAME UROS_ROBOT_MODEL
#define UROS_PING_PUB_PERIOD_MS 10000
#define UROS_TELEM_PUB_PERIOD_MS 50
#define UROS_TIME_SYNC_TIMEOUT_MS 1000
#define UROS_PARAM_LDS_MOTOR_SPEED "lds.motor_speed"

#define LDS_BUF_LEN     400
#define LDS_MOTOR_PWM_FREQ    10000
#define LDS_MOTOR_PWM_BITS    11 // was 8
#define JOINTS_LEN (MOTOR_COUNT)

// WiFi config
#define WIFI_CONN_TIMEOUT_SEC 30

// ESP32 blinks when firmware init fails
#define ERR_WIFI_CONN 1
#define ERR_LDS_START 2
#define ERR_UROS_AGENT_CONN 3
#define ERR_WIFI_LOST 4
#define ERR_UROS_INIT 5
#define ERR_UROS_NODE 6
#define ERR_UROS_PUBSUB 7
#define ERR_UROS_EXEC 8
#define ERR_UROS_TIME_SYNC 9
#define ERR_UROS_SPIN 10
#define ERR_UROS_PARAM 11
#define ERR_SPIFFS_INIT 12

#define ERR_REBOOT_BLINK_CYCLES 3 // Blinki out an error a few times, then reboot
#define LONG_BLINK_MS 1000
#define LONG_BLINK_PAUSE_MS 2000
#define SHORT_BLINK_MS 200
#define SHORT_BLINK_PAUSE_MS 500

#define TELEM_PUB_PERIOD_MS 50
#define SPIN_TELEM_STATS 100 // Undefine for debug
#define PING_PUB_PERIOD_MS 10000

#if !defined(ESP32)
  #error This code builds on ESP32 Dev module only
#endif
