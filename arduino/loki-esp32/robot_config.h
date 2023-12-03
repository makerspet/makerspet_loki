#ifndef ROBOT_CONFIG
#define ROBOT_CONFIG

// Motors config
#define WHEEL_DIA (2*33.5e-3)      // meters
#define WHEEL_BASE (159.063*1e-3)  // wheel base, meters
#define MAX_WHEEL_ACCEL 2.0        // wheel vs floor m2/sec

// ESP32 pin assignment
#define LED_PIN 2
#define LDS_MOTOR_PWM_PIN       15 // LDS motor speed control using PWM
#define LDS_MOTOR_EN_PIN        19 // LDS motor enable pin (was 12)
#define LDS_MOTOR_SPEED_DEFAULT -1 // tristate YDLidar X4 SCTP pin for default motor speed
#define LDS_MOTOR_PWM_CHANNEL    0 // ESP32 PWM channel for LDS motor speed control
#define BAT_ADC_PIN             36

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

#define JOINTS_LEN (MOTOR_COUNT)
#define LDS_BUF_LEN 400
#define LDS_SERIAL_BAUD 128000  // YDLIDAR X4

// WiFi config
#define WIFI_CONN_TIMEOUT_SEC 30
#define RESET_SETTINGS_HOLD_SEC 5 // Hold button this long to reset WiFi

// ESP32 blinks when firmware init fails
#define ERR_WIFI_CONN 1
#define ERR_LDS_INIT 2
#define ERR_UROS_AGENT_CONN 3
#define ERR_WIFI_LOST 4
#define ERR_UROS_INIT 5
#define ERR_UROS_NODE 6
#define ERR_UROS_PUBSUB 7
#define ERR_UROS_EXEC 8
#define ERR_UROS_TIME_SYNC 9
#define ERR_UROS_SPIN 10
#define ERR_UROS_PARAM 11

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

void serial_callback(char c);
void scan_callback(uint8_t quality, float angle_deg,
  float distance_mm, bool startBit);

#endif  // ROBOT_CONFIG
