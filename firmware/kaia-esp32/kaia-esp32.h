#ifndef KAIA_ESP32
#define KAIA_ESP32

// ESP32 pin assignment
#define LED_PIN 2
#define YD_MOTOR_SCTP_PIN 15 // 2 PWM pin for control the speed of YDLIDAR's motor. 
#define YD_MOTOR_EN_PIN   12 // 15 The ENABLE PIN for YDLIDAR's motor                  
#define YD_MOTOR_SPEED 255  // LEAVE DISCONNECTED
#define YD_MOTOR_SPEED_DEFAULT -1  // disconnect SCTP pin for default motor speed
#define YD_MOTOR_SCTP_PWM_CHANNEL 0  // ESP32 PWM channel

// Micro-ROS config
#define MICRO_ROS_AGENT_PORT 8888
#define MICRO_ROS_AGENT_IP "192.168.1.112" // "192.168.226.157"
#define UROS_CLIENT_KEY 0xCA1A0000
#define UROS_TELEM_TOPIC_NAME "telemetry"
#define UROS_LOG_TOPIC_NAME "rosout"
#define UROS_CMD_VEL_TOPIC_NAME "cmd_vel"
#define UROS_NODE_NAME "kaia_esp32_arduino_microros"
#define JOINTS_LEN (MOTOR_COUNT)
#define LDS_BUF_LEN 400

// WiFi config
#define WIFI_CONN_TIMEOUT_MS 10000

// Motors config
#define WHEEL_DIA (2*33.5e-3) // meters
#define WHEEL_BASE ((291.794-29)*1e-3) // wheel base, meters
#define MAX_WHEEL_ACCEL 1.0   // wheel vs floor m2/sec

// ESP32 blinks when firmware init fails
#define ERR_WIFI_CONN 1
#define ERR_UROS_INIT 2
#define ERR_UROS_AGENT_CONN 3
#define ERR_UROS_TIME_SYNC 4
#define ERR_UROS_NODE 5
#define ERR_UROS_PUBSUB 6
#define ERR_UROS_EXEC 7
#define ERR_UROS_SPIN 8
#define ERR_LDS_INIT  9

#if !defined(ESP32)
  #error This code builds on ESP32 Dev module only
#endif

void serial_callback(char c);
void scan_callback(uint8_t quality, float angle_deg,
  float distance_mm, bool startBit);

#endif  // KAIA_ESP32
