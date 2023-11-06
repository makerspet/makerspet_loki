#ifndef KAIA_ESP32_UTIL
#define KAIA_ESP32_UTIL

#include "arduino.h"
#include "robot_config.h"

const float WHEEL_PERIM_LEN_DIV60 = PI * WHEEL_DIA / 60;
const float WHEEL_PERIM_LEN_DIV60_RECIP = 1/WHEEL_PERIM_LEN_DIV60;
#define SPEED_TO_RPM(SPEED_MS) (SPEED_MS*WHEEL_PERIM_LEN_DIV60_RECIP)
#define RPM_TO_SPEED(RPM) (RPM*WHEEL_PERIM_LEN_DIV60)


void enableLdsMotor(bool enable);
void blink(unsigned int delay_ms, unsigned int count);
float absMin(float a, float b_abs);
void printCurrentTime();
void setMotorSpeed(int16_t speed);
void twistToWheelSpeeds(float speed_lin_x, float speed_ang_z,
  float *speed_right, float *speed_left);
void delayYield(unsigned long msec);
String micro_ros_error_string(int err);

String reset_reason_to_string(int reason, bool verbose=false);

#endif  // KAIA_ESP32_UTIL
