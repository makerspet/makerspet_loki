#ifndef KAIA_ESP32_UTIL
#define KAIA_ESP32_UTIL

#include "arduino.h"
#include "kaia-esp32.h"

const float WHEEL_PERIM_LEN_DIV60 = PI * WHEEL_DIA / 60;
const float WHEEL_PERIM_LEN_DIV60_RECIP = 1/WHEEL_PERIM_LEN_DIV60;
#define SPEED_TO_RPM(SPEED_MS) (SPEED_MS*WHEEL_PERIM_LEN_DIV60_RECIP)
#define RPM_TO_SPEED(RPM) (RPM*WHEEL_PERIM_LEN_DIV60)


void enableMotor(bool enable);
void blink(unsigned int delay_ms, unsigned int count);
float absMin(float a, float b_abs);
void printCurrentTime();
void setMotorSpeed(int16_t speed);
void twistToWheelSpeeds(float speed_lin_x, float speed_ang_z,
  float *speed_right, float *speed_left);

#endif  // KAIA_ESP32_UTIL
