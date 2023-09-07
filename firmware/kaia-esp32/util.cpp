#include "util.h"

float absMin(float a, float b_abs) {
  float a_abs = abs(a);
  float min_abs = min(a_abs, b_abs);

  return a >= 0 ? min_abs : -min_abs;
}

void enableMotor(bool enable) {
  int current_state = digitalRead(YD_MOTOR_EN_PIN);
  int new_state = enable ? HIGH : LOW;
  if (current_state != new_state) {
    digitalWrite(YD_MOTOR_EN_PIN, new_state);
    Serial.print(F("Motor "));
    Serial.println(enable ? F("enabled") : F("disabled"));
  }
}

void blink(unsigned int delay_ms, unsigned int count) {
  for (unsigned int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(delay_ms);
    digitalWrite(LED_PIN, HIGH);
    delay(delay_ms);
  }
}

void printCurrentTime() {
  // prints time correctly, even after power-up
  char strftime_buf[64];
  time_t now;

  setenv("TZ", "CEST-1CET,M3.2.0/2:00:00,M11.1.0/2:00:00", 1); // for local time
  tzset();
  time(&now);

  struct tm *tt = gmtime(&now);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", tt);
  Serial.print("UTC time ");
  Serial.println(strftime_buf);

  //localtime_r(&now, tt);
  //strftime(strftime_buf, sizeof(strftime_buf), "%c", tt);
  //Serial.print("The current local date/time according to ESP32: ");
  //Serial.println(strftime_buf);
}

void setMotorSpeed(int16_t speed) {
  Serial.print(F("Motor speed "));
  if (speed < 0) {
    pinMode(YD_MOTOR_SCTP_PIN, INPUT);
    Serial.println(F("default"));
    return;
  }

  pinMode(YD_MOTOR_SCTP_PIN, OUTPUT);
  speed = speed > 255 ? 255 : speed;
  ledcWrite(YD_MOTOR_SCTP_PWM_CHANNEL, speed);
  Serial.println(speed);
}

void twistToWheelSpeeds(float speed_lin_x, float speed_ang_z,
  float *speed_right, float *speed_left)
{
  float ang_component = speed_ang_z*WHEEL_BASE*0.5;
  *speed_right = speed_lin_x + ang_component;
  *speed_left  = speed_lin_x - ang_component;
}
