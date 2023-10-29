#include "util.h"

float absMin(float a, float b_abs) {
  float a_abs = abs(a);
  float min_abs = min(a_abs, b_abs);

  return a >= 0 ? min_abs : -min_abs;
}

void enableLdsMotor(bool enable) {
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
  digitalWrite(LED_PIN, LOW);
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

void delayYield(unsigned long msec) {
    unsigned long time_msec = millis();
    while(millis() - time_msec < msec)
      yield();
}

String reset_reason_to_string(int reason, bool verbose) {
  if (verbose) {
    switch ( reason)
    {
      case 1  : return ("Vbat power on reset");
      case 3  : return ("Software reset digital core");
      case 4  : return ("Legacy watch dog reset digital core");
      case 5  : return ("Deep Sleep reset digital core");
      case 6  : return ("Reset by SLC module, reset digital core");
      case 7  : return ("Timer Group0 Watch dog reset digital core");
      case 8  : return ("Timer Group1 Watch dog reset digital core");
      case 9  : return ("RTC Watch dog Reset digital core");
      case 10 : return ("Instrusion tested to reset CPU");
      case 11 : return ("Time Group reset CPU");
      case 12 : return ("Software reset CPU");
      case 13 : return ("RTC Watch dog Reset CPU");
      case 14 : return ("for APP CPU, reseted by PRO CPU");
      case 15 : return ("Reset when the vdd voltage is not stable");
      case 16 : return ("RTC Watch dog reset digital core and rtc module");
      default : return ("NO_MEAN");
    }
  } else {
    switch ( reason)
    {
      case 1 : return ("POWERON_RESET");
      case 3 : return ("SW_RESET");
      case 4 : return ("OWDT_RESET");
      case 5 : return ("DEEPSLEEP_RESET");
      case 6 : return ("SDIO_RESET");
      case 7 : return ("TG0WDT_SYS_RESET");
      case 8 : return ("TG1WDT_SYS_RESET");
      case 9 : return ("RTCWDT_SYS_RESET");
      case 10 : return ("INTRUSION_RESET");
      case 11 : return ("TGWDT_CPU_RESET");
      case 12 : return ("SW_CPU_RESET");
      case 13 : return ("RTCWDT_CPU_RESET");
      case 14 : return ("EXT_CPU_RESET");
      case 15 : return ("RTCWDT_BROWN_OUT_RESET");
      case 16 : return ("RTCWDT_RTC_RESET");
      default : return ("NO_MEAN");
    }
  }
}

String micro_ros_error_string(int err) {
  String errCode = String(err);
  switch (err)
  {
    case 0 : return ("RMW_RET_OK");
    case 1 : return ("RMW_RET_ERROR");
    case 2 : return ("RMW_RET_TIMEOUT");
    case 23 : return ("RMW_GID_STORAGE_SIZE");
    default : return errCode;
  }
}
