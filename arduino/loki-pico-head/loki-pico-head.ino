// https://deepbluembedded.com/arduino-i2c-slave/
// https://arduino-pico.readthedocs.io/en/latest/wire.html
// https://randomnerdtutorials.com/programming-raspberry-pi-pico-w-arduino-ide/
// https://github.com/earlephilhower/arduino-pico/tree/master/libraries/Servo/src
#include <Wire.h>
#include <Servo.h>
#include "robot_config.h"

// TODO fan speed
// TODO port touch
// TODO servo speed
//   servo torque? PWM tristate
//   servo range
//   servo auto-calibration?
// TODO I2C fan, touch, servo
// TODO INT touch

Servo leftServo;
Servo rightServo;
Servo extServo;
Servo fan;

void I2C_RxHandler(int numBytes) {
  while(Wire.available()) {  // Read Any Received Data
    byte RxByte = Wire.read();
    Serial.print(RxByte);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FAN_EN_PIN, OUTPUT);

  Serial.begin(115200);
  delay(2000);
  
  Serial.println("Start setup()");
  if (!Wire.setSDA(I2C_SDA_HOST_PIN))
    Serial.println("SDA pin config failed");
  if (!Wire.setSCL(I2C_SCL_HOST_PIN))
    Serial.println("SCL pin config failed");
  
  Wire.begin(0x55); // Initialize I2C (Slave Mode: address=0x55 )
  Wire.onReceive(I2C_RxHandler);

  leftServo.attach(13);
  rightServo.attach(14);
  extServo.attach(15);
  analogReadResolution(12);

  Serial.println("Done setup()");
}

void loop() {
//  byte fan_pwm = 0;

  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(FAN_EN_PIN, HIGH);
//  leftServo.write(0);
//  rightServo.write(90);
//  extServo.write(90);
    delay(1500);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(FAN_EN_PIN, LOW);
  int base = analogRead(ADC3_PIN);
  Serial.print(base);
  Serial.print(" ");

//  leftServo.write(70);
//  analogWrite(FAN_EN_PIN, fan_pwm);
//  fan_pwm += 32;

/*
  for (int i = 0; i < 500; i++) {
    int v = analogRead(ADC3_PIN);
    int delta = v - base;
    if (abs(delta) > 15) {
      Serial.print(i);
      Serial.print(":");
      Serial.print(delta);
      Serial.print(" ");
    }
    delay(1);
  }
*/
  delay(1500);
  Serial.println();

//float analogReadTemp(float vref = 3.3f)
}
