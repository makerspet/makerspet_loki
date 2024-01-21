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

#include "drive.h"

#define DEBUG true
volatile long int encoder[MOTOR_COUNT] = {0, 0};
volatile bool CW[MOTOR_COUNT] = {true, true};

DriveController drive;

void IRAM_ATTR encLeftIsr() {
  if (CW[MOTOR_LEFT] ^ FLIP_ROTATION)
    encoder[MOTOR_LEFT]++;
  else
    encoder[MOTOR_LEFT]--;
}

void IRAM_ATTR encRightIsr() {
  if (CW[MOTOR_RIGHT] ^ FLIP_ROTATION)
    encoder[MOTOR_RIGHT]++;
  else
    encoder[MOTOR_RIGHT]--;
}

float DriveController::getShaftAngle(unsigned char motorID) {
  if (motorID < MOTOR_COUNT)
    return TWO_PI * encoder[motorID] / encoderTPR[motorID];
  return 0;
}

void DriveController::setPIDUpdatePeriod(unsigned char motorID, float period) {
  if (motorID >= MOTOR_COUNT)
    return;

  // same period for all motors for now
  pidUpdatePeriodUs = (unsigned int) round(period * 1e6);
//  pid[motorID]->SetReferenceSampleTime(period);  // unnecessary in PID library
}

void DriveController::enablePID(unsigned char motorID, bool en) {
  if (motorID < MOTOR_COUNT)
    pid[motorID]->enable(en);
}

void DriveController::setMaxRPM(unsigned char motorID, float rpm) {
  if (motorID >= MOTOR_COUNT || rpm <= 0)
    return;
  maxRPM[motorID] = rpm;
}

void DriveController::setEncoderTPR(unsigned char motorID, float tpr) {
  if (motorID >= MOTOR_COUNT || tpr <= 0)
    return;
  encoderTPR[motorID] = tpr;
  ticksPerMicroSecToRPM[motorID] = 1e6 * 60.0 / tpr;
}

void DriveController::setKp(unsigned char motorID, float k) {
  if (motorID < MOTOR_COUNT) {
    kp[motorID] = k;
    pid[motorID]->SetTunings(kp[motorID], ki[motorID], kd[motorID]);
  }
}

void DriveController::setKi(unsigned char motorID, float k) {
  if (motorID < MOTOR_COUNT) {
    ki[motorID] = k;

    pid[motorID]->SetTunings(kp[motorID], ki[motorID], kd[motorID]);
  }
}

void DriveController::setKd(unsigned char motorID, float k) {
  if (motorID < MOTOR_COUNT) {
    kd[motorID] = k;
    pid[motorID]->SetTunings(kp[motorID], ki[motorID], kd[motorID]);
  }
}

void DriveController::setProportionalMode(unsigned char motorID, bool onMeasurement) {
  if (motorID < MOTOR_COUNT)
    pid[motorID]->SetTunings(kp[motorID], ki[motorID], kd[motorID], onMeasurement ? PID::P_ON_M : PID::P_ON_E);
}

void DriveController::initOnce(logFuncT logFunc) {
  logDebug = logFunc;
  tickSampleTimePrev = 0;

  // Encoders
  pinMode(MOT_FG_LEFT_PIN, INPUT); // INPUT_PULLUP
  attachInterrupt(MOT_FG_LEFT_PIN, encLeftIsr, CHANGE);

  pinMode(MOT_FG_RIGHT_PIN, INPUT);
  attachInterrupt(MOT_FG_RIGHT_PIN, encRightIsr, CHANGE);

  for (unsigned char motorID = 0; motorID < MOTOR_COUNT; motorID++) {
    pinMode(cwPin[motorID], OUTPUT);
    targetRPM[motorID] = 0;
    measuredRPM[motorID] = 0;
    pidPWM[motorID] = 0;
    encPrev[motorID] = 0;
    //brakingEnabled[motorID] = false;
    setPointHasChanged[motorID] = false;
    switchingCw[motorID] = false;
    kp[motorID] = PID_KP_WHEEL;
    ki[motorID] = PID_KI_WHEEL;
    kd[motorID] = PID_KD_WHEEL;
    pwmFreq[motorID] = 1; // force update
    setPWMFreq(motorID, PWM_FREQ);
    
    // https://playground.arduino.cc/Code/PIDLibrary/
    pid[motorID] = new PID(&measuredRPM[motorID], &pidPWM[motorID], &targetRPM[motorID],
      kp[motorID], ki[motorID], kd[motorID], PID_UPDATE_PERIOD, PID_MODE, PID::DIRECT);
    pid[motorID]->SetOutputLimits(-1, 1);

    setMaxRPM(motorID, MOTOR_WHEEL_MAX_RPM);
    setEncoderTPR(motorID, WHEEL_ENCODER_TPR);
    setPIDUpdatePeriod(motorID, PID_UPDATE_PERIOD);
    enablePID(motorID, true);

    PWM[motorID] = 1; // force update
    setPWM(motorID, 0);
    ledcAttachPin(pwmPin[motorID], motorID);
  }
}

bool DriveController::setRPM(unsigned char motorID, float rpm) {
  if (motorID >= MOTOR_COUNT) // TODO print log error
    return false;

  if (targetRPM[motorID] == rpm)
    return false;

  targetRPM[motorID] = rpm;
  setPointHasChanged[motorID] = true;
  return true;
}

void DriveController::setPWMFreq(unsigned char motorID, unsigned short int freq) {
  if (motorID >= MOTOR_COUNT || pwmFreq[motorID] == freq)
    return;

  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  //ledcSetup(pwmMotChannel[motorID], freq, PWM_BITS);
  ledcSetup(motorID, freq, PWM_BITS);

  pwmFreq[motorID] = freq;
}

void DriveController::setPWM(unsigned char motorID, float value) {
  if (motorID >= MOTOR_COUNT)
    return;

  if (switchingCw[motorID])
    return;
  
  value = min(value, 1.0f);  // clamp to range
  value = max(value, -1.0f);
  
  int pwm = (int) round(value * PWM_MAX);

  if (pwm == PWM[motorID])
    return;

  bool cw = (pwm >= 0) ^ FLIP_ROTATION;

  int prevPWM = PWM[motorID];
  if ((prevPWM > 0 && pwm < 0) || (prevPWM < 0 && pwm > 0)) {
    // when cw/ccw changes, stop pwm:=0, verify 0 enc pulses
    // then flip encoder inc/dec, proceed
    switchingCw[motorID] = true;
    pwm = 0;
    cw = CW[motorID]; // freeze cw for encoders until we stop
  }

  int pwmValue = PWM_MAX - abs(pwm); //(pwm >= 0 ? pwm : -pwm);
  ledcWrite(motorID, pwmValue);
  digitalWrite (cwPin[motorID], cw ? LOW : HIGH);

  CW[motorID] = cw;
  PWM[motorID] = pwm;

  //char logMsg[100];
  //sprintf(logMsg, "Motor %d PWM %d CW %d", motorID, pwm, cw);
  //logDebug(logMsg);
}

void DriveController::resetEncoders() {
  for (unsigned char motorID = 0; motorID < MOTOR_COUNT; motorID++)
    encoder[motorID] = 0;
}

DriveController::DriveController() {
  pwmPin[MOTOR_LEFT] = MOT_PWM_LEFT_PIN;
  pwmPin[MOTOR_RIGHT] = MOT_PWM_RIGHT_PIN;

  cwPin[MOTOR_LEFT] = MOT_CW_LEFT_PIN;
  cwPin[MOTOR_RIGHT] = MOT_CW_RIGHT_PIN;
}

// TODO detect stuck (stalled) motor, limit current, let robot know
void DriveController::update() {
  bool anySetPointHasChanged = false;
  for (unsigned char motorID = 0; motorID < MOTOR_COUNT; motorID++)
    anySetPointHasChanged |= setPointHasChanged[motorID];
  
  unsigned long tickTime = esp_timer_get_time();
  unsigned long tickTimeDelta = tickTime - tickSampleTimePrev;
  if ((tickTimeDelta < pidUpdatePeriodUs) && !anySetPointHasChanged)
    return;

  tickSampleTimePrev = tickTime;
  tickSampleTimeDelta = tickTimeDelta;
  
  // TODO fix threads-and-iterrupts race condition
  // TODO move into ESP32 timer? or disable interrupts?
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_timer.html
  unsigned char motorID;
  for (motorID = 0; motorID < MOTOR_COUNT; motorID++) {
    long int encNow = encoder[motorID];
    encDelta[motorID] = encNow - encPrev[motorID];    
    encPrev[motorID] = encNow;
    double ticksPerMicroSec = ((double) encDelta[motorID]) / ((double) tickSampleTimeDelta);
    measuredRPM[motorID] = ticksPerMicroSec * ticksPerMicroSecToRPM[motorID];

//    if (brakingEnabled[motorID] && targetRPM[motorID] == 0)

// TODO maybe uncomment
//    if (targetRPM[motorID] == 0)
//        pid[motorID]->clearErrorIntegral();
    setPointHasChanged[motorID] = false;

    if (encDelta[motorID] == 0)
      switchingCw[motorID] = false;
  }

  if (targetRPM[MOTOR_LEFT] == 0 && measuredRPM[MOTOR_LEFT] == 0 &&
      targetRPM[MOTOR_RIGHT] == 0 && measuredRPM[MOTOR_RIGHT] == 0) {
      
      // Prevent wheels from twitching or slowly turning after stop
      pid[MOTOR_LEFT]->clearErrorIntegral();
      pid[MOTOR_RIGHT]->clearErrorIntegral();
  }
  
  double sampleTime = tickSampleTimeDelta * 1e-6;
  for (motorID = 0; motorID < MOTOR_COUNT; motorID++) {
    pid[motorID]->Compute(sampleTime);

    // Brushless motor hard-brakes when PWM == 0
//    if (brakingEnabled[motorID] && targetRPM[motorID] == 0)

// TODO maybe uncomment
//    if (targetRPM[motorID] == 0)
//        pidPWM[motorID] = 0;

    setPWM(motorID, (float) pidPWM[motorID]);
  }
}

double DriveController::getCurrentRPM(unsigned char motorID) {
  if (motorID >= MOTOR_COUNT)
    return 0;
  return measuredRPM[motorID];
}

double DriveController::getTargetRPM(unsigned char motorID) {
  if (motorID >= MOTOR_COUNT)
    return 0;
  return targetRPM[motorID];
}

float DriveController::getCurrentPWM(unsigned char motorID) {
  if (motorID >= MOTOR_COUNT)
    return 0;
  return float(PWM[motorID]) / PWM_MAX;
}

float DriveController::getMaxRPM(unsigned char motorID) {
  if (motorID >= MOTOR_COUNT)
    return 0;
  return maxRPM[motorID];
}
