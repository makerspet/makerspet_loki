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

#include <math.h>
#include <Arduino.h>
#include <PID_Timed.h>

#define PID_UPDATE_PERIOD          0.03 // default in seconds
#define PID_KP_WHEEL               0.001 // 0.003 P_ON_E
#define PID_KI_WHEEL               0.001 // 0.0002
#define PID_KD_WHEEL               0
#define PID_MODE                   (PID::P_ON_M) // P_ON_E

#define PWM_FREQ                   20000 // 15..25KHz
#define PWM_BITS                   10
#define PWM_MAX                    (1<<10)  // 1024

#define MOTOR_LEFT                 0
#define MOTOR_RIGHT                1
#define MOTOR_COUNT                (MOTOR_RIGHT+1)

#define MOT_PWM_LEFT_PIN           33
#define MOT_CW_LEFT_PIN            23 // was 32
#define MOT_FG_LEFT_PIN            34

#define MOT_PWM_RIGHT_PIN          13 
#define MOT_CW_RIGHT_PIN           25
#define MOT_FG_RIGHT_PIN           35 // was 27


// Chihai Motor CHR-GM25-BL2418 24V 200RPM max
#define MOTOR_MAX_RPM              200  // no-load
#define MOTOR_RATED_RPM            145
#define MOTOR_GEAR_RATIO           45.0 // gearbox reduction ratio
#define MOTOR_ENCODER_PPR          6    // pulses per revolution

// Chihai Motor CHR-GM25-BL2418 24V 260RPM max
//#define MOTOR_MAX_RPM              200  // no-load
//#define MOTOR_RATED_RPM            190
//#define MOTOR_GEAR_RATIO           34.0 // gearbox reduction ratio
//#define MOTOR_ENCODER_PPR          6    // pulses per revolution

// Chihai Motor CHR-GM25-BL2418 24V 450 RPM max
//#define MOTOR_MAX_RPM              450  // no-load
//#define MOTOR_RATED_RPM            325
//#define MOTOR_GEAR_RATIO           20.0 // gearbox reduction ratio
//#define MOTOR_ENCODER_PPR          6    // pulses per revolution

// Far Along JGA25-BL2418 24V 245RPM max
//#define MOTOR_MAX_RPM              245  // no-load
//#define MOTOR_RATED_RPM            185
//#define MOTOR_GEAR_RATIO           35.0 // gearbox reduction ratio
//#define MOTOR_ENCODER_PPR          18   // pulses per revolution

// AliExpress China motor store JGA25-BL2418 24V 408RPM max
//#define MOTOR_MAX_RPM              408  // no-load
//#define MOTOR_RATED_RPM            308
//#define MOTOR_GEAR_RATIO           21.3 // gearbox reduction ratio
//#define MOTOR_ENCODER_PPR          6    // pulses per revolution; TODO check


#define MOTOR_WHEEL_MAX_RPM        (0.9*MOTOR_MAX_RPM)
#define WHEEL_ENCODER_TPR          (MOTOR_GEAR_RATIO*MOTOR_ENCODER_PPR*2)
                                   // ticks per revolution, 2 edges per pulse
#define FLIP_ROTATION              true

typedef void (*logFuncT)(char*);

class DriveController
{
  public:
    DriveController();
    void initOnce(logFuncT logFunc);
    bool setRPM(unsigned char motorID, float rpm);
    void resetEncoders();
    void update();
    float getShaftAngle(unsigned char motorID);
    void setPIDUpdatePeriod(unsigned char motorID, float period);
    void setPWM(unsigned char motorID, float pwm);
    void enablePID(unsigned char motorID, bool en);
    void setMaxRPM(unsigned char motorID, float rpm);
    void setEncoderTPR(unsigned char motorID, float tpr);

    void setKp(unsigned char motorID, float k);
    void setKi(unsigned char motorID, float k);
    void setKd(unsigned char motorID, float k);
    void setProportionalMode(unsigned char motorID, bool onMeasurement);
    void setPWMFreq(unsigned char motorID, unsigned short int freq);

    double getCurrentRPM(unsigned char motorID);
    double getTargetRPM(unsigned char motorID);
    float getCurrentPWM(unsigned char motorID);
    float getMaxRPM(unsigned char motorID);

  private:
    PID *pid[MOTOR_COUNT];
    float kp[MOTOR_COUNT];
    float ki[MOTOR_COUNT];
    float kd[MOTOR_COUNT];
    unsigned int pidUpdatePeriodUs;
    unsigned long tickSampleTimePrev;
    unsigned long tickSampleTimeDelta;

    double targetRPM[MOTOR_COUNT];
    double measuredRPM[MOTOR_COUNT];
    double pidPWM[MOTOR_COUNT];
    int PWM[MOTOR_COUNT];

    double ticksPerMicroSecToRPM[MOTOR_COUNT];
    long int encDelta[MOTOR_COUNT];
    float maxRPM[MOTOR_COUNT];
    float encoderTPR[MOTOR_COUNT];
    long int encPrev[MOTOR_COUNT];
    bool setPointHasChanged[MOTOR_COUNT];

    unsigned short int pwmFreq[MOTOR_COUNT];
    unsigned char pwmPin[MOTOR_COUNT];
    unsigned char cwPin[MOTOR_COUNT];

    bool switchingCw[MOTOR_COUNT];
    logFuncT logDebug;
};

extern DriveController drive;
