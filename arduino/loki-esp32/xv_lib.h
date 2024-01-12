#pragma once

#include <Arduino.h>
#include "PID_v1_0_0.h"

typedef void (*XvScanPointCallback)(uint16_t, uint16_t, uint16_t, byte);
typedef void (*XvPacketCallback)(uint16_t, byte*, uint16_t);
typedef void (*XvMotorCallback)(float);

class XV {

  public:
    XV();
    void processByte(int inByte);
    void enableMotor(bool enable);
    bool loop();
    float getMotorRPM();
    bool isMotorEnabled();
    int lastRevPeriodMs();

    bool setMotorRPM(float rpm);
    void setMotorPIDCoeffs(float Kp, float Ki, float Kd);
    void setScanPointCallback(XvScanPointCallback scan_callback);
    void setMotorCallback(XvMotorCallback motor_callback);
    void setPacketCallback(XvPacketCallback packet_callback);
    void setMotorPIDSamplePeriod(int sample_period_ms);

    static const int BAUD_RATE = 115200;
    static constexpr float RPM_DEFAULT = 300.0;

    // REF: https://github.com/Xevel/NXV11/wiki
    // The bit 7 of byte 1 seems to indicate that the distance could not be calculated.
    // It's interesting to see that when this bit is set, the second byte is always 80, and the values of the first byte seem to be
    // only 02, 03, 21, 25, 35 or 50... When it's 21, then the whole block is 21 80 XX XX, but for all the other values it's the
    // data block is YY 80 00 00 maybe it's a code to say what type of error ? (35 is preponderant, 21 seems to be when the beam is
    // interrupted by the supports of the cover) .
    static const byte INVALID_DATA_FLAG = (1 << 7);   // Mask for byte 1 of each data quad "Invalid data"
    // The bit 6 of byte 1 is a warning when the reported strength is greatly inferior to what is expected at this distance.
    // This may happen when the material has a low reflectance (black material...), or when the dot does not have the expected
    // size or shape (porous material, transparent fabric, grid, edge of an object...), or maybe when there are parasitic
    // reflections (glass... ).
    static const byte STRENGTH_WARNING_FLAG = (1 << 6);  // Mask for byte 1 of each data quat "Strength Warning"
    static const byte CRC_ERROR_FLAG = (1 << 0);

  protected:
    uint16_t ProcessIndex();
    void ProcessSpeed();
    byte ProcessDistance(int iQuad);
    void ProcessSignalStrength(int iQuad);
    bool IsValidPacket();
    bool motorCheck();    
    void ClearVars();

    XvScanPointCallback scan_callback;
    XvMotorCallback motor_callback;
    XvPacketCallback packet_callback;

    float rpm_setpoint;         // desired RPM (uses float to be compatible with PID library)
    float rpm_min;
    float rpm_max;
    //float pwm_max;              // max analog value.  probably never needs to change from 1023
    //float pwm_min;              // min analog pulse value to spin the motor
  
    bool motor_enable;        // to spin the laser or not.  No data when not spinning
    int eState;
    int rev_period_ms;

    static const unsigned char COMMAND = 0xFA;        // Start of new packet
    static const int INDEX_LO = 0xA0;                 // lowest index value
    static const int INDEX_HI = 0xF9;                 // highest index value
    
    static const uint16_t N_DATA_QUADS = 4;                // there are 4 groups of data elements
    static const uint16_t N_ELEMENTS_PER_QUAD = 4;         // viz., 0=distance LSB; 1=distance MSB; 2=sig LSB; 3=sig MSB
    
    // Offsets to bytes within 'Packet'
    static const uint16_t OFFSET_TO_START = 0;
    static const uint16_t OFFSET_TO_INDEX = OFFSET_TO_START + 1;
    static const uint16_t OFFSET_TO_SPEED_LSB = OFFSET_TO_INDEX + 1;
    static const uint16_t OFFSET_TO_SPEED_MSB = OFFSET_TO_SPEED_LSB + 1;
    static const uint16_t OFFSET_TO_4_DATA_READINGS = OFFSET_TO_SPEED_MSB + 1;
    static const uint16_t OFFSET_TO_CRC_L = OFFSET_TO_4_DATA_READINGS + (N_DATA_QUADS * N_ELEMENTS_PER_QUAD);
    static const uint16_t OFFSET_TO_CRC_M = OFFSET_TO_CRC_L + 1;
    static const uint16_t PACKET_LENGTH = OFFSET_TO_CRC_M + 1;  // 22 length of a complete packet
    // Offsets to the (4) elements of each of the (4) data quads
    static const uint16_t OFFSET_DATA_DISTANCE_LSB = 0;
    static const uint16_t OFFSET_DATA_DISTANCE_MSB = OFFSET_DATA_DISTANCE_LSB + 1;
    static const uint16_t OFFSET_DATA_SIGNAL_LSB = OFFSET_DATA_DISTANCE_MSB + 1;
    static const uint16_t OFFSET_DATA_SIGNAL_MSB = OFFSET_DATA_SIGNAL_LSB + 1;
    
    byte Packet[PACKET_LENGTH];                 // an input packet
    uint16_t ixPacket;                          // index into 'Packet' array

    static const byte BAD_DATA_MASK = (INVALID_DATA_FLAG | STRENGTH_WARNING_FLAG);
    
    static const byte eState_Find_COMMAND = 0;                        // 1st state: find 0xFA (COMMAND) in input stream
    static const byte eState_Build_Packet = eState_Find_COMMAND + 1;  // 2nd state: build the packet
    
    float pwm_val;
    float pwm_last;
    float motor_rpm;
    unsigned long now;
    unsigned long motor_check_timer;
    unsigned long motor_check_interval;
    unsigned int rpm_err_thresh;  // 2 seconds (10 * 200ms) to shutdown motor with improper RPM and high voltage
    unsigned int rpm_err;
    unsigned long curMillis;
    unsigned long lastMillis;
    
    PID_v1 rpmPID;
    
    uint16_t aryDist[N_DATA_QUADS];    // there are (4) distances, one for each data quad
                                       // so the maximum distance is 16383 mm (0x3FFF)
    uint16_t aryQuality[N_DATA_QUADS]; // same with 'quality'
    uint16_t motor_rph;
};
