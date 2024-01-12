#include "xv_lib.h"

XV::XV() {
  scan_callback = NULL;
  motor_callback = NULL;
  packet_callback = NULL;

  eState = XV::eState_Find_COMMAND;
  ixPacket = 0;                          // index into 'Packet' array
  motor_enable = false;

  rpm_min = rpm_setpoint*0.8;
  rpm_max = rpm_setpoint*1.1;
  pwm_val = 0.62;

  //rpm_setpoint = 0;
  rpm_setpoint = RPM_DEFAULT;  // desired RPM 1.8KHz/5FPS/360 = 1 deg resolution
  rpmPID.init(&motor_rpm, &pwm_val, &rpm_setpoint, 3.0e-3, 1.0e-3, 0.0, DIRECT);
  rpmPID.SetOutputLimits(0, 1.0);
  rpmPID.SetSampleTime(20);
  rpmPID.SetMode(AUTOMATIC);

  ClearVars();

  // so the maximum distance is 16383 mm (0x3FFF)
  motor_rph = 0;
  rev_period_ms = 0;

  motor_check_timer = millis();
  motor_check_interval = 200;
  rpm_err_thresh = 10;  // 2 seconds (10 * 200ms) to shutdown motor with improper RPM and high voltage
  rpm_err = 0;
  lastMillis = millis();
}

void XV::setPacketCallback(XvPacketCallback packet_callback) {
  this->packet_callback = packet_callback;
}

void XV::setMotorCallback(XvMotorCallback motor_callback) {
  this->motor_callback = motor_callback;
}

// xv_lds.setMotorMaxPWMDuty(2<<LDS_MOTOR_PWM_BITS-1);

void XV::setScanPointCallback(XvScanPointCallback scan_callback) {
  this->scan_callback = scan_callback; 
}


bool XV::setMotorRPM(float rpm) {
  rpm_setpoint = (rpm <= 0) ? RPM_DEFAULT : rpm;
  return true;
}

void XV::setMotorPIDCoeffs(float Kp, float Ki, float Kd) {
  rpmPID.SetTunings(Kp, Ki, Kd);
}

void XV::setMotorPIDSamplePeriod(int sample_period_ms) {
  rpmPID.SetSampleTime(sample_period_ms);
}


void XV::processByte(int inByte) {
  // Switch, based on 'eState':
  // State 1: We're scanning for 0xFA (COMMAND) in the input stream
  // State 2: Build a complete data packet
  if (eState == XV::eState_Find_COMMAND) {      // flush input until we get COMMAND byte
    if (inByte == XV::COMMAND) {
      eState++;                                 // switch to 'build a packet' state
      Packet[ixPacket++] = inByte;              // store 1st byte of data into 'Packet'
    }
  } else {
    Packet[ixPacket++] = inByte;        // keep storing input into 'Packet'
    if (ixPacket == XV::PACKET_LENGTH) {
      // we've got all the input bytes, so we're done building this packet
      
      if (XV::IsValidPacket()) {      // Check packet CRC
        byte aryInvalidDataFlag[XV::N_DATA_QUADS] = {0, 0, 0, 0}; // non-zero = INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set

        // the first scan angle (of group of 4, based on 'index'), in degrees (0..359)
        // get the starting angle of this group (of 4), e.g., 0, 4, 8, 12, ...
        uint16_t startingAngle = XV::ProcessIndex();

        if (packet_callback)
          packet_callback(startingAngle, Packet, XV::PACKET_LENGTH);

        XV::ProcessSpeed();

        // process each of the (4) sets of data in the packet
        for (int ix = 0; ix < XV::N_DATA_QUADS; ix++)   // process the distance
          aryInvalidDataFlag[ix] = XV::ProcessDistance(ix);
        for (int ix = 0; ix < XV::N_DATA_QUADS; ix++) { // process the signal strength (quality)
          aryQuality[ix] = 0;
          if (aryInvalidDataFlag[ix] == 0)
            XV::ProcessSignalStrength(ix);
        }

        for (int ix = 0; ix < XV::N_DATA_QUADS; ix++) {
          byte err = aryInvalidDataFlag[ix] & XV::BAD_DATA_MASK;
          if (scan_callback)
            scan_callback(startingAngle + ix, int(aryDist[ix]), aryQuality[ix], err);
        }

      } else {
        // Bad packet
        if (packet_callback)
          packet_callback(0, 0, 0);
      }

      XV::ClearVars();   // initialize a bunch of stuff before we switch back to State 1
    }
  }
}

void XV::ClearVars() {
  for (int ix = 0; ix < XV::N_DATA_QUADS; ix++) {
    aryDist[ix] = 0;
    aryQuality[ix] = 0;
    //aryInvalidDataFlag[ix] = 0;
  }
  for (ixPacket = 0; ixPacket < XV::PACKET_LENGTH; ixPacket++)  // clear out this packet
    Packet[ixPacket] = 0;
  ixPacket = 0;
  eState = XV::eState_Find_COMMAND; // This packet is done -- look for next COMMAND byte
}


/*
   ProcessIndex - Process the packet element 'index'
   index is the index byte in the 90 packets, going from A0 (packet 0, readings 0 to 3) to F9
      (packet 89, readings 356 to 359).
   Enter with: N/A
   Uses:       Packet
               ledState gets toggled if angle = 0
               ledPin = which pin the LED is connected to
               ledState = LED on or off
               this->show_dist = true if we're supposed to show distance
               curMillis = milliseconds, now
               lastMillis = milliseconds, last time through this subroutine
               this->show_interval = true ==> display time interval once per revolution, at angle 0
   Calls:      digitalWrite() - used to toggle LED pin
               Serial.print
   Returns:    The first angle (of 4) in the current 'index' group
*/
uint16_t XV::ProcessIndex() {
  uint16_t angle = 0;
  uint16_t data_4deg_index = Packet[OFFSET_TO_INDEX] - INDEX_LO;
  angle = data_4deg_index * N_DATA_QUADS;     // 1st angle in the set of 4

  if (angle == 0) {
    curMillis = millis();
    // Time Interval in ms since last complete revolution
    rev_period_ms = curMillis - lastMillis;
    lastMillis = curMillis;
  }
  return angle;
}

int XV::lastRevPeriodMs() {
  return rev_period_ms;
}

void XV::ProcessSpeed() {
  // Extract motor speed from packet - two bytes little-endian, equals RPM/64
  uint8_t motor_rph_low_byte = Packet[OFFSET_TO_SPEED_LSB];
  uint8_t motor_rph_high_byte = Packet[OFFSET_TO_SPEED_MSB];
  motor_rph = (motor_rph_high_byte << 8) | motor_rph_low_byte;
  motor_rpm = float( (motor_rph_high_byte << 8) | motor_rph_low_byte ) / 64.0;
}

/*
   Data 0 to Data 3 are the 4 readings. Each one is 4 bytes long, and organized as follows :
     byte 0 : <distance 7:0>
     byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>
     byte 2 : <signal strength 7:0>
     byte 3 : <signal strength 15:8>
*/
/*
   xvProcessDistance- Process the packet element 'distance'
   Enter with: iQuad = which one of the (4) readings to process, value = 0..3
   Uses:       Packet
               dist[] = sets distance to object in binary: ISbb bbbb bbbb bbbb
                                       so maximum distance is 0x3FFF (16383 decimal) millimeters (mm)
   Calls:      N/A
   Exits with: 0 = okay
   Error:      1 << 7 = INVALID_DATA_FLAG is set
               1 << 6 = STRENGTH_WARNING_FLAG is set
*/
byte XV::ProcessDistance(int iQuad) {
  uint8_t dataL, dataM;
  aryDist[iQuad] = 0;                     // initialize
  int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_DISTANCE_LSB;
  // byte 0 : <distance 7:0> (LSB)
  // byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8> (MSB)
  dataM = Packet[iOffset + 1];           // get MSB of distance data + flags
  if (dataM & BAD_DATA_MASK)             // if either INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set...
    return dataM & BAD_DATA_MASK;        // ...then return non-zero
  dataL = Packet[iOffset];               // LSB of distance data
  aryDist[iQuad] = dataL | ((dataM & 0x3F) << 8);
  return 0;                              // okay
}

/*
   xvProcessSignalStrength- Process the packet element 'signal strength'
   Enter with: iQuad = which one of the (4) readings to process, value = 0..3
   Uses:       Packet
               quality[] = signal quality
   Calls:      N/A
*/
void XV::ProcessSignalStrength(int iQuad) {
  uint8_t dataL, dataM;
  aryQuality[iQuad] = 0;                        // initialize
  int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_SIGNAL_LSB;
  dataL = Packet[iOffset];                  // signal strength LSB
  dataM = Packet[iOffset + 1];
  aryQuality[iQuad] = dataL | (dataM << 8);
}

/*
   ValidatePacket - Validate 'Packet'
   Enter with: 'Packet' is ready to check
   Uses:       CalcCRC
   Exits with: 0 = Packet is okay
   Error:      non-zero = Packet is no good
*/
bool XV::IsValidPacket() {
  unsigned long chk32;
  unsigned long checksum;
  const int bytesToCheck = PACKET_LENGTH - 2;
  const int CalcCRC_Len = bytesToCheck / 2;
  unsigned int CalcCRC[CalcCRC_Len];

  byte b1a, b1b, b2a, b2b;
  int ix;

  for (int ix = 0; ix < CalcCRC_Len; ix++)       // initialize 'CalcCRC' array
    CalcCRC[ix] = 0;

  // Perform checksum validity test
  for (ix = 0; ix < bytesToCheck; ix += 2)      // build 'CalcCRC' array
    CalcCRC[ix / 2] = Packet[ix] + ((Packet[ix + 1]) << 8);

  chk32 = 0;
  for (ix = 0; ix < CalcCRC_Len; ix++)
    chk32 = (chk32 << 1) + CalcCRC[ix];
  checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
  checksum &= 0x7FFF;
  b1a = checksum & 0xFF;
  b1b = Packet[XV::OFFSET_TO_CRC_L];
  b2a = checksum >> 8;
  b2b = Packet[XV::OFFSET_TO_CRC_M];

  return ((b1a == b1b) && (b2a == b2b));
}

bool XV::loop() {
  if (!motor_enable)
    return false;

  rpmPID.Compute();
  if (pwm_val != pwm_last) {
    if (motor_callback)
      motor_callback(float(pwm_val));
    pwm_last = pwm_val;
  }
  return XV::motorCheck();
}

void XV::enableMotor(bool enable) {
  if (enable) {
    motor_enable = true;
    if (motor_callback)
      motor_callback(float(pwm_val));
    rpm_err = 0;  // reset rpm error
  } else {
    motor_enable = false;
    if (motor_callback)
      motor_callback(0);  
  }
}

bool XV::motorCheck() {  // Make sure the motor RPMs are good else shut it down
  now = millis();
  if (now - motor_check_timer <= motor_check_interval)
    return false;
  
  if (motor_enable && ((motor_rpm < rpm_min) || (motor_rpm > rpm_max))) {
    rpm_err++;
  } else {
    rpm_err = 0;
  }

  motor_check_timer = millis();

  // TODO instead, check how long the RPM has been out of bounds
  return (rpm_err > rpm_err_thresh);
}

float XV::getMotorRPM() {
  return float(motor_rpm);
}

bool XV::isMotorEnabled() {
  return motor_enable;
}
