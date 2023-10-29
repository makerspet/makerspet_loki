/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR Arduino
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.eaibot.com
 * 
 */
#include "YDLidar.h"

YDLidar::YDLidar(ScanPointCallback scan_callback,
  SerialCharCallback serial_callback) : _bined_serialdev(NULL)
{
  this->scan_callback = scan_callback;
  this->serial_callback = serial_callback;
  point.distance = 0;
  point.angle = 0;
  point.quality = 0;
  //point.sampleIndex = 0;
  //point.firstSampleAngle = 0;
  //point.intervalSampleAngle = 0;
}


YDLidar::~YDLidar()
{
  end();
}

// open the given serial interface and try to connect to the YDLIDAR
bool YDLidar::begin(HardwareSerial &serialobj,uint32_t baudrate)
{
    if (isOpen()) {
      end(); 
    }
    _bined_serialdev = &serialobj;
    _bined_serialdev->end();
    //_bined_serialdev->begin(baudrate, SERIAL_8N1, 4, 16);//RX,TX
    _bined_serialdev->begin(baudrate);
  return true;
}

// close the currently opened serial interface
void YDLidar::end(void)
{
    if (isOpen()) {
       _bined_serialdev->end();
       _bined_serialdev = NULL;
    }
}


// check whether the serial interface is opened
bool YDLidar::isOpen(void)
{
    return _bined_serialdev?true:false; 
}

// ask the YDLIDAR for its device health

result_t YDLidar::getHealth(device_health & health, uint32_t timeout) {
    result_t  ans;
  uint8_t  recvPos = 0;
    uint32_t currentTs = millis();
    uint32_t remainingtime;
    uint8_t *infobuf = (uint8_t*)&health;
  lidar_ans_header response_header;
  if (!isOpen()) {
    return RESULT_FAIL;
  }

  {

    ans = sendCommand(LIDAR_CMD_GET_DEVICE_HEALTH,NULL,0);
    if (ans != RESULT_OK) {
      return ans;
    }


    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVHEALTH) {
      return RESULT_FAIL;
    }

    if (response_header.size < sizeof(device_health)) {
      return RESULT_FAIL;
    }

    while ((remainingtime=millis() - currentTs) <= timeout) {
            int currentbyte = _bined_serialdev->read();
            if (currentbyte<0) continue;    
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(device_health)) {
                return RESULT_OK;
            }
        }
  }
  return RESULT_TIMEOUT;
}


// ask the YDLIDAR for its device info 
result_t YDLidar::getDeviceInfo(device_info & info, uint32_t timeout) {
    result_t  ans;
  uint8_t  recvPos = 0;
    uint32_t currentTs = millis();
    uint32_t remainingtime;
    uint8_t *infobuf = (uint8_t*)&info;
  lidar_ans_header response_header;
  if (!isOpen()) {
    return RESULT_FAIL;
  }

  {

    ans = sendCommand(LIDAR_CMD_GET_DEVICE_INFO,NULL,0);
    if (ans != RESULT_OK) {
      return ans;
    }


    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size < sizeof(lidar_ans_header)) {
      return RESULT_FAIL;
    }

    while ((remainingtime=millis() - currentTs) <= timeout) {
            int currentbyte = _bined_serialdev->read();
            if (currentbyte<0) continue;    
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(device_info)) {
                return RESULT_OK;
            }
        }
  }

  return RESULT_TIMEOUT;
}

// stop the scanPoint operation
result_t YDLidar::stop(void)
{
    if (!isOpen()) return RESULT_FAIL;
    result_t ans = sendCommand(LIDAR_CMD_FORCE_STOP,NULL,0);
    return ans;
}

// start the scanPoint operation
result_t YDLidar::startScan(bool force, uint32_t timeout ) {
    result_t ans;

    if (!isOpen()) return RESULT_FAIL;
    
    stop(); //force the previous operation to stop

    {
       
        if ((ans = sendCommand(force?LIDAR_CMD_FORCE_SCAN:LIDAR_CMD_SCAN, NULL, 0)) != RESULT_OK) {
    return ans;
  }

  lidar_ans_header response_header;
  if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
    return ans;
  }

  if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
    return RESULT_FAIL;
  }

  if (response_header.size < sizeof(node_info)) {
    return RESULT_FAIL;
  }
    }
    return RESULT_OK;
}

// wait scan data
result_t YDLidar::waitScanDot() {
  static int recvPos = 0;
  static uint8_t package_Sample_Num = 0;
  static int package_recvPos = 0;
  static int package_sample_sum = 0;
  static int currentByte = 0;

  static node_package package;
  static uint8_t *packageBuffer = (uint8_t*)&package.package_Head;

  static uint16_t package_Sample_Index = 0;
  static float IntervalSampleAngle = 0;
  static float IntervalSampleAngle_LastPackage = 0;
  static uint16_t FirstSampleAngle = 0;
  static uint16_t LastSampleAngle = 0;
  static uint16_t CheckSum = 0;

  static uint16_t CheckSumCal = 0;
  static uint16_t SampleNumlAndCTCal = 0;
  static uint16_t LastSampleAngleCal = 0;
  static bool CheckSumResult = true;
  static uint16_t Valu8Tou16 = 0;

  static uint8_t state = 0;

  switch(state) {
    case 1:
      goto state1;
    case 2:
      goto state2;
  }

  // Read in a packet; a packet contains up to 40 samples
  // Each packet has a Start and End (absolute) angles
  if(package_Sample_Index == 0) {
    
    // Read in, parse the packet header: first PackagePaidBytes=10 bytes
    package_Sample_Num = 0;
    package_recvPos = 0;
    //uint32_t waitTime;
    recvPos = 0;
    //uint32_t startTs = millis();

    while (true) {
state1:
      currentByte = _bined_serialdev->read();
      if (currentByte<0) {
        state = 1;
        return RESULT_NOT_READY;
      } else
        serial_callback((char)currentByte);

      switch (recvPos) {
      case 0:
        if(currentByte!=(PH&0xFF)){
          continue;
        }
        break;
      case 1:
        CheckSumCal = PH;
        if(currentByte!=(PH>>8)){
          recvPos = 0;
          continue;
        }
        break;
      case 2:
        SampleNumlAndCTCal = currentByte;
        if ((currentByte != CT_Normal) && (currentByte != CT_RingStart)){ 
          recvPos = 0;
          continue;
        }
        break;
      case 3:
        SampleNumlAndCTCal += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        package_Sample_Num = currentByte;
        break;
      case 4:
        if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
          FirstSampleAngle = currentByte;
        } else {
          recvPos = 0;
          continue;
        }
        break;
      case 5:
        FirstSampleAngle += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        CheckSumCal ^= FirstSampleAngle;
        FirstSampleAngle = FirstSampleAngle>>1;
        break;
      case 6:
        if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
          LastSampleAngle = currentByte;
        } else {
          recvPos = 0;
          continue;
        }
        break;
      case 7:
        LastSampleAngle += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        LastSampleAngleCal = LastSampleAngle;
        LastSampleAngle = LastSampleAngle>>1;
        if(package_Sample_Num == 1){
          IntervalSampleAngle = 0;
        }else{
          if(LastSampleAngle < FirstSampleAngle){
            if((FirstSampleAngle > 17280) && (LastSampleAngle < 5760)){
              IntervalSampleAngle = ((float)(23040 + LastSampleAngle - FirstSampleAngle))/(package_Sample_Num-1);
              IntervalSampleAngle_LastPackage = IntervalSampleAngle;
            } else{
              IntervalSampleAngle = IntervalSampleAngle_LastPackage;
            }
          } else{
            IntervalSampleAngle = ((float)(LastSampleAngle -FirstSampleAngle))/(package_Sample_Num-1);
            IntervalSampleAngle_LastPackage = IntervalSampleAngle;
          }
        }
        break;
      case 8:
        CheckSum = currentByte; 
        break;
      case 9:
        CheckSum += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        break;
      }     
      packageBuffer[recvPos++] = currentByte;

      if (recvPos  == PackagePaidBytes ){
        package_recvPos = recvPos;
        break;        

      }
    }

    // Read in the rest of the packet, i.e. samples
    if(PackagePaidBytes == recvPos){
      //startTs = millis();
      recvPos = 0;
      package_sample_sum = package_Sample_Num<<1;

      while (true) {
state2:
        currentByte = _bined_serialdev->read();
        if (currentByte<0){
          state = 2;
          return RESULT_NOT_READY;
        } else
          serial_callback((char)currentByte);
        if((recvPos &1) == 1){
          Valu8Tou16 += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
          CheckSumCal ^= Valu8Tou16;
        }else{
          Valu8Tou16 = currentByte; 
        }
                    
        packageBuffer[package_recvPos+recvPos] =currentByte;          
        recvPos++;
        if(package_sample_sum == recvPos){
          package_recvPos += recvPos;
          break;
        }
      }

      if(package_sample_sum != recvPos){
        state = 0;
        return RESULT_FAIL;
      }
    } else {
      state = 0;
      return RESULT_FAIL;
    }
    CheckSumCal ^= SampleNumlAndCTCal;
    CheckSumCal ^= LastSampleAngleCal;

    if(CheckSumCal != CheckSum){  
      CheckSumResult = false;
    }else{
      CheckSumResult = true;
    }
  }

  while(true) {
  
    uint8_t package_CT;
    node_info node;
  
    package_CT = package.package_CT;    
    if(package_CT == CT_Normal){
      node.sync_quality = Node_Default_Quality + Node_NotSync;
    } else{
      node.sync_quality = Node_Default_Quality + Node_Sync;
    }
  
    if(CheckSumResult == true){
      int32_t AngleCorrectForDistance;
      node.distance_q2 = package.packageSampleDistance[package_Sample_Index];
            
      if(node.distance_q2/4 != 0){
        AngleCorrectForDistance = (int32_t)((atan(((21.8*(155.3 - (node.distance_q2*0.25f)) )/155.3)/(node.distance_q2*0.25f)))*3666.93);
      }else{
        AngleCorrectForDistance = 0;    
      }
      float sampleAngle = IntervalSampleAngle*package_Sample_Index;
      if((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) < 0){
        node.angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle + AngleCorrectForDistance + 23040))<<LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
      }else{
        if((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) > 23040){
          node.angle_q6_checkbit = ((uint16_t)((FirstSampleAngle + sampleAngle + AngleCorrectForDistance - 23040))<<LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
        }else{
          node.angle_q6_checkbit = ((uint16_t)((FirstSampleAngle + sampleAngle + AngleCorrectForDistance))<<LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
        } 
      }
    }else{
      node.sync_quality = Node_Default_Quality + Node_NotSync;
      node.angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
      node.distance_q2 = 0;
      package_Sample_Index = 0;
      state = 0;
      return RESULT_CRC_ERROR;
    }
  
    // Dump out processed data
    point.distance = node.distance_q2*0.25f;
    point.angle = (node.angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
    point.quality = (node.sync_quality>>LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
    point.startBit = (node.sync_quality & LIDAR_RESP_MEASUREMENT_SYNCBIT);
    //point.sampleIndex = package_Sample_Index;
    //point.firstSampleAngle = FirstSampleAngle/64.0f;
    //point.intervalSampleAngle = IntervalSampleAngle/64.0f;
    //point.angleCorrectionForDistance = AngleCorrectForDistance/64.0f;

    scan_callback(point.quality, point.angle, point.distance, point.startBit);

    // Dump finished?
    package_Sample_Index++;
    uint8_t nowPackageNum = package.nowPackageNum;  
    if(package_Sample_Index >= nowPackageNum){
      package_Sample_Index = 0;
      break;
    }
  }
  state = 0;
  return RESULT_OK;
}


//send data to serial
result_t YDLidar::sendCommand(uint8_t cmd, const void * payload, size_t payloadsize) {
  cmd_packet pkt_header;
  cmd_packet * header = &pkt_header;
  uint8_t checksum = 0;
  if (payloadsize && payload) { 
    cmd |= LIDAR_CMDFLAG_HAS_PAYLOAD;
  }

  header->syncByte = LIDAR_CMD_SYNC_BYTE;
  header->cmd_flag = cmd&0xff;

  _bined_serialdev->write((uint8_t *)header, 2) ;
  if((cmd & LIDAR_CMDFLAG_HAS_PAYLOAD)){
    checksum ^= LIDAR_CMD_SYNC_BYTE;
    checksum ^= (cmd&0xff);
    checksum ^= (payloadsize & 0xFF);
    for (size_t pos = 0; pos < payloadsize; ++pos) {
      checksum ^= ((uint8_t *)payload)[pos];    }

    uint8_t sizebyte = payloadsize;
    _bined_serialdev->write(&sizebyte, 1);
    _bined_serialdev->write((const uint8_t *)payload, sizebyte);
    _bined_serialdev->write(&checksum, 1);
  }
  return RESULT_OK;
}


// wait response header
result_t YDLidar::waitResponseHeader(lidar_ans_header * header, uint32_t timeout) {
  int  recvPos = 0;
  uint32_t startTs = millis();
  uint8_t  *headerBuffer = (uint8_t *)(header);
  uint32_t waitTime;

  while ((waitTime=millis() - startTs) <= timeout) {
    int currentbyte = _bined_serialdev->read();
        if (currentbyte<0) continue;
          switch (recvPos) {
            case 0:
                if (currentbyte != LIDAR_ANS_SYNC_BYTE1) {
                    continue;
                }
                break;
            case 1:
                if (currentbyte != LIDAR_ANS_SYNC_BYTE2) {
                    recvPos = 0;
                    continue;
                }
                break;
          }
          headerBuffer[recvPos++] = currentbyte;

          if (recvPos == sizeof(lidar_ans_header)) {
                return RESULT_OK;
          }
  }
    return RESULT_TIMEOUT;
}
