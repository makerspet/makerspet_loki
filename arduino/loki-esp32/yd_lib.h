//https://github.com/EAIBOT/ydlidar_arduino works
//https://github.com/YDLIDAR/ydlidar_arduino broken
#pragma once

#include "Arduino.h"
#include "v8stdint.h"

// TODO take Serial out
// TODO LDS shim

typedef enum {
  YD_CT_NORMAL = 0,
  YD_CT_RING_START  = 1,
  YD_CT_TAIL,
} YD_CT;

struct yd_node_info {
  uint8_t    sync_quality;
  uint16_t   angle_q6_checkbit;
  uint16_t   distance_q2;
} __attribute__((packed)) ;

struct yd_device_info{
  uint8_t   model;
  uint16_t  firmware_version;
  uint8_t   hardware_version;
  uint8_t   serialnum[16];
} __attribute__((packed)) ;

struct yd_device_health {
  uint8_t   status;
  uint16_t  error_code;
} __attribute__((packed))  ;

//struct yd_sampling_rate {
//  uint8_t rate;
//} __attribute__((packed))  ;

//struct yd_scan_frequency {
//  uint32_t frequency;
//} __attribute__((packed))  ;

//struct yd_scan_rotation {
//  uint8_t rotation;
//} __attribute__((packed))  ;

struct yd_cmd_packet {
  uint8_t syncByte;
  uint8_t cmd_flag;
  uint8_t size;
  uint8_t data;
} __attribute__((packed)) ;

struct yd_lidar_ans_header {
  uint8_t  syncByte1;
  uint8_t  syncByte2;
  uint32_t size:30;
  uint32_t subType:2;
  uint8_t  type;
} __attribute__((packed));

struct YdScanPoint {
  uint8_t quality;
  float   angle;
  float   distance;
  bool    startBit;
  //uint16_t sampleIndex;
  //float   firstSampleAngle;
  //float   intervalSampleAngle;
  //float   angleCorrectionForDistance;
};


#if defined(_WIN32)
#pragma pack(1)
#endif

typedef void (*YdScanPointCallback)(uint8_t, float, float, bool);
typedef void (*YdSerialCharCallback)(char);

//YDLidarX4 class
class YDLidarX4
{
public:
//    enum {
//        SERIAL_BAUDRATE = 115200,  
//        DEFAULT_TIMEOUT = 500,
//    };
    static const uint32_t BAUD_RATE = 115200;
    static const uint32_t DEFAULT_TIMEOUT = 500;
    static const uint8_t PACKAGE_SAMPLE_MAX_LENGTH = 0x80;

    //construct
    YDLidarX4();
    void setScanPointCallback(YdScanPointCallback scan_callback);
    void setSerialCharCallback(YdSerialCharCallback serial_callback);

    //destructor
    ~YDLidarX4();

    // open the given serial interface and try to connect to the YDLidarX4
    bool begin(HardwareSerial &serialobj, uint32_t baudrate = BAUD_RATE);

    // close the currently opened serial interface
    void end();
  
    // check whether the serial interface is opened
    bool isOpen(); 

    // ask the YDLidarX4 for its health info
    result_t getHealth(yd_device_health & health, uint32_t timeout = DEFAULT_TIMEOUT);
    
    // ask the YDLidarX4 for its device info like the serial number
    result_t getDeviceInfo(yd_device_info & info, uint32_t timeout = DEFAULT_TIMEOUT);

    // stop the scanPoint operation
    result_t stop();

    // start the scanPoint operation
    result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT*2);

    // wait for one sample package to arrive
    result_t waitScanDot();
    
    // retrieve currently received sample point
    const YdScanPoint & getCurrentScanPoint()
    {
        return point;
    }

protected:
    static const uint8_t LIDAR_CMD_STOP = 0x65;
    static const uint8_t LIDAR_CMD_SCAN = 0x60;
    static const uint8_t LIDAR_CMD_FORCE_SCAN = 0x61;
    static const uint8_t LIDAR_CMD_RESET = 0x80;
    static const uint8_t LIDAR_CMD_FORCE_STOP = 0x00;
    static const uint8_t LIDAR_CMD_GET_EAI = 0x55;
    static const uint8_t LIDAR_CMD_GET_DEVICE_INFO = 0x90;
    static const uint8_t LIDAR_CMD_GET_DEVICE_HEALTH = 0x92;
    static const uint8_t LIDAR_CMD_SYNC_BYTE = 0xA5;
    static const uint16_t LIDAR_CMDFLAG_HAS_PAYLOAD = 0x8000;

    static const uint8_t LIDAR_ANS_TYPE_DEVINFO = 0x4;
    static const uint8_t LIDAR_ANS_TYPE_DEVHEALTH = 0x6;
    static const uint8_t LIDAR_ANS_SYNC_BYTE1 = 0xA5;
    static const uint8_t LIDAR_ANS_SYNC_BYTE2 = 0x5A;
    static const uint8_t LIDAR_ANS_TYPE_MEASUREMENT = 0x81;

    static const uint8_t LIDAR_RESP_MEASUREMENT_SYNCBIT = (0x1<<0);
    static const uint8_t LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT = 2;
    static const uint8_t LIDAR_RESP_MEASUREMENT_CHECKBIT = (0x1<<0);
    static const uint8_t LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT = 1;
    static const uint8_t LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT = 8;
    
    static const uint8_t LIDAR_CMD_RUN_POSITIVE = 0x06;
    static const uint8_t LIDAR_CMD_RUN_INVERSION = 0x07;
    static const uint8_t LIDAR_CMD_SET_AIMSPEED_ADDMIC = 0x09;
    static const uint8_t LIDAR_CMD_SET_AIMSPEED_DISMIC = 0x0A;
    static const uint8_t LIDAR_CMD_SET_AIMSPEED_ADD = 0x0B;
    static const uint8_t LIDAR_CMD_SET_AIMSPEED_DIS = 0x0C;
    static const uint8_t LIDAR_CMD_GET_AIMSPEED = 0x0D;
    static const uint8_t LIDAR_CMD_SET_SAMPLING_RATE = 0xD0;
    static const uint8_t LIDAR_CMD_GET_SAMPLING_RATE = 0xD1;
    
    static const uint8_t LIDAR_STATUS_OK = 0x0;
    static const uint8_t LIDAR_STATUS_WARNING = 0x1;
    static const uint8_t LIDAR_STATUS_ERROR = 0x2;
    
    static const uint8_t PACKAGE_SAMPLE_BYTES = 2;
    static const uint16_t NODE_DEFAULT_QUALITY = (10<<2);
    static const uint8_t NODE_SYNC = 1;
    static const uint8_t NODE_NOT_SYNC = 2;
    static const uint8_t PACKAGE_PAID_BYTES = 10;
    static const uint16_t PH = 0x55AA;

protected:
    // send ask command to YDLidarX4
    result_t sendCommand(uint8_t cmd, const void * payload = NULL, size_t payloadsize = 0);
    //wait for response header to arrive
    result_t waitResponseHeader(yd_lidar_ans_header * header,
      uint32_t timeout = DEFAULT_TIMEOUT);

protected:
    HardwareSerial * _bined_serialdev;  
    YdScanPoint point;
    YdScanPointCallback scan_callback;
    YdSerialCharCallback serial_callback;
};

struct yd_node_package {
  uint16_t  package_Head;
  uint8_t   package_CT;
  uint8_t   nowPackageNum;
  uint16_t  packageFirstSampleAngle;
  uint16_t  packageLastSampleAngle;
  uint16_t  checkSum;
  uint16_t  packageSampleDistance[YDLidarX4::PACKAGE_SAMPLE_MAX_LENGTH];
} __attribute__((packed)) ;
