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

#ifndef ESP32
  #error This code runs on ESP32
#endif

#include "robot_config.h"
#include "util.h"
#include <WiFi.h>
#include <stdio.h>
#include <micro_ros_kaia.h>
#include <HardwareSerial.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <kaiaai_msgs/msg/kaiaai_telemetry.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl_interfaces/msg/log.h>
#include <rmw_microros/rmw_microros.h>
//#include <rmw_microros/discovery.h>
#include <rclc_parameter/rclc_parameter.h>
#include "drive.h"
#include "ap.h"

#if defined LDS_YDLIDAR_X4_
  #include "LDS_YDLIDAR_X4.h"
  LDS_YDLIDAR_X4 lds;
#elif defined LDS_LDS02RR_
  #include "LDS_LDS02RR.h"
  LDS_LDS02RR lds;
#endif

#if !defined(IS_MICRO_ROS_KAIA_MIN_VERSION) || !IS_MICRO_ROS_KAIA_MIN_VERSION(2,0,7,3)
#error "Please upgrade micro_ros_kaia library version"
#endif

#define RCCHECK(fn,E) { rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){error_loop((E));}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; \
  if((temp_rc != RCL_RET_OK)){Serial.println("RCSOFTCHECK failed");}}

const float SPEED_DIFF_TO_US = 1e6/MAX_WHEEL_ACCEL;
const float WHEEL_BASE_RECIP = 1/WHEEL_BASE;
const float WHEEL_RADIUS = WHEEL_DIA / 2;

rcl_publisher_t telem_pub;
rcl_publisher_t log_pub;
rcl_subscription_t twist_sub;
kaiaai_msgs__msg__KaiaaiTelemetry telem_msg;
geometry_msgs__msg__Twist twist_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;
rclc_parameter_server_t param_server;

HardwareSerial LdSerial(2); // TX 17, RX 16

float joint_pos[JOINTS_LEN] = {0};
float joint_vel[JOINTS_LEN] = {0};
float joint_prev_pos[JOINTS_LEN] = {0};
uint8_t lds_buf[LDS_BUF_LEN] = {0};

unsigned long telem_prev_pub_time_us = 0;
unsigned long ping_prev_pub_time_us = 0;
unsigned long telem_pub_period_us = UROS_TELEM_PUB_PERIOD_MS*1000;
unsigned long ping_pub_period_us = UROS_PING_PUB_PERIOD_MS*1000;

unsigned long ramp_duration_us = 0;
unsigned long ramp_start_time_us = 0;
float ramp_start_rpm_right = 0;
float ramp_start_rpm_left = 0;
float ramp_target_rpm_right = 0;
float ramp_target_rpm_left = 0;

bool ramp_enabled = true;

#ifdef SPIN_TELEM_STATS
unsigned long stat_sum_spin_telem_period_us = 0;
unsigned long stat_max_spin_telem_period_us = 0;
#endif

size_t lds_serial_write_callback(const uint8_t * buffer, size_t length) {
  return LdSerial.write(buffer, length);
}

int lds_serial_read_callback() {
  return LdSerial.read();
}

void twist_sub_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  float target_speed_lin_x = msg->linear.x;
  float target_speed_ang_z = msg->angular.z;

  if (msg->linear.y != 0) {
    Serial.print("Warning: /cmd_vel linear.y = ");
    Serial.print(msg->linear.y);
    Serial.println(" not zero");
  }

  // Twist to target wheel speeds
  float twist_target_speed_right = 0;
  float twist_target_speed_left = 0;

  twistToWheelSpeeds(target_speed_lin_x, target_speed_ang_z,
    &twist_target_speed_right, &twist_target_speed_left);

  twist_target_speed_left = -twist_target_speed_left;

  // Wheel speeds to RPM
  float twist_target_rpm_right = SPEED_TO_RPM(twist_target_speed_right);
  float twist_target_rpm_left = SPEED_TO_RPM(twist_target_speed_left);

  // Limit target RPM
  float limited_target_rpm_right =
    absMin(twist_target_rpm_right, drive.getMaxRPM(MOTOR_RIGHT));
  float limited_target_rpm_left =
    absMin(twist_target_rpm_left, drive.getMaxRPM(MOTOR_LEFT));

  // Scale down both target RPMs to within limits
  if (twist_target_rpm_right != limited_target_rpm_right ||
    twist_target_rpm_left != limited_target_rpm_left) {

    float rpm_scale_down_factor_right = 1;
    float rpm_scale_down_factor_left = 1;

    if (twist_target_rpm_right != 0) {
      rpm_scale_down_factor_right = limited_target_rpm_right /
        twist_target_rpm_right;
    }
    if (twist_target_rpm_left != 0) {
      rpm_scale_down_factor_left = limited_target_rpm_left /
        twist_target_rpm_left;
    }

    float rpm_scale_down_factor = min(rpm_scale_down_factor_right,
      rpm_scale_down_factor_left);   

    ramp_target_rpm_right = twist_target_rpm_right * rpm_scale_down_factor;
    ramp_target_rpm_left = twist_target_rpm_left * rpm_scale_down_factor;
  } else {
    ramp_target_rpm_right = twist_target_rpm_right;
    ramp_target_rpm_left = twist_target_rpm_left;
  }

  Serial.print("linear.x ");
  Serial.print(msg->linear.x, 3);
  Serial.print(", angular.z ");
  Serial.print(msg->angular.z, 3);
  Serial.print("; target RPM R ");
  Serial.print(ramp_target_rpm_right);
  Serial.print(" L ");
  Serial.println(ramp_target_rpm_left);

  if (!ramp_enabled) {
    setMotorSpeeds(ramp_target_rpm_right, ramp_target_rpm_left);
    return;
  }

  // Calculate change in speeds
  ramp_start_rpm_right = drive.getTargetRPM(MOTOR_RIGHT);
  ramp_start_rpm_left = drive.getTargetRPM(MOTOR_LEFT);
  
  float ramp_start_speed_right = RPM_TO_SPEED(ramp_start_rpm_right);
  float ramp_start_speed_left = RPM_TO_SPEED(ramp_start_rpm_left);

  float ramp_target_speed_right = RPM_TO_SPEED(ramp_target_rpm_right);
  float ramp_target_speed_left = RPM_TO_SPEED(ramp_target_rpm_left);

  float ramp_speed_diff_right = ramp_target_speed_right - ramp_start_speed_right;
  float ramp_speed_diff_left = ramp_target_speed_left - ramp_start_speed_left;

  // Calculate time to accelerate
  float abs_speed_diff_right = abs(ramp_speed_diff_right);
  float abs_speed_diff_left = abs(ramp_speed_diff_left);
  float max_abs_speed_diff = max(abs_speed_diff_right, abs_speed_diff_left);

  ramp_duration_us = max_abs_speed_diff * SPEED_DIFF_TO_US;
  ramp_start_time_us = esp_timer_get_time(); // Start speed ramp

  updateSpeedRamp();
}

void setMotorSpeeds(float ramp_target_rpm_right, float ramp_target_rpm_left) {
  //Serial.print("setRPM() ");
  //Serial.print(ramp_target_rpm_right);
  //Serial.print(" ");
  //Serial.println(ramp_target_rpm_left);

  drive.setRPM(MOTOR_RIGHT, ramp_target_rpm_right);
  drive.setRPM(MOTOR_LEFT, ramp_target_rpm_left);
}

void updateSpeedRamp() {
  if (ramp_target_rpm_right == drive.getTargetRPM(MOTOR_RIGHT) &&
    ramp_target_rpm_left == drive.getTargetRPM(MOTOR_LEFT)) {
    return;
  }

  unsigned long time_now_us = esp_timer_get_time();
  unsigned long ramp_elapsed_time_us = time_now_us - ramp_start_time_us;

  float rpm_right;
  float rpm_left;

  if (ramp_elapsed_time_us < ramp_duration_us) {
    float ratio = (float)ramp_elapsed_time_us / (float)ramp_duration_us; // 0..1
    float rpm_change_right = (ramp_target_rpm_right - ramp_start_rpm_right) * ratio;
    float rpm_change_left = (ramp_target_rpm_left - ramp_start_rpm_left) * ratio;

    rpm_right = ramp_start_rpm_right + rpm_change_right;
    rpm_left = ramp_start_rpm_left + rpm_change_left;
  } else {
    rpm_right = ramp_target_rpm_right;
    rpm_left = ramp_target_rpm_left;
  }

  setMotorSpeeds(rpm_right, rpm_left);
}

void logInfo(char* msg) {
  Serial.println(msg);
}

void delaySpin(unsigned long msec) {
  unsigned long time_msec = millis();
  while(millis() - time_msec < msec) {
    spinResetSettings();
    yield();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(CONFIG::LED_PIN, OUTPUT);
  digitalWrite(CONFIG::LED_PIN, HIGH);  

  setupLDS();

  if (!initSPIFFS())
    blink_error_code(ERR_SPIFFS_INIT);

  if (!initWiFi(getSSID(), getPassw())) {
    digitalWrite(CONFIG::LED_PIN, HIGH);
    ObtainWiFiCreds(spinResetSettings, UROS_ROBOT_MODEL);
    return;
  }

  set_microros_wifi_transports(getDestIP().c_str(), getDestPort().toInt());

  delay(2000);
  
  initRos();
  logMsgInfo((char*)"Micro-ROS initialized");
  
  if (startLDS() != LDS::RESULT_OK)
    blink_error_code(ERR_LDS_START);
    //error_loop(ERR_LDS_START);
  
  drive.initOnce(logInfo);
  drive.resetEncoders();

  //telem_prev_pub_time_us = esp_timer_get_time();
}

static inline void initRos() {
  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator), ERR_UROS_INIT);

  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

  // https://github.com/micro-ROS/micro-ROS-demos/blob/iron/rclc/autodiscover_agent/main.c
  // https://micro.ros.org/blog/2020/09/30/discovery/
  // https://micro.ros.org/docs/api/rmw/
  // https://github.com/micro-ROS/rmw_microxrcedds/blob/iron/rmw_microxrcedds_c/include/rmw_microros/discovery.h
  // https://github.com/micro-ROS/rmw_microxrcedds/blob/iron/rmw_microxrcedds_c/src/rmw_microros/discovery.c
  // Auto discover micro-ROS agent
  // RMW_UXRCE_TRANSPORT=custom
  // RMW_UXRCE_TRANSPORT_UDP
  //Serial.print("micro-ROS agent ");
  //if (rmw_uros_discover_agent(rmw_options) == RCL_RET_OK) {
  //  Serial.println("not ");
  //}
  //Serial.print("found");

  RCCHECK(rmw_uros_options_set_client_key(UROS_CLIENT_KEY, rmw_options),
    ERR_UROS_INIT); // TODO multiple bots

  Serial.print(F("Connecting to Micro-ROS agent ... "));
  //RCCHECK(rclc_support_init(&support, 0, NULL, &allocator), ERR_UROS_AGENT_CONN);
  //RCCHECK(rclc_support_init_with_options(&support, 0, NULL,
  //  &init_options, &allocator), ERR_UROS_AGENT_CONN);
  rcl_ret_t temp_rc = rclc_support_init_with_options(&support, 0, NULL,
    &init_options, &allocator);
  if (temp_rc != RCL_RET_OK) {
    Serial.println("failed");
    error_loop(ERR_UROS_AGENT_CONN);
  }
  Serial.println("success");

  syncRosTime();
  printCurrentTime();

  // https://micro.ros.org/docs/tutorials/programming_rcl_rclc/node/
  RCCHECK(rclc_node_init_default(&node, UROS_NODE_NAME, "", &support),
    ERR_UROS_NODE);

  RCCHECK(rclc_subscription_init_default(&twist_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    UROS_CMD_VEL_TOPIC_NAME), ERR_UROS_PUBSUB);

  RCCHECK(rclc_publisher_init_best_effort(&telem_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(kaiaai_msgs, msg, KaiaaiTelemetry),
    UROS_TELEM_TOPIC_NAME), ERR_UROS_PUBSUB);

  RCCHECK(rclc_publisher_init_default(&log_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
    UROS_LOG_TOPIC_NAME), ERR_UROS_PUBSUB);

  // https://github.com/ros2/rclc/blob/humble/rclc_examples/src/example_parameter_server.c
  // Request size limited to one parameter on Set, Get, Get types and Describe services.
  // List parameter request has no prefixes enabled nor depth.
  // Parameter description strings not allowed, rclc_add_parameter_description is disabled.
  // RCLC_PARAMETER_MAX_STRING_LENGTH = 50
  const rclc_parameter_options_t rclc_param_options = {
      .notify_changed_over_dds = false,
      .max_params = 3,
      .allow_undeclared_parameters = false,
      .low_mem_mode = true };
  
  //RCCHECK(rclc_parameter_server_init_default(&param_server, &node), ERR_UROS_PARAM);
  temp_rc = rclc_parameter_server_init_with_option(&param_server, &node, &rclc_param_options);
  if (temp_rc != RCL_RET_OK) {
    Serial.print("Micro-ROS parameter server init failed.");
    Serial.println("Make sure micro_ros_kaia library version is latest.");
    error_loop(ERR_UROS_PARAM);
  }

  RCCHECK(rclc_executor_init(&executor, &support.context,
    RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 1, &allocator), ERR_UROS_EXEC);

  RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub,
    &twist_msg, &twist_sub_callback, ON_NEW_DATA), ERR_UROS_EXEC);

  RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server,
    on_param_changed), ERR_UROS_EXEC);;

  //RCCHECK(rclc_add_parameter(&param_server, "param_bool", RCLC_PARAMETER_BOOL), ERR_UROS_PARAM);
  //RCCHECK(rclc_add_parameter(&param_server, "param_int", RCLC_PARAMETER_INT), ERR_UROS_PARAM);
  RCCHECK(rclc_add_parameter(&param_server, UROS_PARAM_LDS_MOTOR_SPEED, RCLC_PARAMETER_DOUBLE), ERR_UROS_PARAM);

  //RCCHECK(rclc_parameter_set_bool(&param_server, "param_bool", false), ERR_UROS_PARAM);
  //RCCHECK(rclc_parameter_set_int(&param_server, "param_int", 10), ERR_UROS_PARAM);
  RCCHECK(rclc_parameter_set_double(&param_server, UROS_PARAM_LDS_MOTOR_SPEED, LDS_MOTOR_SPEED_DEFAULT), ERR_UROS_PARAM);

  //rclc_add_parameter_description(&param_server, "param_int", "Second parameter", "Only even numbers");
  //RCCHECK(rclc_add_parameter_constraint_integer(&param_server, "param_int", -10, 120, 2), ERR_UROS_PARAM);

  //rclc_add_parameter_description(&param_server, "param_double", "Third parameter", "");
  //RCCHECK(rclc_set_parameter_read_only(&param_server, "param_double", true), ERR_UROS_PARAM);

  RCCHECK(rclc_add_parameter_constraint_double(&param_server, UROS_PARAM_LDS_MOTOR_SPEED, -1.0, 1.0, 0), ERR_UROS_PARAM);

  //bool param_bool;
  //int64_t param_int;
  //double param_double;

  //RCCHECK(rclc_parameter_get_bool(&param_server, "param_bool", &param_bool), ERR_UROS_PARAM);
  //RCCHECK(rclc_parameter_get_int(&param_server, "param_int", &param_int), ERR_UROS_PARAM);
  //RCCHECK(rclc_parameter_get_double(&param_server, "param_double", &param_double), ERR_UROS_PARAM);

  resetTelemMsg();
}

bool on_param_changed(const Parameter * old_param, const Parameter * new_param, void * context) {
  (void) context;

  if (old_param == NULL && new_param == NULL) {
    Serial.println("old_param == NULL");
    return false;
  }
  if (new_param == NULL) {
    Serial.println("new_param == NULL");
    return false;
  }

  Serial.print("Parameter ");
  Serial.print(old_param->name.data);
  Serial.print(" modified ");
  switch (old_param->value.type) {
    case RCLC_PARAMETER_BOOL:
      Serial.print(old_param->value.bool_value);
      Serial.print(" to ");
      Serial.println(new_param->value.bool_value);
      break;
    case RCLC_PARAMETER_INT:
      Serial.print(old_param->value.integer_value);
      Serial.print(" to ");
      Serial.println(new_param->value.integer_value);
      break;
    case RCLC_PARAMETER_DOUBLE:
      Serial.print(old_param->value.double_value);
      Serial.print(" to ");
      Serial.println(new_param->value.double_value);

      if (strcmp(old_param->name.data, UROS_PARAM_LDS_MOTOR_SPEED) == 0) {
        //int16_t speed_int = round((float)(new_param->value.double_value) * 255);
//        lds.setScanTargetFreqHz(new_param->value.double_value);
      }
      break;
    default:
      break;
  }

  return true;
}

static inline bool initWiFi(String ssid, String passw) {

  if(ssid.length() == 0){
    Serial.println("Undefined SSID");
    return false;
  }

  WiFi.mode(WIFI_STA);
  //localIP.fromString(ip.c_str());
  //localGateway.fromString(gateway.c_str());

  //if (!WiFi.config(localIP, localGateway, subnet)){
  //  Serial.println("STA Failed to configure");
  //  return false;
  //}

  WiFi.begin(ssid, passw);

  Serial.print("Connecting to WiFi ");
  Serial.print(ssid);
  Serial.print(" ");

  unsigned long startMillis = millis();

  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startMillis >= WIFI_CONN_TIMEOUT_SEC*1000) {
      Serial.println(" timed out");
      return false;
    }

    digitalWrite(CONFIG::LED_PIN, HIGH);
    delay(250);
    digitalWrite(CONFIG::LED_PIN, LOW);
    Serial.print('.'); // Don't use F('.'), it crashes code!!
    delay(250);
    spinResetSettings();
  }

  digitalWrite(CONFIG::LED_PIN, LOW);
  Serial.println(F(" connected"));
  Serial.print(F("IP "));
  Serial.println(WiFi.localIP());
  return true;
}

void spinTelem(bool force_pub) {
  static int telem_pub_count = 0;
  unsigned long time_now_us = esp_timer_get_time();
  unsigned long step_time_us = time_now_us - telem_prev_pub_time_us;

  if (!force_pub && (step_time_us < telem_pub_period_us))
    return;

  publishTelem(step_time_us);
  telem_prev_pub_time_us = time_now_us;

  digitalWrite(CONFIG::LED_PIN, !digitalRead(CONFIG::LED_PIN));
  //if (++telem_pub_count % 5 == 0) {
    //Serial.print("RPM L ");
    //Serial.print(drive.getCurrentRPM(MOTOR_LEFT));
    //Serial.print(" R ");
    //Serial.println(drive.getCurrentRPM(MOTOR_RIGHT));
  //}

  #ifdef SPIN_TELEM_STATS
  stat_sum_spin_telem_period_us += step_time_us;
  stat_max_spin_telem_period_us = stat_max_spin_telem_period_us <= step_time_us ?
    step_time_us : stat_max_spin_telem_period_us;
  
  // How often telemetry gets published
  if (++telem_pub_count % SPIN_TELEM_STATS == 0) {
    Serial.print("spinTelem() period avg ");
    Serial.print(stat_sum_spin_telem_period_us / (1000*SPIN_TELEM_STATS));
    Serial.print(" max ");
    Serial.print(stat_max_spin_telem_period_us / 1000);
    Serial.print("ms");

    float rpm = lds.getCurrentScanFreqHz();
    if (rpm >= 0) {
      Serial.print(", LDS RPM ");
      Serial.print(rpm);
    }
    Serial.println();

    stat_sum_spin_telem_period_us = 0;
    stat_max_spin_telem_period_us = 0;
  }
  #endif
}

void publishTelem(unsigned long step_time_us) {
  struct timespec tv = {0};
  clock_gettime(CLOCK_REALTIME, &tv);
  telem_msg.stamp.sec = tv.tv_sec;
  telem_msg.stamp.nanosec = tv.tv_nsec;

  float joint_pos_delta[JOINTS_LEN];
  float step_time = 1e-6 * (float)step_time_us;

  for (unsigned char i = 0; i < MOTOR_COUNT; i++) {
    joint_pos[i]  = drive.getShaftAngle(i);
    joint_pos_delta[i] = joint_pos[i] - joint_prev_pos[i];
    joint_vel[i]  = joint_pos_delta[i] / step_time; 
    joint_prev_pos[i] = joint_pos[i];
  }

  calcOdometry(step_time_us, joint_pos_delta[MOTOR_LEFT],
    joint_pos_delta[MOTOR_RIGHT]);

  RCSOFTCHECK(rcl_publish(&telem_pub, &telem_msg, NULL));
  telem_msg.lds.size = 0;
  telem_msg.seq++;
}

void calcOdometry(unsigned long step_time_us, float joint_pos_delta_right,
  float joint_pos_delta_left) {

  if (step_time_us == 0)
    return;
  // https://automaticaddison.com/how-to-publish-wheel-odometry-information-over-ros/
  float distance_right = -joint_pos_delta_right * WHEEL_RADIUS;
  float distance_left = joint_pos_delta_left * WHEEL_RADIUS;

  // TODO use Runge-Kutta integration for better accuracy
  float average_distance = (distance_right + distance_left) * 0.5;
  float d_yaw = asin((distance_left - distance_right)*WHEEL_BASE_RECIP);

  // Average angle during the motion
  float average_angle = d_yaw*0.5 + telem_msg.odom_pos_yaw;
     
  if (average_angle > PI)
    average_angle -= TWO_PI;
  else if (average_angle < -PI)
    average_angle += TWO_PI;

  // Calculate the new pose (x, y, and theta)
  float d_x = cos(average_angle) * average_distance;
  float d_y = sin(average_angle) * average_distance;
 
  telem_msg.odom_pos_x += d_x;
  telem_msg.odom_pos_y += d_y;
  telem_msg.odom_pos_yaw += d_yaw;

  if (telem_msg.odom_pos_yaw > PI)
    telem_msg.odom_pos_yaw -= TWO_PI;
  else if (telem_msg.odom_pos_yaw < -PI)
    telem_msg.odom_pos_yaw += TWO_PI;

  float d_time = 1e-6 * (float)step_time_us;
  telem_msg.odom_vel_x = average_distance / d_time;
  telem_msg.odom_vel_yaw = d_yaw / d_time;
}

void lds_scan_point_callback(float angle_deg, float distance_mm, float quality,
  bool scan_completed) {
  return;

  static int i=0;

  if ((i++ % 20 == 0) || scan_completed) {
    Serial.print(i);
    Serial.print(' ');
    Serial.print(distance_mm);
    Serial.print(' ');
    Serial.print(angle_deg);
    if (scan_completed)
      Serial.println('*');
    else
      Serial.println();
  }
}

void lds_packet_callback(uint8_t * packet, uint16_t packet_length, bool scan_completed) {

  bool packet_sent = false;
  while (packet_length-- > 0) {
    if (telem_msg.lds.size >= telem_msg.lds.capacity) {
      spinTelem(true);
      packet_sent = true;
    }
    telem_msg.lds.data[telem_msg.lds.size++] = *packet;
    packet++;
  }

  if (scan_completed && !packet_sent)
    spinTelem(true); // Opional: reduce lag a little
}

void lds_motor_pin_callback(float value, LDS::lds_pin_t lds_pin) {
  /*
  Serial.print("LDS pin ");
  Serial.print(lds.pinIDToString(lds_pin));
  Serial.print(" set ");
  if (lds_pin > 0)
    Serial.print(value); // PWM value
  else
    Serial.print(lds.pinStateToString((LDS::lds_pin_state_t)value));
  Serial.print(", RPM ");
  Serial.println(lds.getCurrentScanFreqHz());
  */
  
  int pin = (lds_pin == LDS::LDS_MOTOR_EN_PIN) ?
    CONFIG::LDS_MOTOR_EN_PIN : CONFIG::LDS_MOTOR_PWM_PIN;

  if (value <= LDS::DIR_INPUT) {
    // Configure pin direction
    if (value == LDS::DIR_OUTPUT_PWM) {
      pinMode(pin, OUTPUT);
      ledcSetup(LDS_MOTOR_PWM_CHANNEL, LDS_MOTOR_PWM_FREQ, LDS_MOTOR_PWM_BITS);
      ledcAttachPin(pin, LDS_MOTOR_PWM_CHANNEL);
    } else
      pinMode(pin, (value == LDS::DIR_INPUT) ? INPUT : OUTPUT);
    return;
  }

  if (value < LDS::VALUE_PWM) // set constant output
    digitalWrite(pin, (value == LDS::VALUE_HIGH) ? HIGH : LOW);
  else { // set PWM duty cycle
    int pwm_value = ((1<<LDS_MOTOR_PWM_BITS)-1)*value;
    ledcWrite(LDS_MOTOR_PWM_CHANNEL, pwm_value);
  }
}

void spinPing() {
  unsigned long time_now_us = esp_timer_get_time();
  unsigned long step_time_us = time_now_us - ping_prev_pub_time_us;
  
  if (step_time_us >= ping_pub_period_us) {
    // timeout_ms, attempts
    rmw_ret_t rc = rmw_uros_ping_agent(1, 1);
    //int battery_level = analogRead(CONFIG::BAT_ADC_PIN);
    //Serial.print("Battery level ");
    //Serial.println(battery_level);
    ping_prev_pub_time_us = time_now_us;
    Serial.println(rc == RCL_RET_OK ? "Ping OK" : "Ping error");
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    lds.stop();
    drive.setRPM(MOTOR_RIGHT, 0);
    drive.setRPM(MOTOR_LEFT, 0);
    return;
  }

  lds.loop();
  
  // Process micro-ROS callbacks
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)), ERR_UROS_SPIN);

  spinTelem(false);
  spinPing();
  spinResetSettings();
  updateSpeedRamp(); // update ramp less frequently?
  drive.update();
}

unsigned int reset_settings_check_period_ms = 1000; // Check once a second
unsigned char button_pressed_seconds = 0;
unsigned long reset_settings_prev_check_time_ms = 0;

void resetSettings() {
  Serial.println("** Resetting settings **");
  resetWiFiSettings();
  blink(LONG_BLINK_MS, 5);
  Serial.flush();

  ESP.restart();
}

void spinResetSettings() {
  unsigned long time_now_ms = millis();
  unsigned long step_time_ms = time_now_ms - reset_settings_prev_check_time_ms;

  if (step_time_ms >= reset_settings_check_period_ms) {

    bool button_pressed = !digitalRead(0);
    if (button_pressed && button_pressed_seconds > CONFIG::RESET_SETTINGS_HOLD_SEC)
      resetSettings();

    button_pressed_seconds = button_pressed ? button_pressed_seconds + 1 : 0;
    reset_settings_prev_check_time_ms = time_now_ms;
  }
}

void resetTelemMsg() {
  telem_msg.seq = 0;
  telem_msg.odom_pos_x = 0;
  telem_msg.odom_pos_y = 0;
  telem_msg.odom_pos_yaw = 0;
  telem_msg.odom_vel_x = 0;
  telem_msg.odom_vel_yaw = 0;
  
  telem_msg.joint_pos.data = joint_pos;
  telem_msg.joint_pos.capacity = JOINTS_LEN;
  telem_msg.joint_pos.size = JOINTS_LEN;

  telem_msg.joint_vel.data = joint_vel;
  telem_msg.joint_vel.capacity = JOINTS_LEN;
  telem_msg.joint_vel.size = JOINTS_LEN;

  telem_msg.lds.data = lds_buf;
  telem_msg.lds.capacity = LDS_BUF_LEN;
  telem_msg.lds.size = 0;

  for (int i = 0; i < JOINTS_LEN; i++) {
    joint_pos[i] = 0;
    joint_vel[i] = 0; 
    joint_prev_pos[i] = 0;
  }
}

void syncRosTime() {
  const int timeout_ms = UROS_TIME_SYNC_TIMEOUT_MS;

  Serial.print("Syncing time ... ");
  RCCHECK(rmw_uros_sync_session(timeout_ms), ERR_UROS_TIME_SYNC);
  // https://micro.ros.org/docs/api/rmw/
  int64_t time_ms = rmw_uros_epoch_millis();
  
  if (time_ms > 0) {
    time_t time_seconds = time_ms/1000;
    time_t time_micro_seconds = (time_ms - time_seconds*1000)*1000; 
    
    // https://gist.github.com/igrr/d7db8a78170bf6981f2e606b42c4361c
    setenv("TZ", "GMT0", 1);
    tzset();
    
    // https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-time.c
    timeval epoch = {time_seconds, time_micro_seconds};
    if (settimeofday((const timeval*)&epoch, NULL) != 0)
      Serial.println("settimeofday() error");
    else
      Serial.println("OK");
  } else {
    Serial.print("rmw_uros_sync_session() failed, error code: ");
    Serial.println((int) time_ms);
  }
}

static inline void logMsgInfo(char* msg) {
  logMsg(msg, rcl_interfaces__msg__Log__INFO);
}

void logMsg(char* msg, uint8_t severity_level) {
  if (WiFi.status() != WL_CONNECTED) {
    rcl_interfaces__msg__Log msgLog;
    // https://docs.ros2.org/foxy/api/rcl_interfaces/msg/Log.html
    // builtin_interfaces__msg__Time stamp;
    struct timespec tv = {0};
    clock_gettime(CLOCK_REALTIME, &tv);
    msgLog.stamp.sec = tv.tv_sec;
    msgLog.stamp.nanosec = tv.tv_nsec;
    
    msgLog.level = severity_level;
    msgLog.name.data = (char*)UROS_NODE_NAME; // Logger name
    msgLog.name.size = strlen(msgLog.name.data);
    msgLog.msg.data = msg;
    msgLog.msg.size = strlen(msgLog.msg.data);
    //char fileName[] = __FILE__;
    msgLog.file.data = (char*)""; // Source code file name
    msgLog.file.size = strlen(msgLog.file.data);
    msgLog.function.data = (char*)""; // Source code function name
    msgLog.function.size = strlen(msgLog.function.data);
    msgLog.line = 0; // Source code line number
    RCSOFTCHECK(rcl_publish(&log_pub, &msgLog, NULL));
  }
  
  String s = "UNDEFINED";
  switch(severity_level) {
    case rcl_interfaces__msg__Log__INFO:
      s = "INFO";
      break;
    case rcl_interfaces__msg__Log__FATAL:
      s = "FATAL";
      break;
    case rcl_interfaces__msg__Log__DEBUG:
      s = "DEBUG";
      break;
    case rcl_interfaces__msg__Log__ERROR:
      s = "ERROR";
      break;
    case rcl_interfaces__msg__Log__WARN:
      s = "WARN";
      break;
  }
  Serial.print("LOG_");
  Serial.print(s);
  Serial.print(": ");
  Serial.println(msg);
}

void lds_info_callback(LDS::info_t code, String info) {
  Serial.print("LDS info ");
  Serial.print(lds.infoCodeToString(code));
  Serial.print(": ");
  Serial.println(info);
}

void lds_error_callback(LDS::result_t code, String aux_info) {
  if (code != LDS::ERROR_NOT_READY) {
    Serial.print("LDS error ");
    Serial.print(lds.resultCodeToString(code));
    Serial.print(": ");
    Serial.println(aux_info);
  }
}

void setupLDS() {
  lds.setScanPointCallback(lds_scan_point_callback);
  lds.setPacketCallback(lds_packet_callback);
  lds.setSerialWriteCallback(lds_serial_write_callback);
  lds.setSerialReadCallback(lds_serial_read_callback);
  lds.setMotorPinCallback(lds_motor_pin_callback);
  lds.setInfoCallback(lds_info_callback);
  lds.setErrorCallback(lds_error_callback);

  Serial.print("LDS RX buffer size "); // default 128 hw + 256 sw
  Serial.print(LdSerial.setRxBufferSize(1024)); // must be before .begin()
  uint32_t baud_rate = lds.getSerialBaudRate();
  Serial.print(", baud rate ");
  Serial.println(baud_rate);

  LdSerial.begin(baud_rate);
  while (LdSerial.read() >= 0);  

  lds.stop();
}

LDS::result_t startLDS() {  
  LDS::result_t result = lds.start();
  Serial.print("startLDS() result: ");
  Serial.println(lds.resultCodeToString(result));

  if (result < 0)
    Serial.println("WARNING: is LDS connected to ESP32?");

  return result;
}

void blink_error_code(int n_blinks) {
  unsigned int i = 0;
  while(i++ < ERR_REBOOT_BLINK_CYCLES){
    blink(LONG_BLINK_MS, 1);
    digitalWrite(CONFIG::LED_PIN, LOW);
    delay(SHORT_BLINK_PAUSE_MS);
    blink(SHORT_BLINK_MS, n_blinks);
    delay(LONG_BLINK_PAUSE_MS);

    while(!digitalRead(0)) {
      spinResetSettings();
    }
  }
}

void error_loop(int n_blinks){
  lds.stop();

  char buffer[40];
  sprintf(buffer, "Error code %d", n_blinks);  
  logMsg(buffer, rcl_interfaces__msg__Log__FATAL);

  blink_error_code(n_blinks);

  Serial.println("Rebooting...");
  Serial.flush();

  ESP.restart();
}
