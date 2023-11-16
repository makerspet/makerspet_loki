// Scan publishes, WiFi AP works

//TODO detect micro-ros agent disconnect
//TODO debug /odom NaN
//TODO discover ROS2 PC automatically
//  #define RMW_UXRCE_TRANSPORT_UDP

#include "robot_config.h"
#include "util.h"
#include <WiFi.h>
#include <stdio.h>
#include <micro_ros_kaia.h>
#include <HardwareSerial.h>
#include "YDLidar.h"
//#include <sys/time.h>
//#include "time.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <kaiaai_msgs/msg/kaiaai_telemetry.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl_interfaces/msg/log.h>
#include <rmw_microros/rmw_microros.h>
//#include <rmw_microros/discovery.h>
#include "drive.h"
#include "ap.h"

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

HardwareSerial LdSerial(2); // TX 17, RX 16
YDLidar lds(scan_callback, serial_callback);

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
  Serial.print("setRPM() ");
  Serial.print(ramp_target_rpm_right);
  Serial.print(" ");
  Serial.println(ramp_target_rpm_left);

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

  initSPIFFS();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  pinMode(YD_MOTOR_SCTP_PIN, INPUT);
  pinMode(YD_MOTOR_EN_PIN, OUTPUT);
  enableLdsMotor(false);

  if (!initWiFi(getSSID(), getPassw())) {
    digitalWrite(LED_PIN, HIGH);
    ObtainWiFiCreds(spinResetSettings, UROS_ROBOT_MODEL);
    return;
  }

  set_microros_wifi_transports(getDestIP().c_str(), getDestPort().toInt());

  delay(2000);
  
  initRos();
  logMsgInfo((char*)"Micro-ROS initialized");
  
  if (initLDS() != 0)
    error_loop(ERR_LDS_INIT);
  
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

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator),
    ERR_UROS_EXEC);
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub,
    &twist_msg, &twist_sub_callback, ON_NEW_DATA), ERR_UROS_EXEC);

  resetTelemMsg();
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

    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    Serial.print('.'); // Don't use F('.'), it crashes code!!
    delay(250);
    spinResetSettings();
  }

  digitalWrite(LED_PIN, LOW);
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

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
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
    Serial.println("ms");
    stat_sum_spin_telem_period_us = 0;
    stat_max_spin_telem_period_us = 0;
  }
  #endif
}

void publishTelem(unsigned long step_time_us)
{
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

void scan_callback(uint8_t quality, float angle_deg,
  float distance_mm, bool startBit) {
  return;

  static int i=0;
  static int j=0;

  if (startBit) {
    i = 0;
    Serial.print('s');
    if (++j % 60 == 0)
      Serial.println();
  } else {
    if (i++ % 20 == 0) {
      Serial.print(i);
      Serial.print(' ');
      Serial.print(distance_mm);
      Serial.print(' ');
      Serial.println(angle_deg);
    }
  }
}

void serial_callback(char c) {
  if (telem_msg.lds.size >= telem_msg.lds.capacity)
    spinTelem(true);
  telem_msg.lds.data[telem_msg.lds.size++] = c;
}

void spinLDS() {
  if (!LdSerial.available())
    return;

  switch(lds.waitScanDot()) {
    case RESULT_OK:
      break;
    case RESULT_CRC_ERROR:
      Serial.println("CRC error");
      break;
    case RESULT_NOT_READY:
//      Serial.print(".");
      break;
    case RESULT_FAIL:
      Serial.println("FAIL");
      break;
    default:
      Serial.println("Unexpected result code");
  }
}

void spinPing() {
  unsigned long time_now_us = esp_timer_get_time();
  unsigned long step_time_us = time_now_us - ping_prev_pub_time_us;
  
  if (step_time_us >= ping_pub_period_us) {
    // timeout_ms, attempts
    rmw_ret_t rc = rmw_uros_ping_agent(1, 1);
    ping_prev_pub_time_us = time_now_us;
    Serial.println(rc == RCL_RET_OK ? "Ping OK" : "Ping error");
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    enableLdsMotor(false);
    drive.setRPM(MOTOR_RIGHT, 0);
    drive.setRPM(MOTOR_LEFT, 0);
    return;
  }

  spinLDS();
  
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
    if (button_pressed && button_pressed_seconds > RESET_SETTINGS_HOLD_SEC)
      resetSettings();

    button_pressed_seconds = button_pressed ? button_pressed_seconds + 1 : 0;
    reset_settings_prev_check_time_ms = time_now_ms;
  }
}

void resetTelemMsg()
{
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

int initLDS() {
  Serial.print("LDS RX buffer size "); // default 128 hw + 256 sw
  Serial.println(LdSerial.setRxBufferSize(1024)); // must be before .begin()
  lds.begin(LdSerial, LDS_SERIAL_BAUD);
  ledcSetup(0, 10000, 8);
  ledcAttachPin(YD_MOTOR_SCTP_PIN, 0);
//  pinMode(YD_MOTOR_SCTP_PIN, INPUT);
//  pinMode(YD_MOTOR_EN_PIN, OUTPUT);

  setMotorSpeed(YD_MOTOR_SPEED_DEFAULT);
  enableLdsMotor(false);
  while (LdSerial.read() >= 0) {};
  
  device_info deviceinfo;
  if (lds.getDeviceInfo(deviceinfo, 100) != RESULT_OK) {
    Serial.println(F("lds.getDeviceInfo() error!"));
    return -1;
  }

  int _samp_rate = 4;
  String model;
  float freq = 7.0f;
  switch (deviceinfo.model) {
    case 1:
      model = "F4";
      _samp_rate = 4;
      freq = 7.0;
      break;
    case 4:
      model = "S4";
      _samp_rate = 4;
      freq = 7.0;
      break;
    case 5:
      model = "G4";
      _samp_rate = 9;
      freq = 7.0;
      break;
    case 6:
      model = "X4";
      _samp_rate = 5;
      freq = 7.0;
      break;
    default:
      model = "Unknown";
  }

  uint16_t maxv = (uint16_t)(deviceinfo.firmware_version >> 8);
  uint16_t midv = (uint16_t)(deviceinfo.firmware_version & 0xff) / 10;
  uint16_t minv = (uint16_t)(deviceinfo.firmware_version & 0xff) % 10;
  if (midv == 0) {
    midv = minv;
    minv = 0;
  }

  Serial.print(F("Firmware version:"));
  Serial.print(maxv, DEC);
  Serial.print(F("."));
  Serial.print(midv, DEC);
  Serial.print(F("."));
  Serial.println(minv, DEC);

  Serial.print(F("Hardware version:"));
  Serial.println((uint16_t)deviceinfo.hardware_version, DEC);

  Serial.print(F("Model:"));
  Serial.println(model);

  Serial.print(F("Serial:"));
  for (int i = 0; i < 16; i++) {
    Serial.print(deviceinfo.serialnum[i] & 0xff, DEC);
  }
  Serial.println("");

  Serial.print(F("Sampling Rate:"));
  Serial.print(_samp_rate, DEC);
  Serial.println(F("K"));

  Serial.print(F("Scan Frequency:"));
  Serial.print(freq, DEC);
  Serial.println(F("Hz"));
  delay(100);

  device_health healthinfo;
  if (lds.getHealth(healthinfo, 100) != RESULT_OK) {
    Serial.println(F("lds.getHealth() error!"));
    return -1;
  } else {
    Serial.print(F("YDLIDAR running correctly! The health status: "));
    Serial.println(healthinfo.status == 0 ? F("OK") : F("bad"));
    if (lds.startScan() != RESULT_OK) {
      Serial.println(F("lds.startScan() error!"));
      return -1;
    } else {
//      isScanning = true;
      enableLdsMotor(true);
      Serial.println(F("lds.startScan() successful"));
      delay(1000);
    }
  }
  return 0;
}

void blink_error_code(int n_blinks) {
  unsigned int i = 0;
  while(i++ < ERR_REBOOT_BLINK_CYCLES){
    blink(LONG_BLINK_MS, 1);
    digitalWrite(LED_PIN, LOW);
    delay(SHORT_BLINK_PAUSE_MS);
    blink(SHORT_BLINK_MS, n_blinks);
    delay(LONG_BLINK_PAUSE_MS);

    while(!digitalRead(0)) {
      spinResetSettings();
    }
  }
}

void error_loop(int n_blinks){
  enableLdsMotor(false);

  char buffer[40];
  sprintf(buffer, "Error code %d", n_blinks);  
  logMsg(buffer, rcl_interfaces__msg__Log__FATAL);

  blink_error_code(n_blinks);

  Serial.println("Rebooting...");
  Serial.flush();

  ESP.restart();
}
