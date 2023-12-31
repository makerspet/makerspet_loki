# Release notes

12/02/2023
- BREAKING ESP32 pinout assignment change to support the newly ESP32 breakout board
  - the new ESP32 breakout board works
  - the motor pin change fixes the "motor kick" upon ESP32 hard reboot
  - the LDS pin change fixes the LDS motor enabled by ESP32 upon hard reboot
  - MOT_FG_RIGHT has changed from GPIO27 to GPIO35_IN
  - LDS_MOT_EN has changed from GPIO12_OUT to GPIO19
  - MOT_CW_LEFT has changed from GPIO32 to GPIO23
- requires micro_ros_kaia Arduino library version 2.0.7-any.3 minimum
- added ROS2 parameter server
  - works successfully
- added lds.motor_speed parameter
  - controls the laser distance sensor motor speed
  - type double; set lds.motor=0 to stop LDS motor; set lds.motor=1.0 for maximum speed
  - set lds.motor=-1.0 for LDS default motor speed
- added minimum micro_ros_kaia library version check
  - Arduino build errors out at compile time if the library version is too old
- renamed some #define symbols from YDLidar-specific to generic LDS