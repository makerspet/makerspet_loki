# [Kaia.ai ](https://kaia.ai) Arduino firmware

[Kaia.ai](https://kaiaai) is a platform for 3D-printable pet robots. Please sign up for an early launch invite [here](https://remake.ai).

This repo contains Arduino ESP32 firmware for [Kaia.ai](https://kaia.ai) home pet robots.
This firmware uses a Micro-ROS [library](https://github.com/kaiaai/micro_ros_arduino_kaia) for Arduino.

## Installation
- Download the Kaia.ai firmware project code from this [repo](https://github.com/kaiaai/arduino_fw)
- Install the Micro-ROS Kaia.ai library using Arduino Library Manager. Learn more [here](https://github.com/kaiaai/micro_ros_arduino_kaia)
- open the downloaded `kaia_esp32.ino` project file in your Arduino IDE
- In your Arduino IDE, configure Tools -> Board as "ESP32 Dev Module" and leave the board settings at their defaults
- Cick the Arduino IDE build button. The project should build successfully
- Upload the compiled firmware to your ESP32 Dev Module

## Modding the robot and its firmware
- Start with an existing Kaia.ai robot, e.g. [Snoopy](https://github.com/kaiaai/kaia_descriptions/)
- If needed, mod the robot's 3D-printable robot body. You can use [Snoopy's design](https://github.com/kaiaai/3D_printables/) as a starting point
- If needed, mod the robot's description (i.e. sofware configuration and model), see [Snoopy](https://github.com/kaiaai/kaia_descriptions/)
- If needed, mod the robot's firmware - use this repo as a starting point
