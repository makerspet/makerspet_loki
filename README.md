# Maker's Pet Loki DIY home robot

A 200mm DIY 3D-printable pet robot compatible with [Kaia.ai](https://kaia.ai) software [platform](https://github.com/kaiaai/). 3D print, build, run and mod your own home pet robot! Please learn more at [Maker's Pet](https://makerspet.com), [REMAKE.AI](https://remake.ai) and join our [FB group](https://www.facebook.com/groups/243730868651472/).

Please install these Arduino libraries (using Arduino Library Manager) before building this firmware:
- [micro_ros_kaia](https://github.com/kaiaai/micro_ros_arduino_kaiaai)
- [LDS](https://github.com/kaiaai/LDS/)
- [PID_Timed](https://github.com/kaiaai/arduino_pid_timed)
- ESPAsyncWebSrv including AsyncTCP, ESPAsyncTCP

Supported laser distance scan sensors:
- YDLIDAR X4, X3, X3-PRO, X2/X2L
- Neato XV11
- LDS02RR from Xiaomi 1st gen vacuum cleaners (~$15 used off AliExpress including shipping)
- RPLIDAR A1

![Loki cropped](https://github.com/makerspet/makerspet_loki/assets/143911662/9e3857b0-df87-4fda-9d94-bfc53fed399d)

## Assembly instructions video
<a href="http://www.youtube.com/watch?feature=player_embedded&v=WPB2B1DPf_s" target="_blank">
 <img src="http://img.youtube.com/vi/WPB2B1DPf_s/maxresdefault.jpg" alt="Watch the assembly instructions video" width="720" height="405" border="10" />
</a>

## PC, firmware setup instructions video
<a href="http://www.youtube.com/watch?feature=player_embedded&v=XOc5kCE3MC0" target="_blank">
 <img src="http://img.youtube.com/vi/XOc5kCE3MC0/maxresdefault.jpg" alt="Watch the one-time PC setup, firmware upload instructions video" width="720" height="405" border="10" />
</a>

## Bring-up instructions video
<a href="http://www.youtube.com/watch?feature=player_embedded&v=L_XbkA4pwRc" target="_blank">
 <img src="http://img.youtube.com/vi/L_XbkA4pwRc/maxresdefault.jpg" alt="Watch the bring-up instructions video" width="720" height="405" border="10" />
</a>

## 3D printing instructions
<a href="http://www.youtube.com/watch?feature=player_embedded&v=4k6W1QyJMMw" target="_blank">
 <img src="http://img.youtube.com/vi/L_XbkA4pwRc/maxresdefault.jpg" alt="Watch the bring-up instructions video" width="720" height="405" border="10" />
</a>

## Arduino ESP32 breakout board setup instructions
<a href="http://www.youtube.com/watch?feature=player_embedded&v=zizGI8MjANU" target="_blank">
 <img src="http://img.youtube.com/vi/zizGI8MjANU/maxresdefault.jpg" alt="Watch the Arduino ESP32 breakout board setup instructions video" width="720" height="405" border="10" />
</a>

## Features
- room mapping using a 360-degree laser distance sensor (ROS2-based).
- autonomous self-driving (ROS2-based).
- code your character, skills (work in progress)
- 3D-printable, 200mm round base
- Arduino ESP32 micro-controller

## Instructions
- 3D printing [STL](https://github.com/makerspet/3d_models/tree/main/loki_200mm/stl), [3MF](https://github.com/makerspet/3d_models/tree/main/loki_200mm/3mf)
- Arduino ESP32 [firmware](https://github.com/kaiaai/firmware)
- PC end user and developer [setup](https://github.com/kaiaai/docker)
- KiCad [schematic, PCB](https://github.com/makerspet/pcb)
- Software setup, configuration [instructions](https://www.youtube.com/playlist?list=PLOSXKDW70aR8SA16wTB0ou9ClKhv7micy)
- Fusion 360 3D CAD [design files](https://github.com/makerspet/3d_models/tree/main/loki_200mm/fusion360)
- ROS2 software mod [instructions](https://github.com/makerspet/makerspet_loki/tree/main/urdf)

## Open-source design
Authored in Fusion 360. Printed using a Prusa MK3.5S.
