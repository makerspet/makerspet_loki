# [Kaia.ai](https://kaia.ai) Loki pet robot model and configurations

## Modding Snoopy
- copy `kaia_loki_description` to `r2d2_description`, where `r2d2` a name you picked for your modded pet robot
- rename references files, folders and file contents to replace `kaia_loki` with `r2d2`, incuding `package.xml`,
`CMakeLists.txt`, `urdf/kaia_loki.urdf.xacro`, `sdf/kaia_loki`, `sdf/kaia_loki/model.*`
- edit parameters in `urdf/r2d2.urdf.xacro` to tweak your pet robot size
- generate SDF file as follows
```
cd /ros_ws/src/r2d2_description/urdf
ros2 run kaia_loki urdf2sdf.sh . r2d2
cd /ros_ws
colcon build --symlink-install --packages-select r2d2_description

ros2 launch kaia_gazebo world.launch.py description:=r2d2_description
```

## Command cheat sheet

```
# Launch the physical robot
ros2 launch kaia_bringup main.launch.py description:=kaia_loki_description

# Monitor robot's sensors
ros2 launch kaia_bringup rviz2.launch.py description:=kaia_loki_description

# Launch the robot in a simulation - drive manually or let it self-drive
ros2 launch kaia_gazebo world.launch.py description:=kaia_loki_description
ros2 run kaia_teleop teleop_keyboard description:=kaia_loki_description
ros2 launch kaia_gazebo self_drive_gazebo.launch.py description:=kaia_loki_description
ros2 launch kaia_bringup rviz2.launch.py description:=kaia_loki_description

# Launch the robot in a simulation - create, save a map
ros2 launch kaia_gazebo world.launch.py description:=kaia_loki_description
ros2 launch kaia_bringup cartographer.launch.py use_sim_time:=true description:=kaia_loki_description
ros2 launch kaia_gazebo self_drive_gazebo.launch.py description:=kaia_loki_description
ros2 run nav2_map_server map_saver_cli -f $HOME/my_map

# Launch the robot in a simulation - let it navigate automatically using an existing map
ros2 launch kaia_gazebo world.launch.py description:=kaia_loki_description
ros2 launch kaia_bringup navigation.launch.py use_sim_time:=true map:=$HOME/my_map.yaml description:=kaia_loki_description

# Inspect or edit robot's URDF model - useful when modding a robot
ros2 launch kaia_bringup inspect_urdf.launch.py description:=kaia_loki_description model:=my_model.urdf
ros2 launch kaia_bringup edit_urdf.launch.py description:=kaia_loki_description model:=my_model.urdf

# Convert URDF robot model file into SDF Gazebo simulation model file
ros2 run kaia_loki_description urdf2sdf.sh /ros_ws/src/kaia_loki_description/urdf/ r2d2
cd /ros_ws && colcon build --symlink-install --packages-select kaia_loki_description
```
