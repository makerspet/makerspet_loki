# Maker's Pet Loki robot model and configurations

## Modding the robot
- Clone `makerspet_loki` robot into a new `jacks_droid` robot.
```
cp -r /ros_ws/src/makerspet_loki /ros_ws/src/jacks_droid
rm -rf /ros_ws/src/jacks_droid/.git
```
- Edit `jacks_droid/package.xml` to update the author, website, description, version, email
- Edit `jacks_droid/README.md` to update the description
- Edit `jacks_droid/sdf/jacks_droid/model.config` to update the author, description, version, email
- If needed, edit the robot config files in `jacks_robot/config/`
- If needed, edit the robot model in `jacks_droid/urdf/robot.urdf.xacro`
- Regenerate your simulation model and compile your `jacks_droid` description
```
ros2 run kaiaai_gazebo urdf2sdf.sh /ros_ws/src/jacks_droid
cd /ros_ws
colcon build --symlink-install --packages-select jacks_droid
```

## Command cheat sheet

Operate a modded pet robot residing in `jacks_droid` repo:

```
# Launch the physical robot
ros2 launch kaiaai_bringup physical.launch.py robot_model:=jacks_droid

# Monitor robot's sensors
ros2 launch kaiaai_bringup rviz2.launch.py robot_model:=jacks_droid

# Launch the robot in a simulation - drive manually or let it self-drive
ros2 launch kaiaai_gazebo world.launch.py robot_model:=jacks_droid
ros2 run kaiaai_teleop teleop_keyboard robot_model:=jacks_droid
ros2 launch kaiaai_gazebo self_drive_gazebo.launch.py robot_model:=jacks_droid
ros2 launch kaiaai_bringup rviz2.launch.py robot_model:=jacks_droid

# Launch the robot in a simulation - create, save a map
ros2 launch kaiaai_gazebo world.launch.py robot_model:=jacks_droid
ros2 launch kaiaai_bringup cartographer.launch.py use_sim_time:=true robot_model:=jacks_droid
ros2 launch kaiaai_gazebo self_drive_gazebo.launch.py robot_model:=jacks_droid
ros2 run nav2_map_server map_saver_cli -f $HOME/my_map

# Launch the robot in a simulation - let it navigate automatically using an existing map
ros2 launch kaiaai_gazebo world.launch.py robot_model:=jacks_droid
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true map:=$HOME/my_map.yaml robot_model:=jacks_droid

# Inspect or edit robot's URDF model - useful when modding a robot
ros2 launch kaiaai_bringup inspect_urdf.launch.py robot_model:=jacks_droid
ros2 launch kaiaai_bringup edit_urdf.launch.py robot_model:=jacks_droid

# Convert URDF robot model file into SDF Gazebo simulation model file
ros2 run kaiaai_gazebo urdf2sdf.sh /ros_ws/src/jacks_droid
cd /ros_ws && colcon build --symlink-install --packages-select jacks_droid
```
