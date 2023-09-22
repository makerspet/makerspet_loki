# [Maker's Pet](https://makerspet.com) makerspet_loki Pet robot model and configurations

## Modding the robot
- Clone `makerspet_loki` robot into a new `awesome_droid` robot.
```
ros2 run makerspet_loki clone_robot_description.sh /ros_ws/src/awesome_droid
```
- Edit `awesome_droid/package.xml` to update the author, website, description, version, email
- Edit `awesome_droid/README.md` to update the description
- Edit `awesome_droid/sdf/awesome_droid/model.config` to update the author, description, version, email
- If needed, edit the robot config files in `awesome_robot/config/`
- If needed, edit the robot model in `awesome_droid/urdf/awesome_droid.urdf`
- Regenerate your simulation model and compile your `awesome_droid` description
```
ros2 run kaiaai_gazebo urdf2sdf.sh /ros_ws/src/awesome_droid/urdf/ awesome_droid
cd /ros_ws
colcon build --symlink-install --packages-select awesome_droid
```

## Command cheat sheet

<details>
<summary>Operate a modded pet robot residing `awesome_droid` repo:</summary>

```
# Launch the physical robot
ros2 launch kaiaai_bringup main.launch.py description:=awesome_droid

# Monitor robot's sensors
ros2 launch kaiaai_bringup rviz2.launch.py description:=awesome_droid

# Launch the robot in a simulation - drive manually or let it self-drive
ros2 launch kaiaai_gazebo world.launch.py description:=awesome_droid
ros2 run kaiaai_teleop teleop_keyboard description:=awesome_droid
ros2 launch kaiaai_gazebo self_drive_gazebo.launch.py description:=awesome_droid
ros2 launch kaiaai_bringup rviz2.launch.py description:=awesome_droid

# Launch the robot in a simulation - create, save a map
ros2 launch kaiaai_gazebo world.launch.py description:=awesome_droid
ros2 launch kaiaai_bringup cartographer.launch.py use_sim_time:=true description:=awesome_droid
ros2 launch kaiaai_gazebo self_drive_gazebo.launch.py description:=awesome_droid
ros2 run nav2_map_server map_saver_cli -f $HOME/my_map

# Launch the robot in a simulation - let it navigate automatically using an existing map
ros2 launch kaiaai_gazebo world.launch.py description:=awesome_droid
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true map:=$HOME/my_map.yaml description:=awesome_droid

# Inspect or edit robot's URDF model - useful when modding a robot
ros2 launch kaiaai_bringup inspect_urdf.launch.py description:=awesome_droid model:=my_model
ros2 launch kaiaai_bringup edit_urdf.launch.py description:=awesome_droid model:=my_model

# Convert URDF robot model file into SDF Gazebo simulation model file
ros2 run awesome_droid urdf2sdf.sh /ros_ws/src/awesome_droid/urdf/ awesome_droid
cd /ros_ws && colcon build --symlink-install --packages-select awesome_droid
```
</details>
