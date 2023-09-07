# [Kaia.ai](https://kaia.ai) ready-to-use pet robot models

[Kaia.ai](https://kaiaai) is a platform for 3D-printable pet robots. Please sign up for an early launch invite [here](https://remake.ai).

This repo contains "off-the-shelf", ready-to-use Kaia.ai robot models as a ROS2 meta-package.
- 3D printing files for these models are [here](https://github.com/kaiaai/3d_printables/)
- hardware kits (motors, laser scanner, wheels, ESP32, sensors, servos, Raspberry Pi Pico) will be available for purchase at [Kaia.ai](https://kaia.ai)
- electronics for these models [here](https://github.com/kaiaai/electronics/)
- software stack [here](https://github.com/kaiaai/kaia)

## Mod a Kaia.ai robot - an example

### Run Kaia.ai developer Docker image
Executed from a Windows or Linux shell:
```
docker run --name kaia-ros-dev-humble -it --rm -p 8888:8888/udp -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 kaiaai/kaia-ros-dev:humble
```
### Clone an existing package
Let's clone `kaia_snoopy` robot to `waldo`:
```
ros2 run kaia_snoopy_description clone_robot_description.sh /ros_ws/src/waldo_description
cd /ros_ws && colcon build --symlink-install --packages-select waldo_description
```
Edit `waldo` files:
- `waldo_description/package.xml` to update author, website, description, version, email.
- `waldo_description/README.md` to update description.
- `waldo_description/sdf/kaia_fido/model.config` to update author, description, version, email

Mod your robot:
- robot size, speed limits, etc. in `waldo_description/config/*.yaml`
- robot size, appearance in `waldo_description/urdf/waldo.urdf.xacro`
- `waldo_description/config/`
- run commands below to convert URDF model to SDF for simulation (TODO automate)
```
ros2 run waldo_description urdf2sdf.sh /ros_ws/src/waldo_description/urdf/ waldo
cd /ros_ws
colcon build --symlink-install --packages-select waldo_description
. install/setup.bash
```

Save your robot to GitHub:
- log in (or sign up) to GitHub and create a `waldo_description` repository
- make sure you have created a "classic token" as your password (TODO tutorial)
- run commands below to upload `waldo_description` to your GitHub repo
```
git config --global user.email "your-email@example.com"
git config --global user.name "Your Name"

ros2 run kaia_bringup upload_robot_description_github.sh /ros_ws/src/waldo_description your-github-user-name
```

### Inspect your modded model - URDF
```
ros2 launch kaia_bringup inspect_urdf.launch.py description:=waldo_description
```

### Monitor your modded robot in action
```
ros2 launch kaia_bringup rviz2.launch.py description:=waldo_description
```

## Publish your robot
- Create a Github repo containing your robot's description following instructions above.
- Add `kaia-ai-robot` hashtag to your repo - browse to your repo, click About and type in the tag.

Your robot will show up automatically in the [list of robots](https://github.com/topics/kaia-ai-robot) compatible with Kaia.ai software platform.

## Advanced: creating multiple robots
If you have more than one robot, it may be convenient to keep those robot in the same Git repository,
so they can be installed using a single command. In this case, please create a ROS2 meta-package similar to `kaia_descriptions`, populate that meta-package with your own robot description packages and update its `package.xml` to include all descriptions.

For starters, go ahead and
- if necessary, clone [kaia_descriptions](https://github.com/kaiaai/kaia_descriptions)
- copy the `kaia_descriptions` GitHub repo as a starter templage
- pick a unique prefix for your robot descriptions. For example, this could be your GitHub usename
- rename the robot description packages to your own robot names as shown below - and delete those description packages that you won't be using
- edit each `package.xml` and `CMakeLists.txt`
- run `colcon build` to compile your new robot package(s) one by one
- push your new repo to GitHub - let's assume your GitHub usesrname is `galaxy42`
```
# TODO automate this with CLI
# cd /ros_ws/src && git clone https://github.com/kaiaai/kaia_descriptions
cd /ros_ws/src
cp -r kaia_descriptions galaxy42_descriptions
cd galaxy42_descriptions
mv kaia_snoopy_description waldo_description
mv kaia_loki_description fido_description
rm -rf kaia_snoopy_description
# Edit files in galaxy42_descriptions/galaxy42_descriptions: package.xml, CMakeLists.txt
# Edit files in galaxy42_descriptions/waldo_description: package.xml, CMakeLists.txt, etc.
# Edit files in galaxy42_descriptions/fido_description: package.xml, CMakeLists.txt, etc.
cd /ros_ws
colcon build --packages-select waldo_description
colcon build --packages-select fido_description
cd /ros_ws/src/galaxy42_descriptions
rm -rf .git
git init -b master
git add .
git commit -m "Created waldo, fido robot descriptions"
git remote add origin https://github.com/galaxy42/galaxy42_descriptions
git remote -v
# Open your browser, log in to GitHub and create galaxy42_descriptions repo
git push origin master
```

## Hints
- pick your robot's model name to be fairly rare, yet not unique. For example, `Jack` would not be a good name
for your robot because someone - a human or TV, radio, smartphone computer - may utter the word `Jack` aloud while not
referring to the robot. Yet, the robot may interpret that utterance as being called upon and respond. A made-up name
like `Zutza` may be a poor choice as well because the robot's speech recognition software may not have such made-up
word in its vocabulary. Of course, you can change your robot's name if there is a problem, but robot's model name
is not easy to change once it is published - and most users might just keep calling the robot by its model name.
