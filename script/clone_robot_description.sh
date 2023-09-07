#!/bin/bash

# Clone the robot description (where the script resides) into a new one
#   ros2 run kaia_snoopy_description clone_robot_description.sh /ros_ws/src/kaia_fido_description
[[ -z "$1" ]] && { echo "Destination path missing" ; exit 1; }
[[ -d "$1" ]] && { echo "Destination path already exists" ; exit 1; }

clone_path=$1

# Get this script's path
SOURCE=${BASH_SOURCE[0]}
while [ -L "$SOURCE" ]; do
  DIR=$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )
  SOURCE=$(readlink "$SOURCE")
  [[ $SOURCE != /* ]] && SOURCE=$DIR/$SOURCE
done
DIR=$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )

# echo $DIR
cd $DIR && cd ..
# echo $PWD

# Get folder name without path
desc_name=${PWD##*/}
desc_name=${desc_name:-/}
# echo $desc_name

# Extract robot model name prefix
robot_name=${desc_name%"_description"}
# echo $robot_name

# echo $clone_path
# Extract clone robot model name prefix
clone_desc_name=${clone_path##*/}
clone_desc_name=${clone_desc_name:-/}
# echo $clone_desc_name
clone_name=${clone_desc_name%"_description"}
# echo $clone_name

[[ "$clone_name" == "$clone_desc_name" ]] && { echo "Syntax: /path/cloned_description" ; exit 1; }

echo "Cloning $robot_name to $clone_name"
cd ..
cp -rf "$desc_name/" "$1/"

cd $1
sed_arg_desc="s/$desc_name/$clone_desc_name/g"
sed_arg_name="s/$robot_name/$clone_name/g"
echo "Updating CMakeLists.txt"
sed -i $sed_arg_desc CMakeLists.txt

echo "Updating package.xml"
sed -i $sed_arg_desc package.xml

echo "Updating README.md"
sed -i $sed_arg_name README.md

cd rviz
echo "Renaming rviz/$robot_name.rviz to rviz/$clone_name.rviz"
mv "$robot_name.rviz" "$clone_name.rviz"

cd ../urdf
echo "Renaming urdf/$robot_name.urdf.xacro to urdf/$clone_name.urdf.xacro"
mv "$robot_name.urdf.xacro" "$clone_name.urdf.xacro"

echo "Updating urdf/$clone_name.urdf.xacro"
sed -i $sed_arg_name "$clone_name.urdf.xacro"

cd ../sdf
echo "Renaming sdf/$robot_name/ to sdf/$clone_name/"
mv $robot_name $clone_name

cd $clone_name
echo "Updating sdf/$clone_name/model.config"
sed -i $sed_arg_name model.config

echo "Updating sdf/$clone_name/model.sdf"
sed -i $sed_arg_name model.sdf

echo
echo "* TODO Finish cloning:"
echo "  *  Edit package.xml  - update author, website, description, version, email"
echo "  *  Edit README.md    - update description"
echo "  *  Edit $clone_desc_name/sdf/$clone_name/model.config - update author, description, version, email"
echo "* Mod the robot:"
echo "  *  Edit robot config - $clone_desc_name/config/"
echo "  *  Edit robot model  - $clone_desc_name/urdf/$clone_name.urdf, run urdf2sdf"
echo "* Run"
echo "    cd /ros_ws && colcon build --symlink-install --packages-select $clone_desc_name"

#xacro $2.urdf.xacro > $2.urdf
#gz sdf -p $2.urdf > $2.sdf
#sed_arg="s/_description\/sdf\/$2//g"
#sed -i $sed_arg $2.sdf
#rm $2.urdf
#mv $2.sdf ../sdf/$2/model.sdf
