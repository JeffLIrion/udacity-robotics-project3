#!/bin/bash

SCRIPT_ROOT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
RUN_CATKIN=false


# 1. Overview
if [ "$CATKIN_RUN" == true ]; then
  sudo apt install ros-kinetic-navigation ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-amcl
fi

# 2. Simulation setup
mkdir -p "$SCRIPT_ROOT_DIRECTORY/catkin_ws"
cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws"

if [ ! -d "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src" ]; then
  git clone https://github.com/JeffLIrion/udacity-robotics-project2.git src
  rm -rf "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/.git"
  git add src
fi

cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src"
if [ "$CATKIN_RUN" == true ]; then
  catkin_init_workspace
fi

cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws"
if [ "$CATKIN_RUN" == true ]; then
  catkin_make
  source devel/setup.bash
fi


# 3. Map setup
mkdir -p "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/my_robot/maps"
if [ "$CATKIN_RUN" == true ]; then
  sudo apt install libignition-math2-dev protobuf-compiler
fi

cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src"
if [ "$CATKIN_RUN" == true ]; then
  rm -rf "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/pgm_map_creator"
  git clone https://github.com/udacity/pgm_map_creator.git

  cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws"
  catkin_make
else
  mkdir -p "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/pgm_map_creator/world"
fi

rm -f "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/pgm_map_creator/world/myworld.world"
cp "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/my_robot/worlds/myworld.world" "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/pgm_map_creator/world/myworld.world"
sed -i 's|</world>|<plugin filename="libcollision_map_creator.so" name="collision_map_creator"/>\n  </world>|g' "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/pgm_map_creator/world/myworld.world"
git add "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/pgm_map_creator/world/myworld.world"

# Create the PGM map
cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws"
#gzserver src/pgm_map_creator/world/myworld.world
#roslaunch pgm_map_creator request_publisher.launch

# If the map is cropped, edit the settings in `pgm_map_creator/launch/request_publisher.launch`

# Add the map to your package
mkdir -p "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/my_robot/maps"
if [ -f "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/pgm_map_creator/maps/map.pgm" ]; then
  cp "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/pgm_map_creator/maps/map.pgm" "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/my_robot/maps/myworld.pgm"
  git add "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/my_robot/maps/myworld.pgm"
fi

# Create a YAML file for the map
if [ ! -f "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/my_robot/maps/myworld.yaml" ]; then
(
cat <<-EOF
image: "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/my_robot/maps/myworld.pgm"
resolution: 0.01
origin: [-15.0, -15.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
EOF
) > "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/my_robot/maps/myworld.yaml"

git add "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/my_robot/maps/myworld.yaml"
fi


# 5. AMCL Launch file
if [ ! -f "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/my_robot/launch/amcl.launch" ]; then
(
cat <<-EOF
<launch>
  <!-- TODO: Add nodes here -->
</launch>
EOF
) > "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/my_robot/launch/amcl.launch"

git add "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/my_robot/launch/amcl.launch"
fi


# 8. AMCL Launch file: base move node

cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/my_robot"

if [ ! -d "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/my_robot/config" ]; then
  wget https://s3-us-west-1.amazonaws.com/udacity-robotics/Resource/where_am_i/config.zip
  unzip config.zip
  rm config.zip
  rm -rf __MACOSX
  mkdir -p "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/my_robot/config"
  mv *.yaml "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/my_robot/config/"

  cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/my_robot/config/"
  sed -i "s|udacity_bot|my_robot|g" *.yaml
  git add *.yaml
fi  


# 9. Teleop package (optional)

if [ "$CATKIN_RUN" == true ]; then
  cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src"
  git clone https://github.com/ros-teleop/teleop_twist_keyboard
  cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws"
  catkin_make
  source devel/setup.bash
fi

# To run the teleop script:
#   rosrun teleop_twist_keyboard teleop_twist_keyboard.py


# Create a main package with a main.launch
if [ "$CATKIN_RUN" == true ]; then
  cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src"
  catkin_create_pkg main

  # Build the package
  cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws"
  catkin_make
fi


# Create and edit the main.launch file
mkdir -p "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/main/launch"

if [ ! -f "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/main/launch/main.launch" ]; then
(
cat <<- EOF
<?xml version="1.0"?>
<launch>
  <include file="\$(find my_robot)/launch/world.launch"/>
  <include file="\$(find my_robot)/launch/amcl.launch"/>
</launch>
EOF
) > "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/main/launch/main.launch"

git add "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/main/launch/main.launch"
fi
