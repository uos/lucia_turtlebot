lucia_turtlebot
===============

This repo contains launch + config files to bring up multiple turtlebots
including navigation in the Lucia arena, both in simulation and (later) on the
real robots.

Installation
------------

Install prerequisites:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep ros-indigo-catkin git build-essential cmake
sudo rosdep init
rosdep update
```

Create a Catkin work space, clone and build our ROS stack:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
catkin_init_workspace
git clone <this package>
cd lucia_turtlebot

# Use rosdep to install all dependencies (including ROS itself)
rosdep install --from-paths ./ -i -y --rosdistro indigo

source /opt/ros/indigo/setup.bash
cd ~/catkin_ws

catkin_make -DCMAKE_BUILD_TYPE=Release
```


Running everything in Gazebo
----------------------------

```bash
roslaunch lucia_launch lucia_gazebo.launch
```


Package overview
----------------

### lucia_launch

This ROS package contains all top-level launch files:

- `lucia_gazebo.launch` - Brings up everything on 3 Turtlebots in Gazebo.
- **TODO:** `lucia_robot.launch` - Same as above for physical robots.

### lucia_gazebo_worlds

Gazebo world, model and launch files for the Lucia arena.

### multi_turtlebot_navigation

Launch files to bring up navigation (move_base, amcl) on multiple turtlebots.
