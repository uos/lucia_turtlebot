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
sudo apt-get install -qq -y python-rosdep python-wstool ros-indigo-catkin git build-essential cmake
sudo rosdep init
rosdep update
```

Create a Catkin work space, clone and build our ROS stack:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
wstool init -j4 . https://raw.githubusercontent.com/uos/uos_rosinstalls/master/lucia2016-indigo.rosinstall

# Use rosdep to install all dependencies (including ROS itself)
rosdep install --from-paths ./ -i -y --rosdistro indigo
sudo apt-get install -qq -y ros-indigo-rviz

source /opt/ros/indigo/setup.bash
catkin_init_workspace
cd ~/catkin_ws

catkin_make -DCMAKE_BUILD_TYPE=Release
```

The generated `setup.bash` file has to be sourced in each terminal that uses
this catkin workspace. It's best to add it to the `.bashrc` so that it is
sourced automatically whenever a new terminal is opened:

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> .bashrc
```

Now close all running terminals and open a new one.


Updating
--------

To update to the latest version of all packages, do this:

```bash
cd ~/catkin_ws/src
wstool merge https://raw.githubusercontent.com/uos/uos_rosinstalls/master/lucia2016-indigo.rosinstall
wstool up
rosdep install --from-paths ./ -i -y --rosdistro indigo
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```


Running in Gazebo
-----------------

```bash
roslaunch lucia_launch lucia_gazebo.launch
```

Now, you can use the "2D Nav Goal" tool in RViz to send navigation goals to the
first turtlebot. If the robot moves to the goal position, you have everything
set up correctly. You can command the other two turtlebots by opening "Panels"
-> "Tool Properties" and changing the "2D Nav Goal" topic.


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
