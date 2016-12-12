lucia_turtlebot
===============

This repo contains launch + config files to bring up multiple turtlebots
including navigation in the Lucia arena, both in simulation and (later) on the
real robots.

Installation
------------

Install ROS prerequisites:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep python-wstool ros-indigo-catkin git build-essential cmake
sudo rosdep init
rosdep update
```

Install the [Las Vegas Reconstruction toolkit](https://github.com/lasvegasrc/Las-Vegas-Reconstruction/):

```bash
sudo apt-get install -qq -y libfreenect-dev libopencv-dev libflann-dev libeigen3-dev libvtk5-dev libvtk5.8-qt4 python-vtk libvtk-java libboost-all-dev freeglut3-dev libxmu-dev libusb-1.0.0-dev
mkdir -p ~/lucia_software
cd ~/lucia_software
git clone https://github.com/lasvegasrc/Las-Vegas-Reconstruction.git
mkdir -p Las-Vegas-Reconstruction/build
cd Las-Vegas-Reconstruction/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install
```

Install the 3D Toolkit [3DTK](http://slam6d.sourceforge.net/):

```bash
sudo apt-get install -qq -y libzip-dev libann-dev libsuitesparse-dev libnewmat10-dev subversion
cd ~/lucia_software
svn checkout svn://svn.code.sf.net/p/slam6d/code/trunk slam6d-code
mkdir -p slam6d-code/build
cd slam6d-code/build
rm ../3rdparty/CMakeModules/FindCUDA.cmake
cmake -DWITH_FBR=ON ..
cmake -DWITH_FBR=ON ..
make
```

Be sure to make the cmake -DWITH_FBR ... call twice to get the software compiled.

Create a Catkin workspace, clone and build our ROS stacks:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
wstool init -j4 . https://raw.githubusercontent.com/uos/uos_rosinstalls/master/lucia2016-indigo.rosinstall

# Use rosdep to install all dependencies (including ROS itself)
sudo apt-get install -qq -y linux-image-extra-$(uname -r)  # workaround for ros-indigo-realsense-camera
rosdep install --from-paths ./ -i -y --rosdistro indigo
sudo apt-get install -qq -y ros-indigo-rviz ros-indigo-rosbash

source /opt/ros/indigo/setup.bash
catkin_init_workspace
cd ~/catkin_ws

catkin_make -DCMAKE_BUILD_TYPE=Release
```

If you have `rosjava` installed, you may have to repeat the `catkin_make`
command several times until it succeeds.
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
source ~/catkin_ws/devel/setup.bash
genjava_message_artifacts
catkin_make -DCMAKE_BUILD_TYPE=Release
```

If you have `rosjava` installed, you may have to repeat the `catkin_make`
command several times until it succeeds.


Setting up your Eclipse workspace for the MetaCSP tutorial (Wed)
----------------------------------------------------------------

```bash
cd ~/catkin_ws/src/lucia16/metacsp_tutorial
./gradlew eclipse
```

Go to Eclipse:

- make a new Java Project
- unclick "Use default location"
- Browse to ~/catkin_ws/src/lucia16/metacsp_tutorial/multi_robot_active_perception/
- Click Finish


Instructions for the OWL-DL tutorial (Tue)
------------------------------------------

```bash
roscd lvr_plane_classifier/
git pull
roscore

# start rviz in a new terminal:
rosrun rviz rviz -d $(rospack find lvr_plane_classifier)/rviz/semantic_furniture_recognition.rviz

# start semantic_furniture_classifier_node in a new terminal:
roscd semantic_furniture_classifier/semantic_furniture_classifier_project/
build/install/semantic_furniture_classifier_project/bin/semantic_furniture_classifier_project com.github.semantic_furniture_classifier.semantic_furniture_classifier_project.SemanticFurnitureClassifierNode

# start lvr_plane_classifier_node in a new terminal:
roscd lvr_plane_classifier/
wget http://kos.informatik.uos.de/lucia_data/tableandchairs.zip
unzip tableandchairs.zip
mkdir -p /tmp/tableandchairs
rosrun lvr_plane_classifier lvr_plane_classifier_node tableandchairs/tableandchairs1.ply

# instead of restarting the lvr_plane_classifier_node over and over, you can later run this to save time:
roscd lvr_plane_classifier/
rosbag play --clock -d 1 tableandchairs/tableandchairs1.bag
```

* Download [Protégé 5.1.0](http://protege.stanford.edu/products.php#desktop-protege), unzip, start `run.sh`.
* Enable Window -> Tabs -> SWRL Tab.
* Open `semantic_furniture_classifier/semantic_furniture_classifier_project/owl/furniture.owl`.
* Look around in the "Entities" tab. You don't need to change anything here.
* Open the SWRL tab. Your goal is to add rules to classify a chair. For this,
  you need to add two rules: one that classifies certain HorizontalPlanes as
  ChairSeatPlanes, and one that classifies certain VerticalPlanes as
  ChairBackrestPlanes. If you want to take measurements, you can either use the
  "Measure" tool in RViz, or simply take a ruler to the actual chairs.
* Save the ontology file.
* Restart `semantic_furniture_classifier_node` and `rosbag`, and look at the
  RViz markers. When two ChairSeatPlane labels appear, you have succeeded! :)


Running in Gazebo
-----------------

```bash
roslaunch lucia_launch lucia_gazebo.launch
```

Now, you can use the "2D Nav Goal" tool in RViz to send navigation goals to the
first turtlebot. If the robot moves to the goal position, you have everything
set up correctly. You can command the other two turtlebots by opening "Panels"
-> "Tool Properties" and changing the "2D Nav Goal" topic.

You can disable specific turtlebots like this (for example, to speed up the
Gazebo simulation):

```bash
roslaunch lucia_launch lucia_gazebo.launch turtlebot2:=false turtlebot3:=false
```

To start FLAP4CAOS active perception, run this:

```bash
ROS_NAMESPACE=turtlebot1 rosrun race_object_search object_search_manager_test p lucia_area_sw 0.4 lucia_area_nw 0.4 lucia_area_e 0.4 min_p_succ 0.05
```


Trying out your code with the Turtlebots
----------------------------------------

### Relevant addresses and other info

- Basestation: 10.0.0.200
- Turtlebot laptops:

  * perception1 (10.0.0.211 iran/admin123)
  * perception2 (10.0.0.212 robot/robot)
  * perception3 (10.0.0.213 robot/robot)

### Instructions

- Start the following on basestation

```bash
roscore
roslaunch lucia_launch rviz.launch
```

- **For the FLAP4CAOS tutorial (Tue):** make sure your code is pushed to your
  group's Git repo, checked out and compiled on perception1-3

- Start the following on perception1-3

```bash
roslaunch lucia_launch lucia_robot.launch robot:=turtlebot1
roslaunch lucia_launch lucia_robot.launch robot:=turtlebot2
roslaunch lucia_launch lucia_robot.launch robot:=turtlebot3
```

- Unplug the turtlebots you are using, place them in the arena and provide pose
  estimate for each via Rviz
- Send a 2D nav goal for each robot
- Start the following on perception1-3

```bash
roslaunch lucia_launch perception_laptop.launch turtlebot1:=true
roslaunch lucia_launch perception_laptop.launch turtlebot2:=true
roslaunch lucia_launch perception_laptop.launch turtlebot3:=true
```

- **For the FLAP4CAOS tutorial (Tue):** also start the following on perception1-3

```bash
ROS_NAMESPACE=turtlebot1 rosrun race_object_search object_search_manager_test p lucia_area_sw 0.4 lucia_area_nw 0.4 lucia_area_e 0.4 min_p_succ 0.05
ROS_NAMESPACE=turtlebot2 rosrun race_object_search object_search_manager_test p lucia_area_sw 0.4 lucia_area_nw 0.4 lucia_area_e 0.4 min_p_succ 0.05
ROS_NAMESPACE=turtlebot3 rosrun race_object_search object_search_manager_test p lucia_area_sw 0.4 lucia_area_nw 0.4 lucia_area_e 0.4 min_p_succ 0.05
```

- **For the MetaCSP tutorial (Wed):** Connect your laptop to LuciaSchool2 network (password on the whiteboard)
- **For the MetaCSP tutorial (Wed):** add the following to your `.bashrc` and source it

```bash
ROS_MASTER_URI=http://10.0.0.200:11311
ROS_HOSTNAME=<your-IP-address>
```


Package overview
----------------

### lucia_launch

This ROS package contains all top-level launch files:

- `lucia_gazebo.launch` - Brings up everything on 3 Turtlebots in Gazebo.
- `lucia_robot.launch` + `perception_laptop.launch` - Same as above for physical robots.

### lucia_turtlebot_description

URDF model of the modified Turtlebots used in the Lucia school (i.e., regular
Turtlebots with a pole and second Xtion added).

### lucia_gazebo_worlds

Gazebo world, model and launch files for the Lucia arena.

### multi_turtlebot_navigation

Launch files to bring up navigation (move_base, amcl) on multiple turtlebots.
