Prerequisites:

Run

apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio

to install Gstreamer


Run

sudo apt update && sudo apt install -y cmake g++ wget unzip
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
unzip opencv.zip
mkdir -p build && cd build
cmake  ../opencv-4.x
cmake --build .

to install OpenCV


Run

sudo apt-get install ros-noetic-cv-bridge
sudo apt-get install ros-noetic-sensor-msgs
sudo apt-get install ros-noetic-image-transport
sudo apt-get install ros-noetic-roscpp

to install ROS packages


Change CMake build configuration to develop these applications that depend on LibTorch.

Open the CMakeLists.txt in both of gscam and description_v2_moveit. Change the absolute path to libtorch.
Just direct the path to catkin_ws 

set(CMAKE_PREFIX_PATH /home/user/catkin_ws/src/dependencies/libtorch)


Run

sudo apt-get install ros-noetic-octomap

to install OctoMap as stand-alone libraries with no ROS dependencies (so the package can also be used in a non-ROS setting).


Run

sudo apt install ros-noetic-moveit

to install moveit and its packages

description_v2 package: contains robot urdf and launch files which launches the robot in a gazebo simulation along with rgbd camera and controllable joints

description_v2_moveit package: using moveit to motion plan and control the robot (launch demo_gazebo.launch to launch rviz and gazebo and control the robot there)

copy package folders into src folder and run (in workspace's directory):

catkin_make

to make the workspace then (in workspace's directory):

source devel/setup.bash

launching: launch robot with camera and moveit planner:

roslaunch description_v2_moveit demo_gazebo.launch


Run the "publisher" node to deliver images from robot's camera to deep learning model:
in catkin_ws, change directory to description_v2_moveit by running:

roscd description_v2_moveit

Run the "publisher" node by running:

rosrun description_v2_moveit publisher


Run the "ros_node" node in the Gscam package to deliver images from laptop camera to deep learning model:
in catkin_ws, change directory to gscam by running:

roscd gscam

Run the "ros_node" node by running:

rosrun gscam ros_node

