# Introduction
Accept data in BESTUTM/HEADINGA format through the serial port and convert it into topics in ROS for publishing.


# Install

In order to convert the RTK message into NovatelUtmPosition which supported by ROS, you need install novatel gps driver:

```bash
sudo apt-get install ros-${ROS_DISTRO}-novatel-gps-driver
```

Then you can compile and run the program

```bash
mkdir GPS2ROS_ws
cd GPS2ROS_ws
mkdir src
cd src
git clone https://github.com/ArthurMorgan7/GPS2ROS.git
cd ..
catkin_make
source ./devel/setup.bash
roslaunch gps start.launch
```
