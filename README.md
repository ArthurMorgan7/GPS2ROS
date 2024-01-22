# Introduction
Accept data in BESTUTM/HEADINGA format through the serial port and convert it into topics in ROS for publishing.


# Install

In order to convert the RTK message into NovatelUtmPosition which supported by ROS, you need install novatel gps driver:

```bash
sudo apt-get install ros-${ROS_DISTRO}-novatel-gps-driver
```

You may need to modify the USB permissions:

```bash
sudo chmod 777 /dev/ttyUSB0
```

Then you can compile and run the program:

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

Simulate a receiver:

```bash
rosrun gps listener
```

# Novatel receiver configuration

**It is recommended to use the novatel client under Windows for configuration, and then accept it on other platforms (such as Linux) without shutting down.**

First, check the RTK state, the GPS quality must be 4(better)/5:

```bash
log gpgga once
```

Then you can configure the Novatel receiver:

```bash
log com1 bestutma ontime 1
log com1 headinga onchanged 
log com2 gpgga ontime 1
serialconfig com2 115200
interfacemode com2 rtcmv3 novatel on
savecomfig
```
