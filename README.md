# Introduction
Accept data in GPGGA/BESTUTM format through the serial port and convert it into topics in ROS for publishing



# Install
```bash
mkdir GPGGA2ROS_ws
cd GPGGA2ROS_ws
mkdir src
cd src
git clone https://github.com/ArthurMorgan7/GPS2ROS.git
cd ..
catkin_make

source ./devel/setup.bash
rosrun serialPort pubneg
```

# NOTICE

```
Default topic: "/GPS"
Default baud rate: 115200
Default command: log bestutm ontime 1
Optional command:log gpgga ontime 1
```
