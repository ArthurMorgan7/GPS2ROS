# Introduction
Accept data in GPGGA format through the serial port and convert it into topics in ROS for publishing

# Install
```bash
mkdir GPGGA2ROS_ws
cd GPGGA2ROS_ws
mkdir src
cd src
git clone 
cd ..
catkin_make

source ./devel/setup.bash
rosrun serialPort serialPort
```

# NOTICE
Default baud rate: 115200

Default command: log gpgga ontime 1


0:SOL_COMPUTED,
1:NARROW_INT,
2:11,
3:U,
4:5666936.4417,
5:707279.3875,
6:1063.8401,
7:-16.2712,
8:WGS84,
9:0.0135,
10:0.0084,
11:0.0173,
12:"AAAA",
13:1.000,
14:0.000,
8,8,8,8,0,01,0,03*a6d06321\r\n
