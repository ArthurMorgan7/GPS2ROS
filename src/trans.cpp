#include <ros/ros.h> 
#include <std_msgs/String.h> 
#include <serial/serial.h>
#include <string>
#include <sstream>

using namespace std;

serial::Serial ser; 

int main(int argc, char** argv)
{
    std_msgs::String msg;
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    
    ros::Publisher msg_pub = nh.advertise<std_msgs::String>("GPS_msg",1000);
    try
    {
        ser.setPort("/dev/ttyUSB0");  // 设置实际对应的串口
        ser.setBaudrate(115200);      // 设置波特率
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 串口连接的超时时间(毫秒) 1000ms = 1s
        ser.setTimeout(to);
        ser.open();   // 打开串口
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open Serial Port !");   
        return -1;
    }
    
    if (ser.isOpen()){
        ROS_INFO_STREAM("Serial Port Open Successfully");
    }
    else{
        return -1;
    }

	ros::Rate loop_rate(2000);
    while (ros::ok())   
    {
        if (ser.available()>0)  // 有数据
        {
            msg.data = ser.read(ser.available());
            msg_pub.publish(msg);
            ROS_INFO("%s", msg.data.c_str());
        }    
        loop_rate.sleep();
    }

    return 0;
}
