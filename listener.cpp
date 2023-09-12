// 这个读取只是模拟ROS工程中接受数据的一小部分
// 仅实现读取功能

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serialPort/GPS.h"
#include <iomanip>
void chatterCallback(const serialPort::GPSConstPtr& msg)
{
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(7)  // 保留小数点后7位
    << "纬度：" << msg->lat 
    << " 经度：" << msg->lon << "\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("GPS", 1000, chatterCallback);
    ros::spin();
    return 0;
}

