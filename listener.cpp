// 这个读取只是模拟ROS工程中接受数据的一小部分
// 仅实现读取功能

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serialPort/JWG.h"
#include "serialPort/NEG.h"
#include <iomanip>

void negCallback(const serialPort::NEGConstPtr& msg)
{
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(7)  // 保留小数点后7位
              << "时间："<< msg->stamp  << ";"
              << "东：" << msg->easting << ";"
              << "北：" << msg->northing << ";"
              << "高：" << msg->height << "; "
              << "diff_age：" << msg->diff_age << "\n";
}
void jwgCallback(const serialPort::JWGConstPtr& msg)
{
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(7)  // 保留小数点后7位
              << "时间："<< msg->stamp  << ";"
              << "纬度：" << msg->lat << "; "
              << "经度：" << msg->lon << "; "
              << "高度：" << msg->alt << "\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    
    // $GPGGA
    // ros::Subscriber sub = n.subscribe("GPS", 1000, jwgCallback);

    // #bestutma
    ros::Subscriber sub = n.subscribe("GPS", 1000, negCallback);
    
    ros::spin();
    return 0;
}

