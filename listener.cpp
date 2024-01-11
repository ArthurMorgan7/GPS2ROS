#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gps/JWG.h"
#include "gps/NEG.h"
#include "gps/HEADING.h"
#include <iomanip>

void negCallback(const gps::NEGConstPtr& msg)
{
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(7)  // 保留小数点后7位
              << "时间："<< msg->stamp  << ";"
              << "东：" << msg->easting << ";"
              << "北：" << msg->northing << ";"
              << "高：" << msg->height << "\n";
}
void jwgCallback(const gps::JWGConstPtr& msg)
{
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(7)  // 保留小数点后7位
              << "时间："<< msg->stamp  << ";"
              << "纬度：" << msg->lat << "; "
              << "经度：" << msg->lon << "; "
              << "高度：" << msg->alt << "\n";
}
void headingCallback(const gps::HEADINGConstPtr& msg)
{
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(7)  // 保留小数点后7位
              << "时间："<< msg->stamp  << ";"
              << "偏航角：" << msg->heading << "\n";
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    
    // $GPGGA
    // ros::Subscriber sub = n.subscribe("RTK", 1000, jwgCallback);

    // #bestutma
    ros::Subscriber sub_neg = n.subscribe("RTK_NEG", 1000, negCallback);
    
    // #headinga
    ros::Subscriber sub_heading = n.subscribe("RTK_heading", 1000, headingCallback);
    
    ros::spin();
    return 0;
}

