#include <iomanip>
#include <iostream>
#include <novatel_gps_msgs/NovatelUtmPosition.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gps/NEG.h"
#include "gps/HEADING.h"

using namespace std;

void negCallback(const gps::NEGConstPtr& msg)
{
    cout << setiosflags(ios::fixed) << setprecision(7)  // 保留小数点后7位
              << "时间："<< msg->stamp  << ";"
              << "东：" << msg->easting << ";"
              << "北：" << msg->northing << ";"
              << "高：" << msg->height << "\n";
}
void headingCallback(const gps::HEADINGConstPtr& msg)
{
    cout << setiosflags(ios::fixed) << setprecision(7)  // 保留小数点后7位
              << "时间："<< msg->stamp  << ";"
              << "偏航角：" << msg->heading << "\n";
}
void rtkCallback(const novatel_gps_msgs::NovatelUtmPositionConstPtr& msg)
{
    cout << setiosflags(ios::fixed) << setprecision(7)  // 保留小数点后7位
              << "时间："<< msg->header.stamp  << ";"
              << "北：" << msg->northing << ";"
              << "东：" << msg->easting << ";"
              << "高：" << msg->height << ";"
              << "偏航角：" << msg->diff_age << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;

    // #bestutma
    //ros::Subscriber sub_neg = nh.subscribe("RTK_NEG", 1000, negCallback);
    
    // #headinga
    // ros::Subscriber sub_heading = nh.subscribe("RTK_heading", 1000, headingCallback);

    // RTK
    ros::Subscriber sub_rtk = nh.subscribe("RTK", 1000, rtkCallback);
    
    ros::spin();
    return 0;
}

