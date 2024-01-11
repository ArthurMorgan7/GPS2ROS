#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gps/NEG.h"
#include "gps/HEADING.h"
#include <iomanip>
#include <novatel_gps_msgs/NovatelUtmPosition.h>
#include <ros/time.h>

using namespace std;

// 全局变量
novatel_gps_msgs::NovatelUtmPosition rtk;
double time_neg=0, time_heading=0;

// 函数
void negCallback(const gps::NEGConstPtr& msg);
void headingCallback(const gps::HEADINGConstPtr& msg);

///////////////////////////////// main ///////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;

    // #bestutma
    ros::Subscriber sub_neg = nh.subscribe("RTK_NEG", 1000, negCallback);
    
    // #headinga
    ros::Subscriber sub_heading = nh.subscribe("RTK_heading", 1000, headingCallback);

    // NovatelUtmPosition.msg
    ros::Publisher pub_rtk = nh.advertise<novatel_gps_msgs::NovatelUtmPosition>("RTK",1000);

    
    
    ros::Rate loop_rate(5);    
    while (ros::ok()) 
    {
        ros::spinOnce();
        if(floor(time_neg) == floor(time_heading)){
            rtk.header.stamp = ros::Time().fromSec(time_neg);
            pub_rtk.publish(rtk);
            // cout << "rtk.diff_age:" <<  rtk.diff_age << "\t";
            // cout << "rtk.northing:" <<  rtk.northing << "\t";
            // cout << "rtk.easting:" <<  rtk.easting << "\t";
            // cout << "rtk.height:" <<  rtk.height << endl;
        }
        loop_rate.sleep();
    }
    return 0;
}

///////////////////////////////////////////////////////////////////////
void negCallback(const gps::NEGConstPtr& msg)
{
    time_heading = msg->stamp;
    rtk.height = msg->height;
    rtk.easting = msg->easting;
    rtk.northing = msg->northing;
}

void headingCallback(const gps::HEADINGConstPtr& msg)
{
    time_neg = msg->stamp;
    cout << "msg->heading" << msg->heading  <<endl;
    rtk.diff_age = msg->heading;
}
