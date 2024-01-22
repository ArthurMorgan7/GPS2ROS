#include <iomanip>
#include <ros/time.h>
#include <ros/ros.h>
#include <novatel_gps_msgs/NovatelUtmPosition.h>
#include "std_msgs/String.h"
#include "gps/NEG.h"
#include "gps/HEADING.h"

using namespace std;

novatel_gps_msgs::NovatelUtmPosition rtk;
double time_neg=0, time_heading=0;

void negCallback(const gps::NEGConstPtr& msg);
void headingCallback(const gps::HEADINGConstPtr& msg);

///////////////////////////////// main ///////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    ros::Subscriber sub_neg = nh.subscribe("RTK_NEG", 1000, negCallback);
    ros::Subscriber sub_heading = nh.subscribe("RTK_heading", 1000, headingCallback);
    ros::Publisher pub_rtk = nh.advertise<novatel_gps_msgs::NovatelUtmPosition>("RTK",1000);
    ros::Rate loop_rate(5); 
    double time_old = 0;   
    while (ros::ok()) 
    {
        ros::spinOnce();
        if(floor(time_neg) == floor(time_heading) && floor(time_neg)!=time_old){
            rtk.header.stamp = ros::Time().fromSec(time_neg);
        
            pub_rtk.publish(rtk);
            time_old = floor(time_neg);
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
    rtk.diff_age = msg->heading;
    //cout << "msg->heading" << msg->heading  <<endl;
    // 如果顺为正
    //if(msg->heading <  180){
    //    rtk.diff_age = - msg->heading;
    //}
    //else{
    //    rtk.diff_age = 360 - msg->heading;
    //}
    
    // 如果逆为正
    // if(msg->heading <  180){
    //     rtk.diff_age =  msg->heading;
    // }
    // else{
    //     rtk.diff_age = msg->heading - 360;
    // }
}
