#include <ros/ros.h> 
#include <ros/time.h>
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h>
#include <std_msgs/Empty.h> 
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <cstdlib>//string转化为double
#include <iomanip>//保留有效小数
#include "gps/NEG.h"

using namespace std;

string strRece = "";
string tmp = "";
string tmp_old = "";

void bestutma(string s , double& easting , double& nourthing ,double& height);
void negCallback(const std_msgs::StringConstPtr& msg);

///////////////////////////////// main ///////////////////////////////////
int main(int argc, char** argv)
{
    ros::init(argc, argv, "neg");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("GPS_msg", 1000, negCallback);
    ros::Publisher NEG_pub = nh.advertise<gps::NEG>("RTK_NEG",1000);
    
    string gstart = "#B"; 
    string gend = "\r\n";

    ros::Rate loop_rate(2000);    
    while (ros::ok())   
    {
        int start = -1, end = -1;
        ros::spinOnce();
        if(tmp == tmp_old) {
            continue;
        }
        tmp_old = tmp;
        strRece += tmp; 
        /* ------------------------------- find start ------------------------------- */
        start = strRece.find(gstart);
        if ( start == -1)
        {
            strRece = "";
            continue;
        }
        /* -------------------------------- find end -------------------------------- */
        else
        {
            end = strRece.find(gend, start);
            if (end == -1){
                strRece = strRece.substr(start);    // 截取与丢弃
                continue;
            }
            else    
            {
                /* --------------------------- All has been finded -------------------------- */
                double easting, northing, height;               
                bestutma(strRece.substr(start,end+2-start),easting,northing,height); // +2 是为了把\r\n包括进去   
                gps::NEG GPS_data;
                GPS_data.stamp = ros::Time::now().toSec();
                GPS_data.easting = easting;
                GPS_data.northing = northing;
                GPS_data.height = height;
                NEG_pub.publish(GPS_data);
                strRece = strRece.substr(end+2);
            }
        }
        loop_rate.sleep();  
    }
}

void bestutma(std::string s , double& easting , double& nourthing ,double& height)
{
    std::vector<std::string> v;
    std::string::size_type pos1, pos2;
    pos1 = s.find(";") + 1;
    pos2 = s.find(",",pos1);

    while ( std::string::npos !=pos2 ) 
    {
        v.push_back( s.substr( pos1, pos2-pos1 ) );  
        pos1 = pos2 + 1;       
        pos2 = s.find(",",pos1); 
    }

    if ( pos1 != s.length() ){
        v.push_back( s.substr( pos1 ));
    }
        
    // 北解析
    if (v[4] != ""){
        nourthing = std::atof(v[4].c_str());
    }
    else{
        nourthing = -1.0;
    }
    // 东解析
    if (v[5] != ""){
        easting = std::atof(v[5].c_str());
    }
    else{
        easting = -1.0;
    }
    // 高
    if (v[6] != ""){
        height = std::atof(v[6].c_str());
    }
    else{
        height = -1.0;
    }
}

void negCallback(const std_msgs::StringConstPtr& msg)
{
    tmp = msg->data;
}
