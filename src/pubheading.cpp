#include <ros/ros.h> 
#include <ros/time.h>
#include <serial/serial.h> 
#include <std_msgs/String.h>
#include <std_msgs/Empty.h> 
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include "gps/HEADING.h"

using namespace std;

string strRece = "";
string tmp = "";
string tmp_old = "";

void headinga(string s , float& heading);
void negCallback(const std_msgs::StringConstPtr& msg);

///////////////////////////////// main ///////////////////////////////////
int main(int argc, char** argv)
{
    ros::init(argc, argv, "heading");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("GPS_msg", 2000, negCallback);
    ros::Publisher GPS_pub = nh.advertise<gps::HEADING>("RTK_heading",1000);
    
    ros::Rate loop_rate(2000);    
    while (ros::ok()) 
    {
        std::string gstart = "#H";     // 起始标志
        std::string gend = "\r\n";     // 终止标志
        int i = 0, start = -1, end = -1;
        
        ros::spinOnce();
        
        if(tmp == tmp_old) {
		    continue;
        }
	    tmp_old = tmp;
        strRece += tmp; 

        while ( i < strRece.length() )
        {
            /* ------------------------------- find start ------------------------------- */
            start = strRece.find(gstart);
            if ( start == -1)  
            {
                strRece = "";
            }
            else    
            {
                /* -------------------------------- find end -------------------------------- */
                end = strRece.find(gend, start);    
                if (end == -1) 
                {
                    strRece = strRece.substr(start);    // 截取与丢弃
                    break;
                }
                else    
                {
                    /* --------------------------- All has been finded -------------------------- */
                    float heading;
                    headinga(strRece.substr(start,end+2-start),heading); // +2 是为了把\r\n包括进去 
                    gps::HEADING GPS_data;
                    GPS_data.stamp = ros::Time::now().toSec();
                    GPS_data.heading = heading;
                    GPS_pub.publish(GPS_data);
                    strRece = strRece.substr(end+2);
                }
            }
        }
        loop_rate.sleep(); 
    }
}

///////////////////////////////////////////////////////////////////////
void headinga(string s , float& heading)
{
    vector<string> v;
    string::size_type pos1, pos2;
    pos1 = s.find(";") + 1;
    pos2 = s.find(",",pos1); 

    while ( string::npos !=pos2 ) 
    {
        v.push_back( s.substr( pos1, pos2-pos1 ) );  
        pos1 = pos2 + 1;        
        pos2 = s.find(",",pos1);   
    }
    if ( pos1 != s.length() ){
        v.push_back( s.substr( pos1 ));
    }
        
    // 偏航角解析
    if (v[3] != ""){
        heading = atof(v[3].c_str());
    }
    else{
        heading = -1.0;
    }
}

void negCallback(const std_msgs::StringConstPtr& msg)
{
    tmp = msg->data;
}

