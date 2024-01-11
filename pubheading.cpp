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

// 命名空间
using namespace std;

// 全局变量
string strRece = "";
string tmp = "";
string tmp_old = "";

// 函数声明
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
        std::string gstart = "#H";     // #HEADA 起始标志
        std::string gend = "\r\n";     // 终止标志
        int i = 0, start = -1, end = -1;
        
        ros::spinOnce();
        
        if(tmp == tmp_old) {
		    continue;
        }
	    tmp_old = tmp;
        strRece += tmp; // 将串口的GPS数据追加到strRece尾部

        while ( i < strRece.length() )
        {
            // 找起始标志
            start = strRece.find(gstart);
            if ( start == -1)   // 没找到，丢弃这部分数据
            {
                // DEBUG
                // ROS_INFO_STREAM("Can not find START!\n");
                // ROS_INFO("%s\n",strRece.c_str());
                strRece = "";
            }
            else    // 如果找到了起始标志，开始找终止标志
            {
                // ROS_INFO_STREAM("START has been finded!\n");

                // 找终止标志
                end = strRece.find(gend, start);    
                if (end == -1)  // 如果没找到，把起始标志开始的数据留下，前面的数据丢弃，然后跳出循环
                {
                    strRece = strRece.substr(start);    // 截取与丢弃
                    // ROS_INFO_STREAM("Can not find END!\n");
                    break;
                }
                else    // 如果找到了终止标志，把这段有效的数据剪切给解析的函数，剩下的继续开始寻找
                {
                    // ROS_INFO_STREAM("END has been finded!\n");
                    
                    // 把有效的数据给解析的函数以获取偏航角
                    float heading;
                    headinga(strRece.substr(start,end+2-start),heading); // +2 是为了把\r\n包括进去 
                    
                    // std::cout << std::setiosflags(std::ios::fixed)<<std::setprecision(7)
                    //             << "偏航角：" << heading << "\n";
                    
                    // 发布消息到话题
                    gps::HEADING GPS_data;
                    GPS_data.stamp = ros::Time::now().toSec();
                    GPS_data.heading = heading;
                    GPS_pub.publish(GPS_data);
                    
                    strRece = strRece.substr(end+2);
                }
            }
        }

        loop_rate.sleep();  // 用 sleep 来控制频率，睡眠时间自适应，以达到指定的频率
    }

}

///////////////////////////////////////////////////////////////////////
void headinga(string s , float& heading)
{
    /* ---------------------------  分割有效数据，存入vector中 -------------------------- */

    vector<string> v;
    string::size_type pos1, pos2;
    pos1 = s.find(";") + 1;
    pos2 = s.find(",",pos1); // 返回第一个该字符所在位置

    // 把字符串s中的有效数据存入容器v中
    while ( string::npos !=pos2 )  // find函数没有找到，会返回 npos
    {
        v.push_back( s.substr( pos1, pos2-pos1 ) );  // 把从pos1开始到pos2-pos1的字段整体存入容器v
        pos1 = pos2 + 1;        // pos1 指向剩余字符的开始
        pos2 = s.find(",",pos1);    // 从pos1开始，寻找下一个","
    }

    if ( pos1 != s.length() )
        v.push_back( s.substr( pos1 ));
    
    // v = [SOL_COMPUTED,NARROW_INT,12.801044464,160.432525635.....]

    /* ----------------------------- 从vector中直接索引数据 ----------------------------- */
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

