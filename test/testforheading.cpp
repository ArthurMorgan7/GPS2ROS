#include <ros/ros.h> 
#include <ros/time.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h> 
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include "gps/HEADING.h"

using namespace std;

void headinga(std::string s , float& heading)
{
    std::vector<std::string> v;
    std::string::size_type pos1, pos2;
    pos1 = s.find(";") + 1;
    pos2 = s.find(",",pos1); // 返回第一个该字符所在位置

    // 把字符串s中的有效数据存入容器v中
    while ( std::string::npos !=pos2 )  // find函数没有找到，会返回 npos
    {
        v.push_back( s.substr( pos1, pos2-pos1 ) );  // 把从pos1开始到pos2-pos1的字段整体存入容器v
        pos1 = pos2 + 1;        // pos1 指向剩余字符的开始
        pos2 = s.find(",",pos1);    // 从pos1开始，寻找下一个","
    }

    // 把校验位加进去，校验位之后没有','了
    if ( pos1 != s.length() )
        v.push_back( s.substr( pos1 ));

    // v = [SOL_COMPUTED,NARROW_INT,12.801044464,160.432525635.....]

    /* ----------------------------- 从vector中直接索引数据 ----------------------------- */
    // 偏航角解析
    if (v[3] != ""){
        heading = std::atof(v[3].c_str());
    }
    else{
        heading = -1.0;
    }
}

int main(int argc, char** argv)
{
    /* ------------------------------- Step 1 初始操作 ------------------------------ */
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    ros::Publisher GPS_pub = nh.advertise<gps::HEADING>("RTK_heading",1000);

    string s = "#HEADINGA,COM1,0,66.5,FINESTEERING,1844,505873.200,00040020,22a9,13306;SOL_COMPUTED,NARROW_INT,12.801044464,160.432525635,-0.015716553,0.0,0.018702479,0.029530477,\"G097\",18,16,16,16,00,01,00,33*c9cd21f6\r\n";
    string gstart = "#HEADINGA";     // GPS起始标志
    string gend = "\r\n";      // GPS终止标志
    float heading;
    gps::HEADING GPS_data;
    int start = -1, end = -1;

    ros::Rate loop_rate(20);    // 控制循环频率为20Hz
    while(ros::ok())
    {
        ros::spinOnce();

        
        start = s.find(gstart);
        end = s.find(gend);

        headinga(s.substr(start,end+2-start),heading); 
        GPS_data.stamp = ros::Time::now().toSec();
        GPS_data.heading = heading;
        GPS_pub.publish(GPS_data);

        loop_rate.sleep();  // 用 sleep 来控制频率，睡眠时间自适应，以达到指定的频率
    }

}
