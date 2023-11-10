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
#include "serialPort/NEG.h"

void bestutma(std::string s , double& easting , double& nourthing ,double& height,float& diff_age)
{
    /* ---------------------------  分割有效数据，存入vector中 -------------------------- */
    // 实例数据：s = #BESTUTMA,COM1,0,73.0,FINESTEERING,1419,336209.000,00000040,eb16,2724;
    //              SOL_COMPUTED,NARROW_INT,11,U,5666936.4417,707279.3875,1063.8401,-16.2712,WGS84,
    //              0.0135,0.0084,0.0173,"AAAA",1.000,0.000,8,8,8,8,0,01,0,03*a6d06321\r\n
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
    
    // v = [SOL_COMPUTED,NARROW_INT,11,U,5666936.4417,707279.3875,1063.8401,-16.2712,WGS84.....]

    /* ----------------------------- 从vector中直接索引数据 ----------------------------- */
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
    if (v[13] != ""){
        diff_age = std::atof(v[13].c_str());
    }
    else{
        diff_age = -1.0;
    }
}

int main(int argc, char** argv)
{
    /* ------------------------------- Step 1 初始操作 ------------------------------ */
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    ros::Publisher GPS_pub = nh.advertise<serialPort::NEG>("GPS",1000);

    std::string s = "#BESTUTMA,COM1,0,73.0,FINESTEERING,1419,336209.000,00000040,eb16,2724;SOL_COMPUTED,NARROW_INT,11,U,5666936.4417,707279.3875,1063.8401,-16.2712,WGS84,0.0135,0.0084,0.0173,\"AAAA\",1.000,0.000,8,8,8,8,0,01,0,03*a6d06321\r\n";
    std::string gstart = "#BESTUTMA";     // GPS起始标志
    std::string gend = "\r\n";      // GPS终止标志
    
    ros::Rate loop_rate(20);    // 控制循环频率为20Hz
    while(ros::ok())
    {
        int i = 0, start = -1, end = -1;
        start = s.find(gstart);
        end = s.find(gend);

        double easting, northing, height;
        float diff_age;
        bestutma(s.substr(start,end+2-start),easting,northing,height,diff_age); 
        
        serialPort::NEG GPS_data;
        GPS_data.stamp = ros::Time::now().toSec();
        GPS_data.easting = easting;
        GPS_data.northing = northing;
        GPS_data.height = height;
        GPS_data.diff_age = diff_age;
        GPS_pub.publish(GPS_data);

        ros::spinOnce();
        loop_rate.sleep();  // 用 sleep 来控制频率，睡眠时间自适应，以达到指定的频率
    }

}