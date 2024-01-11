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

// 全局变量
string strRece = "";
string tmp = "";
string tmp_old = "";

// 函数
void bestutma(string s , double& easting , double& nourthing ,double& height);
void negCallback(const std_msgs::StringConstPtr& msg);

///////////////////////////////// main ///////////////////////////////////

int main(int argc, char** argv)
{
    ros::init(argc, argv, "neg");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("GPS_msg", 1000, negCallback);
    ros::Publisher NEG_pub = nh.advertise<gps::NEG>("RTK_NEG",1000);
    
    string gstart = "#B";     // #BESTUTMA 起始标志
    string gend = "\r\n";     // GPS终止标志

    ros::Rate loop_rate(2000);    
    while (ros::ok())   // 这里虽然是while循环，但是存在睡眠间歇
    {
        int start = -1, end = -1;
        
        ros::spinOnce();
        
        if(tmp == tmp_old) {
            continue;
        }
        tmp_old = tmp;
        strRece += tmp; // 将串口的GPS数据追加到strRece尾部，读之后清空缓存


        // Step 3.2.1 找起始标志
        start = strRece.find(gstart);
        // 如果没找到，丢弃这部分数据，但要留下最后2位,避免遗漏掉起始标志
        if ( start == -1)
        {
            // ROS_INFO_STREAM("Can not find START!\n");
            strRece = "";
            continue;
        }
        // 如果找到了起始标志，开始找终止标志
        else
        {
            ROS_INFO_STREAM("START has been finded!\n");
            end = strRece.find(gend, start);
            // 如果没找到，把起始标志开始的数据留下，前面的数据丢弃，然后跳出循环
            if (end == -1)
            {
                strRece = strRece.substr(start);    // 截取与丢弃
                // ROS_INFO_STREAM("Can not find END!\n");
                continue;
            }
            else    // 如果找到了终止标志，把这段有效的数据剪切给解析的函数，剩下的继续开始寻找
            {
                ROS_INFO_STREAM("END has been finded!\n");
                double easting, northing, height;
                
                bestutma(strRece.substr(start,end+2-start),easting,northing,height); // +2 是为了把\r\n包括进去 
                
                // 发布消息到话题
                gps::NEG GPS_data;
                GPS_data.stamp = ros::Time::now().toSec();
                GPS_data.easting = easting;
                GPS_data.northing = northing;
                GPS_data.height = height;
               
                NEG_pub.publish(GPS_data);

                strRece = strRece.substr(end+2);
            }
        }
        loop_rate.sleep();  // 用 sleep 来控制频率，睡眠时间自适应，以达到指定的频率
    }

}

void bestutma(std::string s , double& easting , double& nourthing ,double& height)
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
    
}

void negCallback(const std_msgs::StringConstPtr& msg)
{
    tmp = msg->data;
}