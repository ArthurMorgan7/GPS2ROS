#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h>
#include <std_msgs/Empty.h> 
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <cstdlib>//string转化为double
#include <iomanip>//保留有效小数
#include "serialPort/JWG.h"

// 声明全局变量 串口对象 ser
serial::Serial ser; 

// 解析GPS
void RecePro(std::string s , double& lat , double& lon, double& alt)
{
    // 分割有效数据，存入vector中
    // 实例数据：s = $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n
    std::vector<std::string> v;
    std::string::size_type pos1, pos2;
    pos2 = s.find(","); // 返回第一个该字符所在位置
    pos1 = 0;

    // 把字符串s中的有效数据存入容器v中
    while ( std::string::npos !=pos2 )  // find函数没有找到，会返回 npos
    {
        v.push_back( s.substr( pos1, pos2-pos1 ) );  // 把从pos1开始到pos2-pos1的字段整体存入容器v
        pos1 = pos2 + 1;        // pos1 指向剩余字符的开始
        pos2 = s.find(",",pos1);    // 从pos1开始，寻找下一个","
    }
    if ( pos1 != s.length() )
        v.push_back( s.substr( pos1 ));
    

    // 解析出经纬度
    if (v.max_size() >= 6 && (v[6] == "1" || v[6] == "2" || v[6] == "3" || v[6] == "4" || v[6] == "5" || v[6] == "6" || v[6] == "8" || v[6] == "9"))
    {
        // 纬度解析
        if (v[2] != "") lat = std::atof(v[2].c_str()) / 100;
        int ilat = (int)floor(lat) % 100;       // 截取整数部分，取余是害怕大于100
        lat = ilat + (lat - ilat) * 100 / 60;   // 转化为分读值的纬度，double 类型 （4807.038 = 48.1173度 ）
        // 经度解析
        if (v[4] != "") lon = std::atof(v[4].c_str()) / 100;
        int ilon = (int)floor(lon) % 1000;
        lon = ilon + (lon - ilon) * 100 / 60;
        // 解析高度
        if (v[8] != "") alt = std::atof(v[8].c_str());
    }
}

int main(int argc, char** argv)
{
    /* ------------------------------- Step 1 初始操作 ------------------------------ */
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    
    // 创建一个发布者对象 GPS_pub，把话题发布到 “GPS”
    ros::Publisher GPS_pub = nh.advertise<serialPort::JWG>("GPS",1000);
    
    
    /* ------------------------------- Step 2 串口设置 ------------------------------ */
    try
    {
      ser.setPort("/dev/ttyUSB0");  // 设置实际对应的串口
      ser.setBaudrate(115200);      // 设置波特率
      serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 串口连接的超时时间(毫秒) 1000ms = 1s
      ser.setTimeout(to);
      ser.open();   // 打开串口
    }
    // try-catch 结构，如果try块内出错，则执行catch，输出错误信息。
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open Serial Port !");   
        return -1;
    }
    
    // 如果成功打开，输出成功消息。
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    ser.write("log gpgga ontime 1");

    ros::Rate loop_rate(20);    // 控制循环频率为20Hz

    /* --------------------------- Step 3 接受数据并发布到ROS话题 --------------------------- */
    // 这里用的就是串口通信的一套
    std::string strRece;    // 用于保存暂时接受到的雷达数据（会被不断覆盖）
    while (ros::ok())   // 这里虽然是while循环，但是存在睡眠间歇
    {
        // 用 available 检查串口是否有可用数据
        if (ser.available())  // 有数据
        {
            // Step 3.1 读取串口信息
            ROS_INFO_STREAM("Reading from serial port\n");
            //通过ROS串口对象读取串口信息
            //std::cout << ser.available()<<"\n";
            //std::cout << ser.read(ser.available())<<"\n";
            strRece += ser.read(ser.available()); // 将串口的GPS数据追加到strRece尾部，读之后清空缓存
            std::cout <<"strRece:" << strRece << "\n" ;
            // GPS数据实例：strRece = "$GNGGA,122020.70,3908.17943107,N,11715.45190423,E,1,17,1.5,19.497,M,-8.620,M,,*54\r\n";
            
            // Step 3.2 截取数据、解析数据
            std::string gstart = "$GPGGA";     // GPS起始标志
            std::string gend = "\r\n";      // GPS终止标志
            int i = 0, start = -1, end = -1;
            while ( i < strRece.length() )
            {
                // Step 3.2.1 找起始标志
                start = strRece.find(gstart);
                // 如果没找到，丢弃这部分数据，但要留下最后2位,避免遗漏掉起始标志
                if ( start == -1)
                {
                    ROS_INFO_STREAM("Can not find START!\n");
                    // 该信息用于DEBUG
                    ROS_INFO("Data:%s------------\n",strRece.c_str());
                    if (strRece.length() > 2)   
                        strRece = strRece.substr(strRece.length()-3);   // 从“strRece.length()-3”开始的所有字符，此时会有一个覆盖
                        break;
                }
                // 如果找到了起始标志，开始找终止标志
                else
                {
                    ROS_INFO_STREAM("START is finded!\n");
                    // 找终止标志
                    end = strRece.find(gend);
                    // 如果没找到，把起始标志开始的数据留下，前面的数据丢弃，然后跳出循环
                    if (end == -1)
                    {
                        if (end != 0)
                        strRece = strRece.substr(start);
                        ROS_INFO_STREAM("Can not find END!\n");
                        break;
                    }
                    // 如果找到了终止标志，把这段有效的数据剪切给解析的函数，剩下的继续开始寻找
                    else
                    {
                        ROS_INFO_STREAM("END is finded!\n");
                        i = end;

                        // 把有效的数据给解析的函数以获取经纬度
                        double lat, lon, alt;
                        RecePro(strRece.substr(start,end+2-start),lat,lon,alt); // 返回lat,lon
                        std::cout << std::setiosflags(std::ios::fixed)<<std::setprecision(7)
                                  << "纬度：" << lat << "; " 
                                  << "经度：" << lon << "; "
                                  << "高度：" << alt << "\n" ;
                        
                        // 发布消息到话题
                        serialPort::JWG GPS_data;
                        GPS_data.stamp = ros::Time::now().toSec();
                        GPS_data.lat = lat;
                        GPS_data.lon = lon;
                        GPS_data.alt = alt;
                        GPS_pub.publish(GPS_data);
                        
                        // 如果剩下的字符大于等于4，则继续循环寻找有效数据,如果所剩字符小于等于3则跳出循环
                        if ( i+5 < strRece.length())
                            strRece = strRece.substr(end+2);
                        else
                        {   strRece = strRece.substr(end+2);
                            break;
                        }
                    }
                }
            }
        }
        else    // 无数据
        {
            ROS_INFO_STREAM("No Data!!!\n");
        }
        ros::spinOnce();
        loop_rate.sleep();  // 用 sleep 来控制频率，睡眠时间自适应，以达到指定的频率
    }

}