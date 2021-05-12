/** 
*  Copyright (C) 2021, SEU Robot Lab. All rights reserved.
*  @file     gamectrl/main.cpp                                                  
*  @brief    1. 比赛状态控制
*  @author   谢家鹏
*  @todo     1. 现在只有最简单的状态转换，需进一步完善
             2. udp_service.run()，实现远程状态控制
*/
#include <ros/ros.h>
#include "gctrl.hpp"
#include <common/GcInfo.h>
#include <common/datadef.hpp>

using namespace common;
using namespace std;

boost::asio::io_service udp_service;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "communication");
    ros::NodeHandle node;
    bool params_seted = false;
    while (!params_seted && ros::ok())
    {
        try
        {
        ros::param::get("params", params_seted);  //获取params参数
        }
        catch (const ros::InvalidNameException &e)
        {
            ROS_WARN("%s", e.what());
        }
        usleep(100000);
    }
    if(!params_seted) return 0;

    ros::Publisher gcPublisher = node.advertise<common::GcInfo>("/sensor/gctrl", 1);
    common::GcInfo info;
    int timeCount = 0;
    ros::Rate rate(100);
    
    while(ros::ok())
    {
        if(timeCount<10)
        {
            info.state = GC_INITIAL;
            timeCount++;
        }
        else
        {
            info.state = GC_PLAY;
        }
        gcPublisher.publish(info);
        rate.sleep();

    }

    /* //调试暂时停用
    GameCtrl gc(node, udp_service);
    gc.start();
    udp_service.run();
    gc.stop();
    */
    return 0;
}