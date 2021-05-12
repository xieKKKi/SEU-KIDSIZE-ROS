/** 
*  Copyright (C) 2021, SEU Robot Lab. All rights reserved.
*  @file     player.cpp                                                  
*  @brief    1. 读取并发布imu值
             2. 读取并发布头部姿态
*  @author   谢家鹏
*  @todo     1. 接受LED任务，控制LED
             2. 增加imu reset服务随时重置imum
*/

#include <common/ImuData.h>
#include <common/LedTask.h>
#include <common/common.hpp>
#include <common/BodyAngles.h>
#include <common/HeadAngles.h>
#include <common/GetAngles.h>
#include <seumath/math.hpp>
#include <std_srvs/Empty.h>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <config/basic_parser.hpp>
#include "imu/imu.hpp"

using namespace std;
using namespace common;


LedTask ledTask;
GetAngles getsrv;
ImuData imudata;
HeadAngles headAngles;
shared_ptr<Imu> imu;
ros::Publisher imuPublisher;
ros::Publisher headPublisher;
bool imuReset = false;

bool ResetImuService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  imuReset = true;
}

void Run(const ros::TimerEvent& event)
{
  imudata.fall = imu->fall_direction();  //更新IMU
  imudata.yaw = imu->imu_yaw();
  //ROS_INFO("yaw: %f ", imudata.yaw);
  imuPublisher.publish(imudata);
  headPublisher.publish(headAngles); //发布头部姿态

  ros::service::call("/get_angles", getsrv);  //获取当前角度，非闭环
  if (getsrv.response.degs.size() > 0) 
  {
    headAngles = getsrv.response.head;
  }
  headPublisher.publish(headAngles); //发布头部姿态
}

void LedTaskUpdate(const LedTask::ConstPtr &p)
{
  ledTask = *p;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "player");
  ros::NodeHandle node;
  ros::service::waitForService("/get_angles");

  imuPublisher = node.advertise<ImuData>("/sensor/imu", 1);  //发布imu
  headPublisher = node.advertise<HeadAngles>("/sensor/head", 1);  //发布头部姿态
  ros::Subscriber ledSub = node.subscribe("/task/led", 1, LedTaskUpdate);  //订阅LED状态
  ros::ServiceServer imuRstService = node.advertiseService("/imu_reset", ResetImuService);  //提供imu重置服务


  //读取配置文件
  std::string hardware_file;
  try{
      ros::param::get("hardware_file", hardware_file);
      ROS_INFO("hardware_file: %s",hardware_file.c_str());
  }
  catch(ros::InvalidNameException &e){
      ROS_ERROR("%s", e.what());
      return false;
  }
  common::bpt::ptree pt;
  if(!common::get_tree_from_file(hardware_file, pt))
  {
      ROS_ERROR("parse file: [%s] failed", hardware_file.c_str());
      return false;
  }

  getsrv.request.player = "real";
 

  //开启IMU
  imu = std::make_shared<Imu>();
  string imuName = pt.get<string>("imu.dev_name");
  unsigned int imuBaudrate = pt.get<int>("imu.baudrate");
  ROS_INFO("IMU device: %s, imuBaudrate: %d", imuName.c_str(), imuBaudrate);
  if(!imu->start(imuName, imuBaudrate))
  {
    ROS_ERROR("IMU start failed, exit");
    return 0;
  }

  ros::Timer imuTimer = node.createTimer(ros::Duration(0.03), Run);
  ros::spin();

  imu->stop();
  return 0;
}