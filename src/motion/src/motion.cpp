/** 
*  Copyright (C) 2021, SEU Robot Lab. All rights reserved.
*  @file     motion.cpp                                                  
*  @brief    1. 根据bodytask和headtask生成身体的关节角度队列
             2. 控制电机
*  @author   谢家鹏
*  @todo     1. Motor类本应在src/motor/motor.hpp里实现，但因为要调用GetAnglesService_withoutROS函数会产生一些编译上的问题，现在放在本文件中实现使得代码很乱,有强迫症可以再整理一下
             2. 判断电机是否上电，现在的情况是如果运行程序电机未上电会没有反应
*/

#include <bits/stdc++.h>
#include <common/common.hpp>
#include <common/AddAngles.h>
#include <common/BodyAngles.h>
#include <common/BodyTask.h>
#include <common/GetActions.h>
#include <common/GetAngles.h>
#include <common/HeadAngles.h>
#include <common/HeadTask.h>
#include <common/ImuData.h>
#include <common/Angles.h>
#include <config/basic_parser.hpp>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <robot/action_engine.hpp>
#include <robot/robot.hpp>
#include "scan/ScanEngine.hpp"
#include "walk/WalkEngine.hpp"
//#include "motor/motor.hpp"
#include <vector>



#include <memory>
#include <list>
#include <mutex>
#include <atomic>
#include <map>
#include <time.h>
#include "motor/timer.hpp"
#include "motor/dynamixel/dynamixel_sdk.h"
#include "motor/class_exception.hpp"
#include <seumath/math.hpp>
#include <fstream>
#include "motor/logger.hpp"



#define ZERO_POS 2048
#define MAX_POS  4096
#define MIN_POS  0

#define ADDR_ID     7
#define ADDR_TORQ   64
#define ADDR_LED    65
#define ADDR_GPOS   116
#define ADDR_PPOS   132
#define ADDR_VOLT   144

#define SIZE_ID     1
#define SIZE_TORQ   1
#define SIZE_LED    1
#define SIZE_GPOS   4
#define SIZE_PPOS   4
#define SIZE_VOLT   2

#define DXL_ZERO_POS  2048
#define DXL_MAX_POS  4096
#define DXL_MIN_POS  0


using namespace std;
using namespace common;
using namespace seumath;
using namespace Eigen;
using namespace dynamixel;

class Motor: public Timer
{
public:
    Motor();
    
    bool start(std::string motorName, float motorVersion, int motorBaudrate);
    void closeMotor();
    void run();

private:
    bool openMotor(int Baudrate);
    void set_torq(uint8_t e);
    void set_led(uint8_t s);
    void set_gpos();
    //bool read_all_pos();
    //bool read_legs_pos();
    //bool read_head_pos();

private:
    bool is_open_motor;
    common::Angles anglesGet;
    common::HeadAngles headAngles;
    uint8_t ledstatus;
    int start_id;
    std::vector<float> degs;

    std::shared_ptr<dynamixel::PortHandler> portHandler_;
    std::shared_ptr<dynamixel::PacketHandler> packetHandler_;
    std::shared_ptr<dynamixel::GroupSyncWrite> torqWrite_;
    std::shared_ptr<dynamixel::GroupSyncWrite> gposWrite_;
    std::shared_ptr<dynamixel::GroupSyncWrite> ledWrite_;
    std::shared_ptr<dynamixel::GroupSyncRead> pposRead_;
};


void UpdateBodyTask(const common::BodyTask::ConstPtr &p);
common::BodyTask GetBodyTask();
void UpdateHeadTask(const common::HeadTask::ConstPtr &p);
common::HeadTask GetHeadTask();
void UpdateImu(const common::ImuData::ConstPtr &p);
common::ImuData GetImuData();
bool GetAnglesService(common::GetAngles::Request &req,
                      common::GetAngles::Response &res);
bool AddAnglesService(common::AddAngles::Request &req,
                      common::AddAngles::Response &res);
bool GetActionsService(common::GetActions::Request &req,
                       common::GetActions::Response &res);

std::shared_ptr<robot::ActionEngine> actionEng;
std::shared_ptr<WalkEngine> walkEng;
std::shared_ptr<ScanEngine> scanEng;
std::shared_ptr<robot::Robot> maxwell;
std::shared_ptr<Motor> motor;

common::BodyTask bodyTask;
std::mutex bodyMutex;
common::HeadTask headTask;
std::mutex headMutex;
common::ImuData imuData;
std::mutex imuMutex;
bool debug = false;

std::deque<common::BodyAngles> bodyAngles;  //队列实现动作序列
std::deque<common::HeadAngles> headAngles;

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion");
    ros::NodeHandle node;

    bool params_seted = false;
    while (!params_seted && ros::ok()) {  //等待参数更新
        try {
            ros::param::get("params", params_seted);
        } catch (const ros::InvalidNameException &e) {
            ROS_WARN("%s", e.what());
        }
        usleep(100000);
    }
    if (!params_seted) return 0;

    std::string act_file, robot_file, offset_file, walk_file, hardware_file;  //加载配置文件
    try {
        ros::param::get("action_file", act_file);
        ros::param::get("robot_file", robot_file);
        ros::param::get("offset_file", offset_file);
        ros::param::get("walk_file", walk_file);
        ros::param::get("hardware_file", hardware_file);
    } catch (ros::InvalidNameException &e) {
        ROS_ERROR("%s", e.what());
        return 0;
    }
    maxwell = std::make_shared<robot::Robot>(robot_file, offset_file);
    actionEng = std::make_shared<robot::ActionEngine>(act_file, maxwell);
    walkEng = std::make_shared<WalkEngine>(walk_file, maxwell);
    scanEng = std::make_shared<ScanEngine>();

    ros::Subscriber bodySub = node.subscribe("/task/body", 1, UpdateBodyTask); 
    ros::Subscriber headSub = node.subscribe("/task/head", 1, UpdateHeadTask);
    ros::Subscriber imuSub = node.subscribe("/sensor/imu", 1, UpdateImu);
    ros::ServiceServer getAglSrv =
        node.advertiseService("/get_angles", GetAnglesService);
    ros::ServiceServer addAglSrv =
        node.advertiseService("/add_angles", AddAnglesService);
    ros::ServiceServer getActsSrv =
        node.advertiseService("/get_actions", GetActionsService);

    double phase = 0.0;
    bool isWalking = false;
    ros::Rate loop_rate(30);  //30ms周期
    std::vector<common::BodyAngles> angles = actionEng->runAction("ready");  //首先进入ready状态
    bodyAngles.insert(bodyAngles.end(), angles.begin(), angles.end());
    common::HeadAngles ha;  
    ha.pitch = 45;
    headAngles.push_back(ha);


    common::bpt::ptree pt;
    if(!common::get_tree_from_file(hardware_file, pt))
    {
        ROS_ERROR("parse file: [%s] failed", hardware_file.c_str());
        return false;
    }

    // 开启电机
    
    motor = std::make_shared<Motor>();
    std::string motorName = pt.get<std::string>("motor.dev_name");
    float motorVersion = pt.get<float>("motor.version");
    int motorBaudrate = pt.get<int>("motor.baudrate");
    ROS_INFO("motorName: %s, motorVersion: %f, motorBaudrate: %d",motorName.c_str(), motorVersion, motorBaudrate);
    if(!motor->start(motorName, motorVersion, motorBaudrate))
    {
        ROS_ERROR("MOTOR start failed, exit");
        return 0;
    }

    unsigned int timeCount = 0;
    while (ros::ok()) {
        if (debug) {
            ros::spinOnce();
            ROS_INFO("DEBUG");

            continue;
        }

        auto imu = GetImuData();
        if (imu.fall != imu.FALL_NONE)  //摔倒的话立即将头摆直
        {  
                headAngles.clear();
                ha.pitch = 0;
                headAngles.push_back(ha);
        }
        else 
        {
            auto task = GetHeadTask();
            if (task.mode == task.ModeLookAt) //看向特定位置，可打断scan过程
            {  
                headAngles.clear();  //打断scan过程
                ha.pitch = task.pitch;
                ha.yaw = task.yaw;
                headAngles.push_back(ha);
            }
            else if (headAngles.size() < 5)  //scan
            {
                std::vector<common::HeadAngles> vheadAngles = scanEng->runScan(task);
                headAngles.insert(headAngles.end(), vheadAngles.begin(), vheadAngles.end());
            }
        }

        if (bodyAngles.size() < 5) //做完上一步动作后再执行下一步
        {
            auto task = GetBodyTask();  //每次GetBodyTask后会让bodyTask.count = -1，下一次UpdateBodyTask后才会大于0
            angles.clear();
            if (imu.fall == imu.FALL_FORWARD) {  //摔倒爬起
                angles = actionEng->runAction("front_getup");
            } else if (imu.fall == imu.FALL_BACKWARD) {
                angles = actionEng->runAction("back_getup");
            } else if (imu.fall == imu.FALL_LEFT) {
                angles = actionEng->runAction("front_getup");
            } else if (imu.fall == imu.FALL_RIGHT) {
                angles = actionEng->runAction("front_getup");
            } else 
            {
                if (task.type == common::BodyTask::TASK_WALK)  //步行模式
                {
                    if (task.count > 0)  //task.count > 0才走
                    {
                        if (!isWalking)  //先进去walking的状态、姿势，再正式开始walk
                        {
                            angles = walkEng->runWalk(Vector3d(0.0, 0.0, 0.0),
                                                      2, phase);
                            isWalking = true;
                        } 
                        else 
                        {
                            angles = walkEng->runWalk(
                                Vector3d(task.step, task.lateral, task.turn),
                                task.count, phase);
                        }
                    } 
                    else 
                    {
                        if (isWalking) 
                        {
                            angles = walkEng->runWalk(Vector3d(0.0, 0.0, 0.0),
                                                      0, phase);
                            isWalking = false;
                        }
                    }
                } 
                else 
                {
                    if (task.type == common::BodyTask::TASK_ACT)  //特定动作模式
                    {
                        if (task.count >= 0) 
                        {
                            if (isWalking) //打断步行状态
                            {
                                angles = walkEng->runWalk(Vector3d(0.0, 0.0, 0.0),
                                                        0, phase);
                                isWalking = false;
                            }
                            angles = actionEng->runAction(task.actname);
                        }
                    } 
                    else 
                    {
                        if (isWalking) 
                        {
                            angles =
                                walkEng->runWalk(Vector3d(0.0, 0.0, 0.0), 0, phase);
                            isWalking = false;
                        }
                    }
                }
            }
            bodyAngles.insert(bodyAngles.end(), angles.begin(), angles.end());
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    motor->closeMotor();
    return 0;
}

//每次服务被调用，都会让angles队头出队
void GetAnglesService_withoutROS(int &start_id_, vector<float> &degs_) {
    // ROS_INFO("getangles");
    bodyMutex.lock();
    if (!bodyAngles.empty()) {
        maxwell->set_body(bodyAngles.front());
        bodyAngles.pop_front();
    }
    bodyMutex.unlock();
    headMutex.lock();
    if (!headAngles.empty()) {
        maxwell->set_head(headAngles.front());
        headAngles.pop_front();
    }
    headMutex.unlock();

    degs_.clear();
    {
        int sid = maxwell->joint_start_id();
        start_id_ = sid;
        for (int i = 0; i < maxwell->joint_count(); i++) {
            auto joint = maxwell->get_joint(sid + i);
            float deg = (joint->current_deg + joint->offset) * joint->inverse;
            degs_.push_back(deg);
        }
        //angles_get.head.yaw = maxwell->get_joint("jhead1")->current_deg;
        //angles_get.head.pitch = maxwell->get_joint("jhead2")->current_deg;
    }
}

void UpdateBodyTask(const common::BodyTask::ConstPtr &p) {
    std::lock_guard<std::mutex> lk(bodyMutex);
    bodyTask = *p;
}

common::BodyTask GetBodyTask() {
    std::lock_guard<std::mutex> lk(bodyMutex);
    auto ret = bodyTask;
    bodyTask.count = -1;
    return ret;
}

void UpdateHeadTask(const common::HeadTask::ConstPtr &p) {
    std::lock_guard<std::mutex> lk(headMutex);
    headTask = *p;
}

common::HeadTask GetHeadTask() {
    std::lock_guard<std::mutex> lk(headMutex);
    return headTask;
}

void UpdateImu(const common::ImuData::ConstPtr &p) {
    std::lock_guard<std::mutex> lk(imuMutex);
    imuData = *p;
}

common::ImuData GetImuData() {
    std::lock_guard<std::mutex> lk(imuMutex);
    return imuData;
}

//每次服务被调用，angles队头不会出队
bool GetAnglesService(common::GetAngles::Request &req,
                      common::GetAngles::Response &res) {
    /*
    // ROS_INFO("getangles");
    bodyMutex.lock();
    if (!bodyAngles.empty()) {
        maxwell->set_body(bodyAngles.front());
        //bodyAngles.pop_front();
    }
    bodyMutex.unlock();
    headMutex.lock();
    if (!headAngles.empty()) {
        maxwell->set_head(headAngles.front());
        //headAngles.pop_front();
    }
    headMutex.unlock();
    */
    if (req.player == "real") {
        int sid = maxwell->joint_start_id();
        res.start_id = sid;
        for (int i = 0; i < maxwell->joint_count(); i++) {
            auto joint = maxwell->get_joint(sid + i);
            float deg = (joint->current_deg + joint->offset) * joint->inverse;
            res.degs.push_back(deg);
        }
        res.head.yaw = maxwell->get_joint("jhead1")->current_deg;
        res.head.pitch = maxwell->get_joint("jhead2")->current_deg;
    } else {
        res.head.yaw = maxwell->get_joint("jhead1")->current_deg;
        res.head.pitch = maxwell->get_joint("jhead2")->current_deg;
        res.body.left_shoulder = maxwell->get_joint("jlshoulder1")->current_deg;
        res.body.left_elbow = maxwell->get_joint("jlelbow")->current_deg;
        res.body.right_shoulder =
            maxwell->get_joint("jrshoulder1")->current_deg;
        res.body.right_elbow = maxwell->get_joint("jrelbow")->current_deg;

        res.body.left_hip_yaw = maxwell->get_joint("jlhip3")->current_deg;
        res.body.left_hip_roll = maxwell->get_joint("jlhip2")->current_deg;
        res.body.left_hip_pitch = maxwell->get_joint("jlhip1")->current_deg;
        res.body.left_knee = maxwell->get_joint("jlknee")->current_deg;
        res.body.left_ankle_pitch = maxwell->get_joint("jlankle2")->current_deg;
        res.body.left_ankle_roll = maxwell->get_joint("jlankle1")->current_deg;

        res.body.right_hip_yaw = maxwell->get_joint("jrhip3")->current_deg;
        res.body.right_hip_roll = maxwell->get_joint("jrhip2")->current_deg;
        res.body.right_hip_pitch = maxwell->get_joint("jrhip1")->current_deg;
        res.body.right_knee = maxwell->get_joint("jrknee")->current_deg;
        res.body.right_ankle_pitch =
            maxwell->get_joint("jrankle2")->current_deg;
        res.body.right_ankle_roll = maxwell->get_joint("jrankle1")->current_deg;
    }
    return true;
}

bool AddAnglesService(common::AddAngles::Request &req,
                      common::AddAngles::Response &res) {
    ROS_INFO("AddAnglesService");
    bodyAngles.insert(bodyAngles.end(), req.angles.begin(), req.angles.end());
    debug = true;
    return true;
}

bool GetActionsService(common::GetActions::Request &req,
                       common::GetActions::Response &res) {
    res.actions = actionEng->getActions();
    return true;
}

















Motor::Motor(): Timer(20)  //周期20ms
{
    is_open_motor = false;
    HeadAngles headAngles;
    ledstatus = 1;
}

bool Motor::start(string motorName, float motorVersion, int motorBaudrate)
{
    portHandler_ = PortHandler::getPortHandler(motorName.c_str());
    packetHandler_ = PacketHandler::getPacketHandler(motorVersion);
    gposWrite_ = make_shared<GroupSyncWrite>(portHandler_, packetHandler_, ADDR_GPOS, SIZE_GPOS);
    ledWrite_ = make_shared<GroupSyncWrite>(portHandler_, packetHandler_, ADDR_LED, SIZE_LED);
    torqWrite_ = make_shared<GroupSyncWrite>(portHandler_, packetHandler_, ADDR_TORQ, SIZE_TORQ);
    pposRead_ = make_shared<GroupSyncRead>(portHandler_, packetHandler_, ADDR_PPOS, SIZE_PPOS);
    GetAnglesService_withoutROS(start_id, degs); //获取角度控制量角度
    if (!is_open_motor)
    {
      is_open_motor = openMotor(motorBaudrate);
      if(!is_open_motor)
      {
        ROS_ERROR("MOTOR start failed, exit");
        return false;
      }
      set_led(ledstatus);//电机上的LED常亮
    }

    Timer::is_alive_ = true;
    start_timer();
    return true;
}

void Motor::run()
{
  clock_t t1 =clock();

  GetAnglesService_withoutROS(start_id, degs); //获取角度控制量角度

  gposWrite_->clearParam();
  if (degs.size() > 0)  //角度控制量不为空
  {
    //headAngles = anglesGet.head;
    set_gpos();  //写电机
  }
  double t3 = (double)(clock()-t1)/CLOCKS_PER_SEC;
}


void Motor::set_torq(uint8_t e)
{
    torqWrite_->clearParam();
    uint8_t torq_data;
    torq_data = e;

    for (int j = 0; j < degs.size(); j++)
    {
      uint8_t mid = static_cast<uint8_t>(j + start_id);
      if (!torqWrite_->addParam(mid, &torq_data))
      {
          ROS_ERROR("torqWrite_->addParam ERROR!");
          return;
      }
    }
    torqWrite_->txPacket();
}

void Motor::set_led(uint8_t s)
{
    ledWrite_->clearParam();
    uint8_t led_data;
    led_data = s;

    for (int j = 0; j < degs.size(); j++)
    {
      uint8_t mid = static_cast<uint8_t>(j + start_id);
      if (!ledWrite_->addParam(mid, &led_data))
      {
          ROS_ERROR("ledWrite_->addParam ERROR!");
          return;
      }
    }
    ledWrite_->txPacket();
}

void Motor::set_gpos()
{
    gposWrite_->clearParam();
    uint8_t gpos_data[4];
    uint32_t gpos;
    float deg;

    for (int j = 0; j < degs.size(); j++)
    {
      uint32_t gpos = static_cast<uint32_t>(degs[j]
        /(360.0f / (float)(DXL_MAX_POS - DXL_MIN_POS)) + DXL_ZERO_POS);
      uint8_t mid = static_cast<uint8_t>(j + start_id);
      gpos_data[0] = DXL_LOBYTE(DXL_LOWORD(gpos));
      gpos_data[1] = DXL_HIBYTE(DXL_LOWORD(gpos));
      gpos_data[2] = DXL_LOBYTE(DXL_HIWORD(gpos));
      gpos_data[3] = DXL_HIBYTE(DXL_HIWORD(gpos));

      if (!gposWrite_->addParam(static_cast<uint8_t>(mid), gpos_data))
      {
          ROS_ERROR("gposWrite_->addParam ERROR!");
          return;
      }
    }
    gposWrite_->txPacket();
}

bool Motor::openMotor(int Baudrate)
{
  if (!portHandler_->openPort())
  {
      ROS_ERROR("openMotorPort ERROR!");
      return false;
  }

  if (!portHandler_->setBaudRate(Baudrate))
  {
    ROS_ERROR("openMotor setBaudRate ERROR!");
      return false;
  }

  set_torq(1); //打开成功，舵机上力
  return true;
}

void Motor::closeMotor()
{
    if (is_open_motor)
    {
        is_open_motor = false;
        portHandler_->closePort();
    }
    set_led(0);
}
