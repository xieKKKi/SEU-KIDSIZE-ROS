/** 
*  Copyright (C) 2021, SEU Robot Lab. All rights reserved.
*  @file     control/main.cpp                                                  
*  @brief    1. 状态控制
*  @author   谢家鹏
*  @todo     1. 完善状态机，参考seu unibot代码。现在状态机很简陋。
*/
#include <common/BodyTask.h>
#include <common/GcInfo.h>
#include <common/HeadAngles.h>
#include <common/HeadTask.h>
#include <common/ImageResult.h>
#include <common/ImuData.h>
#include <common/LedTask.h>
#include <common/PlayerInfo.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <common/datadef.hpp>
#include <config/basic_parser.hpp>
#include <fstream>
#include <mutex>
#include <common/ObjInfo.h>
#include <common/PlayerInfo.h>
#include <seumath/math.hpp>
#include <memory>
#include <stdlib.h>

using namespace common;
using namespace std;
using namespace Eigen;
using namespace seumath;

const char *GameState[]{"Initial", "Ready", "Set", "Playing", "Finish"};
enum State {
    STATE_READY,        // 0
    STATE_GETUP,        // 1
    STATE_SEARCH_BALL,  // 2
    STATE_TURNTO_BALL,  // 3
    STATE_GOTO_BALL,    // 4
    STATE_KICK_BALL,    // 5
    STATE_FINISH       // 6
};

// searchBallTable = [(-90, 0), (-45, 0), (0, 0), (45, 0), (90, 0),
//     (90, 30), (45, 30), (0, 30), (-45, 30), (-90, 30),
//     (-70, 55), (-35, 55), (0, 55), (35, 55), (70, 55)]
// searchPostTable = [(-90, 0), (-45, 0), (0, 0), (45, 0), (90, 0),
//     (90, 30), (45, 30), (0, 30), (-45, 30), (-90, 30)]

ImageResult imgResult;
HeadAngles headAngles;
ImuData imuData;
GcInfo gcData;
HeadTask htask;
BodyTask btask;
PlayerInfo pinfo;
int fsmState;
int targetDir;

void imgResUpdate(const ImageResult::ConstPtr &p) { imgResult = *p; }

void headUpdate(const HeadAngles::ConstPtr &p) { headAngles = *p; }

void imuUpdate(const ImuData::ConstPtr &p) { imuData = *p; }

void gcUpdate(const GcInfo::ConstPtr &p) { gcData = *p; }

common::BodyTask turntoBall() 
{
    common::BodyTask btask;
    if(imuData.yaw<targetDir)
    {
        btask.turn = 30;  //30度 左转
    }
    else
    {
        btask.turn = -30;
    }
    btask.type = common::BodyTask::TASK_WALK;
    btask.count = 2;
    btask.lateral = 0;
    btask.step = 0;
    return btask;
}

common::BodyTask gotoBall(common::ObjInfo ball) 
{
    const float width = 640;
    const float height = 480;
    float alpha = (ball.x + ball.w / 2) / width - 0.5;
    float beta = (ball.y + ball.h / 2) / height - 0.5;
    common::BodyTask btask;
    btask.type = common::BodyTask::TASK_WALK;
    btask.count = 2;

    if (alpha > -0.05) {
        btask.lateral = -0.015;
    } else if (alpha < -0.12) {
        btask.lateral = 0.015;
    } else{
        btask.lateral = 0;
    }

    if (beta < 0.2) {
        btask.step = 0.03;
    } else {
        btask.step = 0;
        if(headAngles.pitch<70)
        {
            htask.pitch+=1;
        }
        
    } 
    return btask;
}

common::BodyTask penalty_kick(common::ObjInfo ball) 
{
    const float width = 640;
    const float height = 480;
    float alpha = (ball.x + ball.w / 2) / width - 0.5;
    float beta = (ball.y + ball.h / 2) / height - 0.5;
    common::BodyTask btask;
    btask.type = common::BodyTask::TASK_WALK;
    btask.count = 2;
    if (alpha > -0.05) {
        btask.lateral = -0.01;
        return btask;
    } else if (alpha < -0.12) {
        btask.lateral = 0.01;
        return btask;
    } else {
        if (beta < 0.32) {
            btask.step = 0.01;
            return btask;
        } else if (beta > 0.4) {
            btask.step = -0.01;
            return btask;
        } else {
            btask.type = common::BodyTask::TASK_ACT;
            btask.actname = "left_little_kick";
            return btask;
        }
    }
}

common::BodyTask get_ready()
{
    common::BodyTask btask;
    btask.type = BodyTask::TASK_ACT;
    btask.actname = "ready";
    btask.count = 1;
    btask.step = 0.00;
    btask.lateral = 0;
    btask.turn = 0;
    return btask;
}

void play()
{
    static unsigned int noResultCount = 100;  //连续三次看不到球才改变状态
    switch (fsmState)
    {
    case STATE_READY:
        fsmState = STATE_SEARCH_BALL;
        ROS_INFO("STATE_SEARCH_BALL");
        //break; //不加break

    case STATE_SEARCH_BALL:
        btask = get_ready();
        if(!imgResult.has_ball)
        {
            htask.mode = htask.ModeScanBall;
        }
        else
        {
            htask.mode = htask.ModeLookAt;
            htask.pitch = headAngles.pitch;
            htask.yaw = headAngles.yaw;
            float alpha = imgResult.ball.x / 640.0 - 0.5;  //左负右正，没有w
            targetDir = imuData.yaw + headAngles.yaw - alpha*50;  //有可能大于180或者小于-180
            ROS_INFO("imuData.yaw:%f  headAngles.yaw:%f  alpha: %f targetDir: %d",imuData.yaw, headAngles.yaw, alpha, targetDir);
            ROS_INFO("STATE_TURNTO_BALL");
            fsmState = STATE_TURNTO_BALL;
        }   
        break;

    case STATE_TURNTO_BALL:
        htask.mode = htask.ModeLookAt;
        htask.pitch = headAngles.pitch;
        htask.yaw = 0;  //此后htask.yaw一直是0

        int targetDir_;
        if(targetDir>180)
            targetDir_ = targetDir-360;
        else if(targetDir<-180)
            targetDir_ = targetDir+360;
        else
            targetDir_ = targetDir;
        if(abs(targetDir_-imuData.yaw)>5)  //角度误差大于5
        {
            btask = turntoBall();
        }
        else
        {
            fsmState = STATE_GOTO_BALL;
            ROS_INFO("STATE_GOTO_BALL");
        }
        
        break;

    case STATE_GOTO_BALL:
        if(!imgResult.has_ball)
        {
            btask = get_ready();
            if(noResultCount>0)
            {
                noResultCount--;
                break;
            }
            else
            {
                noResultCount = 100;
                fsmState = STATE_READY;
            }
                
        }
        else
        {
            noResultCount = 100;
            if(headAngles.pitch>=70)
            {
                float alpha = imgResult.ball.x  / 640.0 - 0.5;
                float beta = imgResult.ball.y / 480.0 - 0.5;
                if(alpha>-0.15 && alpha<0.5 && beta>0.1)
                {
                    ROS_INFO("STATE_KICK_BALL");
                    fsmState = STATE_KICK_BALL;
                }
            }
            btask = gotoBall(imgResult.ball);
        }  
        break;

    case STATE_KICK_BALL:
        if(!imgResult.has_ball)
        {
            btask = get_ready();
            if(noResultCount>0)
            {
                noResultCount--;
                break;
            }
            else
            {
                noResultCount = 100;
                fsmState = STATE_READY;
            }
                
        }
        else
        {
            noResultCount = 100;
            btask = penalty_kick(imgResult.ball);
            if(btask.actname == "left_little_kick")
            {
                ROS_INFO("penalty_kick!");
                fsmState = STATE_READY;
            }
        }  
        break;

    default:
        btask = get_ready();
        htask.mode = htask.ModeLookAt;
        htask.pitch = headAngles.pitch;
        htask.yaw = headAngles.yaw;
        fsmState = STATE_READY;
        break;
    }
    htask.pitch = (htask.pitch<20) ? 20:htask.pitch;
    htask.pitch = (htask.pitch>70) ? 70:htask.pitch;

    return;
}

int main(int argc, char **argv) {
    uint8_t id = 0;  // TODO: read from conf
    int gcstattmp = -1;
    ros::init(argc, argv, "strategy");
    ros::NodeHandle node;
    ros::Publisher bodyTaskPublisher =
        node.advertise<BodyTask>("/task/body", 1);
    ros::Publisher headTaskPublisher =
        node.advertise<HeadTask>("/task/head", 1);
    ros::Publisher ledTaskPublisher = node.advertise<LedTask>("/task/led", 1);
    ros::Publisher playerInfoPublisher =
        node.advertise<PlayerInfo>("/sensor/playerInfo", 1);
    ros::Subscriber imgResSubscriber =
        node.subscribe("/result/vision/imgproc", 1, imgResUpdate);
    ros::Subscriber headSubcriber =
        node.subscribe("/sensor/head", 1, headUpdate);
    ros::Subscriber imuSubcriber = node.subscribe("/sensor/imu", 1, imuUpdate);
    ros::Subscriber gcSubscriber = node.subscribe("/sensor/gctrl", 1, gcUpdate);

    // std::string cfgpath = ros::package::getPath("config") + "/conf/";
    // std::string config_str = cfgpath + "config.conf";
    // bpt::ptree pt;
    // get_tree_from_file(config_str, pt);
    // bpt::ptree ptstrategy = pt.get_child("strategy");
    // bpt::ptree ptguard = ptstrategy.get_child("guard");
    const float init_pos[2] = {-3.0, 0.0};
    const float start_pos[2] = {-3.0, 3.0};
    const float attack_range[2] = {-4.5, 0.0};

    ros::Rate rate(30);

    while (ros::ok()) {
        pinfo.id = id;

        if (gcstattmp != gcData.state) {
            gcstattmp = gcData.state;
            ROS_INFO("Game State Entering %s", GameState[gcstattmp]);  //每次gameCtrl状态更新提示
        }

        switch (gcstattmp) {
            case GC_INITIAL:  
                fsmState = STATE_READY;
                btask = get_ready();
                htask.mode = htask.ModeLookAt;
                htask.pitch = 0;
                htask.yaw = 0;
                break;

            case GC_READY:
                fsmState = STATE_READY;
                btask = get_ready();
                htask.mode = htask.ModeLookAt;
                htask.pitch = 45;
                htask.yaw = 0;
                pinfo.self_x = init_pos[0];
                pinfo.self_y = init_pos[1];
                break;

            case GC_SET:
                fsmState = STATE_READY;
                btask = get_ready();
                htask.mode = htask.ModeLookAt;
                htask.pitch = 0;
                htask.yaw = 0;
                break;

            case GC_PLAY:
                if(imuData.fall)
                {
                    fsmState = STATE_GETUP;
                }
                else
                {
                    if(fsmState == STATE_GETUP)
                    {
                        fsmState = STATE_READY;
                    }
                    //btask.type = common::BodyTask::TASK_ACT;
                    //btask.actname = "left_little_kick";
                    play();
                }
                break; 

            default:  //GC_FINISH
                fsmState = STATE_FINISH;
                btask.type = BodyTask::TASK_ACT;
                btask.actname = "reset";  //所有关节归零位
                htask.mode = htask.ModeLookAt;
                htask.pitch = 0;
                htask.yaw = 0;
                break;
        }
        headTaskPublisher.publish(htask);
        bodyTaskPublisher.publish(btask);
        playerInfoPublisher.publish(pinfo);
        ros::spinOnce();
        rate.sleep();
    }
}
