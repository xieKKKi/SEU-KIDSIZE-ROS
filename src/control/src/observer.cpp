#include <common/BodyAngles.h>
#include <common/BodyTask.h>
#include <common/GcInfo.h>
#include <common/HeadAngles.h>
#include <common/HeadTask.h>
#include <common/ImageResult.h>
#include <common/ImuData.h>
#include <common/LedTask.h>
#include <common/PlayerInfo.h>
#include <common/ObjInfo.h>

#include <ros/package.h>
#include <ros/ros.h>

using namespace common;

ImageResult imgResult;
HeadAngles headAngles;
ImuData imuData;
GcInfo gcData;
HeadTask headTask;
BodyTask bodyTask;
LedTask ledTask;


bool showImageResult = true; 
bool showheadAngles = false;
bool showimuData = false;
bool showgcData = false;
bool showbodyTask = false;
bool showheadTask = false;
bool showledTask = false;


void imgResUpdate(const ImageResult::ConstPtr &p) 
{ 
    imgResult = *p; 
    if(showImageResult)
    {
        ROS_INFO("ImageResult has_ball: %d has_post: %d", imgResult.has_ball, imgResult.has_post);
        if(imgResult.has_ball)
        {
            ObjInfo ballBox = imgResult.ball;
            ROS_INFO("Ball X:%d Y:%d W:%d H:%d", ballBox.x,ballBox.y,ballBox.w,ballBox.h);
        }
    }
}
void headUpdate(const HeadAngles::ConstPtr &p) 
{ 
    headAngles = *p; 
    if(showheadAngles)
    {
        ROS_INFO("has_ball: %s %d", imgResult.has_ball);
    }
}
void imuUpdate(const ImuData::ConstPtr &p)
{ 
    imuData = *p; 
    if(showimuData)
    {
        ROS_INFO("has_ball: %s %d", imgResult.has_ball);
    }
}
void gcUpdate(const GcInfo::ConstPtr &p) 
{ 
    gcData = *p; 
    if(showgcData)
    {
        ROS_INFO("GcInfo: %d", gcData.state);
    }
}
void UpdateBodyTask(const common::BodyTask::ConstPtr &p) 
{ 
    bodyTask = *p; 
    if(showbodyTask)
    {
        ROS_INFO("bodyTask: %d %d", bodyTask.type, bodyTask.count);
    }
}
void UpdateHeadTask(const common::HeadTask::ConstPtr &p) 
{ 
    headTask = *p; 
    if(showheadTask)
    {
        ROS_INFO("headTask: %d %f %f", headTask.mode, headTask.pitch, headTask.yaw);
    }
}
void LedTaskUpdate(const LedTask::ConstPtr &p)
{ 
    ledTask = *p; 
    if(showledTask)
    {
        ROS_INFO("has_ball: %s %d", imgResult.has_ball);
    }
}




int main(int argc, char **argv) {
    ros::init(argc, argv, "observer");
    ros::NodeHandle node;

    ros::Subscriber imgResSubscriber =  node.subscribe("/result/vision/imgproc", 1, imgResUpdate);
    ros::Subscriber headSubcriber = node.subscribe("/sensor/head", 1, headUpdate);
    ros::Subscriber imuSubcriber = node.subscribe("/sensor/imu", 1, imuUpdate);
    ros::Subscriber gcSubscriber = node.subscribe("/sensor/gctrl", 1, gcUpdate);
    ros::Subscriber bodyTaskSub = node.subscribe("/task/body", 1, UpdateBodyTask);
    ros::Subscriber headTaskSub = node.subscribe("/task/head", 1, UpdateHeadTask);
    ros::Subscriber ledSub = node.subscribe("/task/led", 1, LedTaskUpdate);

    while (ros::ok()) 
    {
        ros::spinOnce();
    }
}