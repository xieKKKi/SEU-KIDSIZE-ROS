#ifndef __UNIROBOT_HPP
#define __UNIROBOT_HPP

#include <ros/ros.h>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/LED.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/PositionSensor.hpp>
#include <sensor_msgs/Image.h>
#include <common/BodyAngles.h>
#include <common/HeadAngles.h>
#include <common/LedTask.h>
#include <std_srvs/Empty.h>
#include <common/ImuData.h>


class SimRobot: public webots::Robot
{
public:
    SimRobot(ros::NodeHandle *node);
    int myStep();

    common::BodyAngles mAngles;
    common::HeadAngles mHAngles;

private:
    int mTimeStep;
    int totalTime;

    ros::NodeHandle *mNode;
    ros::Publisher mImagePublisher;
    ros::Publisher mImuPublisher;
    ros::Publisher mHeadPublisher;
    ros::Subscriber mLedSubscriber;
    ros::ServiceServer imuRstService;
    bool ResetImuService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void LedTaskUpdate(const common::LedTask::ConstPtr &p);
    void PublishImage();
    void setPositions();
    void checkFall();
    
    void wait(int ms);

    webots::Camera *mCamera;
    webots::InertialUnit *mIMU;
    std::vector<webots::LED*> mLEDs;

    common::LedTask ledTask;
    int fallType;
    bool imuReset;
    double mInitYaw;
};


#endif
