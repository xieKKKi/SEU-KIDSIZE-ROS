#include <common/HeadAngles.h>
#include <sensor_msgs/image_encodings.h>
#include <cstdlib>
#include <fstream>
#include <seumath/math.hpp>
#include "SimRobot.hpp"

using namespace std;
using namespace webots;
using namespace common;

const char *Neck[2] = { "Neck", "Neck2" };
const char *LeftArmJoint[2] = { "LeftShoulder", "LeftElbow" };
const char *RightArmJoint[2] = { "RightShoulder", "RightElbow" };
const char *LeftLegJoint[6] = { "LeftLegX", "LeftLegY", "LeftLegZ", "LeftKnee", "LeftAnkleX", "LeftAnkleY" };
const char *RightLegJoint[6] = { "RightLegX", "RightLegY", "RightLegZ", "RightKnee", "RightAnkleX", "RightAnkleY" };
const double fall_thresh = 0.75;

SimRobot::SimRobot(ros::NodeHandle *node) : Robot()
{
  totalTime = 0;
  mTimeStep = getBasicTimeStep();
  mCamera = getCamera("Camera");
  mCamera->enable(5 * mTimeStep);
  mIMU = getInertialUnit("IMU");
  mIMU->enable(mTimeStep);
  mLEDs.resize(2);
  mLEDs[0] = getLED("Led0");
  mLEDs[1] = getLED("Led1");

  fallType = ImuData::FALL_NONE;
  mInitYaw = 0.0;
  imuReset = true;
  mHAngles.yaw = 0.0;
  mHAngles.pitch = 0.0;
  ledTask.led1 = 1;
  ledTask.led2 = 1;
  mNode = node;
  mImagePublisher = mNode->advertise<sensor_msgs::Image>("/sensor/image", 1);
  mHeadPublisher = mNode->advertise<common::HeadAngles>("/sensor/head", 1);
  mImuPublisher = mNode->advertise<common::ImuData>("/sensor/imu", 1);
  imuRstService = mNode->advertiseService("/imu_reset", &SimRobot::ResetImuService, this);
  mLedSubscriber = mNode->subscribe("/task/led", 1, &SimRobot::LedTaskUpdate, this);
}

void SimRobot::LedTaskUpdate(const common::LedTask::ConstPtr &p)
{
  ledTask = *p;
}

void SimRobot::PublishImage()
{
  const unsigned char *data = mCamera->getImage();
  int h = mCamera->getHeight(), w = mCamera->getWidth();

  if (data != NULL)
  {
    sensor_msgs::Image image;
    image.header.stamp = ros::Time::now();
    image.header.frame_id = "camera";
    image.width = w;
    image.height = h;
    image.step = 3 * w;
    image.encoding = sensor_msgs::image_encodings::RGB8;
    image.data.resize(3 * w * h);
    for (int i = 0; i < h; i++)
    {
      for (int j = 0; j < w; j++)
      {
        image.data[i * w * 3 + j * 3 + 0] = *(data + i * w * 4 + j * 4 + 2);
        image.data[i * w * 3 + j * 3 + 1] = *(data + i * w * 4 + j * 4 + 1);
        image.data[i * w * 3 + j * 3 + 2] = *(data + i * w * 4 + j * 4 + 0);
      }
    }
    mImagePublisher.publish(image);
  }
}

bool SimRobot::ResetImuService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  imuReset = true;
}

int SimRobot::myStep()
{
  common::HeadAngles head;
  head.yaw = seumath::rad2deg(mHAngles.yaw);
  head.pitch = seumath::rad2deg(mHAngles.pitch);
  mHeadPublisher.publish(head);
  mLEDs[0]->set(0xff0000*ledTask.led1);
  mLEDs[1]->set(0xff0000*ledTask.led2);
  setPositions();
  checkFall();
  totalTime += mTimeStep;
  if (totalTime % (5 * mTimeStep) == 0)
    PublishImage();
  return step(mTimeStep);
}

void SimRobot::checkFall()
{
  const double *rpy = mIMU->getRollPitchYaw();
  if (rpy[1] > fall_thresh)
    fallType = ImuData::FALL_BACKWARD;
  else if (rpy[1] < -fall_thresh)
    fallType = ImuData::FALL_FORWARD;
  else if (rpy[0] < -fall_thresh)
    fallType = ImuData::FALL_LEFT;
  else if (rpy[0] > fall_thresh)
    fallType = ImuData::FALL_RIGHT;
  else
    fallType = ImuData::FALL_NONE;
  ImuData imu;
  imu.yaw = seumath::rad2deg(rpy[2]);
  imu.pitch = seumath::rad2deg(rpy[1]);
  imu.roll = seumath::rad2deg(rpy[0]);
  imu.fall = fallType;
  imu.stamp = ros::Time::now().toNSec();
  if(imuReset)
  {
    imuReset = false;
    mInitYaw = imu.yaw;
  }
  imu.yaw = seumath::normalizeRad<double>(imu.yaw - mInitYaw);
  mImuPublisher.publish(imu);
  // printf("roll=%f, pitch=%f, yaw=%f\n", rpy[0], rpy[1], rpy[2]);
  // printf("%d\n", fallType);
}

void SimRobot::wait(int ms)
{
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

void SimRobot::setPositions()
{
  Motor *motor;
  motor = getMotor(LeftLegJoint[0]);
  motor->setPosition(seumath::deg2rad(mAngles.left_hip_roll));
  motor = getMotor(LeftLegJoint[1]);
  motor->setPosition(seumath::deg2rad(mAngles.left_hip_pitch));
  motor = getMotor(LeftLegJoint[2]);
  motor->setPosition(seumath::deg2rad(-mAngles.left_hip_yaw));
  motor = getMotor(LeftLegJoint[3]);
  motor->setPosition(seumath::deg2rad(mAngles.left_knee));
  motor = getMotor(LeftLegJoint[4]);
  motor->setPosition(seumath::deg2rad(mAngles.left_ankle_roll));
  motor = getMotor(LeftLegJoint[5]);
  motor->setPosition(seumath::deg2rad(mAngles.left_ankle_pitch));

  motor = getMotor(RightLegJoint[0]);
  motor->setPosition(seumath::deg2rad(mAngles.right_hip_roll));
  motor = getMotor(RightLegJoint[1]);
  motor->setPosition(seumath::deg2rad(mAngles.right_hip_pitch));
  motor = getMotor(RightLegJoint[2]);
  motor->setPosition(seumath::deg2rad(-mAngles.right_hip_yaw));
  motor = getMotor(RightLegJoint[3]);
  motor->setPosition(seumath::deg2rad(mAngles.right_knee));
  motor = getMotor(RightLegJoint[4]);
  motor->setPosition(seumath::deg2rad(mAngles.right_ankle_roll));
  motor = getMotor(RightLegJoint[5]);
  motor->setPosition(seumath::deg2rad(mAngles.right_ankle_pitch));

  motor = getMotor(LeftArmJoint[0]);
  motor->setPosition(seumath::deg2rad(mAngles.left_shoulder));
  motor = getMotor(LeftArmJoint[1]);
  motor->setPosition(seumath::deg2rad(mAngles.left_elbow));
  motor = getMotor(RightArmJoint[0]);
  motor->setPosition(seumath::deg2rad(-mAngles.right_shoulder));
  motor = getMotor(RightArmJoint[1]);
  motor->setPosition(seumath::deg2rad(-mAngles.right_elbow));

  motor = getMotor(Neck[1]);
  motor->setPosition(seumath::deg2rad(-mHAngles.pitch));
  motor = getMotor(Neck[0]);
  motor->setPosition(seumath::deg2rad(mHAngles.yaw));
}


