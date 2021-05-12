#include "motor.hpp"
#include "class_exception.hpp"
#include <seumath/math.hpp>
#include <fstream>
#include "logger.hpp"

using namespace std;
using namespace common;
using namespace seumath;
using namespace Eigen;
using namespace dynamixel;

Motor::Motor(): Timer(20)  //周期10ms
{
    is_open_motor = false;
    HeadAngles headAngles;
    ledstatus = 1;
    ms_period = 5;
}

bool Motor::start(string motorName, float motorVersion, int motorBaudrate)
{
    ROS_INFO("1");
    portHandler_ = PortHandler::getPortHandler(motorName.c_str());
    packetHandler_ = PacketHandler::getPacketHandler(motorVersion);
    gposWrite_ = make_shared<GroupSyncWrite>(portHandler_, packetHandler_, ADDR_GPOS, SIZE_GPOS);
    ledWrite_ = make_shared<GroupSyncWrite>(portHandler_, packetHandler_, ADDR_LED, SIZE_LED);
    torqWrite_ = make_shared<GroupSyncWrite>(portHandler_, packetHandler_, ADDR_TORQ, SIZE_TORQ);
    pposRead_ = make_shared<GroupSyncRead>(portHandler_, packetHandler_, ADDR_PPOS, SIZE_PPOS);
    ROS_INFO("2");
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
      ROS_INFO("333");
    }
    ROS_INFO("3");

    Timer::is_alive_ = true;
    start_timer();
    ROS_INFO("4");
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

