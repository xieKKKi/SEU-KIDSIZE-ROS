#pragma once

#ifndef __MOTOR_HPP
#define __MOTOR_HPP

#include <memory>
#include <list>
#include <mutex>
#include <atomic>
#include <map>
#include <time.h>
#include "timer.hpp"
#include "dynamixel/dynamixel_sdk.h"
#include <common/BodyAngles.h>
#include <common/HeadAngles.h>
#include <common/GetAngles.h>
#include <ros/ros.h>
#include <vector>
//#include "../motion.cpp"

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
    int ms_period;
    int start_id;
    std::vector<float> degs;

    std::shared_ptr<dynamixel::PortHandler> portHandler_;
    std::shared_ptr<dynamixel::PacketHandler> packetHandler_;
    std::shared_ptr<dynamixel::GroupSyncWrite> torqWrite_;
    std::shared_ptr<dynamixel::GroupSyncWrite> gposWrite_;
    std::shared_ptr<dynamixel::GroupSyncWrite> ledWrite_;
    std::shared_ptr<dynamixel::GroupSyncRead> pposRead_;
};

#endif
