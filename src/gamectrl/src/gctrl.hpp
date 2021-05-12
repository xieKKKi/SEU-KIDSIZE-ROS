#ifndef __GAME_CTRL_HPP
#define __GAME_CTRL_HPP

#include <bits/stdc++.h>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include "gcdata/RoboCupGameControlData.h"

class GameCtrl
{
public:
    GameCtrl(ros::NodeHandle &node, boost::asio::io_service &svc);
    bool start();
    void stop();
private:
    void receive();
    enum {gc_data_size = sizeof(RoboCupGameControlData)};
    RoboCupGameControlData data_;
    RoboCupGameControlReturnData retData_;
    boost::asio::ip::udp::socket recv_socket_;
    boost::asio::ip::udp::endpoint recv_point_;
    boost::asio::ip::udp::socket send_socket_;
    boost::asio::ip::udp::endpoint send_point_;
    ros::Publisher gcPublisher;
    int teamId, playerId, teamIdx;
};

#endif