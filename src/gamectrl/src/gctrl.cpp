// 发布比赛状况控制信息
#include "gctrl.hpp"
#include <common/GcInfo.h>

using namespace std;
using boost::asio::ip::udp;


GameCtrl::GameCtrl(ros::NodeHandle &node, boost::asio::io_service &svc)
    :recv_socket_ (svc, udp::endpoint(udp::v4(), GAMECONTROLLER_DATA_PORT)),
    send_socket_(svc, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)),
    send_point_(boost::asio::ip::address_v4::broadcast(), GAMECONTROLLER_RETURN_PORT)
{
    gcPublisher = node.advertise<common::GcInfo>("/sensor/gctrl", 1);
    try
    {
        ros::param::get("id", playerId);
        ros::param::get("team_number", teamId);
    }
    catch (ros::InvalidNameException &e)
    {
        ROS_ERROR("%s", e.what());
    }
    teamIdx = -1;
    retData_.player = playerId;
    retData_.message = GAMECONTROLLER_RETURN_MSG_ALIVE;
    retData_.team = teamId;
}

bool GameCtrl::start()
{
    this->receive();
    return true;
}

void GameCtrl::receive()
{
    if(!ros::ok()) return;
    recv_socket_.async_receive_from(boost::asio::buffer((char *)&data_, gc_data_size), recv_point_,
                                    [this](boost::system::error_code ec, std::size_t bytes_recvd)
    {
        if (!ec && bytes_recvd > 0)
        {
            string recv_header;
            recv_header.append(data_.header, sizeof(RoboCupGameControlData::header));
            if (recv_header == GAMECONTROLLER_STRUCT_HEADER)
            {
                if(teamIdx < 0)
                {
                    teamIdx = data_.teams[0].teamNumber == teamId ? 0 : 1;
                }
                common::GcInfo info;
                info.state = data_.state;
                info.firstHalf = data_.firstHalf;
                info.kickOffTeam = data_.kickOffTeam;
                info.secondaryState = data_.secondaryState;
                info.secsRemaining = data_.secsRemaining;
                info.secondaryTime = data_.secondaryTime;
                info.penalty = data_.teams[teamIdx].players[playerId-1].penalty;
                gcPublisher.publish(info);
                send_socket_.async_send_to(boost::asio::buffer((char *)(&retData_), 
                    sizeof(RoboCupGameControlReturnData)), send_point_,
                    [this](boost::system::error_code ec, std::size_t bytes_sent) {});
            }
        }
        receive();
    });
}

void GameCtrl::stop()
{
    recv_socket_.cancel();
    recv_socket_.close();
}
