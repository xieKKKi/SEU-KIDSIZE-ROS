#include "team_monitor.hpp"
#include <config/basic_parser.hpp>
#include <ros/package.h>

using boost::asio::ip::udp;
using namespace std;

boost::asio::io_service udp_service;

bool parse_field(const std::string &filename, FieldInfo &field)
{
    common::bpt::ptree pt;
    if (!common::get_tree_from_file(filename, pt))
    {
        return false;
    }

    field.field_length = pt.get<int>("field_length");
    field.field_width = pt.get<int>("field_width");
    field.goal_depth = pt.get<int>("goal_depth");
    field.goal_width = pt.get<int>("goal_width");
    field.goal_height = pt.get<int>("goal_height");
    field.goal_area_length = pt.get<int>("goal_area_length");
    field.goal_area_width = pt.get<int>("goal_area_width");
    field.penalty_mark_distance = pt.get<int>("penalty_mark_distance");
    field.center_circle_diameter = pt.get<int>("center_circle_diameter");
    field.border_strip_width_min = pt.get<int>("border_strip_width_min");
    return true;
}


TeamMonitor::TeamMonitor(): socket_(udp_service, udp::endpoint(udp::v4(), TC_COMM_PORT))
{
    std::string cfgpath = ros::package::getPath("config")+"/conf/";
    std::string field_file = cfgpath+"model/field.conf";
    parse_field(field_file, field_);
    setFixedSize(field_.field_length + 2 * field_.border_strip_width_min, field_.field_width + 2 * field_.border_strip_width_min);
    setStyleSheet("background:green");
    td_ = std::move(std::thread([this]()
    {
        this->receive();
        udp_service.run();
    }));
}

TeamMonitor::~TeamMonitor()
{
    if(td_.joinable())
    {
        td_.join();
    }
}

void TeamMonitor::receive()
{
    socket_.async_receive_from(boost::asio::buffer((char *)&pkt_, sizeof(TeamCommData)), point_,
                               [this](boost::system::error_code ec, std::size_t bytes_recvd)
    {
        if (!ec && bytes_recvd > 0)
        {
            string recv_header;
            recv_header.append(pkt_.header, sizeof(TeamCommData::header));
            if(recv_header==TC_STRUCT_HEADER)
            {
                p_mutex_.lock();
                players_[pkt_.player.id] = pkt_.player;
                states_[pkt_.player.id] = pkt_.player.state;
                //cout<<pkt_.info.x<<'\t'<<pkt_.info.y<<'\t'<<pkt_.info.dir<<endll;
                p_mutex_.unlock();
                update();
            }
        }

        receive();
    });
}

void TeamMonitor::closeEvent(QCloseEvent *event)
{
    socket_.cancel();
    udp_service.stop();
    for(auto &m:state_monitors_)
        m.second->close();
    emit closed();
}

void TeamMonitor::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.translate(field_.field_length / 2 + field_.border_strip_width_min, field_.field_width / 2 + field_.border_strip_width_min);
    painter.setPen(QPen(Qt::white, 2, Qt::SolidLine, Qt::FlatCap));
    painter.drawEllipse(-field_.center_circle_diameter / 2, -field_.center_circle_diameter / 2, field_.center_circle_diameter, field_.center_circle_diameter);
    painter.drawLine(0, -field_.field_width / 2, 0, field_.field_width / 2);
    painter.drawLine(-field_.field_length / 2, -field_.field_width / 2, field_.field_length / 2, -field_.field_width / 2);
    painter.drawLine(-field_.field_length / 2, -field_.field_width / 2, -field_.field_length / 2, field_.field_width / 2);
    painter.drawLine(-field_.field_length / 2, field_.field_width / 2, field_.field_length / 2, field_.field_width / 2);
    painter.drawLine(field_.field_length / 2, -field_.field_width / 2, field_.field_length / 2, field_.field_width / 2);
    painter.drawLine(-field_.field_length / 2, -field_.goal_area_width / 2, -(field_.field_length / 2 - field_.goal_area_length), -field_.goal_area_width / 2);
    painter.drawLine(-field_.field_length / 2, field_.goal_area_width / 2, -(field_.field_length / 2 - field_.goal_area_length), field_.goal_area_width / 2);
    painter.drawLine(-(field_.field_length / 2 - field_.goal_area_length), -field_.goal_area_width / 2, -(field_.field_length / 2 - field_.goal_area_length), field_.goal_area_width / 2);
    painter.drawLine(-(field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, -field_.field_length / 2, -field_.goal_width / 2);
    painter.drawLine(-(field_.field_length / 2 + field_.goal_depth), field_.goal_width / 2, -field_.field_length / 2, field_.goal_width / 2);
    painter.drawLine(-(field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, -(field_.field_length / 2 + field_.goal_depth), field_.goal_width / 2);
    painter.drawLine(field_.field_length / 2, -field_.goal_area_width / 2, (field_.field_length / 2 - field_.goal_area_length), -field_.goal_area_width / 2);
    painter.drawLine(field_.field_length / 2, field_.goal_area_width / 2, (field_.field_length / 2 - field_.goal_area_length), field_.goal_area_width / 2);
    painter.drawLine((field_.field_length / 2 - field_.goal_area_length), -field_.goal_area_width / 2, (field_.field_length / 2 - field_.goal_area_length), field_.goal_area_width / 2);
    painter.drawLine((field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, field_.field_length / 2, -field_.goal_width / 2);
    painter.drawLine((field_.field_length / 2 + field_.goal_depth), field_.goal_width / 2, field_.field_length / 2, field_.goal_width / 2);
    painter.drawLine((field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, (field_.field_length / 2 + field_.goal_depth), field_.goal_width / 2);

    painter.setPen(QPen(Qt::white, 2, Qt::SolidLine, Qt::FlatCap));
    painter.setBrush(QBrush(Qt::white, Qt::NoBrush));
    painter.drawRect(-(field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, field_.goal_depth, field_.goal_width);
    painter.drawRect((field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, -field_.goal_depth, field_.goal_width);
    painter.setPen(QPen(Qt::white, 1, Qt::SolidLine, Qt::FlatCap));
    painter.setBrush(QBrush(Qt::white, Qt::SolidPattern));
    painter.drawRect(-4, -4, 8, 8);
    painter.drawRect((field_.field_length / 2 - field_.penalty_mark_distance) - 4, -4, 8, 8);
    painter.drawRect(-(field_.field_length / 2 - field_.penalty_mark_distance) + 4, -4, 8, 8);

    int ballsize = 10;
    painter.setBrush(QBrush(Qt::black, Qt::SolidPattern));
    p_mutex_.lock();

    for (auto &p : players_)
    {
        painter.setPen(QPen(Qt::black, 2, Qt::SolidLine, Qt::FlatCap));
        painter.setBrush(QBrush(Qt::black, Qt::SolidPattern));
        painter.save();
        float s=field_.scale;
        painter.translate(p.second.self_x * 100*s, -p.second.self_y * 100*s);
        painter.drawEllipse(-ballsize / 2, -ballsize / 2, ballsize, ballsize);
        painter.drawText(-ballsize / 2, -ballsize / 2, QString::number(p.second.id));
        painter.rotate(-p.second.self_direction);
        painter.drawLine(0, 0, 2 * ballsize, 0);
        painter.restore();

        painter.drawEllipse(p.second.ball_x * 100*s - ballsize / 2, -p.second.ball_y * 100*s - ballsize / 2, ballsize, ballsize);
        painter.drawText(p.second.ball_x*s * 100 - ballsize / 2, -p.second.ball_y*s * 100 - ballsize / 2, QString::number(p.second.id));

        if(state_monitors_.find(p.second.id) == state_monitors_.end())
        {
            state_monitors_[p.second.id] = new StateMonitor(p.second.id);
            state_monitors_[p.second.id]->show();
        }
        else
        {
            state_monitors_[p.second.id]->update_state(states_[p.second.id]);
        }
    }
    p_mutex_.unlock();
}

void TeamMonitor::keyPressEvent(QKeyEvent *event)
{
    switch (event->key())
    {
        case Qt::Key_C:
            p_mutex_.lock();
            players_.clear();
            p_mutex_.unlock();
            break;
        default:
            break;
    }
    this->update();
}