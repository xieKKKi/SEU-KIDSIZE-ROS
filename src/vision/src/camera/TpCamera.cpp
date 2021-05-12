#include "TpCamera.hpp"
#include <unistd.h>

using namespace cv;

TpCamera::TpCamera(ros::NodeHandle &node, std::string topic): Camera(), 
    node_(node), topic_(topic), raw_(height, width, type)
{
    alive_ = false;
}

bool TpCamera::Open()
{
    subscriber_ = node_.subscribe(topic_, 1, &TpCamera::ImageUpdate, this);
    return true;
}

bool TpCamera::Close()
{
    return true;
}

void TpCamera::Run()
{
    
}

void TpCamera::ImageUpdate(const sensor_msgs::Image::ConstPtr &p)
{
    raw_mutex_.lock();
    time_ = get_clock();
    memcpy(raw_.data, &(p->data[0]), width*height*3);
    raw_mutex_.unlock();
}