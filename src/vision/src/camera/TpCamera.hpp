#ifndef __TP_CAMERA_HPP
#define __TP_CAMERA_HPP

#include "Camera.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class TpCamera: public Camera
{
public:
    TpCamera(ros::NodeHandle &node, std::string topic);
    bool Open();
    bool Close();
    void Run();
    cv::Mat& GetData(uint32_t &t)
    {
        std::lock_guard<std::mutex> lk(raw_mutex_);
        t = time_;
        return raw_;
    }
    bool Available() { return true;}
    int Width(){ return width; }
    int Height(){ return height; }
    int Type(){ return type; }

    const int width = 640;
    const int height = 480;
    const int type = CV_8UC3;

    void ImageUpdate(const sensor_msgs::Image::ConstPtr &p);
private:
    ros::Subscriber subscriber_;
    cv::Mat raw_;
    uint32_t time_;
    std::string topic_;
    ros::NodeHandle &node_;
    mutable std::mutex raw_mutex_;
};

#endif
