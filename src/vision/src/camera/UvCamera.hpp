#ifndef __UV_CAMERA_HPP
#define __UV_CAMERA_HPP

#include "Camera.hpp"

class UvCamera: public Camera
{
public:
    UvCamera(int idx);
    ~UvCamera();
    bool Open();
    bool Close();
    void Run();
    cv::Mat& GetData(uint32_t &t)
    {
        std::lock_guard<std::mutex> lk(raw_mutex_);
        t = time_;
        return raw_;
    }
    bool Available() { return cap_.isOpened();}
    int Width(){ return width; }
    int Height(){ return height; }
    int Type(){ return type; }

    const int width = 640;
    const int height = 480;
    const int type = CV_8UC3;

private:
    cv::VideoCapture cap_;
    cv::Mat raw_;
    int idx_;
    uint32_t time_;
    mutable std::mutex raw_mutex_;
};

#endif
