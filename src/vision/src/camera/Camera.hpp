#ifndef __CAMERA_HPP
#define __CAMERA_HPP

#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <common/common.hpp>
#define GET_CHANNELS(t) (((t)>>CV_CN_SHIFT)+1)
class Camera
{
public:
    Camera() = default;
    ~Camera(){ if(td_.joinable()) td_.join(); }
    virtual bool Open() = 0;
    virtual bool Close() = 0;
    virtual bool Available() = 0;
    virtual int Width() = 0;
    virtual int Height() = 0;
    virtual int Type() = 0;
    virtual void Run() = 0;
    virtual cv::Mat& GetData(uint32_t &t) = 0;
protected:
    bool alive_;
    std::thread td_;
};

#endif
