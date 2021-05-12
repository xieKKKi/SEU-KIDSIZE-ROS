#ifndef __MV_CAMERA_HPP
#define __MV_CAMERA_HPP

#include "Camera.hpp"
#include <CameraApi.h>

class MvCamera: public Camera
{
public:
    MvCamera(std::string cfg);
    ~MvCamera();
    bool Open();
    bool Close();
    void Run();
    cv::Mat& GetData(uint32_t &t)
    {
        std::lock_guard<std::mutex> lk(raw_mutex_);
        t = time_;
        return raw_;
    }
    bool Available() { return camera_fd_>0;}
    int Width(){ return width; }
    int Height(){ return height; }
    int Type(){ return type; }

    const int width = 1280;
    const int height = 960;
    const int type = CV_8UC1;

private:
    bool InitSDK(std::string cfg);

private:
    int camera_fd_;
    cv::Mat raw_;
    uint32_t time_;
    mutable std::mutex raw_mutex_;
    std::string cfg_file_;
};

#endif
