#include "MvCamera.hpp"
#include <unistd.h>

using namespace cv;

MvCamera::MvCamera(std::string cfg): Camera(), camera_fd_(-1),
    cfg_file_(cfg), raw_(height, width, type)
{
    alive_ = false;
}

MvCamera::~MvCamera()
{
    if(camera_fd_>0)
        CameraUnInit(camera_fd_);
}

bool MvCamera::Open()
{
    if(!InitSDK(cfg_file_))
    {
        printf("Open MvCamera failed!\n");
        return false;
    }
    printf("Open MvCamera success!\n");
    return true;
}

bool MvCamera::Close()
{
    alive_ = false;
    return true;
}

void MvCamera::Run()
{
    alive_ = true;
    td_ = std::move(std::thread([this](){
        tSdkFrameHead sFrameInfo;
        uint8_t *buffer = nullptr;
        while(alive_)
        {
            CameraSdkStatus status = CameraGetImageBuffer(camera_fd_, &sFrameInfo, &buffer, 1000);
            if (status == CAMERA_STATUS_SUCCESS)
            {
                raw_mutex_.lock();
                time_ = get_clock();
                memcpy(raw_.data, buffer, width*height);
                raw_mutex_.unlock();
                CameraReleaseImageBuffer(camera_fd_, buffer);
            }
            usleep(100);
        }
    }));
}

bool MvCamera::InitSDK(std::string cfg)
{
    int                     iCameraCounts = 1;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList;
    tSdkFrameHead           g_tFrameHead;       //图像帧头信息
    tSdkCameraCapbility     g_tCapability;      //设备描述信息

    //sdk初始化  0 English 1中文
    CameraSdkInit(1);

    //枚举设备，并建立设备列表
    CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
    //没有连接设备
    if(iCameraCounts==0){
        printf("No device found!\n");
        return false;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&camera_fd_);
    //初始化失败
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        printf("Init failed!\n");
        return false;
    }
    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(camera_fd_,&g_tCapability);
    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(camera_fd_);
    printf("Init camera with file: %s\n", cfg.c_str());
    CameraReadParameterFromFile(camera_fd_, (char*)cfg.c_str());
    return true;
}