#include "UvCamera.hpp"
#include <unistd.h>

using namespace cv;

UvCamera::UvCamera(int idx): Camera(), idx_(idx), 
    raw_(height, width, type)
{
    alive_ = false;
}

UvCamera::~UvCamera()
{
    if(cap_.isOpened())
        cap_.release();
}

bool UvCamera::Open()
{
    try
    {
        cap_.open(idx_);
        cap_.set(CV_CAP_PROP_FRAME_WIDTH, width);
        cap_.set(CV_CAP_PROP_FRAME_HEIGHT, height);
        printf("Open UvCamera [%d] success!\n", idx_);
        return true;
    }
    catch(const std::exception& e)
    {
        printf("Open UvCamera [%d] failed!\n", idx_);
        return false;
    }
}

bool UvCamera::Close()
{
    alive_ = false;
    return true;
}

void UvCamera::Run()
{
    alive_ = true;
    td_ = std::move(std::thread([this](){
        Mat tmp;
        while(alive_)
        {
            if(cap_.read(tmp))
            {
                raw_mutex_.lock();
                time_ = get_clock();
                cvtColor(tmp, raw_, CV_BGR2RGB);
                raw_mutex_.unlock();
            }
            waitKey(30);
        }
    }));
}