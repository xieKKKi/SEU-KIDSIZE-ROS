#include "ScanEngine.hpp"
#include <seumath/math.hpp>
#include <ros/ros.h>
#include <common/HeadAngles.h>

using namespace std;
using namespace seumath;
using namespace Eigen;

const float search_post_div = 0.8;
const float head_pitch_min_angle = 0.0f;
const float head_pitch_mid_angle = 30.0f;
const float head_pitch_max_angle = 60.0f;
const int scan_post = 8;
const float post_search_table[][2] = 
{
    {head_pitch_min_angle, 90.0},
    {head_pitch_min_angle, -90.0},
    {head_pitch_mid_angle, -90.0},
    {head_pitch_mid_angle, 90.0}
};

ScanEngine::ScanEngine(): pitch_range_(20.0, 70.0),  //pitch_range_(0.0, 70.0), 
    yaw_range_(-110.0, 110.0), head_init_deg_(0.0, 40.0)
{
    vector<float> head_pitch_table = {pitch_range_[1], (pitch_range_[0]+pitch_range_[1])/2.0f, pitch_range_[0]};
    const float bottom_yaw = 50.0f;
    vector<float> head_yaw_range = {bottom_yaw, (yaw_range_[1]+bottom_yaw)/2.0f, yaw_range_[1]};
    vector<int> div_counts = {3, 4, 5};
    float flag = 1.0;
    for(int i=0; i<3; i++)
    {
        float yaw_div = 2*head_yaw_range[i]/(div_counts[i]-1);
        for(int j=0; j<div_counts[i]; j++)
            ball_search_table_.push_back(Vector2f(head_pitch_table[i], flag*(head_yaw_range[i]-j*yaw_div)));
        flag *= -1.0;
    }
}

std::vector<common::HeadAngles> ScanEngine::runScan(const common::HeadTask &task)
{
    std::vector<common::HeadAngles> res;
    common::HeadAngles hAngles;
    if(task.mode == common::HeadTask::ModeScanBall)
    {
        for(int i=0; i<ball_search_table_.size(); i++)
        {
            for(int j=0; j<50; j++)
            {
                hAngles.pitch = ball_search_table_[i][0];
                hAngles.yaw = ball_search_table_[i][1];
                res.push_back(hAngles);
            }
        }
    }
    else if(task.mode == common::HeadTask::ModeScanPost)
    {
        for(int i=1;i>=0;i--)
        {
            hAngles.pitch = post_search_table[i*2][0];
            for(float ya=post_search_table[i*2][1]; fabs(ya)<=fabs(post_search_table[i*2+1][1])+0.1; 
                    ya+=pow(-1, i+1)*search_post_div)
            {
                hAngles.yaw = ya;
                res.push_back(hAngles);
            }
        }
    }
    else
    {
        hAngles.pitch = task.pitch;
        hAngles.yaw = task.yaw;
        res.push_back(hAngles);
    }
    return res;
}
