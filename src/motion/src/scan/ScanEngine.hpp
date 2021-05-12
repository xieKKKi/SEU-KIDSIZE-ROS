#ifndef __SCAN_ENGINE_HPP
#define __SCAN_ENGINE_HPP

#include <bits/stdc++.h>
#include <eigen3/Eigen/Dense>
#include <common/HeadTask.h>
#include <common/HeadAngles.h>

class ScanEngine
{
public:
    ScanEngine();
    std::vector<common::HeadAngles> runScan(const common::HeadTask &task);
private:
    Eigen::Vector2f head_init_deg_;
    Eigen::Vector2f pitch_range_, yaw_range_;
    std::vector<Eigen::Vector2f> ball_search_table_;
};

#endif
