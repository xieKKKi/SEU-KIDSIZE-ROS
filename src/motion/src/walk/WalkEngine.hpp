#ifndef __WALK_ENGINE_HPP
#define __WALK_ENGINE_HPP

#include <bits/stdc++.h>
#include <eigen3/Eigen/Dense>
#include "IKWalk.hpp"
#include <ros/ros.h>
#include <robot/robot.hpp>
#include <common/BodyAngles.h>

class WalkEngine
{
public:
    WalkEngine(std::string wkfile, std::shared_ptr<robot::Robot> rbt);
    std::vector<common::BodyAngles> runWalk(Eigen::Vector3d p, int steps, double& phase);
    
private:
    double engine_frequency_;
    double time_length_;
    double XOffset_, YOffset_, DOffset_;
    Rhoban::IKWalkParameters params_;
    Eigen::Vector2d xrange_, yrange_, drange_;
    std::shared_ptr<robot::Robot> rbt_;
};

#endif
