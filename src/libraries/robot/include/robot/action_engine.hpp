#ifndef __ACTION_ENGINE_HPP
#define __ACTION_ENGINE_HPP

#include <ros/ros.h>
#include <robot/robot.hpp>
#include <seumath/math.hpp>
#include <common/BodyAngles.h>

namespace robot
{
    class ActionEngine
    {
    public:
        ActionEngine(std::string act_file, std::shared_ptr<robot::Robot> rbt);
        std::vector<common::BodyAngles> runAction(std::string act, int idx=100);
        std::vector<std::string> getActions();

        robot::ActMap& get_act_map()
        {
            return act_map_;
        }

        robot::PosMap& get_pos_map()
        {
            return pos_map_;
        }

    private:
        std::vector< std::map<robot::RobotMotion, robot::RobotPose> > 
                get_poses(std::map<robot::RobotMotion, robot::RobotPose> &pos1,
                std::map<robot::RobotMotion, robot::RobotPose> &pos2, int act_time);

        bool get_degs(robot::PoseMap &act_pose, common::BodyAngles &bAngles);

    private:
        robot::ActMap act_map_;
        robot::PosMap pos_map_;
        std::shared_ptr<robot::Robot> rbt_;
    };
} // namespace robot


#endif
