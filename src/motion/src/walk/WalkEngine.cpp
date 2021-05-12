#include "WalkEngine.hpp"
#include "CubicSpline.hpp"
#include "SmoothSpline.hpp"
#include "Polynom.hpp"
#include <seumath/math.hpp>
#include <cmath>
#include <fstream>
#include <ros/ros.h>
#include <common/BodyAngles.h>
#include <config/basic_parser.hpp>

using namespace Eigen;
using namespace seumath;
using namespace std;

WalkEngine::WalkEngine(std::string wkfile, std::shared_ptr<robot::Robot> rbt): rbt_(rbt)
{
    ROS_INFO("walk_file: %s", wkfile.c_str());
    common::bpt::ptree pt;
    bool ret = common::parse_file(wkfile, pt);
    if(!ret) return;
    XOffset_ = pt.get<double>("XOffset");
    YOffset_ = pt.get<double>("YOffset");
    DOffset_ = pt.get<double>("DOffset");
    
    xrange_ = common::get_config_vector<double, 2>(pt, "x");
    yrange_ = common::get_config_vector<double, 2>(pt, "y");
    drange_ = common::get_config_vector<double, 2>(pt, "dir");
    /**
     * Model leg typical length between
     * each rotation axis
     */
    params_.distHipToKnee = rbt_->get_bone("rthigh")->length;
    params_.distKneeToAnkle = rbt_->get_bone("rshank")->length;
    params_.distAnkleToGround = rbt_->get_bone("rfoot1")->length;
    params_.distFeetLateral = rbt_->get_bone("hip")->length;
    /**
     * Complete (two legs) walk cycle frequency
     * in Hertz
     */
    params_.freq =  pt.get<double>("freq");
    /**
     * Global gain multiplying all time
     * dependant movement between 0 and 1.
     * Control walk enabled/disabled smoothing.
     * 0 is walk disabled.
     * 1 is walk fully enabled
     */
    params_.enabledGain = 1.0;
    /**
     * Length of double support phase
     * in phase time
     * (between 0 and 1)
     * 0 is null double support and full single support
     * 1 is full double support and null single support
     */
    params_.supportPhaseRatio = pt.get<double>("doubleSupportRatio");
    /**
     * Lateral offset on default foot 
     * position in meters (foot lateral distance)
     * 0 is default
     * > 0 is both feet external offset
     */
    params_.footYOffset = pt.get<double>("footYOffset");
    /**
     * Forward length of each foot step
     * in meters
     * >0 goes forward
     * <0 goes backward
     * (dynamic parameter)
     */
    params_.stepGain = 0.0;
    /**
     * Vertical rise height of each foot
     * in meters (positive)
     */
    params_.riseGain = pt.get<double>("rise");
    /**
     * Angular yaw rotation of each 
     * foot for each step in radian.
     * 0 does not turn
     * >0 turns left
     * <0 turns right
     * (dynamic parameter)
     */
    params_.turnGain = 0.0;
    /**
     * Lateral length of each foot step
     * in meters.
     * >0 goes left
     * <0 goes right
     * (dynamic parameter)
     */
    params_.lateralGain = 0.0;
    /**
     * Vertical foot offset from trunk 
     * in meters (positive)
     * 0 is in init position
     * > 0 set the robot lower to the ground
     */
    params_.trunkZOffset = pt.get<double>("trunkZOffset");
    /**
     * Lateral trunk oscillation amplitude
     * in meters (positive)
     */
    params_.swingGain = pt.get<double>("swingGain");
    /**
     * Lateral angular oscillation amplitude
     * of swing trunkRoll in radian
     */
    params_.swingRollGain = 0.0;
    /**
     * Phase shift of lateral trunk oscillation
     * between 0 and 1
     */
    params_.swingPhase = pt.get<double>("swingPhase");
    /**
     * Foot X-Z spline velocities
     * at ground take off and ground landing.
     * Step stands for X and rise stands for Z
     * velocities.
     * Typical values ranges within 0 and 5.
     * >0 for DownVel is having the foot touching the
     * ground with backward velocity.
     * >0 for UpVel is having the foot going back
     * forward with non perpendicular tangent.
     */
    params_.stepUpVel = 4.0;
    params_.stepDownVel = 4.0;
    params_.riseUpVel = 4.0;
    params_.riseDownVel = 4.0;
    /**
     * Time length in phase time
     * where swing lateral oscillation
     * remains on the same side
     * between 0 and 0.5
     */
    params_.swingPause = 0.0;
    /**
     * Swing lateral spline velocity (positive).
     * Control the "smoothness" of swing trajectory.
     * Typical values are between 0 and 5.
     */
    params_.swingVel = 4.0;
    /**
     * Forward trunk-foot offset 
     * with respect to foot in meters
     * >0 moves the trunk forward
     * <0 moves the trunk backward
     */
    params_.trunkXOffset = pt.get<double>("trunkXOffset");
    /**
     * Lateral trunk-foot offset
     * with respect to foot in meters
     * >0 moves the trunk on the left
     * <0 moves the trunk on the right
     */
    params_.trunkYOffset = pt.get<double>("trunkYOffset");
    /**
     * Trunk angular rotation
     * around Y in radian
     * >0 bends the trunk forward
     * <0 bends the trunk backward
     */
    params_.trunkPitch = deg2rad(pt.get<double>("trunkPitch"));
    /**
     * Trunk angular rotation
     * around X in radian
     * >0 bends the trunk on the right
     * <0 bends the trunk on the left
     */
    params_.trunkRoll = 0.0;
    /**
     * Add extra offset on X, Y and Z
     * direction on left and right feet
     * in meters
     * (Can be used for example to implement 
     * dynamic kick)
     */
    params_.extraLeftX = 0.0;
    params_.extraLeftY = 0.0;
    params_.extraLeftZ = 0.0;
    params_.extraRightX = 0.0;
    params_.extraRightY = 0.0;
    params_.extraRightZ = 0.0;
    /**
     * Add extra angular offset on
     * Yaw, Pitch and Roll rotation of 
     * left and right foot in radians
     */
    params_.extraLeftYaw = 0.0;
    params_.extraLeftPitch = 0.0;
    params_.extraLeftRoll = 0.0;
    params_.extraRightYaw = 0.0;
    params_.extraRightPitch = 0.0;
    params_.extraRightRoll = 0.0;

    //The walk is started while walking on place
    params_.enabledGain = 0.0;
    params_.stepGain = 0.0;
    params_.lateralGain = 0.0;
    params_.turnGain = 0.0;

    engine_frequency_ = 1000.0/20;
    time_length_ = 1.0/params_.freq;

}

std::vector<common::BodyAngles> WalkEngine::runWalk(Eigen::Vector3d p, int steps, double& phase)
{
    params_.stepGain = p.x();
    params_.lateralGain = p.y();
    params_.turnGain = p.z();
    params_.enabledGain = steps>0?1.0:0.0;
    if(steps==0) steps=2;

    bound(xrange_[0], xrange_[1], params_.stepGain);
    bound(yrange_[0], yrange_[1], params_.lateralGain);
    bound(drange_[0], drange_[1], params_.turnGain);
    params_.turnGain = deg2rad(params_.turnGain);

    std::vector<common::BodyAngles> ret;

    Rhoban::IKWalkOutputs outputs;
    std::map<int, float> jdegs;
    for (double t=0.0;t<=steps*time_length_;t+=1.0/engine_frequency_) 
    {
        bool success = Rhoban::IKWalk::walk(params_, 1.0/engine_frequency_, phase, outputs);
        if (success)  
        {
            common::BodyAngles bAngles;
            bAngles.left_hip_yaw = rad2deg(outputs.left_hip_yaw);
            bAngles.left_hip_roll = rad2deg(outputs.left_hip_roll);
            bAngles.left_hip_pitch = rad2deg(outputs.left_hip_pitch);
            bAngles.left_knee = rad2deg(outputs.left_knee);
            bAngles.left_ankle_pitch = rad2deg(outputs.left_ankle_pitch);
            bAngles.left_ankle_roll = rad2deg(outputs.left_ankle_roll);

            bAngles.right_hip_yaw = rad2deg(outputs.right_hip_yaw);
            bAngles.right_hip_roll = rad2deg(outputs.right_hip_roll);
            bAngles.right_hip_pitch = rad2deg(outputs.right_hip_pitch);
            bAngles.right_knee = rad2deg(outputs.right_knee);
            bAngles.right_ankle_pitch = rad2deg(outputs.right_ankle_pitch);
            bAngles.right_ankle_roll = rad2deg(outputs.right_ankle_roll);

            bAngles.left_shoulder = 0;
            bAngles.left_elbow = -170;
            bAngles.right_shoulder = 0;
            bAngles.right_elbow = 170;
            ret.push_back(bAngles);
        }
        else{
            ROS_WARN("kinematics failed");
        }
    }
    return ret;
}