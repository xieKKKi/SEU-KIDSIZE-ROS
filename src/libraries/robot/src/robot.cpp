#include <robot/robot.hpp>
#include <config/basic_parser.hpp>
#include <ros/ros.h>
#include <unordered_set>

namespace robot
{
#define acts_key_ "acts"
#define poses_key_ "poses"

using namespace std;
using namespace Eigen;
using namespace seumath;
using namespace common;

robot::JointPtr parse_joint(const std::string &j_name, const bpt::ptree &pt,
                            robot::BoneMap &BoneMap, robot::JointMap &JointMap);
robot::BonePtr parse_bone(const bpt::ptree &pt, robot::BoneMap &BoneMap,
                          robot::JointMap &JointMap);

static RobotPose get_pose_from_string(std::string str)
{
    std::stringstream ss;
    ss << str;
    robot::RobotPose pose;
    ss >> pose.x;
    ss >> pose.y;
    ss >> pose.z;
    ss >> pose.pitch;
    ss >> pose.roll;
    ss >> pose.yaw;
    return pose;
}

static bool pos_exist(const std::string &name, const PosMap &poses)
{
    if (poses.find(name) != poses.end())
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool parse(std::string act_file, ActMap &acts, PosMap &poses)
{
    bpt::ptree pt;
    acts.clear();
    poses.clear();

    if (!get_tree_from_file(act_file, pt))
        return false;

    bpt::ptree act_pt, pos_pt;

    try
    {
        act_pt = pt.get_child(acts_key_);
        pos_pt = pt.get_child(poses_key_);
    }
    catch (bpt::ptree_error &e)
    {
        return false;
    }

    robot::RobotPos t_pos;

    for (auto &pos : pos_pt)
    {
        t_pos.name.clear();
        t_pos.pose_info.clear();
        t_pos.name = pos.first;

        for (auto &info : pos.second)
        {
            t_pos.pose_info[robot::get_motion_by_name(info.first)] = get_pose_from_string(info.second.data());
        }

        poses[t_pos.name] = t_pos;
    }

    robot::RobotAct t_act;
    robot::RobotOnePos t_one_pos;

    for (auto &act : act_pt)
    {
        t_act.name.clear();
        t_act.poses.clear();
        t_act.name = act.first;

        for (auto &pos : act.second)
        {
            if (pos_exist(pos.first, poses))
            {
                t_one_pos.act_time = 0;
                t_one_pos.pos_name.clear();
                t_one_pos.pos_name = pos.first;
                t_one_pos.act_time = pos.second.get_value<int>();
                t_act.poses.push_back(t_one_pos);
            }
            else
            {
                return false;
            }
        }

        acts[t_act.name] = t_act;
    }
    return true;
}

std::string get_string_from_pose(const RobotPose &pose)
{
    std::string str = "";
    str += (std::to_string(pose.x) + " ");
    str += (std::to_string(pose.y) + " ");
    str += (std::to_string(pose.z) + " ");
    str += (std::to_string(pose.pitch) + " ");
    str += (std::to_string(pose.roll) + " ");
    str += (std::to_string(pose.yaw));
    ;
    return str;
}

void save(const std::string &act_file, const ActMap &acts, const PosMap &poses)
{
    bpt::ptree pt;
    bpt::ptree act_pt, pos_pt;

    std::unordered_set<std::string> saved_poses;

    bpt::ptree act_info_child;
    bpt::ptree pos_info_child;

    for (auto &act : acts)
    {
        act_info_child.clear();

        for (size_t i = 0; i < act.second.poses.size(); i++)
        {
            std::string pos_name = act.second.poses[i].pos_name;
            act_info_child.add<int>(pos_name, act.second.poses[i].act_time);
            if (saved_poses.find(pos_name) != saved_poses.end())
                continue;
            pos_info_child.clear();
            auto pos_iter = poses.find(pos_name);
            for (auto &p_info : pos_iter->second.pose_info)
            {
                pos_info_child.add(get_name_by_motion(p_info.first), get_string_from_pose(p_info.second));
            }
            pos_pt.add_child(pos_name, pos_info_child);
            saved_poses.insert(pos_name);
        }

        act_pt.add_child(act.second.name, act_info_child);
    }

    pt.add_child(acts_key_, act_pt);
    pt.add_child(poses_key_, pos_pt);
    write_tree_to_file(act_file, pt);
}

Robot::Robot(const std::string &robot_file, const std::string &offset_file)
{
    bpt::ptree pt;
    start_id_ = 0xff;
    if (!get_tree_from_file(robot_file, pt))
        return;
    main_bone_ = parse_bone(pt, bone_map_, joint_map_);

    if (main_bone_.get() == nullptr)
        return;

    if (get_tree_from_file(offset_file, pt))
    {
        for (auto &offset : pt)
        {
            joint_map_[offset.first]->offset = offset.second.get_value<float>();
        }
    }

    hip_distance = bone_map_["hip"]->length;
    thigh_length = bone_map_["rthigh"]->length;
    shank_length = bone_map_["rshank"]->length;
    foot_length = bone_map_["rfoot1"]->length;
    arm_length = bone_map_["ruparm"]->length;
    hand_length = bone_map_["rlowarm"]->length;
    trunk_length = bone_map_["torso"]->length;
    neck_length = bone_map_["head1"]->length;
    head_length = bone_map_["camera1"]->length;
    for(auto &j:joint_map_)
    {
        start_id_ = std::min(start_id_, j.second->jid);
    }
}

TransformMatrix Robot::get_foot_mat_from_pose(const RobotPose &pose, bool left)
{
    double sg = (left ? 1.0 : -1.0);
    TransformMatrix foot_mat;
    Quaternion<double> quat;
    AngleAxisd yawRot, pitchRot, rollRot;

    yawRot = AngleAxisd(deg2rad(pose.yaw), Vector3d::UnitZ());
    pitchRot = AngleAxisd(deg2rad(pose.pitch), Vector3d::UnitY());
    rollRot = AngleAxisd(deg2rad(pose.roll), Vector3d::UnitX());
    quat = rollRot * pitchRot * yawRot;
    foot_mat.setP(Vector3d(pose.x, pose.y + sg * hip_distance / 2.0, pose.z));
    foot_mat.setR(quat.matrix());
    return foot_mat;
}

TransformMatrix Robot::get_body_mat_from_pose(const RobotPose &pose)
{
    TransformMatrix body_mat;
    Quaternion<double> quat;
    AngleAxisd yawRot, pitchRot, rollRot;

    yawRot = AngleAxisd(deg2rad(pose.yaw), Vector3d::UnitZ());
    pitchRot = AngleAxisd(deg2rad(pose.pitch), Vector3d::UnitY());
    rollRot = AngleAxisd(deg2rad(pose.roll), Vector3d::UnitX());
    quat = rollRot * pitchRot * yawRot;
    body_mat.setP(Vector3d(pose.x, pose.y, pose.z + leg_length()));
    body_mat.setR(quat.matrix());
    return body_mat;
}

bool Robot::arm_inverse_kinematics(const Vector3d &hand, vector<double> &deg)
{
    double x = hand[0];
    double z = hand[2] - (arm_length + hand_length);
    double l = sqrt(x * x + z * z);

    if (l > arm_length + hand_length)
    {
        return false;
    }

    double q3t = acos((arm_length * arm_length + hand_length * hand_length - l * l) / (2 * arm_length * hand_length));
    double q3 = M_PI - q3t;
    double q1t0 = atan2(z, x);
    double q1t1;

    if (z <= 0)
    {
        q1t1 = -M_PI / 2.0 - q1t0;
    }
    else
    {
        if (x <= 0)
        {
            q1t1 = 3.0 * M_PI / 2.0 - q1t0;
        }
        else
        {
            q1t1 = -(M_PI / 2.0 + q1t0);
        }
    }

    double q1t2 = acos((arm_length * arm_length - hand_length * hand_length + l * l) / (2 * arm_length * l));
    double q1 = (q1t1 + q1t2);
    deg.clear();
    deg.push_back(q1);
    deg.push_back(q3);
    return true;
}

TransformMatrix Robot::leg_forward_kinematics(vector<double> degs, bool left)
{
    double sg = (left ? 1.0 : -1.0);

    if (degs.size() < 6)
    {
        return TransformMatrix();
    }

    TransformMatrix foot;
    TransformMatrix body;
    body = foot * TransformMatrix(0, 0, foot_length) * TransformMatrix(-degs[5], 'x') * TransformMatrix(-degs[4], 'y') * TransformMatrix(0, 0, shank_length) * TransformMatrix(-degs[3], 'y') * TransformMatrix(0, 0, thigh_length) * TransformMatrix(-degs[2], 'y') * TransformMatrix(-degs[1], 'x') * TransformMatrix(-degs[0], 'z') * TransformMatrix(0, -sg * hip_distance / 2.0, bone_map_["rhip2"]->length);
    return body;
}

bool Robot::leg_inverse_kinematics(const TransformMatrix &body,
                                   const TransformMatrix &foot,
                                   vector<double> &deg, bool left)
{
    double sg = (left ? 1.0 : -1.0);
    Vector3d p16 = foot.P() + foot_length * foot.a();
    Vector3d p11 = body.P() + body.R() * Vector3d(0, sg * hip_distance / 2.0, 0);
    Vector3d r = foot.R().transpose() * (p11 - p16);
    double Lr = r.norm();

    if (Lr > thigh_length + shank_length)
    {
        ROS_ERROR("%s: Lr > thigh_length + shank_length  %f %f %f", left ? "left" : "right", Lr, thigh_length, shank_length);
        return false;
    }

    double alpha = acos((thigh_length * thigh_length + shank_length * shank_length - Lr * Lr) / (2 * thigh_length * shank_length));
    double q4 = M_PI - alpha;

    double beta = asin(thigh_length * sin(alpha) / Lr);
    double q5 = -atan2(r[0], sign(r[2]) * sqrt(r[1] * r[1] + r[2] * r[2])) - beta;

    double q6 = atan2(r[1], r[2]);

    if (q6 > M_PI / 2.0)
    {
        q6 = q6 - M_PI;
    }
    else if (q6 < -M_PI / 2.0)
    {
        q6 = q6 + M_PI;
    }

    MatrixX3d R = body.R().transpose() * foot.R() * xRotationMat(-q6) * yRotationMat(-q5) * yRotationMat(-q4);
    double q1 = atan2(-R(0, 1), R(1, 1));
    double q3 = atan2(-R(2, 0), R(2, 2));
    double cz = cos(q1), sz = sin(q1);
    double q2 = atan2(R(2, 1), -R(0, 1) * sz + R(1, 1) * cz);

    deg.clear();
    deg.push_back(q1);
    deg.push_back(q2);
    deg.push_back(q3);
    deg.push_back(q4);
    deg.push_back(q5);
    deg.push_back(q6);
    return true;
}

void Robot::set_degs(const std::map<int, float> &jdmap)
{
    std::lock_guard<std::mutex> lk(robot_mtx_);

    for (auto &j : jdmap)
    {
        get_joint(j.first)->current_deg = j.second;
    }
}

void Robot::set_head(const common::HeadAngles &head)
{
    std::lock_guard<std::mutex> lk(robot_mtx_);
    int sid = get_joint("jhead1")->jid;
    get_joint(sid--)->current_deg = head.yaw;
    get_joint(sid--)->current_deg = head.pitch;
}

void Robot::set_body(const common::BodyAngles &body)
{
    std::lock_guard<std::mutex> lk(robot_mtx_);
    int sid = get_joint("jrshoulder1")->jid;
    get_joint(sid++)->current_deg = body.right_shoulder;
    get_joint(sid++)->current_deg = body.right_elbow;
    get_joint(sid++)->current_deg = body.left_shoulder;
    get_joint(sid++)->current_deg = body.left_elbow;
    get_joint(sid++)->current_deg = body.right_hip_yaw;
    get_joint(sid++)->current_deg = body.right_hip_roll;
    get_joint(sid++)->current_deg = body.right_hip_pitch;
    get_joint(sid++)->current_deg = body.right_knee;
    get_joint(sid++)->current_deg = body.right_ankle_pitch;
    get_joint(sid++)->current_deg = body.right_ankle_roll;
    get_joint(sid++)->current_deg = body.left_hip_yaw;
    get_joint(sid++)->current_deg = body.left_hip_roll;
    get_joint(sid++)->current_deg = body.left_hip_pitch;
    get_joint(sid++)->current_deg = body.left_knee;
    get_joint(sid++)->current_deg = body.left_ankle_pitch;
    get_joint(sid++)->current_deg = body.left_ankle_roll;
}

std::vector<double> Robot::get_foot_degs(int support)
{
    std::lock_guard<std::mutex> lk(robot_mtx_);
    std::vector<double> res;

    if (support == LEFT_SUPPORT)
    {
        res.push_back(joint_map_["jlhip3"]->current_deg);
        res.push_back(joint_map_["jlhip2"]->current_deg);
        res.push_back(joint_map_["jlhip1"]->current_deg);
        res.push_back(joint_map_["jlknee"]->current_deg);
        res.push_back(joint_map_["jlankle2"]->current_deg);
        res.push_back(joint_map_["jlankle1"]->current_deg);
    }
    else
    {
        res.push_back(joint_map_["jrhip3"]->current_deg);
        res.push_back(joint_map_["jrhip2"]->current_deg);
        res.push_back(joint_map_["jrhip1"]->current_deg);
        res.push_back(joint_map_["jrknee"]->current_deg);
        res.push_back(joint_map_["jrankle2"]->current_deg);
        res.push_back(joint_map_["jrankle1"]->current_deg);
    }

    return res;
}

robot::BonePtr parse_bone(const bpt::ptree &pt, robot::BoneMap &BoneMap, robot::JointMap &JointMap)
{
    robot::BonePtr b = std::make_shared<robot::Bone>();
    b->name = pt.get<std::string>("name");
    b->length = pt.get<float>("length");
    bpt::ptree c_pt = pt.get_child("cp");
    std::vector<float> temp;
    temp.clear();

    for (auto &p : c_pt)
    {
        temp.push_back(p.second.get_value<float>());
    }

    b->cp << temp[0], temp[1], temp[2];

    c_pt = pt.get_child("cr");
    temp.clear();

    for (auto &r : c_pt)
    {
        temp.push_back(r.second.get_value<float>());
    }

    b->cr << temp[0], temp[1], temp[2];
    b->joints.clear();
    BoneMap[b->name] = b;

    try
    {
        bpt::ptree cb = pt.get_child("joints");

        for (auto &j : cb)
        {
            b->joints.push_back(parse_joint(j.first, j.second, BoneMap, JointMap));
        }
    }
    catch (bpt::ptree_error &e)
    {
        return b;
    }

    return b;
}

robot::JointPtr parse_joint(const std::string &j_name, const bpt::ptree &pt, robot::BoneMap &BoneMap, robot::JointMap &JointMap)
{
    robot::JointPtr j = std::make_shared<robot::Joint>();
    robot::BonePtr b = std::make_shared<robot::Bone>();
    bpt::ptree cb;
    j->name = j_name;
    j->can_turn = pt.get<bool>("ct");

    bpt::ptree c_pt = pt.get_child("cp");
    std::vector<float> temp;
    temp.clear();

    for (auto &p : c_pt)
    {
        temp.push_back(p.second.get_value<float>());
    }

    j->cp << temp[0], temp[1], temp[2];
    c_pt = pt.get_child("cr");
    temp.clear();

    for (auto &r : c_pt)
    {
        temp.push_back(r.second.get_value<float>());
    }

    j->cr << temp[0], temp[1], temp[2];

    if (j->can_turn)
    {
        j->init_deg = pt.get<float>("init");
        j->jid = pt.get<int>("jid");
        j->current_deg = pt.get<float>("cur");

        try
        {
            j->inverse = (pt.get<bool>("inver") ? -1.0 : 1.0);
        }
        catch (bpt::ptree_error &e)
        {
            j->inverse = 1.0;
        }

        JointMap[j_name] = j;
    }

    try
    {
        j->next_bone = parse_bone(pt.get_child("bone"), BoneMap, JointMap);
    }
    catch (bpt::ptree_error &e)
    {
        throw std::exception();
    }

    return j;
}
} // namespace robot
