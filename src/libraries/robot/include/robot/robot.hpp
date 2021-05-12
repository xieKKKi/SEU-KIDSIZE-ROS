#ifndef __SEUROBOT_HPP
#define __SEUROBOT_HPP

#include <common/BodyAngles.h>
#include <common/HeadAngles.h>
#include <geometry_msgs/Transform.h>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <seumath/math.hpp>
#include <string>
#include <vector>

namespace robot {
class Bone;
class Joint;

typedef std::shared_ptr<Joint> JointPtr;
typedef std::shared_ptr<Bone> BonePtr;
typedef std::map<std::string, JointPtr> JointMap;
typedef std::map<std::string, BonePtr> BoneMap;

class Bone {
public:
  std::string name;
  float length;
  Eigen::Vector3f cp;
  Eigen::Vector3f cr;
  std::list<JointPtr> joints;
};

class Joint {
public:
  int jid;
  std::string name;
  bool can_turn;
  BonePtr next_bone;
  float init_deg;
  float inverse;
  float offset;
  Eigen::Vector3f cp;
  Eigen::Vector3f cr;
  float current_deg;
};

enum RobotMotion {
  MOTION_NONE = 0,
  MOTION_RIGHT_HAND = 1,
  MOTION_LEFT_HAND = 2,
  MOTION_BODY = 3,
  MOTION_RIGHT_FOOT = 4,
  MOTION_LEFT_FOOT = 5,
  MOTION_HEAD = 6
};

enum SupportFoot { DOUBLE_SUPPORT = 0, LEFT_SUPPORT = 1, RIGHT_SUPPORT = 2 };

const std::map<std::string, RobotMotion> namemotion_map = {
    {"none", MOTION_NONE},
    {"right_hand", MOTION_RIGHT_HAND},
    {"left_hand", MOTION_LEFT_HAND},
    {"body", MOTION_BODY},
    {"right_foot", MOTION_RIGHT_FOOT},
    {"left_foot", MOTION_LEFT_FOOT},
    {"head", MOTION_HEAD}};

inline std::string get_name_by_motion(const RobotMotion &motion) {
  for (auto &nm : namemotion_map) {
    if (nm.second == motion) {
      return nm.first;
    }
  }

  return "";
}

inline RobotMotion get_motion_by_name(const std::string &name) {
  for (auto &nm : namemotion_map) {
    if (nm.first == name) {
      return nm.second;
    }
  }

  return MOTION_NONE;
}

struct RobotPose {
  float x, y, z;
  float pitch, roll, yaw;
};

struct RobotPos {
  std::string name;
  std::map<RobotMotion, RobotPose> pose_info;
};

struct RobotOnePos {
  std::string pos_name;
  int act_time;
};

struct RobotAct {
  std::string name;
  std::vector<RobotOnePos> poses;
};

struct RobotJointDeg {
  int id;
  float deg;
};

struct JointInfo {
  int id;
  float inverse;
  float offset;
};

struct JointsInfo {
  int head_start;
  int head_count;
  int body_start;
  int body_count;
  std::vector<JointInfo> infos;
};

typedef std::map<std::string, RobotAct> ActMap;
typedef std::map<std::string, RobotPos> PosMap;
typedef std::map<RobotMotion, RobotPose> PoseMap;

bool parse(std::string act_file, ActMap &acts, PosMap &poses);
void save(const std::string &act_file, const ActMap &acts, const PosMap &poses);

inline void PoseToTrans(const RobotPose &pose,
                        geometry_msgs::Transform &trans) {
  trans.translation.x = pose.x;
  trans.translation.y = pose.y;
  trans.translation.z = pose.z;
  trans.rotation.x = pose.roll;
  trans.rotation.y = pose.pitch;
  trans.rotation.z = pose.yaw;
}

class Robot {
public:
  Robot(const std::string &robot_file, const std::string &offset_file);

  seumath::TransformMatrix get_foot_mat_from_pose(const RobotPose &pose,
                                                  bool left);
  seumath::TransformMatrix get_body_mat_from_pose(const RobotPose &pose);

  seumath::TransformMatrix leg_forward_kinematics(std::vector<double> degs,
                                                  bool left);
  bool arm_inverse_kinematics(const Eigen::Vector3d &hand,
                              std::vector<double> &deg);
  bool leg_inverse_kinematics(const seumath::TransformMatrix &body,
                              const seumath::TransformMatrix &foot,
                              std::vector<double> &deg, bool left = false);

  void set_degs(const std::map<int, float> &jdmap);
  void set_body(const common::BodyAngles &body);
  void set_head(const common::HeadAngles &head);

  std::vector<double> get_foot_degs(int support);
  std::vector<double> get_head_degs() {
    std::vector<double> res = {joint_map_["jhead1"]->current_deg,
                               joint_map_["jhead2"]->current_deg};
    return res;
  }

  JointPtr get_joint(const int &id) {
    for (auto &j : joint_map_) {
      if (j.second->jid == id) {
        return j.second;
      }
    }

    return JointPtr();
  }

  JointPtr get_joint(const std::string &name) {
    for (auto &j : joint_map_) {
      if (j.second->name == name) {
        return j.second;
      }
    }

    return JointPtr();
  }

  double leg_length() const {
    return thigh_length + shank_length + foot_length;
  }
  double leg_length_without_foot() const { return thigh_length + shank_length; }

  JointMap &get_joint_map() {
    std::lock_guard<std::mutex> lk(robot_mtx_);
    return joint_map_;
  }

  BoneMap &get_bone_map() { return bone_map_; }

  BonePtr get_main_bone() { return main_bone_; }
  BonePtr get_bone(std::string name) {
    auto it = bone_map_.find(name);
    if (it == bone_map_.end())
      return BonePtr();
    return it->second;
  }

  int joint_count() { return joint_map_.size(); }
  int joint_start_id() {
    return start_id_;
  }

public:
  double hip_distance, thigh_length, shank_length, foot_length;
  double arm_length, hand_length;
  double trunk_length, neck_length, head_length;

private:
  mutable std::mutex robot_mtx_;
  BonePtr main_bone_;
  JointMap joint_map_;
  BoneMap bone_map_;
  int start_id_;
};
} // namespace robot

#endif
