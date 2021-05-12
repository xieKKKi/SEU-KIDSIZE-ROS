#ifndef __DATADEF_HPP
#define __DATADEF_HPP

#include <common/PlayerInfo.h>
#include <eigen3/Eigen/Dense>

struct CameraParams
{
    float fx, fy;
    float cx, cy;
    float k1, k2;
    float p1, p2;
};

struct DetObject
{
    int id;
    float prob;
    int x, y, w, h;
    DetObject(int i = 0, float p = 1, int x_ = 0, int y_ = 0, 
                int w_ = 0, int h_ = 0)
        : id(i), prob(p), x(x_), y(y_), w(w_), h(h_) {}
    bool operator< (const DetObject &obj)
    {
        return prob < obj.prob;
    }
};

struct ObjectFilter
{
    float min_prob;
    Eigen::Vector2f r_range;
    Eigen::Vector2i w_range;
    Eigen::Vector2i h_range;
};

#define TC_COMM_PORT 6868
#define TC_STRUCT_HEADER "SEU"
struct TeamCommData
{
    char header[3] = {'S', 'E', 'U'};
    common::PlayerInfo player;
};

enum RobocupGameState
{
    GC_INITIAL, // define STATE_INITIAL 0
    GC_READY, // define STATE_READY 1
    GC_SET, // define STATE_SET 2; !!ATTENTION_PLEASE: DROPBALL will use this pattern
    GC_PLAY, // define STATE_PLAYING 3
    GC_FINISH // define STATE_FINISH 4
};

enum RobocupGameSecondaryState
{
    GC_NORMAL, // define STATE2_NORMAL 0
    GC_PENALTYSHOOT, // define STATE2_PENALTYSHOOT 1
    GC_OVERTIME, // define STATE2_OVERTIME 2
    GC_TIMEOUT, // define STATE2_TIMEOUT 3
    GC_DIRECT_FREEKICK,
    GC_INDIRECT_FREEKICK,
    GC_PENALTYKICK,
};

enum RobocupPlayerState
{
    P_PENALTY_NONE = 0,
    P_PENALTY_PUSHING = 2,
    P_SUBSTITUTE = 15,
    P_PENALTY_BALL_MANIPULATION = 30, // define PENALTY_HL_KID_BALL_MANIPULATION 1
    P_PENALTY_PHYSICAL_CONTACT, // define PENALTY_HL_KID_PHYSICAL_CONTACT 2
    P_PENALTY_ILLEGAL_ATTACK, // define PENALTY_HL_KID_ILLEGAL_ATTACK 3
    P_PENALTY_ILLEGAL_DEFENSE, // define PENALTY_HL_KID_ILLEGAL_DEFENSE 4
    P_PENALTY_PICKUP_OR_INCAPABLE, // defien PENALTY_HL_KID_REQUEST_FOR_PICKUP 5
    P_PENALTY_SERVICE, // define PENALTY_HL_KID_REQUEST_FOR_SERVICE 6
};

enum FreeKickStage
{
    FK_ADAJUST = 0,
    FK_EXECUTE
};

enum FsmState {
    FSM_STATE_READY,        // 0
    FSM_STATE_GETUP,        // 1
    FSM_STATE_SEARCH_BALL,  // 2
    FSM_STATE_GOTO_BALL,    // 3
    FSM_STATE_KICK_BALL,    // 4
    FSM_STATE_DRIBBLE,      // 5
    FSM_STATE_SL            // 6
};

struct FieldInfo {
    int field_length;
    int field_width;
    int goal_depth;
    int goal_width;
    int goal_height;
    int goal_area_length;
    int goal_area_width;
    int penalty_mark_distance;
    int center_circle_diameter;
    int border_strip_width_min;

    float scale=1.0;

    void scale_field(float s=1.0)
    {
        scale *= s;
        field_length *= s;
        field_width *= s;
        goal_depth *= s;
        goal_width *= s;
        goal_height *= s;;
        goal_area_length *= s;
        goal_area_width *= s;
        penalty_mark_distance *= s;
        center_circle_diameter *= s;
        border_strip_width_min *= s;
    }
};

#endif
