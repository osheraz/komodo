

#ifndef ARMADILLO2_TELEOP_ARMADILLO_JOY_H
#define ARMADILLO2_TELEOP_ARMADILLO_JOY_H

#include <ros/forwards.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "joy_profile.h"

struct joints_state_indx
{
    static const uint8_t TORSO = 11;
    static const uint8_t PAN = 0;
    static const uint8_t TILT = 1;
};

class Komodo2Teleop
{
private:
    //ros::NodeHandle *nh_;
    ros::Publisher torso_real_pub_,
                   torso_sim_pub_,
                   twist_pub_,
                   head_pub_;
    ros::Subscriber joy_sub_,
                    joints_states_sub_,
                    gripper_sub_;
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface *arm_grp_;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> *gripper_client_;

    joints_state_indx states_;
    joy_profile joy_;
    bool tele_arm_ = false;

    void drive();
    void moveTorso();
    void moveArm();
    void moveGripper();
    void moveHead();
    void resetArm();
    void update(const sensor_msgs::Joy::ConstPtr& joy);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    bool loadProfile(const std::string &profile_name);
    void jointsUpdateCB(const sensor_msgs::JointState::ConstPtr& msg);
    void gripperGapCB(const std_msgs::Float32::ConstPtr& msg);

public:
    Komodo2Teleop();
    ~Komodo2Teleop()
    {
        delete gripper_client_;
        delete arm_grp_;
    }
};

#endif //ARMADILLO2_TELEOP_ARMADILLO_JOY_H
