
#ifndef ARMADILLO2_TELEOP_JOY_PROFILE_H
#define ARMADILLO2_TELEOP_JOY_PROFILE_H

struct joy_puppet
{
    bool init_state_recorded = false;
};
struct joy_twist : joy_puppet
{
    float axis_val_linear = 0;
    float axis_val_angular = 0;

    float scale_angular = 0;
    float scale_linear = 0;

    int joy_axis_linear = 0;
    int joy_axis_angular = 0;
};

struct joy_torso : joy_puppet
{
    float axis_val_updown = 0;
    float increment = 0; //meters

    int joy_axis_updown = 0;

    float limit_upper = 0;
    float limit_lower = 0;
};

struct joy_arm : joy_puppet
{
    std::vector<double> axes_vals;
    std::vector<double> axes_vals_prev;

    static const uint8_t INDX_ROTATION1 = 0;
    static const uint8_t INDX_SHOULDER1 = 1;
    static const uint8_t INDX_SHOULDER2 = 2;
    static const uint8_t INDX_ROTATION2 = 3;
    static const uint8_t INDX_SHOULDER3 = 4;
    static const uint8_t INDX_WRIST = 5;
    static const uint8_t DOF = 6;

    int joy_axis_rotation1 = 0;
    int joy_axis_shoulder1 = 0;
    int joy_axis_shoulder2 = 0;
    int joy_axis_rotation2 = 0;
    int joy_btn_shoulder3_up = 0;
    int joy_btn_shoulder3_down = 0;
    int joy_btn_wrist_cw = 0;
    int joy_btn_wrist_ccw = 0;
    int joy_btn_reset = 0;
    float increment = 0;

    std::string start_pos = "ninety_deg";

    joy_arm()
    {
        axes_vals.reserve(6);
        axes_vals_prev.reserve(6);
    }
};

struct joy_gripper : joy_puppet
{
    int joy_axis = 0;
    control_msgs::GripperCommandGoal goal;
    float increment = 0;
    float limit_upper = 0;
    float limit_lower = 0;
};

struct joy_pan_tilt : joy_puppet
{
    float axis_val_pan = 0;
    float axis_val_tilt = 0;

    float inc_pan = 0;
    float inc_tilt = 0;

    int joy_btn_pan_right = 0;
    int joy_btn_pan_left = 0;
    int joy_btn_tilt_up = 0;
    int joy_btn_tilt_down = 0;

    float limit_upper_pan = 0;
    float limit_lower_pan = 0;
    float limit_upper_tilt = 0;
    float limit_lower_tilt = 0;
};

struct joy_utils
{
    int joy_btn_arm_mode = 0;
    int joy_btn_safety = 0;
};

struct joy_profile
{
    joy_torso torso;
    joy_twist twist;
    joy_arm arm;
    joy_gripper gripper;
    joy_pan_tilt head;
    joy_utils utils;
};


#endif //ARMADILLO2_TELEOP_JOY_PROFILE_H
