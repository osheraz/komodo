#ifndef KOMODO2_HW_ROBOTEQ_DIFF_DRIVE_H
#define KOMODO2_HW_ROBOTEQ_DIFF_DRIVE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <roboteq/roboteq.h>
#include <roboteq/serial_controller.h>
#include <std_msgs/String.h>

#define ROBOTEQ_PORT_PARAM "~roboteq_port"
#define ROBOTEQ_BAUD_PARAM "~roboteq_baud"
#define RIGHT_REAR_WHEEL_JOINT_PARAM "~right_rear_wheel_joint"
#define LEFT_REAR_WHEEL_JOINT_PARAM "~left_rear_wheel_joint"
#define RIGHT_FRONT_WHEEL_JOINT_PARAM "~right_front_wheel_joint"
#define LEFT_FRONT_WHEEL_JOINT_PARAM "~left_front_wheel_joint"

typedef boost::chrono::steady_clock time_source;

class RoboteqDiffDrive
{
private:

    ros::NodeHandle *nh_;
    roboteq::serial_controller *roboteq_serial_;
    roboteq::Roboteq *roboteq_;
    time_source::time_point last_time_ = time_source::now();

    std::string roboteq_port_;
    std::string right_rear_wheel_joint_,
            left_rear_wheel_joint_,
            right_front_wheel_joint_,
            left_front_wheel_joint_;
    int roboteq_baud_ = 0;
    bool load_roboteq_hw_ = false;
    /* if first time, subtract previous values */
    bool first_time_ = true;
    ros::Publisher espeak_pub_;

    void speakMsg(std::string msg, int sleep_time)
    {
        std_msgs::String speak_msg;
        speak_msg.data = msg;
        espeak_pub_.publish(speak_msg);
        if (sleep_time > 0)
            sleep(sleep_time);
    }


public:
    ~RoboteqDiffDrive() { delete roboteq_; }
    RoboteqDiffDrive(ros::NodeHandle &nh);
    void read(const ros::Duration elapsed);
    void write(const ros::Duration elapsed);
    void registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                         hardware_interface::VelocityJointInterface &velocity_joint_interface);
};


#endif //KOMODO2_HW_ROBOTEQ_DIFF_DRIVE_H
