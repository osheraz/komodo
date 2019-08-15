
#ifndef KOMODO2_HW_KOMODO_HW_H
#define KOMODO2_HW_KOMODO_HW_H

#include "battery_pub.h"
#include "ricboard_pub.h"
#include "roboteq_diff_drive.h"
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <std_msgs/String.h>


namespace komodo2_hw
{
    class Komodo2HW : public hardware_interface::RobotHW
    {
    private:

        ros::Time prev_time_;
        ros::NodeHandle *node_handle_;

        /* interfaces */
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_interface_;
        hardware_interface::PosVelJointInterface posvel_interface_;
        hardware_interface::VelocityJointInterface velocity_interface_;
        hardware_interface::EffortJointInterface effort_interface_;

        /* robot close loop components */
        BatteryPub battery_;
        RicboardPub ric_;
        RoboteqDiffDrive roboteq_;

        ros::Publisher espeak_pub_;

        void registerInterfaces();
        void straighHead();
        void speakMsg(std::string msg, int sleep_time)
        {
            std_msgs::String speak_msg;
            speak_msg.data = msg;
            espeak_pub_.publish(speak_msg);
            if (sleep_time > 0)
                sleep(sleep_time);
        }

    public:

        Komodo2HW(ros::NodeHandle &nh);
        void read();
        void write();
    };
}

#endif //KOMODO2_HW_KOMODO_HW_H
