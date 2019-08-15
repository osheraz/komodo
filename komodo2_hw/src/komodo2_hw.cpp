
#include "komodo2_hw.h"

namespace komodo2_hw
{

    Komodo2HW::Komodo2HW(ros::NodeHandle &nh) :
            battery_(nh), ric_(nh), roboteq_(nh)
    {
        node_handle_ = &nh;

        /* register handles */
        roboteq_.registerHandles(joint_state_interface_,
                                 velocity_interface_);

        /* register interfaces */
        registerInterface(&joint_state_interface_);
        registerInterface(&posvel_interface_);
        registerInterface(&position_interface_);
        registerInterface(&velocity_interface_);
        registerInterface(&effort_interface_);

        prev_time_ = ros::Time::now();

        ric_.startLoop();

        ROS_INFO("[komodo2_hw]: komodo hardware interface loaded successfully");
        espeak_pub_ = node_handle_->advertise<std_msgs::String>("/espeak_node/speak_line", 10);
        speakMsg("i am ready", 1);
    }

    void Komodo2HW::read()
    {
        ros::Duration period = ros::Time::now() - prev_time_;
        roboteq_.read(period);
    }

    void Komodo2HW::write()
    {
        ros::Duration period = ros::Time::now() - prev_time_;
        roboteq_.write(period);
        prev_time_ = ros::Time::now();
    }
}