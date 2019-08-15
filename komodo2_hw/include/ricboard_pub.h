
#ifndef ARMADILLO2_HW_RICBOARD_PUB_H
#define ARMADILLO2_HW_RICBOARD_PUB_H

#include <ric_interface/ric_interface.h>
#include <ric_interface/ric_exception.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <boost/thread/thread.hpp>
#include <boost/chrono/chrono.hpp>
#include <control_msgs/JointControllerState.h>


#define RIC_PORT_PARAM "~ric_port"
#define RIC_PUB_INTERVAL 0.1 //secs
#define RIC_WRITE_INTERVAL 0.05 //secs
#define RIC_DEAD_TIMEOUT 1 //secs
#define MAX_RIC_DISCONNECTIONS 5
#define G_FORCE 9.80665
#define HSV_URF_MIN_RANGE 0.3
#define HSV_URF_MAX_RANGE 3.0
#define HSV_URF_FOV 0.7f

class RicboardPub
{
private:

    bool  load_ric_hw_ = true;

    int ric_disconnections_counter_ = 0;

    std::string ric_port_;

    ros::Publisher ric_gps_pub_;

    ros::Publisher rear_urf_pub_,
            right_urf_pub_,
            left_urf_pub_;
    ros::Publisher ric_imu_pub_;
    ros::Publisher ric_mag_pub_;

    ros::Timer ric_pub_timer_,
               ric_dead_timer_;

    ric::RicInterface ric_;
    ros::NodeHandle *nh_;
    boost::thread* t;

    ros::Publisher espeak_pub_;

    /* handles */
    std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
    std::vector<hardware_interface::JointHandle> pos_handles_;

    void pubTimerCB(const ros::TimerEvent& event);
    void ricDeadTimerCB(const ros::TimerEvent& event);
    void loop();
    void speakMsg(std::string msg, int sleep_time)
    {
        std_msgs::String speak_msg;
        speak_msg.data = msg;
        espeak_pub_.publish(speak_msg);
        if (sleep_time > 0)
            sleep(sleep_time);
    }


public:
    RicboardPub(ros::NodeHandle &nh);
    void startLoop();
    void stopLoop();
    void registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                         hardware_interface::EffortJointInterface &position_interface);
};


#endif //ARMADILLO2_HW_RICBOARD_PUB_H
