#ifndef KOMODO2_HW_BATTERY_PUB_H
#define KOMODO2_HW_BATTERY_PUB_H

#include <ros/ros.h>
#include <bms_interface/bms_interface.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>

#define BATT_PUB_INTERVAL 2 //secs
#define SPEAK_LOW_BAT_INTERVAL 30 //secs
#define BATT_PORT_PARAM "~batt_port"

namespace komodo2_hw
{
    class BatteryPub
    {
    private:
        ros::Publisher bat_pub_;
        ros::Timer bat_pub_timer_;
        ros::Timer speak_low_batt_timer_;
        bms::BMSInterface bms_;

        std::string batt_port_;
        int low_batt_val_ = -1;
        bool show_warnings_ = false;
        bool load_battery_hw_ = true;

        ros::Publisher espeak_pub_;

        void pubBatTimerCB(const ros::TimerEvent &event);

        void speakLowTimerCB(const ros::TimerEvent& event);

        void speakMsg(std::string msg, int sleep_time)
        {
            std_msgs::String speak_msg;
            speak_msg.data = msg;
            espeak_pub_.publish(speak_msg);
            if (sleep_time > 0)
                sleep(sleep_time);
        }

    public:
        BatteryPub(ros::NodeHandle nh);
    };
}



#endif //KOMODO2_HW_BATTERY_PUB_H
