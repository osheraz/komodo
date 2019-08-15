#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <komodo2_teleop/armadillo_teleop.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "komodo2_teleop_node");
    ros::NodeHandle nh;
    /* must use multi threaded spinner for moveit */
    ros::AsyncSpinner spinner(2);
    spinner.start();

    Komodo2Teleop armadillo_teleop;
    ros::waitForShutdown();
    return 0;
}

