
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "komodo2_hw.h"

#define LOOP_HZ 100.0
#define THREADS_NUM 2

int main(int argc, char **argv)
{
    ros::init(argc, argv, "komodo2_hw_node");
    ros::NodeHandle nh;

    komodo2_hw::Komodo2HW komodo(nh);
    controller_manager::ControllerManager controller_manager(&komodo);

    ros::AsyncSpinner asyncSpinner(THREADS_NUM);
    asyncSpinner.start();

    ros::Time last_time = ros::Time::now();

    while (ros::ok())
    {
        komodo.read();

        ros::Duration(1.0 / (2.0 * LOOP_HZ)).sleep();

        ros::Duration duration = ros::Time::now() - last_time;
        controller_manager.update(ros::Time::now(), duration);
        last_time = ros::Time::now();

        komodo.write();

        ros::Duration(1.0 / (2.0 * LOOP_HZ)).sleep();

        ros::spinOnce;
    }
}