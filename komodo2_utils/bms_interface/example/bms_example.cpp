//
// Created by sub on 23/10/17.
//

#include <ros/ros.h>
#include <bms_interface/bms_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ip_estimator_node");
    ros::NodeHandle nh;

    bms::BMSInterface bms;
    try
    {
        bms.connect("/dev/armadillo2/BMS");
    }
    catch (bms::BMSException exp)
    {
        ROS_ERROR("[bms_test]: %s", exp.what());
        //ros::shutdown();
    }

    ros::Rate loop_rate(2);


    while (ros::ok())
    {

        try
        {
            bms.read();
        }
        catch(bms::BMSErrorException exp)
        {
            ROS_ERROR("[bms_test]: %s", exp.what());
            ros::shutdown();
            exit(EXIT_FAILURE);
        }
        catch(bms::BMSWarnException exp)
        {
            ROS_WARN("[bms_test]: %s", exp.what());
        }


        loop_rate.sleep();
        ros::spinOnce();
    }

}