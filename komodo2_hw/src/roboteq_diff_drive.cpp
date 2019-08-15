#include "roboteq_diff_drive.h"


RoboteqDiffDrive::RoboteqDiffDrive(ros::NodeHandle &nh)
{
    nh_ = &nh;

    /* get roboteq params */
    ros::param::get("~load_roboteq_hw", load_roboteq_hw_);

    if (load_roboteq_hw_)
    {
        if (!ros::param::get(ROBOTEQ_PORT_PARAM, roboteq_port_))
        {
            ROS_ERROR(
                    "[komodo2_hw/roboteq_diff_drive]: %s param is missing on param server. make sure that this param exist in ricboard_config.yaml "
                            "and that your launch includes this param file. shutting down...", ROBOTEQ_PORT_PARAM);
            ros::shutdown();
            exit(EXIT_FAILURE);
        }

        if (!ros::param::get(ROBOTEQ_BAUD_PARAM, roboteq_baud_))
        {
            ROS_ERROR(
                    "[komodo2_hw/roboteq_diff_drive]: %s param is missing on param server. make sure that this param exist in ricboard_config.yaml "
                            "and that your launch includes this param file. shutting down...", ROBOTEQ_BAUD_PARAM);
            ros::shutdown();
            exit(EXIT_FAILURE);
        }

        if (!ros::param::get(RIGHT_REAR_WHEEL_JOINT_PARAM, right_rear_wheel_joint_))
        {
            ROS_ERROR(
                    "[komodo2_hw/roboteq_diff_drive]: %s param is missing on param server. make sure that this param exist in ricboard_config.yaml "
                            "and that your launch includes this param file. shutting down...", RIGHT_REAR_WHEEL_JOINT_PARAM);
            ros::shutdown();
            exit(EXIT_FAILURE);
        }

        if (!ros::param::get(LEFT_REAR_WHEEL_JOINT_PARAM, left_rear_wheel_joint_))
        {
            ROS_ERROR(
                    "[komodo2_hw/roboteq_diff_drive]: %s param is missing on param server. make sure that this param exist in ricboard_config.yaml "
                            "and that your launch includes this param file. shutting down...", LEFT_REAR_WHEEL_JOINT_PARAM);
            ros::shutdown();
            exit(EXIT_FAILURE);
        }

        if (!ros::param::get(RIGHT_FRONT_WHEEL_JOINT_PARAM, right_front_wheel_joint_))
        {
            ROS_ERROR(
                    "[komodo2_hw/roboteq_diff_drive]: %s param is missing on param server. make sure that this param exist in ricboard_config.yaml "
                            "and that your launch includes this param file. shutting down...", RIGHT_REAR_WHEEL_JOINT_PARAM);
            ros::shutdown();
            exit(EXIT_FAILURE);
        }

        if (!ros::param::get(LEFT_FRONT_WHEEL_JOINT_PARAM, left_front_wheel_joint_))
        {
            ROS_ERROR(
                    "[komodo2_hw/roboteq_diff_drive]: %s param is missing on param server. make sure that this param exist in ricboard_config.yaml "
                            "and that your launch includes this param file. shutting down...", LEFT_REAR_WHEEL_JOINT_PARAM);
            ros::shutdown();
            exit(EXIT_FAILURE);
        }

        /* try connect to roboteq */
        roboteq_serial_ = new roboteq::serial_controller(roboteq_port_, roboteq_baud_);
        /* run the serial controller */
        bool start = roboteq_serial_->start();
        if (!start)
        {
            ROS_ERROR(
                    "[komodo2_hw/roboteq_diff_drive]: failed opening roboteq port. make sure roboteq is connected. shutting down...");
            ros::shutdown();
            exit(1);
        }
        ROS_INFO("[komodo2_hw/roboteq_diff_drive]: roboteq port opened successfully \nport name: %s \nbaudrate: %d",
                 roboteq_port_.c_str(), roboteq_baud_);
        /* initialize roboteq controller */
        roboteq_ = new roboteq::Roboteq(nh, nh, roboteq_serial_);

        /* initialize the motor parameters */
        roboteq_->initialize();

        ROS_INFO("[komodo2_hw/roboteq_diff_drive]: roboteq is up");
        espeak_pub_ = nh.advertise<std_msgs::String>("/espeak_node/speak_line", 10);
        /* speakMsg("robotek is up", 1); */
    }
    else
        ROS_WARN("[komodo2_hw/roboteq_diff_drive]: roboteq hardware is disabled");

}

void RoboteqDiffDrive::write(const ros::Duration elapsed)
{
    if (!load_roboteq_hw_)
        return;
    roboteq_->write(ros::Time::now(), elapsed);
}

void RoboteqDiffDrive::read(const ros::Duration elapsed)
{
    if (!load_roboteq_hw_)
        return;
    roboteq_->read(ros::Time::now(), elapsed);
}

void RoboteqDiffDrive::registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                                       hardware_interface::VelocityJointInterface &velocity_joint_interface)
{
    if (!load_roboteq_hw_)
        return;
    /* initialize all interfaces and setup diagnostic messages */
    roboteq_->initializeInterfaces(joint_state_interface, velocity_joint_interface);
}
