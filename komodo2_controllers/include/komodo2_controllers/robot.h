
#ifndef ROBOTICAN_HARDWARE_ROBOT_H
#define ROBOTICAN_HARDWARE_ROBOT_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <komodo2_controllers/utils.h>

struct JointInfo_t {
    double position;
    double effort;
    double velocity;
    double cmd;
    JointInfo_t() {
        position = effort = velocity = cmd = 0;
    }
};

class RobotArm : public hardware_interface::RobotHW {
public:
    RobotArm();
    ros::Time getTime();
    ros::Duration getPeriod();
    friend std::ostream& operator <<(std::ostream &output, RobotArm& robotArm) {
        size_t size = robotArm._arm.size();
        for(std::map<std::string, JointInfo_t>::iterator it = robotArm._arm.begin(); it != robotArm._arm.end(); ++it)
            output << " Name: " << it->first << " CMD: " << it->second.cmd << std::endl;
        return output;
    }

private:
    hardware_interface::JointStateInterface _jointStateInterface;
    hardware_interface::PositionJointInterface _positionJointInterface;
    ros::NodeHandle _nodeHandle;
    std::map<std::string, JointInfo_t> _arm;
    ros::Time _time;

};

#endif //ROBOTICAN_HARDWARE_ROBOT_H
