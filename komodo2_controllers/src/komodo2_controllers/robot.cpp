

#include "komodo2_controllers/robot.h"

RobotArm::RobotArm() {
    std::vector<std::string> names;
    if(_nodeHandle.getParam("joint_names_list", names)) {
        size_t size = names.size();
        for(int i = 0; i < size; ++i) {
            _arm.insert(std::pair<std::string, JointInfo_t>(names[i], JointInfo_t()));
            hardware_interface::JointStateHandle jointStateHandle(names[i], &_arm[names[i]].position, &_arm[names[i]].velocity, &_arm[names[i]].effort);
            _jointStateInterface.registerHandle(jointStateHandle);
            hardware_interface::JointHandle jointHandle(_jointStateInterface.getHandle(names[i]), &_arm[names[i]].cmd);
            _positionJointInterface.registerHandle(jointHandle);


        }
        registerInterface(&_jointStateInterface);
        registerInterface(&_positionJointInterface);
        rosUtil::rosInfo("Active");



    }
    else {
        rosUtil::rosError("The parameter 'joint_names' is not loaded");
        ros::shutdown();
    }
}

ros::Time RobotArm::getTime() {
    return ros::Time::now();
}

ros::Duration RobotArm::getPeriod() {
    ros::Time now = ros::Time::now();
    ros::Duration period = now - _time;
    _time = now;
    return period;
}

