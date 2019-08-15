

#include "komodo2_controllers/utils.h"


void rosUtil::rosInfo(const char *info) {
    ROS_INFO("[%s]: %s", ros::this_node::getName().c_str(), info);
}

void rosUtil::rosError(const char *err) {
    ROS_ERROR("[%s]: %s", ros::this_node::getName().c_str(), err);
}


void rosUtil::rosWarn(const char *warn) {
    ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), warn);
}

template <typename T>
void rosUtil::rosInfoStream(T &obj) {
    ROS_INFO_STREAM("["<< ros::this_node::getName()  << "]: " << obj);
}