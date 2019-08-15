
#include <ros/ros.h>
#include <robotican_controllers/robot.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv) {
    ros::init(argc, argv,"komodo2_controllers");
    RobotArm robotArm;
    controller_manager::ControllerManager controllerManager(&robotArm);
    ros::Rate rate(50);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    while (ros::ok()) {
        controllerManager.update(robotArm.getTime(), robotArm.getPeriod());
        rate.sleep();
    }
    return 0;
}
