

#ifndef ROBOTICAN_CONTROLLERS_HARDWARE_INTERFACE_ADAPTER_H
#define ROBOTICAN_CONTROLLERS_HARDWARE_INTERFACE_ADAPTER_H


#include <cassert>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>

template <class HardwareInterface, class State>
class HardwareInterfaceAdapter
{
public:
    bool init(std::vector<typename HardwareInterface::ResourceHandleType>& /*joint_handles*/, ros::NodeHandle& /*controller_nh*/)
    {
        return false;
    }

    void starting(const ros::Time& /*time*/) {}
    void stopping(const ros::Time& /*time*/) {}

    void updateCommand(const ros::Time&     /*time*/,
                       const ros::Duration& /*period*/,
                       const State&         /*desired_state*/,
                       const State&         /*state_error*/) {}
};

template <class State>
class HardwareInterfaceAdapter<hardware_interface::PosVelJointInterface, State> {
private:
    std::vector<hardware_interface::PosVelJointHandle>* joint_handles_ptr_;
public:
    HardwareInterfaceAdapter() : joint_handles_ptr_(NULL) {}
    bool init(std::vector<hardware_interface::PosVelJointHandle>& joint_handles, ros::NodeHandle& controller_nh){
        joint_handles_ptr_ = &joint_handles;
    }

    void starting(const ros::Time& /*time*/) {
        if (!joint_handles_ptr_) {return;}
        for (unsigned int i = 0; i < joint_handles_ptr_->size(); ++i)
        {
            (*joint_handles_ptr_)[i].setCommand((*joint_handles_ptr_)[i].getPosition(), 0.05);
        }
    }

    void stopping(const ros::Time& /*time*/) { }

    void updateCommand(const ros::Time&     /*time*/,
                       const ros::Duration& /*period*/,
                       const State&         desired_state,
                       const State&         /*state_error*/)
    {
        // Forward desired position to command
        const unsigned int n_joints = joint_handles_ptr_->size();
        for (unsigned int i = 0; i < n_joints; ++i) {(*joint_handles_ptr_)[i].setCommand(desired_state.position[i], desired_state.velocity[i]);}
    }


};

#endif //ROBOTICAN_CONTROLLERS_HARDWARE_INTERFACE_ADAPTER_H
