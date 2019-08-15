
#include "komodo2_sim_interface/komodo2_sim_interface.h"
namespace
{

    double clamp(const double val, const double min_val, const double max_val)
    {
        return std::min(std::max(val, min_val), max_val);
    }

}

namespace gazebo_ros_control
{


    bool Komodo2RobotHWSim::initSim(
            const std::string& robot_namespace,
            ros::NodeHandle model_nh,
            gazebo::physics::ModelPtr parent_model,
            const urdf::Model *const urdf_model,
            std::vector<transmission_interface::TransmissionInfo> transmissions)
    {
        // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
        // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
        const ros::NodeHandle joint_limit_nh(model_nh, robot_namespace);

        // Resize vectors to our DOF
        n_dof_ = transmissions.size();
        joint_names_.resize(n_dof_);
        joint_types_.resize(n_dof_);
        joint_lower_limits_.resize(n_dof_);
        joint_upper_limits_.resize(n_dof_);
        joint_effort_limits_.resize(n_dof_);
        joint_control_methods_.resize(n_dof_);
        pid_controllers_.resize(n_dof_);
        joint_position_.resize(n_dof_);
        joint_velocity_.resize(n_dof_);
        joint_effort_.resize(n_dof_);
        joint_effort_command_.resize(n_dof_);
        joint_position_command_.resize(n_dof_);
        joint_velocity_command_.resize(n_dof_);

        // Initialize values
        for(unsigned int j=0; j < n_dof_; j++)
        {
            // Check that this transmission has one joint
            if(transmissions[j].joints_.size() == 0)
            {
                ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
                                                                             << " has no associated joints.");
                continue;
            }
            else if(transmissions[j].joints_.size() > 1)
            {
                ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
                                                                             << " has more than one joint. Currently the default robot hardware simulation "
                                                                             << " interface only supports one.");
                continue;
            }

            std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
            if (joint_interfaces.empty() &&
                !(transmissions[j].actuators_.empty()) &&
                !(transmissions[j].actuators_[0].hardware_interfaces_.empty()))
            {
                // TODO: Deprecate HW interface specification in actuators in ROS J
                joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
                ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "The <hardware_interface> element of tranmission " <<
                                                                                                                 transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
                                                                                                                 "The transmission will be properly loaded, but please update " <<
                                                                                                                 "your robot model to remain compatible with future versions of the plugin.");
            }
            if (joint_interfaces.empty())
            {
                ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
                                                                       " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
                                                                       "Not adding it to the robot hardware simulation.");
                continue;
            }
            else if (joint_interfaces.size() > 1)
            {
                ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
                                                                       " of transmission " << transmissions[j].name_ << " specifies multiple hardware interfaces. " <<
                                                                       "Currently the default robot hardware simulation interface only supports one. Using the first entry!");
                //continue;
            }

            // Add data from transmission
            joint_names_[j] = transmissions[j].joints_[0].name_;
            joint_position_[j] = 1.0;
            joint_velocity_[j] = 0.0;
            joint_effort_[j] = 1.0;  // N/m for continuous joints
            joint_effort_command_[j] = 0.0;
            joint_position_command_[j] = 0.0;
            joint_velocity_command_[j] = 0.0;

            const std::string& hardware_interface = joint_interfaces.front();

            // Debug
            ROS_DEBUG_STREAM_NAMED("komodo2_hw_sim","Loading joint '" << joint_names_[j]
                                                                            << "' of type '" << hardware_interface << "'");

            // Create joint state interface for all joints
            js_interface_.registerHandle(hardware_interface::JointStateHandle(
                    joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

            // Decide what kind of command interface this actuator/joint has
            hardware_interface::JointHandle joint_handle;
            hardware_interface::PosVelJointHandle jointHandlePosVel;
            if(hardware_interface == "EffortJointInterface" || hardware_interface == "hardware_interface/EffortJointInterface")
            {
                // Create effort joint interface
                joint_control_methods_[j] = EFFORT;
                joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                               &joint_effort_command_[j]);
                ej_interface_.registerHandle(joint_handle);
            }
            else if(hardware_interface == "PositionJointInterface" || hardware_interface == "hardware_interface/PositionJointInterface")            
            {
                // Create position joint interface
                joint_control_methods_[j] = POSITION;
                joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                               &joint_position_command_[j]);
                pj_interface_.registerHandle(joint_handle);
            }
            else if(hardware_interface == "VelocityJointInterface" || hardware_interface == "hardware_interface/VelocityJointInterface")            
            {
                // Create velocity joint interface
                joint_control_methods_[j] = VELOCITY;
                joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                               &joint_velocity_command_[j]);
                vj_interface_.registerHandle(joint_handle);
            }
            else if(hardware_interface == "PosVelJointInterface" || hardware_interface == "hardware_interface/PosVelJointInterface")
            {
                // Create velocity joint interface
                joint_control_methods_[j] = POS_VEL;

                jointHandlePosVel = hardware_interface::PosVelJointHandle(js_interface_.getHandle(joint_names_[j]), &joint_position_command_[j], &joint_velocity_command_[j]);
                pvj_interface_.registerHandle(jointHandlePosVel);
            }
            else
            {
                ROS_FATAL_STREAM_NAMED("default_robot_hw_sim","No matching hardware interface found for '"
                        << hardware_interface );
                return false;
            }

            // Get the gazebo joint that corresponds to the robot joint.
            //ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim", "Getting pointer to gazebo joint: "
            //  << joint_names_[j]);
            gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
            if (!joint)
            {
                ROS_ERROR_STREAM("This robot has a joint named \"" << joint_names_[j]
                                                                   << "\" which is not in the gazebo model.");
                return false;
            }
            sim_joints_.push_back(joint);
            if(!joint_control_methods_[j] != POS_VEL) {
                registerJointLimits(joint_names_[j], joint_handle, joint_control_methods_[j],
                                    joint_limit_nh, urdf_model,
                                    &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
                                    &joint_effort_limits_[j]);
            }
            else {
                registerJointLimits(joint_names_[j], jointHandlePosVel, joint_control_methods_[j],
                                    joint_limit_nh, urdf_model,
                                    &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
                                    &joint_effort_limits_[j]);

            }
//ROS_ERROR("LIMIT on: %s     UP: %f    DOWN: %f",joint_names_[j].c_str(),joint_lower_limits_[j],joint_upper_limits_[j]);

            if (joint_control_methods_[j] != EFFORT)
            {
                // Initialize the PID controller. If no PID gain values are found, use joint->SetAngle() or
                // joint->SetVelocity() to control the joint.
                const ros::NodeHandle nh(model_nh, robot_namespace + "/gazebo_ros_control/pid_gains/" +
                                                   joint_names_[j]);
                if (pid_controllers_[j].init(nh, true))
                {
                    switch (joint_control_methods_[j])
                    {
                        case POSITION:
                            joint_control_methods_[j] = POSITION_PID;
                            break;
                        case VELOCITY:
                            joint_control_methods_[j] = VELOCITY_PID;
                            break;
 			case POS_VEL:
			    joint_control_methods_[j] = POSITION_PID;
                          //  ROS_ERROR("PID on: %s",joint_names_[j].c_str());
                            break;
                    }
                }
                else
                {
                    // joint->SetMaxForce() must be called if joint->SetAngle() or joint->SetVelocity() are
                    // going to be called. joint->SetMaxForce() must *not* be called if joint->SetForce() is
                    // going to be called.
                    #if GAZEBO_MAJOR_VERSION > 2
                                        joint->SetParam("fmax", 0, joint_effort_limits_[j]);
                    #else
                                        joint->SetMaxForce(0, joint_effort_limits_[j]);
                    #endif

                }
            }
        }

        // Register interfaces
        registerInterface(&js_interface_);
        registerInterface(&ej_interface_);
        registerInterface(&pj_interface_);
        registerInterface(&vj_interface_);
        registerInterface(&pvj_interface_);

        // Initialize the emergency stop code.
        e_stop_active_ = false;
        last_e_stop_active_ = false;

        return true;
    }

    void Komodo2RobotHWSim::readSim(ros::Time time, ros::Duration period)
    {
        for(unsigned int j=0; j < n_dof_; j++)
        {
            // Gazebo has an interesting API...
            if (joint_types_[j] == urdf::Joint::PRISMATIC)
            {
                joint_position_[j] = sim_joints_[j]->GetAngle(0).Radian();
            }
            else
            {
                joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
                                                                        sim_joints_[j]->GetAngle(0).Radian());
            }
            joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
            joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
        }
    }

    void Komodo2RobotHWSim::writeSim(ros::Time time, ros::Duration period)
    {
        // If the E-stop is active, joints controlled by position commands will maintain their positions.
        if (e_stop_active_)
        {
            if (!last_e_stop_active_)
            {
                last_joint_position_command_ = joint_position_;
                last_e_stop_active_ = true;
            }
            joint_position_command_ = last_joint_position_command_;
        }
        else
        {
            last_e_stop_active_ = false;
        }

        ej_sat_interface_.enforceLimits(period);
        ej_limits_interface_.enforceLimits(period);
        pj_sat_interface_.enforceLimits(period);
        pj_limits_interface_.enforceLimits(period);
        vj_sat_interface_.enforceLimits(period);
        vj_limits_interface_.enforceLimits(period);

        for(unsigned int j=0; j < n_dof_; j++)
        {
            switch (joint_control_methods_[j])
            {

                case POS_VEL:

//sim_joints_[j]->SetVelocityLimit(0,joint_velocity_command_[j]);
#if GAZEBO_MAJOR_VERSION >= 4
                    sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
#else
                    sim_joints_[j]->SetAngle(0, joint_position_command_[j]);
#endif
                    sim_joints_[j]->SetVelocity(0, e_stop_active_ ? 0 : joint_velocity_command_[j]);

                    break;

                case EFFORT:
                {
                    const double effort = e_stop_active_ ? 0 : joint_effort_command_[j];
                    sim_joints_[j]->SetForce(0, effort);
                }
                    break;

                case POSITION:
#if GAZEBO_MAJOR_VERSION >= 4
                    sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
#else
                    sim_joints_[j]->SetAngle(0, joint_position_command_[j]);
#endif
                    break;

                case POSITION_PID:
                {
                    double error;
                    switch (joint_types_[j])
                    {
                        case urdf::Joint::REVOLUTE:
                            angles::shortest_angular_distance_with_limits(joint_position_[j],
                                                                          joint_position_command_[j],
                                                                          joint_lower_limits_[j],
                                                                          joint_upper_limits_[j],
                                                                          error);
                            break;
                        case urdf::Joint::CONTINUOUS:
                            error = angles::shortest_angular_distance(joint_position_[j],
                                                                      joint_position_command_[j]);
                            break;
                        default:
                            error = joint_position_command_[j] - joint_position_[j];
                    }

                    const double effort_limit = joint_effort_limits_[j];
                    const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
                                                -effort_limit, effort_limit);
                    sim_joints_[j]->SetForce(0, effort);
                }
                    break;

                case VELOCITY:
                    sim_joints_[j]->SetVelocity(0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
                    break;

                case VELOCITY_PID:
                    double error;
                    if (e_stop_active_)
                        error = -joint_velocity_[j];
                    else
                        error = joint_velocity_command_[j] - joint_velocity_[j];
                    const double effort_limit = joint_effort_limits_[j];
                    const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
                                                -effort_limit, effort_limit);
                    sim_joints_[j]->SetForce(0, effort);
                    break;

            }
        }
    }

    void Komodo2RobotHWSim::eStopActive(const bool active)
    {
        e_stop_active_ = active;
    }

// Register the limits of the joint specified by joint_name and joint_handle. The limits are
// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
    void Komodo2RobotHWSim::registerJointLimits(const std::string& joint_name,
                                                   const hardware_interface::JointHandle& joint_handle,
                                                   const ControlMethod ctrl_method,
                                                   const ros::NodeHandle& joint_limit_nh,
                                                   const urdf::Model *const urdf_model,
                                                   int *const joint_type, double *const lower_limit,
                                                   double *const upper_limit, double *const effort_limit)
    {
        *joint_type = urdf::Joint::UNKNOWN;
        *lower_limit = -std::numeric_limits<double>::max();
        *upper_limit = std::numeric_limits<double>::max();
        *effort_limit = std::numeric_limits<double>::max();

        joint_limits_interface::JointLimits limits;
        bool has_limits = false;
        joint_limits_interface::SoftJointLimits soft_limits;
        bool has_soft_limits = false;

        if (urdf_model != NULL)
        {
            const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
            if (urdf_joint != NULL)
            {
                *joint_type = urdf_joint->type;
                // Get limits from the URDF file.
                if (joint_limits_interface::getJointLimits(urdf_joint, limits))
                    has_limits = true;
                if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
                    has_soft_limits = true;
            }
        }
        // Get limits from the parameter server.
        if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
            has_limits = true;

        if (!has_limits)
            return;

        if (*joint_type == urdf::Joint::UNKNOWN)
        {
            // Infer the joint type.

            if (limits.has_position_limits)
            {
                *joint_type = urdf::Joint::REVOLUTE;
            }
            else
            {
                if (limits.angle_wraparound)
                    *joint_type = urdf::Joint::CONTINUOUS;
                else
                    *joint_type = urdf::Joint::PRISMATIC;
            }
        }

        if (limits.has_position_limits)
        {
            *lower_limit = limits.min_position;
            *upper_limit = limits.max_position;
        }
        if (limits.has_effort_limits)
            *effort_limit = limits.max_effort;

        if (has_soft_limits)
        {
            switch (ctrl_method)
            {
                case EFFORT:
                {
                    const joint_limits_interface::EffortJointSoftLimitsHandle
                            limits_handle(joint_handle, limits, soft_limits);
                    ej_limits_interface_.registerHandle(limits_handle);
                }
                    break;
                case POSITION:
                {
                    const joint_limits_interface::PositionJointSoftLimitsHandle
                            limits_handle(joint_handle, limits, soft_limits);
                    pj_limits_interface_.registerHandle(limits_handle);
                }
                    break;
                case VELOCITY:
                {
                    const joint_limits_interface::VelocityJointSoftLimitsHandle
                            limits_handle(joint_handle, limits, soft_limits);
                    vj_limits_interface_.registerHandle(limits_handle);
                }
                    break;
            }
        }
        else
        {
            switch (ctrl_method)
            {
                case EFFORT:
                {
                    const joint_limits_interface::EffortJointSaturationHandle
                            sat_handle(joint_handle, limits);
                    ej_sat_interface_.registerHandle(sat_handle);
                }
                    break;
                case POSITION:
                {
                    const joint_limits_interface::PositionJointSaturationHandle
                            sat_handle(joint_handle, limits);
                    pj_sat_interface_.registerHandle(sat_handle);
                }
                    break;
                case VELOCITY:
                {
                    const joint_limits_interface::VelocityJointSaturationHandle
                            sat_handle(joint_handle, limits);
                    vj_sat_interface_.registerHandle(sat_handle);
                }
                    break;
            }
        }
    }


    void Komodo2RobotHWSim::registerJointLimits(const std::string& joint_name,
                                                   const hardware_interface::PosVelJointHandle& joint_handle,
                                                   const ControlMethod ctrl_method,
                                                   const ros::NodeHandle& joint_limit_nh,
                                                   const urdf::Model *const urdf_model,
                                                   int *const joint_type, double *const lower_limit,
                                                   double *const upper_limit, double *const effort_limit)
    {
        *joint_type = urdf::Joint::UNKNOWN;
        *lower_limit = -std::numeric_limits<double>::max();
        *upper_limit = std::numeric_limits<double>::max();
        *effort_limit = std::numeric_limits<double>::max();

        joint_limits_interface::JointLimits limits;
        bool has_limits = false;
        joint_limits_interface::SoftJointLimits soft_limits;
        bool has_soft_limits = false;

        if (urdf_model != NULL)
        {
            const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
            if (urdf_joint != NULL)
            {
                *joint_type = urdf_joint->type;
                // Get limits from the URDF file.
                if (joint_limits_interface::getJointLimits(urdf_joint, limits))
                    has_limits = true;
                if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
                    has_soft_limits = true;
            }
        }
        // Get limits from the parameter server.
        if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
            has_limits = true;

        if (!has_limits)
            return;

        if (*joint_type == urdf::Joint::UNKNOWN)
        {
            // Infer the joint type.

            if (limits.has_position_limits)
            {
                *joint_type = urdf::Joint::REVOLUTE;
            }
            else
            {
                if (limits.angle_wraparound)
                    *joint_type = urdf::Joint::CONTINUOUS;
                else
                    *joint_type = urdf::Joint::PRISMATIC;
            }
        }

        if (limits.has_position_limits)
        {
            *lower_limit = limits.min_position;
            *upper_limit = limits.max_position;
        }
        if (limits.has_effort_limits)
            *effort_limit = limits.max_effort;

        /*  if (has_soft_limits)
          {
              switch (ctrl_method)
              {
                  case POS_VEL:
                  {
                      const joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(joint_handle., limits, soft_limits);
                      pj_limits_interface_.registerHandle(limits_handle);
                  }
                      break;
              };
          }
          else
          {
              switch (ctrl_method)
              {
                  case EFFORT:
                  {
                      const joint_limits_interface::EffortJointSaturationHandle
                              sat_handle(joint_handle, limits);
                      ej_sat_interface_.registerHandle(sat_handle);
                  }
                      break;
                  case POSITION:
                  {
                      const joint_limits_interface::PositionJointSaturationHandle
                              sat_handle(joint_handle, limits);
                      pj_sat_interface_.registerHandle(sat_handle);
                  }
                      break;
                  case VELOCITY:
                  {
                      const joint_limits_interface::VelocityJointSaturationHandle
                              sat_handle(joint_handle, limits);
                      vj_sat_interface_.registerHandle(sat_handle);
                  }
                      break;
              }
          }*/
    }
    PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::Komodo2RobotHWSim, gazebo_ros_control::RobotHWSim)
}


