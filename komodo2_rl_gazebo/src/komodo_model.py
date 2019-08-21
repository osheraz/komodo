#! /usr/bin/env python
from __future__ import print_function
#from builtins import range

import rospy
import time
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelConfiguration, SetModelConfigurationRequest
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetLinkState, GetLinkStateRequest
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3Stamped
import numpy as np
from std_msgs.msg import Int32MultiArray
from sklearn.preprocessing import MinMaxScaler


class Actions:
    def __init__(self):
        self.des_to_pub = Int32MultiArray()
        self.des_cmd = np.array([350, 350, 120, 120], dtype=np.int32)
        self.arm_bucket_pub = rospy.Publisher('/arm/des_cmd', Int32MultiArray, queue_size=10)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()
        # assumption we are moving just in x-axis
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

    def move(self, cmd): # cmd [velocity , arm , bucket ]
        self.vel_msg.linear.x = cmd[0]
        arm_cmd = cmd[1]
        bucket_cmd = cmd[2]
        self.des_to_pub.data = self.normalize_arm_cmd(arm_cmd, bucket_cmd)
        self.arm_bucket_pub.publish(self.des_to_pub)
        self.vel_pub.publish(self.vel_msg)

    def reset_move(self, cmd):
        self.vel_msg.linear.x = cmd[0]
        arm_cmd = cmd[1]
        bucket_cmd = cmd[2]
        self.des_to_pub.data = self.normalize_arm_cmd(arm_cmd, bucket_cmd)
        self.arm_bucket_pub.publish(self.des_to_pub)
        self.vel_pub.publish(self.vel_msg)

    def normalize_arm_cmd(self,arm_cmd , bucket_cmd):
        des_cmd = np.array([arm_cmd, arm_cmd, bucket_cmd, bucket_cmd])
        des_cmd[:2] = np.interp(des_cmd[:2],(250, 950))
        des_cmd[2:] = np.interp(des_cmd[:2],(30, 450))
        return des_cmd



class KomodoEnvironment:
    def __init__(self):

        rospy.init_node('RL_Node')

        self.bucket_init_pos = 0
        self.arm_init_pos = 0
        self.vel_init = 0
        self.nb_actions = 3  # base , arm , bucket
        self.nb_links = 3  # base , arm , bucket

        self.state_shape = (
        self.nb_actions * 2 + 8 + 4 + 3 + 3,)  # joint states + diff + orientation + lin acc + w acc + arm_state(3) + model data(2)
        self.action_shape = (self.nb_actions,)
        self.flag = True
        self.particle = 0
        self.x_tip = 0
        self.z_tip = 0
        self.bucket_link_x = 0
        self.bucket_link_z = 0
        self.velocity = 0
        self.wheel_vel = 0

        self.joint_name_lst = ['arm_joint', 'bucket_joint', 'front_left_wheel_joint', 'front_right_wheel_joint',
                               'rear_left_wheel_joint', 'rear_right_wheel_joint']

        self.actions = Actions()
        self.starting_pos = np.array([self.vel_init, self.arm_init_pos, self.bucket_init_pos])

        self.last_pos = np.zeros(3)
        self.last_ori = np.zeros(4)

        self.max_limit = np.array([0.1, 0.32, 0.548])
        self.min_limit = np.array([-0.1, -0.2, -0.5])

        self.action_range = self.max_limit - self.min_limit
        self.action_mid = (self.max_limit + self.min_limit) / 2.0
        self.joint_pos = self.starting_pos

        self.joint_state = np.zeros(self.nb_actions)

        self.joint_state_subscriber = rospy.Subscriber('/joint_states',JointState,self.joint_state_subscriber_callback)
        self.velocity_subscriber = rospy.Subscriber('/GPS/fix_velocity',Vector3Stamped,self.velocity_subscriber_callback)
        self.imu_subscriber = rospy.Subscriber('/IMU',Imu,self.imu_subscriber_callback)

        self.orientation = np.zeros(4)
        self.angular_vel = np.zeros(3)
        self.linear_acc = np.zeros(3)

        self.joint_coef = 3.0 # 3.0

        self.normed_sp = self.normalize_joint_state(self.starting_pos)
        self.state = np.zeros(self.state_shape)
        self.diff_state_coeff = 3.0
        self.action_coeff = 1.0
        self.linear_acc_coeff = 0.1
        self.last_action = np.zeros(self.nb_actions)

    def normalize_joint_state(self,joint_pos):
        return joint_pos * self.joint_coef

    def imu_subscriber_callback(self,imu):
        self.orientation = np.array([imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w])
        self.angular_vel = np.array([imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z])
        self.linear_acc = np.array([imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z])

    def velocity_subscriber_callback(self, data):
        vel = -1*data.vector.x
        self.joint_state[0] = vel # fixed velocity
        self.velocity = vel

    def joint_state_subscriber_callback(self,joint_state):
        self.joint_state[1]= joint_state.position[0] # arm
        self.joint_state[2] = joint_state.position[1] # bucket
        self.wheel_vel = joint_state.velocity[2]

    def reset(self):
        self.joint_pos = self.starting_pos
        self.actions.reset_move(self.starting_pos)
        rospy.sleep(0.5)
        self.state = np.zeros(self.state_shape)
        self.last_joint = self.joint_state
        diff_joint = np.zeros(self.nb_actions)
        normed_js = self.normalize_joint_state(self.joint_state)

        model_state = self.get_model_state_proxy(self.get_model_state_req)
        pos = np.array([model_state.pose.position.x, model_state.pose.position.y, model_state.pose.position.z])

        self.state = np.concatenate((normed_js,diff_joint,self.orientation,self.angular_vel,self.linear_acc_coeff*self.linear_acc)).reshape(1,-1)
        self.last_action = np.zeros(self.nb_actions)

        return self.state

    def step(self, action):
        print('action:',action)
        action = action * self.action_range * self.action_coeff
        self.joint_pos = np.clip(self.joint_pos + action,a_min=self.min_limit,a_max=self.max_limit)
        self.actions.move_jtp(self.joint_pos)
        print('joint pos:',self.joint_pos)

        rospy.sleep(15.0/60.0)

        #normed_js = self.normalize_joint_state(self.joint_state)
        normed_js = self.normalize_joint_state(self.joint_pos * self.joint_pos_to_state_factor)

        diff_joint = self.diff_state_coeff * (normed_js - self.last_joint)

        self.state = np.concatenate((normed_js,diff_joint,self.orientation,self.angular_vel,self.linear_acc_coeff*self.linear_acc)).reshape(1,-1)

        self.last_joint = normed_js
        self.last_action = action

        print('state',self.state)
        return self.state