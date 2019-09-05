#! /usr/bin/env python
from __future__ import print_function
#from builtins import range

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist, Vector3Stamped
import numpy as np
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from scipy.interpolate import interp1d
motor_con = 4



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
        des_cmd[:2] = np.interp(des_cmd[:2], (-0.2 , 0.32), (250, 950))
        des_cmd[2:] = np.interp(des_cmd[2:], (-0.5, 0.548), (10, 450))
        return des_cmd



class KomodoEnvironment:
    def __init__(self):

        rospy.init_node('RL_Node')
        Hz = 50
        rate = rospy.Rate(Hz)

        # TODO: Robot information
        self.bucket_init_pos = 0
        self.arm_init_pos = 0
        self.vel_init = 0
        self.HALF_KOMODO = 0.53 / 2
        self.particle = 0
        self.x_tip = 0
        self.z_tip = 0
        self.bucket_link_x = 0
        self.bucket_link_z = 0
        self.velocity = 0
        self.wheel_vel = 0
        self.position_from_pile = np.array([0])
        self.joint_name_lst = ['arm_joint', 'bucket_joint', 'front_left_wheel_joint', 'front_right_wheel_joint',
                               'rear_left_wheel_joint', 'rear_right_wheel_joint']
        self.last_pos = np.zeros(3)
        self.last_ori = np.zeros(4)
        self.max_limit = np.array([0.1, 0.32, 0.548])
        self.min_limit = np.array([-0.1, -0.2, -0.5])
        self.orientation = np.zeros(4)
        self.angular_vel = np.zeros(3)
        self.linear_acc = np.zeros(3)

        self.arm_data = np.array([0, 0, 0, 0], dtype=np.float)
        self.fb = np.zeros((motor_con,), dtype=np.int32)
        self.old_fb = np.zeros((motor_con,), dtype=np.int32)
        self.velocity_motor = np.zeros((motor_con,), dtype=np.int32)

        # TODO: RL information
        self.nb_actions = 3  # base , arm , bucket
        self.state_shape = (self.nb_actions * 2 + 5,)
        self.action_shape = (self.nb_actions,)
        self.actions = Actions()
        self.starting_pos = np.array([self.vel_init,self.arm_init_pos, self.bucket_init_pos])
        self.action_range = self.max_limit - self.min_limit
        self.action_mid = (self.max_limit + self.min_limit) / 2.0
        self.last_action = np.zeros(self.nb_actions)
        self.joint_state = np.zeros(self.nb_actions)
        self.joint_pos = self.starting_pos
        self.state = np.zeros(self.state_shape)


        # TODO: Robot information Subscribers
        self.joint_state_subscriber = rospy.Subscriber('/arm/pot_fb', Int32MultiArray, self.update_fb)
        self.velocity_subscriber = rospy.Subscriber('/mobile_base_controller/odom',Odometry,self.velocity_subscriber_callback)
        self.imu_subscriber = rospy.Subscriber('/IMU',Imu,self.imu_subscriber_callback)
        self.distance_subscriber = rospy.Subscriber('/scan', LaserScan, self.update_distace)
        self.arm_data_subscriber = rospy.Subscriber('/arm/data', Float32MultiArray, self.update_arm_data)





    def update_fb(self, data):
        """
        :param data:
        :type data:
        :return:       range      0 - 1023
        :rtype:
        """
        dt = 0.02
        self.old_fb = np.array(self.fb)
        self.fb = np.array(data.data)
        self.velocity_motor = (self.fb - self.old_fb) / dt  # SensorValue per second
        self.joint_state[1] = np.interp(self.fb[0],(250, 950),(-0.2, 0.32))
        self.joint_state[2] = np.interp(self.fb[2],(10, 450),(-0.5, 0.548))

    def update_distace(self,msg):
        """
        :param data:
        :type data:
        :return:       middle value of laser sensor
        :rtype:
        """
        self.position_from_pile = np.array([msg.ranges[360]])

    def update_arm_data(self,data):
        self.arm_data = data.data


    def normalize_joint_state(self,joint_pos):
        joint_coef = 3.0
        return joint_pos * joint_coef

    def imu_subscriber_callback(self,imu):
        self.orientation = np.array([imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w])
        self.angular_vel = np.array([imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z])
        self.linear_acc = np.array([imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z])

    def velocity_subscriber_callback(self, data):
        vel = data.twist.twist.linear.x
        self.joint_state[0] = vel # fixed velocity
        self.velocity = vel

    def check_particle_in_bucket(self):
        return 'wow'

    def dump_pile(self):
        return 'dumped'

    def reset(self):
        self.joint_pos = self.starting_pos
        self.actions.reset_move(self.starting_pos)
        rospy.sleep(0.5)
        self.state = np.zeros(self.state_shape)
        self.last_joint = self.joint_state
        diff_joint = np.zeros(self.nb_actions)
        normed_js = self.normalize_joint_state(self.joint_state)
        self.particle = 0

        self.state = np.concatenate((self.arm_data, self.position_from_pile, normed_js, diff_joint)).reshape(1, -1)
        # self.state = np.concatenate((self.particle,self.arm_data, self.position_from_pile, normed_js, diff_joint)).reshape(1, -1)

        self.last_action = np.zeros(self.nb_actions)

        return self.state

    def step(self, action):


        print('action:',action)
        action = action * self.action_range
        self.joint_pos = np.clip(self.joint_pos + action,a_min=self.min_limit,a_max=self.max_limit)
        self.actions.move(self.joint_pos)
        # print('joint pos:',self.joint_pos)

        rospy.sleep(15.0/60.0)
        self.particle = self.check_particle_in_bucket() # update particle in bucket

        #normed_js = self.normalize_joint_state(self.joint_pos)
        normed_js = self.normalize_joint_state(self.joint_state)

        diff_joint = normed_js - self.last_joint

        self.state = np.concatenate((self.arm_data, self.position_from_pile, normed_js, diff_joint)).reshape(1, -1)
        # self.state = np.concatenate((self.particle, self.arm_data, self.position_from_pile, normed_js, diff_joint)).reshape(1, -1)

        self.last_joint = normed_js
        self.last_action = action

        print('state', self.state)
        return self.state