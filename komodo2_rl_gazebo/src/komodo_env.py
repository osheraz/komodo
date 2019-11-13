#! /usr/bin/env python
# coding=utf-8

import rospy
import time
import numpy as np
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelConfiguration, SetModelConfigurationRequest
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetLinkState, GetLinkStateRequest
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3Stamped
from Spawner import Spawner
from matplotlib import path
import math
import pandas


class Actions:
    def __init__(self):
        self.arm_pos_pub = rospy.Publisher('/arm_position_controller/command', Float64, queue_size=10)
        self.bucket_pos_pub = rospy.Publisher('/bucket_position_controller/command', Float64, queue_size=10)
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
        self.arm_pos_pub.publish(arm_cmd)
        self.bucket_pos_pub.publish(bucket_cmd)
        self.vel_pub.publish(self.vel_msg)

    def reset_move(self, cmd):
        self.vel_msg.linear.x = cmd[0]
        self.arm_pos_pub.publish(cmd[1])
        self.bucket_pos_pub.publish(cmd[2])
        self.vel_pub.publish(self.vel_msg)


class Pile:

    def __init__(self):

        self.length = 1
        self.width = 1
        self.height = 1
        self.size = 0.1
        self.radius = 0.035
        self.num_particle = 0
        self.z_max = 0.26
        self.x_min = 0
        self.sand_box_x = 0.35
        self.sand_box_y = 0.301
        self.sand_box_z = 0.0
        self.sand_box_height = 0.25
        self.center_x = self.sand_box_x / 2
        self.center_z = self.sand_box_z / 2
        self.HALF_KOMODO = 0.53 / 2
        self.spawner = Spawner()
        self.spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
        self.get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)


        # Spawn Box
        box_req = self.spawner.create_box_request('sand_box', self.sand_box_x, self.sand_box_y, self.sand_box_z,0.0, 0.0, 0.0)
        self.spawn_srv(box_req)

        self.pile_box_req = SetModelStateRequest()
        self.pile_box_req.model_state = ModelState()
        self.pile_box_req.model_state.model_name = 'sand_box'
        self.pile_box_req.model_state.pose.position.x = self.sand_box_x
        self.pile_box_req.model_state.pose.position.y = self.sand_box_y
        self.pile_box_req.model_state.pose.position.z = self.sand_box_z
        self.pile_box_req.model_state.pose.orientation.x = 0.0
        self.pile_box_req.model_state.pose.orientation.y = 0.0
        self.pile_box_req.model_state.pose.orientation.z = 0.0
        self.pile_box_req.model_state.pose.orientation.w = 0.0
        self.pile_box_req.model_state.twist.linear.x = 0.0
        self.pile_box_req.model_state.twist.linear.y = 0.0
        self.pile_box_req.model_state.twist.linear.z = 0.0
        self.pile_box_req.model_state.twist.angular.x = 0.0
        self.pile_box_req.model_state.twist.angular.y = 0.0
        self.pile_box_req.model_state.twist.angular.z = 0.0
        self.pile_box_req.model_state.reference_frame = 'world'

    def create_pile(self):

        count = 0
        l = int(self.length/self.size)
        w = int(self.width/self.size)
        h = int(self.height/self.size)
        for k in range(h):
            #w = w - 1
            l = l - 1
            for j in range(-w/2 + 1, w/2):
                for i in range(0,l):
                    count +=1
                    name = "particle" + str(count)
                    pos = [i*self.size*0.25 , j*self.size*0.25 , self.radius*(1+2*k) ]
                    rot = [0.0, 0.0, 0.0]
                    # req = self.spawner.create_cube_request(name, pos[0], pos[1], pos[2],
                    #                                              rot[0], rot[1], rot[2],
                    #                                              self.size, self.size, self.size)
                    req = self.spawner.create_sphere_request(name, pos[0], pos[1], pos[2],
                                                                 rot[0], rot[1], rot[2],
                                                                 self.radius)
                    self.spawn_srv(req)

        self.num_particle = count

    def set_pile(self):
        count = 0
        l = int(self.length/self.size)
        w = int(self.width/self.size)
        h = int(self.height/self.size)
        self.model_state_proxy(self.pile_box_req)

        for k in range(h):
            #w = w - 1
            l = l - 1
            for j in range(-w/2 + 1, w/2):
                for i in range(0,l):
                    count +=1
                    self.pile_state_req = SetModelStateRequest()
                    self.pile_state_req.model_state = ModelState()
                    self.pile_state_req.model_state.model_name = 'particle'+str(count)
                    self.pile_state_req.model_state.pose.position.x = i*self.size*0.25
                    self.pile_state_req.model_state.pose.position.y = j*self.size*0.25
                    self.pile_state_req.model_state.pose.position.z = self.radius*(1+2*k)
                    self.pile_state_req.model_state.pose.orientation.x = 0.0
                    self.pile_state_req.model_state.pose.orientation.y = 0.0
                    self.pile_state_req.model_state.pose.orientation.z = 0.0
                    self.pile_state_req.model_state.pose.orientation.w = 0.0
                    self.pile_state_req.model_state.twist.linear.x = 0.0
                    self.pile_state_req.model_state.twist.linear.y = 0.0
                    self.pile_state_req.model_state.twist.linear.z = 0.0
                    self.pile_state_req.model_state.twist.angular.x = 0.0
                    self.pile_state_req.model_state.twist.angular.y = 0.0
                    self.pile_state_req.model_state.twist.angular.z = 0.0
                    self.pile_state_req.model_state.reference_frame = 'world'
                    self.model_state_proxy(self.pile_state_req)

    def particle_location(self,num_p):
        px_arr = np.zeros(num_p)
        py_arr = np.zeros(num_p)
        pz_arr = np.zeros(num_p)
        for i in range(1, num_p+1):
            get_particle_state_req = GetModelStateRequest()
            get_particle_state_req.model_name = 'particle'+str(i)
            get_particle_state_req.relative_entity_name = 'base_footprint' # 'world'
            particle_state = self.get_model_state_proxy(get_particle_state_req)
            x = abs(particle_state.pose.position.x) + self.HALF_KOMODO
            y = particle_state.pose.position.y
            z = particle_state.pose.position.z
            orientation = particle_state.pose.orientation
            (roll, pitch, theta) = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w])
            px_arr[i-1] = x
            py_arr[i-1] = y
            pz_arr[i-1] = z
        return px_arr, pz_arr, py_arr

    def in_bucket_2d(self,xq, yq, xv, yv):
        shape = xq.shape
        xq = xq.reshape(-1)
        yq = yq.reshape(-1)
        xv = xv.reshape(-1)
        yv = yv.reshape(-1)
        q = [(xq[i], yq[i]) for i in range(xq.shape[0])]
        p = path.Path([(xv[i], yv[i]) for i in range(xv.shape[0])])
        return p.contains_points(q).reshape(shape)

class KomodoEnvironment:
    def __init__(self):

        rospy.init_node('RL_Node')
        
        # TODO:  Pile information
        self.pile = Pile() # (1.75, 2.8, 1.05, 0.34)
        self.pile.length = 1.75
        self.pile.width = 2.8
        self.pile.height = 1.05
        self.pile.size = 0.34
        self.pile_flag = True

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
        self.joint_name_lst = ['arm_joint', 'bucket_joint', 'front_left_wheel_joint', 'front_right_wheel_joint',
                               'rear_left_wheel_joint', 'rear_right_wheel_joint']
        self.last_pos = np.zeros(3)
        self.last_ori = np.zeros(4)
        self.max_limit = np.array([0.1, 0.32, 0.548])
        self.min_limit = np.array([-0.1, -0.2, -0.5])
        self.orientation = np.zeros(4)
        self.angular_vel = np.zeros(3)
        self.linear_acc = np.zeros(3)

        # TODO: RL information
        self.nb_actions = 3  # base , arm , bucket
        self.state_shape = (self.nb_actions * 2 +6,)
        self.action_shape = (self.nb_actions,)
        self.actions = Actions()
        self.starting_pos = np.array([self.vel_init,self.arm_init_pos, self.bucket_init_pos])
        self.action_range = self.max_limit - self.min_limit
        self.action_mid = (self.max_limit + self.min_limit) / 2.0
        self.last_action = np.zeros(self.nb_actions)
        self.joint_state = np.zeros(self.nb_actions)
        self.joint_pos = self.starting_pos
        self.state = np.zeros(self.state_shape)
        self.reward = 0.0
        self.done = False
        self.episode_start_time = 0.0
        self.max_sim_time = 8.0

        # TODO: Robot information Subscribers
        self.joint_state_subscriber = rospy.Subscriber('/joint_states',JointState,self.joint_state_subscriber_callback)
        self.velocity_subscriber = rospy.Subscriber('/mobile_base_controller/odom',Odometry,self.velocity_subscriber_callback)
        self.imu_subscriber = rospy.Subscriber('/IMU',Imu,self.imu_subscriber_callback)

        # TODO: Gazebo stuff
        self.pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics',Empty)
        self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world',Empty)

        self.model_config_proxy = rospy.ServiceProxy('/gazebo/set_model_configuration',SetModelConfiguration)
        self.model_config_req = SetModelConfigurationRequest()
        self.model_config_req.model_name = 'komodo2'
        self.model_config_req.urdf_param_name = 'robot_description'
        self.model_config_req.joint_names = self.joint_name_lst
        self.model_config_req.joint_positions = self.starting_pos

        self.model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
        self.model_state_req = SetModelStateRequest()
        self.model_state_req.model_state = ModelState()
        self.model_state_req.model_state.model_name = 'komodo2'
        self.model_state_req.model_state.pose.position.x = 1.0
        self.model_state_req.model_state.pose.position.y = 0.0
        self.model_state_req.model_state.pose.position.z = 0.0
        self.model_state_req.model_state.pose.orientation.x = 0.0
        self.model_state_req.model_state.pose.orientation.y = 0.0
        self.model_state_req.model_state.pose.orientation.z = 0.0
        self.model_state_req.model_state.pose.orientation.w = 0.0
        self.model_state_req.model_state.twist.linear.x = 0.0
        self.model_state_req.model_state.twist.linear.y = 0.0
        self.model_state_req.model_state.twist.linear.z = 0.0
        self.model_state_req.model_state.twist.angular.x = 0.0
        self.model_state_req.model_state.twist.angular.y = 0.0
        self.model_state_req.model_state.twist.angular.z = 0.0
        self.model_state_req.model_state.reference_frame = 'world'

        self.get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        self.get_model_state_req = GetModelStateRequest()
        self.get_model_state_req.model_name = 'komodo2'
        self.get_model_state_req.relative_entity_name = 'world'

        self.get_link_state_proxy = rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
        self.get_link_state_req = GetLinkStateRequest()
        self.get_link_state_req.link_name = 'bucket'
        self.get_link_state_req.reference_frame = 'world'

    def normalize_joint_state(self,joint_pos):
        joint_coef = 3.0 # 3.0
        return joint_pos * joint_coef

    def velocity_subscriber_callback(self, data):
        vel = data.twist.twist.linear.x
        self.joint_state[0] = vel # fixed velocity
        self.velocity = vel

    def joint_state_subscriber_callback(self,joint_state):
        self.joint_state[1]= joint_state.position[0] # arm
        self.joint_state[2] = joint_state.position[1] # bucket
        self.wheel_vel = joint_state.velocity[2]

    def imu_subscriber_callback(self,imu):
        self.orientation = np.array([imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w])
        self.angular_vel = np.array([imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z])
        self.linear_acc = np.array([imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z])

    def reset(self):
        #pause physics
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause_proxy()
        except rospy.ServiceException as e:
            print('/gazebo/pause_physics service call failed')

        #set models pos from world
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            self.model_state_proxy(self.model_state_req)
        except rospy.ServiceException as e:
            print('/gazebo/set_model_state call failed')

        #set model's joint config
        rospy.wait_for_service('/gazebo/set_model_configuration')
        try:
            self.model_config_proxy(self.model_config_req)
        except rospy.ServiceException as e:
            print('/gazebo/set_model_configuration call failed')

        self.joint_pos = self.starting_pos
        self.actions.reset_move(self.starting_pos)

        #spawner
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            if self.pile_flag:
                self.pile.create_pile()
                self.pile_flag = False
            else:
                self.pile.set_pile()
        except rospy.ServiceException as e:
            print('/gazebo/unpause_physics service call failed')

        #unpause physics
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause_proxy()
        except rospy.ServiceException as e:
            print('/gazebo/unpause_physics service call failed')

        rospy.sleep(0.5)
        self.reward = 0.0 # Init reward
        self.state = np.zeros(self.state_shape)

        rospy.wait_for_service('/gazebo/get_model_state')
        model_state = self.get_model_state_proxy(self.get_model_state_req)
        pos = np.array([model_state.pose.position.x, model_state.pose.position.y, model_state.pose.position.z])
        done = False

        rospy.wait_for_service('/gazebo/get_link_state')

        self.last_joint = self.joint_state
        self.last_pos = pos

        diff_joint = np.zeros(self.nb_actions)
        normed_js = self.normalize_joint_state(self.joint_state)

        arm_data = np.array([self.particle, self.x_tip, self.z_tip, self.bucket_link_x, self.bucket_link_z])

        model_data = np.array([pos[0]])

        self.state = np.concatenate((arm_data, model_data, normed_js, diff_joint)).reshape(1, -1)

        self.episode_start_time = rospy.get_time()
        self.last_action = np.zeros(self.nb_actions)
        return self.state, done

    def check_particle_in_bucket(self):

        dp = 0.215  # bucket geometry
        d1 = 0.124
        d2 = 0.1
        self.get_link_state_req.reference_frame = 'base_footprint'
        bucket_state = self.get_link_state_proxy(self.get_link_state_req)

        # self.get_link_state_req.reference_frame = 'world'
        # bucket_state = self.get_link_state_proxy(self.get_link_state_req)

        x = bucket_state.link_state.pose.position.x

        self.bucket_link_x = abs(x) + self.HALF_KOMODO
        y = bucket_state.link_state.pose.position.y
        z = bucket_state.link_state.pose.position.z
        self.bucket_link_z = z
        orientation = bucket_state.link_state.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

        self.x_tip = self.bucket_link_x + dp*np.cos(roll)
        self.z_tip = z + dp*np.sin(roll)
        x_up = self.bucket_link_x + d1*np.cos(roll + math.radians(46))
        z_up = z + d1*np.sin(roll + math.radians(46))
        x_down = self.bucket_link_x + d2*np.cos(math.radians(41) - roll)
        z_down = z - d2*np.sin(math.radians(41) - roll)

        xv = np.array([self.bucket_link_x, x_up, self.x_tip, x_down, self.bucket_link_x])
        zv = np.array([z, z_up, self.z_tip, z_down, z])

        xq, zq, yq = self.pile.particle_location(self.pile.num_particle)
        index = np.where(abs(yq) >= 0.2)
        # xq = np.delete(xq, index)
        # zq = np.delete(zq, index)

        particle_in_bucket = self.pile.in_bucket_2d(xq, zq, xv, zv)
        self.particle = (particle_in_bucket == 1).sum()

        rospy.logdebug('BUCKET: x: '+str(round(x, 2)) +' y: '+str(round(y, 3))+' z: '+str(round(z, 2))+ ' x_tip: '+str(round(self.x_tip, 2)))
        rospy.logdebug('BUCKET: roll: '+str(round(roll,2)) +' pitch: '+str(round(pitch, 3))+' theta: '+str(round(theta, 2)))
        rospy.logdebug('xv: '+str(np.round(xv, 2)) + ' zv: ' + str(np.round(zv, 2)))
        rospy.logdebug('xq: '+str(np.round(xq, 2)) + ' zq: ' + str(np.round(zq, 2)))
        rospy.logdebug('Particle in bucket: ' + str(self.particle))

        return self.particle


    def reward_function(self, pos, particles):
        """Update reward for a given State.
        where:
            pos -> current position
            particles -> number of particle in the bucket
        Params
        ======
            reward_pos ->  going along x axis
            reward_ori ->  straight orientationn
            reward_particle -> particle in bucket
        """
        # arm_joint = self.joint_state[1]  # TODO : compensation for unnecessary arm movements
        # bucket_joint = self.joint_state[2]
        reward_ground = 0
        reward_arm = 0
        reward_par = 0

        max_particle = 10
        ground = 0.02  # Ground treshhold

        bucket_link_x_pile = pos[0] - self.bucket_link_x + self.HALF_KOMODO
        bucket_pos = np.array([bucket_link_x_pile, self.bucket_link_z])
        min_end_pos = np.array([self.pile.sand_box_x, self.pile.sand_box_height])  # [ 0.35,0.25]
        arm_dist = math.sqrt((bucket_pos[0] - min_end_pos[0])**2 + (bucket_pos[1] - min_end_pos[1])**2)
        loc_dist = math.sqrt((bucket_pos[0] - min_end_pos[0]) ** 2)

        # Positive Rewards:
        reward_dist = 0.25 * (1 - math.tanh(arm_dist) ** 2)
        reward_tot = reward_dist
        w = 1 - (abs(self.particle - max_particle) / max(max_particle, self.particle)) ** 0.4

        if self.particle:
            reward_par = 0.125 + 0.25 * w
            reward_arm = -self.joint_state[1] - self.joint_state[2]
            reward_tot += reward_par + reward_arm
        else:
            reward_arm = -0.5*self.bucket_link_z  # 0.X
            reward_tot += reward_arm

        # Negative Rewards:
        # if self.z_tip < ground:# case z is bigger then ground
        #     reward_ground = -0.125
        #     reward_tot += reward_ground

        return reward_tot

    def step(self, action):

        np.set_printoptions(precision=1)

        # print('Velocity: {:0.2f}   Arm: {:0.2f}   Bucket: {:0.2f}').format(self.joint_state[0],self.joint_state[1],
        #                                                                        self.joint_state[2])
        keys = ["Particle", "X_tip", "Z_tip", "Bucket_x", "Bucket_z", "Distance", "Velocity", "Arm", "Bucket", "Diff_vel", "Diff_arm", "Diff_Bucket"]
        df = pandas.DataFrame((np.round(self.state,2)), columns=keys)
        print(df.to_string(index=False))

        # print('state:   {}'.format(np.round(self.state,2)))



        print('Action : {:0.2f}     {:0.2f}     {:0.2f}').format(action[0], action[1], action[2])  # action : [ vel , arm , bucket ]

        action = action * self.action_range
        self.joint_pos = np.clip(self.joint_pos + action, a_min=self.min_limit, a_max=self.max_limit)
        self.actions.move(self.joint_pos)
        # print('joint pos:', self.joint_pos)

        rospy.sleep(15.0/60.0)
        rospy.wait_for_service('/gazebo/get_model_state')
        model_state = self.get_model_state_proxy(self.get_model_state_req)
        pos = np.array([model_state.pose.position.x, model_state.pose.position.y, model_state.pose.position.z])

        p_in_bucket = self.check_particle_in_bucket()

        self.reward = self.reward_function(pos, p_in_bucket)

        print('Reward:    {:0.2f}').format(self.reward)

        normed_js = self.normalize_joint_state(self.joint_state) # TODO: Check without diff + Pos Z

        diff_joint = (normed_js - self.last_joint)

        arm_data = np.array([self.particle, self.x_tip, self.z_tip, self.bucket_link_x , self.bucket_link_z])

        model_data = np.array([pos[0]])

        self.state = np.concatenate((arm_data, model_data, normed_js, diff_joint)).reshape(1,-1)

        self.last_joint = normed_js
        self.last_pos = pos
        self.last_action = action

        curr_time = rospy.get_time()
        print('Time:    {:0.2f}').format(curr_time - self.episode_start_time)

        if (curr_time - self.episode_start_time) > self.max_sim_time:
            self.done = True
            self.reset()
        else:
            self.done = False

        # print('next state:  {}').format(np.round(self.state,2))
        # df_n = pandas.DataFrame((np.round(self.state,2)), columns=keys)
        # print(df_n.to_string(index=False))

        self.reward = np.clip(self.reward, a_min=-20, a_max=20)
        return self.state, self.reward, self.done

