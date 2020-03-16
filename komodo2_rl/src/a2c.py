# coding=utf-8
import numpy as np
import tensorflow as tf
import time
import os
import pickle
from utils import ReplayBuffer,OUNoise


# ===========================
#   Actor and Critic DNNs
# ===========================

class Models:

    def __init__(self, state_shape, action_shape, actor_lr=0.001, critic_lr=0.001, gamma=0.99,use_layer_norm=True):

        tf.reset_default_graph()

        # parameter definition
        self.state_shape = state_shape
        self.action_shape = action_shape
        self.num_actions = np.prod(self.action_shape)
        self.actor_lr = actor_lr
        self.critic_lr = critic_lr
        self.gamma = gamma
        self.use_layer_norm = use_layer_norm

        # inputs
        self.input_state = tf.placeholder(tf.float32, (None,) + self.state_shape, name='input_state')
        self.input_state_next = tf.placeholder(tf.float32, (None,) + self.state_shape, name='input_state_next')
        self.input_action = tf.placeholder(tf.float32, (None,) + self.action_shape, name='input_action')
        self.rewards = tf.placeholder(tf.float32, (None,1), name='rewards')
        self.dones = tf.placeholder(tf.float32, (None,1), name='dones')


        self.actor_actions, self.actor_dist = self.actor_net(self.input_state, self.num_actions, name='actor',
                                                use_layer_norm=self.use_layer_norm)
        self.critic = self.critic_net(self.input_state, self.input_action, name='critic',
                                                use_layer_norm=self.use_layer_norm)
        self.critic_next = self.critic_net(self.input_state_next, self.input_action, name='critic',reuse=True,
                                                use_layer_norm=self.use_layer_norm)

        self.actor_loss, self.critic_loss = self.set_model_loss(self.actor_actions,self.actor_dist,self.critic,self.critic_next,self.rewards, self.dones, self.gamma)


        self.actor_opt, self.critic_opt = self.set_model_opt(self.actor_loss, self.critic_loss,
                                    self.actor_lr, self.critic_lr)


    def actor_net(self, state, num_actions, name, reuse=False, training=True, use_layer_norm=True):
        """Build an actor (policy) network that maps states -> actions.
        Params
        ======
            state : Input state to the net
            num_actions (int): Number of action
            name (int): Name of the net
            fc1_units (int): Number of nodes in first hidden layer - right now 130
            fc2_units (int): Number of nodes in second hidden layer - right now 100
        """
        with tf.variable_scope(name, reuse=reuse):
            init_xavier = tf.contrib.layers.xavier_initializer()
            x = tf.layers.Dense(130,kernel_initializer=init_xavier)(state)  # outputs = activation(inputs * kernel + bias)
            if use_layer_norm:
                x = tf.contrib.layers.layer_norm(x)  # Adds a Layer Normalization layer.
            x = tf.nn.elu(x)
            x = tf.layers.Dense(100,kernel_initializer=init_xavier)(x)
            if use_layer_norm:
                x = tf.contrib.layers.layer_norm(x)
            x = tf.nn.elu(x)

            # Scale output to -action_bound to action_bound
            mu = tf.layers.Dense(num_actions, kernel_initializer=init_xavier)(x)
            sigma = tf.layers.Dense(num_actions, kernel_initializer=init_xavier)(x)
            sigma = tf.nn.softplus(sigma) + 1e-5
            norm_dist = tf.contrib.distributions.Normal(mu, sigma)
            action_tf_var = tf.squeeze(norm_dist.sample([1]), axis=0)

            return action_tf_var, norm_dist

    def critic_net(self, state, action, name, reuse=False, training=True, use_layer_norm=True):
        """Build a critic (value) network that maps (state)  -> V-values.
        Params
        ======
            state : Input state to the net
            action : Input Action
            name (String): Name of the net
            fc1_units (int): Number of nodes in first hidden layer - right now 130
            fc2_units (int): Number of nodes in second hidden layer - right now 100
        """
        num_outputs = 1
        with tf.variable_scope(name, reuse=reuse):
            init_xavier = tf.contrib.layers.xavier_initializer()
            x = tf.layers.Dense(130,kernel_initializer=init_xavier)(state)
            if use_layer_norm:
                x = tf.contrib.layers.layer_norm(x)
            x = tf.nn.elu(x)
            x = tf.layers.Dense(100,kernel_initializer=init_xavier)(x)
            if use_layer_norm:
                x = tf.contrib.layers.layer_norm(x)
            x = tf.nn.elu(x)
            v = tf.layers.Dense(num_outputs,kernel_initializer=init_xavier)(x)
        return v

    def set_model_loss(self, actor_actions,actor_dist, critic,critic_next, rewards, dones, gamma):
        """Update policy and value parameters using given batch of experience tuples.
        Q_targets = r + γ * critic_target(next_state, actor_target(next_state))
        where:
            actor_target(state) -> action
            critic_target(state, action) -> Q-value
        Params
        ======
            critic -> Q_expected { input: action , Output: Q }
            critic_of_actor -> Q_expected_actor with actor output action { input: actor action , Output: Q }
            critic_target -> Q_target_next { input: action_next , Output: Q_next }
        """

        # Compute Q targets for current states (y_i)
        # critic_target - Get predicted next-state actions and Q values from target  (Q_target_next_state)
        target = rewards + gamma * np.squeeze(critic_next)
        td = target - critic
        actor_loss = -tf.log(actor_dist.prob(actor_actions) + 1e-5) * td
        actor_loss -= 1e-4 * actor_dist.entropy()   # Add cross entropy cost to encourage exploration

        critic_loss = tf.reduce_mean(tf.squared_difference(tf.squeeze(critic), target))

        return actor_loss, critic_loss

    def set_model_opt(self, actor_loss, critic_loss, actor_lr, critic_lr):
        train_vars = tf.trainable_variables()
        actor_vars = [var for var in train_vars if var.name.startswith('actor')]
        critic_vars = [var for var in train_vars if var.name.startswith('critic')]
        with tf.control_dependencies(tf.get_collection(tf.GraphKeys.UPDATE_OPS)):
            actor_opt = tf.train.AdamOptimizer(actor_lr).minimize(actor_loss, var_list=actor_vars)
            critic_opt = tf.train.AdamOptimizer(critic_lr).minimize(critic_loss, var_list=critic_vars)
        return actor_opt, critic_opt


def build_summaries():
    episode_reward = tf.Variable(0.)
    tf.summary.scalar("Reward", episode_reward)
    episode_ave_max_q = tf.Variable(0.)
    tf.summary.scalar("Qmax Value", episode_ave_max_q)

    summary_vars = [episode_reward, episode_ave_max_q]
    summary_ops = tf.summary.merge_all()

    return summary_ops, summary_vars


# ===========================
#   A2C Agent
# ===========================

class A2C:
    """Reinforcement Learning agent that learns using Advantage actor critic."""

    def __init__(self,state_shape,action_shape,gamma=0.995,
                    actor_lr=0.0001, critic_lr=0.001,use_layer_norm=True):

        self.state_shape = state_shape
        self.action_shape = action_shape
        self.num_actions = np.prod(self.action_shape)

        # Replay memory -> for logging, not really used
        self.buffer_size = 100000
        self.memory = ReplayBuffer(self.buffer_size,self.action_shape, self.state_shape)

        # Algorithm parameters
        self.gamma = gamma # discount factor
        self.actor_lr = actor_lr
        self.critic_lr = critic_lr

        #initialize
        self.models = Models(self.state_shape, self.action_shape, actor_lr=self.actor_lr, critic_lr=self.critic_lr,
                       gamma=self.gamma, use_layer_norm=use_layer_norm)

        self.initialize()
        self.saver = tf.train.Saver()
        self.current_path = os.getcwd()

        #initial episode vars
        self.last_state = None
        self.last_action = None
        self.total_reward = 0.0
        self.count = 0
        self.episode_num = 0

    def reset_episode_vars(self):
        self.last_state = None
        self.last_action = None
        self.total_reward = 0.0
        self.count = 0

    def step(self, state, reward, done):

        action = self.act(state)
        self.count += 1
        if self.last_state is not None and self.last_action is not None:
            self.total_reward += reward
            self.memory.add(self.last_state, self.last_action, reward, state, done)
                     # add(self, state, action, reward, next_state, done):
            experiences = {'states':self.last_state,
                           'actions':self.last_action.reshape((1,3)),
                           'rewards':reward.reshape(-1,1),
                           'next_states':state,
                           'dones':np.array(int(done)).reshape(-1,1)}
            self.learn(experiences)
        self.last_state = state
        self.last_action = action
        if done:
            self.episode_num += 1
            eps_reward = self.total_reward
            print('Episode {}: total reward={:7.4f}, count={}'.format(self.episode_num,self.total_reward,self.count))
            self.reset_episode_vars()
            return action, eps_reward
        else:
            return action

    def act(self, states):# TODO:  noise inside the sample method
        """Returns actions for given state(s) as per current policy."""
        actions = self.sess.run(self.models.actor_actions, feed_dict={self.models.input_state:states})
        return np.clip(actions,a_min=-1.,a_max=1.).reshape(self.action_shape)

    def act_without_noise(self, states): # TODO:  fix to take mean instead of sample
        """Returns actions for given state(s) as per current policy."""
        actions = self.sess.run(self.models.actor_actions, feed_dict={self.models.input_state:states})
        return np.array(actions).reshape(self.action_shape)

    def learn(self, experiences):
        """Update policy and value parameters using given batch of experience tuples."""
        states = experiences['states']
        actions = experiences['actions']
        rewards = experiences['rewards']
        next_states = experiences['next_states']
        dones = experiences['dones']

        #actor critic update
        self.sess.run([self.models.actor_opt,self.models.critic_opt],feed_dict={self.models.input_state: states,
                                                                                self.models.input_action: actions,
                                                                                self.models.input_state_next: next_states,
                                                                                self.models.rewards: rewards,
                                                                                self.models.dones: dones})
    def initialize(self):
        """Soft update model parameters.
        θ_target = τ*θ_local + (1 - τ)*θ_target

        """
        self.sess = tf.Session()
        self.sess.run(tf.global_variables_initializer())
        actor_var = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope='actor')
        critic_var = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope='critic')


    def save_model(self):
        self.saver.save(self.sess,self.current_path + '/model_a2c/model.ckpt')

    def load_model(self,path):
        self.saver.restore(self.sess,path)

    def save_memory(self):
        mem_file = open(self.current_path + '/replay_buffer_memory_a2c.p','w')
        pickle.dump(self.memory,mem_file)
        mem_file.close()

    def load_memory(self,path):
        mem_file = open(self.current_path + '/replay_buffer_memory_a2c.p','r')
        mem = pickle.load(mem_file)
        self.memory = mem
        mem_file.close()