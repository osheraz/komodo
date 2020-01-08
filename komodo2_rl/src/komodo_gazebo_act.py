from __future__ import print_function
from komodo_env import KomodoEnvironment
from ddpg import DDPG
from torque_listener import TorqueListener
import numpy as np
import os
from datetime import datetime
import rospy
import matplotlib.pyplot as plt

save = 0
current_path = os.getcwd()
t_listener = TorqueListener()
env = KomodoEnvironment()
state_shape = env.state_shape
action_shape = env.action_shape
agent = DDPG(state_shape,action_shape,batch_size=128,gamma=0.995,tau=0.001,
                                        actor_lr=0.0005, critic_lr=0.001, use_layer_norm=True)
print('DDPG agent configured')
agent.load_model(agent.current_path + '/model/model.ckpt')
max_episode = 5
particle_arr = np.array([1])
time_arr = np.array([1])

for i in range(max_episode):
    print('---------------------------env reset---------------------------------------------------------')
    observation, done = env.reset()
    action = agent.act_without_noise(observation)
    observation, reward, done = env.step(action)
    step_num = 0
    observation_arr = observation
    action_arr = action
    while done == False:
        step_num += 1
        action = agent.act_without_noise(observation)
        observation, reward, done = env.step(action)
        observation_arr = np.vstack((observation_arr, observation))
        action_arr= np.vstack((action_arr, action))
        print('Reward:', round(reward,3), 'Episode:', i, 'Step:', step_num)
        print('------------------------------------------------------------------------------------------')
        if (observation[0,6] < 0):
            particle_arr = np.vstack((particle_arr, observation[0,0]))
            time_arr = np.vstack((time_arr, step_num))

np.save(current_path + '/data/sim/particle_end', particle_arr)
np.save(current_path + '/data/sim/time_end', time_arr)

if save:
    date_time =  str(datetime.now().strftime('%d_%m_%Y_%H_%M'))
    np.save(current_path + '/data/sim/observation_' + date_time,observation_arr)
    np.save(current_path + '/data/sim/action_' + date_time,action_arr)
    t_listener.force_plot()

#plt.plot(observation_arr[:][:])
#plt.show()