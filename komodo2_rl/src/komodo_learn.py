from __future__ import print_function
from komodo_env import KomodoEnvironment
from ddpg import OUNoise, DDPG
from a2c import A2C

import numpy as np
from datetime import datetime
import os
import matplotlib.pyplot as plt

HALF_KOMODO = 0.53 / 2
np.set_printoptions(precision=1)
current_path = os.getcwd()
env = KomodoEnvironment()
state_shape = env.state_shape
action_shape = env.action_shape

model = 'ddpg'

if model == 'ddpg':
    agent = DDPG(state_shape,action_shape,batch_size=128,gamma=0.995,tau=0.001,
                                            actor_lr=0.0001, critic_lr=0.001, use_layer_norm=True)
    print('DDPG agent configured')
elif model == 'a2c':
    agent = A2C(state_shape,action_shape,gamma=0.995,actor_lr=0.0001, critic_lr=0.001, use_layer_norm=True)
    print('A2C agent configured')

max_episode = 1500
tot_rewards = []
print('env reset')
observation, done = env.reset()
action = agent.act(observation)
observation, reward, done = env.step(action)
noise_sigma = 0.15
save_cutoff = 1
cutoff_count = 0
save_count = 0
curr_highest_eps_reward = -1000.0
save = 1
particle_arr = np.array([1])
time_arr = np.array([1])

for i in range(max_episode):
    if i % 100 == 0 and noise_sigma>0.03 and model == 'ddpg':
        agent.noise = OUNoise(agent.num_actions,sigma=noise_sigma)
        noise_sigma /= 2.0
    step_num = 0
    flag = 1
    while done == False:
        step_num += 1
        action = agent.step(observation, reward, done)
        observation, reward, done = env.step(action)
        print('reward:',round(reward,3),'episode:', i, 'step:',step_num,'highest reward:',round(curr_highest_eps_reward, 3), 'saved:',save_count, 'cutoff count:', cutoff_count)
        print('\n-----------------------------------------------------------------------------------------------------\n')

        if reward > 3 and flag:
            particle = observation[0,0]  # amount of particle at the end
            timer =  step_num  # time elapsed from episode start
            print('Particle:', round(observation[0,0], 3),  'Step:', step_num)
            flag = 0

    action, eps_reward = agent.step(observation, reward, done)
    tot_rewards.append(eps_reward)
    if eps_reward > curr_highest_eps_reward:
        cutoff_count += 1
        curr_highest_eps_reward = eps_reward
    if cutoff_count >= save_cutoff:
        save_count += 1
        #print('saving_model at episode:',i)
        agent.save_model()
        agent.save_memory()
        cutoff_count = 0
    if flag:
        particle_arr = np.vstack((particle_arr, 0))  # amount of particle at the end
        time_arr = np.vstack((time_arr, 20))  # time elapsed from episode start
    else:
        particle_arr = np.vstack((particle_arr, particle))  # amount of particle at the end
        time_arr = np.vstack((time_arr, timer))  # time elapsed from episode start
    observation, done = env.reset()

if save:
    date_time = str(datetime.now().strftime('%d_%m_%Y_%H_%M'))
    np.save(current_path + '/data/sim/eps_rewards' + model + date_time, tot_rewards)
    np.save(current_path + '/data/sim/particle_end' + model + date_time, particle_arr)
    np.save(current_path + '/data/sim/time_end' + model + date_time, time_arr)
plt.plot(tot_rewards)
plt.show()