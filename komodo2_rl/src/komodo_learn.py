from __future__ import print_function
from komodo_env import KomodoEnvironment
from ddpg import OUNoise, DDPG
import numpy as np
from datetime import datetime
import os
import matplotlib.pyplot as plt

np.set_printoptions(precision=1)
current_path = os.getcwd()
env = KomodoEnvironment()
state_shape = env.state_shape
action_shape = env.action_shape
agent = DDPG(state_shape,action_shape,batch_size=128,gamma=0.995,tau=0.001,
                                        actor_lr=0.0001, critic_lr=0.001, use_layer_norm=True)
print('DDPG agent configured')
max_episode = 1000
tot_rewards = []
print('env reset')
observation, done = env.reset()
action = agent.act(observation)
observation, reward, done = env.step(action)
noise_sigma = 0.3
save_cutoff = 1
cutoff_count = 0
save_count = 0
curr_highest_eps_reward = -1000.0

for i in range(max_episode):
    if i % 100 == 0 and noise_sigma>0.03:
        agent.noise = OUNoise(agent.num_actions,sigma=noise_sigma)
        noise_sigma /= 2.0
    step_num = 0
    while done == False:
        step_num += 1
        action = agent.step(observation, reward, done)
        observation, reward, done = env.step(action)
        print('reward:',round(reward,3),'episode:', i, 'step:',step_num,'highest reward:',round(curr_highest_eps_reward, 3), 'saved:',save_count, 'cutoff count:', cutoff_count)
        print('\n-----------------------------------------------------------------------------------------------------\n')


    action, eps_reward = agent.step(observation, reward, done)
    tot_rewards.append(eps_reward)
    if eps_reward > curr_highest_eps_reward:
        cutoff_count += 1
        curr_highest_eps_reward = eps_reward
    if cutoff_count >= save_cutoff:
        save_count += 1
        print('saving_model at episode:',i)
        agent.save_model()
        agent.save_memory()
        cutoff_count = 0
    observation, done = env.reset()

date_time = str(datetime.now().strftime('%d_%m_%Y_%H_%M'))
np.save(current_path + '/data/sim/eps_rewards' + date_time, tot_rewards)

plt.plot(tot_rewards)
plt.show()