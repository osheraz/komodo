from __future__ import print_function
from komodo_env import KomodoEnvironment
from ddpg import DDPG
from torque_listener import TorqueListener
import numpy as np

t_listener = TorqueListener()
env = KomodoEnvironment()
state_shape = env.state_shape
action_shape = env.action_shape
agent = DDPG(state_shape,action_shape,batch_size=128,gamma=0.995,tau=0.001,
                                        actor_lr=0.0005, critic_lr=0.001, use_layer_norm=True)
print('DDPG agent configured')
agent.load_model(agent.current_path + '/model/model.ckpt')
max_episode = 3
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

t_listener.force_plot()
np.save('observation',observation_arr)
np.save('action',observation_arr)

import matplotlib.pyplot as plt
#plt.plot(observation_arr[:][:])
#plt.show()