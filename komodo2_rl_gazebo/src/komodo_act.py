from __future__ import print_function
from komodo_model import KomodoEnvironment, torque_listener
from ddpg import DDPG
import time
import numpy as np

t_listener = torque_listener()
env = KomodoEnvironment()
state_shape = env.state_shape
action_shape = env.action_shape
agent = DDPG(state_shape,action_shape,batch_size=128,gamma=0.995,tau=0.001,
                                        actor_lr=0.0005, critic_lr=0.001, use_layer_norm=True)
print('DDPG agent configured')
agent.load_model(agent.current_path + '/model/model.ckpt')
print('Resetting joint positions')
observation = env.reset()
observation_arr = observation
print('Reset!')
time.sleep(2.0)
for i in range(40):
    action = agent.act_without_noise(observation)
    if i == 0 : action_arr = action
    observation = env.step(action)
    action_arr = np.vstack((action_arr, action))
    observation_arr = np.vstack((observation_arr, observation))
    print('------------------------------------------------------------------')
    time.sleep(0.1)

t_listener.torque_plot()
print('Resetting joint positions')
env.reset()
print('Reset')
import matplotlib.pyplot as plt
#plt.plot(observation_arr[:][:])
#plt.show()