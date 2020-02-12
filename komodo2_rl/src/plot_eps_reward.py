import matplotlib.pyplot as plt
import numpy as np
import os

current_path = os.getcwd()
r = np.load(current_path + '/data/sim/eps_rewards08_01_2020_20_28.npy')
p = np.load(current_path + '/data/sim/particle_end.npy')
t = np.load(current_path + '/data/sim/time_end.npy')

N = 30
cumsum, moving_aves = [0], []
for i, x in enumerate(r, 1):
    cumsum.append(cumsum[i-1] + x)
    if i>=N:
        moving_ave = (cumsum[i] - cumsum[i-N])/N
        moving_aves.append(moving_ave)

r_fil = np.array(moving_aves)
fig = plt.figure(figsize=(10,7))
plt.title('Episode Rewards')
plt.plot(r,'bo',alpha=0.1,label='episode rewards')
plt.plot(r_fil,'k',label='average episode rewards')
plt.legend()
plt.grid()
plt.show()