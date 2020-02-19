import matplotlib.pyplot as plt
import numpy as np
import os

current_path = os.getcwd()
r = np.load(current_path + '/data/sim/eps_rewards.npy')
p = np.load(current_path + '/data/sim/particle_end.npy')
t = np.load(current_path + '/data/sim/time_end.npy')
p =p[:800]
t = t[:800] * 8 / 20
N = 30
cumsum, moving_aves = [0], []
for i, x in enumerate(p, 1):
    cumsum.append(cumsum[i-1] + x)
    if i>=N:
        moving_ave = (cumsum[i] - cumsum[i-N])/N
        moving_aves.append(moving_ave)

r_fil = np.array(moving_aves)
fig = plt.figure(figsize=(10,7))
plt.plot(p,'bo',alpha=0.1,label='Episode end time')
plt.plot(r_fil,'k',label='Episode end time')
plt.legend()
plt.grid()
plt.show()