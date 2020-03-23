import matplotlib.pyplot as plt
import numpy as np
import os

current_path = os.getcwd()
r = np.load(current_path + '/data/sim/eps_rewards_ddpg.npy')
r1 = np.load(current_path + '/data/sim/eps_rewards_a2c.npy')
# r2 = np.load(current_path + '/data/sim/eps_rewards.npy')

# p = np.load(current_path + '/data/sim/particle_end.npy')
# t = np.load(current_path + '/data/sim/time_end.npy')
# p =p[:800]
# r = np.where(r>0,r,0)
# t = t[:800] * 8 / 20
# t = np.where(t<1, 8,t)
N = 30
check = r
cumsum, moving_aves = [0], []
for i, x in enumerate(check, 1):
    cumsum.append(cumsum[i-1] + x)
    if i>=N:
        moving_ave = (cumsum[i] - cumsum[i-N])/N
        moving_aves.append(moving_ave)

r_fil = np.array(moving_aves)
fig = plt.figure(figsize=(10,7))
plt.plot(r,'k',label='ddpg')
# plt.plot(r_fil,'r',label='Episode end time')
plt.plot(r1,'r',label='a2c')
# plt.plot(r2,'g',label='d2')

plt.legend()
plt.grid()
plt.show()