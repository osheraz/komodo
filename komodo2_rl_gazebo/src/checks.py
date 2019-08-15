from __future__ import print_function
from komodo_env import KomodoEnvironment


env = KomodoEnvironment()
state_shape = env.state_shape
action_shape = env.action_shape

print('CHECKER')
observation, done = env.reset()
while True:
    print(env.check_particle_in_bucket())