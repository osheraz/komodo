# coding=utf-8
import numpy as np

# ===========================
#   Replay Buffer
# ===========================

class ReplayBuffer:
    """Fixed-size buffer to store experience tuples."""
    def __init__(self, maxlen, action_shape, state_shape, dtype=np.float32):
        """Initialize a ReplayBuffer object."""
        self.maxlen = maxlen
        self.start = 0
        self.length = 0
        self.state_data = np.zeros((maxlen,) + state_shape).astype(dtype)
        self.action_data = np.zeros((maxlen,) + action_shape).astype(dtype)
        self.reward_data = np.zeros((maxlen,1)).astype(dtype)
        self.next_state_data = np.zeros((maxlen,) + state_shape).astype(dtype)
        self.done_data = np.zeros((maxlen,1)).astype(dtype)

    def add(self, state, action, reward, next_state, done):
        """Add a new experience to memory."""
        if self.length == self.maxlen:
            self.start = (self.start + 1) % self.maxlen
        else:
            self.length += 1
        idx = (self.start + self.length - 1) % self.maxlen
        self.state_data[idx] = state
        self.action_data[idx] = action
        self.reward_data[idx] = reward
        self.next_state_data[idx] = next_state
        self.done_data[idx] = done

    def sample(self, batch_size=64):
        """Randomly sample a batch of experiences from memory."""
        idxs = np.random.randint(0,self.length - 1, size=batch_size)
        sampled = {'states':self.set_min_ndim(self.state_data[idxs]),
                   'actions':self.set_min_ndim(self.action_data[idxs]),
                   'rewards':self.set_min_ndim(self.reward_data[idxs]),
                   'next_states':self.set_min_ndim(self.next_state_data[idxs]),
                   'dones':self.set_min_ndim(self.done_data[idxs])}
        return sampled

    def set_min_ndim(self,x):
        """set numpy array minimum dim to 2 (for sampling)"""
        if x.ndim < 2:
            return x.reshape(-1,1)
        else:
            return x

    def __len__(self):
        return self.length

# ===========================
#   Noise
# ===========================

class OUNoise:
    """Ornstein-Uhlenbeck process."""

    def __init__(self, size, mu=None, theta=0.15, sigma=0.03, dt=1e-2):
        """Initialize parameters and noise process."""
        self.size = size
        self.mu = mu if mu is not None else np.zeros(self.size)
        self.theta = theta
        self.sigma = sigma
        self.dt = dt
        self.state = np.ones(self.size) * self.mu
        self.reset()

    def reset(self):
        """Reset the internal state (= noise) to mean (mu)."""
        self.state = np.ones(self.size) * self.mu

    def sample(self):
        """Update internal state and return it as a noise sample."""
        x = self.state
        dx = self.theta * (self.mu - x) * self.dt + self.sigma * np.sqrt(self.dt) * np.random.randn(len(x))
        self.state = x + dx
        return self.state