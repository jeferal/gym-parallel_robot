import gym
from gym import error, spaces, utils
from gym.utils import seeding

class PREnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        raise NotImplementedError

    def step(self, action):
        raise NotImplementedError

    def reset(self):
        raise NotImplementedError

    def render(self):
        raise NotImplementedError

    def close(self):
        raise NotImplementedError