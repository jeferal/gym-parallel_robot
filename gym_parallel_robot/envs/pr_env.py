import gym
from gym import error, spaces, utils
from gym.utils import seeding

class PREnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        print("Env initialized")

    def step(self, action):
        print("Step Success")

    def reset(self):
        print("Env reset")

    def render(self):
        print("Env rendered")

    def close(self):
        print("Env closed")