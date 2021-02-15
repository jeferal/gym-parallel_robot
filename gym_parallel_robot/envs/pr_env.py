import gym
from gym import error, spaces, utils
from gym.utils import seeding

import numpy as np

import rclpy
from rclpy.node import Node

from pr_msgs.msg import PRState
from pr_msgs.msg import PRArrayH

class PREnv(gym.Env, Node):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super().__init__('pr_env_node')
        print("Env initialized")

        #initialize variables
        self.obs = 0

        #create ROS2 communication
        self.publisher_ = self.create_publisher(
            PRArrayH, 
            'control_action', 
            1)
        self.subscription_ = self.create_subscription(
            PRState, 
            'pr_state',
            self._state_callback, 
            1)

    def step(self, action):
        #set action
        self._set_action(action)

        #get observation
        obs_ = self._get_obs()

        #calculate reward
        reward = self._calculate_reward()

        #check if done
        done = False

        #create info dic
        info = {
            "pr": 'parallel_robot'
            }

        print("Step Success")

        return (obs_, reward, done, info)

    def reset(self):
        print("Env reset")

    def render(self):
        print("Env rendered")

    def close(self):
        print("Env closed")
        self.destroy_node()
        rclpy.shutdown()

    def _state_callback(self, msg):
        #Update obs
        self.obs = np.array([msg.q.data, msg.q_vel.data])
        print("State cb")

    def _set_action(self, action):
        msg = PRArrayH()
        msg.current_time = self.get_clock().now()
        msg.data = action
        
        self.publisher_.publish(msg)

    def _get_obs(self):
        return self.obs

    def _calculate_reward(self):
        print("calculating reward")
        reward = 0
        return reward