import gym
from gym import error, spaces, utils
from gym.utils import seeding

import numpy as np

import rclpy
from rclpy.node import Node

from pr_msgs.msg import PRState
from pr_msgs.msg import PRArrayH

class PREnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        print("Env initialized")

        #create ROS2 node
        self.node = rclpy.create_node("pr_env")

        #initialize variables
        self.obs = np.array([])
        self.target_position = np.array([0.6, 0.6, 0.6, 0.6])

        #create ROS2 communication
        self.publisher_ = self.node.create_publisher(
            PRArrayH, 
            'control_action', 
            1)
        self.subscription_ = self.node.create_subscription(
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
        reward = self._calculate_reward(self.target_position, obs_)

        #check if done
        done = False

        #create info dic
        info = {
            "pr": 'parallel_robot'
            }

        return (obs_, reward, done, info)

    def reset(self):
        obs = self._get_obs()
        print("Env reset")
        return obs

    def render(self):
        print("Env rendered")

    def close(self):
        print("Env closed")
        self.node.destroy_node()
        rclpy.shutdown()

    def _state_callback(self, msg):
        #Update obs
        self.obs = np.array([msg.q.data, msg.q_vel.data])

    def _set_action(self, action):
        msg = PRArrayH()
        msg.current_time = self.node.get_clock().now().to_msg()
        msg.data = action
        
        self.publisher_.publish(msg)

    def _get_obs(self):
        """
        Take observation from the environment and return it
        """
        rclpy.spin_once(self.node)
        #Check that the observation is not prior to the action TODO
        return self.obs

    def _calculate_reward(self, target, pos):
        reward = - np.square(np.subtract(target, pos)).sum()
        print(reward)
        return reward