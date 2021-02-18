import gym
from gym import error, spaces, utils
from gym.utils import seeding

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from pr_msgs.msg import PRState
from pr_msgs.msg import PRArrayH
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool, String

from time import sleep

class PREnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        print("Env initialized")
        #define spaces
        low_action = np.array([0.0, 0.0, 0.0, 0.0])
        high_action = np.array([9.5, 9.5, 9.5, 9.5])

        high_obs = np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])
        low_obs = -high_obs

        self.action_space = spaces.Box(low_action, high_action, shape=(4,))
        self.observation_space = spaces.Box(low_obs, high_obs, shape=(8,))
        self.reward_range = (-np.inf, 0)

        #create ROS2 node
        rclpy.init(args=None)
        self.node = rclpy.create_node("pr_env")

        #initialize variables
        self.obs = np.array([[0.695245, 0.691370, 0.693736, 0.654154],
                            [0.0, 0.0, 0.0, 0.0]])
        self.target_position = np.array([0.830578, 0.847096, 0.831132, 0.792133])
        self.status = "starting"
        self.time_obs_ns = 0
        self.time_obs_ns_ = self.time_obs_ns

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
        
        self.publisher_reset_vel = self.node.create_publisher(
            Bool,
            'der_reset',
            1)

        sim_qos = QoSProfile(depth=1)
        sim_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        sim_qos.history = QoSHistoryPolicy.KEEP_LAST
        sim_qos.durability = QoSDurabilityPolicy.VOLATILE

        self.subscription_sim_ = self.node.create_subscription(
            Quaternion,
            "posicion_sim",
            self._sim_callback,
            sim_qos)

        #create pub and sub for matlab macro
        self.publisher_start_ = self.node.create_publisher(
            Bool,
            "start_flag", 
            1)

        self.subscription_end_ = self.node.create_subscription(
            String,
            "status_sim",
            self._end_callback,
            1)

        start = input('Press any key to start')

    def step(self, action):
        #set action
        self._set_action(action)

        #get observation
        obs_ = self._get_obs()

        #calculate reward
        reward = self._calculate_reward(self.target_position, obs_)

        #check if done
        if self.status == 'all_clear' or self.status == 'error_sim':
            done = True
            reward = -1000
        else:
            done = False

        #create info dic
        info = {
            "pr": 'parallel_robot'
            }

        return (obs_.resize(8), reward, done, info)

    def reset(self):
        #start simulink thread
        self.status = "waiting for simulation"

        reset_msg = Bool()
        reset_msg.data = True
        self.publisher_reset_vel.publish(reset_msg)

        #send signal and wait for response
        while self.status != 'starting_simulation':
            msg = Bool()
            msg.data = True
            rclpy.spin_once(self.node, timeout_sec=1)
            self.publisher_start_.publish(msg)

        #wait for simulink
        while self.status != 'running':
            print('waiting for simulink')
            rclpy.spin_once(self.node, timeout_sec=1)

        print("simulation started")
        self.obs = np.array([[0.695245, 0.691370, 0.693736, 0.654154],
                            [0.0, 0.0, 0.0, 0.0]])
        obs = self.obs
        print(obs)
        return obs.resize(8)

    def render(self):
        print("Env rendered")

    def close(self):
        print("Env closed")
        self.node.destroy_node()
        rclpy.shutdown()

    def _state_callback(self, msg):
        #Update obs
        self.obs = np.array([msg.q.data, msg.q_vel.data])
        self.time_obs_ns = msg.current_time.nanosec

    def _set_action(self, action):
        print(action)
        print(type(action))
        msg = PRArrayH()
        msg.current_time = self.node.get_clock().now().to_msg()
        msg.data = action
        
        self.publisher_.publish(msg)

    def _get_obs(self):
        """
        Take observation from the environment and return it
        """
        #Sping until a new observation (with a different time stamp)
        while self.time_obs_ns == self.time_obs_ns_:
            rclpy.spin_once(self.node)
            if self.status == 'all_clear' or self.status == 'error_sim':
                break

        print(self.time_obs_ns)
        self.time_obs_ns_ = self.time_obs_ns
        return self.obs

    def _calculate_reward(self, target, pos):
        reward = - np.square(np.subtract(target, pos)).sum()
        return reward
    
    def _sim_callback(self, msg):
        self.status = "running"

    def _end_callback(self, msg):
        self.status = msg.data
        print(self.status)