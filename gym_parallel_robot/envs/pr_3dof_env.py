import gym
from gym import error, spaces, utils
from gym.utils import seeding

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from pr_3dof_msgs.msg import PRState
from pr_3dof_msgs.msg import PRArrayH
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String

from time import sleep

class PR3DOFEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        print("Env initialized")
        #define spaces
        self.max_v = 9.5
        self.min_v = -9.5
        self.max_obs = 2.0
        self.min_obs = -2.0

        self.iter = 0.0
        self.time = 0
        self.time_0 = 0

        #Parameters Reward function
        self.a = 1000
        self.b = 35

        self.action_space = spaces.Box(
            self.min_v, 
            self.max_v, 
            shape=(3,),
            dtype=np.float32
        )

        self.observation_space = spaces.Box(
            self.min_obs, 
            self.max_obs, 
            shape=(9,),
            dtype=np.float32
        )
        
        self.reward_range = (-np.inf, 0)

        #create ROS2 node
        rclpy.init(args=None)
        self.node = rclpy.create_node("pr_env")

        #initialize variables
        self.target_position = np.array([0.4854255416633526, -0.1666953653732201, 0.2495862280273856])

        first_position = np.array([0.1203, 0.2731, 0.2425])
        first_velocity = np.array([0.0, 0.0, 0.0])
        self.first_obs = np.concatenate((first_position, first_velocity, (self.target_position-first_position)), axis=0)

        self.obs = self.first_obs

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
            Point,
            "position_sim",
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

        self.seed()
        start = input('Press any key to start')

    def step(self, action):
        #set action
        self._set_action(action)

        #get observation
        obs_ = self._get_obs()

        #calculate reward
        pos = obs_[0:3]
        vel = obs_[3:6]
        reward = self._calculate_reward(self.target_position, pos, vel, action)

        done = False
        #check if done
        if self.status == 'all_clear' or self.status == 'error_sim':
            done = True
            print("Done = True from simulation")
        
        #TODO Stop when obs > 1.99
        """
        if len(pos[pos>1.99]) > 0 or len(pos[pos<-1.99]) > 0:
            done = True
            print("Done = True from overflow")
            self.status = 'limit_overflow'
            reward = - self.a*(1+np.exp(-self.time/self.b))
            while self.status != 'error_sim':
                print('waiting for finishing simulation')
                rclpy.spin_once(self.node)
                if self.status == 'all_clear':
                    break
            self.status = 'done'
        """

        if self.status == 'error_sim' or self.status == 'limit_overflow':
            print('Reward crash: ')
            print(self.iter)
            print(reward)
            reward = - self.a*(1+np.exp(-self.time/self.b))

        #create info dic
        info = {
            "pr": 'parallel_robot'
            }

        self.iter = self.iter + 1.0
        return (obs_, reward, done, info)

    def reset(self):
        #start simulink thread
        self.status = "waiting for simulation"
        self.time_0 = 0
        self.iter = 0

        #send signal and wait for response
        while self.status != 'starting_simulation':
            print('waiting for MATLAB')
            msg = Bool()
            msg.data = True
            self.publisher_start_.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=1)
        
        #Reset velocity
        reset_msg = Bool()
        reset_msg.data = True
        self.publisher_reset_vel.publish(reset_msg)

        #wait for simulink
        while self.status != 'running':
            msg = Bool()
            msg.data = True
            self.publisher_start_.publish(msg)
            print('waiting for simulink')
            rclpy.spin_once(self.node, timeout_sec=1)

        print("simulation started")
        obs = self.first_obs

        print(obs)
        return obs

    def render(self):
        raise NotImplementedError

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def _state_callback(self, msg):
        #Update obs
        if self.time_0 == 0:
            self.time_0 = msg.current_time.sec + msg.current_time.nanosec/1000000000
            
        self.obs = np.concatenate((msg.q.data, msg.q_vel.data, (self.target_position-msg.q.data)), axis=0)
        self.time_obs_ns = msg.current_time.nanosec
        time = msg.current_time.sec + msg.current_time.nanosec/1000000000
        self.time = time - self.time_0

    def _set_action(self, action):
        msg = PRArrayH()
        msg.current_time = self.node.get_clock().now().to_msg()
        msg.data[0] = action[0]
        msg.data[1] = action[1]
        msg.data[2] = action[2]
        
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

        self.time_obs_ns_ = self.time_obs_ns
        return self.obs

    def _calculate_reward(self, target, pos, vel, action):
        reward = - np.power(np.subtract(target, pos),2).mean() 
        #+ 0.1*np.power(vel,2).mean() + 0.001*np.power(action,2).mean())
        return reward
    
    def _sim_callback(self, msg):
        self.status = "running"

    def _end_callback(self, msg):
        print(self.status)
        self.status = msg.data

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]