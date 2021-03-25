from gym.envs.registration import register

register(
    id='ParallelRobot-v0',
    entry_point='gym_parallel_robot.envs:PREnv',
)

register(
    id='ParallelRobot3DOF-v0',
    entry_point='gym_parallel_robot.envs:PR3DOFEnv',
)