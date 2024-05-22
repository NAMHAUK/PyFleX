import os
import numpy as np
import pyflex
import math
from stable_baselines3 import PPO
import time
import gymnasium as gym

import test_env as ENV


# env =ENV.Continuous_Env()
# num_cpu = 4  # Number of processes to use
#     # Create the vectorized environment
# vec_env = make_vec_env(ENV.Continuous_Env(), n_envs = num_cpu)
print("Aa")
# pyflex.init()
env =ENV.MyEnv()
print("환경 선언 성공")

# model = PPO("MlpPolicy", env, tensorboard_log="./ppo_spring_tensorboard/")
# print("model 생성 성공")

# model.learn(total_timesteps=1000000, tb_log_name="first_run")
# print("model learn 성공")

# model.save("spring_test")
# print("model save 성공")

# model = PPO.load("spring_test")
# print("model load 성공")

obs, info = env.reset()
print("env reset 성공")
terminated = False
while (terminated==False):
    # action, _states = model.predict(obs)
    action = np.array([0,70])
    obs, rewards, terminated, truncated, info = env.step(action)
    if truncated == True:
        obs, info = env.reset()

env.close()
