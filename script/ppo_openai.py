import gym

from stable_baselines.common.policies import CnnPolicy
from stable_baselines.common import make_vec_env
from stable_baselines.common.env_checker import check_env
from stable_baselines import PPO2
import gym_minigrid
from gym_minigrid.wrappers import *
import matplotlib.pyplot as plt
from stable_baselines.common.callbacks import EvalCallback

# multiprocess environment
env = make_vec_env('MiniGrid-HiPRLGrid-v0', n_envs=1)
#env = gym.make('MiniGrid-HiPRLGrid-v0')
#check_env(env)
eval_callback = EvalCallback(env, best_model_save_path='./logs/',
                             log_path='./logs/', eval_freq=500, n_eval_episodes=100,
                             deterministic=True, render=False)
model = PPO2(CnnPolicy, env, verbose=1, nminibatches=32, noptepochs=10, tensorboard_log="/home/morin/catkin_ws/src/hiprl_replicate/Visualization/Baselines/")
model.learn(total_timesteps=50000, callback=eval_callback)
model.save("ppo2_HiPRLGrid")

#del model # remove to demonstrate saving and loading

model = PPO2.load("ppo2_HiPRLGrid")

# Enjoy trained agent
s_prime = env.reset()
dones = False
while not dones:
    meta_action, _states = model.predict(s_prime)
    s_prime, rewards, dones, info = env.invoke(meta_action, render = True)
