import gym

from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
import gym_minigrid
from gym_minigrid.wrappers import *
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
import torch as th

# multiprocess environment
policy_kwargs = dict(activation_fn=th.nn.Tanh,
                     net_arch=[dict(pi=[256, 256], vf=[256, 256])])
env = make_vec_env('MiniGrid-MultiHiPRLGridPose-v0', n_envs=8, start_index = 0)
#env = gym.make('MiniGrid-MultiHiPRLGrid-v0')
#check_env(env)
#env = Monitor(env)
#env = DummyVecEnv([lambda: env])
eval_callback = EvalCallback(env, best_model_save_path='./logs_hiprl_multi_pose_2048_max200_edit_success_rate/',
                             log_path='./logs_hiprl_multi_pose_2048_max200_edit_success_rate/', eval_freq=100, n_eval_episodes=100,
                             deterministic=True, render=False)
model = PPO("MlpPolicy", env, verbose=1, policy_kwargs = policy_kwargs, n_steps=256, batch_size=64, 
            ent_coef=0.005, n_epochs=10, learning_rate = 0.0002,
            tensorboard_log="/home/morin/catkin_ws/src/hiprl_replicate/Visualization/Baselines/")
model.learn(total_timesteps=900000, callback=eval_callback, tb_log_name="multi_hiprl_pose_2048_max200_edit_success_rate")
#model.learn(total_timesteps=50000, tb_log_name="test_stable_baseline3")

model.save("ppo2_HiPRLGrid_multi_pose_2048_max200_edit_success_rate")

del model # remove to demonstrate saving and loading

model = PPO.load("ppo2_HiPRLGrid_multi_pose_2048_max200_edit_success_rate")

# Enjoy trained agent
s_prime = env.reset()
dones = False
done_counter = 0
while done_counter < 20:
    while not dones:
        meta_action, _states = model.predict(s_prime)
        s_prime, rewards, dones, info = env.invoke(meta_action, render = True)
    if env.success == True:
        print("*****************************")
        print("Success: {}".format(done_counter))
        print("*****************************")
    s_prime = env.reset()
    done_counter += 1
    dones = False
    
