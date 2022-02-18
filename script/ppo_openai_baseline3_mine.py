import gym

from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
import gym_minigrid
from gym_minigrid.wrappers import *
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
import time
# multiprocess environment
#env = make_vec_env('MiniGrid-MyHiPRLGrid-v0', n_envs=8, start_index = 24)
env = gym.make('MiniGrid-MyHiPRLGrid-v0', process_num=8)
#check_env(env)
#env = Monitor(env)
#env = DummyVecEnv([lambda: env])
#eval_callback = EvalCallback(env, best_model_save_path='./logs/',
#                             log_path='./logs/', eval_freq=100, n_eval_episodes=100,
#                             deterministic=True, render=False)
#model = PPO("MlpPolicy", env, verbose=1, n_steps=128, batch_size=64, ent_coef=0.02, n_epochs=10, 
#            tensorboard_log="/home/morin/catkin_ws/src/hiprl_replicate/Visualization/Baselines/")
#model.learn(total_timesteps=100000, callback=eval_callback, 
#            tb_log_name="test_stable_baseline3_mine_new")
#model.learn(total_timesteps=50000, tb_log_name="test_stable_baseline3")

#model.save("ppo2_MyHiPRLGrid_new")

#del model # remove to demonstrate saving and loading

model = PPO.load("ppo2_MyHiPRLGrid_new")

# Enjoy trained agent
test_seed = 1338
env.seed(test_seed)
s_prime = env.reset()
dones = False
done_counter = 0
ep_success = []
ep_makespan = []
ep_reward = []
reward_sum = 0
while done_counter < 20:
    while not dones:
        meta_action, _states = model.predict(s_prime)
        s_prime, rewards, dones, info = env.invoke(meta_action, render = True)
        reward_sum = reward_sum + rewards
    
    ep_reward.append(reward_sum)
    reward_sum = 0
    ep_success.append(info['is_mission_succeeded'])
    done_counter = done_counter + 1    
    if env.success == True:
        ep_makespan.append(info['mission_completion_time'])
        print("*****************************")
        print("Success: {}".format(done_counter))
        print("*****************************")
    s_prime = env.reset()
    dones = False
    
print("IE-HiP-RL's 100 Evaluation Results:")
mean_rew = sum(ep_reward)/len(ep_reward)
if ep_makespan:
    mean_makespan = sum(ep_makespan) / len(ep_makespan)
else:
    mean_makespan = -1
mean_SR = sum(ep_success) / len(ep_success)
print("reward: {}, makespan: {}, success_rate: {}".format(mean_rew, mean_makespan, mean_SR))
