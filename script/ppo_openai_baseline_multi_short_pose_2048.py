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
import csv

# multiprocess environment
policy_kwargs = dict(activation_fn=th.nn.Tanh,
                     net_arch=[dict(pi=[256, 256], vf=[256, 256])])
#env = make_vec_env('MiniGrid-MultiHiPRLGridShortPose-v0', n_envs=8, start_index = 8)
env = gym.make('MiniGrid-MultiHiPRLGridShortPose-v0', process_num = 0)
#check_env(env)
#env = Monitor(env)
#env = DummyVecEnv([lambda: env])
#eval_callback = EvalCallback(env, best_model_save_path='./logs_multi_hiprl_ent002/',
#                             log_path='./logs_multi_hiprl_ent002/', eval_freq=100, n_eval_episodes=100,
#                             deterministic=True, render=False)
#model = PPO("MlpPolicy", env, verbose=1, policy_kwargs = policy_kwargs, n_steps=256, batch_size=64, 
#            ent_coef=0.02, n_epochs=10, learning_rate = 0.0002,
#            tensorboard_log="/home/morin/catkin_ws/src/hiprl_replicate/Visualization/Baselines/")
#model.learn(total_timesteps=300000, callback=eval_callback, tb_log_name="multi_hiprl_ent002")
#model.learn(total_timesteps=50000, tb_log_name="test_stable_baseline3")

#model.save("ppo_multi_HiPRL_ent002")

#del model # remove to demonstrate saving and loading
mean_rew_ = []
mean_makespan_ = []
mean_SR_ = []
for i in range(20):
    model = PPO.load("multi_hiprl_short_pose_256_final" + str(i+1))
    
    
    # Enjoy trained agent
    test_seed = 1338
    env.seed(test_seed)
    s_prime = env.reset()
    dones = False
    done_counter = 0
    ep_reward = []
    ep_success = []
    ep_makespan = []
    while done_counter < 1000:
        reward = 0
        while not dones:
            meta_action, _states = model.predict(s_prime)
            s_prime, rewards, dones, info = env.invoke(meta_action, render = False)
            reward = reward + rewards
        if env.success == True:
            ep_success.append(1)
            ep_makespan.append(env.step_count)
            print("*****************************")
            print("Success: {}, Makespan: {}".format(done_counter, env.step_count))
            print("*****************************")
        else:
            ep_success.append(0)
        
        ep_reward.append(reward)
        s_prime = env.reset()
        done_counter += 1
        dones = False
    
    mean_rew = sum(ep_reward)/len(ep_reward)
    if ep_makespan:
        mean_makespan = sum(ep_makespan) / len(ep_makespan)
    else:
        mean_makespan = -1
    mean_SR = sum(ep_success) / len(ep_success)
    assert len(ep_reward) == 1000
    assert len(ep_success) == 1000
    print("*****************************")
    print("Success rate: {}, Mean Makespan: {}, Mean episode reward sum: {}".format(mean_SR, mean_makespan, mean_rew))
    print("*****************************")
    mean_rew_.append(mean_rew)
    mean_makespan_.append(mean_makespan)
    mean_SR_.append(mean_SR)

with open('Validation_Curve_HiPRL_multi', 'w') as f:
    write = csv.writer(f)
    write.writerow(['Step', 'Success Rate', 'Makespan', 'Episode Reward'])
    for j in range(20):
        write.writerow([j, mean_SR_[j], mean_makespan_[j], mean_rew_[j]])