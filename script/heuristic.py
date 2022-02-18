import gym
import csv
from stable_baselines3.common.env_util import make_vec_env
import gym_minigrid
from gym_minigrid.wrappers import *
import matplotlib.pyplot as plt
import time

# multiprocess environment
env = make_vec_env('MiniGrid-MyHiPRLGrid-v0', n_envs=1, start_index = 40)
#env = gym.make('MiniGrid-MyHiPRLGrid-v0')
#check_env(env)

# Enjoy trained agent
s_prime = env.reset()
done_counter = 0
ep_success = []
ep_makespan = []
ep_reward = []
reward_sum = 0
while done_counter < 100:
    num_box = len(env.env_method('unchecked_seen_box')[0])
    found_ball = env.env_method('found_ball')[0]
    carrying = (env.env_method('Carrying')[0] != None)
    found_goal = env.env_method('found_goal')[0]
    
    if env.env_method('steps_remained')[0] == 100:
        meta_action = [1]
    
    elif num_box > 0 and not found_ball:
        meta_action = [2]
    
    elif (found_ball and not carrying):
        meta_action = [2]
    
    elif (carrying and found_goal):
        meta_action = [2]
    
    else:
        meta_action = [0]
        
    s_prime, reward, done, info = env.invoke(meta_action)
    time.sleep(1.0)
    reward_sum = reward_sum + reward[0]
    done = done[0]
    info = info[0]
    if done:
        reward_sum = 0
        ep_reward.append(reward[0])
        if info['is_mission_succeeded'] == True:
            ep_makespan.append(info['mission_completion_time'])
        ep_success.append(info['is_mission_succeeded'])
        done_counter = done_counter + 1

env.close()

    
print("Heuristic algorithm's 100 Evaluation Results:")
mean_rew = sum(ep_reward)/len(ep_reward)
if ep_makespan:
    mean_makespan = sum(ep_makespan) / len(ep_makespan)
else:
    mean_makespan = -1
mean_SR = sum(ep_success) / len(ep_success)
print("reward: {}, makespan: {}, success_rate: {}".format(mean_rew, mean_makespan, mean_SR))


with open('logging_heuristic.csv', 'w', newline='') as f:
    write = csv.writer(f)
    write.writerow(ep_success)
    write.writerow(ep_reward)
    write.writerow(ep_makespan)
