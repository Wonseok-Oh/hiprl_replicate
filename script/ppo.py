import gym
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Categorical
import gym_minigrid
from gym_minigrid.wrappers import *
import matplotlib.pyplot as plt
from gym_minigrid.window import Window
import time

#Hyperparameters
learning_rate = 0.0005
gamma         = 0.98
lmbda         = 0.95
eps_clip      = 0.1
K_epoch       = 3
T_horizon     = 20
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class PPO(nn.Module):
    def __init__(self):
        super(PPO, self).__init__()
        self.data = []
        
        # Suppose input width, height is W, H respectively FH = Kernel(Filter) size = K
        # Ouput Height for CNN = (H + 2P - FH)/S + 1
        # size calculation referenced http://taewan.kim/post/cnn/
        # single channel case
        # self.obj_conv1 = nn.Conv2d(1, 32, 7, stride = 2, padding = 1)
        self.obj_conv1 = nn.Conv2d(7, 32, 7, stride = 2, padding = 1) 
        self.obj_conv2 = nn.Conv2d(32, 32, 5, stride = 1, padding = 1)
        self.obj_conv3 = nn.Conv2d(32, 32, 3, stride = 1, padding = 1)
    
        self.fc1 = nn.Linear(32, 512)
        #self.fc1 = nn.Linear(3872, 512)
        self.fc_pi = nn.Linear(512, 3)
        self.fc_v = nn.Linear(512, 1)
        self.optimizer = optim.Adam(self.parameters(), lr=learning_rate)

    def pi(self, x, softmax_dim = 1):
        # forward propagation object_map_input
        x = F.elu(self.obj_conv1(x))
        x = F.elu(self.obj_conv2(x))
        x = F.elu(self.obj_conv3(x))
        x = torch.flatten(x, 1)
        #print(x.size())
        x = F.elu(self.fc1(x))
        x = self.fc_pi(x)
        prob = F.softmax(x, dim=softmax_dim)
        return prob
    
    def v(self, x):
        x = F.elu(self.obj_conv1(x))
        x = F.elu(self.obj_conv2(x))
        x = F.elu(self.obj_conv3(x))
        x = torch.flatten(x, 1)

        x = F.elu(self.fc1(x))
        v = self.fc_v(x)
        return v
      
    def put_data(self, transition):
        self.data.append(transition)
        
    def make_batch(self):
        s_lst, a_lst, r_lst, s_prime_lst, prob_a_lst, done_lst = [], [], [], [], [], []
        for transition in self.data:
            s, a, r, s_prime, prob_a, done = transition
            if len(list(torch.from_numpy(s).size())) > 1 and len(list(torch.from_numpy(s_prime).size())) > 1:
                s_lst.append(s)
                a_lst.append([a])
                r_lst.append([r])
                s_prime_lst.append(s_prime)
                prob_a_lst.append([prob_a])
                done_mask = 0 if done else 1
                done_lst.append([done_mask])
            
        s,a,r,s_prime,done_mask, prob_a = torch.tensor(s_lst, dtype=torch.float).to(device), torch.tensor(a_lst).to(device), \
                                          torch.tensor(r_lst).to(device), torch.tensor(s_prime_lst, dtype=torch.float).to(device), \
                                          torch.tensor(done_lst, dtype=torch.float).to(device), torch.tensor(prob_a_lst).to(device)
        #print("{}, {}, {}, {}, {}, {}".format(s,a,r,s_prime,done_mask, prob_a))
        self.data = []
        return s, a, r, s_prime, done_mask, prob_a
        
    def train_net(self):
        loss_list = []
        s, a, r, s_prime, done_mask, prob_a = self.make_batch()
        #s = s.unsqueeze(1)
        #s_prime = s_prime.unsqueeze(1)
        print("s.size(): {}, s_prime.size(): {}".format(s.size(), s_prime.size()))
        for i in range(K_epoch):
            td_target = r + gamma * self.v(s_prime) * done_mask
            delta = (td_target - self.v(s)).to("cpu")
            delta = delta.detach().numpy()

            advantage_lst = []
            advantage = 0.0
            for delta_t in delta[::-1]:
                advantage = gamma * lmbda * advantage + delta_t[0]
                advantage_lst.append([advantage])
            advantage_lst.reverse()
            advantage = torch.tensor(advantage_lst, dtype=torch.float).to(device)

            pi = self.pi(s, softmax_dim=1)
            pi_a = pi.gather(1,a)
            ratio = torch.exp(torch.log(pi_a) - torch.log(prob_a)).to(device)  # a/b == exp(log(a)-log(b))

            surr1 = ratio * advantage
            surr2 = torch.clamp(ratio, 1-eps_clip, 1+eps_clip).to(device) * advantage
            loss = -torch.min(surr1, surr2) + F.smooth_l1_loss(self.v(s) , td_target.detach()).to(device)
            loss_list.append(loss.mean())

            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()
        
        return sum(loss_list, 0.0) / len(loss_list)

        
def main():
    window = Window('gym_minigrid - ' + 'MiniGrid-HiPRLGrid-v0')
    env = gym.make('MiniGrid-HiPRLGrid-v0')
    model = PPO().to(device)
    score = 0.0
    print_interval = 20

    loss_list = []
    reward_list = []

    for n_epi in range(10000):
        s = env.reset()
        done = False
        loss_epi = []
        reward_epi = []
        timing_data_epi = []

        while not done:
            while not done:
                print("remaining steps: {}".format(env.steps_remaining))
                input = torch.from_numpy(s).float()
                print(input.size())
                #input = input.unsqueeze(0)
                input = input.unsqueeze(0)
                
                prob = model.pi(input.to(device))
                print(input)
                print(prob)
                m = Categorical(prob)
                a = m.sample().item()
                obs_prime, r, done, info, s_prime = env.invoke(a)
                reward_epi.append(r)
                    
                prob = torch.squeeze(prob)
                model.put_data((s, a, r, s_prime, prob[a].item(), done))
                s = s_prime
                print("selected meta-action: {}, r: {}".format(a,r))
                score += r
                if done:
                    break
                
            if len(model.data) > 0:
                loss_mean = model.train_net()
                loss_epi.append(loss_mean)

        loss_list.append(sum(loss_epi, 0.0) / len(loss_epi))
        print(reward_epi)
        print(sum(reward_epi))
        reward_list.append(sum(reward_epi))

        with open("/home/morin/catkin_ws/src/hiprl_replicate/Visualization/PPO/loss.txt", "w") as f:
            for s in loss_list:
                f.write(str(s.item()) +"\n")
            
        plt.plot(loss_list)
        plt.title("Loss of PPO Agent")
        plt.savefig("/home/morin/catkin_ws/src/hiprl_replicate/Visualization/PPO/loss.png")
        plt.close('all')

        with open("/home/morin/catkin_ws/src/hiprl_replicate/Visualization/PPO/reward.txt", "w") as f:
            for s in reward_list:
                f.write(str(s) +"\n")

        plt.plot(reward_list)
        plt.title("Reward of PPO Agent")
        plt.savefig("/home/morin/catkin_ws/src/hiprl_replicate/Visualization/PPO/reward.png")
        plt.close('all')
        
        if n_epi%print_interval==0:
            print("# of episode :{}, avg score : {:.1f}".format(n_epi, score/print_interval))
            score = 0.0
            torch.save(model.state_dict(), "/home/morin/catkin_ws/src/hiprl_replicate/learned_agent/agent.txt")
            #print("Visualizing and evaluating the current policy")
            
            #s = env.reset()
            #for step in range(T_horizon):
            #    time.sleep(0.1)
            #    input = torch.from_numpy(s).float()
            #    input = input.unsqueeze(0)
            #    input = input.unsqueeze(0)
            #    
            #    prob = model.pi(input.to(device))
            #    m = Categorical(prob)
            #    a = m.sample().item()
            #    print("selected meta-action: {}".format(a))
            #    obs_prime, r, done, info, s_prime = env.invoke(a)
            #    print('step=%s, reward=%.2f' % (env.step_count, r))

#                if done:
#                    print('done!')
#                else:
#                    img = env.render('rgb_array', tile_size=32)
#                    window.show_img(img)
        
#            window.close()
            
    env.close()
    
if __name__ == '__main__':
    main()