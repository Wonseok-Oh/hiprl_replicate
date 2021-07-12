from gym_minigrid.wrappers import *
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Categorical
import torch.multiprocessing as mp
import time

# Hyperparameters
n_train_processes = 3
learning_rate = 0.0002
update_interval = 5
gamma = 0.98
max_train_ep = 300
max_test_ep = 400
#device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
device = torch.device("cpu")

class ActorCritic(nn.Module):
    def __init__(self):
        super(ActorCritic, self).__init__()
        # Suppose input width, height is W, H respectively FH = Kernel(Filter) size = K
        # Ouput Height for CNN = (H + 2P - FH)/S + 1
        # size calculation referenced http://taewan.kim/post/cnn/
        self.obj_conv1 = nn.Conv2d(1, 32, 7, stride = 2, padding = 1)
        self.obj_conv2 = nn.Conv2d(32, 32, 5, stride = 1, padding = 1)
        self.obj_conv3 = nn.Conv2d(32, 32, 3, stride = 1, padding = 1)
        
        self.fc1 = nn.Linear(3872, 512)
        #self.gru = nn.GRU(5, hidden_size, 1)
        self.fc_pi = nn.Linear(512, 3)
        self.fc_v = nn.Linear(512, 1)

    def pi(self, input, softmax_dim=1):
        # forward propagation object_map_input
        x = F.elu(self.obj_conv1(input))
        x = F.elu(self.obj_conv2(x))
        x = F.elu(self.obj_conv3(x))
        x = torch.flatten(x, 1)
        
        # forward propagation observation input
        #y = F.elu(self.obs_conv1(observation_input))
        #y = F.elu(self.obs_conv2(y))
        #y = F.elu(self.obs_conv3(y))
        #y = torch.flatten(y, 1)
        #y = F.elu(self.gru(y))

        # Concatenate 
        #z = torch.cat((x,y), dim=1)
        x = F.elu(self.fc1(x))
        x = self.fc_pi(x)
        prob = F.softmax(x, dim=softmax_dim)
        return prob

    def v(self, input):
        # forward propagation object_map_input
        x = F.elu(self.obj_conv1(input))
        x = F.elu(self.obj_conv2(x))
        x = F.elu(self.obj_conv3(x))
        x = torch.flatten(x, 1)
        
        # forward propagation observation input
        #y = F.elu(self.obs_conv1(observation_input))
        #y = F.elu(self.obs_conv2(y))
        #y = F.elu(self.obs_conv3(y))
        #y = torch.flatten(y, 1)
        #y = F.elu(self.gru(y))

        # Concatenate 
        #z = torch.cat((x,y), dim=1)
        x = F.elu(self.fc1(x))
        x = self.fc_v(x)
        return x


def train(global_model, rank):
    local_model = ActorCritic().to(device)
    local_model.load_state_dict(global_model.state_dict())

    optimizer = optim.Adam(global_model.parameters(), lr=learning_rate)

    env = gym.make('MiniGrid-HiPRLGrid-v0')

    for n_epi in range(max_train_ep):
        done = False
        map = env.reset()
        while not done:
            maps_lst, a_lst, r_lst = [], [], []
            for t in range(update_interval):
                input = torch.from_numpy(map).float()
                input = input.unsqueeze(0)
                input = input.unsqueeze(0)
                #print(input.size())
                prob = local_model.pi(input.to(device))
                m = Categorical(prob)
                a = m.sample().item()
                obs_prime, r, done, info, map_prime = env.invoke(a)

                maps_lst.append(map)
                a_lst.append([a])
                r_lst.append(r/100.0)

                map = map_prime
                if done:
                    break

            map_final = torch.tensor(map_prime, dtype=torch.float)
            map_final = map_final.unsqueeze(0)
            map_final = map_final.unsqueeze(0).to(device)
            print(map_final.size())
            R = 0.0 if done else local_model.v(map_final).item()
            td_target_lst = []
            for reward in r_lst[::-1]:
                R = gamma * R + reward
                td_target_lst.append([R])
            td_target_lst.reverse()
            
            s_batch, a_batch, td_target = torch.tensor(maps_lst, dtype=torch.float).to(device), torch.tensor(a_lst).to(device), \
                torch.tensor(td_target_lst).to(device)
                
            s_batch = s_batch.unsqueeze(1).to(device)
            
            advantage = td_target - local_model.v(s_batch)

            pi = local_model.pi(s_batch, softmax_dim=1)
            pi_a = pi.gather(1, a_batch)
            loss = -torch.log(pi_a) * advantage.detach() + \
                F.smooth_l1_loss(local_model.v(s_batch), td_target.detach())

            optimizer.zero_grad()
            loss.mean().backward()
            for global_param, local_param in zip(global_model.parameters(), local_model.parameters()):
                global_param._grad = local_param.grad
            optimizer.step()
            local_model.load_state_dict(global_model.state_dict())

    env.close()
    print("Training process {} reached maximum episode.".format(rank))


def test(global_model):
    env = gym.make('MiniGrid-HiPRLGrid-v0')
    score = 0.0
    print_interval = 20

    for n_epi in range(max_test_ep):
        done = False
        s = env.reset()
        while not done:
            input = torch.from_numpy(s).float()
            input = input.unsqueeze(0)
            input = input.unsqueeze(0)
            #print(input.size())
            prob = global_model.pi(input.to(device))
            a = Categorical(prob).sample().item()
            dummy_obs, r, done, info, s_prime = env.invoke(a)
            s = s_prime
            score += r

        if n_epi % print_interval == 0 and n_epi != 0:
            print("# of episode :{}, avg score : {:.1f}".format(
                n_epi, score/print_interval))
            score = 0.0
            time.sleep(1)
    env.close()


if __name__ == '__main__':
    print(device)
    global_model = ActorCritic().to(device)
    global_model.share_memory()

    processes = []
    for rank in range(n_train_processes + 1):  # + 1 for test process
        if rank == 0:
            p = mp.Process(target=test, args=(global_model,))
        else:
            p = mp.Process(target=train, args=(global_model, rank,))
        p.start()
        processes.append(p)
    for p in processes:
        p.join()