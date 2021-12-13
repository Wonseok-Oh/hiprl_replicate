import dgl
import numpy as np
import torch.nn as nn
import torch
import torch.optim as opt
torch.set_printoptionis(linewidth=120)
import torch.nn.functional as F
import torchvision
import torchvision.transforms as transforms
from torch.utils.tensorboard import SummaryWriter

import math
import random
import matplotlib.pyplot as plt
from IPython.display import clear_output
import gym
import gym_minigrid
import csv

from collections import namedtuple

device = ("cuda" if torch.cuda.is_available() else cpu)

class MLP(nn.Module):
    # in hw file, act function was LeakyReLU
    def __init__(self, in_dim, out_dim, hidden_dims=[64], hidden_act=nn.Tanh(), out_act=nn.Tanh()):
        super(MLP, self).__init__()
        self.layers = nn.ModuleList()
        self.activations = nn.ModuleList()
        
        in_dims = [in_dim] + hidden_dims
        out_dims = hidden_dims + [out_dim]
        
        for _in, _out in zip(in_dims, out_dims):
            self.layers.append(nn.Linear(_in, _out))
        
        for i in range(len(hidden_dims)):
            self.activations.append(hidden_act)
        self.activations.append(out_act)
        
    def forward(self, x):
        for l, a in zip(self.layers, self.activations):
            x = l(x)
            x = a(x)
            
        return x    

class MultiHeadAttnLayer(nn.Module):
    def __init__(self, model_dim, num_heads=2, merge='cat'):
        super(MultiHeadAttnLayer, self).__init__()

        self.K = nn.ModuleList()
        self.Q = nn.ModuleList()
        self.V = nn.ModuleList()
        
        self.num_heads = num_heads
        self.merge = merge

        for i in range(num_heads):
            self.K.append(nn.Linear(model_dim, model_dim))
            self.Q.append(nn.Linear(model_dim, model_dim))
            self.V.append(nn.Linear(model_dim, model_dim))

        if merge == 'cat':
            self.O = nn.Linear(model_dim * num_heads, model_dim)
        else:
            # merge using 'mean'
            self.O = nn.Linear(model_dim, model_dim)


    def forward(self, graph, node_feature):
        graph.ndata['nf'] = node_feature
        graph.update_all(self.message_func, self.reduce_func, self.apply_nodes)

        out_nf = graph.ndata.pop('out_nf')
        _ = graph.ndata.pop('nf')
        weight = graph.ndata.pop('weight')

        return out_nf, weight

    def message_func(self, edges):
        # (i,j)
        src_nf = edges.src['nf'] # h_i, shape = [batch , model_dim]
        dst_nf = edges.dst['nf'] # h_j, shape = [batch , model_dim]

        ret_dict = dict()

        for h in range(self.num_heads):
            # computing score
            k = self.K[h](src_nf)  # shape = [batch , model_dim]
            q = self.Q[h](dst_nf)  # shape = [batch , model_dim]
            score = (k * q).sum(-1) # summation over model dimension
            score = score.reshape(-1, 1) # shape = [batch , 1]
            ret_dict['score_{}'.format(h)] = score
            
            # computing value
            ret_dict['value_{}'.format(h)] = self.V[h](src_nf) # shape = [batch, model_dim]

        return ret_dict

    def reduce_func(self, nodes):

        ret_dict = dict()

        for h in range(self.num_heads):
            value = nodes.mailbox['value_{}'.format(h)]  # shape = [batch , num_edges , model_dim]
            score = nodes.mailbox['score_{}'.format(h)]  # shape = [batch , num_edges , 1]
            
            weight = torch.softmax(score, dim=1)  # shape = [batch , num_edges , 1]

            weighted_value_sum = (weight * value).sum(1)  # summation over edge dimension
            ret_dict['wv_{}'.format(h)] = weighted_value_sum
            
            ret_dict['w_{}'.format(h)] = weight.squeeze()

        return ret_dict

    def apply_nodes(self, nodes):
        o_input = []
        weights = []
        for h in range(self.num_heads):
            o_input.append(nodes.data['wv_{}'.format(h)])  # shape = [# nodes , model_dim]
            weights.append(nodes.data['w_{}'.format(h)])

        if self.merge == 'cat':
            o_input = torch.cat(o_input, dim=-1)
        else:
            o_input = torch.stack(o_input)  # shape = [# heads , # nodes , model_dim]
            o_input = o_input.mean(0)  # mean over attn head dimension
            
        weights = torch.stack(weights)
        weights = weights.mean(0)

        o = self.O(o_input)
        return {'out_nf': o, 'weight':weights}

class MultiHeadAttnLayer(nn.Module):
    def __init__(self, model_dim, num_heads=2, merge='cat'):
        super(MultiHeadAttnLayer, self).__init__()

        self.K = nn.ModuleList()
        self.Q = nn.ModuleList()
        self.V = nn.ModuleList()
        
        self.num_heads = num_heads
        self.merge = merge

        for i in range(num_heads):
            self.K.append(nn.Linear(model_dim, model_dim))
            self.Q.append(nn.Linear(model_dim, model_dim))
            self.V.append(nn.Linear(model_dim, model_dim))

        if merge == 'cat':
            self.O = nn.Linear(model_dim * num_heads, model_dim)
        else:
            # merge using 'mean'
            self.O = nn.Linear(model_dim, model_dim)


    def forward(self, graph, node_feature):
        graph.ndata['nf'] = node_feature
        graph.update_all(self.message_func, self.reduce_func, self.apply_nodes)

        out_nf = graph.ndata.pop('out_nf')
        _ = graph.ndata.pop('nf')
        weight = graph.ndata.pop('weight')

        return out_nf, weight

    def message_func(self, edges):
        # (i,j)
        src_nf = edges.src['nf'] # h_i, shape = [batch , model_dim]
        dst_nf = edges.dst['nf'] # h_j, shape = [batch , model_dim]

        ret_dict = dict()

        for h in range(self.num_heads):
            # computing score
            k = self.K[h](src_nf)  # shape = [batch , model_dim]
            q = self.Q[h](dst_nf)  # shape = [batch , model_dim]
            score = (k * q).sum(-1) # summation over model dimension
            score = score.reshape(-1, 1) # shape = [batch , 1]
            ret_dict['score_{}'.format(h)] = score
            
            # computing value
            ret_dict['value_{}'.format(h)] = self.V[h](src_nf) # shape = [batch, model_dim]

        return ret_dict

    def reduce_func(self, nodes):

        ret_dict = dict()

        for h in range(self.num_heads):
            value = nodes.mailbox['value_{}'.format(h)]  # shape = [batch , num_edges , model_dim]
            score = nodes.mailbox['score_{}'.format(h)]  # shape = [batch , num_edges , 1]
            
            weight = torch.softmax(score, dim=1)  # shape = [batch , num_edges , 1]

            weighted_value_sum = (weight * value).sum(1)  # summation over edge dimension
            ret_dict['wv_{}'.format(h)] = weighted_value_sum
            
            ret_dict['w_{}'.format(h)] = weight.squeeze()

        return ret_dict

    def apply_nodes(self, nodes):
        o_input = []
        weights = []
        for h in range(self.num_heads):
            o_input.append(nodes.data['wv_{}'.format(h)])  # shape = [# nodes , model_dim]
            weights.append(nodes.data['w_{}'.format(h)])

        if self.merge == 'cat':
            o_input = torch.cat(o_input, dim=-1)
        else:
            o_input = torch.stack(o_input)  # shape = [# heads , # nodes , model_dim]
            o_input = o_input.mean(0)  # mean over attn head dimension
            
        weights = torch.stack(weights)
        weights = weights.mean(0)

        o = self.O(o_input)
        return {'out_nf': o, 'weight':weights}

def generate_graph_with_obs(obs):
    g = dgl.DGLGraph()
    n_agents = len(obs)
    g.add_nodes(n_agents)
    
    from_idx, to_idx = get_fully_connected_edges(n_agents, self_edge=True)
    g.add_edges(from_idx, to_idx)
    
    # we save observation as the feature of the nodes
    g.ndata['obs'] = torch.Tensor(obs) # shape = (n_agents x state_dim)

    return g

if __name__ == "__main__":
    env = gym.make('MiniGrid-MultiHiPRLGrid-v0')
    
    n_episodes = 10000
    
    n_agents = len(env.agents)
    state_dim = env.observation_space[0].shape[0]
    n_meta_actions = env.meta_action_space.n
    
    # agent hyperparameters
    hidden_dim = 20
    gamma = 0.99
    memory_size = 1000
    batch_size = 64
    epsilon_start = 0.99
    epsilon_decay = 0.9999
    epsilon_min = 0.01
    update_target = 50
    lamda = 0.0
    agent = Agent(n_agents, state_dim, n_actions, 
                  hidden_dim=hidden_dim,
                  gamma=gamma, 
                  memory_size=memory_size,
                  epsilon_start=epsilon_start,
                  epsilon_decay=epsilon_decay,
                  epsilon_min=epsilon_min,
                  update_target=update_target,
                  lamda = lamda
                 )
    
    # environment parameter
    max_timestep = 100
    
    # logging
    ep_reward_traj = []
    loss_traj = []
    epsilon_traj = []
    makespan_traj = []
    tb = SummaryWriter()
    for e in range(n_episodes):
        t = 0
        env.reset()
        terminated = False
        episode_reward = 0
        
        state = env.reset()
        graph = generate_graph_with_obs(state)
        
        while True:
            t += 1
            meta_action, env_meta_action = agent.get_meta_action(graph)
            # Make env_meta_action only available for agents whose action list is empty
            next_state, reward, terminated, info = env.invoke(env_meta_action)
            next_graph = generate_graph_with_obs(next_state)
            
            normalized_reward = [r for r in reward]
            
            agent.save_samples(graph, meta_action, normalized_reward, next_graph, terminated)
            
            trained, loss = agent.fit()
            
            if trained:
                loss_traj.append(loss)
                
            episode_reward += sum(normalized_reward)
            
            if all(terminated) or t >= max_timestep:
                ep_reward_traj.apend(episode_reward)
                epsilon_traj.append(agent.epsilon)
                
                plt.plot(np.convolve(loss_traj, np.ones(100)/100, mode='valid'))
                plt.title('loss')
                plt.savefig("./images/loss_avg.png")
                plt.close('all')
            
                '''plt.show()'''
                
                plt.plot(epsilon_traj)
                plt.title('epsilon')
                plt.savefig("./images/epsilon_avg.png")
                plt.close('all')
                '''    plt.show()'''
                
                plt.plot(np.convolve(ep_reward_traj, np.ones(100)/100, mode='valid'))
                plt.title('reward')
                plt.savefig("./images/reward_avg.png")
            
                print("EP:{}, SUM_RWD:{:5.2f}, epsilon:{:5.2f}, loss:{:5.2f}".format(e, episode_reward, agent.epsilon, loss))
                break
            
        tb.add_scalar("EP Loss", sum(loss), e)
        tb.add_scalar("EP Reward", episode_reward, e)
        tb.add_scalar("Epsilon", agent.epsilon, e)
    env.close()
    
    with open('logging.csv', 'w', newline='') as f:
        write = csv.writer(f)
        write.writerow(loss_traj)
        write.writerow(ep_reward_traj)
        write.writerow(epsilon_traj)
    
    
            