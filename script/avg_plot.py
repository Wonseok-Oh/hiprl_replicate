import numpy as np
import matplotlib.pyplot as plt

def main():
    loss = []
    reward = []
    #with open("/home/morin/catkin_ws/src/hiprl_replicate/Visualization/PPO/loss.txt", "r") as f:
    #  for line in f:
    #    loss.append(float(line.strip()))
    
    with open("/home/morin/catkin_ws/src/hiprl_replicate/Visualization/PPO/reward.txt", "r") as f:
      for line in f:
        reward.append(float(line.strip()))
    N = 1000
    #avg_loss = np.convolve(loss, np.ones(N)/N, mode='valid')
    #plt.plot(avg_loss)
    #plt.title("Avg window size 10-Loss of PPO Agent")
    #plt.savefig("/home/morin/catkin_ws/src/hiprl_replicate/Visualization/PPO/avg_loss.png")
    #plt.close('all')

    avg_reward = np.convolve(reward, np.ones(N)/N, mode='valid')
    plt.plot(avg_reward)
    plt.title("Avg window size 1000-Reward of PPO Agent")
    plt.savefig("/home/morin/catkin_ws/src/hiprl_replicate/Visualization/PPO/avg_reward.png")
    plt.close('all')


if __name__ == '__main__':
    main()

