import pandas as pd
import plotly.express as px
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.ticker as ticker
mpl.rcParams['toolbar'] = 'None'
df = pd.read_csv('~/Downloads/multi_data/iehiprl_loss.csv')
df2 = pd.read_csv('~/Downloads/multi_data/hiprl_loss.csv')

df4 = pd.read_csv('~/Downloads/multi_data/iehiprl_r_training.csv')
df5 = pd.read_csv('~/Downloads/multi_data/hiprl_r_training.csv')


df['EWM'] = df['Value'].ewm(alpha=0.1, adjust=False).mean()
df2['EWM'] = df2['Value'].ewm(alpha=0.1, adjust=False).mean()
df4['EWM'] = df4['Value'].ewm(alpha=0.1, adjust=False).mean()
df5['EWM'] = df5['Value'].ewm(alpha=0.1, adjust=False).mean()

fig = plt.figure()

ax1 = fig.add_subplot(1,1,1)
#ax2 = fig.add_subplot(1,1,1)
ax1.plot(df2['Step'], df2['EWM'], label = "extended HiP-RL", color='red')
ax1.plot(df['Step'], df['EWM'], label = "IE-HiP-RL (proposed)", color='blue')
ax1.set_title('Loss')
ax1.grid()
handles, labels = ax1.get_legend_handles_labels()
ax1.xaxis.set_major_formatter(ticker.EngFormatter())


#ax2.plot(df5['Step'], df5['EWM'], label = "extended HiP-RL", color='red')
#ax2.plot(df4['Step'], df4['EWM'], label = "IE-HiP-RL (proposed)", color='blue')
#ax2.set_title('Episode Reward')
#ax2.grid()
#handles, labels = ax2.get_legend_handles_labels()
#ax2.xaxis.set_major_formatter(ticker.EngFormatter())

box = ax1.get_position()
ax1.set_position([box.x0, box.y0 + box.height * 0.2,
                 box.width, box.height * 0.8])
#box = ax2.get_position()
#ax2.set_position([box.x0, box.y0 + box.height * 0.2,
#                 box.width, box.height * 0.8])



fig.legend(loc='lower center', bbox_to_anchor=(0.5, 0.0), ncol=3)
fig.text(0.5, 0.15, '# of option selections', ha='center')
plt.show()
