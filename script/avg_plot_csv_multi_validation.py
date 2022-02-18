import pandas as pd
import plotly.express as px
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.ticker as ticker
mpl.rcParams['toolbar'] = 'None'
df = pd.read_csv('./Validation_Curve_HiPRL_multi.csv')
df2 = pd.read_csv('./Validation_Curve_IEHiPRL_multi.csv')
  


fig = plt.figure()

#ax1 = fig.add_subplot(1,1,1)
#ax2 = fig.add_subplot(1,1,1)
ax3 = fig.add_subplot(1,1,1)
#ax1.plot(df['Step'], df['Success Rate'], label = "extended HiP-RL", color='red')
#ax1.plot(df2['Step'], df2['Success Rate'], label = "IE-HiP-RL (proposed)", color='blue')
#ax1.set_title('Success Rate')
#ax1.grid()
#handles, labels = ax1.get_legend_handles_labels()
#ax1.xaxis.set_major_formatter(ticker.EngFormatter())


#ax2.plot(df['Step'], df['Episode Reward'], label = "extended HiP-RL", color='red')
#ax2.plot(df2['Step'], df2['Episode Reward'], label = "IE-HiP-RL (proposed)", color='blue')
#ax2.set_title('Episode Reward')
#ax2.grid()
#handles, labels = ax2.get_legend_handles_labels()
#ax2.xaxis.set_major_formatter(ticker.EngFormatter())

ax3.plot(df['Step'], df['Makespan'], label = "extended HiP-RL", color='red')
ax3.plot(df2['Step'], df2['Makespan'], label = "IE-HiP-RL (proposed)", color='blue')
ax3.set_title('Makespan')
ax3.set( ylabel='timesteps')
ax3.grid()
handles, labels = ax3.get_legend_handles_labels()
ax3.xaxis.set_major_formatter(ticker.EngFormatter())

#box = ax1.get_position()
#ax1.set_position([box.x0, box.y0 + box.height * 0.2,
#                 box.width, box.height * 0.8])
#box = ax2.get_position()
#ax2.set_position([box.x0, box.y0 + box.height * 0.2,
#                 box.width, box.height * 0.8])
box = ax3.get_position()
ax3.set_position([box.x0, box.y0 + box.height * 0.2,
                 box.width, box.height * 0.8])



fig.legend(loc='lower center', bbox_to_anchor=(0.5, 0.0), ncol=3)
fig.text(0.5, 0.15, '# of option selections', ha='center')
plt.show()
