import pandas as pd
import plotly.express as px
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.ticker as ticker
mpl.rcParams['toolbar'] = 'None'
#df = pd.read_csv('~/Downloads/single_data/iehiprl_sr.csv')
df2 = pd.read_csv('~/Downloads/single_data/hiprl_sr.csv')
df3 = pd.read_csv('~/Downloads/single_data/iehiprl_sr.csv')

#df4 = pd.read_csv('~/Downloads/test_stable_baseline3_mine_no_pi_1_mr.csv')
df5 = pd.read_csv('~/Downloads/single_data/hiprl_r.csv')
df6 = pd.read_csv('~/Downloads/single_data/iehiprl_r.csv')

#df7 = pd.read_csv('~/Downloads/test_stable_baseline3_mine_no_pi_1.csv')
df8 = pd.read_csv('~/Downloads/single_data/hiprl_makespan.csv')
df9 = pd.read_csv('~/Downloads/single_data/iehiprl_makespan.csv')


#df['EWM'] = df['Value'].ewm(alpha=0.1, adjust=False).mean()
df2['EWM'] = df2['Value'].ewm(alpha=0.1, adjust=False).mean()
df3['EWM'] = df3['Value'].ewm(alpha=0.1, adjust=False).mean()
#df4['EWM'] = df4['Value'].ewm(alpha=0.1, adjust=False).mean()
df5['EWM'] = df5['Value'].ewm(alpha=0.1, adjust=False).mean()
df6['EWM'] = df6['Value'].ewm(alpha=0.1, adjust=False).mean()
#df7['EWM'] = df7['Value'].ewm(alpha=0.1, adjust=False).mean()
df8['EWM'] = df8['Value'].ewm(alpha=0.1, adjust=False).mean()
df9['EWM'] = df9['Value'].ewm(alpha=0.1, adjust=False).mean()

fig = plt.figure()

#ax1 = fig.add_subplot(1,1,1)
#ax2 = fig.add_subplot(1,1,1)
ax3 = fig.add_subplot(1,1,1)
#ax1.plot(df2['Step'], df2['EWM'], label = "HiP-RL", color='black')
#ax1.plot(df3['Step'], df3['EWM'], label = "IE-HiP-RL (proposed)", color='blue')
#ax1.axhline(y=0.8, xmin = 0, xmax = 100000, c="red", linestyle='dashed', linewidth=2.0, zorder = 0)
#ax1.set_title('Success Rate')
#ax1.grid()
#ax1.xaxis.set_major_formatter(ticker.EngFormatter())


#ax2.plot(df5['Step'], df5['EWM'], label = "HiP-RL", color='black')
#ax2.plot(df6['Step'], df6['EWM'], label = "IE-HiP-RL (proposed)", color='blue')
#ax2.axhline(y=0.292, xmin = 0, xmax = 100000, c="red", linestyle='dashed', linewidth=2.0, zorder = 0)

#ax2.set_title('Episode Reward')
#ax2.grid()
#handles, labels = ax2.get_legend_handles_labels()
#labels.append("heuristic")

#ax2.xaxis.set_major_formatter(ticker.EngFormatter())

ax3.plot(df8['Step'], df8['EWM'], label = "HiP-RL", color='black')
ax3.plot(df9['Step'], df9['EWM'], label = "IE-HiP-RL (proposed)",color='blue')
ax3.axhline(y=22.34, xmin = 0, xmax = 100000, c="red", linestyle='dashed', linewidth=2.0, zorder = 0)

ax3.set_title('Makespan')
ax3.set( ylabel='timesteps')
ax3.grid()
handles, labels = ax3.get_legend_handles_labels()
labels.append("heuristic")

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



fig.legend(loc='lower center', bbox_to_anchor=(0.5, 0.0), ncol=3, labels = labels)
fig.text(0.5, 0.15, '# of option selections', ha='center')
plt.show()
#fig = px.line(df, x = 'Step', y = 'EWM', title='success_rate')
#fig.add_line(df, x='Step', y = 'Value')
#fig.show()
