import pandas as pd
import plotly.express as px
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.ticker as ticker
mpl.rcParams['toolbar'] = 'None'
df = pd.read_csv('~/Downloads/test_stable_baseline3_mine_no_pi_1_sr.csv')
df2 = pd.read_csv('~/Downloads/test_stable_baseline3_4_sr.csv')
df3 = pd.read_csv('~/Downloads/test_stable_baseline3_mine_3_sr.csv')

df4 = pd.read_csv('~/Downloads/test_stable_baseline3_mine_no_pi_1_mr.csv')
df5 = pd.read_csv('~/Downloads/test_stable_baseline3_4_mr.csv')
df6 = pd.read_csv('~/Downloads/test_stable_baseline3_mine_3_mr.csv')

df7 = pd.read_csv('~/Downloads/test_stable_baseline3_mine_no_pi_1.csv')
df8 = pd.read_csv('~/Downloads/test_stable_baseline3_4.csv')
df9 = pd.read_csv('~/Downloads/test_stable_baseline3_mine_3.csv')


df['EWM'] = df['Value'].ewm(alpha=0.1, adjust=False).mean()
df2['EWM'] = df2['Value'].ewm(alpha=0.1, adjust=False).mean()
df3['EWM'] = df3['Value'].ewm(alpha=0.1, adjust=False).mean()
df4['EWM'] = df4['Value'].ewm(alpha=0.1, adjust=False).mean()
df5['EWM'] = df5['Value'].ewm(alpha=0.1, adjust=False).mean()
df6['EWM'] = df6['Value'].ewm(alpha=0.1, adjust=False).mean()
df7['EWM'] = df7['Value'].ewm(alpha=0.1, adjust=False).mean()
df8['EWM'] = df8['Value'].ewm(alpha=0.1, adjust=False).mean()
df9['EWM'] = df9['Value'].ewm(alpha=0.1, adjust=False).mean()

fig = plt.figure()

ax3 = fig.add_subplot(1,1,1)
#ax2 = fig.add_subplot(1,3,2)
#ax3 = fig.add_subplot(1,3,3)
'''
ax1.plot(df['Step'], df['EWM'], label = "IE-HiP-RL w/o P.I.", color='black')
ax1.plot(df['Step'], df2['EWM'], label = "HiP-RL", color='red')
ax1.plot(df['Step'], df3['EWM'], label = "IE-HiP-RL (proposed)", color='blue')
ax1.set_title('Success Rate')
#ax1.set(xlabel='# of option selections')
ax1.grid()
handles, labels = ax1.get_legend_handles_labels()
ax1.xaxis.set_major_formatter(ticker.EngFormatter())


ax2.plot(df['Step'], df4['EWM'], label = "IE-HiP-RL w/o P.I.", color='black')
ax2.plot(df['Step'], df5['EWM'], label = "HiP-RL", color='red')
ax2.plot(df['Step'], df6['EWM'], label = "IE-HiP-RL (proposed)", color='blue')
ax2.set_title('Episode Reward')
#ax2.set(xlabel='# of option selections')
ax2.grid()
handles, labels = ax2.get_legend_handles_labels()
ax2.xaxis.set_major_formatter(ticker.EngFormatter())
'''

ax3.plot(df['Step'], df7['EWM'], label = "IE-HiP-RL w/o P.I.", color='black')
ax3.plot(df['Step'], df8['EWM'], label = "HiP-RL", color='red')
ax3.plot(df['Step'], df9['EWM'], label = "IE-HiP-RL (proposed)", color='blue')
ax3.set_title('Makespan')
ax3.set( ylabel='timesteps')
ax3.grid()
ax3.xaxis.set_major_formatter(ticker.EngFormatter())
'''
box = ax1.get_position()
ax1.set_position([box.x0, box.y0 + box.height * 0.2,
                 box.width, box.height * 0.8])
box = ax2.get_position()
ax2.set_position([box.x0, box.y0 + box.height * 0.2,
                 box.width, box.height * 0.8])
box = ax3.get_position()
ax3.set_position([box.x0, box.y0 + box.height * 0.2,
                 box.width, box.height * 0.8])
'''

box = ax3.get_position()
ax3.set_position([box.x0, box.y0 + box.height * 0.2,
                 box.width, box.height * 0.8])
fig.legend(loc='lower center', bbox_to_anchor=(0.5, 0.0), ncol=3)
fig.text(0.5, 0.15, '# of option selections', ha='center')
plt.show()
#fig = px.line(df, x = 'Step', y = 'EWM', title='success_rate')
#fig.add_line(df, x='Step', y = 'Value')
#fig.show()
