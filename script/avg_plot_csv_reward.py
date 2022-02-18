import pandas as pd
import plotly.express as px
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.ticker as ticker
mpl.rcParams['toolbar'] = 'None'
#df = pd.read_csv('~/Downloads/single_data/iehiprl_sr.csv')
#df2 = pd.read_csv('~/Downloads/reward_study/hiprl_sr.csv')
df0_sr = pd.read_csv('~/Downloads/reward_study/pf_sr.csv')
df1_sr = pd.read_csv('~/Downloads/reward_study/pfball_sr.csv')
df2_sr = pd.read_csv('~/Downloads/reward_study/pfballbox_sr.csv')
df3_sr = pd.read_csv('~/Downloads/reward_study/pfballboxcov_sr.csv')
df4_sr = pd.read_csv('~/Downloads/single_data/iehiprl_sr.csv')

#df4 = pd.read_csv('~/Downloads/test_stable_baseline3_mine_no_pi_1_mr.csv')
#df5 = pd.read_csv('~/Downloads/single_data/hiprl_r.csv')
df0_t = pd.read_csv('~/Downloads/reward_study/pf_eplen.csv')
df1_t = pd.read_csv('~/Downloads/reward_study/pfball_eplen.csv')
df2_t = pd.read_csv('~/Downloads/reward_study/pfballbox_eplen.csv')
df3_t = pd.read_csv('~/Downloads/reward_study/pfballboxcov_eplen.csv')
df4_t = pd.read_csv('~/Downloads/single_data/iehiprl_makespan.csv')

#df7 = pd.read_csv('~/Downloads/test_stable_baseline3_mine_no_pi_1.csv')
#df8 = pd.read_csv('~/Downloads/reward_study/hiprl_makespan.csv')
#df9 = pd.read_csv('~/Downloads/single_data/iehiprl_makespan.csv')

df0_sr['EWM'] = df0_sr['Value'].ewm(alpha=0.4, adjust=False).mean()
df1_sr['EWM'] = df1_sr['Value'].ewm(alpha=0.4, adjust=False).mean()
df2_sr['EWM'] = df2_sr['Value'].ewm(alpha=0.4, adjust=False).mean()
df3_sr['EWM'] = df3_sr['Value'].ewm(alpha=0.4, adjust=False).mean()
df4_sr['EWM'] = df4_sr['Value'].ewm(alpha=0.4, adjust=False).mean()

df0_t['EWM'] = df0_t['Value'].ewm(alpha=0.4, adjust=False).mean()
df1_t['EWM'] = df1_t['Value'].ewm(alpha=0.4, adjust=False).mean()
df2_t['EWM'] = df2_t['Value'].ewm(alpha=0.4, adjust=False).mean()
df3_t['EWM'] = df3_t['Value'].ewm(alpha=0.4, adjust=False).mean()
df4_t['EWM'] = df4_t['Value'].ewm(alpha=0.4, adjust=False).mean()


fig = plt.figure()

ax1 = fig.add_subplot(1,2,1)
ax2 = fig.add_subplot(1,2,2)
#ax1.plot(df['Step'], df['EWM'], label = "IE-HiP-RL w/o P.I.", color='black')
ax1.plot(df0_sr['Step'][0:47], df0_sr['EWM'][0:47], label = "IE-HiP-RL (mis)", color='black')
ax1.plot(df1_sr['Step'][0:47], df1_sr['EWM'][0:47], label = "IE-HiP-RL (rsc)", color='gray')
ax1.plot(df2_sr['Step'][0:47], df2_sr['EWM'][0:47], label = "IE-HiP-RL (bd)", color='orange')
ax1.plot(df3_sr['Step'][0:47], df3_sr['EWM'][0:47], label = "IE-HiP-RL (cov)", color='red')
ax1.plot(df4_sr['Step'][0:47], df4_sr['EWM'][0:47], label = "IE-HiP-RL (proposed)", color='blue')
ax1.set_title('Success Rate')
#ax1.set(xlabel='# of option selections')
ax1.grid()
handles, labels = ax1.get_legend_handles_labels()
handles.append(ax1.axhline(y=0.8, xmin = 0, xmax = 100000, c="red", linestyle='dashed', linewidth=0.5, zorder = 0))
ax1.xaxis.set_major_formatter(ticker.EngFormatter())


#ax2.plot(df['Step'], df4['EWM'], color='black')
ax2.plot(df0_t['Step'][0:47], df0_t['EWM'][0:47], label = "IE-HiP-RL (mis)", color='black')
ax2.plot(df1_t['Step'][0:47], df1_t['EWM'][0:47], label = "IE-HiP-RL (rsc)", color='gray')
ax2.plot(df2_t['Step'][0:47], df2_t['EWM'][0:47], label = "IE-HiP-RL (bd)", color='orange')
ax2.plot(df3_t['Step'][0:47], df3_t['EWM'][0:47], label = "IE-HiP-RL (cov)", color='red')
ax2.plot(df4_t['Step'][0:47], df4_t['EWM'][0:47], label = "IE-HiP-RL (proposed)", color='blue')

ax2.set_title('Episode length')
#ax2.set(xlabel='# of option selections')
ax2.grid()
ax2.xaxis.set_major_formatter(ticker.EngFormatter())

#ax3.plot(df['Step'], df7['EWM'], color='black')
#ax3.plot(df8['Step'], df8['EWM'], color='black')
#ax3.plot(df9['Step'], df9['EWM'], color='blue')
#ax3.axhline(y=22.34, xmin = 0, xmax = 100000, c="red", linestyle='dashed', linewidth=2.0, zorder = 0)

#ax3.set_title('Makespan')
#ax3.set( ylabel='timesteps')
#ax3.grid()
#ax3.xaxis.set_major_formatter(ticker.EngFormatter())

box = ax1.get_position()
ax1.set_position([box.x0, box.y0 + box.height * 0.2,
                 box.width, box.height * 0.8])
box = ax2.get_position()
ax2.set_position([box.x0, box.y0 + box.height * 0.2,
                 box.width, box.height * 0.8])
#box = ax3.get_position()
#ax3.set_position([box.x0, box.y0 + box.height * 0.2,
#                 box.width, box.height * 0.8])



fig.legend(loc='lower center', bbox_to_anchor=(0.5, 0.0), ncol=3, labels = labels)
fig.text(0.5, 0.15, '# of option selections', ha='center')
plt.show()
#fig = px.line(df, x = 'Step', y = 'EWM', title='success_rate')
#fig.add_line(df, x='Step', y = 'Value')
#fig.show()
