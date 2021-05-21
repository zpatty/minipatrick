import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ani
import pandas as pd
import sys

plt.rcParams.update({'font.size': 22})

df = pd.read_csv('strain-output.csv')
df.columns = ['Strain']
print(len(df.index))
time = np.arange(0, 0.1*len(df.index), 0.1)
df['Time'] = time
df['Strain'] = df['Strain']#/(2**19)
df = df.loc[df['Time']>358]

color = ['red', 'green', 'blue', 'orange']
fig = plt.figure(figsize=(15,9))
plt.xticks(rotation=45, ha="right", rotation_mode="anchor") #rotate the x-axis values
plt.subplots_adjust(bottom = 0.2, top = 0.9) #ensuring the dates (on the x-axis) fit in the screen
plt.ylabel('Differential Capacitance (pF)')
plt.xlabel('Time (s)')

Writer = ani.writers['ffmpeg']
writer = Writer(fps=10, metadata=dict(artist='Me'), bitrate=1800)

def buildmebarchart(i=int):
    #plt.legend(df.columns)
    p = plt.plot(df[:i].Time, df[:i].Strain) #note it only returns the dataset, up to the point i
    # for i in range(0,4):
    #     p[i].set_color(color[i]) #set the colour of each curve

import matplotlib.animation as ani
animator = ani.FuncAnimation(fig, buildmebarchart, interval = 100, save_count=sys.maxsize)

animator.save('new_strain.mp4', writer=writer)
#plt.plot(df.Time, df.Strain)
#plt.show()