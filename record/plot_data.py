import matplotlib.pyplot as plt
import numpy as np
import os

log_file = os.path.abspath(os.path.join(os.path.dirname(__file__), '../install/record/datalog.log'))

data = np.loadtxt(log_file, delimiter=',')

t = data[:, 0]
r = data[:, 1]
pos = data[:, 2:13]
vel = data[:, 13:24]
tor = data[:, 24:35]
cmd = data[:, 35:46]

fig1, axes = plt.subplots(2, 2)

axes[0][0].set_title('position')
axes[0][1].set_title('velocity')
axes[1][0].set_title('torque')
axes[1][1].set_title('command')

for i in range(2):
    for j in range(2):
        axes[i][j].grid(True)

for i in range(11):
    axes[0][0].plot(t, pos[:, i], label=f'motor_{i + 1}')
    axes[0][1].plot(t, vel[:, i])
    axes[1][0].plot(t, tor[:, i])
    axes[1][1].plot(t, cmd[:, i])

handles, labels = axes[0][0].get_legend_handles_labels()
fig1.legend(handles, labels)

fig2, ax = plt.subplots()
ax.plot(t, r)
ax.set_title('frequency')

plt.show()
