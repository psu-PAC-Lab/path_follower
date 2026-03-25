import numpy as np
import matplotlib.pyplot as plt
import sys

# get file name
filename = sys.argv[1]

# read data
t = np.array([])
x = np.array([])
y = np.array([])
v = np.array([])
th = np.array([])
om = np.array([])
x_r = np.array([])
y_r = np.array([])
v_r = np.array([])
th_r = np.array([])
om_r = np.array([])
v_cmd = np.array([])
th_cmd = np.array([])
om_cmd = np.array([])
# x_path_plan = np.array([])
# y_path_plan = np.array([])
# v_path_plan = np.array([])
# th_path_plan = np.array([])
# om_path_plan = np.array([])


with open(filename, 'r') as f:
    lines = f.readlines()
    for line in lines:
        data = line.split(' ')
        t = np.append(t, float(data[0]))
        x = np.append(x, float(data[1]))
        y = np.append(y, float(data[2]))
        v = np.append(v, float(data[3]))
        th = np.append(th, float(data[4]))
        om = np.append(om, float(data[5]))
        x_r = np.append(x_r, float(data[6]))
        y_r = np.append(y_r, float(data[7]))
        v_r = np.append(v_r, float(data[8]))
        th_r = np.append(th_r, float(data[9]))
        om_r = np.append(om_r, float(data[10]))
        v_cmd = np.append(v_cmd, float(data[11]))
        th_cmd = np.append(th_cmd, float(data[12]))
        om_cmd = np.append(om_cmd, float(data[13]))
        # x_path_plan = np.append(x_path_plan, float(data[14]))
        # y_path_plan = np.append(y_path_plan, float(data[15]))
        # v_path_plan = np.append(v_path_plan, float(data[16]))
        # th_path_plan = np.append(th_path_plan, float(data[17]))
        # om_path_plan = np.append(om_path_plan, float(data[18]))

# zero time
t = t - t[0]

# plot
fig, axs = plt.subplots(3,1)
        
axs[0].plot(t, v_r, '-g')
axs[0].plot(t, v_cmd, '--r')
axs[0].plot(t, v, '-k')
# axs[0].plot(t, v_path_plan, '-b')
axs[0].set_ylabel('v [m/s]')
axs[0].set_title('Velocity')
axs[0].legend(['ref', 'cmd', 'actual', 'path plan'])

axs[1].plot(t, th_r*(180.0/np.pi), '-g')
axs[1].plot(t, th_cmd*(180.0/np.pi), '--r')
axs[1].plot(t, th*(180.0/np.pi), '-k')
# axs[1].plot(t, th_path_plan*(180.0/np.pi), '-b')
axs[1].set_ylabel('theta [deg]')
axs[1].set_title('Heading')

axs[2].plot(t, om_r*(180.0/np.pi), '-g')
axs[2].plot(t, om_cmd*(180.0/np.pi), '--r')
axs[2].plot(t, om*(180.0/np.pi), '-k')
# axs[2].plot(t, om_path_plan*(180.0/np.pi), '-b')
axs[2].set_ylabel('omega [deg/s]')
axs[2].set_title('Turn Rate')
axs[2].set_xlabel('Time [s]')

# plot position states
fig, axs = plt.subplots(2,1)
axs[0].plot(t, x_r, '-g')
axs[0].plot(t, x, '-k')
# axs[0].plot(t, x_path_plan, '-b')
axs[0].set_ylabel('X [m]')
axs[0].set_title('X Position')
axs[0].legend(['ref', 'actual', 'path plan'])
axs[1].plot(t, y_r, '-g')
axs[1].plot(t, y, '-k')
# axs[1].plot(t, y_path_plan, '-b')
axs[1].set_ylabel('Y [m]')
axs[1].set_title('Y Position')
axs[1].set_xlabel('Time [s]')


# plot position trajectory
fig, axs = plt.subplots(1,1)
axs.plot(x_r, y_r, '-g')
axs.plot(x, y, '-k')
# axs.plot(x_path_plan, y_path_plan, '-b')
axs.set_xlabel('X [m]')
axs.set_ylabel('Y [m]')
axs.set_title('Position Trajectory')
axs.set_aspect('equal', 'box')
axs.legend(['ref', 'actual', 'path plan'])


plt.show()
