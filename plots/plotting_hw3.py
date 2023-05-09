#!/usr/bin/env python3

# read text files and plot them

import matplotlib.pyplot as plt
import numpy as np
import sys
import math

# data files to read
file1a = "../data_files/q1-a.txt"
file1c = "../data_files/q1-c.txt"
file2d = "../data_files/q2-d.txt"
file2e = "../data_files/q2-e.txt"
file2f = "../data_files/q2-f.txt"
file2g = "../data_files/q2-g.txt"
file3 = "../data_files/q3.txt"
file4a = "../data_files/q4-a.txt"
file4b = "../data_files/q4-b.txt"

traj1a_x = np.loadtxt(file1a, skiprows=0)[:, 8:11]
traj1c_x = np.loadtxt(file1c, skiprows=0)[:, 8:11]
traj2d_q = np.loadtxt(file2d, skiprows=0)[:, 1:8]
traj2d_x = np.loadtxt(file2d, skiprows=0)[:, 8:11]
traj2e_q = np.loadtxt(file2e, skiprows=0)[:, 1:8]
traj2e_x = np.loadtxt(file2e, skiprows=0)[:, 8:11]
traj2f_q = np.loadtxt(file2f, skiprows=0)[:, 1:8]
traj2f_x = np.loadtxt(file2f, skiprows=0)[:, 8:11]
traj2g_q = np.loadtxt(file2g, skiprows=0)[:, 1:8]
traj2g_x = np.loadtxt(file2g, skiprows=0)[:, 8:11]
traj3_x = np.loadtxt(file3, skiprows=0)[:, 8:11]
traj3_dphi = np.loadtxt(file3, skiprows=0)[:, 11:]
traj4a_x = np.loadtxt(file4a, skiprows=0)[:, 8:11]
traj4a_xdot = np.loadtxt(file4a, skiprows=0)[:, 11:]
traj4b_x = np.loadtxt(file4b, skiprows=0)[:, 8:11]
traj4b_xdot = np.loadtxt(file4b, skiprows=0)[:, 11:]

time1a = np.loadtxt(file1a, skiprows=0)[:, 0]
time1c = np.loadtxt(file1c, skiprows=0)[:, 0]
time2d = np.loadtxt(file2d, skiprows=0)[:, 0]
time2e = np.loadtxt(file2e, skiprows=0)[:, 0]
time2f = np.loadtxt(file2f, skiprows=0)[:, 0]
time2g = np.loadtxt(file2g, skiprows=0)[:, 0]
time3 = np.loadtxt(file3, skiprows=0)[:, 0]
time4a = np.loadtxt(file4a, skiprows=0)[:, 0]
time4b = np.loadtxt(file4b, skiprows=0)[:, 0]

x_des1 = (-0.1, 0.15, 0.2) # desired end-effector position for Q2d and Q2e
x_des2 = (-0.65, -0.45, 0.7) # desired end-effector position for Q2f and Q2g
x_des3 = (0.6, 0.3, 0.5) # desired end-effector position for Q3
x_des4 = (0.6, 0.3, 0.4) # desired end-effector position for Q4

# desired circular trajectory for Q1
xd1a = 0.3 + 0.1*np.sin(np.pi*time1a)
yd1a = 0.1 + 0.1*np.cos(np.pi*time1a)
xd1c = 0.3 + 0.1*np.sin(np.pi*time1c)
yd1c = 0.1 + 0.1*np.cos(np.pi*time1c)

# velocity magnitude for Q4
vmag4a = np.linalg.norm(traj4a_xdot, axis=1)
vmag4b = np.linalg.norm(traj4b_xdot, axis=1)

# plotting
############# Question 1 #############

plt.figure(1)
for i in range(traj1a_x.shape[1]):
	plt.plot(time1a, traj1a_x[:, i])
plt.gca().set_prop_cycle(None)
plt.plot(time1a, xd1a, '--')
plt.plot(time1a, yd1a, '--')
plt.plot(time1a, np.full_like(time1a, 0.5), '--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$', '$y$', '$z$', '$x_d$', '$y_d$', '$z_d$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure1.png", bbox_inches="tight")
plt.show()

plt.figure(2)
plt.plot(traj1a_x[:, 0], traj1a_x[:, 1])
plt.xlabel('x (m) $\\rightarrow$')
plt.ylabel('y (m) $\\rightarrow$')
plt.title('Plot of y versus x')
plt.axis('equal')
plt.grid()
plt.savefig("Figure2.png", bbox_inches="tight")
plt.show()

plt.figure(3)
for i in range(traj1c_x.shape[1]):
	plt.plot(time1c, traj1c_x[:, i])
plt.gca().set_prop_cycle(None)
plt.plot(time1c, xd1c, '--')
plt.plot(time1c, yd1c, '--')
plt.plot(time1c, np.full_like(time1c, 0.5), '--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$', '$y$', '$z$', '$x_d$', '$y_d$', '$z_d$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure3.png", bbox_inches="tight")
plt.show()

plt.figure(4)
plt.plot(traj1c_x[:, 0], traj1c_x[:, 1])
plt.xlabel('x (m) $\\rightarrow$')
plt.ylabel('y (m) $\\rightarrow$')
plt.title('Plot of y versus x')
plt.axis('equal')
plt.grid()
plt.savefig("Figure4.png", bbox_inches="tight")
plt.show()

############# Question 2 #############

plt.figure(5)
for i in range(traj2d_x.shape[1]):
	plt.plot(time2d, traj2d_x[:, i])
plt.gca().set_prop_cycle(None)
for i in range(traj2d_x.shape[1]):
	plt.plot(time2d, np.full_like(time2d, x_des1[i]), '--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$', '$y$', '$z$', '$x_d$', '$y_d$', '$z_d$'])
plt.xlim(0, 3)
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure5.png")
plt.show()

fig, (ax1, ax2) = plt.subplots(2, sharex=True, num=6)
fig.suptitle('Plot of $q_4$ and $q_6$ versus time')
ax1.plot(time2d, traj2d_q[:, 3], 'b-')
ax1.axhline(y=-170.0*np.pi/180.0, color='b', linestyle='--')
ax1.axhline(y=-30.0*np.pi/180.0, color='b', linestyle='--')
ax2.plot(time2d, traj2d_q[:, 5], 'r-')
ax2.axhline(y=0, color='r', linestyle='--')
ax2.axhline(y=210.0*np.pi/180.0, color='r', linestyle='--')
ax1.set(ylabel='$q_4$ (rad) $\\rightarrow$')
ax2.set(xlabel='time (sec) $\\rightarrow$', ylabel='$q_6$ (rad) $\\rightarrow$')
ax1.legend(['$q_4$', '$q_{4lower}$', '$q_{4upper}$'])
ax2.legend(['$q_6$', '$q_{6lower}$', '$q_{6upper}$'])
plt.xlim(0, 3)
ax1.grid()
ax2.grid()
plt.savefig("Figure6.png")
plt.show()

##########################################################

plt.figure(7)
for i in range(traj2e_x.shape[1]):
	plt.plot(time2e, traj2e_x[:, i])
plt.gca().set_prop_cycle(None)
for i in range(traj2e_x.shape[1]):
	plt.plot(time2e, np.full_like(time2e, x_des1[i]), '--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$', '$y$', '$z$', '$x_d$', '$y_d$', '$z_d$'])
plt.xlim(0, 3)
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure7.png")
plt.show()

fig, (ax1, ax2) = plt.subplots(2, sharex=True, num=8)
fig.suptitle('Plot of $q_4$ and $q_6$ versus time')
ax1.plot(time2e, traj2e_q[:, 3], 'b-')
ax1.axhline(y=-170.0*np.pi/180.0, color='b', linestyle='--')
ax1.axhline(y=-30.0*np.pi/180.0, color='b', linestyle='--')
ax2.plot(time2e, traj2e_q[:, 5], 'r-')
ax2.axhline(y=0, color='r', linestyle='--')
ax2.axhline(y=210.0*np.pi/180.0, color='r', linestyle='--')
ax1.set(ylabel='$q_4$ (rad) $\\rightarrow$')
ax2.set(xlabel='time (sec) $\\rightarrow$', ylabel='$q_6$ (rad) $\\rightarrow$')
ax1.legend(['$q_4$', '$q_{4lower}$', '$q_{4upper}$'])
ax2.legend(['$q_6$', '$q_{6lower}$', '$q_{6upper}$'])
plt.xlim(0, 3)
ax1.grid()
ax2.grid()
plt.savefig("Figure8.png")
plt.show()

##########################################################

plt.figure(9)
for i in range(traj2f_x.shape[1]):
	plt.plot(time2f, traj2f_x[:, i])
plt.gca().set_prop_cycle(None)
for i in range(traj2f_x.shape[1]):
	plt.plot(time2f, np.full_like(time2f, x_des2[i]), '--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$', '$y$', '$z$', '$x_d$', '$y_d$', '$z_d$'])
plt.xlim(0, 3)
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure9.png")
plt.show()

fig, (ax1, ax2) = plt.subplots(2, sharex=True, num=10)
fig.suptitle('Plot of $q_4$ and $q_6$ versus time')
ax1.plot(time2f, traj2f_q[:, 3], 'b-')
ax1.axhline(y=-170.0*np.pi/180.0, color='b', linestyle='--')
ax1.axhline(y=-30.0*np.pi/180.0, color='b', linestyle='--')
ax2.plot(time2f, traj2f_q[:, 5], 'r-')
ax2.axhline(y=0, color='r', linestyle='--')
ax2.axhline(y=210.0*np.pi/180.0, color='r', linestyle='--')
ax1.set(ylabel='$q_4$ (rad) $\\rightarrow$')
ax2.set(xlabel='time (sec) $\\rightarrow$', ylabel='$q_6$ (rad) $\\rightarrow$')
ax1.legend(['$q_4$', '$q_{4lower}$', '$q_{4upper}$'])
ax2.legend(['$q_6$', '$q_{6lower}$', '$q_{6upper}$'])
plt.xlim(0, 3)
ax1.grid()
ax2.grid()
plt.savefig("Figure10.png")
plt.show()

##########################################################

plt.figure(11)
for i in range(traj2g_x.shape[1]):
	plt.plot(time2g, traj2g_x[:, i])
plt.gca().set_prop_cycle(None)
for i in range(traj2g_x.shape[1]):
	plt.plot(time2g, np.full_like(time2g, x_des2[i]), '--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$', '$y$', '$z$', '$x_d$', '$y_d$', '$z_d$'])
# plt.xlim(0, 3)
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure11.png")
plt.show()

fig, (ax1, ax2) = plt.subplots(2, sharex=True, num=12)
fig.suptitle('Plot of $q_4$ and $q_6$ versus time')
ax1.plot(time2g, traj2g_q[:, 3], 'b-')
ax1.axhline(y=-170.0*np.pi/180.0, color='b', linestyle='--')
ax1.axhline(y=-30.0*np.pi/180.0, color='b', linestyle='--')
ax2.plot(time2g, traj2g_q[:, 5], 'r-')
ax2.axhline(y=0, color='r', linestyle='--')
ax2.axhline(y=210.0*np.pi/180.0, color='r', linestyle='--')
ax1.set(ylabel='$q_4$ (rad) $\\rightarrow$')
ax2.set(xlabel='time (sec) $\\rightarrow$', ylabel='$q_6$ (rad) $\\rightarrow$')
ax1.legend(['$q_4$', '$q_{4lower}$', '$q_{4upper}$'])
ax2.legend(['$q_6$', '$q_{6lower}$', '$q_{6upper}$'])
# plt.xlim(0, 3)
ax1.grid()
ax2.grid()
plt.savefig("Figure12.png")
plt.show()

# ############# Question 3 #############

plt.figure(13)
for i in range(traj3_x.shape[1]):
	plt.plot(time3, traj3_x[:, i])
plt.gca().set_prop_cycle(None)
for i in range(traj3_x.shape[1]):
	plt.plot(time3, np.full_like(time3, x_des3[i]), '--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$', '$y$', '$z$', '$x_d$', '$y_d$', '$z_d$'])
plt.xlim(0, 3)
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure13.png")
plt.show()

plt.figure(14)
for i in range(traj3_dphi.shape[1]):
	plt.plot(time3, traj3_dphi[:, i])
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Orientation error $\\partial\\phi$ $\\rightarrow$')
plt.legend(['$\\partial\\phi_{1}$', '$\\partial\\phi_{2}$', '$\\partial\\phi_{3}$'])
plt.xlim(0, 3)
plt.title('Plot of orientation error versus time')
plt.grid()
plt.savefig("Figure14.png")
plt.show()

############# Question 4 #############

plt.figure(15)
for i in range(traj4a_x.shape[1]):
	plt.plot(time4a, traj4a_x[:, i])
plt.gca().set_prop_cycle(None)
for i in range(traj4a_x.shape[1]):
	plt.plot(time4a, np.full_like(time4a, x_des4[i]), '--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$', '$y$', '$z$', '$x_d$', '$y_d$', '$z_d$'])
plt.xlim(0, 3)
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure15.png")
plt.show()

plt.figure(16)
for i in range(traj4a_xdot.shape[1]):
	plt.plot(time4a, traj4a_xdot[:, i])
plt.plot(time4a, vmag4a, 'r-')
plt.axhline(y=0.1, color='r', linestyle='--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point velocity (m/s) $\\rightarrow$')
plt.legend(['$\\dot{x}$', '$\\dot{y}$', '$\\dot{z}$', '|v|', '$V_{max}$'])
plt.xlim(0, 3)
plt.title('Plot of operational point velocity versus time')
plt.grid()
plt.savefig("Figure16.png")
plt.show()

#########################################

plt.figure(17)
for i in range(traj4b_x.shape[1]):
	plt.plot(time4b, traj4b_x[:, i])
plt.gca().set_prop_cycle(None)
for i in range(traj4b_x.shape[1]):
	plt.plot(time4b, np.full_like(time4b, x_des4[i]), '--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$', '$y$', '$z$', '$x_d$', '$y_d$', '$z_d$'])
plt.xlim(0, 7)
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure17.png")
plt.show()

plt.figure(18)
for i in range(traj4b_xdot.shape[1]):
	plt.plot(time4b, traj4b_xdot[:, i])
plt.plot(time4b, vmag4b, 'r-')
plt.axhline(y=0.1, color='r', linestyle='--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point velocity (m/s) $\\rightarrow$')
plt.legend(['$\\dot{x}$', '$\\dot{y}$', '$\\dot{z}$', '|v|', '$V_{max}$'])
plt.xlim(0, 7)
plt.title('Plot of operational point velocity versus time')
plt.grid()
plt.savefig("Figure18.png")
plt.show()