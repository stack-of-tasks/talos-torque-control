import pinocchio as pin
import numpy as np
import seaborn as sns
from pinocchio.rpy import rpyToMatrix
import matplotlib.pyplot as plt
from numpy import genfromtxt
import math
from pinocchio.rpy import matrixToRpy

# waist = np.loadtxt("waist_rpy.dat")

# se3_id = pin.SE3().Identity()

# size = len(waist)
# waist_se3 = np.zeros((size, 12))

# for i in range(size):
#     se3 = pin.SE3()
#     se3.translation = se3_id.translation
#     se3.rotation = rpyToMatrix(waist[i, 0], waist[i, 1], waist[i, 2])
#     waist_se3[i] = np.concatenate((se3.translation, se3.rotation.reshape(9)))

# np.savetxt("waist.dat", waist_se3)

# com = np.loadtxt("com.dat")

# am = np.zeros((size, 3))
# test_am = np.zeros((size, 3))
# feet_xy = np.zeros((size, 2))
# diff = np.zeros((size, 3))
# mass = 100
# g = np.array([0.0, 0.0, -9.81])
# for j in range(len(com)):
#     am[j] = mass * np.cross(com[j, 0:3], (com[j, 6:9] - g))
#     test_am[j] = mass * np.dot(pin.skew(com[j, 0:3]), (com[j, 6:9] - g))
#     feet_xy[j,1] = (com[j, 2]*com[j, 7])/(-9.81) + com[j, 1]
#     feet_xy[j,0] = (com[j, 2]*com[j, 6])/(-9.81) + com[j, 0]
#     diff = test_am - am

# # print(diff)
# np.savetxt("am.dat", am)
# np.savetxt("am_test.dat", test_am)
# np.savetxt("feet_xy.dat", feet_xy)

# LF = genfromtxt('leftFoot.dat')
# RF = genfromtxt('rightFoot.dat')
# com = genfromtxt('com.dat')
# phases = genfromtxt('phases.dat')

# size = len(com) - 6202
# LF_trans = np.copy(LF[6202:])
# RF_trans = np.copy(RF[6202:])
# com_trans = np.copy(com)
# for i in range(len(com)):
#     com_trans[i,2] += 0.05
# phases_trans = np.copy(phases[6202:])

# com_trans += np.array([0.00679821, 0.08693283, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# np.savetxt("com.dat", com_trans)
# np.savetxt("LF.dat", LF_trans)
# np.savetxt("RF.dat", RF_trans)
# np.savetxt("phases.dat", phases_trans)

######### plots ##########

sns.palplot(sns.color_palette("cubehelix", 8))
palette = sns.color_palette("cubehelix", 20)
palette2 = sns.cubehelix_palette(20, start=.5, rot=-.75, reverse=True)

LF_ref = genfromtxt('leftFoot.dat')
RF_ref = genfromtxt('rightFoot.dat')
com_ref = genfromtxt('com.dat')
# waist_ref = genfromtxt('waist_rpy.dat')
# phases = genfromtxt('phases.dat')
# feet_xy = genfromtxt('feet_xy.dat')
# am = genfromtxt('am.dat')

# com_ref = genfromtxt('online_E1540_4_d20_100_4/com_traj.txt', delimiter=',', skip_header=1)
# LF_ref = genfromtxt('bag/LF_ref_09.txt', delimiter=',', skip_header=1)
# RF_ref = genfromtxt('bag/RF_ref_09.txt', delimiter=',', skip_header=1)
# LF_ref = genfromtxt('online_E1540_4_d20_100_4/LF_traj.txt', delimiter=',', skip_header=1)
# RF_ref = genfromtxt('online_E1540_4_d20_100_4/RF_traj.txt', delimiter=',', skip_header=1)
LF = genfromtxt('online_E1540_4_d20_100_4/LF.txt', delimiter=',', skip_header=1)
RF = genfromtxt('online_E1540_4_d20_100_4/RF.txt', delimiter=',', skip_header=1)
com = genfromtxt('online_E1540_4_d20_100_4/com.txt', delimiter=',', skip_header=1)
#q = genfromtxt('bag/q_23_28_almost.txt', delimiter=',', skip_header=1)
# waist = genfromtxt('online_E1540_4_d20_100_4/waist.txt', delimiter=',', skip_header=1)

dim = len(LF)
time = np.linspace(0.0, dim*1e-3, num=dim)
dim2 = len(LF_ref)
time2 = np.linspace(0.0, dim*1e-3, num=dim2)
print(len(time))
LF_rpy = np.zeros((dim, 6))
RF_rpy = np.zeros((dim, 6))
LF_rpy_traj = np.zeros((dim2, 6))
RF_rpy_traj = np.zeros((dim2, 6))
waist_rpy = np.zeros((dim, 6))
waist_ref = np.zeros((dim2, 3))

for i in range(dim):
    # quat = pin.Quaternion(q[i, 4:8].reshape(4,1)).normalized()
    # R = quat.toRotationMatrix()
    # rpy = matrixToRpy(R)
    # waist_rpy[i] = np.concatenate((waist[i, 1:4], matrixToRpy(waist[i, 4:].reshape(3,3))))
    # LF_rpy[i] = np.concatenate((LF[i, 1:4], matrixToRpy(LF[i, 4:].reshape(3,3))))
    # RF_rpy[i] = np.concatenate((RF[i, 1:4], matrixToRpy(RF[i, 4:].reshape(3,3)))) 
    quat = pin.Quaternion(LF[i, 4:8].reshape(4,1)).normalized()
    R = quat.toRotationMatrix()
    rpy = matrixToRpy(R)
    LF_rpy[i] = np.concatenate((LF[i, 1:4], rpy))
    # quat = pin.Quaternion(RF[i, 4:8].reshape(4,1)).normalized()
    # R = quat.toRotationMatrix()
    # rpy = matrixToRpy(R)
    # RF_rpy[i] = np.concatenate((RF[i, 1:4], rpy))

for i in range(len(LF_ref)):
    LF_rpy_traj[i] = np.concatenate((LF_ref[i, 0:3], matrixToRpy(LF_ref[i, 3:].reshape(3,3))))
    RF_rpy_traj[i] = np.concatenate((RF_ref[i, 0:3], matrixToRpy(RF_ref[i, 3:].reshape(3,3))))
    waist_ref[i] = 0.5 * (LF_rpy_traj[i, 3:]+RF_rpy_traj[i, 3:])

for i in range(len(RF)):
    quat = pin.Quaternion(RF[i, 4:8].reshape(4,1)).normalized()
    R = quat.toRotationMatrix()
    rpy = matrixToRpy(R)
    RF_rpy[i] = np.concatenate((RF[i, 1:4], rpy))
    
# #if __name__== "__main__":
sns.despine()
sns.set_style("ticks")
fig, (ax1, ax) = plt.subplots(nrows=1, ncols=2)
# ax1 = fig.add_subplot(111)
# ax = fig.add_subplot(211)
ax2 = ax.twinx()
# plt.rc('xtick',labelsize=30)
# plt.rc('ytick',labelsize=30)
ax.xaxis.set_tick_params(labelsize=20)
ax.yaxis.set_tick_params(labelsize=20)
ax2.xaxis.set_tick_params(labelsize=20)
ax2.yaxis.set_tick_params(labelsize=20)
ax1.xaxis.set_tick_params(labelsize=20)
ax1.yaxis.set_tick_params(labelsize=20)
# plt.xlim(0,48)
lw = 5

# plt.plot(com[:,0], linewidth=lw)
# plt.plot(com[:,1], linewidth=lw)
# plt.plot(com_traj[:,1], linewidth=lw, linestyle='dashed')
# plt.plot(com_traj[:,2], linewidth=lw, linestyle='dashed')
# plt.plot(time, com[:,3], linewidth=lw)
# plt.plot(com[:,0], com[:,1], linewidth=lw, color=palette[6])
# plt.plot(LF[:,0], LF[:,1], linewidth=lw, color=palette[11])
# plt.plot(RF[:,0], RF[:,1], linewidth=lw, color=palette[14])

# plt.plot(LF[:,1], linewidth=lw)
# plt.plot(LF[:,2], linewidth=lw)
# plt.plot(RF[:,2], linewidth=lw)
# plt.plot(LF[:,3], linewidth=lw)
# plt.plot(LF_traj[:,1], linewidth=lw, linestyle='dashed')
# plt.plot(LF_traj[:,2], linewidth=lw, linestyle='dashed')
# plt.plot(LF_traj[:,3], linewidth=lw, linestyle='dashed')

# plt.plot(LF_rpy[:,3], linewidth=lw)
# plt.plot(LF_rpy[:,4], linewidth=lw)
# plt.plot(LF_rpy[:,5], linewidth=lw)
# plt.plot(LF_rpy_traj[:,3], linewidth=lw, linestyle='dashed')
# plt.plot(LF_rpy_traj[:,4], linewidth=lw, linestyle='dashed')
# plt.plot(LF_rpy_traj[:,5], linewidth=lw, linestyle='dashed')

# ax.plot(com_ref[:,0], com_ref[:,1], linewidth=lw, color=palette[6])

# ax.plot(LF_ref[:,0], LF_ref[:,1], linewidth=lw, color=palette[11])
# ax.plot(RF_ref[:,0], RF_ref[:,1], linewidth=lw, color=palette[14])

# ax.plot(time, com_ref[:,0], linewidth=lw, color=palette[6])

# ax.plot(time, LF_ref[:,0], linewidth=lw, color=palette[11])
# ax.plot(time, RF_ref[:,0], linewidth=lw, color=palette[14])
# ax1.plot(time, com_ref[:,1], linewidth=lw, color=palette[6])

# ax1.plot(time, LF_ref[:,1], linewidth=lw, color=palette[11])
# ax1.plot(time, RF_ref[:,1], linewidth=lw, color=palette[14])

# 11405 10400 11650
ax.plot(com[:9200,1], com[:9200,2], linewidth=lw)#, color=palette[6])
ax.plot(LF[:9200,1], LF[:9200,2], linewidth=lw)#, color=palette[11])
ax.plot(RF[:9200,1], RF[:9200,2], linewidth=lw)#, color=palette[14])
# #13500 7890
ax.plot(com_ref[:,0], com_ref[:,1], linewidth=3, linestyle='dashed') #, linewidth=lw, color=palette[6])

ax.plot(LF_rpy_traj[:,0], LF_rpy_traj[:,1], linewidth=3, linestyle='dashed') #, linewidth=lw, color=palette[11])
ax.plot(RF_rpy_traj[:,0], RF_rpy_traj[:,1], linewidth=3, linestyle='dashed') #, linewidth=lw, color=palette[14])

# ax1.plot(time[:7950], -waist_rpy[:7950,5], linewidth=lw)#, color=palette[6])
# ax1.plot(time[:7950], LF_rpy[:7950,5], linewidth=lw)#, color=palette[11])
# ax1.plot(time[:7950], RF_rpy[:7950,5], linewidth=lw)#, color=palette[14])
# ax1.plot(time[:11900], waist_ref[:11900,2], linewidth=3, linestyle='dashed') #, linewidth=lw, color=palette[6])
# ax1.plot(time2[:11900], LF_rpy_traj[:11900,5], linewidth=3, linestyle='dashed') #, linewidth=lw, color=palette[11])
# ax1.plot(time2[:11900], RF_rpy_traj[:11900,5], linewidth=3, linestyle='dashed') #, linewidth=lw, color=palette[14])

#plt.plot(phases)
# plt.plot(feet_xy[:,0], feet_xy[:,1], linewidth=lw)
# plt.plot(LF[:,0], linewidth=lw, color=palette[11])
# plt.plot(RF[:,0], linewidth=lw, color=palette[14])
# plt.plot(feet_xy[:,0], linewidth=lw, color=palette[6], linestyle='dashed')
ax.legend(['CoM', 'Left Foot', 'Right Foot', 'CoM reference', 'Left Foot reference', 'Right Foot reference'], prop={'size': 20}, loc='upper left', ncol=1)
ax1.legend(['Free-Flyer orientation', 'Left Foot orientation', 'Right Foot orientation', 'Free-Flyer reference orientation', 'Left Foot reference orientation', 'Right Foot reference orientation'], prop={'size': 20}, loc='upper left', ncol=1)

ax1.set_ylabel("z axis orientation [rad]", fontsize=30)
ax.set_ylabel("y [m]", fontsize=30)
ax2.set_ylabel("Aerial view", fontsize=30)
ax2.get_yaxis().set_ticks([])

# plt.legend(['CoM y', 'LF z', 'RF z'], prop={'size': 25}, loc='upper left', ncol=1)

# plt.legend(['LF rot x','LF rot y','LF rot z', 'LF_traj rot x', 'LF_traj rot y', 'LF_traj rot z'], prop={'size': 25}, loc='upper left', ncol=1)
# plt.plot(am[:,0], linewidth=lw)
# plt.plot(am[:,1], linewidth=lw)
# plt.plot(am[:,2], linewidth=lw)
# plt.legend(['AM x', 'AM y', 'AM z'], prop={'size': 25}, loc='lower left', ncol=1)
ax.set_xlabel("x [m]", fontsize=30)
ax1.set_xlabel("Time [s]", fontsize=30)
ax1.grid(True)
ax.grid(True)
plt.show()

raw_input("Enter to quit")