import pinocchio as pin
from numpy import genfromtxt
import numpy as np
import itertools
import math
import curves
from multicontact_api import ContactSequence
from curves import SE3Curve, piecewise_SE3
pin.switchToNumpyArray()

cs_wb = ContactSequence()
cs_wb.loadFromBinary("mc_api_2_trajs/walk_20cm_WB.cs") #step_in_place
cs_ref = ContactSequence()
cs_ref.loadFromBinary("mc_api_2_trajs/walk_20cm_REF.cs") #step_in_place

t_init = cs_ref.contactPhases[0].timeInitial
t_end = cs_ref.contactPhases[-1].timeFinal
t = np.linspace(0, t_end, int(t_end/0.001))

# COM
p = cs_ref.concatenateCtrajectories()
v = cs_ref.concatenateDCtrajectories()
a = cs_ref.concatenateDDCtrajectories()

c = np.zeros((int(t_end/0.001)+1, 9))
for i in t:
    c[int(i/0.001)] = np.concatenate((p(i), v(i), a(i)))

# repeat_c = np.repeat(c, 3, axis=0)
np.savetxt("dat/com.dat", c)

# AM
traj_am = cs_ref.concatenateLtrajectories()
traj_dam = cs_ref.concatenateDLtrajectories()
am = np.zeros((int(t_end/0.001)+1, 6))
for i in t:
    am[int(i/0.001)] = np.concatenate((traj_am(i), traj_dam(i)))

# repeat_am = np.repeat(am, 3, axis=0)
np.savetxt("dat/am.dat", am)

# Feet traj
traj_mid_left = cs_ref.concatenateEffectorTrajectories('leg_left_sole_fix_joint')
traj_begin_left = SE3Curve(traj_mid_left(traj_mid_left.min()), traj_mid_left(traj_mid_left.min()), t_init, traj_mid_left.min())
traj_end_left = SE3Curve(traj_mid_left(traj_mid_left.max()), traj_mid_left(traj_mid_left.max()), traj_mid_left.max(), t_end)
traj_left = piecewise_SE3()
traj_left.append(traj_begin_left)
traj_left.append(traj_mid_left)
traj_left.append(traj_end_left)


traj_mid_right = cs_ref.concatenateEffectorTrajectories('leg_right_sole_fix_joint')
traj_begin_right = SE3Curve(traj_mid_right(traj_mid_right.min()), traj_mid_right(traj_mid_right.min()), t_init, traj_mid_right.min())
traj_end_right = SE3Curve(traj_mid_right(traj_mid_right.max()), traj_mid_right(traj_mid_right.max()), traj_mid_right.max(), t_end)
traj_right = piecewise_SE3()
traj_right.append(traj_begin_right)
traj_right.append(traj_mid_right)
traj_right.append(traj_end_right)

left_foot_pose_init = (0.00018613902637534818, 0.08484247188164483, 0.10499893952050893, 0.9999993219959397, 1.5026205931086e-07, 0.0011644774099903166, 4.460331463820307e-07, 0.9999998688916376, -0.0005120708040054116, -0.001164477334262359, 0.0005120709762148502, 0.9999991908875991)
right_foot_pose_init = (-0.00018606594667042176, -0.08484243593632501, 0.10496893511785998, 0.9999996580947196, 2.0881799427841137e-06, 0.0008269256817475812, -1.8919417197805814e-06, 0.9999999718398825, -0.00023731130305498609, -0.0008269261540099154, 0.0002373096574218008, 0.9999996299385627)

zeros = np.zeros(12)
left_foot = np.zeros((int(t_end/0.001) +1, 12))
right_foot = np.zeros((int(t_end/0.001)+1, 12))
a = np.concatenate((traj_left.translation(0), np.reshape(traj_left.rotation(0), 9)))
diff_l = left_foot_pose_init - a
b = np.concatenate((traj_right.translation(0), np.reshape(traj_right.rotation(0), 9)))
diff_r = right_foot_pose_init - b
for i in t:
    a = np.concatenate((traj_left.translation(i), np.reshape(traj_left.rotation(i), 9)))
    # a[2] += diff_l[2]
    left_foot[int(i/0.001)] = a + diff_l
    b = np.concatenate((traj_right.translation(i), np.reshape(traj_right.rotation(i), 9)))
    # b[2] += diff_r[2]
    right_foot[int(i/0.001)] = b + diff_r

# repeat_left_foot = np.repeat(left_foot, 3, axis=0)
# repeat_right_foot = np.repeat(right_foot, 3, axis=0)
np.savetxt("dat/leftFoot.dat", left_foot)
np.savetxt("dat/rightFoot.dat", right_foot)


# Feet contact force
traj_left = cs_wb.concatenateContactForceTrajectories('leg_left_sole_fix_joint')
traj_right = cs_wb.concatenateContactForceTrajectories('leg_right_sole_fix_joint')

left_foot = np.zeros((int(t_end/0.001) +1, 6))
right_foot = np.zeros((int(t_end/0.001)+1, 6))
for i in t:
    left_foot[int(i/0.001)] = np.reshape(traj_left(i), 12)[:6]
    right_foot[int(i/0.001)] = np.reshape(traj_right(i), 12)[:6]

# repeat_left_foot = np.repeat(left_foot, 3, axis=0)
# repeat_right_foot = np.repeat(right_foot, 3, axis=0)
np.savetxt("dat/leftForceFoot.dat", left_foot)
np.savetxt("dat/rightForceFoot.dat", right_foot)

