from balance_ctrl_conf import *


''' *********************** USER-PARAMETERS *********************** '''

# COM_DES = (0.00, 0.0, 0.875)

# CONTROLLER GAINS
NJ = 32
# kp_posture  = (10., 5., 5., 1., 1., 10., 10., 5., 5., 1., 1., 10., 5000., 5000., 50., 100., 10., 10., 10., 10., 100., 50., 50., 100., 10., 10., 10., 10., 100., 50., 100., 100.)  # proportional gain of postural task
# kp_posture  = (50., 50.,50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 500., 500., 50., 50., 50., 50., 50., 50., 50., 10., 50., 50., 50., 50., 50., 50., 50., 10., 100., 100.)  # proportional gain of postural task

# kp_posture  = np.array((30., 30., 30., 10., 30., 30., 30., 30., 30., 10., 30., 30., 1500., 1500., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 100., 100.)) # proportional gain of postural task

kp_posture  = np.array((10., 5., 5., 1., 10., 10., 10., 5., 5., 1., 10., 10., 100., 100., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 50., 100., 100.))  # proportional gain of postural task
kd_posture = np.array(2 * np.sqrt(kp_posture))

kp_pos = np.array(
(1300.,  1300.,  1300.,  1300.,  50., 50.,
 1300.,  1300.,  1300.,  1300.,  50., 50.,
 100., 100.,
 500.,  500.,  500.,  500.,  500.,  500., 500.,
 10.,
 500.,  500.,  500.,  500.,  500.,  500., 500.,
 10.,
 10.,10.))

kd_pos = np.array(
 (20.,  20.,  20.,  20.,  0.,  0.,
  20.,  20.,  20.,  20.,  0.,  0.,
  10.,  10.,
  5.,  5.,  5.,  5.,  5.,  5., 5.,   
  0.1,
  5.,  5.,  5.,  5.,  5.,  5., 5.,
  0.1,
  0.1, 0.1))

kp_tau = 0.1 * np.array(
(2.5, 2.5,  2.5,  2.5,  8.5,  8.5,
 2.5, 2.5,  2.5,  2.5,  8.5,  8.5,
 2.5, 2.5,
 2.5, 2.5,  2.5,  2.5,  2.5,  0.0, 0.0,
 0.0,
 2.5, 2.5,  2.5,  2.5,  2.5,  0.0, 0.0,
 0.0,
 0.0, 0.0))

kff_tau = np.array(
(1.0, 1.0,  1.0,  1.0,  1.0,  1.0,
 1.0, 1.0,  1.0,  1.0,  1.0,  1.0,
 1.0, 1.0,
 1.0, 1.0,  1.0,  1.0,  1.0,  1.0, 1.0,
 1.0,
 1.0, 1.0,  1.0,  1.0,  1.0,  1.0, 1.0,
 1.0,
 1.0, 1.0))

kff_dq = 0. * np.array(
(1.0, 1.0,  1.0,  1.0,  1.0,  1.0,
 1.0, 1.0,  1.0,  1.0,  1.0,  1.0,
 1.0, 1.0,
 1.0, 1.0,  1.0,  1.0,  1.0,  1.0, 1.0,
 1.0,
 1.0, 1.0,  1.0,  1.0,  1.0,  1.0, 1.0,
 1.0,
 1.0, 1.0))

kp_contact  = 30 # constraint proportional feedback gain
kd_contact  = 2*sqrt(kp_contact) # constraint derivative feedback gain
kp_com      = 100.0
kd_com      = 5#2.0*sqrt(kp_com)
kp_com_vel  = 100.0
kd_com_vel  = 2.0*sqrt(kp_com)
kp_waist    = 100.0
kd_waist    = 2.0*sqrt(kp_waist)
kp_am       = 10
kd_am       = 2.0*sqrt(kp_am)
kp_feet     = 100.0
kd_feet     = 5#2.0*sqrt(kp_feet)

# # CONTROLLER WEIGTHS
w_com = 1e6
w_com_vel  = 1e6
w_posture = 1e-1 # weight of postural task
w_forces = 1e-3
w_waist = 10.0
w_am = 2e-2
w_feet = 1e3

# CONTACT PARAMETERS
RIGHT_FOOT_SIZES  = np.array((0.1,  -0.11,  0.069,  -0.069)) # pos x, neg x, pos y, neg y size 
LEFT_FOOT_SIZES = np.array((0.1, -0.11,  0.069, -0.069)) # pos x, neg x, pos y, neg y size 

RIGHT_FOOT_CONTACT_POINTS  = np.array(((RIGHT_FOOT_SIZES[0], RIGHT_FOOT_SIZES[0], RIGHT_FOOT_SIZES[1], RIGHT_FOOT_SIZES[1]),
                              (RIGHT_FOOT_SIZES[3], RIGHT_FOOT_SIZES[2], RIGHT_FOOT_SIZES[3], RIGHT_FOOT_SIZES[2]),
                              (-0.107, -0.107, -0.107, -0.107)))    # contact points in local reference frame

LEFT_FOOT_CONTACT_POINTS  = np.matrix([[LEFT_FOOT_SIZES[0], LEFT_FOOT_SIZES[3], -0.107],
                                     [LEFT_FOOT_SIZES[0], LEFT_FOOT_SIZES[2], -0.107],
                                     [LEFT_FOOT_SIZES[1], LEFT_FOOT_SIZES[3], -0.107],
                                     [LEFT_FOOT_SIZES[1], LEFT_FOOT_SIZES[2], -0.107]]).T    # contact points in local reference frame
FOOT_CONTACT_NORMAL = np.array((0.0, 0.0, 1.0))
mu  = np.array([0.3, 0.1])          # force and moment friction coefficient
fMin = 1.0                       # minimum normal force
fMax = 1e3                       # maximum normal force

RF_FORCE_DES = (-1.046429493171104411e-04, -7.562765419402753819e-01, 4.353980355770353299e+02, 7.602554432679653473e-04, -2.207972285084590425e+00, -1.498280724396473806e-02)
LF_FORCE_DES = (1.046429491392032729e-04, -7.562766302521330974e-01, 4.501708762222422138e+02, 7.602554434527064586e-04, -2.207972285084379038e+00, -1.498280720937920224e-02)
# Gazebo
# left_foot_pose_init:
#(0.00018613902637534818, 0.08484247188164483, 0.10499893952050893, 0.9999993219959397, 1.5026205931086e-07, 0.0011644774099903166, 4.460331463820307e-07, 0.9999998688916376, -0.0005120708040054116, -0.001164477334262359, 0.0005120709762148502, 0.9999991908875991)
# right_foot_pose_init:
#(-0.00018606594667042176, -0.08484243593632501, 0.10496893511785998, 0.9999996580947196, 2.0881799427841137e-06, 0.0008269256817475812, -1.8919417197805814e-06, 0.9999999718398825, -0.00023731130305498609, -0.0008269261540099154, 0.0002373096574218008, 0.9999996299385627)