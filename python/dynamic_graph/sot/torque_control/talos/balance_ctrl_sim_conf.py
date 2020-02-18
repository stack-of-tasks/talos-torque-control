from balance_ctrl_conf import *

# CONTROLLER GAINS
# kd_posture  = NJ*(0*sqrt(kp_posture[0]),)
# kd_constr   = 0.0*2*sqrt(kp_constr)   # constraint derivative feedback gain
# kd_com      = 0.0*2*sqrt(kp_com)
# kd_feet     = 0.0*2*sqrt(kp_feet)

''' *********************** USER-PARAMETERS *********************** '''

# COM_DES = (0.00, 0.0, 0.875)

# CONTROLLER GAINS
NJ = 32
kp_posture  = (10., 5., 5., 1., 1., 10., 10., 5., 5., 1., 1., 10., 500., 500., 50., 10., 10., 10., 10., 10., 10., 10., 50., 10., 10., 10., 10., 10., 10., 10., 100., 100.)  # proportional gain of postural task
# kp_posture  = tuple(5 * np.array(kp_posture)) 
# 
# kp_posture = np.array(
#  (10.,  10.,  10.,  10.,  10.,  10.,
#   10.,  10.,  10.,  10.,  10.,  10.,
#   0.,  0.,
#   10.0, 10.,  10.,  10.,  0.,  0.,  0.,   
#   0.0,
#   10.,  10.,  10.,  10.,  0.,  0.,  0.,
#   0.0,
#   0.0, 0.0))
# kp_posture  = tuple(0.0 *np.array((500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 200, 1000, 200, 200, 200, 200, 50, 50, 50, 0, 200, 200, 200, 200, 50, 50, 50, 0, 0, 0)))   # proportional gain of postural task
# kd_posture  = tuple((2) * np.array(kp_posture))
# kp_pos = np.array(
# (1500.,  1500.,  1500.,  1500.,  1500.,  1500.,
#  1500.,  1500.,  1500.,  1500.,  1500.,  1500.,
#  0., 0.,
#  500, 500, 500, 500, 0, 0, 0,
#  0.,
#  500, 500, 500, 500, 0, 0, 0,
#  0.,
#  0.,0.))
# kp_pos = tuple(2 * np.sqrt(kp_pos))
# kd_pos = np.array(
#  (20.,  20.,  20.,  20.,  20.,  20.,
#   20.,  20.,  20.,  20.,  20.,  20.,
#   10.,  10.,
#   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   
#   0.0,
#   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#   0.0,
#   0.1, 0.1))

# kp_posture = np.array(
# (1500.,  1500.,  1500.,  1500.,  1500.,  1500.,
#  1500.,  1500.,  1500.,  1500.,  1500.,  1500.,
#  0, 0, 
#  500, 500, 500, 500, 0, 0, 0,
#  0,
#  500, 500, 500, 500, 0, 0, 0,
#  0,
#  0, 0))
kd_posture = tuple(2 * np.sqrt(kp_posture))
# kp_posture = np.array(
#  (1000.,  1000.,  1000.,  1000.,  1000.,  1000.,
#   1000.,  1000.,  1000.,  1000.,  1000.,  1000.,
#   0.,0.,
#   1000.0, 1000.,  1000.,  1000.,  0.0, 0.0, 0.0,
#   0.,
#   1000.0, 1000.,  1000.,  1000.,  0.0, 0.0, 0.0,
#   0.,
#   0.,0.))
# kd_posture = 0.001 * np.array(
#  (20.,  20.,  20.,  20.,  20.,  20.,
#   20.,  20.,  20.,  20.,  20.,  20.,
#   10.,  10.,
#   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   
#   0.0,
#   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#   0.0,
#   0.1, 0.1))

# kp_pos      = NJ*(5.0,)   # proportional gain of position controller
# kd_pos      = NJ*(2*sqrt(kp_pos[0]),)
kp_contact  = 30.0 #1e-8  # constraint proportional feedback gain
kd_contact  = 2*sqrt(kp_contact)   # constraint derivative feedback gain
kp_com      = 20.0
kd_com      = 2*sqrt(kp_com)
# kp_feet     = 0.0
# kd_feet     = 2*sqrt(kp_feet)
# kp_hands    = 0.0
# # kd_hands    = 0.0
kp_waist    = 500.0
kd_waist    = 2.0*sqrt(kp_waist)
# # kp_admittance     = (0,0,0,.0*1,.0*1,0.)
# # ki_admittance     = 6*(0.0,)
# # constraint_mask = np.array([True, True, True, True, True, True]).T
# # ee_mask         = np.array([True, True, True, True, True, True]).T

# # CONTROLLER WEIGTHS
w_com  = 1.0
# # w_feet = 1e-3
# # w_hands = 0.0
w_posture = 1e-1 # weight of postural task
w_forces = 1e-3
w_waist = 1.0
# w_torques = 0.0

# CONTACT PARAMETERS
RIGHT_FOOT_SIZES  = (0.1,  -0.11,  0.069,  -0.069) # pos x, neg x, pos y, neg y size 
LEFT_FOOT_SIZES = (0.1, -0.11,  0.069, -0.069) # pos x, neg x, pos y, neg y size 

RIGHT_FOOT_CONTACT_POINTS  = ((RIGHT_FOOT_SIZES[0], RIGHT_FOOT_SIZES[0], RIGHT_FOOT_SIZES[1], RIGHT_FOOT_SIZES[1]),
                              (RIGHT_FOOT_SIZES[3], RIGHT_FOOT_SIZES[2], RIGHT_FOOT_SIZES[3], RIGHT_FOOT_SIZES[2]),
                              (-0.107, -0.107, -0.107, -0.107))    # contact points in local reference frame

LEFT_FOOT_CONTACT_POINTS  = np.matrix([[LEFT_FOOT_SIZES[0], LEFT_FOOT_SIZES[3], -0.107],
                                     [LEFT_FOOT_SIZES[0], LEFT_FOOT_SIZES[2], -0.107],
                                     [LEFT_FOOT_SIZES[1], LEFT_FOOT_SIZES[3], -0.107],
                                     [LEFT_FOOT_SIZES[1], LEFT_FOOT_SIZES[2], -0.107]]).T    # contact points in local reference frame
FOOT_CONTACT_NORMAL = (0.0, 0.0, 1.0)
mu  = np.array([0.3, 0.1])          # force and moment friction coefficient
fMin = 1.0                       # minimum normal force
fMax = 1e3                       # maximum normal force

RF_FORCE_DES = (0, 0, 500, 0, 0, 0.)
LF_FORCE_DES = (0, 0, 500, 0, 0, 0.)