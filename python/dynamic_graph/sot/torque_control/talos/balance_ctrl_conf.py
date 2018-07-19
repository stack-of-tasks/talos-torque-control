from math import sqrt
import numpy as np

''' *********************** USER-PARAMETERS *********************** '''

COM_DES = (0.00, 0.0, 0.81);

# CONTROLLER GAINS
NJ = 32;
kp_posture  = NJ*(100.0,);   # proportional gain of postural task
kd_posture  = NJ*(0*sqrt(kp_posture[0]),);
kp_pos      = NJ*(0.0,);   # proportional gain of position controller
kd_pos      = NJ*(0*sqrt(kp_pos[0]),);
kp_constr   = .0*1.0;   # constraint proportional feedback gain
kd_constr   = 0*sqrt(kp_constr);   # constraint derivative feedback gain
kp_com      = .0*30.0;
kd_com      = 0.0;
kp_feet     = .0*30.0;
kd_feet     = 0.0;
kp_admittance     = (0,0,0,.0*1,.0*1,0.);
ki_admittance     = 6*(0.0,);
constraint_mask = np.array([True, True, True, True, True, True]).T;
ee_mask         = np.array([True, True, True, True, True, True]).T;

# CONTROLLER WEIGTHS
w_com               = .0*1.0;
w_feet              = .0*1.0;
w_posture           = 1e-3;  # weight of postural task
w_forces            = .0*1e-6;
w_base_orientation  = 0.0;
w_torques           = 0.0;

# CONTACT PARAMETERS
RIGHT_FOOT_SIZES  = (0.130,  -0.100,  0.056,  -0.075); # pos x, neg x, pos y, neg y size 
LEFT_FOOT_SIZES = (0.130, -0.100,  0.075, -0.056); # pos x, neg x, pos y, neg y size 

RIGHT_FOOT_SIZES  = (0.130,  -0.100,  0.056,  -0.056); # pos x, neg x, pos y, neg y size 
RIGHT_FOOT_CONTACT_POINTS  = ((RIGHT_FOOT_SIZES[0], RIGHT_FOOT_SIZES[0], RIGHT_FOOT_SIZES[1], RIGHT_FOOT_SIZES[1]),
                              (RIGHT_FOOT_SIZES[3], RIGHT_FOOT_SIZES[2], RIGHT_FOOT_SIZES[3], RIGHT_FOOT_SIZES[2]),
                              (-0.105, -0.105, -0.105, -0.105));    # contact points in local reference frame

LEFT_FOOT_CONTACT_POINTS  = np.matrix([[LEFT_FOOT_SIZES[0], LEFT_FOOT_SIZES[3], -0.105],
                                     [LEFT_FOOT_SIZES[0], LEFT_FOOT_SIZES[2], -0.105],
                                     [LEFT_FOOT_SIZES[1], LEFT_FOOT_SIZES[3], -0.105],
                                     [LEFT_FOOT_SIZES[1], LEFT_FOOT_SIZES[2], -0.105]]).T    # contact points in local reference frame
FOOT_CONTACT_NORMAL = (0.0, 0.0, 1.0);
mu  = np.array([0.3, 0.1]);          # force and moment friction coefficient
fMin = 1.0;              		    # minimum normal force
fMax = 1e3;					    # maximum normal force

RF_FORCE_DES = (0, 0, 300, 0, 0, 0.);
LF_FORCE_DES = (0, 0, 300, 0, 0, 0.);
