# -*- coding: utf-8 -*-
"""
Created on Mon Feb  9 13:55:16 2015

@author: adelpret
"""
import numpy as np

NJ = 32;

COULOMB_FRICTION_COMPENSATION_PERCENTAGE = 0.0;
k_p_torque                              = np.array(NJ*[0.5,]);   # torque control proportional gains
k_d_torque                              = np.array(NJ*[0.0,]);   # torque control derivative gains
k_i_torque                              = np.array(NJ*[3.0,]);   # torque control integral gains
torque_integral_saturation              = np.array(NJ*[0.0,]);
poly_sign_dq                            = 3;                     # order of polynomial to approximate Coulomb friction around zero velocity
k_d_velocity                            = np.array(NJ*[0.0,]);   # torque control derivative gains
alpha_leaking                           = np.array([75.,75.,75.,75.,75.,75.,
                                                    75.,75.,75.,75.,75.,75.,
                                                    0.0,0.0,
                                                    0.0,0.0,0.0,0.0,0.0,0.0,0.0,
                                                    0.0,
                                                    0.0,0.0,0.0,0.0,0.0,0.0,0.0,
                                                    0.0,
                                                    0.0,0.0]);

## PARAMETERS OF R_hip_y JOINT 0
#k_p_torque[0] = 12.0;
## PARAMETERS OF R_hip_r JOINT 1
#k_p_torque[1] = 5; #15.0 # could easily go up to 20, but it's a bit less stable
## PARAMETERS OF R_hip_p JOINT 2
#k_p_torque[2] = 6; #with delay 30 ms
## PARAMETERS OF R_knee JOINT 3
#k_p_torque[3] = 10.0;  # with 12 it starts vibrating at low velocity
## PARAMETERS OF R_ankle pitch JOINT 4
#k_p_torque[4] = 10.0;  # 10 feels good, but maybe i could go higher
## PARAMETERS OF R_ankle roll JOINT 5
#k_p_torque[5] = 15.0; # could go higher, but it feels already good
## PARAMETERS OF Left hip pitch JOINT 8
#k_p_torque[8] = 6.0;   # with delay 20 ms
