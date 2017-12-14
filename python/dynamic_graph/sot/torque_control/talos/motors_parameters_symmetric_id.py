# -*- coding: utf-8 -*-
"""
Created on Mon Feb  9 13:55:16 2015

@author: adelpret
"""
from numpy import zeros as zeros

NJ = 32;
k_tau = zeros(NJ);
k_v   = zeros(NJ);

# PARAMETERS OF R_hip_y JOINT 0
k_v[0] = 0.017585
k_tau[0] = 0.000355
# PARAMETERS OF R_hip_r JOINT 1
k_v[1] = 0.006573
k_tau[1] = 0.000036
# PARAMETERS OF R_hip_p JOINT 2
k_v[2] = 0.008817
k_tau[2] = 0.000109
# PARAMETERS OF R_knee JOINT 3
k_v[3] = 0.006774
k_tau[3] = 0.000060
# PARAMETERS OF R_ankle pitch JOINT 4
k_v[4] = 0.008107
k_tau[4] = 0.000226
# PARAMETERS OF R_ankle roll JOINT 5
k_v[5] = 0.007444
k_tau[5] = 0.000273
