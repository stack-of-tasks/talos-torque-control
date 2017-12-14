# -*- coding: utf-8 -*-
"""
Created on Mon Feb  9 13:55:16 2015

Position control PID gains stored according to the controller time step (1, 2, 5 or 10 ms).

@author: adelpret
"""
import numpy as np

NJ = 32;
kp_pos = {};   # joint position control proportional gains
kd_pos = {};   # joint position control derivative gains
ki_pos = {};   # joint position control integral gains

key = round(0.001,3);
'''kp_pos[key] = np.array(
              (400.,  400.,  400.,  400.,  800.,  800.,  
               400.,  400.,  400.,  400.,  800.,  800.,  
               800.,  800.,
               500.,  500.,  500.,  500.,  500.,  500.,   50.,
               1.0,
               500.,  500.,  500.,  500.,  500.,  500.,   50.,
               1.0,
               1.0,1.0));'''
kp_pos[key] = np.array(
 (1.,  1.,  1.,  1.,  1.,  1.,
  1.,  1.,  1.,  1.,  1.,  1.,
  1.,  1.,
  1.,  1.,  1.,  1.,  1.,  1.,   1.,
  1.0,
  1.,  1.,  1.,  1.,  1.,  1.,   1.,
  1.0,
  1.0,1.0));
kd_pos[key] = np.zeros(NJ);
ki_pos[key] = np.zeros(NJ);
               
key = round(0.002,3);
kp_pos[key] = kp_pos[round(0.001,3)];
kd_pos[key] = kd_pos[round(0.001,3)];
ki_pos[key] = ki_pos[round(0.001,3)];

key = round(0.003,3);
kp_pos[key] = kp_pos[round(0.001,3)];
kd_pos[key] = kd_pos[round(0.001,3)];
ki_pos[key] = ki_pos[round(0.001,3)];
               
key = round(0.005,3);
kp_pos[key] = kp_pos[round(0.001,3)];
kd_pos[key] = kd_pos[round(0.001,3)];
ki_pos[key] = ki_pos[round(0.001,3)];
               
key = round(0.010,3);
kp_pos[key] = kp_pos[round(0.001,3)];
kd_pos[key] = kd_pos[round(0.001,3)];
ki_pos[key] = ki_pos[round(0.001,3)];
