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

kp_pos[key] = np.array(
 (5000.,  5000.,  5000.,  5000.,  5000.,  5000.,
  5000.,  5000.,  5000.,  5000.,  5000.,  5000.,
  10000.,  10000.,
  10000.0, 10000.,  5000.,  10000.,  3000.,  3000.,  3000.,   
  1000.0,
  10000.,  10000.,  5000.,  10000.,  3000.,  3000.,  3000.,
  1000.0,
  300.0, 300.0));

kd_pos[key] = np.array(
 (20.,  20.,  20.,  20.,  20.,  20.,
  20.,  20.,  20.,  20.,  20.,  20.,
  10.,  10.,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   
  0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0,
  0.1, 0.1));

ki_pos[key] = np.array(
 (5.,  5.,  5.,  5.,  5.,  5.,
  5.,  5.,  5.,  5.,  5.,  5.,
  1.,  1.,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   
  0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0,
  1.0, 1.0));
               
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

iclamp = np.array((7.0, 14.0, 14.0, 25.0, 14.0, 9.0, 7.0, 14.0, 14.0, 25.0, 14.0, 9.0, 10.0, 10.0, 14.0, 14.0, 9.0, 9.0, 5.0, 3.0, 3.0, 10.0, 14.0, 14.0, 9.0, 9.0, 5.0, 3.0, 3.0, 10.0, 5.0, 1.5));
