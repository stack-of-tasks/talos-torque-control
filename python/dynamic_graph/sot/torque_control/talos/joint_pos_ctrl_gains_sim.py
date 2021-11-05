# -*- coding: utf-8 -*-
"""
Created on Mon Feb  9 13:55:16 2015

Position control PID gains stored according to the controller time step (1, 2, 5 or 10 ms).

@author: adelpret
"""
from joint_pos_ctrl_gains import *

key = round(0.001,3);
# kp_pos[key] = np.array(
#  (1000.,  1000.,  1000.,  1300.,  750.,  750.,
#   1000.,  1000.,  1000.,  1300.,  750.,  750.,
#   4000.,  4000.,
#   200.0, 1000.,  500.,  500.,  500.,  300.,  300.,
#   0.2,
#   200.0, 1000.,  500.,  500.,  500.,  300.,  300.,
#   0.2,
#   10.,10.));
#   (1000, 5000, 1000, 1000, 500, 500, 
#    1000, 5000, 1000, 1000, 500, 500, 
#    10000, 10000, 
#    500, 500, 500, 500, 100, 100, 100, 
#    50, 
#    500, 500, 500, 500, 100, 100, 100, 
#    50, 
#    10, 10));   # proportional gain of postural task

# kp_pos = np.array(
#  (20000.,  20000.,  20000.,  20000.,  20000.,  20000.,
#   20000.,  20000.,  20000.,  20000.,  20000.,  20000.,
#   10000.,  10000.,
#   10000.0, 10000.,  5000.,  10000.,  3000.,  3000.,  3000.,
#   1000.0,
#   10000.0, 10000.,  5000.,  10000.,  3000.,  3000.,  3000.,
#   1000.0,
#   300.0, 300.0));
# kd_pos = np.array(
#  (40.,  40.,  40.,  40.,  40.,  40.,
#   40.,  40.,  40.,  40.,  40.,  40.,
#   10.,  10.,
#   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   
#   0.0,
#   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#   0.0,
#   0.1, 0.1));
# ki_pos = np.array(
#  (5.,  5.,  5.,  5.,  5.,  5.,
#   5.,  5.,  5.,  5.,  5.,  5.,
#   1.,  1.,
#   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   
#   0.0,
#   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#   0.0,
#   1.0, 1.0));

kp_pos[key] = np.array(
 (5000.,  5000.,  5000.,  5000.,  5000.,  5000.,
  5000.,  5000.,  5000.,  5000.,  5000.,  5000.,
  10000.,  10000.,
  10000.0, 20000.,  5000.,  5000.,  500.,  500.,  500.,   
  1000.0,
  10000.,  20000.,  5000.,  5000.,  500.,  500.,  500.,
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

'''kp_pos[key] = 20*np.array(
             (500.,  500.,  500.,  500.,  800.,  800.,
              500.,  500.,  500.,  500.,  800.,  800.,
              500.,  500.,
              100.,  100.,  100.,  100.,  100.,  100.,   100.,
              10.0,
              100.,  100.,  100.,  100.,  100.,  100.,   100.,
              10.0,
              100.0,100.0));
kd_pos[key] = .0*np.array(
              (17.96, 103.4, 83.35, 154.36, 29.08, 22.44,
               17.96, 103.4, 83.35, 154.36, 29.08, 22.44,
               48.4, 78.4,
               32.35, 55.86, 10.83, 87.05, 13.87, 11.19, 6.93,
               1.0,
               32.35, 55.86, 10.83, 87.05, 13.87, 11.19, 6.93,
               1.0,
               1.0,1.0));
ki_pos[key] = np.zeros(NJ);'''
               
'''key = round(0.002,3);
kp_pos[key] = np.array(
              (450, 3231.25, 1736.5, 3858.75, 727, 561, 
               450, 3231.25, 1736.5, 3858.75, 727, 561, 
               1210, 1960,
               808.75, 1396.5, 357.25, 2176.25, 346.5, 373, 173.25,
               1.0,
               808.75, 1396.5, 357.25, 2176.25, 346.5, 373, 173.25,
               1.0,
               1.0,1.0));
kd_pos[key] = np.array(
              (8.98, 51.7, 41.675, 77.18, 14.54, 11.22, 
               8.98, 51.7, 41.675, 77.18, 14.54, 11.22, 
               24.2, 39.2,
               16.175, 27.93, 5.415, 43.525, 6.935, 5.595, 3.465,
               1.0,
               16.175, 27.93, 5.415, 43.525, 6.935, 5.595, 3.465,
               1.0,
               1.0,1.0));
ki_pos[key] = ki_pos[round(0.001,3)];
               
key = round(0.005,3);
kp_pos[key] = np.array(
              (288, 2068, 1111.36, 2469.6, 465.28, 359.04, 
               288, 2068, 1111.36, 2469.6, 465.28, 359.04, 
               317.194, 513.802,
               212.009, 366.084, 93.6512, 570.491, 90.8326, 97.7798, 45.4164,
               1.0,
               212.009, 366.084, 93.6512, 570.491, 90.8326, 97.7798, 45.4164,
               1.0,
               1.0,1.0));
kd_pos[key] = np.array(
              (7.184, 41.36, 33.34, 61.744, 11.632, 8.976, 
               7.184, 41.36, 33.34, 61.744, 11.632, 8.976, 
               12.3904, 20.0704,
               8.2816, 14.3002, 2.77248, 22.2848, 3.55072, 2.86464, 1.77408,
               1.0,
               8.2816, 14.3002, 2.77248, 22.2848, 3.55072, 2.86464, 1.77408,
               1.0,
               1.0,1.0));
ki_pos[key] = ki_pos[round(0.001,3)];
               
key = round(0.010,3);
kp_pos[key] = np.array(
              (184.32, 1323.52, 711.27, 1580.54, 850.247, 650.338, 
               184.32, 1323.52, 711.27, 1580.54, 850.247, 650.338, 
               203.004, 328.833,
               86.839, 149.948, 38.3596, 233.673, 23.8113, 25.6324, 11.9057,
               1.0,
               86.839, 149.948, 38.3596, 233.673, 23.8113, 25.6324, 11.9057,
               1.0,
               1.0,1.0));
kd_pos[key] = np.array(
              (5.7472, 33.088, 26.672, 49.3952, 19.0973, 16.335, 
               5.7472, 33.088, 26.672, 49.3952, 19.0973, 16.335, 
               9.91232, 16.0563,
               5.30022, 9.15216, 1.77438, 14.2622, 1.81797, 1.4667, 0.908328,
               1.0,
               5.30022, 9.15216, 1.77438, 14.2622, 1.81797, 1.4667, 0.908328,
               1.0,
               1.0,1.0));
ki_pos[key] = ki_pos[round(0.001,3)];'''
