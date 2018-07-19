# -*- coding: utf-8 -*-
"""
Created on Mon Feb  9 13:55:16 2015

@author: adelpret
"""
import numpy as np

NJ = 32;
TAU_MAX                     = 1.*1e2;   # max joint torques (security check of ControlManager)
CURRENT_MAX                 = 20.0;   # max motor current (security check of ControlManager)
CTRL_MAX                    = 20.0;   # max desired current (security check of ControlManager)
model_path                  = ["/opt/openrobots/share"];
urdfFileName                = model_path[0] + "/talos_data/robots/talos_reduced.urdf";
ImuJointName                = "imu_joint";

mapJointNameToID={
    'lhy' : 0,
    'lhr' : 1,
    'lhp' : 2,
    'lk'  : 3,
    'lap' : 4,
    'lar' : 5,
    'rhy' : 6,
    'rhr' : 7,
    'rhp' : 8,
    'rk'  : 9,
    'rap' : 10,
    'rar' : 11,
    'ty'  : 12,
    'tp'  : 13,
    'lsy'  : 14,
    'lsr'  : 15,
    'lay'  : 16, 
    'le'   : 17,
    'lwy'  : 18,
    'lwp'  : 19,
    'lwr'  : 20,
    'lh'   : 21,
    'rsy'  : 22,
    'rsr'  : 23, 
    'ray'  : 24, 
    're'   : 25, 
    'rwy'  : 26,  
    'rwp'  : 27,  
    'rwr'  : 28,  
    'rh'   : 29, 
    'hp'   : 30, 
    'hy'   : 31
}


    
mapJointLimits={
    0 : [-0.349065850399, 1.57079632679],
    1 : [-0.5236, 0.5236],
    2 : [-2.095, 0.7],
    3  : [0.0, 2.618],
    4 : [-1.309, 0.768],
    5 : [-0.5236, 0.5236],
    6 : [-1.57079632679, 0.349065850399],
    7 : [-0.5236, 0.5236],
    8 : [-2.095, 0.7],
    9  : [0.0, 2.618],
    10 : [-1.309, 0.768],
    11 : [-0.5236, 0.5236],
    12  : [-1.308996939, 1.308996939],
    13  : [-0.261799387799, 0.785398163397],
    14  : [-1.57079632679, 0.523598775598],
    15  : [0.0, 2.87979326579],
    16 : [-2.44346095279, 2.44346095279],
    17 : [-2.35619449019, 0.0],
    18 : [-2.53072741539, 2.53072741539],
    19 : [-1.3962634016, 1.3962634016],
    20 : [-0.698131700798, 0.698131700798],
    21  : [-1.0471975512, 0.0],
    22  : [-0.523598775598, 1.57079632679],
    23 : [-2.87979326579, 0.0],
    24 : [-2.44346095279, 2.44346095279],
    25 : [-2.35619449019, 0.0],
    26 : [-2.53072741539, 2.53072741539],
    27 : [-1.3962634016, 1.3962634016],
    28  : [-0.698131700798, 0.698131700798],
    29  : [-1.0471975512, 0.0],
    30  : [-0.261799387799, 0.785398163397],
    31  : [-1.308996939, 1.308996939]
}
    
vfMax=np.array([100.0,100.0,300.0,80.0,80.0,30.0])
vfMin=-vfMax
mapForceIdToForceLimits={
    0: [vfMin,vfMax],
    1: [vfMin,vfMax],
    2: [vfMin,vfMax],
    3: [vfMin,vfMax]
 }
    
mapNameToForceId={
    "rf": 0,
    "lf": 1,
    "rh": 2,
    "lh": 3
 }
    
indexOfForceSensors= () 

footFrameNames= { 
    "Right": "leg_right_6_joint",
    "Left" : "leg_left_6_joint"
}
      
rightFootSensorXYZ = (0.0,0.0,-0.085)
rightFootSoleXYZ   = (0.0,0.0,-0.105)

urdftosot=(0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31)
