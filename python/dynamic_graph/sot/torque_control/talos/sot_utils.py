# -*- coding: utf-8 -*-
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

from pinocchio.utils import zero as mat_zeros
from pinocchio import Quaternion
from pinocchio.rpy import rpyToMatrix

def config_sot_to_urdf(q):
    # GEPETTO VIEWER Free flyer 0-6, CHEST HEAD 7-10, LARM 11-17, RARM 18-24, LLEG 25-30, RLEG 31-36
    # ROBOT VIEWER # Free flyer0-5, RLEG 6-11, LLEG 12-17, CHEST HEAD 18-21, RARM 22-28, LARM 29-35
    qUrdf = mat_zeros(39);
    qUrdf[:3,0] = q[:3,0];
    quatMat = rpyToMatrix(q[3:6,0]);
    quatVec = Quaternion(quatMat);
    qUrdf[3:7,0]   = quatVec.coeffs();
    '''qUrdf[7:11,0]  = q[18:22,0]; # chest-head
    qUrdf[11:18,0] = q[29:,0]; # larm
    qUrdf[18:25,0] = q[22:29,0]; # rarm
    qUrdf[25:31,0] = q[12:18,0]; # lleg
    qUrdf[31:,0]   = q[6:12,0]; # rleg'''
    return qUrdf;
    
def joints_sot_to_urdf(q):
    # GEPETTO VIEWER Free flyer 0-6, CHEST HEAD 7-10, LARM 11-17, RARM 18-24, LLEG 25-30, RLEG 31-36
    # ROBOT VIEWER # Free flyer0-5, RLEG 6-11, LLEG 12-17, CHEST HEAD 18-21, RARM 22-28, LARM 29-35
    qUrdf = mat_zeros(32);
    '''qUrdf[:4,0]  = q[12:16,0]; # chest-head
    qUrdf[4:11,0] = q[23:,0]; # larm
    qUrdf[11:18,0] = q[16:23,0]; # rarm
    qUrdf[18:24,0] = q[6:12,0]; # lleg
    qUrdf[24:,0]   = q[:6,0]; # rleg'''
    return qUrdf;

