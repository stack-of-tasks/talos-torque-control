# -*- coding: utf-8 -*-
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

from pinocchio.utils import zero as mat_zeros
from pinocchio import Quaternion
from pinocchio.rpy import rpyToMatrix

def config_sot_to_urdf(q):
    qUrdf = mat_zeros(39);
    qUrdf[:3,0] = q[:3,0];
    quatMat = rpyToMatrix(q[3:6,0]);
    quatVec = Quaternion(quatMat);
    qUrdf[3:7,0]   = quatVec.coeffs();
    qUrdf[7:,0]  = q[6:,0];
    return qUrdf;
    
def joints_sot_to_urdf(q):
    qUrdf = mat_zeros(32);
    qUrdf = q
    return qUrdf;

