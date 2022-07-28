# -*- coding: utf-8 -*-
"""
Created on Mon Feb  9 13:55:16 2015

@author: adelpret
"""
from numpy import zeros as zeros

NJ = 32

k_s = zeros(NJ)
# joint proportional gains (used by InverseDynamicsController)
k_d = zeros(NJ)
# joint derivative gains  (used by InverseDynamicsController)
k_f = zeros(6 * 4)
# force proportional gains  (used by InverseDynamicsController)
k_i = zeros(6 * 4)
# force integral gains  (used by InverseDynamicsController)

k_f[:] = 0.5
k_i[:] = 0.0
# max value 0.015

k_s[0] = 1550
# 1550 is equivalent to pos ctrl (or maybe 1500?)
k_d[0] = 30
k_s[1] = 11100
# 11100 is equivalent to position ctrl
k_d[1] = 70
k_s[2] = 2800
# 2800 is equivalent to pos ctrl
k_d[2] = 30
k_s[3] = 6530
# 6530 is equivalent to pos ctrl
k_d[3] = 50
k_s[4] = 1900
# 1900 is equivalent to pos ctrl
k_d[4] = 20
k_s[5] = 1390
# 1390 is equivalent to pos ctrl
k_d[5] = 10
k_s = 0.05 * k_s
