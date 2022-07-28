# -*- coding: utf-8 -*-
"""
Created on Mon Feb  9 13:55:16 2015

@author: adelpret
"""
from current_controller_conf import *

CURRENT_OFFSET_ITERS = 0
# Number of itertion while control is disabled to calibrate current sensors
CURRENT_MAX = 1e6
# max motor current (security check of ControlManager)
CTRL_MAX = 1e6
# max desired current (security check of ControlManager)
CTRL_SATURATION = 1e6
# saturation of the control signal
IN_OUT_GAIN = 1.0
# factor to convert from a [-20.0 ; 20.0] Ampers value to the [-2048 ; 2048] 12bit DAC register
percentage_dead_zone_compensation = NJ * [
    0.0,
]
# percentage of dead zone to compensate (used by ControlManager)
percentage_bemf_compensation = NJ * [
    0.0,
]
kp_current = NJ * [
    0.0,
]
ki_current = NJ * [
    0.0,
]
