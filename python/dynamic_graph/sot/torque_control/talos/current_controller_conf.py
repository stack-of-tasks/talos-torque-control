# -*- coding: utf-8 -*-
"""
Created on Mon Feb  9 13:55:16 2015

@author: adelpret
"""
import numpy as np

NJ = 32
CURRENT_OFFSET_ITERS = 200
# Number of itertion while control is disabled to calibrate current sensors
CURRENT_MAX = 8.0
# max motor current (security check of ControlManager)
CTRL_MAX = 10.0
# max desired current (security check of ControlManager)
CTRL_SATURATION = 2047
# saturation of the control signal
IN_OUT_GAIN = 102.4
# factor to convert from a [-20.0 ; 20.0] Ampers value
# to the [-2048 ; 2048] 12bit DAC register
percentage_dead_zone_compensation = NJ * [
    0.0,
]
# percentage of dead zone to compensate (used by ControlManager)
i_max_dz_comp = NJ * [
    0.05,
]
# value of current tracking error at which deadzone is completely compensated
percentage_bemf_compensation = NJ * [
    0.0,
]
current_sensor_offsets_low_level = NJ * [
    0.0,
]
kp_current = NJ * [
    0.0,
]
ki_current = np.array(
    NJ
    * [
        0.0,
    ]
)

percentage_bemf_compensation[0] = 0.9
percentage_bemf_compensation[1] = 0.9  # a bit unstable
percentage_bemf_compensation[2] = 0.9
percentage_bemf_compensation[3] = 0.9
percentage_bemf_compensation[4] = 1.0
percentage_bemf_compensation[5] = 0.9
percentage_bemf_compensation[6] = 0.9
percentage_bemf_compensation[7] = 0.9
percentage_bemf_compensation[8] = 0.9
percentage_bemf_compensation[9] = 0.9
percentage_bemf_compensation[10] = 0.9
percentage_bemf_compensation[11] = 0.9

percentage_dead_zone_compensation[0] = 0.8
percentage_dead_zone_compensation[1] = 0.8
percentage_dead_zone_compensation[2] = 0.8  # could go up to 1.2
percentage_dead_zone_compensation[3] = 0.8
percentage_dead_zone_compensation[4] = 0.8  # a bit unstable
percentage_dead_zone_compensation[5] = 0.8
percentage_dead_zone_compensation[6] = 0.8
percentage_dead_zone_compensation[7] = 0.8
percentage_dead_zone_compensation[8] = 0.8
percentage_dead_zone_compensation[9] = 0.8
# very asymettric error, a bit unstable on negative currents, may need investigation
percentage_dead_zone_compensation[10] = 0.8
# getting some vibrations for negative errors, may need investigation
percentage_dead_zone_compensation[11] = 0.8

i_max_dz_comp[0] = 0.03

current_sensor_offsets_low_level[0] = 0.04
current_sensor_offsets_low_level[2] = -0.05
current_sensor_offsets_low_level[5] = 0.02
current_sensor_offsets_low_level[6] = 0.01
current_sensor_offsets_low_level[7] = 0.01
current_sensor_offsets_low_level[8] = -0.01
current_sensor_offsets_low_level[10] = -0.11
current_sensor_offsets_low_level[11] = -0.04

ki_current[0] = 3e-3  # could go higher
ki_current[1] = 3e-3  # could go higher
ki_current[2] = 1e-3  # could probably go higher
ki_current[3] = 3e-3
ki_current[4] = 1e-3  # could probably go higher
ki_current[5] = 3e-3  # could probably go higher
ki_current[6] = 1e-3
ki_current[7] = 1e-3
ki_current[8] = 1e-3
ki_current[9] = 1e-3
ki_current[10] = 1e-3
ki_current[11] = 1e-3
# ki_current[:] = 0.0
