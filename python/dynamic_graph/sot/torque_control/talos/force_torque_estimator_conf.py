# -*- coding: utf-8 -*-
"""
Created on Mon Feb  9 13:55:16 2015

@author: adelpret
"""

CURRENT_TORQUE_ESTIMATION_TRUST = 0.0;      # weight for current-base torque estimation (in [0,1]) of ForceTorqueEstimator
SATURATION_CURRENT = 10.0;                  # Saturation of current sensor (used by ForceTorqueEstimator)
DELAY_ENC          = 40;                    # estimation delay expressed relative to the timestep (i.e. delay = DELAY_ENC*dt)
DELAY_ACC          = 40;                    # estimation delay expressed relative to the timestep (i.e. delay = DELAY_ACC*dt)
DELAY_GYRO         = 40;                    # estimation delay expressed relative to the timestep (i.e. delay = DELAY_ACC*dt)
DELAY_FORCE        = 40;                    # estimation delay expressed relative to the timestep (i.e. delay = DELAY_ACC*dt)
DELAY_CURRENT      = 40;                    # estimation delay expressed relative to the timestep (i.e. delay = DELAY_ACC*dt)
