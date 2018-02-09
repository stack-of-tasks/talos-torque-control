# -*- coding: utf-8 -*-
"""
Created on Mon Feb  9 13:55:16 2015

@author: adelpret
"""
from numpy import zeros as zeros

NJ = 32;


GEAR_RATIOS = (150.0, 101.0, 100.0, 144.0, 100.0, 101.0,
               150.0, 101.0, 100.0, 144.0, 100.0, 101.0,
               100.0, 100.0,
               100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
               100.0,
               100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
               100.0,
               100.0,200.0);
ROTOR_INERTIAS = (8.6e-6, 34e-6, 34.0e-5, 61.0e-5, 34.0e-6, 21.0e-6,
                  8.6e-6, 34e-6, 34.0e-5, 61.0e-5, 34.0e-6, 21.0e-6,
                  34.0e-6, 34.0e-6,
                  34.0e-6, 34.0e-6, 21.0e-6, 21.0e-6, 5.8e-6, 1.3e-6, 1.3e-07,
                  1.3,
                  34.0e-6, 34.0e-6, 21.0e-6, 21.0e-6, 5.8e-6, 1.3e-6, 1.3e-07,
                  1.3,
                  1.3,1.3);


### New motor parameters with current measurment (used by ForceTorqueEstimator and JointTorqueController) ###
Kt_p=zeros(NJ)+1.0;
Kt_n=zeros(NJ)+1.0;
Kf_p=zeros(NJ);
Kf_n=zeros(NJ);
Kv_p=zeros(NJ);
Kv_n=zeros(NJ);
Ka_p=zeros(NJ);
Ka_n=zeros(NJ);

deadzone=zeros(NJ);
K_bemf=zeros(NJ);                 # used by ControlManager to compensate back-EMF
cur_sens_gains = zeros(NJ)+1.0;

cur_sens_gains[0] = 1.256534 #Using 20161114_135332_rhy_static
deadzone[0]      = 0.588671 #Using 20161114_135332_rhy_static
deadzone[0]      = 0.575506 #Using 20161114_143152_rhy_vel
K_bemf[0]          = 1.169309 # [Amp/Rad.s-1] Using 20161114_143152_rhy_vel
Kt_p[0]          = 0.055302 #Using 20161114_135332_rhy_static
Kt_n[0]          = 0.053055 #Using 20161114_135332_rhy_static
Kv_p[0]          = 0.777466 #Using 20161114_143152_rhy_vel
Kv_n[0]          = 0.779525 #Using 20161114_143152_rhy_vel
Kf_p[0]          = 0.463513 #Using 20161114_143152_rhy_vel
Kf_n[0]          = 0.231212 #Using 20161114_143152_rhy_vel

cur_sens_gains[1] = 0.997354 #Using 20161114_144232_rhr_static
deadzone[1]      = 0.571099 #Using 20161114_144232_rhr_static
deadzone[1]      = 0.564055 #Using 20161114_150356_rhr_vel
K_bemf[1]          = 1.120005 # [Amp/Rad.s-1] Using 20161114_150356_rhr_vel
Kt_p[1]          = 0.061092 #Using 20161114_144232_rhr_static
Kt_n[1]          = 0.057921 #Using 20161114_144232_rhr_static
Kv_p[1]          = 0.583937 #Using 20161114_150356_rhr_vel
Kv_n[1]          = 0.471164 #Using 20161114_150356_rhr_vel
Kf_p[1]          = 0.139620 #Using 20161114_150356_rhr_vel
Kf_n[1]          = 0.622044 #Using 20161114_150356_rhr_vel

cur_sens_gains[2] = 0.884396 #Using 20161114_150722_rhp_static
deadzone[2]      = 0.671750 #Using 20161114_150722_rhp_static
deadzone[2]      = 0.461780 #Using 20161114_151812_rhp_vel
K_bemf[2]          = 1.171485 # [Amp/Rad.s-1] Using 20161114_151812_rhp_vel
Kt_p[2]          = 0.093924 #Using 20161114_150722_rhp_static
Kt_n[2]          = 0.074335 #Using 20161114_150722_rhp_static
Kv_p[2]          = 0.236554 #Using 20161114_151812_rhp_vel
Kv_n[2]          = 0.200306 #Using 20161114_151812_rhp_vel
Kf_p[2]          = 0.322370 #Using 20161114_151812_rhp_vel
Kf_n[2]          = 0.955010 #Using 20161114_151812_rhp_vel

cur_sens_gains[3] = 0.898618 #Using 20161114_152140_rk_static
deadzone[3]      = 0.611894 #Using 20161114_152140_rk_static
deadzone[3]      = 0.594111 #Using 20161114_153220_rk_vel
K_bemf[3]          = 1.062028 # [Amp/Rad.s-1] Using 20161114_153220_rk_vel
Kt_p[3]          = 0.074025 #Using 20161114_152140_rk_static
Kt_n[3]          = 0.070763 #Using 20161114_152140_rk_static
Kv_p[3]          = 0.310712 #Using 20161114_153220_rk_vel
Kv_n[3]          = 0.302653 #Using 20161114_153220_rk_vel
Kf_p[3]          = 0.562304 #Using 20161114_153220_rk_vel
Kf_n[3]          = 0.590003 #Using 20161114_153220_rk_vel

cur_sens_gains[4] = 0.989997 #Using 20161114_153739_rap_static
deadzone[4]      = 0.485198 #Using 20161114_153739_rap_static
deadzone[4]      = 0.615647 #Using 20161114_154559_rap_vel
K_bemf[4]          = 0.787735 # [Amp/Rad.s-1] Using 20161114_154559_rap_vel
Kt_p[4]          = 0.082806 #Using 20161114_153739_rap_static
Kt_n[4]          = 0.088764 #Using 20161114_153739_rap_static
Kv_p[4]          = 0.222277 #Using 20161114_154559_rap_vel
Kv_n[4]          = 0.225662 #Using 20161114_154559_rap_vel
Kf_p[4]          = 0.559252 #Using 20161114_154559_rap_vel
Kf_n[4]          = 0.259040 #Using 20161114_154559_rap_vel

cur_sens_gains[5] = 1.007775 #Using 20161114_154945_rar_static
deadzone[5]      = 0.595250 #Using 20161114_154945_rar_static
deadzone[5]      = 0.617705 #Using 20161114_160038_rar_vel
K_bemf[5]          = 0.456973 # [Amp/Rad.s-1] Using 20161114_160038_rar_vel
Kt_p[5]          = 0.155565 #Using 20161114_154945_rar_static
Kt_n[5]          = 0.156830 #Using 20161114_154945_rar_static
Kv_p[5]          = 0.511848 #Using 20161114_160038_rar_vel
Kv_n[5]          = 0.517686 #Using 20161114_160038_rar_vel
Kf_p[5]          = 0.564020 #Using 20161114_160038_rar_vel
Kf_n[5]          = 0.321447 #Using 20161114_160038_rar_vel

cur_sens_gains[6]= 1.250000 #Using 20171002_163413_lhy_static (forced by hand)
deadzone[6]      = 0.295741 #Using 20171002_163413_lhy_static
deadzone[6]      = 0.627764 #Using 20171002_151718_lhy_vel
K_bemf[6]        = 1.168112 # [Amp/Rad.s-1] Using 20171002_151718_lhy_vel
Kt_p[6]          = 0.057042 #Using 20171002_163413_lhy_static
Kt_n[6]          = 0.054307 #Using 20171002_163413_lhy_static
Kv_p[6]          = 0.833485 #Using 20171002_151718_lhy_vel
Kv_n[6]          = 0.835893 #Using 20171002_151718_lhy_vel
Kf_p[6]          = 0.444239 #Using 20171002_151718_lhy_vel
Kf_n[6]          = 0.251280 #Using 20171002_151718_lhy_vel

cur_sens_gains[7]= 1.004305 #Using 20171002_164436_lhr_static
deadzone[7]      = 0.640682 #Using 20171002_164436_lhr_static
deadzone[7]      = 0.601328 #Using 20171002_153334_lhr_vel
K_bemf[7]        = 1.161687 # [Amp/Rad.s-1] Using 20171002_153334_lhr_vel
Kt_p[7]          = 0.069472 #Using 20171002_164436_lhr_static
Kt_n[7]          = 0.058696 #Using 20171002_164436_lhr_static
Kv_p[7]          = 0.363950 #Using 20171002_153334_lhr_vel
Kv_n[7]          = 0.421599 #Using 20171002_153334_lhr_vel
Kf_p[7]          = 0.606632 #Using 20171002_153334_lhr_vel
Kf_n[7]          = 0.095194 #Using 20171002_153334_lhr_vel

cur_sens_gains[8]= 0.901796 #Using 20171002_165335_lhp_static
deadzone[8]      = 0.456045 #Using 20171002_165335_lhp_static
deadzone[8]      = 0.607772 #Using 20171002_154449_lhp_vel
K_bemf[8]        = 0.975598 # [Amp/Rad.s-1] Using 20171002_154449_lhp_vel
Kt_p[8]          = 0.076617 #Using 20171002_165335_lhp_static
Kt_n[8]          = 0.087909 #Using 20171002_165335_lhp_static
Kv_p[8]          = 0.122464 #Using 20171002_154449_lhp_vel
Kv_n[8]          = 0.181315 #Using 20171002_154449_lhp_vel
Kf_p[8]          = 0.597530 #Using 20171002_154449_lhp_vel
Kf_n[8]          = 0.375285 #Using 20171002_154449_lhp_vel

cur_sens_gains[9]= 0.897802 #Using 20170113_151748_lk_static
deadzone[9]      = 0.628737 #Using 20170113_151748_lk_static
deadzone[9]      = 0.621934 #Using 20170113_152924_lk_const_vel
K_bemf[9]        = 1.057187 # [Amp/Rad.s-1] Using 20170113_152924_lk_const_vel
Kt_p[9]          = 0.071932 #Using 20170113_151748_lk_static
Kt_n[9]          = 0.065418 #Using 20170113_151748_lk_static
Kv_p[9]          = 0.331607 #Using 20170113_152924_lk_const_vel
Kv_n[9]          = 0.351136 #Using 20170113_152924_lk_const_vel
Kf_p[9]          = 0.363123 #Using 20170113_152924_lk_const_vel
Kf_n[9]          = 0.699079 #Using 20170113_152924_lk_const_vel

cur_sens_gains[10]= 0.978598 #Using 20170113_154007_lap_static
deadzone[10]      = 0.577901 #Using 20170113_154007_lap_static
deadzone[10]      = 0.582928 #Using 20170113_154834_lap_const_vel
K_bemf[10]        = 0.806259 # [Amp/Rad.s-1] Using 20170113_154834_lap_const_vel
Kt_p[10]          = 0.092789 #Using 20170113_154007_lap_static
Kt_n[10]          = 0.093060 #Using 20170113_154007_lap_static
Kv_p[10]          = 0.542156 #Using 20170113_154834_lap_const_vel
Kv_n[10]          = 0.469827 #Using 20170113_154834_lap_const_vel
Kf_p[10]          = 0.063421 #Using 20170113_154834_lap_const_vel
Kf_n[10]          = 0.806020 #Using 20170113_154834_lap_const_vel

cur_sens_gains[11]= 0.995794 #Using 20170113_155150_lar_static
deadzone[11]      = 0.643120 #Using 20170113_155150_lar_static
deadzone[11]      = 0.645305 #Using 20170113_160057_lar_const_vel
K_bemf[11]        = 0.474213 # [Amp/Rad.s-1] Using 20170113_160057_lar_const_vel
Kt_p[11]          = 0.147175 #Using 20170113_155150_lar_static
Kt_n[11]          = 0.150098 #Using 20170113_155150_lar_static
Kv_p[11]          = 0.342627 #Using 20170113_160057_lar_const_vel
Kv_n[11]          = 0.351680 #Using 20170113_160057_lar_const_vel
Kf_p[11]          = 0.263935 #Using 20170113_160057_lar_const_vel
Kf_n[11]          = 0.430722 #Using 20170113_160057_lar_const_vel

# take averages for p and n
for i in range(NJ):
    Kt_av = (Kt_n[i] + Kt_p[i])/2
    Kt_n[i]=Kt_av
    Kt_p[i]=Kt_av
    
    Kv_av = (Kv_n[i] + Kv_p[i])/2
    Kv_n[i]=Kv_av
    Kv_p[i]=Kv_av
    
    Kf_av = (Kf_n[i] + Kf_p[i])/2
    Kf_n[i]=Kf_av
    Kf_p[i]=Kf_av

