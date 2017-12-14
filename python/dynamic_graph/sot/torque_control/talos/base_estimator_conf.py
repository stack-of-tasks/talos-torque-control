''' *********************** USER-PARAMETERS OF BASE ESTIMATOR *********************** '''
K                   = (4034, 23770, 239018, 707, 502, 936);
std_dev_zmp         = 0.02
std_dev_fz          = 50.
normal_force_margin = 30.
zmp_margin          = 0.002
w_imu               = 1.0;
beta                = 0.00329
K_fb_feet_poses     = 0.0;      # gain used for updating foot positions
RIGHT_FOOT_SIZES    = (0.130,  -0.100,  0.056,  -0.075); # pos x, neg x, pos y, neg y size 
LEFT_FOOT_SIZES     = (0.130, -0.100,  0.075, -0.056); # pos x, neg x, pos y, neg y size 
w_lf_in             = 1.0
w_rf_in             = 1.0
#mu                  = 0.3;          # force friction coefficient
