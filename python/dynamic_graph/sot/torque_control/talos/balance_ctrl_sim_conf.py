from balance_ctrl_conf import *

# CONTROLLER GAINS
kd_posture  = NJ*(0*sqrt(kp_posture[0]),);
kd_constr   = 0.0*2*sqrt(kp_constr);   # constraint derivative feedback gain
kd_com      = 0.0*2*sqrt(kp_com);
kd_feet     = 0.0*2*sqrt(kp_feet);
