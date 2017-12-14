from balance_ctrl_conf import *

# CONTROLLER GAINS
kd_posture  = NJ*(sqrt(kp_posture[0]),);
kd_constr   = 2*sqrt(kp_constr);   # constraint derivative feedback gain
kd_com      = 2*sqrt(kp_com);
kd_feet     = 2*sqrt(kp_feet);
