from balance_ctrl_conf import *


""" *********************** USER-PARAMETERS *********************** """

# COM_DES = (0.00, 0.0, 0.875)

# CONTROLLER GAINS
NJ = 32
kp_posture = (
    10.0,
    5.0,
    5.0,
    1.0,
    1.0,
    10.0,
    10.0,
    5.0,
    5.0,
    1.0,
    1.0,
    10.0,
    500.0,
    500.0,
    50.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    50.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    10.0,
    100.0,
    100.0,
)  # proportional gain of postural task
kd_posture = tuple(2 * np.sqrt(kp_posture))

kp_pos = np.array(
    (
        1300.0,
        1300.0,
        1300.0,
        1300.0,
        1300.0,
        1300.0,
        1300.0,
        1300.0,
        1300.0,
        1300.0,
        1300.0,
        1300.0,
        100.0,
        100.0,
        500.0,
        500.0,
        500.0,
        500.0,
        500.0,
        500.0,
        500.0,
        10.0,
        500.0,
        500.0,
        500.0,
        500.0,
        500.0,
        500.0,
        500.0,
        10.0,
        10.0,
        10.0,
    )
)

kd_pos = np.array(
    (
        20.0,
        20.0,
        20.0,
        20.0,
        20.0,
        20.0,
        20.0,
        20.0,
        20.0,
        20.0,
        20.0,
        20.0,
        10.0,
        10.0,
        5.0,
        5.0,
        5.0,
        5.0,
        5.0,
        5.0,
        5.0,
        0.1,
        5.0,
        5.0,
        5.0,
        5.0,
        5.0,
        5.0,
        5.0,
        0.1,
        0.1,
        0.1,
    )
)

kp_contact = 30.0  # constraint proportional feedback gain
kd_contact = 2 * sqrt(kp_contact)  # constraint derivative feedback gain
kp_com = 20.0
kd_com = 2 * sqrt(kp_com)
kp_waist = 500.0
kd_waist = 2.0 * sqrt(kp_waist)

# # CONTROLLER WEIGTHS
w_com = 1.0
w_posture = 1e-1  # weight of postural task
w_forces = 1e-3
w_waist = 1.0

# CONTACT PARAMETERS
RIGHT_FOOT_SIZES = (0.1, -0.11, 0.069, -0.069)  # pos x, neg x, pos y, neg y size
LEFT_FOOT_SIZES = (0.1, -0.11, 0.069, -0.069)  # pos x, neg x, pos y, neg y size

RIGHT_FOOT_CONTACT_POINTS = (
    (
        RIGHT_FOOT_SIZES[0],
        RIGHT_FOOT_SIZES[0],
        RIGHT_FOOT_SIZES[1],
        RIGHT_FOOT_SIZES[1],
    ),
    (
        RIGHT_FOOT_SIZES[3],
        RIGHT_FOOT_SIZES[2],
        RIGHT_FOOT_SIZES[3],
        RIGHT_FOOT_SIZES[2],
    ),
    (-0.107, -0.107, -0.107, -0.107),
)  # contact points in local reference frame

LEFT_FOOT_CONTACT_POINTS = np.matrix(
    [
        [LEFT_FOOT_SIZES[0], LEFT_FOOT_SIZES[3], -0.107],
        [LEFT_FOOT_SIZES[0], LEFT_FOOT_SIZES[2], -0.107],
        [LEFT_FOOT_SIZES[1], LEFT_FOOT_SIZES[3], -0.107],
        [LEFT_FOOT_SIZES[1], LEFT_FOOT_SIZES[2], -0.107],
    ]
).T  # contact points in local reference frame
FOOT_CONTACT_NORMAL = (0.0, 0.0, 1.0)
mu = np.array([0.3, 0.1])  # force and moment friction coefficient
fMin = 1.0  # minimum normal force
fMax = 1e3  # maximum normal force

RF_FORCE_DES = (0, 0, 500, 0, 0, 0.0)
LF_FORCE_DES = (0, 0, 500, 0, 0, 0.0)
