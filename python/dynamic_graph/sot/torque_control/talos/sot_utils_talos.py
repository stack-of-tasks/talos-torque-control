# -*- coding: utf-8 -*-1
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

from __future__ import print_function

import os
from time import sleep

import numpy as np


class Bunch:
    def __init__(self, **kwds):
        self.__dict__.update(kwds)

    def __str__(self, prefix=""):
        res = ""
        for (key, value) in self.__dict__.iteritems():
            if (
                isinstance(value, np.ndarray)
                and len(value.shape) == 2
                and value.shape[0] > value.shape[1]
            ):
                res += prefix + " - " + key + ": " + str(value.T) + "\n"
            elif isinstance(value, Bunch):
                res += (
                    prefix + " - " + key + ":\n" + value.__str__(prefix + "    ") + "\n"
                )
            else:
                res += prefix + " - " + key + ": " + str(value) + "\n"
        return res[:-1]


def start_sot():
    os.system("rosservice call /start_dynamic_graph")


def stop_sot():
    os.system("rosservice call /stop_dynamic_graph")


def start_tracer(
    robot,
    estimator_ft,
    estimator_kin,
    torque_ctrl,
    traj_gen,
    ctrl_manager,
    inv_dyn,
    adm_ctrl,
):
    from dynamic_graph.sot.torque_control.create_entities_utils import create_tracer

    tracer = create_tracer(
        robot.device, traj_gen, estimator_ft, estimator_kin, inv_dyn, torque_ctrl
    )
    tracer.start()
    return tracer


def move_to_initial_configuration(traj_gen):
    traj_gen.moveJoint("lhy", 0, 4.0)
    traj_gen.moveJoint("lhp", 0, 4.0)
    traj_gen.moveJoint("lhr", 0.5, 4.0)
    traj_gen.moveJoint("lk", 1.7, 4.0)
    traj_gen.moveJoint("lap", 0, 4.0)
    traj_gen.moveJoint("lar", 0, 4.0)
    traj_gen.moveJoint("rhy", 0, 4.0)
    traj_gen.moveJoint("rhr", 0, 4.0)
    traj_gen.moveJoint("rhp", -0.6, 4)
    traj_gen.moveJoint("rk", 1.1, 4)
    traj_gen.moveJoint("rap", -0.6, 4)
    traj_gen.moveJoint("rar", 0, 4.0)


def go_to_position(traj_gen, q, T=10.0):
    # put the robot in position q
    # RLEG TO 0 **********************
    traj_gen.moveJoint("lhy", q[0], T)  # 0
    traj_gen.moveJoint("lhr", q[1], T)  # 1
    traj_gen.moveJoint("lhp", q[2], T)  # 2
    traj_gen.moveJoint("lk", q[3], T)  # 3
    traj_gen.moveJoint("lap", q[4], T)  # 4
    traj_gen.moveJoint("lar", q[5], T)  # 5

    # LLEG TO 0 **********************
    traj_gen.moveJoint("rhy", q[6], T)  # 6
    traj_gen.moveJoint("rhr", q[7], T)  # 7
    traj_gen.moveJoint("rhp", q[8], T)  # 8
    traj_gen.moveJoint("rk", q[9], T)  # 9
    traj_gen.moveJoint("rap", q[10], T)  # 10
    traj_gen.moveJoint("rar", q[11], T)  # 11

    # TORSO TO 0
    traj_gen.moveJoint("ty", q[12], T)  # 12
    traj_gen.moveJoint("tp", q[13], T)  # 13

    # RARM TO 0 **********************
    traj_gen.moveJoint("lsy", q[14], T)  # 14
    traj_gen.moveJoint("lsr", q[15], T)  # 15
    traj_gen.moveJoint("lay", q[16], T)  # 16
    traj_gen.moveJoint("le", q[17], T)  # 17
    traj_gen.moveJoint("lwy", q[18], T)  # 28
    traj_gen.moveJoint("lwp", q[19], T)  # 19
    traj_gen.moveJoint("lwr", q[20], T)  # 20
    traj_gen.moveJoint("lh", q[21], T)  # 21

    # LARM TO 0 **********************
    traj_gen.moveJoint("rsy", q[22], T)  # 22
    traj_gen.moveJoint("rsr", q[23], T)  # 23
    traj_gen.moveJoint("ray", q[24], T)  # 24
    traj_gen.moveJoint("re", q[25], T)  # 25
    traj_gen.moveJoint("rwy", q[26], T)  # 26
    traj_gen.moveJoint("rwp", q[27], T)  # 27
    traj_gen.moveJoint("rwr", q[28], T)  # 28
    traj_gen.moveJoint("rh", q[29], T)  # 29

    # HEAD TO 0
    traj_gen.moveJoint("hp", q[30], T)  # 30
    traj_gen.moveJoint("hy", q[31], T)  # 31


def smoothly_set_signal_to_zero(sig):
    v = np.array(sig.value)
    for i in range(40):
        v = 0.95 * v
        sig.value = tuple(v)
        sleep(1)
    print("Setting signal to zero")
    v[:] = 0.0
    sig.value = tuple(v)


def smoothly_set_signal(sig, final_value, duration=5.0, steps=500, prints=10):
    v = np.array(sig.value)
    vf = np.array(final_value)
    for i in range(steps + 1):
        alpha = 1.0 * i / steps
        sig.value = tuple(vf * alpha + (1 - alpha) * v)
        sleep(1.0 * duration / steps)
    print("Signal set")
    sig.value = tuple(final_value)


def monitor_tracking_error(sig, sigRef, dt, time):
    N = int(time / dt)
    err = np.zeros((N, 6))
    for i in range(N):
        err[i, :] = np.array(sig.value) - np.array(sigRef.value)
        sleep(dt)
    for i in range(6):
        print(
            "Max tracking error for axis %d:         %.2f"
            % (i, np.max(np.abs(err[:, i])))
        )
        print(
            "Mean square tracking error for axis %d: %.2f"
            % (i, np.linalg.norm(err[:, i]) / N)
        )


def dump_signal_to_file(sig_list, index, filename, T, dt):
    N = int(T / dt)
    # m = len(sig_list)
    f = open("/tmp/" + filename, "a", 1)
    for t in range(N):
        for s in sig_list:
            f.write("{0}\t".format(s.value[index]))
        f.write("\n")
        sleep(dt)
    f.close()


def go_to_position_sinusoid(robot):

    robot.position1 = (
        # Free flyer
        0.0,
        0.0,
        1.018213,
        0.0,
        0.0,
        0.0,
        # legs
        0.0,
        0.0,
        -0.411354,
        0.859395,
        -0.448041,
        -0.001708,
        0.0,
        0.0,
        -0.411354,
        0.859395,
        -0.448041,
        -0.001708,
        # Chest
        0.0,
        0.006761,
        # arms
        0.25847,
        0.173046,
        -0.0002,
        -0.525366,
        0.0,
        -0.0,
        0.1,
        -0.005,
        -0.25847,
        -0.173046,
        0.4,
        -1.3,
        -1.6,
        0.0,
        0.1,
        -0.005,
        # 0.25847 ,  0.173046, -0.4, -1.3, 1.6, -0.0,  0.1, -0.005,
        # -0.25847 , -0.173046, 0.0002  , -0.525366, 0.0,  0.0,  0.1,-0.005,
        # Head
        0.0,
        0.0,
    )

    go_to_position(robot.traj_gen, robot.position1[6:], 10.0)
    return robot


def start_movement_sinusoid(robot):

    robot.traj_gen.startSinusoid("re", -1.9, 3)
    return robot


def stop_movement_sinusoid(robot):
    robot.traj_gen.stop("re")
    return robot


def go_to_SE3_position_fixed_orientation(traj_gen, pos, T=10.0):
    traj_gen.move(0, pos[0], T)
    traj_gen.move(1, pos[1], T)
    traj_gen.move(2, pos[2], T)
    traj_gen.move(3, 0, T)
    traj_gen.move(4, 0, T)
    traj_gen.move(5, 1, T)
    traj_gen.move(6, 0, T)
    traj_gen.move(7, 1, T)
    traj_gen.move(8, 0, T)
    traj_gen.move(9, -1, T)
    traj_gen.move(10, 0, T)
    traj_gen.move(11, 0, T)


def go_to_SE3_front_orientation(traj_gen, T=10):
    traj_gen.move(3, 0, T)
    traj_gen.move(4, 0, T)
    traj_gen.move(5, 1, T)
    traj_gen.move(6, 0, T)
    traj_gen.move(7, 1, T)
    traj_gen.move(8, 0, T)
    traj_gen.move(9, -1, T)
    traj_gen.move(10, 0, T)
    traj_gen.move(11, 0, T)


def go_to_SE3_right_orientation(traj_gen, T=10):
    traj_gen.move(3, 0, T)
    traj_gen.move(4, 1, T)
    traj_gen.move(5, 0, T)
    traj_gen.move(6, 0, T)
    traj_gen.move(7, 0, T)
    traj_gen.move(8, -1, T)
    traj_gen.move(9, -1, T)
    traj_gen.move(10, 0, T)
    traj_gen.move(11, 0, T)


def go_to_SE3_left_orientation(traj_gen, T=10):
    traj_gen.move(3, 0, T)
    traj_gen.move(4, -1, T)
    traj_gen.move(5, 0, T)
    traj_gen.move(6, 0, T)
    traj_gen.move(7, 0, T)
    traj_gen.move(8, 1, T)
    traj_gen.move(9, -1, T)
    traj_gen.move(10, 0, T)
    traj_gen.move(11, 0, T)


def go_to_SE3_position(traj_gen, pos, T=10):
    traj_gen.move(0, pos[0], T)
    traj_gen.move(1, pos[1], T)
    traj_gen.move(2, pos[2], T)
