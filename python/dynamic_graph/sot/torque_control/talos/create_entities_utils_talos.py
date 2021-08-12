# -*- coding: utf-8 -*-1
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

from __future__ import print_function

from dynamic_graph import plug
import numpy as np
from dynamic_graph.sot.core.latch import Latch
from dynamic_graph.sot.core.operator import Selec_of_vector, Mix_of_vector
from dynamic_graph.sot.torque_control.numerical_difference import NumericalDifference
from dynamic_graph.sot.torque_control.joint_torque_controller import JointTorqueController
from dynamic_graph.sot.torque_control.joint_trajectory_generator import JointTrajectoryGenerator
from dynamic_graph.sot_talos_balance.nd_trajectory_generator import NdTrajectoryGenerator
from dynamic_graph.sot.torque_control.se3_trajectory_generator import SE3TrajectoryGenerator
from dynamic_graph.sot.torque_control.control_manager import ControlManager
from dynamic_graph.sot.torque_control.current_controller import CurrentController
from dynamic_graph.sot_talos_balance.simple_admittance_controller import SimpleAdmittanceController as AdmittanceController
from dynamic_graph.sot.torque_control.position_controller import PositionController
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.sot.torque_control.talos.motors_parameters import NJ
from dynamic_graph.sot.torque_control.talos.motors_parameters import *
from dynamic_graph.sot.torque_control.talos.sot_utils_talos import Bunch
from dynamic_graph.sot.torque_control.utils.filter_utils import create_butter_lp_filter_Wn_05_N_3
from dynamic_graph.sot.core.operator import MatrixHomoToSE3Vector
from dynamic_graph.sot.core.parameter_server import ParameterServer

def get_default_conf():
    import dynamic_graph.sot.torque_control.talos.balance_ctrl_sim_conf as balance_ctrl_conf
    import dynamic_graph.sot_talos_balance.talos.base_estimator_conf as base_estimator_conf
    import dynamic_graph.sot.torque_control.talos.control_manager_conf as control_manager_conf
    import dynamic_graph.sot.torque_control.talos.current_controller_conf as current_controller_conf
    import dynamic_graph.sot.torque_control.talos.force_torque_estimator_conf as force_torque_estimator_conf
    import dynamic_graph.sot.torque_control.talos.joint_torque_controller_conf as joint_torque_controller_conf
    import dynamic_graph.sot.torque_control.talos.joint_pos_ctrl_gains as pos_ctrl_gains
    import dynamic_graph.sot_talos_balance.motor_parameters as motor_params
    import dynamic_graph.sot.torque_control.talos.ddp_controller_conf as ddp_controller_conf
    conf = Bunch()
    conf.balance_ctrl              = balance_ctrl_conf
    conf.base_estimator            = base_estimator_conf
    conf.control_manager           = control_manager_conf
    conf.current_ctrl              = current_controller_conf
    conf.force_torque_estimator    = force_torque_estimator_conf
    conf.joint_torque_controller   = joint_torque_controller_conf
    conf.pos_ctrl_gains            = pos_ctrl_gains
    conf.motor_params              = motor_params
    conf.ddp_controller            = ddp_controller_conf
    return conf

def get_sim_conf():
    import dynamic_graph.sot.torque_control.talos.balance_ctrl_sim_conf as balance_ctrl_conf
    # import dynamic_graph.sot.torque_control.talos.base_estimator_sim_conf as base_estimator_conf
    import dynamic_graph.sot_talos_balance.talos.base_estimator_conf as base_estimator_conf
    import dynamic_graph.sot.torque_control.talos.control_manager_sim_conf as control_manager_conf
    import dynamic_graph.sot.torque_control.talos.current_controller_sim_conf as current_controller_conf
    import dynamic_graph.sot.torque_control.talos.force_torque_estimator_conf as force_torque_estimator_conf
    import dynamic_graph.sot.torque_control.talos.joint_torque_controller_conf as joint_torque_controller_conf
    import dynamic_graph.sot.torque_control.talos.joint_pos_ctrl_gains_sim as pos_ctrl_gains
    import dynamic_graph.sot_talos_balance.motor_parameters as motor_params
    import dynamic_graph.sot.torque_control.talos.ddp_controller_conf as ddp_controller_conf
    conf = Bunch()
    conf.balance_ctrl              = balance_ctrl_conf
    conf.base_estimator            = base_estimator_conf
    conf.control_manager           = control_manager_conf
    conf.current_ctrl              = current_controller_conf
    conf.force_torque_estimator    = force_torque_estimator_conf
    conf.joint_torque_controller   = joint_torque_controller_conf
    conf.pos_ctrl_gains            = pos_ctrl_gains
    conf.motor_params              = motor_params
    conf.ddp_controller            = ddp_controller_conf
    return conf

def create_encoders(robot):
    encoders = Selec_of_vector('qn')
    plug(robot.device.robotState,     encoders.sin);
    encoders.selec(6,NJ+6);
    return encoders

def create_encoders_velocity(robot):
    encoders = Selec_of_vector('dqn')
    plug(robot.device.robotVelocity,     encoders.sin);
    encoders.selec(6,NJ+6);
    return encoders

def create_joint_pos_selector(robot, conf):
    encoders = Selec_of_vector('selecDdpJointPos')
    plug(robot.device.robotState,     encoders.sin);
    encoders.selec(conf.controlled_joint+6, conf.controlled_joint+7);
    return encoders

def create_joint_vel_selector(robot, conf):
    encoders = Selec_of_vector('selecDdpJointVel')
    plug(robot.device.robotVelocity,     encoders.sin);
    encoders.selec(conf.controlled_joint+6, conf.controlled_joint+7);
    return encoders

def create_joint_torque_selector(robot, conf):
    encoders = Selec_of_vector('selecDdpJointTorque')
    plug(robot.device.ptorque,     encoders.sin);
    encoders.selec(conf.controlled_joint, conf.controlled_joint+1);
    return encoders

def create_pos_des_selector(robot, conf):
    encoders = Selec_of_vector('selecDdpJointPosDes')
    plug(robot.traj_gen.q,     encoders.sin);
    encoders.selec(conf.controlled_joint, conf.controlled_joint+1);
    return encoders

def create_motor_pos_selector(robot, conf):
    encoders = Selec_of_vector('selecDdpMotorPos')
    plug(robot.device.motor_angles,     encoders.sin);
    encoders.selec(conf.controlled_joint, conf.controlled_joint+1);
    return encoders

def create_tau_des_selector(robot, conf):
    encoders = Selec_of_vector('selecDdpTauDes')
    plug(robot.inv_dyn.tau_des,     encoders.sin);
    encoders.selec(conf.controlled_joint, conf.controlled_joint+1);
    return encoders

def create_torque_des_selector(robot, conf):
    encoders = Selec_of_vector('selecDdpTorqueDes')
    plug(robot.torque_ctrl.u,     encoders.sin);
    encoders.selec(0, 31);
    return encoders

def create_torque_des_selector2(robot, conf):
    encoders = Selec_of_vector('selecDdpTorqueDes2')
    plug(robot.torque_ctrl.u,     encoders.sin);
    encoders.selec(31, 32);
    return encoders

def create_signal_mixer(robot, conf):
    signal_mixer = Mix_of_vector('mix');
    signal_mixer.setSignalNumber(2);
    plug(robot.torque_des_selec_ddp.sout,      signal_mixer.default);
    #plug(robot.inv_dyn.tau_des,                signal_mixer.default);
    plug(robot.ddp_ctrl.tau,                   signal_mixer.sin1);
    #plug(robot.torque_des_selec_ddp2.sout,     signal_mixer.sin1);
    #plug(robot.inv_dyn.tau_des,                signal_mixer.sin1);

    #signal_mixer.addSelec(1, 1, 31);
    signal_mixer.addSelec(1, 0, 1);

    #signal_mixer.addSelec(1, conf.controlled_joint+1, conf.NJ-conf.controlled_joint);
    #plug(signal_mixer.sout,   robot.torque_ctrl.jointsTorquesDesired);
    return signal_mixer

def create_base_estimator(robot, dt, conf, robot_name="robot"):
    from dynamic_graph.sot.torque_control.base_estimator import BaseEstimator
    base_estimator = BaseEstimator('base_estimator');
    plug(robot.encoders.sout,               base_estimator.joint_positions);
    #plug(robot.device.forceRLEG,            base_estimator.forceRLEG);
    #plug(robot.device.forceLLEG,            base_estimator.forceLLEG);
    plug(robot.filters.ft_LF_filter.x_filtered, base_estimator.forceLLEG)
    plug(robot.filters.ft_RF_filter.x_filtered, base_estimator.forceRLEG)
    plug(robot.filters.ft_LF_filter.dx,         base_estimator.dforceLLEG)
    plug(robot.filters.ft_RF_filter.dx,         base_estimator.dforceRLEG)
    plug(robot.filters.estimator_kin.dx,            base_estimator.joint_velocities);
    plug(robot.imu_filter.imu_quat,         base_estimator.imu_quaternion);
    #plug(robot.imu_offset_compensation.accelerometer_out, base_estimator.accelerometer);
    #plug(robot.imu_offset_compensation.gyrometer_out,     base_estimator.gyroscope);
    plug(robot.filters.gyro_filter.x_filtered,             base_estimator.gyroscope);
    plug(robot.filters.acc_filter.x_filtered,              base_estimator.accelerometer);
    base_estimator.K_fb_feet_poses.value = conf.K_fb_feet_poses;
    try:
        base_estimator.w_lf_in.value = conf.w_lf_in;
        base_estimator.w_rf_in.value = conf.w_rf_in;
    except:
        pass;

    base_estimator.set_imu_weight(conf.w_imu);
    base_estimator.set_stiffness_right_foot(conf.K);
    base_estimator.set_stiffness_left_foot(conf.K);
    base_estimator.set_zmp_std_dev_right_foot(conf.std_dev_zmp)
    base_estimator.set_zmp_std_dev_left_foot(conf.std_dev_zmp)
    base_estimator.set_normal_force_std_dev_right_foot(conf.std_dev_fz)
    base_estimator.set_normal_force_std_dev_left_foot(conf.std_dev_fz)
    base_estimator.set_zmp_margin_right_foot(conf.zmp_margin)
    base_estimator.set_zmp_margin_left_foot(conf.zmp_margin)
    base_estimator.set_normal_force_margin_right_foot(conf.normal_force_margin)
    base_estimator.set_normal_force_margin_left_foot(conf.normal_force_margin)
    base_estimator.set_right_foot_sizes(conf.RIGHT_FOOT_SIZES)
    base_estimator.set_left_foot_sizes(conf.LEFT_FOOT_SIZES)

    base_estimator.init(dt, robot_name);
    return base_estimator;

def create_imu_offset_compensation(robot, dt):
    from dynamic_graph.sot.torque_control.imu_offset_compensation import ImuOffsetCompensation
    imu_offset_compensation = ImuOffsetCompensation('imu_offset_comp');
    plug(robot.device.accelerometer, imu_offset_compensation.accelerometer_in);
    plug(robot.device.gyrometer,     imu_offset_compensation.gyrometer_in);
    imu_offset_compensation.init(dt);
    return imu_offset_compensation;

def create_imu_filter(robot, dt):
    from dynamic_graph.sot.core.madgwickahrs import MadgwickAHRS
    imu_filter = MadgwickAHRS('imu_filter')
    imu_filter.init(dt);
    plug(robot.imu_offset_compensation.accelerometer_out, imu_filter.accelerometer);
    plug(robot.imu_offset_compensation.gyrometer_out,     imu_filter.gyroscope);
    return imu_filter;

def create_com_traj_gen(robot, dt, init_value=None):
    com_traj_gen = NdTrajectoryGenerator("com_traj_gen")
    if init_value is None:
        init_value = robot.dynamic.com.value
    com_traj_gen.initial_value.value = init_value
    com_traj_gen.trigger.value = 1
    com_traj_gen.init(dt,3)
    return com_traj_gen

def create_am_traj_gen(robot, dt, init_value=None):
    am_traj_gen = NdTrajectoryGenerator("am_traj_gen")
    robot.dynamic.angularmomentum.recompute(0)
    if init_value is None:
        init_value = robot.dynamic.angularmomentum.value
    am_traj_gen.initial_value.value = init_value
    am_traj_gen.trigger.value = 1
    am_traj_gen.init(dt,3)
    return am_traj_gen

def create_force_traj_gen(name, initial_value, dt):
    force_traj_gen = NdTrajectoryGenerator(name)
    force_traj_gen.initial_value.value = initial_value
    force_traj_gen.trigger.value = 1
    force_traj_gen.init(dt,6);
    return force_traj_gen ;

def create_foot_traj_gen(name, jointId, robot, dt):
    foot_traj_gen = NdTrajectoryGenerator(name)
    ref_foot = robot.dynamic.data.oMi[robot.dynamic.model.getJointId(jointId)]
    trans = ref_foot.translation
    rot = ref_foot.rotation
    rot = rot.reshape(9)
    initial_value = np.concatenate((trans,rot))
    foot_traj_gen.initial_value.value = initial_value
    foot_traj_gen.trigger.value = 1
    foot_traj_gen.init(dt, 12)
    return foot_traj_gen 


def create_waist_traj_gen(name, robot, dt):
    waist_traj_gen = SE3TrajectoryGenerator(name)
    ref_waist = robot.dynamic.data.oMi[robot.dynamic.model.getJointId('root_joint')]
    trans = ref_waist.translation
    rot = ref_waist.rotation
    rot = rot.reshape(9)
    initial_value = np.concatenate((trans,rot))
    waist_traj_gen.initial_value.value = initial_value
    waist_traj_gen.trigger.value = 1
    waist_traj_gen.init(dt);
    return waist_traj_gen;

def create_trajectory_switch():
    traj_sync = Latch("traj_sync");
    return traj_sync ;

def connect_synchronous_trajectories(switch, list_of_traj_gens):
  for traj_gen in list_of_traj_gens:
    plug(switch.out, traj_gen.trigger);

def create_free_flyer_locator(ent, robot_name="robot"):
    from dynamic_graph.sot.torque_control.free_flyer_locator import FreeFlyerLocator
    ff_locator = FreeFlyerLocator("ffLocator");
    plug(ent.device.robotState,             ff_locator.base6d_encoders);
    plug(ent.filters.estimator_kin.dx,              ff_locator.joint_velocities);
    try:
        plug(ff_locator.base6dFromFoot_encoders, ent.dynamic.position);
    except:
        print("[WARNING] Could not connect to dynamic entity, probably because you are in simulation")
    ff_locator.init(robot_name)
    return ff_locator

def create_flex_estimator(robot, dt=0.001):
    from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force import HRP2ModelBaseFlexEstimatorIMUForce
    flex_est = HRP2ModelBaseFlexEstimatorIMUForce(robot, useMocap=False, dt=dt);
    flex_est.setOn(False);
    flex_est.interface.setExternalContactPresence(False);
    flex_est.interface.enabledContacts_lf_rf_lh_rh.value=(1,1,0,0);
    plug(robot.ff_locator.v, flex_est.leftFootVelocity.sin2);
    plug(robot.ff_locator.v, flex_est.rightFootVelocity.sin2);
    plug(robot.ff_locator.v, flex_est.inputVel.sin2);
    plug(robot.ff_locator.v, flex_est.DCom.sin2);
    return flex_est;

def create_floatingBase(robot):
    from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force import FromLocalToGLobalFrame
    floatingBase = FromLocalToGLobalFrame(robot.flex_est, "FloatingBase")
    plug(robot.ff_locator.freeflyer_aa, floatingBase.sinPos);

    from dynamic_graph.sot.core import Selec_of_vector
    base_vel_no_flex = Selec_of_vector('base_vel_no_flex');
    plug(robot.ff_locator.v, base_vel_no_flex.sin);
    base_vel_no_flex.selec(0, 6);
    plug(base_vel_no_flex.sout,   floatingBase.sinVel);
    return floatingBase

def create_position_controller(robot, gains, dt=0.001, robot_name="robot"):
    posCtrl = PositionController('pos_ctrl')
    posCtrl.Kp.value = tuple(gains.kp_pos[round(dt,3)]);
    posCtrl.Kd.value = tuple(gains.kd_pos[round(dt,3)]);
    posCtrl.Ki.value = tuple(gains.ki_pos[round(dt,3)]);
    posCtrl.dqRef.value = NJ*(0.0,);
    plug(robot.device.robotState,             posCtrl.base6d_encoders);
    try:  # this works only in simulation
        #plug(robot.device.jointsVelocities,    posCtrl.jointsVelocities);
        plug(robot.encoders_velocity.sout,    posCtrl.jointsVelocities);
    except:
        plug(robot.filters.estimator_kin.dx, posCtrl.jointsVelocities);
        pass;
    plug(posCtrl.pwmDes,                robot.device.control);
    try:
        plug(robot.traj_gen.q,       posCtrl.qRef);
    except:
        pass;
    posCtrl.init(dt, robot_name);
    return posCtrl;

def create_joint_trajectory_generator(robot, dt):
    jtg = NdTrajectoryGenerator("jtg")
    jtg.initial_value.value = robot.device.robotState.value[6:]
    jtg.trigger.value = 1
    jtg.init(dt, NJ)
    return jtg

def create_trajectory_generator(robot, dt=0.001, robot_name="robot"):
    jtg = JointTrajectoryGenerator("jtg");
    # jtg.base6d_encoders.value = np.array(robot.halfSitting)
    plug(robot.device.robotState, jtg.base6d_encoders);
    jtg.init(dt, robot_name);
    return jtg;

def create_filters(robot, conf, motor_params, dt):
    filters = Bunch()

    # create low-pass filter for motor currents
    filters.current_filter = create_butter_lp_filter_Wn_05_N_3('current_filter', dt, NJ)

    #filters.current_filter = NumericalDifference("current_filter");
    filters.ft_RF_filter = NumericalDifference("ft_RF_filter");
    filters.ft_LF_filter = NumericalDifference("ft_LF_filter");
    filters.ft_RH_filter = NumericalDifference("ft_RH_filter");
    filters.ft_LH_filter = NumericalDifference("ft_LH_filter");
    filters.acc_filter   = NumericalDifference("dv_filter");
    filters.gyro_filter  = NumericalDifference("w_filter");

    filters.estimator_kin = NumericalDifference("estimator_kin");

    plug(robot.encoders.sout,                             filters.estimator_kin.x);

    plug(robot.imu_offset_compensation.accelerometer_out, filters.acc_filter.x);
    plug(robot.imu_offset_compensation.gyrometer_out,     filters.gyro_filter.x);
    plug(robot.device.forceRLEG,                          filters.ft_RF_filter.x);
    plug(robot.device.forceLLEG,                          filters.ft_LF_filter.x);
    plug(robot.device.forceRARM,                          filters.ft_RH_filter.x);
    plug(robot.device.forceLARM,                          filters.ft_LH_filter.x);
    plug(robot.device.currents,                           filters.current_filter.x);

    #filters.current_filter.init(dt,NJ, conf.DELAY_CURRENT*dt,1)
    filters.ft_RF_filter.init(dt, 6, conf.DELAY_FORCE*dt,1)
    filters.ft_LF_filter.init(dt, 6, conf.DELAY_FORCE*dt,1)
    filters.ft_RH_filter.init(dt, 6, conf.DELAY_FORCE*dt,1)
    filters.ft_LH_filter.init(dt, 6, conf.DELAY_FORCE*dt,1)
    filters.gyro_filter.init(dt, 3, conf.DELAY_GYRO*dt,1)
    filters.acc_filter.init(dt, 3, conf.DELAY_ACC*dt,1)

    filters.estimator_kin.init(dt,NJ, conf.DELAY_ENC*dt,2);

    return filters;

def create_torque_controller(robot, conf, motor_params, dt=0.001, robot_name="robot"):
    torque_ctrl = JointTorqueController("jtc");
    plug(robot.encoders.sout,                           torque_ctrl.jointsPositions);
    plug(robot.filters.estimator_kin.dx,                torque_ctrl.jointsVelocities);
    plug(robot.filters.estimator_kin.ddx,               torque_ctrl.jointsAccelerations);
    #plug(robot.estimator_ft.jointsTorques,              torque_ctrl.jointsTorques);
    plug(robot.device.ptorque,                          torque_ctrl.jointsTorques); #New
    torque_ctrl.jointsTorquesDesired.value              = NJ*(0.0,);
    torque_ctrl.jointsTorquesDerivative.value           = NJ*(0.0,);
    torque_ctrl.dq_des.value                            = NJ*(0.0,);
    torque_ctrl.KpTorque.value                          = tuple(conf.k_p_torque);
    torque_ctrl.KdTorque.value                          = tuple(conf.k_d_torque);
    torque_ctrl.KiTorque.value                          = tuple(conf.k_i_torque);
    torque_ctrl.KdVel.value                             = tuple(conf.k_d_vel);
    torque_ctrl.KiVel.value                             = tuple(conf.k_i_vel);
    torque_ctrl.torque_integral_saturation.value        = tuple(conf.torque_integral_saturation);
    torque_ctrl.coulomb_friction_compensation_percentage.value = NJ*(conf.COULOMB_FRICTION_COMPENSATION_PERCENTAGE,);

    torque_ctrl.motorParameterKt_p.value  = tuple(motor_params.Kt_p)
    torque_ctrl.motorParameterKt_n.value  = tuple(motor_params.Kt_n)
    torque_ctrl.motorParameterKf_p.value  = tuple(motor_params.Kf_p)
    torque_ctrl.motorParameterKf_n.value  = tuple(motor_params.Kf_n)
    torque_ctrl.motorParameterKv_p.value  = tuple(motor_params.Kv_p)
    torque_ctrl.motorParameterKv_n.value  = tuple(motor_params.Kv_n)
    torque_ctrl.motorParameterKa_p.value  = tuple(motor_params.Ka_p)
    torque_ctrl.motorParameterKa_n.value  = tuple(motor_params.Ka_n)
    torque_ctrl.polySignDq.value          = NJ*(conf.poly_sign_dq,);
    torque_ctrl.init(dt, robot_name);
    return torque_ctrl;

def create_balance_controller(robot, conf, motor_params, dt, robot_name='robot', controlType="torque", simu=True, patternGenerator=False):
    from dynamic_graph.sot.torque_control.inverse_dynamics_balance_controller import InverseDynamicsBalanceController
    ctrl = InverseDynamicsBalanceController("invDynBalCtrl")
    ctrl.setControlOutputType(controlType)
    try:
        plug(robot.base_estimator.q, ctrl.q)
        plug(robot.base_estimator.v, ctrl.v)
    except:
        q = Mix_of_vector('selecJointConf')
        q.setSignalNumber(2);
        plug(robot.device.robotState, q.default)
        q.sin1.value = robot.halfSitting
        q.addSelec(1, 0, 6)
        plug(q.sout, ctrl.q)
        plug(robot.device.robotVelocity, ctrl.v)

    try:
        from dynamic_graph.sot.core import Selec_of_vector
        robot.ddq_des = Selec_of_vector('ddq_des')
        plug(ctrl.dv_des, robot.ddq_des.sin)
        robot.ddq_des.selec(6,NJ+6)
    except:
        print("WARNING: Could not connect dv_des from BalanceController to ForceTorqueEstimator")

    plug(robot.device.ptorque, ctrl.tau_measured)
    plug(robot.dynamic.com, ctrl.com_measured)
    plug(robot.device.forceRLEG, ctrl.wrench_right_foot) # New
    plug(robot.device.forceLLEG, ctrl.wrench_left_foot) # New

    if (patternGenerator):
        rfootSE3 = MatrixHomoToSE3Vector("rfootSE3")
        plug(robot.pg.rightfootref, rfootSE3.sin)
        plug(rfootSE3.sout, ctrl.rf_ref_pos)
        rfootSE3_dot = MatrixHomoToSE3Vector("rfootSE3_dot")
        plug(robot.pg.dotrightfootref, rfootSE3_dot.sin)
        plug(rfootSE3_dot.sout, ctrl.rf_ref_vel)
        ctrl.rf_ref_acc.value = np.zeros(12)

        lfootSE3 = MatrixHomoToSE3Vector("lfootSE3")
        plug(robot.pg.leftfootref, lfootSE3.sin)
        plug(lfootSE3.sout, ctrl.lf_ref_pos)
        lfootSE3_dot = MatrixHomoToSE3Vector("lfootSE3_dot")
        plug(robot.pg.dotleftfootref, lfootSE3_dot.sin)
        plug(lfootSE3_dot.sout, ctrl.lf_ref_vel)
        ctrl.lf_ref_acc.value = np.zeros(12)

        plug(ctrl.right_hand_pos,   robot.rh_traj_gen.initial_value);
        plug(robot.rh_traj_gen.x,   ctrl.rh_ref_pos)
        plug(robot.rh_traj_gen.dx,  ctrl.rh_ref_vel)
        plug(robot.rh_traj_gen.ddx, ctrl.rh_ref_acc)

        plug(ctrl.left_hand_pos,    robot.lh_traj_gen.initial_value);
        plug(robot.lh_traj_gen.x,   ctrl.lh_ref_pos)
        plug(robot.lh_traj_gen.dx,  ctrl.lh_ref_vel)
        plug(robot.lh_traj_gen.ddx, ctrl.lh_ref_acc)

        plug(robot.traj_gen.q,   ctrl.posture_ref_pos)
        plug(robot.traj_gen.dq,  ctrl.posture_ref_vel)
        plug(robot.traj_gen.ddq, ctrl.posture_ref_acc)

        plug(robot.pg.comref, ctrl.com_ref_pos)
        plug(robot.pg.dcomref, ctrl.com_ref_vel)
        plug(robot.pg.ddcomref, ctrl.com_ref_acc)
        
        # plug(robot.pg.amref, ctrl.am_ref_L)
        # plug(robot.pg.damref, ctrl.am_ref_dL)
        ctrl.am_ref_L.value = np.zeros(3)
        ctrl.am_ref_dL.value = np.zeros(3)

        waistSE3 = MatrixHomoToSE3Vector("waistSE3")
        plug(robot.pg.waistattitudematrixabsolute, waistSE3.sin)
        plug(waistSE3.sout, ctrl.base_orientation_ref_pos)
        ctrl.base_orientation_ref_vel.value = np.zeros(12)
        ctrl.base_orientation_ref_acc.value = np.zeros(12)
        try:
            plug(robot.pg.contactphase, ctrl.ref_phase)
        except:
            print("WARNING: Could not connect pg contactphase to ref_phase")

    else:
        # plug(ctrl.right_foot_pos, robot.rf_traj_gen.initial_value)
        plug(robot.rf_traj_gen.x,         ctrl.rf_ref_pos)
        plug(robot.rf_traj_gen.dx,        ctrl.rf_ref_vel)
        plug(robot.rf_traj_gen.ddx,       ctrl.rf_ref_acc)

        # plug(ctrl.right_foot_pos, robot.rf_traj_gen.initial_value)
        plug(robot.lf_traj_gen.x,         ctrl.lf_ref_pos)
        plug(robot.lf_traj_gen.dx,        ctrl.lf_ref_vel)
        plug(robot.lf_traj_gen.ddx,       ctrl.lf_ref_acc)

        plug(ctrl.right_hand_pos,         robot.rh_traj_gen.initial_value)
        plug(robot.rh_traj_gen.x,         ctrl.rh_ref_pos)
        plug(robot.rh_traj_gen.dx,        ctrl.rh_ref_vel)
        plug(robot.rh_traj_gen.ddx,       ctrl.rh_ref_acc)

        plug(ctrl.left_hand_pos,          robot.lh_traj_gen.initial_value)
        plug(robot.lh_traj_gen.x,         ctrl.lh_ref_pos)
        plug(robot.lh_traj_gen.dx,        ctrl.lh_ref_vel)
        plug(robot.lh_traj_gen.ddx,       ctrl.lh_ref_acc)

        plug(robot.traj_gen.q,                        ctrl.posture_ref_pos)
        plug(robot.traj_gen.dq,                       ctrl.posture_ref_vel)
        plug(robot.traj_gen.ddq,                      ctrl.posture_ref_acc)
        plug(robot.com_traj_gen.x,                    ctrl.com_ref_pos)
        plug(robot.com_traj_gen.dx,                   ctrl.com_ref_vel)
        plug(robot.com_traj_gen.ddx,                  ctrl.com_ref_acc)
        plug(robot.am_traj_gen.x,                     ctrl.am_ref_L)
        plug(robot.am_traj_gen.dx,                    ctrl.am_ref_dL)
        plug(robot.waist_traj_gen.x,                  ctrl.base_orientation_ref_pos)
        plug(robot.waist_traj_gen.dx,                 ctrl.base_orientation_ref_vel)
        plug(robot.waist_traj_gen.ddx,                ctrl.base_orientation_ref_acc)
        try:
            plug(robot.rf_force_traj_gen.x,               ctrl.f_ref_right_foot)
            plug(robot.lf_force_traj_gen.x,               ctrl.f_ref_left_foot)
        except:
            print("WARNING: Could not connect rf/lf_force_traj_gen to f_ref_right/left_foot")

        try:
            plug(robot.phaseInt.sout, ctrl.ref_phase)
        except:
            print("WARNING: Could not connect phases_traj_gen to ref_phase")

    # rather than giving to the controller the values of gear ratios and rotor inertias
    # it is better to compute directly their product in python and pass the result
    # to the C++ entity, because otherwise we get a loss of precision
    if not simu:
        ctrl.rotor_inertias.value = np.array(([g*g*r for (g,r) in
                                       zip(motor_params.GEAR_RATIOS, motor_params.ROTOR_INERTIAS)]))
        ctrl.gear_ratios.value = np.ones(NJ)
    
    if (controlType=="velocity"):
        rfSE3 = MatrixHomoToSE3Vector("rfSE3")
        plug(robot.wp.footRightDes, rfSE3.sin)
        plug(rfSE3.sout, ctrl.rf_ref_pos)
        lfSE3 = MatrixHomoToSE3Vector("lfSE3")
        plug(robot.wp.footLeftDes, lfSE3.sin)
        plug(lfSE3.sout, ctrl.lf_ref_pos)
        plug(robot.wp.comDes, ctrl.com_ref_pos)
        plug(robot.com_admittance_control.comRef, ctrl.com_adm_ref_pos)
        plug(robot.com_admittance_control.dcomRef, ctrl.com_adm_ref_vel)
        plug(robot.com_admittance_control.ddcomRef, ctrl.com_adm_ref_acc)
        ctrl.kp_com.value = np.array(3*(conf.kp_com_vel,))
        ctrl.kd_com.value = np.array(3*(conf.kd_com_vel,))
        ctrl.w_com.value = conf.w_com_vel
    else:
        ctrl.kp_com.value = np.array(3*(conf.kp_com,))
        ctrl.kd_com.value = np.array(3*(conf.kd_com,))
        ctrl.w_com.value = conf.w_com

    ctrl.contact_normal.value = conf.FOOT_CONTACT_NORMAL
    ctrl.contact_points.value = conf.RIGHT_FOOT_CONTACT_POINTS
    ctrl.f_min.value = conf.fMin
    ctrl.f_max_right_foot.value = conf.fMax
    ctrl.f_max_left_foot.value =  conf.fMax
    ctrl.mu.value = conf.mu[0]
    ctrl.weight_contact_forces.value = np.array((1e2, 1e2, 1e0, 1e3, 1e3, 1e3))
    ctrl.kp_am.value = np.array(3*(conf.kp_am,))
    ctrl.kd_am.value = np.array(3*(conf.kd_am,))
    ctrl.kp_constraints.value = np.array(6*(conf.kp_contact,))
    ctrl.kd_constraints.value = np.array(6*(conf.kd_contact,))
    ctrl.kp_feet.value = np.array(6*(conf.kp_feet,))
    ctrl.kd_feet.value = np.array(6*(conf.kd_feet,))
    ctrl.kp_hands.value = np.array(6*(conf.kp_hands,))
    ctrl.kd_hands.value = np.array(6*(conf.kd_hands,))
    ctrl.kp_posture.value = conf.kp_posture
    ctrl.kd_posture.value = conf.kd_posture
    ctrl.kp_pos.value = conf.kp_pos
    ctrl.kd_pos.value = conf.kd_pos
    ctrl.kp_tau.value = conf.kp_tau
    ctrl.kff_tau.value = conf.kff_tau
    ctrl.kff_dq.value = conf.kff_dq
    ctrl.kp_base_orientation.value = np.array(6*(conf.kp_waist,))
    ctrl.kd_base_orientation.value = np.array(6*(conf.kd_waist,))

    ctrl.w_am.value = conf.w_am
    ctrl.w_feet.value = conf.w_feet
    ctrl.w_hands.value = conf.w_hands
    ctrl.w_forces.value = conf.w_forces
    ctrl.w_posture.value = conf.w_posture
    ctrl.w_base_orientation.value = conf.w_waist
    ctrl.w_torques.value = conf.w_torques

    ctrl.init(dt, robot_name)

    return ctrl;

def create_posture_task(robot, conf, dt, robot_name='robot'):
    from dynamic_graph.sot.torque_control.posture_task import PostureTask
    ctrl = PostureTask("PostureTask")
    try:
        plug(robot.base_estimator.q, ctrl.q)
        plug(robot.base_estimator.v, ctrl.v)
    except:
        q = Mix_of_vector('selecJointConf')
        q.setSignalNumber(2);
        plug(robot.device.robotState, q.default)
        q.sin1.value = robot.halfSitting
        q.addSelec(1, 0, 6)
        plug(q.sout, ctrl.q)
        plug(robot.device.robotVelocity, ctrl.v)
        
    plug(robot.dynamic.com, ctrl.com_measured)

    plug(robot.traj_gen.q,                        ctrl.posture_ref_pos)
    plug(robot.traj_gen.dq,                       ctrl.posture_ref_vel)
    plug(robot.traj_gen.ddq,                      ctrl.posture_ref_acc)    
    # plug(robot.waist_traj_gen.x,                  ctrl.base_orientation_ref_pos)
    # plug(robot.waist_traj_gen.dx,                 ctrl.base_orientation_ref_vel)
    # plug(robot.waist_traj_gen.ddx,                ctrl.base_orientation_ref_acc)
    plug(robot.com_traj_gen.x,                  ctrl.com_ref_pos)
    plug(robot.com_traj_gen.dx,                 ctrl.com_ref_vel)
    plug(robot.com_traj_gen.ddx,                ctrl.com_ref_acc)

    # ctrl.kp_base_orientation.value = np.array(6*(conf.kp_waist,))
    # ctrl.kd_base_orientation.value = np.array(6*(conf.kd_waist,))
    # ctrl.w_base_orientation.value = conf.w_waist
    ctrl.kp_com.value = np.array(3*(conf.kp_com,))
    ctrl.kd_com.value = np.array(3*(conf.kd_com,))
    ctrl.w_com.value = conf.w_com
    ctrl.kp_posture.value = conf.kp_posture
    ctrl.kd_posture.value = conf.kd_posture
    ctrl.w_posture.value = conf.w_posture
    ctrl.kp_constraints.value = np.array(6*(conf.kp_contact,))
    ctrl.kd_constraints.value = np.array(6*(conf.kd_contact,))
    ctrl.w_forces.value = conf.w_forces    
    ctrl.contact_normal.value = conf.FOOT_CONTACT_NORMAL
    ctrl.contact_points.value = conf.RIGHT_FOOT_CONTACT_POINTS
    ctrl.f_min.value = conf.fMin
    ctrl.f_max_right_foot.value = conf.fMax
    ctrl.f_max_left_foot.value =  conf.fMax
    ctrl.mu.value = conf.mu[0]
    ctrl.init(dt, robot_name)
    return ctrl

def create_simple_inverse_dyn_controller(robot, conf, dt, robot_name='robot'):
    from dynamic_graph.sot.torque_control.simple_inverse_dyn import SimpleInverseDyn
    ctrl = SimpleInverseDyn("invDynCtrl")
    try:
        plug(robot.base_estimator.q, ctrl.q)
        plug(robot.base_estimator.v, ctrl.v)
    except:
        q = Mix_of_vector('selecJointConf')
        q.setSignalNumber(2);
        plug(robot.device.robotState, q.default)
        q.sin1.value = robot.halfSitting
        q.addSelec(1, 0, 6)
        plug(q.sout, ctrl.q)
        plug(robot.device.robotVelocity, ctrl.v)
        
    plug(robot.device.ptorque, ctrl.tau_measured)
    plug(robot.dynamic.com, ctrl.com_measured)

    plug(robot.traj_gen.q,                        ctrl.posture_ref_pos)
    plug(robot.traj_gen.dq,                       ctrl.posture_ref_vel)
    plug(robot.traj_gen.ddq,                      ctrl.posture_ref_acc)
    plug(robot.com_traj_gen.x,                    ctrl.com_ref_pos)
    plug(robot.com_traj_gen.dx,                   ctrl.com_ref_vel)
    plug(robot.com_traj_gen.ddx,                  ctrl.com_ref_acc)
    plug(robot.waist_traj_gen.x,                  ctrl.waist_ref_pos)
    plug(robot.waist_traj_gen.dx,                 ctrl.waist_ref_vel)
    plug(robot.waist_traj_gen.ddx,                ctrl.waist_ref_acc)


    ctrl.contact_normal.value = conf.FOOT_CONTACT_NORMAL
    ctrl.contact_points.value = conf.RIGHT_FOOT_CONTACT_POINTS
    ctrl.f_min.value = conf.fMin
    ctrl.f_max.value = conf.fMax
    ctrl.mu.value = conf.mu[0]
    ctrl.kp_com.value = np.array(3*(conf.kp_com,))
    ctrl.kd_com.value = np.array(3*(conf.kd_com,))
    ctrl.kp_contact.value = np.array(6*(conf.kp_contact,))
    ctrl.kd_contact.value = np.array(6*(conf.kd_contact,))

    ctrl.kp_posture.value = conf.kp_posture
    ctrl.kd_posture.value = conf.kd_posture
    ctrl.kp_pos.value = conf.kp_pos
    ctrl.kd_pos.value = conf.kd_pos
    ctrl.kp_tau.value = conf.kp_tau
    ctrl.kp_waist.value = np.array(6*(conf.kp_waist,))
    ctrl.kd_waist.value = np.array(6*(conf.kd_waist,))

    ctrl.w_com.value = conf.w_com
    ctrl.w_forces.value = conf.w_forces
    ctrl.w_posture.value = conf.w_posture
    ctrl.w_waist.value = conf.w_waist

    ctrl.init(dt, robot_name)
    return ctrl

def create_inverse_dynamics(robot, conf, motor_params, dt=0.001):
    inv_dyn_ctrl = InverseDynamicsController("inv_dyn");
    plug(robot.device.robotState,             inv_dyn_ctrl.base6d_encoders);
    plug(robot.filters.estimator_kin.dx,              inv_dyn_ctrl.jointsVelocities);
    plug(robot.traj_gen.q,                    inv_dyn_ctrl.qRef);
    plug(robot.traj_gen.dq,                   inv_dyn_ctrl.dqRef);
    plug(robot.traj_gen.ddq,                  inv_dyn_ctrl.ddqRef);
    plug(robot.estimator_ft.contactWrenchRightSole,   inv_dyn_ctrl.fRightFoot);
    plug(robot.estimator_ft.contactWrenchLeftSole,    inv_dyn_ctrl.fLeftFoot);
    plug(robot.estimator_ft.contactWrenchRightHand,   inv_dyn_ctrl.fRightHand);
    plug(robot.estimator_ft.contactWrenchLeftHand,    inv_dyn_ctrl.fLeftHand);
    plug(robot.traj_gen.fRightFoot,           inv_dyn_ctrl.fRightFootRef);
    plug(robot.traj_gen.fLeftFoot,            inv_dyn_ctrl.fLeftFootRef);
    plug(robot.traj_gen.fRightHand,           inv_dyn_ctrl.fRightHandRef);
    plug(robot.traj_gen.fLeftHand,            inv_dyn_ctrl.fLeftHandRef);
    plug(robot.estimator_ft.baseAngularVelocity, inv_dyn_ctrl.baseAngularVelocity);
    plug(robot.estimator_ft.baseAcceleration,    inv_dyn_ctrl.baseAcceleration);
    plug(inv_dyn_ctrl.tauDes,           robot.torque_ctrl.jointsTorquesDesired);
    plug(inv_dyn_ctrl.tauDes,           robot.estimator_ft.tauDes);
    plug(robot.estimator_ft.dynamicsError,       inv_dyn_ctrl.dynamicsError);

    inv_dyn_ctrl.dynamicsErrorGain.value = (NJ+6)*(0.0,);
    inv_dyn_ctrl.Kp.value = tuple(conf.k_s); # joint proportional conf
    inv_dyn_ctrl.Kd.value = tuple(conf.k_d); # joint derivative conf
    inv_dyn_ctrl.Kf.value = tuple(conf.k_f); # force proportional conf
    inv_dyn_ctrl.Ki.value = tuple(conf.k_i); # force integral conf
    inv_dyn_ctrl.rotor_inertias.value = motor_params.ROTOR_INERTIAS;
    inv_dyn_ctrl.gear_ratios.value    = motor_params.GEAR_RATIOS;
    inv_dyn_ctrl.controlledJoints.value = NJ*(1.0,);
    inv_dyn_ctrl.init(dt);
    return inv_dyn_ctrl;

def create_ddp_controller(robot, conf, dt):
    from dynamic_graph.sot.torque_control.ddp_actuator_solver import DdpActuatorSolver
    ddp_controller = DdpActuatorSolver("ddp_ctrl");
    plug(robot.joint_pos_selec_ddp.sout,        ddp_controller.pos_joint_measure);
    plug(robot.joint_vel_selec_ddp.sout,        ddp_controller.dx_measure);
    plug(robot.pos_des_selec_ddp.sout,          ddp_controller.pos_des);
    plug(robot.joint_torque_selec_ddp.sout,     ddp_controller.tau_measure);
    plug(robot.motor_pos_selec_ddp.sout,        ddp_controller.pos_motor_measure);
    plug(robot.tau_des_selec_ddp.sout,          ddp_controller.tau_des);

    ddp_controller.temp_measure.value = conf.temp_const;

    #plug(ddp_controller.tau,            robot.torque_ctrl.jointsTorquesDesired);

    ddp_controller.init(dt, conf.T, conf.nb_iter, conf.stop_criteria)
    return ddp_controller;

def create_pyrene_ddp_controller(robot, conf, dt):
    from dynamic_graph.sot.torque_control.ddp_pyrene_actuator_solver import DdpPyreneActuatorSolver
    ddp_controller = DdpPyreneActuatorSolver("ddp_ctrl");
    plug(robot.joint_pos_selec_ddp.sout,        ddp_controller.pos_joint_measure)
    plug(robot.joint_vel_selec_ddp.sout,        ddp_controller.dx_joint_measure)
    plug(robot.pos_des_selec_ddp.sout,          ddp_controller.pos_des)
    ddp_controller.tau_des.value = NJ*(0.0,)
    # plug(robot.torque_ctrl.u,                   ddp_controller.tau_des)
    ddp_controller.init(dt, conf.T, conf.nb_iter, conf.stop_criteria)
    return ddp_controller

def create_parameter_server(conf, dt, robot_name='robot'):
    param_server = ParameterServer("param_server")
    fill_parameter_server(param_server, conf, dt, robot_name)

def fill_parameter_server(param_server, conf, dt, robot_name='robot'):
    # Init should be called before addCtrlMode
    # because the size of state vector must be known.
    param_server.init(dt, conf.urdfFileName, robot_name)

    # Set the map from joint name to joint ID
    for key in conf.mapJointNameToID:
        param_server.setNameToId(key, conf.mapJointNameToID[key])

    # Set the map joint limits for each id
    for key in conf.mapJointLimits:
        param_server.setJointLimitsFromId(key, conf.mapJointLimits[key][0], conf.mapJointLimits[key][1])

    # Set the force limits for each id
    for key in conf.mapForceIdToForceLimits:
        param_server.setForceLimitsFromId(key, np.array(conf.mapForceIdToForceLimits[key][0]),
                                          np.array(conf.mapForceIdToForceLimits[key][1]))

    # Set the force sensor id for each sensor name
    for key in conf.mapNameToForceId:
        param_server.setForceNameToForceId(key, conf.mapNameToForceId[key])

    # Set the map from the urdf joint list to the sot joint list
    param_server.setJointsUrdfToSot(np.array(conf.urdftosot))

    # Set the foot frame name
    for key in conf.footFrameNames:
        param_server.setFootFrameName(key, conf.footFrameNames[key])

    # Set IMU hosting joint name
    param_server.setImuJointName(conf.ImuJointName)

    param_server.setRightFootForceSensorXYZ(np.array(conf.rightFootSensorXYZ))
    param_server.setRightFootSoleXYZ(np.array(conf.rightFootSoleXYZ))

    return param_server


def create_ctrl_manager(conf, motor_params, dt, robot_name='robot'):
    ctrl_manager = ControlManager("ctrl_man");

    ctrl_manager.signal('tau_predicted').value = np.zeros(NJ);
    ctrl_manager.signal('i_measured').value = np.zeros(NJ);
    ctrl_manager.signal('tau_max').value = np.ones(NJ)*(conf.TAU_MAX,);
    ctrl_manager.signal('i_max').value = np.ones(NJ)*(conf.CURRENT_MAX,);
    ctrl_manager.signal('u_max').value = np.ones(NJ)*(conf.CTRL_MAX,);
    # Init should be called before addCtrlMode
    # because the size of state vector must be known.
    ctrl_manager.init(dt, conf.urdfFileName, robot_name)
    return ctrl_manager

def connect_ctrl_manager(robot):
    # connect to device
    plug(robot.device.currents,   robot.ctrl_manager.i_measured)
    plug(robot.device.ptorque,    robot.ctrl_manager.tau)
    robot.ctrl_manager.addCtrlMode("torque")
    plug(robot.inv_dyn.u, robot.ctrl_manager.ctrl_torque)

    robot.ctrl_manager.setCtrlMode("all", "torque")
    plug(robot.ctrl_manager.joints_ctrl_mode_torque, robot.inv_dyn.active_joints)
    plug(robot.ctrl_manager.u_safe, robot.device.control)
    return

def create_current_controller(robot, conf, motor_params, dt, robot_name='robot'):
    current_ctrl = CurrentController("current_ctrl");

    current_ctrl.i_max.value                                = NJ*(conf.CURRENT_MAX,);
    current_ctrl.u_max.value                                = NJ*(conf.CTRL_MAX,);
    current_ctrl.u_saturation.value                         = NJ*(conf.CTRL_SATURATION,);
    current_ctrl.percentage_dead_zone_compensation.value    = tuple(conf.percentage_dead_zone_compensation);
    current_ctrl.percentage_bemf_compensation.value         = tuple(conf.percentage_bemf_compensation);
    current_ctrl.i_sensor_offsets_low_level.value           = tuple(conf.current_sensor_offsets_low_level);
    current_ctrl.i_max_dead_zone_compensation.value         = tuple(conf.i_max_dz_comp);
    current_ctrl.in_out_gain.value                          = NJ*(conf.IN_OUT_GAIN,);
    current_ctrl.kp_current.value                           = tuple(conf.kp_current);
    current_ctrl.ki_current.value                           = tuple(conf.ki_current);
    current_ctrl.bemf_factor.value                          = motor_params.K_bemf;
    current_ctrl.dead_zone_offsets.value                    = motor_params.deadzone;
    current_ctrl.i_sens_gains.value                         = motor_params.cur_sens_gains;
    # connect to other entities
    plug(robot.filters.current_filter.x_filtered,   current_ctrl.i_measured)
    plug(robot.filters.estimator_kin.dx,            current_ctrl.dq);
    plug(current_ctrl.u_safe,                       robot.device.control);
    # initialize
    current_ctrl.init(dt, robot_name, conf.CURRENT_OFFSET_ITERS)

    return current_ctrl;


def create_admittance_ctrl(robot, conf, dt=0.001, robot_name='robot'):
    admit_ctrl = AdmittanceController("adm_ctrl");
    plug(robot.encoders.sout,                         admit_ctrl.encoders);
    plug(robot.filters.estimator_kin.dx,              admit_ctrl.jointsVelocities);
    plug(robot.device.forceRLEG,                      admit_ctrl.fRightFoot);
    plug(robot.device.forceLLEG,                      admit_ctrl.fLeftFoot);
    plug(robot.filters.ft_RF_filter.x_filtered,       admit_ctrl.fRightFootFiltered);
    plug(robot.filters.ft_LF_filter.x_filtered,       admit_ctrl.fLeftFootFiltered);
    plug(robot.inv_dyn.f_des_right_foot,              admit_ctrl.fRightFootRef);
    plug(robot.inv_dyn.f_des_left_foot,               admit_ctrl.fLeftFootRef);

    admit_ctrl.damping.value          = 4*(0.05,);
    admit_ctrl.controlledJoints.value = NJ*(1.0,);
    admit_ctrl.kp_force.value         = conf.kp_force;
    admit_ctrl.ki_force.value         = conf.ki_force;
    admit_ctrl.kp_vel.value           = conf.kp_vel;
    admit_ctrl.ki_vel.value           = conf.ki_vel;
    admit_ctrl.force_integral_saturation.value = conf.force_integral_saturation;
    admit_ctrl.force_integral_deadzone.value   = conf.force_integral_deadzone;

    # connect it to torque control
    from dynamic_graph.sot.core import Add_of_vector
    robot.sum_torque_adm = Add_of_vector('sum_torque_adm');
    plug(robot.inv_dyn.tau_des,     robot.sum_torque_adm.sin1);
    plug(admit_ctrl.u,              robot.sum_torque_adm.sin2);
    plug(robot.sum_torque_adm.sout, robot.torque_ctrl.jointsTorquesDesired);

    admit_ctrl.init(dt, robot_name);
    return admit_ctrl;

def create_rospublish(robot, name):
    from dynamic_graph.ros import RosPublish
    rospub = RosPublish(name)
    robot.device.after.addSignal(rospub.name+'.trigger')
    return rospub

def create_topic(rospub, entity, signalName, renameSignal, robot=None, data_type='vector'):
    # check needed to prevent creation of broken topic
    if not entity.hasSignal(signalName):
        raise AttributeError('Entity %s does not have signal %s' %
                             (entity.name, signalName))
    rospub_signalName = '{0}_{1}'.format(entity.name, renameSignal)
    topicname = '/simu/{0}'.format(renameSignal)
    rospub.add(data_type, rospub_signalName, topicname)
    plug(entity.signal(signalName), rospub.signal(rospub_signalName))
    if robot is not None:
        robot.device.after.addSignal('{0}.{1}'.format(entity.name, signalName))


def create_ros_topics(robot):
    from dynamic_graph.ros import RosPublish
    ros = RosPublish('rosPublish');
    try:
        create_topic(ros, robot.device.robotState,      'robotState');
        create_topic(ros, robot.device.gyrometer,       'gyrometer');
        create_topic(ros, robot.device.accelerometer,   'accelerometer');
        create_topic(ros, robot.device.forceRLEG,       'forceRLEG');
        create_topic(ros, robot.device.forceLLEG,       'forceLLEG');
        create_topic(ros, robot.device.currents,        'currents');
#        create_topic(ros, robot.device.forceRARM,       'forceRARM');
#        create_topic(ros, robot.device.forceLARM,       'forceLARM');
        robot.device.after.addDownsampledSignal('rosPublish.trigger',1);
    except:
        pass;

    try:
        create_topic(ros, robot.filters.estimator_kin.dx,                            'jointsVelocities');
        create_topic(ros, robot.estimator_ft.contactWrenchLeftSole,          'contactWrenchLeftSole');
        create_topic(ros, robot.estimator_ft.contactWrenchRightSole,         'contactWrenchRightSole');
        create_topic(ros, robot.estimator_ft.jointsTorques,                  'jointsTorques');
#        create_topic(ros, robot.estimator.jointsTorquesFromInertiaModel,  'jointsTorquesFromInertiaModel');
#        create_topic(ros, robot.estimator.jointsTorquesFromMotorModel,    'jointsTorquesFromMotorModel');
#        create_topic(ros, robot.estimator.currentFiltered,                'currentFiltered');
    except:
        pass;

    try:
        create_topic(ros, robot.torque_ctrl.u, 'i_des_torque_ctrl');
    except:
        pass;

    try:
        create_topic(ros, robot.traj_gen.q,   'q_ref');
#        create_topic(ros, robot.traj_gen.dq,  'dq_ref');
#        create_topic(ros, robot.traj_gen.ddq, 'ddq_ref');
    except:
        pass;

    try:
        create_topic(ros, robot.ctrl_manager.pwmDes,                  'i_des');
        create_topic(ros, robot.ctrl_manager.pwmDesSafe,              'i_des_safe');
#        create_topic(ros, robot.ctrl_manager.signOfControlFiltered,   'signOfControlFiltered');
#        create_topic(ros, robot.ctrl_manager.signOfControl,           'signOfControl');
    except:
        pass;

    try:
        create_topic(ros, robot.inv_dyn.tau_des, 'tau_des');
    except:
        pass;

    try:
        create_topic(ros, robot.ff_locator.base6dFromFoot_encoders,        'base6dFromFoot_encoders');
    except:
        pass;

    try:
        create_topic(ros, robot.floatingBase.soutPos, 'floatingBase_pos');
    except:
        pass;

    return ros;


def addTrace(tracer, entity, signalName):
    """
    Add a signal to a tracer
    """
    signal = '{0}.{1}'.format(entity.name, signalName);
    filename = '{0}-{1}'.format(entity.name, signalName);
    tracer.add(signal, filename);

def addSignalsToTracer(tracer, device):
    addTrace(tracer,device,'robotState');
    addTrace(tracer,device,'gyrometer');
    addTrace(tracer,device,'accelerometer');
    addTrace(tracer,device,'forceRLEG');
    addTrace(tracer,device,'forceLLEG');
    addTrace(tracer,device,'forceRARM');
    addTrace(tracer,device,'forceLARM');
    addTrace(tracer,device,'control');
    addTrace(tracer,device,'currents');
    addTrace(tracer,device,'ptorque');


def create_tracer(device, traj_gen=None, estimator_kin=None,
                  inv_dyn=None, torque_ctrl=None):
    tracer = TracerRealTime('motor_id_trace');
    tracer.setBufferSize(80*(2**20));
    tracer.open('/tmp/','dg_','.dat');
    device.after.addSignal('{0}.triger'.format(tracer.name));

    addSignalsToTracer(tracer, device);

    with open('/tmp/dg_info.dat', 'a') as f:
        if(estimator_kin!=None):
            f.write('Estimator encoder delay: {0}\n'.format(robot.filters.estimator_kin.getDelay()));
        if(inv_dyn!=None):
            f.write('Inv dyn Ks: {0}\n'.format(inv_dyn.Kp.value));
            f.write('Inv dyn Kd: {0}\n'.format(inv_dyn.Kd.value));
            f.write('Inv dyn Kf: {0}\n'.format(inv_dyn.Kf.value));
            f.write('Inv dyn Ki: {0}\n'.format(inv_dyn.Ki.value));
        if(torque_ctrl!=None):
            f.write('Torque ctrl KpTorque: {0}\n'.format (robot.torque_ctrl.KpTorque.value ));
    f.close();
    return tracer;

def dump_tracer(tracer):
    from time import sleep
    tracer.stop()
    sleep(0.2)
    tracer.dump()
    sleep(0.2)
    tracer.close()

def reset_tracer(device,tracer):
    from time import sleep
    tracer.stop();
    sleep(0.2);
    tracer.dump();
    sleep(0.2);
    tracer.close();
    sleep(0.2);
    tracer.clear();
    sleep(0.2);
    tracer = create_tracer(device);
    return tracer;
