from dynamic_graph import plug
import numpy as np
import pinocchio as pin
from dynamic_graph.sot.torque_control.se3_trajectory_generator import SE3TrajectoryGenerator
from dynamic_graph.sot_talos_balance.nd_trajectory_generator import NdTrajectoryGenerator
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_trajectory_switch, connect_synchronous_trajectories, create_force_traj_gen
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import NJ, create_rospublish, create_topic, get_sim_conf, get_default_conf 
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_waist_traj_gen, create_trajectory_generator, create_com_traj_gen
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_filters, create_encoders, create_am_traj_gen
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_balance_controller, create_simple_inverse_dyn_controller
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import addTrace, dump_tracer, create_encoders_velocity
from dynamic_graph.sot_talos_balance.create_entities_utils import create_device_filters, create_imu_filters, create_base_estimator, create_ft_wrist_calibrator
from dynamic_graph.sot.torque_control.talos.sot_utils_talos import go_to_position
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.sot.core.operator import Component_of_vector, Stack_of_vector, Selec_of_vector
from dynamic_graph.sot_talos_balance.round_double_to_int import RoundDoubleToInt
from dynamic_graph.sot_talos_balance.delay import DelayVector
from dynamic_graph.sot_talos_balance.create_entities_utils import fill_parameter_server, create_ctrl_manager
import dynamic_graph.sot_talos_balance.talos.parameter_server_conf as param_server_conf
import dynamic_graph.sot_talos_balance.talos.control_manager_conf as cm_conf
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Vector, MatrixToRPY
import dynamic_graph.sot_talos_balance.talos.ft_wrist_calibration_conf as forceConf
from dynamic_graph.sot_talos_balance.euler_to_quat import EulerToQuat

# --- EXPERIMENTAL SET UP ------------------------------------------------------
#conf = get_sim_conf()
conf = get_default_conf()
dt = robot.timeStep
robot.device.setControlInputType('noInteg') # No integration for torque control
device = 'simu'
endEffector = 'rightWrist'
endEffectorWeight = forceConf.handWeight[device]
rightOC = np.array(forceConf.rightLeverArm)
leftOC = np.array(forceConf.leftLeverArm)

# --- SET INITIAL CONFIGURATION ------------------------------------------------
# TMP: overwrite halfSitting configuration to use SoT joint order
q = [0., 0., 1.018213, 0., 0., 0.] # Free flyer
q += [0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708] # legs
q += [0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708] # legs
q += [0.0, 0.006761] # Chest
# q += [0.35, 0.173046, -0.0002, -0.8, 0.0, -0.0, 0.1, -0.005] # arms
# q += [-0.35, -0.173046, 0.0002, -1., 0.0, 0.0, 0.1, -0.005] # arms
q += [0.25847, 0.173046, -0.0002, -0.525366, 0.0, -0.0, 0.1, -0.005] # arms
q += [-0.25847, -0.173046, 0.0002, -0.525366, 0.0, 0.0, 0.1, -0.005] # arms
q += [0., 0.] # Head

robot.halfSitting = q

# --- CREATE ENTITIES ----------------------------------------------------------

fill_parameter_server(robot.param_server,param_server_conf, dt)
robot.encoders = create_encoders(robot)
robot.encoders_velocity = create_encoders_velocity(robot)

# --- Posture trajectory
robot.traj_gen = create_trajectory_generator(robot, dt)
robot.traj_gen.q.recompute(0)

# --- CoM trajectory
robot.com_traj_gen = create_com_traj_gen(robot, dt)
robot.com_traj_gen.x.recompute(0)

# --- Base orientation (SE3 on the waist) trajectory
robot.waist_traj_gen = create_waist_traj_gen("tg_waist_ref", robot, dt)
robot.waist_traj_gen.x.recompute(0)

# --- Angular momentum trajectory
robot.am_traj_gen = create_am_traj_gen(robot, dt)
robot.am_traj_gen.x.recompute(0)

# --- Contact phases trajectories
robot.phases_traj_gen = NdTrajectoryGenerator("tg_phases")
robot.phases_traj_gen.initial_value.value = np.array([0.0])
robot.phases_traj_gen.init(dt, 1)
# Set phase to int
robot.phaseScalar = Component_of_vector("phase_scalar")
robot.phaseScalar.setIndex(0)
plug(robot.phases_traj_gen.x, robot.phaseScalar.sin)
robot.phaseInt = RoundDoubleToInt("phase_int")
plug(robot.phaseScalar.sout, robot.phaseInt.sin)

# --- Feet trajectories
init_value_rf = np.loadtxt(folder + walk_type + "/rightFoot.dat")[0]
init_value_lf = np.loadtxt(folder + walk_type + "/leftFoot.dat")[0]
robot.rf_traj_gen = NdTrajectoryGenerator("tg_rf")
robot.rf_traj_gen.initial_value.value = init_value_rf
robot.lf_traj_gen = NdTrajectoryGenerator("tg_lf")
robot.lf_traj_gen.initial_value.value = init_value_lf
robot.rf_traj_gen.init(dt, 12)
robot.lf_traj_gen.init(dt, 12)

# --- Hands trajectories
robot.rh_traj_gen = SE3TrajectoryGenerator("tg_rh")
robot.lh_traj_gen = SE3TrajectoryGenerator("tg_lh")
robot.rh_traj_gen.init(dt)
robot.lh_traj_gen.init(dt)

# --- Force hand trajectory
robot.force_hand_traj_gen = NdTrajectoryGenerator("tg_force_hand")
robot.force_hand_traj_gen.initial_value.value = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
robot.force_hand_traj_gen.init(dt, 6)

# --- Switch which synchronizes trajectories
robot.traj_sync = create_trajectory_switch()
trajs = [robot.com_traj_gen, robot.waist_traj_gen, robot.am_traj_gen, robot.phases_traj_gen]
trajs += [robot.rf_traj_gen, robot.lf_traj_gen, robot.rh_traj_gen, robot.lh_traj_gen, robot.force_hand_traj_gen]
connect_synchronous_trajectories(robot.traj_sync, trajs)

# --- Base Estimator
robot.device_filters = create_device_filters(robot, dt)
robot.imu_filters = create_imu_filters(robot, dt)
robot.base_estimator = create_base_estimator(robot, dt, conf.base_estimator)
plug(robot.device_filters.vel_filter.x_filtered, robot.base_estimator.joint_velocities)

robot.base_estimator.q.recompute(0)
robot.base_estimator.v.recompute(0)

# Get configuration vector
robot.e2q = EulerToQuat("e2q")
plug(robot.base_estimator.q, robot.e2q.euler)

robot.forceCalibrator = create_ft_wrist_calibrator(robot, endEffectorWeight, rightOC, leftOC)

# --- Inverse dynamic controller
robot.inv_dyn = create_balance_controller(robot, conf.balance_ctrl,conf.motor_params, dt, controlType="torque")
robot.inv_dyn.active_joints.value = np.ones(32)
robot.inv_dyn.kp_com.value = np.array((50, 50, 50))
robot.inv_dyn.kd_com.value = np.array((5, 5, 5))
robot.inv_dyn.kp_posture.value  = 80 * np.ones(32)
robot.inv_dyn.kd_posture.value = np.array(2 * np.sqrt(robot.inv_dyn.kp_posture.value))
robot.inv_dyn.kp_hands.value  = np.array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0])
robot.inv_dyn.kd_hands.value = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
robot.inv_dyn.w_hands.value = 10

plug(robot.forceCalibrator.leftWristForceOut, robot.inv_dyn.f_ext_left_arm)
# plug(robot.device_filters.ft_LH_filter.dx, robot.inv_dyn.df_ext_left_arm)
plug(robot.force_hand_traj_gen.x, robot.inv_dyn.f_ref_left_arm)
# plug(robot.force_hand_traj_gen.dx, robot.inv_dyn.df_ref_left_arm)
plug(robot.device_filters.torque_filter.x_filtered, robot.inv_dyn.tau_measured)

# --- Reference position of the feet for base estimator
robot.inv_dyn.left_foot_pos_ref_quat.recompute(0)
robot.inv_dyn.right_foot_pos_ref_quat.recompute(0)
plug(robot.inv_dyn.left_foot_pos_ref_quat, robot.base_estimator.lf_ref_xyzquat)
plug(robot.inv_dyn.right_foot_pos_ref_quat, robot.base_estimator.rf_ref_xyzquat)

# --- Connect control manager
robot.ctrl_manager = create_ctrl_manager(cm_conf, dt, robot_name='robot')
effortLimit = robot.dynamic.model.effortLimit[6:]
# effortLimit[0] = 0.6*effortLimit[0]
# effortLimit[6] = 0.6*effortLimit[6]
robot.ctrl_manager.u_max.value = np.concatenate((100*np.ones(6), effortLimit))

robot.ff_torque = Stack_of_vector('ff_torque')
robot.ff_torque.sin1.value = np.zeros(6)
plug(robot.inv_dyn.tau_des, robot.ff_torque.sin2)
robot.ff_torque.selec1(0, 6)
robot.ff_torque.selec2(0, 32)

robot.ctrl_manager.addCtrlMode("torque")
robot.ctrl_manager.setCtrlMode("lhy-lhr-lhp-lk-lap-lar-rhy-rhr-rhp-rk-rap-rar-ty-tp-lsy-lsr-lay-le-lwy-lwp-lwr-rsy-rsr-ray-re-rwy-rwp-rwr", "torque", "torque")
plug(robot.ff_torque.sout, robot.ctrl_manager.signal('ctrl_torque'))

joint_ctrl = np.zeros(38)
joint_ctrl = robot.device.robotState.value
joint_ctrl[27] = -0.01
joint_ctrl[35] = -0.01
robot.ctrl_manager.addCtrlMode("pos")
robot.ctrl_manager.setCtrlMode("lh-rh-hp-hy", "pos", "position")
robot.ctrl_manager.signal('ctrl_pos').value = joint_ctrl

robot.ctrl_manager.addCtrlMode("base")
robot.ctrl_manager.setCtrlMode("freeflyer", "base", "freeflyer")
plug(robot.inv_dyn.q_des, robot.ctrl_manager.signal('ctrl_base'))
plug(robot.ctrl_manager.signal('u_safe'), robot.device.control)

# --- Delay position q
robot.delay_pos = DelayVector("delay_pos")
robot.delay_pos.setMemory(robot.base_estimator.q.value)
robot.device.before.addSignal(robot.delay_pos.name + '.current')
plug(robot.inv_dyn.q_des, robot.delay_pos.sin)

# --- Delay velocity dq
robotDim = len(robot.base_estimator.v.value)
robot.delay_vel = DelayVector("delay_vel")
robot.delay_vel.setMemory(np.zeros(robotDim))
robot.device.before.addSignal(robot.delay_vel.name + '.current')
plug(robot.inv_dyn.v_des, robot.delay_vel.sin)

# --- Fix robot.dynamic inputs
plug(robot.delay_pos.previous, robot.dynamic.position)
plug(robot.delay_vel.previous, robot.dynamic.velocity)
robot.dvdt = Derivator_of_Vector("dv_dt")
robot.dvdt.dt.value = dt
plug(robot.delay_vel.previous, robot.dvdt.sin)
plug(robot.dvdt.sout, robot.dynamic.acceleration)

robot.odom = Selec_of_vector('odom')
plug(robot.base_estimator.q, robot.odom.sin);
robot.odom.selec(0,6);

# --- ROS PUBLISHER ----------------------------------------------------------
robot.publisher = create_rospublish(robot, 'robot_publisher')
create_topic(robot.publisher, robot.odom, 'sout', 'base_odom', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'com', 'inv_dyn_com', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'energy', 'energy', robot=robot, data_type='double')
create_topic(robot.publisher, robot.inv_dyn, 'energy_derivative', 'energy_derivative', robot=robot, data_type='double')
create_topic(robot.publisher, robot.inv_dyn, 'energy_tank', 'energy_tank', robot=robot, data_type='double')
create_topic(robot.publisher, robot.inv_dyn, 'denergy_tank', 'denergy_tank', robot=robot, data_type='double')
create_topic(robot.publisher, robot.inv_dyn, 'energy_bound', 'energy_bound', robot=robot, data_type='double')
create_topic(robot.publisher, robot.inv_dyn, 'task_energy_alpha', 'task_energy_alpha', robot=robot, data_type='double')
create_topic(robot.publisher, robot.inv_dyn, 'task_energy_beta', 'task_energy_beta', robot=robot, data_type='double')
create_topic(robot.publisher, robot.inv_dyn, 'task_energy_gamma', 'task_energy_gamma', robot=robot, data_type='double')
create_topic(robot.publisher, robot.inv_dyn, 'am_L', 'inv_dyn_am', robot=robot, data_type='vector') 
create_topic(robot.publisher, robot.inv_dyn, 'am_dL', 'inv_dyn_dam', robot=robot, data_type='vector') 
create_topic(robot.publisher, robot.inv_dyn, 'q_des', 'q_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'v_des', 'v_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'dv_des', 'dv_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'tau_des', 'tau_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'base_orientation', 'waist_se3', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.base_estimator, 'q', 'base_q', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.base_estimator, 'v', 'base_v', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.com_traj_gen, 'x', 'com_traj_gen', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.waist_traj_gen, 'x', 'waist_traj_gen', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.am_traj_gen, 'x', 'am_traj_gen', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.am_traj_gen, 'dx', 'dam_traj_gen', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'zmp', 'zmp_estim', robot=robot, data_type='vector')  # estimated ZMP
create_topic(robot.publisher, robot.device, 'forceLLEG', 'forceLLEG', robot = robot, data_type='vector') # measured left wrench
create_topic(robot.publisher, robot.device, 'forceRLEG', 'forceRLEG', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.device, 'ptorque', 'tau_meas', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'task_energy_S', 'task_energy_S', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'task_energy_dS', 'task_energy_dS', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'task_energy_A', 'task_energy_A', robot=robot, data_type='double')
create_topic(robot.publisher, robot.device_filters.ft_LH_filter, 'x_filtered', 'ft_LH_filter', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.traj_gen, 'q', 'q_ref', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'f_ref_left_arm', 'f_ref_left_arm', data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'LH_force_world', 'LH_force_world', data_type='vector')
create_topic(robot.publisher, robot.forceCalibrator, 'leftWristForceOut', 'LWForce_calib', robot=robot, data_type='vector')  # calibrated left wrench
create_topic(robot.publisher, robot.forceCalibrator, 'rightWristForceOut', 'RWForce_calib', robot=robot, data_type='vector')  # calibrated right wrench