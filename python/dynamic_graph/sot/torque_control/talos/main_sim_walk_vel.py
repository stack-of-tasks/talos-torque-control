from math import sqrt
import numpy as np
from rospkg import RosPack
from dynamic_graph import plug
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.sot.core.operator import Selec_of_vector, MatrixHomoToPoseQuaternion
from dynamic_graph.sot.core.operator import Substract_of_vector, Component_of_vector, SE3VectorToMatrixHomo
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Vector
from dynamic_graph.sot_talos_balance.nd_trajectory_generator import NdTrajectoryGenerator
from dynamic_graph.sot_talos_balance.round_double_to_int import RoundDoubleToInt
from dynamic_graph.sot.dynamic_pinocchio import DynamicPinocchio
from dynamic_graph.sot.torque_control.se3_trajectory_generator import SE3TrajectoryGenerator
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import NJ, create_rospublish, create_topic, get_sim_conf
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_trajectory_generator, create_com_traj_gen, create_am_traj_gen
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_encoders, create_encoders_velocity, create_waist_traj_gen
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_balance_controller
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import addTrace, dump_tracer
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_trajectory_switch, connect_synchronous_trajectories
from dynamic_graph.sot.torque_control.talos.sot_utils_talos import go_to_position

from dynamic_graph.sot.pattern_generator import PatternGenerator

from dynamic_graph.sot_talos_balance.create_entities_utils import create_device_filters, create_imu_filters, create_base_estimator, create_ft_calibrator
from dynamic_graph.sot_talos_balance.create_entities_utils import fill_parameter_server, create_ctrl_manager
from dynamic_graph.sot_talos_balance.boolean_identity import BooleanIdentity
from dynamic_graph.sot_talos_balance.euler_to_quat import EulerToQuat
from dynamic_graph.sot_talos_balance.dummy_walking_pattern_generator import DummyWalkingPatternGenerator
from dynamic_graph.sot_talos_balance.delay import DelayVector
from dynamic_graph.sot_talos_balance.dcm_estimator import DcmEstimator
from dynamic_graph.sot_talos_balance.dummy_dcm_estimator import DummyDcmEstimator
from dynamic_graph.sot_talos_balance.dcm_controller import DcmController
from dynamic_graph.sot_talos_balance.simple_zmp_estimator import SimpleZmpEstimator
from dynamic_graph.sot_talos_balance.com_admittance_controller import ComAdmittanceController
import dynamic_graph.sot_talos_balance.talos.ft_calibration_conf as ft_conf
import dynamic_graph.sot_talos_balance.talos.parameter_server_conf as param_server_conf
import dynamic_graph.sot_talos_balance.talos.control_manager_conf as cm_conf

# --- EXPERIMENTAL SET UP ------------------------------------------------------
conf = get_sim_conf()
dt = robot.timeStep

# --- Pendulum parameters
robot_name = 'robot'
robot.dynamic.com.recompute(0)
robotDim = robot.dynamic.getDimension()
mass = robot.dynamic.data.mass[0]
h = robot.dynamic.com.value[2]
g = 9.81
omega = sqrt(g / h)

# --- SET INITIAL CONFIGURATION ------------------------------------------------
# TMP: overwrite halfSitting configuration to use SoT joint order
q = [0., 0., 1.018213, 0., 0., 0.] # Free flyer
q += [0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708] # legs
q += [0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708] # legs
q += [0.0, 0.006761] # Chest
q += [0.25847, 0.173046, -0.0002, -0.525366, 0.0, -0.0, 0.1, -0.005] # arms
q += [-0.25847, -0.173046, 0.0002, -0.525366, 0.0, 0.0, 0.1, -0.005] # arms
q += [0., 0.] # Head

robot.halfSitting = q

# --- CREATE ENTITIES ----------------------------------------------------------

cm_conf.CTRL_MAX = 1000.0  # temporary hack
fill_parameter_server(robot.param_server,param_server_conf, dt)
# robot.ctrl_manager = create_ctrl_manager(conf.control_manager, conf.motor_params, dt)
robot.encoders = create_encoders(robot)
robot.encoders_velocity = create_encoders_velocity(robot)

# --- Posture trajectory
robot.traj_gen = create_trajectory_generator(robot, dt)
robot.traj_gen.q.recompute(0)

# --- CoM trajectory
init_value_com = np.loadtxt(folder + walk_type + "/com.dat", usecols=(0,1,2))[0]
robot.com_traj_gen = create_com_traj_gen(robot, dt, init_value_com)
robot.com_traj_gen.x.recompute(0)

# --- Base orientation (SE3 on the waist) trajectory
robot.waist_traj_gen = create_waist_traj_gen("tg_waist_ref", robot, dt)
robot.waist_traj_gen.x.recompute(0)

# --- Angular momentum trajectory
init_value_am = np.loadtxt(folder + walk_type + "/am.dat", usecols=(0,1,2))[0]
robot.am_traj_gen = create_am_traj_gen(robot, dt, init_value_am)
robot.am_traj_gen.x.recompute(0)

# --- Feet force trajectories
# robot.rf_force_traj_gen  = create_force_traj_gen("rf_force_ref", conf.balance_ctrl.RF_FORCE_DES, dt)
# robot.rf_force_traj_gen.x.recompute(0)
# robot.lf_force_traj_gen  = create_force_traj_gen("lf_force_ref", conf.balance_ctrl.LF_FORCE_DES, dt)
# robot.lf_force_traj_gen.x.recompute(0)

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

# --- Switch which synchronizes trajectories
robot.traj_sync = create_trajectory_switch()
trajs = [robot.com_traj_gen, robot.waist_traj_gen, robot.am_traj_gen, robot.phases_traj_gen] # robot.rf_force_traj_gen, robot.lf_force_traj_gen]
trajs += [robot.rf_traj_gen, robot.lf_traj_gen, robot.rh_traj_gen, robot.lh_traj_gen]
connect_synchronous_trajectories(robot.traj_sync, trajs)

# --- CREATE Operational Points ------------------------------------------------
robot.dynamic.createOpPoint('LF', robot.OperationalPointsMap['left-ankle'])
robot.dynamic.createOpPoint('RF', robot.OperationalPointsMap['right-ankle'])
robot.dynamic.signal("LF").recompute(0)
robot.dynamic.signal("RF").recompute(0)

# --- Base Estimator
robot.device_filters = create_device_filters(robot, dt)
robot.imu_filters = create_imu_filters(robot, dt)
robot.base_estimator = create_base_estimator(robot, dt, conf.base_estimator)

robot.m2qLF = MatrixHomoToPoseQuaternion('m2qLF')
plug(robot.dynamic.signal("LF"), robot.m2qLF.sin)
plug(robot.m2qLF.sout, robot.base_estimator.lf_ref_xyzquat)
robot.m2qRF = MatrixHomoToPoseQuaternion('m2qRF')
plug(robot.dynamic.signal("RF"), robot.m2qRF.sin)
plug(robot.m2qRF.sout, robot.base_estimator.rf_ref_xyzquat)

# --- Interface with controller entities

rfootHomo = SE3VectorToMatrixHomo("rfootHomo")
plug(robot.rf_traj_gen.x, rfootHomo.sin)
rfootHomo.sout.recompute(0)
lfootHomo = SE3VectorToMatrixHomo("lfootHomo")
plug(robot.lf_traj_gen.x, lfootHomo.sin)
lfootHomo.sout.recompute(0)

waistToMatrixHomo = SE3VectorToMatrixHomo("waistHomo")
plug(robot.waist_traj_gen.x, waistToMatrixHomo.sin)
waistToMatrixHomo.sout.recompute(0)

dyn_com = robot.dynamic.com.value
diff_translation_com = init_value_com - dyn_com
rotation_identity = np.identity(3).flatten()
refFrame = np.concatenate((diff_translation_com, rotation_identity))
refFrameToMatrixHomo = SE3VectorToMatrixHomo("refFrameHomo")
refFrameToMatrixHomo.sin.value = refFrame
refFrameToMatrixHomo.sout.recompute(0)

dyn_foot = np.array(robot.dynamic.signal("LF").value)[:3, 3]
diff_translation_foot = init_value_lf[:3] - dyn_foot
refFrameFeet = np.concatenate((diff_translation_foot, rotation_identity))
refFrameFeetToMatrixHomo = SE3VectorToMatrixHomo("refFrameFeetHomo")
refFrameFeetToMatrixHomo.sin.value = refFrameFeet
refFrameFeetToMatrixHomo.sout.recompute(0)

wp = DummyWalkingPatternGenerator('dummy_wp')
wp.init()
wp.referenceFrame.value = refFrameToMatrixHomo.sout.value
wp.referenceFrameFeet.value = refFrameFeetToMatrixHomo.sout.value
wp.omega.value = omega
plug(waistToMatrixHomo.sout, wp.waist)
plug(lfootHomo.sout, wp.footLeft)
plug(rfootHomo.sout, wp.footRight)
plug(robot.com_traj_gen.x, wp.com)
plug(robot.com_traj_gen.dx, wp.vcom)
plug(robot.com_traj_gen.ddx, wp.acom)

robot.wp = wp

# --- Compute the values to use them in initialization
robot.wp.comDes.recompute(0)
robot.wp.dcmDes.recompute(0)
robot.wp.zmpDes.recompute(0)

# --- Conversion
e2q = EulerToQuat('e2q')
plug(robot.base_estimator.q, e2q.euler)
robot.e2q = e2q

# --- Kinematic computations
robot.rdynamic = DynamicPinocchio("real_dynamics")
robot.rdynamic.setModel(robot.dynamic.model)
robot.rdynamic.setData(robot.rdynamic.model.createData())
plug(robot.base_estimator.q, robot.rdynamic.signal("position"))
robot.rdynamic.signal("velocity").value = np.zeros(robotDim)
robot.rdynamic.signal("acceleration").value = np.zeros(robotDim)

# --- CoM Estimation
cdc_estimator = DcmEstimator('cdc_estimator')
cdc_estimator.init(dt, robot_name)
plug(robot.e2q.quaternion, cdc_estimator.q)
plug(robot.base_estimator.v, cdc_estimator.v)
robot.cdc_estimator = cdc_estimator

# --- DCM Estimation
estimator = DummyDcmEstimator("dummy")
plug(robot.wp.omegaDes, estimator.omega)
estimator.mass.value = 1.0
plug(robot.cdc_estimator.c, estimator.com)
plug(robot.cdc_estimator.dc, estimator.momenta)
estimator.init()
robot.estimator = estimator

# --- Force calibration
robot.ftc = create_ft_calibrator(robot, ft_conf)

# --- ZMP estimation
zmp_estimator = SimpleZmpEstimator("zmpEst")
robot.rdynamic.createOpPoint('sole_LF', 'left_sole_link')
robot.rdynamic.createOpPoint('sole_RF', 'right_sole_link')
plug(robot.rdynamic.signal("sole_LF"), zmp_estimator.poseLeft)
plug(robot.rdynamic.signal("sole_RF"), zmp_estimator.poseRight)
plug(robot.ftc.left_foot_force_out, zmp_estimator.wrenchLeft)
plug(robot.ftc.right_foot_force_out, zmp_estimator.wrenchRight)
zmp_estimator.init()
robot.zmp_estimator = zmp_estimator

# -------------------------- ADMITTANCE CONTROL --------------------------

# --- DCM controller
Kp_dcm = [8.0]*3
Ki_dcm = [0.0, 0.0, 0.0]  # zero (to be set later)
Kz_dcm = [0.] * 3
gamma_dcm = 0.2

dcm_controller = DcmController("dcmCtrl")

dcm_controller.Kp.value = np.array(Kp_dcm)
dcm_controller.Ki.value = np.array(Ki_dcm)
dcm_controller.Kz.value = np.array(Kz_dcm)
dcm_controller.decayFactor.value = gamma_dcm
dcm_controller.mass.value = mass
plug(robot.wp.omegaDes, dcm_controller.omega)

plug(robot.cdc_estimator.c, dcm_controller.com)
plug(robot.estimator.dcm, dcm_controller.dcm)

plug(robot.wp.zmpDes, dcm_controller.zmpDes)
plug(robot.wp.dcmDes, dcm_controller.dcmDes)

plug(robot.zmp_estimator.zmp, dcm_controller.zmp)

dcm_controller.init(dt)

robot.dcm_control = dcm_controller

Ki_dcm = np.array([0.0, 0.0, 0.0])  # this value is employed later

Kz_dcm = np.array([-1.0, -1.0, -1.0])  # this value is employed later

# --- CoM admittance controller
Kp_adm = np.array([0.0, 0.0, 0.0])  # zero (to be set later)

com_admittance_control = ComAdmittanceController("comAdmCtrl")
com_admittance_control.Kp.value = Kp_adm
plug(robot.zmp_estimator.zmp, com_admittance_control.zmp)
com_admittance_control.zmpDes.value = robot.wp.zmpDes.value  # should be plugged to robot.dcm_control.zmpRef
plug(robot.wp.acomDes, com_admittance_control.ddcomDes)

com_admittance_control.init(dt)
com_admittance_control.setState(robot.wp.comDes.value, np.array([0.0, 0.0, 0.0]))

robot.com_admittance_control = com_admittance_control

Kp_adm = np.array([12.0, 12.0, 0.0])  # this value is employed later

# --- Inverse dynamic controller
robot.inv_dyn = create_balance_controller(robot, conf.balance_ctrl,conf.motor_params, dt, controlType="velocity")
robot.inv_dyn.active_joints.value = np.ones(32)
# plug(robot.device_filters.torque_filter.x_filtered, robot.inv_dyn.tau_measured)
# robot.inv_dyn.ref_pos_final.value = np.array(robot.halfSitting)

# --- Connect control manager
robot.ctrl_manager = create_ctrl_manager(cm_conf, dt, robot_name='robot')
robot.ctrl_manager.addCtrlMode("vel")
robot.ctrl_manager.setCtrlMode("all", "vel")
robot.ctrl_manager.addEmergencyStopSIN('zmp')
plug(robot.inv_dyn.v_des, robot.ctrl_manager.signal('ctrl_vel'))
plug(robot.ctrl_manager.signal('u_safe'), robot.device.control)

# --- Fix robot.dynamic inputs
plug(robot.device.velocity, robot.dynamic.velocity)
plug(robot.device.velocity, robot.vselec.sin)
robot.dvdt = Derivator_of_Vector("dv_dt")
robot.dvdt.dt.value = dt
plug(robot.device.velocity, robot.dvdt.sin)
plug(robot.dvdt.sout, robot.dynamic.acceleration)

# --- Error on the CoM task
robot.errorComTSID = Substract_of_vector('error_com')
plug(robot.inv_dyn.com_ref_pos, robot.errorComTSID.sin2)
plug(robot.inv_dyn.com, robot.errorComTSID.sin1)

# --- Error on the Posture task
robot.errorPoseTSID = Substract_of_vector('error_pose')
plug(robot.inv_dyn.posture_ref_pos, robot.errorPoseTSID.sin2)
plug(robot.encoders.sout, robot.errorPoseTSID.sin1)


# # # --- ROS PUBLISHER ----------------------------------------------------------

robot.publisher = create_rospublish(robot, 'robot_publisher')
create_topic(robot.publisher, robot.errorPoseTSID, 'sout', 'errorPoseTSID', robot=robot, data_type='vector')  
create_topic(robot.publisher, robot.errorComTSID, 'sout', 'errorComTSID', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'com', 'inv_dyn_com', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'com_est', 'inv_dyn_com_est', robot=robot, data_type='vector') 
create_topic(robot.publisher, robot.inv_dyn, 'q_des', 'q_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'v_des', 'v_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'dv_des', 'dv_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'tau_des', 'tau_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'tau_pd_des', 'tau_pd_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'left_foot_pos', 'left_foot_pos', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'right_foot_pos', 'right_foot_pos', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.base_estimator, 'q', 'base_q', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.base_estimator, 'v', 'base_v', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'lf_est', 'lf_estim', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'rf_est', 'rf_estim', robot=robot, data_type='vector')
# create_topic(robot.publisher, robot.lf_force_traj_gen, 'x', 'lf_force_traj_gen', robot=robot, data_type='vector')
# create_topic(robot.publisher, robot.rf_force_traj_gen, 'x', 'rf_force_traj_gen', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.phaseInt, 'sout', 'contactphase', robot=robot, data_type='int')
create_topic(robot.publisher, robot.com_traj_gen, 'x', 'com_traj_gen', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.lf_traj_gen, 'x', 'lf_traj_gen', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.rf_traj_gen, 'x', 'rf_traj_gen', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.device, 'motorcontrol', 'motorcontrol', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.device, 'robotState', 'robotState', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.wp, 'comDes', 'wp_com', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.wp, 'zmpDes', 'wp_zmp', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.com_admittance_control, 'comRef', 'com_adm', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.com_admittance_control, 'dcomRef', 'dcom_adm', robot=robot, data_type='vector')

create_topic(robot.publisher, robot.inv_dyn, 'dcm', 'dcm_estim', robot=robot, data_type='vector')  # estimated DCM
create_topic(robot.publisher, robot.device, 'forceLLEG', 'forceLLEG', robot = robot, data_type='vector') # measured left wrench
create_topic(robot.publisher, robot.device, 'forceRLEG', 'forceRLEG', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.device, 'ptorque', 'tau_meas', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'am_L', 'inv_dyn_am', robot=robot, data_type='vector') 
create_topic(robot.publisher, robot.inv_dyn, 'am_dL', 'inv_dyn_dam', robot=robot, data_type='vector') 
create_topic(robot.publisher, robot.am_traj_gen, 'x', 'am_traj_gen', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.am_traj_gen, 'dx', 'dam_traj_gen', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.dcm_control, 'zmpRef', 'zmp_ref', robot=robot, data_type='vector')  # reference ZMP
create_topic(robot.publisher, robot.dcm_control, 'dcmDes', 'dcm_des', robot=robot, data_type='vector')  # desired DCM
create_topic(robot.publisher, robot.zmp_estimator, 'zmp', 'zmp_estim', robot=robot, data_type='vector')  # estimated ZMP

# # # --- TRACER ----------------------------------------------------------
# robot.tracer = TracerRealTime("inv_dyn_tracer")
# robot.tracer.setBufferSize(80*(2**20))
# robot.tracer.open('/tmp','dg_','.dat')
# robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

# addTrace(robot.tracer, robot.inv_dyn, 'tau_des')
# addTrace(robot.tracer, robot.inv_dyn, 'tau_pd_des')
# addTrace(robot.tracer, robot.inv_dyn, 'q_des')
# addTrace(robot.tracer, robot.inv_dyn, 'v_des')
# addTrace(robot.tracer, robot.inv_dyn, 'dv_des')
# addTrace(robot.tracer, robot.errorPoseTSID, 'sout')
# addTrace(robot.tracer, robot.errorComTSID, 'sout')
# addTrace(robot.tracer, robot.device, 'robotState')
# addTrace(robot.tracer, robot.device, 'motorcontrol')

# robot.tracer.start()
