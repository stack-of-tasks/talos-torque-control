from math import sqrt
from rospkg import RosPack
from dynamic_graph import plug
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Vector
from dynamic_graph.sot.core.operator import Selec_of_vector, MatrixHomoToPoseQuaternion
from dynamic_graph.sot.core.operator import Substract_of_vector, Component_of_vector#, PoseQuatToMatrixHomo
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Vector
from dynamic_graph.sot.dynamic_pinocchio import DynamicPinocchio

from dynamic_graph.sot.torque_control.se3_trajectory_generator import SE3TrajectoryGenerator
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import NJ, create_rospublish, create_topic, get_sim_conf
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_trajectory_generator
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_encoders, create_encoders_velocity
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_balance_controller, create_simple_inverse_dyn_controller
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import addTrace, dump_tracer
from dynamic_graph.sot.torque_control.talos.sot_utils_talos import go_to_position

from dynamic_graph.sot.pattern_generator import PatternGenerator

from sot_talos_balance.create_entities_utils import create_device_filters, create_imu_filters, create_base_estimator, create_ft_calibrator
from sot_talos_balance.create_entities_utils import create_parameter_server, create_ctrl_manager
from sot_talos_balance.boolean_identity import BooleanIdentity
from sot_talos_balance.euler_to_quat import EulerToQuat
from sot_talos_balance.dummy_walking_pattern_generator import DummyWalkingPatternGenerator
from sot_talos_balance.delay import DelayVector
from sot_talos_balance.dcm_estimator import DcmEstimator
from sot_talos_balance.dummy_dcm_estimator import DummyDcmEstimator
from sot_talos_balance.dcm_controller import DcmController
from sot_talos_balance.simple_zmp_estimator import SimpleZmpEstimator
from sot_talos_balance.com_admittance_controller import ComAdmittanceController
import sot_talos_balance.talos.ft_calibration_conf as ft_conf
import sot_talos_balance.talos.parameter_server_conf as param_server_conf
import sot_talos_balance.talos.control_manager_conf as cm_conf

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
q += [0.35, 0.173046, -0.0002, -0.8, 0.0, -0.0, 0.1, -0.005] # arms
q += [-0.35, -0.173046, 0.0002, -0.8, 0.0, 0.0, 0.1, -0.005] # arms
# q += [0.25847, 0.173046, -0.0002, -0.525366, 0.0, -0.0, 0.1, -0.005] # arms
# q += [-0.25847, -0.173046, 0.0002, -0.525366, 0.0, 0.0, 0.1, -0.005] # arms
q += [0., 0.] # Head

robot.halfSitting = q

# --- CREATE ENTITIES ----------------------------------------------------------
cm_conf.CTRL_MAX = 10.0  # temporary hack
create_parameter_server(param_server_conf, dt)
# robot.ctrl_manager = create_ctrl_manager(cm_conf, dt, robot_name='robot')
# robot.ctrl_manager = create_ctrl_manager(conf.control_manager, conf.motor_params, dt)
robot.encoders = create_encoders(robot)
robot.encoders_velocity = create_encoders_velocity(robot)

# --- Posture trajectory
robot.traj_gen = create_trajectory_generator(robot, dt)
robot.traj_gen.q.recompute(0)

# --- Hands trajectories
robot.rh_traj_gen = SE3TrajectoryGenerator("tg_rh")
robot.lh_traj_gen = SE3TrajectoryGenerator("tg_lh")
robot.rh_traj_gen.init(dt)
robot.lh_traj_gen.init(dt)

# --- CREATE Operational Points ------------------------------------------------
robot.dynamic.createOpPoint('LF', robot.OperationalPointsMap['left-ankle'])
robot.dynamic.createOpPoint('RF', robot.OperationalPointsMap['right-ankle'])
robot.dynamic.LF.recompute(0)
robot.dynamic.RF.recompute(0)

# -------------------------- ESTIMATION --------------------------
# --- Base Estimator
robot.device_filters = create_device_filters(robot, dt)
robot.imu_filters = create_imu_filters(robot, dt)
robot.base_estimator = create_base_estimator(robot, dt, conf.base_estimator)

robot.m2qLF = MatrixHomoToPoseQuaternion('m2qLF')
plug(robot.dynamic.LF, robot.m2qLF.sin)
plug(robot.m2qLF.sout, robot.base_estimator.lf_ref_xyzquat)
robot.m2qRF = MatrixHomoToPoseQuaternion('m2qRF')
plug(robot.dynamic.RF, robot.m2qRF.sin)
plug(robot.m2qRF.sout, robot.base_estimator.rf_ref_xyzquat)

# -------------------------- DESIRED TRAJECTORY & PATTERN GENERATOR --------------------------
rospack = RosPack()
robot.pg = PatternGenerator('pg')

# URDF PATH 
talos_data_folder = rospack.get_path('talos_data')
robot.pg.setURDFpath(talos_data_folder + '/urdf/talos_reduced_v2_wpg.urdf')
robot.pg.setSRDFpath(talos_data_folder + '/srdf/talos_wpg.srdf')

robot.pg.buildModel()

robot.pg.parseCmd(":samplingperiod 0.005")
robot.pg.parseCmd(":previewcontroltime 1.6")
robot.pg.parseCmd(":omega 0.0")
robot.pg.parseCmd(':stepheight 0.05')
robot.pg.parseCmd(':doublesupporttime 0.2')
robot.pg.parseCmd(':singlesupporttime 1.0')
robot.pg.parseCmd(":armparameters 0.5")
robot.pg.parseCmd(":LimitsFeasibility 0.0")
robot.pg.parseCmd(":ZMPShiftParameters 0.015 0.015 0.015 0.015")
robot.pg.parseCmd(":TimeDistributeParameters 2.0 3.5 1.7 3.0")
robot.pg.parseCmd(":UpperBodyMotionParameters -0.1 -1.0 0.0")
robot.pg.parseCmd(":comheight 0.876681")
robot.pg.parseCmd(":setVelReference  0.1 0.0 0.0")

plug(robot.dynamic.position, robot.pg.position)
plug(robot.dynamic.com, robot.pg.com)
plug(robot.dynamic.LF, robot.pg.leftfootcurrentpos)
plug(robot.dynamic.RF, robot.pg.rightfootcurrentpos)
robotDim = len(robot.dynamic.velocity.value)

robot.pg.motorcontrol.value = robotDim * (0, )
robot.pg.zmppreviouscontroller.value = (0, 0, 0)
robot.pg.initState()

robot.pg.parseCmd(":doublesupporttime 0.115")
robot.pg.parseCmd(":singlesupporttime 0.900")
robot.pg.parseCmd(":SetAlgoForZmpTrajectory Kajita")
steps_20 = "0.0 -0.085 0.0 0.0 0.2 0.17 0.0 0.0 0.2 -0.17 0.0 0.0 0.2 0.17 0.0 0.0 0.2 -0.17 0.0 0.0 0.2 0.17 0.0 0.0 0.2 -0.17 0.0 0.0 0.0 0.17 0.0 0.0"
steps_on_spot = "0.0 -0.085 0.0 0.0 0.0 0.17 0.0 0.0 0.0 -0.17 0.0 0.0 0.0 0.17 0.0 0.0 0.0 -0.17 0.0 0.0 0.0 0.17 0.0 0.0 0.0 -0.17 0.0 0.0 0.0 0.17 0.0 0.0"
steps = steps_20 if walk_type == "walk_20" else steps_on_spot
robot.pg.parseCmd(":StartOnLineStepSequencing " + steps)
robot.pg.parseCmd(":StopOnLineStepSequencing")
robot.pg.parseCmd(':useDynamicFilter true')

# -------------------------- TRIGGER --------------------------

robot.triggerPG = BooleanIdentity('triggerPG')
robot.triggerPG.sin.value = 0
plug(robot.triggerPG.sout, robot.pg.trigger)
plug(robot.triggerPG.sout, robot.rh_traj_gen.trigger)
plug(robot.triggerPG.sout, robot.lh_traj_gen.trigger)
plug(robot.pg.jointpositionfrompg, robot.traj_gen.base6d_encoders)

# --- Interface with controller entities

wp = DummyWalkingPatternGenerator('dummy_wp')
wp.init()
wp.omega.value = omega
plug(robot.pg.waistattitudematrixabsolute, wp.waist)
plug(robot.pg.leftfootref, wp.footLeft)
plug(robot.pg.rightfootref, wp.footRight)
plug(robot.pg.comref, wp.com)
plug(robot.pg.dcomref, wp.vcom)
plug(robot.pg.ddcomref, wp.acom)

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
plug(robot.base_estimator.q, robot.rdynamic.position)
robot.rdynamic.velocity.value = [0.0] * robotDim
robot.rdynamic.acceleration.value = [0.0] * robotDim

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
plug(robot.rdynamic.sole_LF, zmp_estimator.poseLeft)
plug(robot.rdynamic.sole_RF, zmp_estimator.poseRight)
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

dcm_controller.Kp.value = Kp_dcm
dcm_controller.Ki.value = Ki_dcm
dcm_controller.Kz.value = Kz_dcm
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

Ki_dcm = [1.0, 1.0, 1.0]  # this value is employed later

Kz_dcm = [1.0, 1.0, 1.0]  # this value is employed later

# --- CoM admittance controller
Kp_adm = [0.0, 0.0, 0.0]  # zero (to be set later)

com_admittance_control = ComAdmittanceController("comAdmCtrl")
com_admittance_control.Kp.value = Kp_adm
plug(robot.zmp_estimator.zmp, com_admittance_control.zmp)
com_admittance_control.zmpDes.value = robot.wp.zmpDes.value  # is plugged to robot.dcm_control.zmpRef later
plug(robot.wp.acomDes, com_admittance_control.ddcomDes)

com_admittance_control.init(dt)
com_admittance_control.setState(robot.wp.comDes.value, [0.0, 0.0, 0.0])

robot.com_admittance_control = com_admittance_control

Kp_adm = [12.0, 12.0, 0.0]  # this value is employed later

# --- Inverse dynamic controller
robot.inv_dyn = create_balance_controller(robot, conf.balance_ctrl, conf.motor_params, dt, controlType="velocity", patternGenerator=True)
robot.inv_dyn.active_joints.value = 32*(1.0,)

# --- Connect control manager
robot.ctrl_manager = create_ctrl_manager(cm_conf, dt, robot_name='robot')
robot.ctrl_manager.addCtrlMode("vel")
robot.ctrl_manager.setCtrlMode("all", "vel")
robot.ctrl_manager.addEmergencyStopSIN('zmp')
plug(robot.inv_dyn.v_des, robot.ctrl_manager.ctrl_vel)
plug(robot.ctrl_manager.u_safe, robot.device.control)


# --- Delay position q
robot.delay_pos = DelayVector("delay_pos")
robot.delay_pos.setMemory(robot.base_estimator.q.value)
robot.device.before.addSignal(robot.delay_pos.name + '.current')
plug(robot.inv_dyn.q_des, robot.delay_pos.sin)

# --- Delay velocity dq
robot.delay_vel = DelayVector("delay_vel")
robot.delay_vel.setMemory(robotDim * [0.])
robot.device.before.addSignal(robot.delay_vel.name + '.current')
plug(robot.ctrl_manager.u_safe, robot.delay_vel.sin)

# --- Delay velocity ddq
robot.delay_acc = DelayVector("delay_acc")
robot.delay_acc.setMemory(robotDim * [0.])
robot.device.before.addSignal(robot.delay_acc.name + '.current')
plug(robot.inv_dyn.dv_des, robot.delay_acc.sin)

# --- Fix robot.dynamic inputs
plug(robot.delay_pos.previous, robot.dynamic.position)
plug(robot.delay_vel.previous, robot.dynamic.velocity)
plug(robot.delay_vel.previous, robot.vselec.sin)
plug(robot.delay_acc.previous, robot.dynamic.acceleration)

# --- Error on the CoM task
robot.errorComTSID = Substract_of_vector('error_com')
plug(robot.inv_dyn.com_ref_pos, robot.errorComTSID.sin2)
plug(robot.inv_dyn.com, robot.errorComTSID.sin1)

# --- Error on the Posture task
robot.errorPoseTSID = Substract_of_vector('error_pose')
plug(robot.inv_dyn.posture_ref_pos, robot.errorPoseTSID.sin2)
plug(robot.encoders.sout, robot.errorPoseTSID.sin1)


# --- ROS PUBLISHER ----------------------------------------------------------

robot.publisher = create_rospublish(robot, 'robot_publisher')
create_topic(robot.publisher, robot.errorPoseTSID, 'sout', 'errorPoseTSID', robot=robot, data_type='vector')  
create_topic(robot.publisher, robot.errorComTSID, 'sout', 'errorComTSID', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'com', 'inv_dyn_com', robot=robot, data_type='vector')
# create_topic(robot.publisher, robot.inv_dyn, 'com_est', 'inv_dyn_com_est', robot=robot, data_type='vector') 
create_topic(robot.publisher, robot.inv_dyn, 'q_des', 'q_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'v_des', 'v_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'dv_des', 'dv_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'tau_des', 'tau_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'tau_pd_des', 'tau_pd_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'left_foot_pos', 'left_foot_pos', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'right_foot_pos', 'right_foot_pos', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.base_estimator, 'q', 'base_q', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.base_estimator, 'v', 'base_v', robot=robot, data_type='vector')
# create_topic(robot.publisher, robot.inv_dyn, 'lf_est', 'lf_estim', robot=robot, data_type='vector')
# create_topic(robot.publisher, robot.inv_dyn, 'rf_est', 'rf_estim', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.pg, 'comref', 'com_pg', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.pg, 'dcomref', 'dcom_pg', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.pg, 'amref', 'am_pg', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.pg, 'damref', 'dam_pg', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.pg, 'contactphase', 'contactphase', robot=robot, data_type='int')
create_topic(robot.publisher, robot.pg, 'leftfootref', 'lf_pg', robot=robot, data_type='matrixHomo')
create_topic(robot.publisher, robot.pg, 'rightfootref', 'rf_pg', robot=robot, data_type='matrixHomo')
create_topic(robot.publisher, robot.device, 'motorcontrol', 'motorcontrol', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.device, 'robotState', 'robotState', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.cdc_estimator, 'c', 'com_est', robot=robot, data_type='vector')  # estimated CoM
create_topic(robot.publisher, robot.cdc_estimator, 'dc', 'dcom_est', robot=robot, data_type='vector')  # estimated CoM velocity
create_topic(robot.publisher, robot.com_admittance_control, 'comRef', 'com_adm_ref', robot=robot, data_type='vector')  # reference CoM

create_topic(robot.publisher, robot.wp, 'zmpDes', 'zmp_pg', robot=robot, data_type='vector')  # desired ZMP
create_topic(robot.publisher, robot.dynamic, 'zmp', 'zmp_dyn', robot=robot, data_type='vector')  # SOT ZMP
create_topic(robot.publisher, robot.zmp_estimator, 'zmp', 'zmp_estim', robot=robot, data_type='vector')  # estimated ZMP
create_topic(robot.publisher, robot.dcm_control, 'zmpRef', 'zmp_ref_dcm', robot=robot, data_type='vector')  # reference ZMP
create_topic(robot.publisher, robot.estimator, 'dcm', 'dcm_estim', robot=robot, data_type='vector')  # estimated dcm
create_topic(robot.publisher, robot.dcm_control, 'dcmDes', 'dcm_des', robot=robot, data_type='vector')  # desired DCM
create_topic(robot.publisher, robot.device, 'forceLLEG', 'forceLLEG', robot = robot, data_type='vector') # measured left wrench
create_topic(robot.publisher, robot.device, 'forceRLEG', 'forceRLEG', robot = robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'am_dL', 'am_dL', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'am_L', 'am_L', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.device, 'ptorque', 'tau_meas', robot = robot, data_type='vector')

# # # --- TRACER ----------------------------------------------------------
robot.tracer = TracerRealTime("inv_dyn_tracer")
robot.tracer.setBufferSize(80*(2**20))
robot.tracer.open('/tmp','dg_','.dat')
robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

addTrace(robot.tracer, robot.inv_dyn, 'tau_des')
addTrace(robot.tracer, robot.inv_dyn, 'tau_pd_des')
addTrace(robot.tracer, robot.inv_dyn, 'q_des')
addTrace(robot.tracer, robot.inv_dyn, 'v_des')
addTrace(robot.tracer, robot.inv_dyn, 'dv_des')
addTrace(robot.tracer, robot.errorPoseTSID, 'sout')
addTrace(robot.tracer, robot.errorComTSID, 'sout')
addTrace(robot.tracer, robot.device, 'robotState')
addTrace(robot.tracer, robot.device, 'motorcontrol')

robot.tracer.start()
