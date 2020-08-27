from rospkg import RosPack
from dynamic_graph import plug
from dynamic_graph.sot.torque_control.se3_trajectory_generator import SE3TrajectoryGenerator
from sot_talos_balance.nd_trajectory_generator import NdTrajectoryGenerator
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_trajectory_switch, connect_synchronous_trajectories, create_force_traj_gen
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import NJ, create_rospublish, create_topic, get_sim_conf 
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_waist_traj_gen, create_trajectory_generator, create_com_traj_gen
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_filters, create_encoders, create_am_traj_gen
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_balance_controller, create_simple_inverse_dyn_controller, create_ctrl_manager
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import addTrace, dump_tracer, create_encoders_velocity
from sot_talos_balance.create_entities_utils import create_device_filters, create_imu_filters, create_base_estimator
from dynamic_graph.sot.torque_control.talos.sot_utils_talos import go_to_position
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.sot.core.operator import Substract_of_vector, Component_of_vector, PoseQuatToMatrixHomo#, SE3VectorToMatrixHomo
from sot_talos_balance.boolean_identity import BooleanIdentity
from dynamic_graph.sot.pattern_generator import PatternGenerator
from sot_talos_balance.delay import DelayVector
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Vector

# --- EXPERIMENTAL SET UP ------------------------------------------------------
conf = get_sim_conf()
dt = robot.timeStep
robot.device.setControlInputType('noInteg') # No integration for torque control

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

robot.ctrl_manager = create_ctrl_manager(conf.control_manager, conf.motor_params, dt)
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

# --- Base Estimator
robot.device_filters = create_device_filters(robot, dt)
robot.imu_filters = create_imu_filters(robot, dt)
robot.base_estimator = create_base_estimator(robot, dt, conf.base_estimator)
plug(robot.device_filters.vel_filter.x_filtered, robot.base_estimator.joint_velocities)

robot.base_estimator.q.recompute(0)
robot.base_estimator.v.recompute(0)

robot.quat2mHLF = PoseQuatToMatrixHomo('quat2mHLF')
plug(robot.base_estimator.lf_xyzquat, robot.quat2mHLF.sin)
robot.quat2mHRF = PoseQuatToMatrixHomo('quat2mHRF')
plug(robot.base_estimator.rf_xyzquat, robot.quat2mHRF.sin)

# -------------------------- DESIRED TRAJECTORY --------------------------

rospack = RosPack()  

# -------------------------- PATTERN GENERATOR --------------------------

robot.pg = PatternGenerator('pg')

# URDF PATH 
talos_data_folder = rospack.get_path('talos_data')
robot.pg.setURDFpath(talos_data_folder + '/urdf/talos_reduced_wpg.urdf')
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

# robot.pg.parseCmd(":SetAlgoForZmpTrajectory Naveau")

plug(robot.dynamic.position, robot.pg.position)
# plug(robot.base_estimator.q, robot.pg.position)
plug(robot.dynamic.com, robot.pg.com)
#plug(robot.dynamic.com, robot.pg.comStateSIN)
# plug(robot.dynamic.LF, robot.pg.leftfootcurrentpos)
# plug(robot.dynamic.RF, robot.pg.rightfootcurrentpos)
plug(robot.quat2mHLF.sout, robot.pg.leftfootcurrentpos)
plug(robot.quat2mHRF.sout, robot.pg.rightfootcurrentpos)
robotDim = len(robot.base_estimator.v.value)
robot.pg.motorcontrol.value = robotDim * (0, )
robot.pg.zmppreviouscontroller.value = (0, 0, 0)

robot.pg.initState()

# robot.pg.parseCmd(':setDSFeetDistance 0.162')

# robot.pg.parseCmd(':NaveauOnline')
# robot.pg.parseCmd(':numberstepsbeforestop 2')
# robot.pg.parseCmd(':setfeetconstraint XY 0.091 0.0489')

# robot.pg.parseCmd(':deleteallobstacles')
# robot.pg.parseCmd(':feedBackControl false')
robot.pg.parseCmd(':doublesupporttime 0.115')
robot.pg.parseCmd(':singlesupporttime 0.9')
robot.pg.parseCmd(":SetAlgoForZmpTrajectory Kajita")
robot.pg.parseCmd(":StartOnLineStepSequencing 0.0 -0.085 0.0 0.0 0.2 0.17 0.0 0.0 0.2 -0.17 0.0 0.0 0.2 0.17 0.0 0.0 0.2 -0.17 0.0 0.0 0.2 0.17 0.0 0.0 0.2 -0.17 0.0 0.0 0.0 0.17 0.0 0.0")
robot.pg.parseCmd(":StopOnLineStepSequencing")
robot.pg.parseCmd(':useDynamicFilter true')

# robot.pg.velocitydes.value = (0.1, 0.0, 0.0)  # DEFAULT VALUE (0.1,0.0,0.0)

# -------------------------- TRIGGER --------------------------

robot.triggerPG = BooleanIdentity('triggerPG')
robot.triggerPG.sin.value = 0
plug(robot.triggerPG.sout, robot.pg.trigger)
plug(robot.triggerPG.sout, robot.rh_traj_gen.trigger)
plug(robot.triggerPG.sout, robot.lh_traj_gen.trigger)
plug(robot.pg.jointpositionfrompg, robot.traj_gen.base6d_encoders)
# --- Inverse dynamic controller
robot.inv_dyn = create_balance_controller(robot, conf.balance_ctrl,conf.motor_params, dt, controlType="torque", patternGenerator=True)
robot.inv_dyn.active_joints.value = 32*(1.0,)

# --- Reference position of the feet for base estimator
robot.inv_dyn.left_foot_pos_quat.recompute(0)
robot.inv_dyn.right_foot_pos_quat.recompute(0)
# robot.base_estimator.lf_ref_xyzquat.value = robot.inv_dyn.left_foot_pos_quat.value
# robot.base_estimator.rf_ref_xyzquat.value = robot.inv_dyn.right_foot_pos_quat.value
plug(robot.inv_dyn.left_foot_pos_quat, robot.base_estimator.lf_ref_xyzquat)
plug(robot.inv_dyn.right_foot_pos_quat, robot.base_estimator.rf_ref_xyzquat)

# --- Delay COM
# robot.delay_com = DelayVector("delay_com")
# robot.delay_com.setMemory(robot.dynamic.com.value)
# robot.device.before.addSignal(robot.delay_com.name + '.current')
# plug(robot.inv_dyn.com, robot.delay_com.sin)
# plug(robot.delay_com.previous, robot.pg.com)

# --- Delay position q
robot.delay_pos = DelayVector("delay_pos")
robot.delay_pos.setMemory(robot.base_estimator.q.value)
robot.device.before.addSignal(robot.delay_pos.name + '.current')
plug(robot.inv_dyn.q_des, robot.delay_pos.sin)
plug(robot.delay_pos.previous, robot.pg.position)

# --- Delay velocity dq
robot.delay_vel = DelayVector("delay_vel")
robot.delay_vel.setMemory(robotDim * [0.])
robot.device.before.addSignal(robot.delay_vel.name + '.current')
plug(robot.inv_dyn.v_des, robot.delay_vel.sin)

# --- Delay velocity ddq
robot.delay_acc = DelayVector("delay_acc")
robot.delay_acc.setMemory(robotDim * [0.])
robot.device.before.addSignal(robot.delay_acc.name + '.current')
plug(robot.inv_dyn.dv_des, robot.delay_acc.sin)

# --- Plug inverse_dynamic instead of device
# plug(robot.delay_pos.previous, robot.pselec.sin)
# plug(robot.pselec.sout, robot.base_estimator.joint_positions)
# plug(robot.delay_vel.previous, robot.vselec.sin)

# --- Fix robot.dynamic inputs
plug(robot.delay_pos.previous, robot.dynamic.position)
plug(robot.delay_vel.previous, robot.dynamic.velocity)
plug(robot.delay_acc.previous, robot.dynamic.acceleration)
# robot.dvdt = Derivator_of_Vector("dv_dt")
# robot.dvdt.dt.value = dt
# plug(robot.delay_vel.previous, robot.dvdt.sin)
# plug(robot.dvdt.sout, robot.dynamic.acceleration)

# --- Change actual position feet for pg
# robot.Lfoot_Homo = SE3VectorToMatrixHomo("Lfoot_estim")
# plug(robot.inv_dyn.left_foot_pos, robot.Lfoot_Homo.sin)
# plug(robot.Lfoot_Homo.sout, robot.pg.leftfootcurrentpos)
# robot.Rfoot_Homo = SE3VectorToMatrixHomo("Rfoot_Homo")
# plug(robot.inv_dyn.right_foot_pos, robot.Rfoot_Homo.sin)
# plug(robot.Rfoot_Homo.sout, robot.pg.rightfootcurrentpos)


# --- High gains position controller
from dynamic_graph.sot.torque_control.position_controller import PositionController
posCtrl = PositionController('pos_ctrl')
posCtrl.Kp.value = tuple(conf.pos_ctrl_gains.kp_pos[round(dt,3)]);
posCtrl.Kd.value = tuple(conf.pos_ctrl_gains.kd_pos[round(dt,3)]);
posCtrl.Ki.value = tuple(conf.pos_ctrl_gains.ki_pos[round(dt,3)]);
plug(robot.device.robotState, posCtrl.base6d_encoders);
plug(robot.device_filters.vel_filter.x_filtered, posCtrl.jointsVelocities);
plug(robot.traj_gen.q, posCtrl.qRef);
plug(robot.traj_gen.dq, posCtrl.dqRef);
posCtrl.init(dt, "robot");
robot.pos_ctrl = posCtrl

# --- Connect control manager
plug(robot.device.currents, robot.ctrl_manager.i_measured)
plug(robot.device.ptorque, robot.ctrl_manager.tau)

robot.ctrl_manager.addCtrlMode("torque")
plug(robot.inv_dyn.tau_pd_des, robot.ctrl_manager.ctrl_torque)
robot.ctrl_manager.setCtrlMode("lhy-lhr-lhp-lk-lap-lar-rhy-rhr-rhp-rk-rap-rar-ty-tp-lsy-lsr-lay-le-lwy-lwp-lwr-rsy-rsr-ray-re-rwy-rwp-rwr", "torque")

robot.ctrl_manager.addCtrlMode("pos")
plug(robot.pos_ctrl.pwmDes, robot.ctrl_manager.ctrl_pos)
robot.ctrl_manager.setCtrlMode("lh-rh-hp-hy", "pos")
plug(robot.ctrl_manager.u_safe, robot.device.control)

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
create_topic(robot.publisher, robot.dynamic, 'com', 'dyn_com', robot=robot, data_type='vector') 
create_topic(robot.publisher, robot.inv_dyn, 'q_des', 'q_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'v_des', 'v_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'dv_des', 'dv_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'tau_des', 'tau_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'tau_pd_des', 'tau_pd_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'left_foot_pos', 'left_foot_pos', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'right_foot_pos', 'right_foot_pos', robot=robot, data_type='vector')
# create_topic(robot.publisher, robot.Lfoot_Homo, 'sout', 'left_foot_homo', robot=robot, data_type='matrixHomo')
# create_topic(robot.publisher, robot.Rfoot_Homo, 'sout', 'right_foot_homo', robot=robot, data_type='matrixHomo')
create_topic(robot.publisher, robot.base_estimator, 'q', 'base_q', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.base_estimator, 'v', 'base_v', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.pg, 'comref', 'com_pg', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.pg, 'dcomref', 'dcom_pg', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.pg, 'amref', 'am_pg', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.pg, 'damref', 'dam_pg', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.pg, 'contactphase', 'contactphase', robot=robot, data_type='int')
create_topic(robot.publisher, robot.pg, 'leftfootref', 'lf_pg', robot=robot, data_type='matrixHomo')
create_topic(robot.publisher, robot.pg, 'rightfootref', 'rf_pg', robot=robot, data_type='matrixHomo')
create_topic(robot.publisher, robot.device, 'motorcontrol', 'motorcontrol', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.device, 'robotState', 'robotState', robot=robot, data_type='vector')

create_topic(robot.publisher, robot.pg, 'zmpref', 'zmp_pg', robot=robot, data_type='vector')  # desired ZMP
create_topic(robot.publisher, robot.dynamic, 'zmp', 'zmp_dyn', robot=robot, data_type='vector')  # SOT ZMP
create_topic(robot.publisher, robot.inv_dyn, 'zmp', 'zmp_estim', robot=robot, data_type='vector')  # estimated ZMP
create_topic(robot.publisher, robot.inv_dyn, 'dcm', 'dcm_estim', robot=robot, data_type='vector')  # estimated DCM
create_topic(robot.publisher, robot.device, 'forceLLEG', 'forceLLEG', robot = robot, data_type='vector') # measured left wrench
create_topic(robot.publisher, robot.device, 'forceRLEG', 'forceRLEG', robot = robot, data_type='vector')

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
