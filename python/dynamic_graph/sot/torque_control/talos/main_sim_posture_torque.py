from dynamic_graph import plug
import numpy as np
from dynamic_graph.sot.core.operator import Selec_of_vector, Substract_of_vector, Component_of_vector, Stack_of_vector
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_trajectory_switch, connect_synchronous_trajectories
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import NJ, create_rospublish, create_topic, get_default_conf, get_sim_conf, create_encoders_velocity
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_waist_traj_gen, create_trajectory_generator, create_com_traj_gen, create_encoders
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_posture_task
from dynamic_graph.sot_talos_balance.create_entities_utils import fill_parameter_server, create_ctrl_manager
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import addTrace, dump_tracer
from dynamic_graph.sot.torque_control.talos.sot_utils_talos import go_to_position, go_to_position_sinusoid
from dynamic_graph.sot_talos_balance.create_entities_utils import create_device_filters, create_imu_filters, create_base_estimator
from dynamic_graph.tracer_real_time import TracerRealTime
import dynamic_graph.sot_talos_balance.talos.control_manager_conf as cm_conf

# --- EXPERIMENTAL SET UP ------------------------------------------------------
#conf = get_sim_conf()
conf = get_default_conf()
dt = robot.timeStep
robot.device.setControlInputType('noInteg') # No integration for torque control

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
fill_parameter_server(robot.param_server, conf.control_manager, dt)
robot.encoders = create_encoders(robot)
robot.encoders_velocity = create_encoders_velocity(robot)

# --- Posture trajectory
robot.traj_gen = create_trajectory_generator(robot, dt)
robot.traj_gen.q.recompute(0)

# --- Base Estimator
robot.device_filters = create_device_filters(robot, dt)
robot.imu_filters = create_imu_filters(robot, dt)
robot.base_estimator = create_base_estimator(robot, dt, conf.base_estimator)
plug(robot.device_filters.vel_filter.x_filtered, robot.base_estimator.joint_velocities)

robot.base_estimator.q.recompute(0)
robot.base_estimator.v.recompute(0)

# --- Simple inverse dynamic controller
robot.inv_dyn = create_posture_task(robot, conf.balance_ctrl, dt)
robot.inv_dyn.setControlOutputType("torque")
robot.inv_dyn.w_posture.value = 1
robot.inv_dyn.kp_posture.value  = np.array((3000., 3000., 3000., 3000., 5000., 5000., 3000., 3000., 3000., 3000., 5000., 5000., 3000., 3000., 2000., 2000., 2000., 2000., 100., 100., 100., 100., 2000., 2000., 2000., 2000., 100., 100., 100., 100., 100., 100.)) # proportional gain of postural task
# robot.inv_dyn.kd_posture.value  = np.array(2 * np.sqrt(robot.inv_dyn.kp_posture.value))
robot.inv_dyn.kd_posture.value  = np.array((100., 100., 100., 100., 150., 150., 100., 100., 100., 100., 150., 150., 100., 100., 100., 100., 100., 100., 5., 5., 5., 5., 100., 100., 100., 100., 5., 5., 5., 5., 5, 5)) # derivative gain of postural task
robot.inv_dyn.active_joints.value = np.ones(32)

# --- Reference position of the feet for base estimator
robot.inv_dyn.left_foot_pos.recompute(0)
robot.inv_dyn.right_foot_pos.recompute(0)
robot.base_estimator.lf_ref_xyzquat.value = robot.inv_dyn.left_foot_pos.value
robot.base_estimator.rf_ref_xyzquat.value = robot.inv_dyn.right_foot_pos.value

# --- Connect control manager
robot.ctrl_manager = create_ctrl_manager(cm_conf, dt, robot_name='robot')
effortLimit = 0.9 * robot.dynamic.model.effortLimit[6:]
robot.ctrl_manager.u_max.value = np.concatenate((100*np.ones(6), effortLimit))
robot.ff_torque = Stack_of_vector('ff_torque')
robot.ff_torque.sin1.value = np.zeros(6)
plug(robot.inv_dyn.tau_des, robot.ff_torque.sin2)
robot.ff_torque.selec1(0, 6)
robot.ff_torque.selec2(0, 32) 

robot.ctrl_manager.addCtrlMode("torque")
robot.ctrl_manager.setCtrlMode("lh-rh-hp-hy-lhy-lhr-lhp-lk-lap-lar-rhy-rhr-rhp-rk-rap-rar-ty-tp-lsy-lsr-lay-le-lwy-lwp-lwr-rsy-rsr-ray-re-rwy-rwp-rwr", "torque")
plug(robot.ff_torque.sout, robot.ctrl_manager.signal('ctrl_torque'))


robot.ctrl_manager.addCtrlMode("base")
robot.ctrl_manager.setCtrlMode("freeflyer", "base")
plug(robot.inv_dyn.q_des, robot.ctrl_manager.signal('ctrl_base'))

plug(robot.ctrl_manager.signal('u_safe'), robot.device.control)

robot.odom = Selec_of_vector('odom')
plug(robot.base_estimator.q, robot.odom.sin)
robot.odom.selec(0,6)

# # # --- ROS PUBLISHER ----------------------------------------------------------

robot.publisher = create_rospublish(robot, 'robot_publisher')
# create_topic(robot.publisher, robot.errorPoseTSID, 'sout', 'errorPoseTSID', robot=robot, data_type='vector')  
# create_topic(robot.publisher, robot.errorComTSID, 'sout', 'errorComTSID', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.odom, 'sout', 'base_odom', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'com', 'com', robot=robot, data_type='vector') 
create_topic(robot.publisher, robot.inv_dyn, 'q_des', 'q_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'tau_des', 'tau_des', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.base_estimator, 'q', 'base_q', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.base_estimator, 'v', 'base_v', robot=robot, data_type='vector')

create_topic(robot.publisher, robot.inv_dyn, 'left_foot_pos', 'LF_pos_inv_dyn', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.inv_dyn, 'right_foot_pos', 'RF_pos_inv_dyn', robot=robot, data_type='vector')

create_topic(robot.publisher, robot.device, 'motorcontrol', 'motorcontrol', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.device, 'ptorque', 'tau_meas', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.device, 'robotVelocity', 'device_rV', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.device, 'robotState', 'device_rq', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.device, 'state', 'device_q', robot=robot, data_type='vector')
create_topic(robot.publisher, robot.device, 'velocity', 'device_v', robot=robot, data_type='vector') 
create_topic(robot.publisher, robot.device_filters.vel_filter, 'x_filtered', 'v_filt', robot=robot, data_type='vector')

# # --- TRACER
# robot.tracer = TracerRealTime("tau_tracer")
# robot.tracer.setBufferSize(80*(2**20))
# robot.tracer.open('/tmp','dg_','.dat')
# robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

# addTrace(robot.tracer, robot.inv_dyn, 'tau_des')
# addTrace(robot.tracer, robot.inv_dyn, 'q_des')
# addTrace(robot.tracer, robot.inv_dyn, 'v_des')
# addTrace(robot.tracer, robot.inv_dyn, 'dv_des')
# addTrace(robot.tracer, robot.errorPoseTSID, 'sout')
# addTrace(robot.tracer, robot.errorComTSID, 'sout')
# addTrace(robot.tracer, robot.device, 'robotState')
# addTrace(robot.tracer, robot.device, 'motorcontrol')

# robot.tracer.start()