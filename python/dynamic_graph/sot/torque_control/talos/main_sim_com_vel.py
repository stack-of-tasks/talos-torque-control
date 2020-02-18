# -*- coding: utf-8 -*-1
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

from dynamic_graph import plug
from dynamic_graph.sot.core import Selec_of_vector
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import NJ
from dynamic_graph.sot.torque_control.se3_trajectory_generator import SE3TrajectoryGenerator
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_trajectory_switch, connect_synchronous_trajectories, create_force_traj_gen, create_waist_traj_gen
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_trajectory_generator, create_com_traj_gen, create_encoders, create_encoders_velocity
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_imu_offset_compensation, create_filters, create_imu_filter
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_base_estimator, create_position_controller, create_torque_controller
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_simple_inverse_dyn_controller, create_ctrl_manager
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_free_flyer_locator#, create_flex_estimator, create_floatingBase
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_current_controller, connect_ctrl_manager, addTrace, dump_tracer
from dynamic_graph.ros import RosPublish
from dynamic_graph.sot.torque_control.talos.sot_utils_talos import start_sot, stop_sot, Bunch, start_movement_sinusoid, stop_movement_sinusoid, go_to_position
from dynamic_graph.sot.torque_control.utils.filter_utils import create_chebi2_lp_filter_Wn_03_N_4
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.sot.core import Substract_of_vector
from time import sleep

def create_rospublish(robot, name):
    rospub = RosPublish(name)
    robot.device.after.addSignal(rospub.name+'.trigger')
    return rospub

def create_topic(rospub, entity, signalName, renameSignal, robot=None, data_type='vector'):
    # check needed to prevent creation of broken topic
    if not entity.hasSignal(signalName):
        raise AttributeError('Entity %s does not have signal %s' %
                             (entity.name, signalName))
    rospub_signalName = '{0}_{1}'.format(entity.name, renameSignal)
    topicname = '/ddp/{0}'.format(renameSignal)
    rospub.add(data_type, rospub_signalName, topicname)
    plug(entity.signal(signalName), rospub.signal(rospub_signalName))
    if robot is not None:
        robot.device.after.addSignal('{0}.{1}'.format(entity.name, signalName))


def get_default_conf():
    import dynamic_graph.sot.torque_control.talos.balance_ctrl_conf as balance_ctrl_conf
    import dynamic_graph.sot.torque_control.talos.base_estimator_conf as base_estimator_conf
    import dynamic_graph.sot.torque_control.talos.control_manager_conf as control_manager_conf
    import dynamic_graph.sot.torque_control.talos.force_torque_estimator_conf as force_torque_estimator_conf
    import dynamic_graph.sot.torque_control.talos.joint_torque_controller_conf as joint_torque_controller_conf
    import dynamic_graph.sot.torque_control.talos.joint_pos_ctrl_gains as pos_ctrl_gains
    import dynamic_graph.sot.torque_control.talos.motors_parameters as motor_params
    import dynamic_graph.sot.torque_control.talos.ddp_controller_conf as ddp_controller_conf
    conf = Bunch();
    conf.balance_ctrl              = balance_ctrl_conf;
    conf.adm_ctrl                  = admittance_ctrl_conf;
    conf.base_estimator            = base_estimator_conf;
    conf.control_manager           = control_manager_conf;
    conf.current_ctrl              = current_controller_conf;
    conf.force_torque_estimator    = force_torque_estimator_conf;
    conf.joint_torque_controller   = joint_torque_controller_conf;
    conf.pos_ctrl_gains            = pos_ctrl_gains;
    conf.motor_params              = motor_params;
    return conf;

def get_sim_conf():
    import dynamic_graph.sot.torque_control.talos.balance_ctrl_sim_conf as balance_ctrl_conf
    import dynamic_graph.sot.torque_control.talos.base_estimator_sim_conf as base_estimator_conf
    import dynamic_graph.sot.torque_control.talos.control_manager_sim_conf as control_manager_conf
    import dynamic_graph.sot.torque_control.talos.current_controller_sim_conf as current_controller_conf
    import dynamic_graph.sot.torque_control.talos.force_torque_estimator_conf as force_torque_estimator_conf
    import dynamic_graph.sot.torque_control.talos.joint_torque_controller_conf as joint_torque_controller_conf
    import dynamic_graph.sot.torque_control.talos.joint_pos_ctrl_gains_sim as pos_ctrl_gains
    import dynamic_graph.sot.torque_control.talos.motors_parameters as motor_params
    conf = Bunch();
    conf.balance_ctrl              = balance_ctrl_conf;
    conf.base_estimator            = base_estimator_conf;
    conf.control_manager           = control_manager_conf;
    conf.current_ctrl              = current_controller_conf;
    conf.force_torque_estimator    = force_torque_estimator_conf;
    conf.joint_torque_controller   = joint_torque_controller_conf;
    conf.pos_ctrl_gains            = pos_ctrl_gains;
    conf.motor_params              = motor_params;
    return conf;


''' Main function to call before starting the graph. '''
def main_com(robot, startSoT=True, go_half_sitting=True, use_real_vel=True, use_real_base_state=True, conf=None):
    if(conf is None):
        conf = get_sim_conf();
    dt = robot.timeStep;
    
    # TMP: overwrite halfSitting configuration to use SoT joint order
    robot.halfSitting = (
         # Free flyer
         0., 0., 1.018213, 0., 0. , 0.,
         # legs
         0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,
         0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,
         # Chest
         0.0 ,  0.006761,
         # arms
         0.25847 ,  0.173046, -0.0002, -0.525366, 0.0, -0.0,  0.1, -0.005,
         -0.25847 , -0.173046, 0.0002  , -0.525366, 0.0,  0.0,  0.1,-0.005,
         # Head
         0.0,0.0);
    
    # robot.device.setControlInputType('noInteg')
    robot.ctrl_manager    = create_ctrl_manager(conf.control_manager, conf.motor_params, dt)
    
  
    robot.encoders                              = create_encoders(robot)
    # robot.encoders_velocity                     = create_encoders_velocity(robot)
    # robot.imu_offset_compensation               = create_imu_offset_compensation(robot, dt)
    # robot.filters                               = create_filters(robot, conf.force_torque_estimator, conf.motor_params, dt)
    # robot.imu_filter                            = create_imu_filter(robot, dt)
    # robot.base_estimator                        = create_base_estimator(robot, dt, conf.base_estimator)

    robot.traj_gen        = create_trajectory_generator(robot, dt)
    robot.traj_gen.q.recompute(0)
    robot.com_traj_gen    = create_com_traj_gen(robot, dt)
    robot.com_traj_gen.x.recompute(0)
    # robot.rf_force_traj_gen  = create_force_traj_gen("rf_force_ref", conf.balance_ctrl.RF_FORCE_DES, dt)
    # robot.lf_force_traj_gen  = create_force_traj_gen("lf_force_ref", conf.balance_ctrl.LF_FORCE_DES, dt)
    robot.waist_traj_gen     = create_waist_traj_gen("tg_waist_ref", robot, dt)
    robot.waist_traj_gen.x.recompute(0)

    # robot.traj_sync       = create_trajectory_switch()
    # robot.rf_traj_gen     = SE3TrajectoryGenerator("tg_rf")
    # robot.lf_traj_gen     = SE3TrajectoryGenerator("tg_lf")
    # robot.rh_traj_gen     = SE3TrajectoryGenerator("tg_rh")
    # robot.lh_traj_gen     = SE3TrajectoryGenerator("tg_lh")
    # robot.rf_traj_gen.init(dt)
    # robot.lf_traj_gen.init(dt)
    # robot.rh_traj_gen.init(dt)
    # robot.lh_traj_gen.init(dt)
    
    # connect_synchronous_trajectories(robot.traj_sync,
    #                                  [robot.com_traj_gen, robot.waist_traj_gen])#,
    #                                   robot.rf_force_traj_gen, robot.lf_force_traj_gen, 
    #                                   robot.rf_traj_gen, robot.lf_traj_gen,
    #                                   robot.rh_traj_gen, robot.lh_traj_gen])

    # robot.pos_ctrl        = create_position_controller(robot, conf.pos_ctrl_gains, dt)
    # robot.torque_ctrl     = create_torque_controller(robot, conf.joint_torque_controller, conf.motor_params, dt)
    robot.inv_dyn         = create_simple_inverse_dyn_controller(robot, conf.balance_ctrl, dt)
    robot.inv_dyn.setControlOutputType("velocity")

    # plug(robot.inv_dyn.tau_des, robot.torque_ctrl.jointsTorquesDesired)
    # robot.inv_dyn.active_joints.value=(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0);    

    # Connect control manager
    plug(robot.device.currents,   robot.ctrl_manager.i_measured)
    plug(robot.device.ptorque,    robot.ctrl_manager.tau)
    robot.ctrl_manager.addCtrlMode("vel")
    plug(robot.inv_dyn.v_des, robot.device.control)
    robot.ctrl_manager.setCtrlMode("all", "vel")
    plug(robot.ctrl_manager.joints_ctrl_mode_vel, robot.inv_dyn.active_joints) 
    # plug(robot.ctrl_manager.u_safe,  robot.device.control)

    # Errors
    robot.errorComTSID = Substract_of_vector('error_com')
    plug(robot.inv_dyn.com_ref_pos, robot.errorComTSID.sin2)
    plug(robot.dynamic.com, robot.errorComTSID.sin1)
    

    robot.errorPoseTSID = Substract_of_vector('error_pose')
    plug(robot.inv_dyn.posture_ref_pos,   robot.errorPoseTSID.sin2)
    plug(robot.encoders.sout,     robot.errorPoseTSID.sin1)

    # robot.errorWaistTSID = Add_of_vector('error_waist')
    # plug(-robot.inv_dyn.waist_ref_pos,   robot.errorWaistTSID.sin1)
    # ref_waist = robot.dynamic.data.oMi[robot.dynamic.model.getJointId('root_joint')]
    # trans = ref_waist.translation
    # rot = ref_waist.rotation
    # rot.resize(9,1)
    # matrix = np.concatenate((trans,rot))
    # waist = []
    # for i in range(len(matrix)):
    #     waist += [matrix[i,0]]
    # waist = tuple(waist)
    # plug(robot.robot.dynamic.com,     robot.errorWaistTSID.sin2)
    # robot.errorWaistTSID.sout
    


    '''# force current measurements to zero
    robot.ctrl_manager.i_measured.value = NJ*(0.0,);
    #robot.current_ctrl.i_measured.value = NJ*(0.0,);
    robot.filters.current_filter.x.value = NJ*(0.0,);'''

    # BYPASS TORQUE CONTROLLER
    # plug(robot.inv_dyn.tau_des,     robot.ctrl_manager.ctrl_torque);

    # CREATE SIGNALS WITH ROBOT STATE WITH CORRECT SIZE (36)
    # robot.q = Selec_of_vector("q");
    # plug(robot.device.robotState, robot.q.sin);
    # robot.q.selec(0, NJ+6);
    # plug(robot.q.sout,              robot.pos_ctrl.base6d_encoders);
    # plug(robot.q.sout,              robot.traj_gen.base6d_encoders);

    # # BYPASS JOINT VELOCITY ESTIMATOR
    # if(use_real_vel):
    #     robot.dq = Selec_of_vector("dq");
    #     plug(robot.device.velocity, robot.dq.sin);
    #     robot.dq.selec(6, NJ+6);
    #     # plug(robot.device.velocity,     robot.pos_ctrl.jointsVelocities);
    #     plug(robot.dq.sout,             robot.base_estimator.joint_velocities);
    #     plug(robot.device.gyrometer,    robot.base_estimator.gyroscope);

    # BYPASS BASE ESTIMATOR
    # if(use_real_base_state):
    #     plug(robot.q.sout,              robot.inv_dyn.q);
    #     plug(robot.device.robotVelocity,robot.inv_dyn.v);

    # create low-pass filter for computing joint velocities
    #robot.encoder_filter = create_chebi2_lp_filter_Wn_03_N_4('encoder_filter', dt, conf.motor_params.NJ)
    #plug(robot.encoders.sout,             robot.encoder_filter.x)
    # plug(robot.encoder_filter.dx,         robot.current_ctrl.dq)
    #plug(robot.encoder_filter.dx,         robot.torque_ctrl.jointsVelocities)
    #plug(robot.encoder_filter.x_filtered, robot.base_estimator.joint_positions)
    #plug(robot.encoder_filter.dx,         robot.base_estimator.joint_velocities)'''


    robot.ros = RosPublish('rosPublish');
    robot.device.after.addDownsampledSignal('rosPublish.trigger',1);

    #robot.estimator_ft.dgyro.value = (0.0, 0.0, 0.0);
    #robot.estimator_ft.gyro.value = (0.0, 0.0, 0.0);
#    estimator.accelerometer.value = (0.0, 0.0, 9.81);
    # if(startSoT):
    #     print "Gonna start SoT";
    #     sleep(1.0);
    #     start_sot();

    # if(go_half_sitting):
    #     print "Gonna go to half sitting in 1 sec";
    #     sleep(1.0);
    #     go_to_position(robot.traj_gen, robot.halfSitting[6:], 10.0);


    # # # --- ROS PUBLISHER ----------------------------------------------------------

    robot.publisher = create_rospublish(robot, 'robot_publisher')
    create_topic(robot.publisher, robot.errorPoseTSID, 'sout', 'errorPoseTSID', robot=robot, data_type='vector')  
    create_topic(robot.publisher, robot.errorComTSID, 'sout', 'errorComTSID', robot=robot, data_type='vector')
    create_topic(robot.publisher, robot.dynamic, 'com', 'dynCom', robot=robot, data_type='vector') 
    create_topic(robot.publisher, robot.inv_dyn, 'q_des', 'q_des', robot=robot, data_type='vector')
    create_topic(robot.publisher, robot.device, 'motorcontrol', 'motorcontrol', robot=robot, data_type='vector')
    create_topic(robot.publisher, robot.device, 'robotState', 'robotState', robot=robot, data_type='vector')

    # # # create_topic(robot.publisher, robot.com_traj_gen, 'x', 'com_pos', robot=robot, data_type='vector')
    # # # create_topic(robot.publisher, robot.inv_dyn, 'com_ref_pos', 'inv_com_ref_pos', robot=robot, data_type='vector')
    # create_topic(robot.publisher, robot.inv_dyn, 'tau_des', 'inv_tau_des', robot=robot, data_type='vector')   
    # # # create_topic(robot.publisher, robot.torque_ctrl, 'jointsTorques', 'torque_ctrl_tau', robot=robot, data_type='vector') 
    # # create_topic(robot.publisher, robot.pos_ctrl, 'pwmDes', 'pos_ctrl_tau', robot=robot, data_type='vector')    

    # # --- TRACER
    robot.tracer = TracerRealTime("tau_tracer")
    robot.tracer.setBufferSize(80*(2**20))
    robot.tracer.open('/tmp','dg_','.dat')
    robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name))

    addTrace(robot.tracer, robot.inv_dyn, 'tau_des')
    addTrace(robot.tracer, robot.inv_dyn, 'q_des')
    addTrace(robot.tracer, robot.inv_dyn, 'v_des')
    addTrace(robot.tracer, robot.inv_dyn, 'dv_des')
    addTrace(robot.tracer, robot.errorPoseTSID, 'sout')
    addTrace(robot.tracer, robot.errorComTSID, 'sout')
    addTrace(robot.tracer, robot.device, 'robotState')
    addTrace(robot.tracer, robot.device, 'motorcontrol')
    # addTrace(robot.tracer, robot.inv_dyn, 'check_Ma_tau')
    # addTrace(robot.tracer, robot.pos_ctrl, 'pwmDes')
    
    robot.tracer.start()
    return robot;

