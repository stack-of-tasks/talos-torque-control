# -*- coding: utf-8 -*-
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

import numpy as np
from dynamic_graph import plug
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import *
from dynamic_graph.sot.torque_control.talos.sot_utils_talos import start_sot, stop_sot, Bunch, start_movement_sinusoid, stop_movement_sinusoid, start_tracer, go_to_position_sinusoid, go_to_position, go_to_SE3_position_fixed_orientation, go_to_SE3_front_orientation, go_to_SE3_right_orientation, go_to_SE3_left_orientation, go_to_SE3_position
from dynamic_graph.sot.torque_control.utils.filter_utils import create_chebi2_lp_filter_Wn_03_N_4
from dynamic_graph.ros import RosPublish, RosSubscribe
from dynamic_graph.tracer_real_time import TracerRealTime
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


def get_conf():
    # import dynamic_graph.sot.torque_control.talos.balance_ctrl_conf as balance_ctrl_conf
    # import dynamic_graph.sot.torque_control.talos.base_estimator_conf as base_estimator_conf
    import dynamic_graph.sot.torque_control.talos.control_manager_conf as control_manager_conf
    import dynamic_graph.sot.torque_control.talos.force_torque_estimator_conf as force_torque_estimator_conf
    import dynamic_graph.sot.torque_control.talos.joint_torque_controller_conf as joint_torque_controller_conf
    import dynamic_graph.sot.torque_control.talos.joint_pos_ctrl_gains as pos_ctrl_gains
    import dynamic_graph.sot.torque_control.talos.motors_parameters as motor_params
    import dynamic_graph.sot.torque_control.talos.ddp_controller_conf as ddp_controller_conf
    
    conf = Bunch();
    # conf.balance_ctrl              = balance_ctrl_conf;
    # conf.base_estimator            = base_estimator_conf;
    conf.control_manager           = control_manager_conf;
    conf.force_torque_estimator    = force_torque_estimator_conf;
    conf.joint_torque_controller   = joint_torque_controller_conf;
    conf.pos_ctrl_gains            = pos_ctrl_gains;
    conf.motor_params              = motor_params;
    conf.ddp_controller            = ddp_controller_conf;
    return conf;

def ddp_actuator(robot, startSoT=True):
    # BUILD THE STANDARD GRAPH
    conf = get_conf();
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

    robot.device.setControlInputType('noInteg');
    robot.ctrl_manager    = create_ctrl_manager(conf.control_manager, conf.motor_params, dt);

    robot.traj_gen        = create_trajectory_generator(robot.device, dt);
    # robot.com_traj_gen    = create_com_traj_gen(conf.balance_ctrl, dt);
    # robot.rf_force_traj_gen  = create_force_traj_gen("rf_force_ref", conf.balance_ctrl.RF_FORCE_DES, dt);
    # robot.lf_force_traj_gen  = create_force_traj_gen("lf_force_ref", conf.balance_ctrl.LF_FORCE_DES, dt);

    # robot.traj_sync       = create_trajectory_switch();
    # robot.rf_traj_gen     = SE3TrajectoryGenerator("tg_rf");
    # robot.lf_traj_gen     = SE3TrajectoryGenerator("tg_lf");
    # robot.rh_traj_gen     = SE3TrajectoryGenerator("tg_rh");
    # robot.lh_traj_gen     = SE3TrajectoryGenerator("tg_lh");
    # robot.rf_traj_gen.init(dt);
    # robot.lf_traj_gen.init(dt);
    # robot.rh_traj_gen.init(dt);
    # robot.lh_traj_gen.init(dt);
    
    robot.encoders                              = create_encoders(robot);
    robot.encoders_velocity                     = create_encoders_velocity(robot);
    robot.joint_pos_selec_ddp                   = create_joint_pos_selector(robot, conf.ddp_controller);
    robot.joint_vel_selec_ddp                   = create_joint_vel_selector(robot, conf.ddp_controller);
    robot.joint_torque_selec_ddp                = create_joint_torque_selector(robot, conf.ddp_controller);
    robot.pos_des_selec_ddp                     = create_pos_des_selector(robot, conf.ddp_controller);
    # robot.motor_pos_selec_ddp                   = create_motor_pos_selector(robot, conf.ddp_controller);
        
    # robot.imu_offset_compensation               = create_imu_offset_compensation(robot, dt);
    #(robot.estimator_ft, robot.filters)         = create_estimators(robot, conf.force_torque_estimator, conf.motor_params, dt);
    # robot.filters                               = create_filters(robot, conf.force_torque_estimator, conf.motor_params, dt);
    # robot.imu_filter                            = create_imu_filter(robot, dt);
    # robot.base_estimator                        = create_base_estimator(robot, dt, conf.base_estimator);

    # connect_synchronous_trajectories(robot.traj_sync,
    #                                  [robot.com_traj_gen,
    #                                   robot.rf_force_traj_gen, robot.lf_force_traj_gen,
    #                                   robot.rf_traj_gen, robot.lf_traj_gen,
    #                                   robot.rh_traj_gen, robot.lh_traj_gen]);
    #robot.rf_traj_gen, robot.lf_traj_gen])

    robot.pos_ctrl          = create_position_controller(robot, conf.pos_ctrl_gains, dt);
    # robot.torque_ctrl       = create_torque_controller(robot, conf.joint_torque_controller, conf.motor_params, dt);
    # robot.inv_dyn           = create_balance_controller(robot, conf.balance_ctrl,conf.motor_params, dt);
    robot.ddp_ctrl          = create_pyrene_ddp_controller(robot, conf.ddp_controller, dt);
    
    connect_ctrl_manager(robot);

    # create low-pass filter for computing joint velocities
    # robot.encoder_filter = create_chebi2_lp_filter_Wn_03_N_4('encoder_filter', dt, conf.motor_params.NJ);
    # plug(robot.encoders.sout,             robot.encoder_filter.x);
    # plug(robot.encoder_filter.dx,         robot.torque_ctrl.jointsVelocities);

    # robot.ros = RosPublish('rosPublish');
    # robot.device.after.addDownsampledSignal('rosPublish.trigger',1);
    
    # plug(robot.torque_ctrl.u,    robot.ctrl_manager.ctrl_torque)
    plug(robot.ddp_ctrl.tau,    robot.ctrl_manager.ctrl_torque)  
    robot.ctrl_manager.setCtrlMode("re", "torque") 
    # robot.inv_dyn.active_joints.value=(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0);    


    if(startSoT):
        print "Gonna start SoT";
        sleep(1.0);
        start_sot();

    
    # # # --- ROS PUBLISHER ----------------------------------------------------------

    robot.publisher = create_rospublish(robot, 'robot_publisher')
    create_topic(robot.publisher, robot.joint_pos_selec_ddp, 'sout', 'joint_pos', robot=robot, data_type='vector')
    # create_topic(robot.publisher, robot.torque_ctrl, 'u', 'torque_ctrl_u', robot=robot, data_type='vector')
    create_topic(robot.publisher, robot.traj_gen, 'q', 'q_des', robot=robot, data_type='vector')
    # # create_topic(robot.publisher, robot.ctrl_manager, 'u', 'manager_u', robot=robot, data_type='vector')
    # # create_topic(robot.publisher, robot.ctrl_manager, 'u_safe', 'manager_u_safe', robot=robot, data_type='vector')
    create_topic(robot.publisher, robot.ddp_ctrl, 'tau', 'ddp_ctrl_tau', robot=robot, data_type='vector')    

    # # # # --- ROS SUBSCRIBER
    # robot.subscriber = RosSubscribe("ddp_subscriber")    
    # robot.subscriber.add("vector", "torque_ctrl_u", "/torque_ctrl/u")
    # robot.subscriber.add("vector", "joint_pos", "/joint_pos_selec_ddp/sout")
    # robot.subscriber.add("vector", "q_des", "/traj_gen/q_des")
    # # robot.subscriber.add("vector", "manager_u", "/ctrl_manager/u")
    # # robot.subscriber.add("vector", "manager_u_safe", "/ctrl_manager/u_safe")
    # robot.subscriber.add("vector", "ddp_ctrl_tau", "/ddp_ctrl/tau")

    return robot;
