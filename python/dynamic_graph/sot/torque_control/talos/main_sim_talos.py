# -*- coding: utf-8 -*-
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

import numpy as np
from dynamic_graph import plug
from dynamic_graph.sot.core import Selec_of_vector
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import NJ
from dynamic_graph.sot.torque_control.talos.sot_utils_talos import start_sot, stop_sot, Bunch, start_movement_sinusoid, stop_movement_sinusoid, start_tracer, go_to_position, go_to_SE3_position_fixed_orientation, go_to_SE3_front_orientation, go_to_SE3_right_orientation, go_to_SE3_left_orientation, go_to_SE3_position
from dynamic_graph.ros import RosPublish
# from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_topic
from dynamic_graph.sot.torque_control.talos.main_talos import main_v3, main_v4
from dynamic_graph.tracer_real_time import TracerRealTime
from time import sleep



def get_sim_conf():
    import dynamic_graph.sot.torque_control.talos.balance_ctrl_sim_conf as balance_ctrl_conf
    import dynamic_graph.sot.torque_control.talos.base_estimator_sim_conf as base_estimator_conf
    import dynamic_graph.sot.torque_control.talos.control_manager_sim_conf as control_manager_conf
    #import dynamic_graph.sot.torque_control.talos.current_controller_sim_conf as current_controller_conf
    import dynamic_graph.sot.torque_control.talos.force_torque_estimator_conf as force_torque_estimator_conf
    import dynamic_graph.sot.torque_control.talos.joint_torque_controller_conf as joint_torque_controller_conf
    import dynamic_graph.sot.torque_control.talos.joint_pos_ctrl_gains_sim as pos_ctrl_gains
    import dynamic_graph.sot.torque_control.talos.motors_parameters as motor_params
    import dynamic_graph.sot.torque_control.talos.ddp_controller_conf as ddp_controller_conf
    
    conf = Bunch();
    conf.balance_ctrl              = balance_ctrl_conf;
    conf.base_estimator            = base_estimator_conf;
    conf.control_manager           = control_manager_conf;
    #conf.current_ctrl              = current_controller_conf;
    conf.force_torque_estimator    = force_torque_estimator_conf;
    conf.joint_torque_controller   = joint_torque_controller_conf;
    conf.pos_ctrl_gains            = pos_ctrl_gains;
    conf.motor_params              = motor_params;
    conf.ddp_controller            = ddp_controller_conf;
    return conf;

def test_balance_ctrl_talos_gazebo(robot, use_real_vel=True, use_real_base_state=False, startSoT=True):
    # BUILD THE STANDARD GRAPH
    conf = get_sim_conf();
    robot = main_v3(robot, startSoT=False, go_half_sitting=False, conf=conf);

    '''# force current measurements to zero
    robot.ctrl_manager.i_measured.value = NJ*(0.0,);
    #robot.current_ctrl.i_measured.value = NJ*(0.0,);
    robot.filters.current_filter.x.value = NJ*(0.0,);'''

    plug(robot.torque_ctrl.u,    robot.ctrl_manager.ctrl_torque);
    # BYPASS TORQUE CONTROLLER
    #plug(robot.inv_dyn.tau_des,     robot.ctrl_manager.ctrl_torque);

    # CREATE SIGNALS WITH ROBOT STATE WITH CORRECT SIZE (36)
    robot.q = Selec_of_vector("q");
    plug(robot.device.robotState, robot.q.sin);
    robot.q.selec(0, NJ+6);
    plug(robot.q.sout,              robot.pos_ctrl.base6d_encoders);
    plug(robot.q.sout,              robot.traj_gen.base6d_encoders);
    #plug(robot.q.sout,              robot.estimator_ft.base6d_encoders);

    robot.ros = RosPublish('rosPublish');
    robot.device.after.addDownsampledSignal('rosPublish.trigger',1);

    # BYPASS JOINT VELOCITY ESTIMATOR
    if(use_real_vel):
        robot.dq = Selec_of_vector("dq");
        #plug(robot.device.robotVelocity, robot.dq.sin); # to check robotVelocity empty
        plug(robot.device.velocity, robot.dq.sin);
        robot.dq.selec(6, NJ+6);
        # plug(robot.dq.sout,             robot.pos_ctrl.jointsVelocities); # generate seg fault
        plug(robot.dq.sout,             robot.base_estimator.joint_velocities);
        plug(robot.device.gyrometer,    robot.base_estimator.gyroscope);

    # BYPASS BASE ESTIMATOR
    # robot.v = Selec_of_vector("v");
    #plug(robot.device.robotVelocity, robot.dq.sin); # to check robotVelocity empty
    # plug(robot.device.velocity, robot.dq.sin);
    # robot.v.selec(0, NJ+6);
    if(use_real_base_state):
        plug(robot.q.sout,              robot.inv_dyn.q);
        plug(robot.dq.sout,             robot.inv_dyn.v);
    
    plug(robot.inv_dyn.tau_des,   robot.torque_ctrl.jointsTorquesDesired);
    robot.inv_dyn.active_joints.value=(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0);

    robot.pos = (
         # Free flyer
         0., 0., 1.018213, 0., 0. , 0.,
         # legs
         0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,
         0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,
         # Chest
         0.0 ,  0.006761,
         # arms
         0.25847 ,  0.173046, -0.0002, -0.525366, 0.0, -0.0,  0.1, -0.005,
         -0.4 , -0.3, 0.30 , -1.57, 0.0,  0.0,  0.2, -0.005,
         # Head
         0.0,0.0);
    start_tracer_perso(robot);

    if(startSoT):
        start_sot();
        # RESET FORCE/TORQUE SENSOR OFFSET
        # sleep(10*robot.timeStep);
        #robot.estimator_ft.setFTsensorOffsets(24*(0.0,));

    return robot;

def test_ddp_actuator(robot, startSoT=True):
    from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_ddp_controller 
    # BUILD THE STANDARD GRAPH
    conf = get_sim_conf();
    robot = main_v4(robot, startSoT=False, go_half_sitting=False, conf=conf);
    
    #plug(robot.sig_mix.sout,     robot.ctrl_manager.ctrl_torque);
    plug(robot.ddp_ctrl.tau,     robot.ctrl_manager.ctrl_torque);
    #plug(robot.torque_ctrl.u,    robot.ctrl_manager.ctrl_torque);
    robot.inv_dyn.active_joints.value=(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0);    
    robot.pos = (
         # Free flyer
         0., 0., 1.018213, 0., 0. , 0.,
         # legs
         0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,
         0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,
         # Chest
         0.0 ,  0.006761,
         # arms
         0.25847 ,  0.173046, -0.0002, -0.525366, 0.0, -0.0,  0.1, -0.005,
         -0.4 , -0.3, 0.30 , -1.57, 0.0,  0.0,  0.2, -0.005,
         # Head
         0.0,0.0);
    #start_tracer_perso(robot);

    if(startSoT):
        start_sot();
        # RESET FORCE/TORQUE SENSOR OFFSET
        # sleep(10*robot.timeStep);
        #robot.estimator_ft.setFTsensorOffsets(24*(0.0,));

    return robot;

def start_tracer_perso(robot):
    robot.tracer = TracerRealTime('robot_trace');
    robot.tracer.setBufferSize(80*(2**20));
    robot.tracer.open('~/tmp/tracer/','dg_','.dat');
    robot.device.after.addSignal('{0}.triger'.format(robot.tracer.name));

    #robot.tracer.add('PYRENE.control','PYRENE-control');
    #robot.tracer.add('invDynBalCtrl.tau_des','invDynBalCtrl-tau_des');
    #robot.tracer.add('pos_ctrl.qRef','pos_ctrl-qRef');
    #robot.tracer.add('PYRENE.robotState','PYRENE-robotState');
    #robot.tracer.add('mix.sout','sig_mix-sout');
    #robot.tracer.add('selecDdpTorqueDes.sout','selecDdpTorqueDes-sout');
    robot.tracer.start();

    return robot;

def stop_tracer_perso(robot):
    robot.tracer.stop();
    robot.tracer.dump();
    robot.tracer.close();
    return robot;
