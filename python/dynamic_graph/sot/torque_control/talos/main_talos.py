# -*- coding: utf-8 -*-1
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

from dynamic_graph import plug
from dynamic_graph.sot.torque_control.se3_trajectory_generator import SE3TrajectoryGenerator
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_trajectory_switch, connect_synchronous_trajectories, create_force_traj_gen
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_trajectory_generator, create_com_traj_gen, create_encoders, create_encoders_velocity
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_joint_pos_selector, create_joint_vel_selector, create_joint_torque_selector, create_pos_des_selector, create_motor_pos_selector, create_torque_des_selector2, create_torque_des_selector, create_tau_des_selector
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_imu_offset_compensation, create_filters, create_imu_filter
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_base_estimator, create_position_controller, create_torque_controller
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_balance_controller, create_ctrl_manager, create_ros_topics
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_free_flyer_locator#, create_flex_estimator, create_floatingBase
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_current_controller, connect_ctrl_manager, create_ddp_controller
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_tracer, create_topic, create_admittance_ctrl
from dynamic_graph.ros import RosPublish
from dynamic_graph.sot.torque_control.talos.sot_utils_talos import start_sot, stop_sot, go_to_position, Bunch, start_movement_sinusoid, stop_movement_sinusoid
from dynamic_graph.sot.torque_control.utils.filter_utils import create_chebi2_lp_filter_Wn_03_N_4
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import create_signal_mixer

from time import sleep

def get_default_conf():
    import dynamic_graph.sot.torque_control.talos.balance_ctrl_conf as balance_ctrl_conf
    import dynamic_graph.sot.torque_control.talos.admittance_ctrl_conf as admittance_ctrl_conf
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
    conf.force_torque_estimator    = force_torque_estimator_conf;
    conf.joint_torque_controller   = joint_torque_controller_conf;
    conf.pos_ctrl_gains            = pos_ctrl_gains;
    conf.motor_params              = motor_params;
    conf.ddp_controller            = ddp_controller_conf;
    return conf;

''' Main function to call before starting the graph. '''
def main_v3(robot, startSoT=True, go_half_sitting=True, conf=None):
    if(conf is None):
        conf = get_default_conf();
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

    #robot.device.setControlInputType('noInteg');
    robot.ctrl_manager    = create_ctrl_manager(conf.control_manager, conf.motor_params, dt);

    robot.traj_gen        = create_trajectory_generator(robot.device, dt);
    robot.com_traj_gen    = create_com_traj_gen(conf.balance_ctrl, dt);
    robot.rf_force_traj_gen  = create_force_traj_gen("rf_force_ref", conf.balance_ctrl.RF_FORCE_DES, dt);
    robot.lf_force_traj_gen  = create_force_traj_gen("lf_force_ref", conf.balance_ctrl.LF_FORCE_DES, dt);

    robot.traj_sync       = create_trajectory_switch();
    robot.rf_traj_gen     = SE3TrajectoryGenerator("tg_rf");
    robot.lf_traj_gen     = SE3TrajectoryGenerator("tg_lf");
    robot.rh_traj_gen     = SE3TrajectoryGenerator("tg_rh");
    robot.lh_traj_gen     = SE3TrajectoryGenerator("tg_lh");
    robot.rf_traj_gen.init(dt);
    robot.lf_traj_gen.init(dt);
    robot.rh_traj_gen.init(dt);
    robot.lh_traj_gen.init(dt);
    
    robot.encoders                              = create_encoders(robot);
    robot.encoders_velocity                     = create_encoders_velocity(robot);
    robot.imu_offset_compensation               = create_imu_offset_compensation(robot, dt);
    #(robot.estimator_ft, robot.filters)         = create_estimators(robot, conf.force_torque_estimator, conf.motor_params, dt);
    robot.filters                               = create_filters(robot, conf.force_torque_estimator, conf.motor_params, dt);
    robot.imu_filter                            = create_imu_filter(robot, dt);
    robot.base_estimator                        = create_base_estimator(robot, dt, conf.base_estimator);

    connect_synchronous_trajectories(robot.traj_sync,
                                     [robot.com_traj_gen,
                                      robot.rf_force_traj_gen, robot.lf_force_traj_gen,
                                      robot.rf_traj_gen, robot.lf_traj_gen,
                                      robot.rh_traj_gen, robot.lh_traj_gen])
    #robot.rf_traj_gen, robot.lf_traj_gen])

    robot.pos_ctrl        = create_position_controller(robot, conf.pos_ctrl_gains, dt);
    robot.torque_ctrl     = create_torque_controller(robot, conf.joint_torque_controller, conf.motor_params, dt);
    robot.inv_dyn         = create_balance_controller(robot, conf.balance_ctrl,conf.motor_params, dt);
    #robot.current_ctrl    = create_current_controller(robot, conf.current_ctrl, conf.motor_params, dt);
    connect_ctrl_manager(robot);

    # create low-pass filter for computing joint velocities
    robot.encoder_filter = create_chebi2_lp_filter_Wn_03_N_4('encoder_filter', dt, conf.motor_params.NJ)
    plug(robot.encoders.sout,             robot.encoder_filter.x)
    #plug(robot.encoder_filter.dx,         robot.current_ctrl.dq);
    plug(robot.encoder_filter.dx,         robot.torque_ctrl.jointsVelocities);
    #plug(robot.encoder_filter.x_filtered, robot.base_estimator.joint_positions);
    #plug(robot.encoder_filter.dx,         robot.base_estimator.joint_velocities);

    robot.ros = RosPublish('rosPublish');
    robot.device.after.addDownsampledSignal('rosPublish.trigger',1);

    #robot.estimator_ft.dgyro.value = (0.0, 0.0, 0.0);
    #robot.estimator_ft.gyro.value = (0.0, 0.0, 0.0);
#    estimator.accelerometer.value = (0.0, 0.0, 9.81);
    if(startSoT):
        print "Gonna start SoT";
        sleep(1.0);
        start_sot();

        if(go_half_sitting):
            print "Gonna go to half sitting in 1 sec";
            sleep(1.0);
            go_to_position(robot.traj_gen, robot.halfSitting[6:], 10.0);

    return robot;

def main_v4(robot, startSoT=True, go_half_sitting=True, conf=None):
    if(conf is None):
        conf = get_default_conf();
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

    #robot.device.setControlInputType('noInteg');
    robot.ctrl_manager    = create_ctrl_manager(conf.control_manager, conf.motor_params, dt);

    robot.traj_gen        = create_trajectory_generator(robot.device, dt);
    robot.com_traj_gen    = create_com_traj_gen(conf.balance_ctrl, dt);
    robot.rf_force_traj_gen  = create_force_traj_gen("rf_force_ref", conf.balance_ctrl.RF_FORCE_DES, dt);
    robot.lf_force_traj_gen  = create_force_traj_gen("lf_force_ref", conf.balance_ctrl.LF_FORCE_DES, dt);

    robot.traj_sync       = create_trajectory_switch();
    robot.rf_traj_gen     = SE3TrajectoryGenerator("tg_rf");
    robot.lf_traj_gen     = SE3TrajectoryGenerator("tg_lf");
    robot.rh_traj_gen     = SE3TrajectoryGenerator("tg_rh");
    robot.lh_traj_gen     = SE3TrajectoryGenerator("tg_lh");
    robot.rf_traj_gen.init(dt);
    robot.lf_traj_gen.init(dt);
    robot.rh_traj_gen.init(dt);
    robot.lh_traj_gen.init(dt);
    
    robot.encoders                              = create_encoders(robot);
    robot.encoders_velocity                     = create_encoders_velocity(robot);
    robot.joint_pos_selec_ddp                   = create_joint_pos_selector(robot, conf.ddp_controller);
    robot.joint_vel_selec_ddp                   = create_joint_vel_selector(robot, conf.ddp_controller);
    robot.joint_torque_selec_ddp                = create_joint_torque_selector(robot, conf.ddp_controller);
    robot.pos_des_selec_ddp                     = create_pos_des_selector(robot, conf.ddp_controller);
    robot.motor_pos_selec_ddp                   = create_motor_pos_selector(robot, conf.ddp_controller);
        
    robot.imu_offset_compensation               = create_imu_offset_compensation(robot, dt);
    #(robot.estimator_ft, robot.filters)         = create_estimators(robot, conf.force_torque_estimator, conf.motor_params, dt);
    robot.filters                               = create_filters(robot, conf.force_torque_estimator, conf.motor_params, dt);
    robot.imu_filter                            = create_imu_filter(robot, dt);
    robot.base_estimator                        = create_base_estimator(robot, dt, conf.base_estimator);

    connect_synchronous_trajectories(robot.traj_sync,
                                     [robot.com_traj_gen,
                                      robot.rf_force_traj_gen, robot.lf_force_traj_gen,
                                      robot.rf_traj_gen, robot.lf_traj_gen,
                                      robot.rh_traj_gen, robot.lh_traj_gen]);
    #robot.rf_traj_gen, robot.lf_traj_gen])

    robot.pos_ctrl          = create_position_controller(robot, conf.pos_ctrl_gains, dt);
    robot.torque_ctrl       = create_torque_controller(robot, conf.joint_torque_controller, conf.motor_params, dt);
    robot.inv_dyn           = create_balance_controller(robot, conf.balance_ctrl,conf.motor_params, dt);
    robot.tau_des_selec_ddp = create_tau_des_selector(robot, conf.ddp_controller);
    robot.ddp_ctrl          = create_ddp_controller(robot, conf.ddp_controller, dt);
    
    #robot.torque_des_selec_ddp                  = create_torque_des_selector(robot, conf.ddp_controller);
    #robot.torque_des_selec_ddp2                 = create_torque_des_selector2(robot, conf.ddp_controller);
    #robot.sig_mix         = create_signal_mixer(robot, conf.ddp_controller);
    #robot.current_ctrl    = create_current_controller(robot, conf.current_ctrl, conf.motor_params, dt);
    connect_ctrl_manager(robot);

    # create low-pass filter for computing joint velocities
    robot.encoder_filter = create_chebi2_lp_filter_Wn_03_N_4('encoder_filter', dt, conf.motor_params.NJ);
    plug(robot.encoders.sout,             robot.encoder_filter.x);
    #plug(robot.encoder_filter.dx,         robot.current_ctrl.dq);
    plug(robot.encoder_filter.dx,         robot.torque_ctrl.jointsVelocities);
    #plug(robot.encoder_filter.x_filtered, robot.base_estimator.joint_positions);
    #plug(robot.encoder_filter.dx,         robot.base_estimator.joint_velocities);

    robot.ros = RosPublish('rosPublish');
    robot.device.after.addDownsampledSignal('rosPublish.trigger',1);

    #robot.estimator_ft.dgyro.value = (0.0, 0.0, 0.0);
    #robot.estimator_ft.gyro.value = (0.0, 0.0, 0.0);
#    estimator.accelerometer.value = (0.0, 0.0, 9.81);
    if(startSoT):
        print "Gonna start SoT";
        sleep(1.0);
        start_sot();

        if(go_half_sitting):
            print "Gonna go to half sitting in 1 sec";
            sleep(1.0);
            go_to_position(robot.traj_gen, robot.halfSitting[6:], 10.0);

    return robot;

''' Main function to call after having started the graph. '''
def main_post_start(robot):
    ros = create_ros_topics(robot);
    return ros;
