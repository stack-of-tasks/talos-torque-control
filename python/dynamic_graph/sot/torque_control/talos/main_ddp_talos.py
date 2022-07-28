from dynamic_graph import plug
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import (
    create_rospublish,
    create_topic,
)
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import (
    get_default_conf,
    get_sim_conf,
)
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import (
    create_ctrl_manager,
    create_trajectory_generator,
    create_encoders,
    create_encoders_velocity,
)
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import (
    create_joint_pos_selector,
    create_joint_vel_selector,
    create_joint_torque_selector,
    create_pos_des_selector,
)
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import (
    create_position_controller,
    create_pyrene_ddp_controller,
)

# --- EXPERIMENTAL SET UP ------------------------------------------------------

robot = locals()["robot"]
dt = robot.timeStep
robot.device.setControlInputType("noInteg")
conf_default = locals()["conf_default"]
conf = get_default_conf() if conf_default else get_sim_conf()


# --- SET INITIAL CONFIGURATION ------------------------------------------------
# TMP: overwrite halfSitting configuration to use SoT joint order
q = [0.0, 0.0, 1.018213, 0.0, 0.0, 0.0]  # Free flyer
q += [0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708]  # legs
q += [0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708]  # legs
q += [0.0, 0.006761]  # Chest
q += [0.25847, 0.173046, -0.0002, -0.525366, 0.0, -0.0, 0.1, -0.005]  # arms
q += [-0.25847, -0.173046, 0.0002, -0.525366, 0.0, 0.0, 0.1, -0.005]  # arms
q += [0.0, 0.0]  # Head

robot.halfSitting = q

# --- CREATE ENTITIES ----------------------------------------------------------
# --- Control manager (checks the limits in position, velocity)
robot.ctrl_manager = create_ctrl_manager(conf.control_manager, conf.motor_params, dt)

# --- Posture trajectory
robot.traj_gen = create_trajectory_generator(robot, dt)

# --- Signals for encoders and selectors of the joint on which the ddp is applied
robot.encoders = create_encoders(robot)
robot.encoders_velocity = create_encoders_velocity(robot)
robot.joint_pos_selec_ddp = create_joint_pos_selector(robot, conf.ddp_controller)
robot.joint_vel_selec_ddp = create_joint_vel_selector(robot, conf.ddp_controller)
robot.joint_torque_selec_ddp = create_joint_torque_selector(robot, conf.ddp_controller)
robot.pos_des_selec_ddp = create_pos_des_selector(robot, conf.ddp_controller)

# --- Default torque control with high gains -> behave like position controller
robot.pos_ctrl = create_position_controller(robot, conf.pos_ctrl_gains, dt)
# --- DDP controller
robot.ddp_ctrl = create_pyrene_ddp_controller(robot, conf.ddp_controller, dt)

# connect to device
plug(robot.device.currents, robot.ctrl_manager.i_measured)
plug(robot.device.ptorque, robot.ctrl_manager.tau)
# Position controller
robot.ctrl_manager.addCtrlMode("pos")
plug(robot.pos_ctrl.pwmDes, robot.ctrl_manager.ctrl_pos)
# Ddp controller
robot.ctrl_manager.addCtrlMode("torque")
plug(robot.ddp_ctrl.tau, robot.ctrl_manager.ctrl_torque)
robot.ctrl_manager.setCtrlMode("all", "pos")
plug(robot.ctrl_manager.u_safe, robot.device.control)

# Set the joint where the ddp is applied (right elbow) to torque control
robot.ctrl_manager.setCtrlMode("re", "torque")

# # # --- ROS PUBLISHER ----------------------------------------------------------

robot.publisher = create_rospublish(robot, "robot_publisher")
create_topic(
    robot.publisher,
    robot.joint_pos_selec_ddp,
    "sout",
    "joint_pos",
    robot=robot,
    data_type="vector",
)
create_topic(
    robot.publisher, robot.traj_gen, "q", "q_des", robot=robot, data_type="vector"
)
create_topic(
    robot.publisher,
    robot.ddp_ctrl,
    "tau",
    "ddp_ctrl_tau",
    robot=robot,
    data_type="vector",
)
