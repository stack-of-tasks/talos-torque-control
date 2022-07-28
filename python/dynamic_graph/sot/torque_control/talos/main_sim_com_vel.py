from dynamic_graph import plug
from dynamic_graph.sot.core import Selec_of_vector
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import (
    NJ,
    create_rospublish,
    create_topic,
    get_sim_conf,
)
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import (
    create_waist_traj_gen,
    create_trajectory_generator,
    create_com_traj_gen,
    create_encoders,
)
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import (
    create_simple_inverse_dyn_controller,
    create_ctrl_manager,
)
from dynamic_graph.sot.torque_control.talos.create_entities_utils_talos import (
    addTrace,
    dump_tracer,
)
from dynamic_graph.sot.torque_control.talos.sot_utils_talos import go_to_position
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.sot.core import Substract_of_vector

# --- EXPERIMENTAL SET UP ------------------------------------------------------
conf = get_sim_conf()
dt = robot.timeStep

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

robot.ctrl_manager = create_ctrl_manager(conf.control_manager, conf.motor_params, dt)
robot.encoders = create_encoders(robot)

# --- Posture trajectory
robot.traj_gen = create_trajectory_generator(robot, dt)
robot.traj_gen.q.recompute(0)
# --- CoM trajectory
robot.com_traj_gen = create_com_traj_gen(robot, dt)
robot.com_traj_gen.x.recompute(0)
# --- Base orientation (SE3 on the waist) trajectory
robot.waist_traj_gen = create_waist_traj_gen("tg_waist_ref", robot, dt)
robot.waist_traj_gen.x.recompute(0)

# --- Simple inverse dynamic controller
robot.inv_dyn = create_simple_inverse_dyn_controller(robot, conf.balance_ctrl, dt)
robot.inv_dyn.setControlOutputType("velocity")

# --- Connect control manager
plug(robot.device.currents, robot.ctrl_manager.i_measured)
plug(robot.device.ptorque, robot.ctrl_manager.tau)
robot.ctrl_manager.addCtrlMode("vel")
plug(robot.inv_dyn.v_des, robot.device.control)
robot.ctrl_manager.setCtrlMode("all", "vel")
plug(robot.ctrl_manager.joints_ctrl_mode_vel, robot.inv_dyn.active_joints)

# --- Error on the CoM task
robot.errorComTSID = Substract_of_vector("error_com")
plug(robot.inv_dyn.com_ref_pos, robot.errorComTSID.sin2)
plug(robot.dynamic.com, robot.errorComTSID.sin1)

# --- Error on the Posture task
robot.errorPoseTSID = Substract_of_vector("error_pose")
plug(robot.inv_dyn.posture_ref_pos, robot.errorPoseTSID.sin2)
plug(robot.encoders.sout, robot.errorPoseTSID.sin1)


# # # --- ROS PUBLISHER ----------------------------------------------------------

robot.publisher = create_rospublish(robot, "robot_publisher")
create_topic(
    robot.publisher,
    robot.errorPoseTSID,
    "sout",
    "errorPoseTSID",
    robot=robot,
    data_type="vector",
)
create_topic(
    robot.publisher,
    robot.errorComTSID,
    "sout",
    "errorComTSID",
    robot=robot,
    data_type="vector",
)
create_topic(
    robot.publisher, robot.dynamic, "com", "dynCom", robot=robot, data_type="vector"
)
create_topic(
    robot.publisher, robot.inv_dyn, "q_des", "q_des", robot=robot, data_type="vector"
)
create_topic(
    robot.publisher,
    robot.device,
    "motorcontrol",
    "motorcontrol",
    robot=robot,
    data_type="vector",
)
create_topic(
    robot.publisher,
    robot.device,
    "robotState",
    "robotState",
    robot=robot,
    data_type="vector",
)

# # --- TRACER
robot.tracer = TracerRealTime("tau_tracer")
robot.tracer.setBufferSize(80 * (2**20))
robot.tracer.open("/tmp", "dg_", ".dat")
robot.device.after.addSignal("{0}.triger".format(robot.tracer.name))

addTrace(robot.tracer, robot.inv_dyn, "tau_des")
addTrace(robot.tracer, robot.inv_dyn, "q_des")
addTrace(robot.tracer, robot.inv_dyn, "v_des")
addTrace(robot.tracer, robot.inv_dyn, "dv_des")
addTrace(robot.tracer, robot.errorPoseTSID, "sout")
addTrace(robot.tracer, robot.errorComTSID, "sout")
addTrace(robot.tracer, robot.device, "robotState")
addTrace(robot.tracer, robot.device, "motorcontrol")

robot.tracer.start()
