#!/usr/bin/python
import time, subprocess, os
from sys import argv
from run_test_utils import runCommandClient, run_test

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

folder = str(os.path.dirname(os.path.abspath(__file__)))
folder = folder + "/../traj_multicontact_api/dat/"
walk_type = "on_spot"
pattern_generator = False

if len(argv) == 2 and argv[1] == "on_spot":
    print("Starting script with folder " + folder + " walking on spot.")
elif len(argv) == 2 and argv[1] == "walk_20":
    print("Starting script with folder " + folder + " walking with 20cm step.")
    walk_type = "walk_20"
elif len(argv) == 2 and argv[1] == "plateforms":
    print("Starting script with folder " + folder + " walking on the plateforms.")
    walk_type = "plateforms"
elif len(argv) == 2 and argv[1] == "stairs":
    print("Starting script with folder " + folder + " climbing stairs.")
    walk_type = "stairs"
elif len(argv) == 3 and argv[1] == "on_spot" and argv[2] == "pattern_generator":
    pattern_generator = True
    print("Starting script with pattern_generator walking on spot.")
elif len(argv) == 3 and argv[1] == "walk_20" and argv[2] == "pattern_generator":
    pattern_generator = True
    walk_type = "walk_20"
    print("Starting script with pattern_generator walking with 20cm step.")
elif len(argv) == 3 and argv[1] == "walk_60" and argv[2] == "pattern_generator":
    pattern_generator = True
    walk_type = "walk_60"
    print("Starting script with pattern_generator walking with 60cm step.")
elif len(argv) == 3 and argv[1] == "on_spot":
    folder = argv[2]
    print("Starting script with folder " + folder + " walking on spot.")
elif len(argv) == 3 and argv[1] == "walk_20":
    folder = argv[2]
    print("Starting script with folder " + folder + " walking with 20cm step.")
    walk_type = "walk_20"
else: 
    print("Usage: python sim_walk_torque.py walk_type:=[on_spot|walk_20|walk_60] {pattern_generator|path_folder_of_the_reference_trajectories}")
    print("By giving only the walk_type the script starts using the default file trajectories")
    raise ValueError("Bad options")

if not pattern_generator:
    runCommandClient('folder = "' + folder + '"')
    runCommandClient('walk_type = "' + walk_type + '"')
    print("Starting script whith inverse_dyn_balance_controller main_sim_walk_torque.py")
    run_test('../python/dynamic_graph/sot/torque_control/talos/main_sim_walk_torque.py')
else:
    runCommandClient('walk_type = "' + walk_type + '"')
    print("Starting script whith inverse_dyn_balance_controller main_sim_walk_torque_online.py")
    run_test('../python/dynamic_graph/sot/torque_control/talos/main_sim_walk_torque_online.py')

input("Waiting before writing the graph")
runCommandClient("from dynamic_graph import writeGraph")

print("WriteGraph in /tmp/sot_talos_tsid.dot")
runCommandClient("writeGraph('/tmp/sot_talos_tsid_walk.dot')")
print("Convert graph to PDF in /tmp/sot_talos_tsid_walk.pdf")
proc3 = subprocess.Popen(["dot", "-Tpdf", "/tmp/sot_talos_tsid_walk.dot", "-o", "/tmp/sot_talos_tsid_walk_torque.pdf"])

if pattern_generator:
    input("Waiting before going to halfSitting")
    runCommandClient("go_to_position(robot.traj_gen, robot.halfSitting[6:], 5.0)")
    input("Waiting before setting gains")
    print("Setting gains")
    runCommandClient("robot.inv_dyn.kp_feet.value = 6*(1200,)")
    runCommandClient("robot.inv_dyn.kp_com.value = (100, 100, 10)")
    runCommandClient("robot.inv_dyn.kd_feet.value = 6*(30,)")
    runCommandClient("robot.inv_dyn.kd_com.value = 3*(3,)")
    input("Waiting before playing trajectories")
    runCommandClient('robot.triggerPG.sin.value = 1')
    time.sleep(3.0)
    runCommandClient("robot.inv_dyn.kp_com.value = 3*(20,)")
else:
    input("Waiting before setting gains")
    print("Setting gains")
    runCommandClient("robot.inv_dyn.kp_feet.value = 12*(1200,)")
    runCommandClient("robot.inv_dyn.kp_com.value = 3*(15,)")
    runCommandClient("robot.inv_dyn.kd_feet.value = 12*(30,)")
    runCommandClient("robot.inv_dyn.kd_com.value = (1, 1, 4)")
    input("Waiting before setting trajectories")
    runCommandClient('robot.com_traj_gen.playTrajectoryFile(folder + walk_type + "/com.dat")')
    runCommandClient('robot.am_traj_gen.playTrajectoryFile(folder + walk_type + "/am.dat")')
    runCommandClient('robot.phases_traj_gen.playTrajectoryFile(folder + walk_type + "/phases.dat")')
    runCommandClient('robot.rf_traj_gen.playTrajectoryFile(folder + walk_type + "/rightFoot.dat")')
    runCommandClient('robot.lf_traj_gen.playTrajectoryFile(folder + walk_type + "/leftFoot.dat")')
    input("Waiting before playing trajectories")
    print("Playing trajectories")
    runCommandClient("robot.traj_sync.turnOn()")
    input("Waiting before stopping the trajectories")
    print("Stop trajectories")
    runCommandClient("robot.traj_sync.turnOff()")

time.sleep(2.0)
input("Wait before dumping the data")
runCommandClient('dump_tracer(robot.tracer)')
