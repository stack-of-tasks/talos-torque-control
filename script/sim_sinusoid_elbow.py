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

runCommandClient('folder = "' + folder + '"')
runCommandClient('walk_type = "' + walk_type + '"')

print("Starting script whith inverse_dyn_balance_controller main_sim_walk_torque.py")
run_test('../python/dynamic_graph/sot/torque_control/talos/main_sim_walk_torque.py')

input("Waiting before writing the graph")
runCommandClient("from dynamic_graph import writeGraph")

print("WriteGraph in /tmp/sot_talos_tsid.dot")
runCommandClient("writeGraph('/tmp/sot_talos_tsid_walk.dot')")
print("Convert graph to PDF in /tmp/sot_talos_tsid_walk.pdf")
proc3 = subprocess.Popen(["dot", "-Tpdf", "/tmp/sot_talos_tsid_sinusoid_elbow.dot", "-o", "/tmp/sot_talos_tsid_sinusoid_elbow.pdf"])

input("Wait before going to sinusoid pose")

runCommandClient("robot.traj_gen.moveJoint('le',-1.2,5.0)")

input("Waiting before starting sinusoid")
print("Start sinusoid")
runCommandClient("robot.traj_gen.startSinusoid('le',-1.5,2.0)")

input("Waiting before stopping sinusoid")
print("Stop Sinusoid")
runCommandClient("robot.traj_gen.stop('le')")
time.sleep(5.0)
print("Putting the robot back...")
runCommandClient("go_to_position(robot.traj_gen, robot.halfSitting[6:], 5.0)")

