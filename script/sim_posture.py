#!/usr/bin/python
import time, subprocess
from sys import argv
from run_test_utils import runCommandClient, run_test

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass


print("Starting script in torque control")
run_test('../python/dynamic_graph/sot/torque_control/talos/main_sim_posture_torque.py')

input("Waiting before writing the graph")
runCommandClient("from dynamic_graph import writeGraph")

print("WriteGraph in /tmp/sot_talos_tsid.dot")
runCommandClient("writeGraph('/tmp/sot_talos_tsid_posture.dot')")
print("Convert graph to PDF in /tmp/sot_talos_tsid_posture.pdf")
proc3 = subprocess.Popen(["dot", "-Tpdf", "/tmp/sot_talos_tsid_posture.dot", "-o", "/tmp/sot_talos_tsid_posture.pdf"])

# input("Wait before going to halfSitting")
# runCommandClient("go_to_position(robot.traj_gen, robot.halfSitting[6:], 5.0)")

input("Waiting before moving elbow")
print("Go to sinusoid pose")
runCommandClient("robot.traj_gen.moveJoint('le', -1.2, 3)") 

input("Waiting before starting sinusoid move")
print("Start Sinusoid move")
runCommandClient("robot.traj_gen.startSinusoid('le', -1.6, 3)") 

input("Waiting before stopping sinusoid move")
print("Stop Sinusoid move")
runCommandClient("robot.traj_gen.stop('le')")

print("Go back to HalfSitting")
runCommandClient("go_to_position(robot.traj_gen, robot.halfSitting[6:], 5.0)")

#runCommandClient('dump_tracer(robot.tracer)')

