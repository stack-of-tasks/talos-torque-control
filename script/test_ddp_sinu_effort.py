#!/usr/bin/python
import subprocess
from sys import argv
from run_test_utils import runCommandClient, run_test

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

if len(argv) == 2 and argv[1] == "robot":
    print("Starting script for real experiment on the robot")
    runCommandClient("conf_default = True")
else:
    print("Starting script for simulation")
    runCommandClient("conf_default = False")

run_test("../python/dynamic_graph/sot/torque_control/talos/main_ddp_talos.py")

input("Waiting before writing the graph")
runCommandClient("from dynamic_graph import writeGraph")

print("WriteGraph in /tmp/sot_ddp_talos_effort.dot")
runCommandClient("writeGraph('/tmp/sot_ddp_talos_effort.dot')")
print("Convert graph to PDF in /tmp/sot_ddp_talos_effort.pdf")
proc3 = subprocess.Popen(
    [
        "dot",
        "-Tpdf",
        "/tmp/sot_ddp_talos_effort.dot",
        "-o",
        "/tmp/sot_ddp_talos_effort.pdf",
    ]
)

input("Waiting before going to sinusoid pose")
print("Go to sinusoid pose")
runCommandClient("go_to_position_sinusoid(robot)")

input("Waiting before starting sinusoid move")
print("Start Sinusoid move")
runCommandClient("robot.traj_gen.startSinusoid('re', -1.9, 1.5)")

input("Waiting before stopping sinusoid move")
print("Stop Sinusoid move")
runCommandClient("robot.traj_gen.stop('re')")

print("Go back to HalfSitting")
runCommandClient("go_to_position(robot.traj_gen, robot.halfSitting[6:], 10.0)")
