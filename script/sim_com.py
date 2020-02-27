#!/usr/bin/python
import time, subprocess
from sys import argv
from run_test_utils import runCommandClient, run_test

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

if len(argv) == 2 and argv[1] == "torque":
    print("Starting script in torque control")
    run_test('../python/dynamic_graph/sot/torque_control/talos/main_sim_com_torque.py')
else:
    print("Starting script in position control")
    run_test('../python/dynamic_graph/sot/torque_control/talos/main_sim_com_vel.py')

input("Waiting before writing the graph")
runCommandClient("from dynamic_graph import writeGraph")

print("WriteGraph in /tmp/sot_talos_tsid.dot")
runCommandClient("writeGraph('/tmp/sot_talos_tsid_com.dot')")
print("Convert graph to PDF in /tmp/sot_talos_tsid_com.pdf")
proc3 = subprocess.Popen(["dot", "-Tpdf", "/tmp/sot_talos_tsid_com.dot", "-o", "/tmp/sot_talos_tsid_com.pdf"])

input("Wait before going to halfSitting")
runCommandClient("go_to_position(robot.traj_gen, robot.halfSitting[6:], 5.0)")

input("Waiting before going to sinusoid pose")
print("Go to sinusoid pose")
runCommandClient("robot.com_traj_gen.move(1,-0.025,1.0)")

input("Waiting before starting sinusoid")
print("Start sinusoid")
runCommandClient("robot.com_traj_gen.startSinusoid(1,0.025,2.0)")

input("Waiting before stopping sinusoid")
print("Stop Sinusoid")
runCommandClient("robot.com_traj_gen.stop(1)")
time.sleep(1.0)
print("Putting the robot back...")
runCommandClient('robot.com_traj_gen.move(1,0.0,1.0)')
time.sleep(1.0)
print("The robot is back in position!")

runCommandClient('dump_tracer(robot.tracer)')

