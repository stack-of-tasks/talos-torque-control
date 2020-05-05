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
runCommandClient('folder = "' + folder + '"')

print("Starting script whith inverse_dyn_balance_controller")
run_test('../python/dynamic_graph/sot/torque_control/talos/main_sim_inverse_dyn_balance_controller.py')

input("Waiting before writing the graph")
runCommandClient("from dynamic_graph import writeGraph")

print("WriteGraph in /tmp/sot_talos_tsid.dot")
runCommandClient("writeGraph('/tmp/sot_talos_tsid_walk.dot')")
print("Convert graph to PDF in /tmp/sot_talos_tsid_walk.pdf")
proc3 = subprocess.Popen(["dot", "-Tpdf", "/tmp/sot_talos_tsid_walk.dot", "-o", "/tmp/sot_talos_tsid_walk.pdf"])

# input("Wait before going to halfSitting")
# runCommandClient("go_to_position(robot.traj_gen, robot.halfSitting[6:], 5.0)")

# input("Waiting before reading trajectories")
# runCommandClient('robot.com_traj_gen.playTrajectoryFile(folder + "/../traj_multicontact_api/dat/on_spot/com.dat")')
# runCommandClient('robot.am_traj_gen.playTrajectoryFile(folder + "/../traj_multicontact_api/dat/on_spot/am.dat")')
# runCommandClient('robot.rf_force_traj_gen.playTrajectoryFile(folder + "/../traj_multicontact_api/dat/on_spot/rightForceFoot.dat")')
# runCommandClient('robot.lf_force_traj_gen.playTrajectoryFile(folder + "/../traj_multicontact_api/dat/on_spot/leftForceFoot.dat")')
# runCommandClient('robot.rf_traj_gen.playTrajectoryFile(folder + "/../traj_multicontact_api/dat/on_spot/rightFoot.dat")')
# runCommandClient('robot.lf_traj_gen.playTrajectoryFile(folder + "/../traj_multicontact_api/dat/on_spot/leftFoot.dat")')

input("Waiting before playing trajectories")
print("Playing trajectories")
runCommandClient("robot.traj_sync.turnOn()")

# input("Waiting before stopping the trajectories")
# print("Stop trajectories")
# runCommandClient("robot.traj_sync.turnOff()")
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

# time.sleep(2.0)
# print("Putting the robot back...")
# runCommandClient("go_to_position(robot.traj_gen, robot.halfSitting[6:], 5.0)")
# time.sleep(5.0)
# print("The robot is back in position!")

# runCommandClient('dump_tracer(robot.tracer)')

