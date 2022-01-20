#!/usr/bin/python
import time, subprocess, os
from sys import argv
from run_test_utils import runCommandClient, run_test
from dynamic_graph.sot_talos_balance.utils.run_test_utils import run_ft_wrist_calibration

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
print("Starting script whith inverse_dyn_balance_controller main_sim_wrist_force.py")
run_test('../python/dynamic_graph/sot/torque_control/talos/main_sim_wrist_force.py')


input("Waiting before writing the graph")
runCommandClient("from dynamic_graph import writeGraph")

print("WriteGraph in /tmp/sot_talos_tsid.dot")
runCommandClient("writeGraph('/tmp/sot_talos_tsid_walk.dot')")
print("Convert graph to PDF in /tmp/sot_talos_tsid_walk.pdf")
proc3 = subprocess.Popen(["dot", "-Tpdf", "/tmp/sot_talos_tsid_walk.dot", "-o", "/tmp/sot_talos_tsid_walk_torque.pdf"])

input("Wait before going to contact pose")

# Contact downward
runCommandClient("robot.traj_gen.moveJoint('le',-0.8, 10.0)")
runCommandClient("robot.traj_gen.moveJoint('lsr',0.8, 10.0)")
runCommandClient("robot.traj_gen.moveJoint('lsy',-0.8, 10.0)")
runCommandClient("robot.traj_gen.moveJoint('lay', -1.6, 10.0)")
runCommandClient("robot.traj_gen.moveJoint('lwp',-0.0, 10.0)")
runCommandClient("robot.traj_gen.moveJoint('lwr',0.0, 10.0)")

run_ft_wrist_calibration('robot.forceCalibrator')
input("Wait before adding EnergyTask")
runCommandClient("robot.inv_dyn.addTaskEnergy(0.0)")
input("Wait before adding TaskLeftHandContact")
runCommandClient("robot.inv_dyn.addTaskLeftHandContact()")

input("Wait before setting 10N force")
runCommandClient("robot.force_hand_traj_gen.move(2, 10, 3)")
time.sleep(20)
runCommandClient("robot.force_hand_traj_gen.move(2, 30, 5)")
time.sleep(40)
print("REMOVE BLOCK ! ")
time.sleep(2)
runCommandClient("robot.inv_dyn.removeTaskLeftHandContact(0.0)")

 