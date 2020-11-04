#!/usr/bin/python
import time, subprocess, os
from sys import argv
from run_test_utils import runCommandClient, run_test
from sot_talos_balance.utils.run_test_utils import run_ft_calibration

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
elif len(argv) == 3 and argv[1] == "on_spot":
    folder = argv[2]
    print("Starting script with folder " + folder + " walking on spot.")
elif len(argv) == 3 and argv[1] == "walk_20":
    folder = argv[2]
    print("Starting script with folder " + folder + " walking with 20cm step.")
    walk_type = "walk_20"
else: 
    print("Usage: python sim_walk_vel.py walk_type:=[on_spot|walk_20] {pattern_generator|path_folder_of_the_reference_trajectories}")
    print("By giving only the walk_type the script starts using the default file trajectories")
    raise ValueError("Bad options")

if not pattern_generator:
    runCommandClient('folder = "' + folder + '"')
    runCommandClient('walk_type = "' + walk_type + '"')
    print("Starting script whith inverse_dyn_balance_controller")
    run_test('../python/dynamic_graph/sot/torque_control/talos/main_sim_walk_vel.py')
else:
    runCommandClient('walk_type = "' + walk_type + '"')
    print("Starting script whith inverse_dyn_balance_controller")
    run_test('../python/dynamic_graph/sot/torque_control/talos/main_sim_walk_vel_online.py')

run_ft_calibration('robot.ftc')

input("Waiting before writing the graph")
runCommandClient("from dynamic_graph import writeGraph")

print("WriteGraph in /tmp/sot_talos_tsid.dot")
runCommandClient("writeGraph('/tmp/sot_talos_tsid_walk.dot')")
print("Convert graph to PDF in /tmp/sot_talos_tsid_walk.pdf")
proc3 = subprocess.Popen(["dot", "-Tpdf", "/tmp/sot_talos_tsid_walk.dot", "-o", "/tmp/sot_talos_tsid_walk.pdf"])

input("Waiting before setting gains and going to initial position")
runCommandClient("go_to_position(robot.traj_gen, robot.halfSitting[6:], 5.0)")
print("Setting gains")
runCommandClient("robot.inv_dyn.kp_feet.value = 6*(1400,)")
runCommandClient("robot.inv_dyn.kd_feet.value = 6*(20,)")
if walk_type == "walk_20" or walk_type == "on_spot":
    runCommandClient("robot.inv_dyn.kp_com.value = 3*(1000,)")
    runCommandClient("robot.inv_dyn.kd_com.value = 3*(500,)")
else:
    runCommandClient("robot.inv_dyn.kp_com.value = 3*(3000,)")
    runCommandClient("robot.inv_dyn.kd_com.value = 3*(1500,)")

# Connect ZMP reference and reset controllers
input("Waiting before connecting the ZMP reference")
print('Connect ZMP reference')
runCommandClient('plug(robot.zmp_estimator.emergencyStop,robot.ctrl_manager.emergencyStop_zmp)')
runCommandClient('plug(robot.dcm_control.zmpRef,robot.com_admittance_control.zmpDes)')
runCommandClient('robot.com_admittance_control.setState(robot.wp.comDes.value,[0.0,0.0,0.0])')
runCommandClient('robot.com_admittance_control.Kp.value = Kp_adm')
runCommandClient('robot.dcm_control.resetDcmIntegralError()')
runCommandClient('robot.dcm_control.Ki.value = Ki_dcm')
runCommandClient('robot.dcm_control.Kz.value = Kz_dcm')
time.sleep(2.0)

if pattern_generator:
    input("Waiting before playing trajectories")
    runCommandClient('robot.triggerPG.sin.value = 1')
else:
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
