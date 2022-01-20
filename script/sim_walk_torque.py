#!/usr/bin/python
import time, subprocess, os
from sys import argv
from run_test_utils import runCommandClient, run_test
from distutils.util import strtobool
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
elif len(argv) == 2 and argv[1] == "platforms":
    print("Starting script with folder " + folder + " walking on the platforms.")
    walk_type = "platforms"
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

c = input("Adding Energy Task? [y/N] ")
try:
  cb = strtobool(c)
  print("Add Energy Task !")
except Exception:
  cb = False
if cb:
  runCommandClient("robot.inv_dyn.addTaskEnergy(0.0)")

if pattern_generator:
  input("Waiting before setting gains")
  print("Setting gains")
  runCommandClient("robot.inv_dyn.kp_feet.value = np.array(6*(1200,))")
  runCommandClient("robot.inv_dyn.kd_feet.value = np.array(6*(30,))")
  if walk_type != "walk_60":
    runCommandClient("robot.inv_dyn.kp_com.value = np.array((600, 600, 600))")
    runCommandClient("robot.inv_dyn.kd_com.value = np.array((5, 5, 5))")
    input("Waiting before playing trajectories")
    runCommandClient('robot.triggerPG.sin.value = 1')
  else:
    runCommandClient("robot.inv_dyn.kp_com.value = np.array((100, 100, 40))")
    runCommandClient("robot.inv_dyn.kd_com.value = np.array(3*(3,))")
    input("Waiting before playing trajectories")
    runCommandClient('robot.triggerPG.sin.value = 1')  
    time.sleep(3.0)
    runCommandClient("robot.inv_dyn.kp_com.value = np.array(3*(12,))")
else:
  input("Waiting before setting gains")
  print("Setting gains")
  if walk_type == "platforms":
    runCommandClient("robot.inv_dyn.kp_feet.value = np.array(6*(1200,))")
    runCommandClient("robot.inv_dyn.kd_feet.value = np.array(6*(12,))")
    runCommandClient("robot.inv_dyn.kp_com.value = np.array((250, 250, 600))")
    runCommandClient("robot.inv_dyn.kd_com.value = np.array((2, 2, 5))")
    runCommandClient("robot.inv_dyn.kp_constraints.value = np.array(6*(100,))")
    runCommandClient("robot.inv_dyn.kd_constraints.value = np.array(6*(200,))")
  else:
    runCommandClient("robot.inv_dyn.kp_com.value = np.array((50, 50, 50))")
    runCommandClient("robot.inv_dyn.kd_com.value = np.array((5, 5, 5))")
  if walk_type == "isa":
    input("Waiting before going to isa pose")
    print("Go to isa pose")   
    runCommandClient('init_value_com = np.loadtxt(folder + walk_type + "/com.dat", usecols=(0,1,2))[0]')
    runCommandClient("robot.com_traj_gen.move(0,init_value_com[0],5.0)")
    runCommandClient("robot.com_traj_gen.move(1,init_value_com[1],5.0)")
    runCommandClient("robot.com_traj_gen.move(2,init_value_com[2],5.0)")
    time.sleep(6.5)
    runCommandClient("robot.phases_traj_gen.set(0,1.0) ")

  input("Waiting before setting trajectories")
  runCommandClient('robot.com_traj_gen.playTrajectoryFile(folder + walk_type + "/com.dat")')    
  runCommandClient('robot.phases_traj_gen.playTrajectoryFile(folder + walk_type + "/phases.dat")')
  runCommandClient('robot.rf_traj_gen.playTrajectoryFile(folder + walk_type + "/rightFoot.dat")')
  runCommandClient('robot.lf_traj_gen.playTrajectoryFile(folder + walk_type + "/leftFoot.dat")')
  if (walk_type != "isa") and (walk_type != "platforms"):
    runCommandClient('robot.am_traj_gen.playTrajectoryFile(folder + walk_type + "/am.dat")')
  if walk_type == "isa":
    runCommandClient("robot.inv_dyn.kp_com.value = np.array((800, 950, 600))")
    runCommandClient("robot.inv_dyn.kd_com.value = np.array((3, 2, 4))")
    runCommandClient("robot.inv_dyn.kp_am.value = np.array(3*(2.5,))")

  input("Waiting before playing trajectories")
  runCommandClient("robot.inv_dyn.setEnergyTank(5.0)")
  print("Playing trajectories")
  runCommandClient("robot.traj_sync.turnOn()")
  input("Waiting before stopping the trajectories")
  print("Stop trajectories")
  runCommandClient("robot.traj_sync.turnOff()")

