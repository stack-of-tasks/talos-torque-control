#!/usr/bin/python
import time, subprocess, os, rospy
from sys import argv
from integ_run_test_utils import runCommandClient, run_test
import rospkg

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

PKG_NAME='talos-torque-control'

rospack = rospkg.RosPack()
pkg_path = rospack.get_path(PKG_NAME)
folder= pkg_path+ "/traj_multicontact_api/dat/"
walk_type = "on_spot_test"
pattern_generator = False

if len(argv) == 4 and argv[1] == "on_spot":
    print("Starting script with folder " + folder + " walking on spot.")
elif len(argv) == 4 and argv[1] == "walk_20":
    print("Starting script with folder " + folder + " walking with 20cm step.")
    walk_type = "walk_20_test"
elif len(argv) == 5 and argv[1] == "on_spot" and argv[2] == "pattern_generator":
    pattern_generator = True
    print("Starting script with pattern_generator walking on spot.")
elif len(argv) == 5 and argv[1] == "walk_20" and argv[2] == "pattern_generator":
    pattern_generator = True
    walk_type = "walk_20_test"
    print("Starting script with pattern_generator walking with 20cm step.")
elif len(argv) == 5 and argv[1] == "on_spot":
    folder = argv[2]
    print("Starting script with folder " + folder + " walking on spot.")
elif len(argv) == 5 and argv[1] == "walk_20":
    folder = argv[2]
    print("Starting script with folder " + folder + " walking with 20cm step.")
    walk_type = "walk_20"
# Default option: on_spot_test, without pattern generator
elif len(argv) == 3:
    print("Starting script with folder " + folder + " walking on spot.")
else: 
    print("Usage: python sim_walk_vel.py walk_type:=[on_spot|walk_20] {pattern_generator|path_folder_of_the_reference_trajectories}")
    print("By giving only the walk_type the script starts using the default file trajectories")
    raise ValueError("Bad options")

if not pattern_generator:
    runCommandClient('folder = "' + folder + '"')
    runCommandClient('walk_type = "' + walk_type + '"')
    print("Starting script whith inverse_dyn_balance_controller")
    run_test(pkg_path+'/../../lib/'+PKG_NAME+'/main_sim_walk_vel.py')
else:
    runCommandClient('walk_type = "' + walk_type + '"')
    print("Starting script whith inverse_dyn_balance_controller")
    run_test(pkg_path+'/../../lib/'+PKG_NAME+'/main_sim_walk_vel_online.py')

#initializing node to read ros time
rospy.init_node('test_walking_vel', anonymous=True)

#input("Waiting before writing the graph")
time.sleep(5)
runCommandClient("from dynamic_graph import writeGraph")

print("WriteGraph in /tmp/sot_talos_tsid.dot")
runCommandClient("writeGraph('/tmp/sot_talos_tsid_walk.dot')")
print("Convert graph to PDF in /tmp/sot_talos_tsid_walk.pdf")
proc3 = subprocess.Popen(["dot", "-Tpdf", "/tmp/sot_talos_tsid_walk.dot", "-o", "/tmp/sot_talos_tsid_walk.pdf"])

#input("Waiting before setting gains")
time.sleep(3)
print("Setting gains")
# runCommandClient("robot.inv_dyn.kp_feet.value = 6*(500,)")
# runCommandClient("robot.inv_dyn.kp_com.value = 3*(100,)")
# runCommandClient("robot.inv_dyn.kd_feet.value = 6*(12,)")
# runCommandClient("robot.inv_dyn.kd_com.value = 3*(7,)")

if pattern_generator:
    #input("Waiting before playing trajectories")
    time.sleep(2)
    runCommandClient('robot.triggerPG.sin.value = 1')
    runCommandClient('plug(robot.pg.leftfootref, robot.m2qLF.sin)')
    runCommandClient('plug(robot.pg.rightfootref, robot.m2qLF.sin)')
    #input("Wait before stopping the trajectory")
    rospy.sleep(70)
    runCommandClient('robot.pg.velocitydes.value=(0.0,0.0,0.0)')
else:
    #input("Waiting before playing trajectories")
    time.sleep(2)
    print("Playing trajectories")
    runCommandClient("robot.traj_sync.turnOn()")
    #input("Waiting before stopping the trajectories")
    time.sleep(64)
    print("Stop trajectories")
    runCommandClient("robot.traj_sync.turnOff()")
# input("Waiting before reading trajectories")
# runCommandClient('robot.com_traj_gen.playTrajectoryFile(folder + walk_type + "/com.dat")')
# runCommandClient('robot.am_traj_gen.playTrajectoryFile(folder + walk_type + "/am.dat")')
# runCommandClient('robot.phases_traj_gen.playTrajectoryFile(folder + walk_type + "/phases.dat")')
# runCommandClient('robot.rf_traj_gen.playTrajectoryFile(folder + walk_type + "/rightFoot.dat")')
# runCommandClient('robot.lf_traj_gen.playTrajectoryFile(folder + walk_type + "/leftFoot.dat")')

time.sleep(2.0)
#input("Wait before going to halfSitting")
runCommandClient("go_to_position(robot.traj_gen, robot.halfSitting[6:], 5.0)")
time.sleep(5.0)
print("The robot is back in position!")
runCommandClient('dump_tracer(robot.tracer)')

