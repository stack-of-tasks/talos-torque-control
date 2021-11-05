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
walk_type = "on_spot"
pattern_generator = False

if len(argv) == 4 and argv[1] == "on_spot":
    print("Starting script with folder " + folder + " walking on spot.")
elif len(argv) == 4 and argv[1] == "walk_20":
    print("Starting script with folder " + folder + " walking with 20cm step.")
    walk_type = "walk_20"
elif len(argv) == 5 and argv[1] == "on_spot" and argv[2] == "pattern_generator":
    pattern_generator = True
    print("Starting script with pattern_generator walking on spot.")
elif len(argv) == 5 and argv[1] == "walk_20" and argv[2] == "pattern_generator":
    pattern_generator = True
    walk_type = "walk_20"
    print("Starting script with pattern_generator walking with 20cm step.")
elif len(argv) == 5 and argv[1] == "on_spot":
    folder = argv[2]
    print("Starting script with folder " + folder + " walking on spot.")
elif len(argv) == 5 and argv[1] == "walk_20":
    folder = argv[2]
    print("Starting script with folder " + folder + " walking with 20cm step.")
    walk_type = "walk_20"
# Default option: on_spot, without pattern generator
elif len(argv) == 3:
    print("Starting script with folder " + folder + " walking on spot.")

else: 
    print("Usage: python sim_walk_torque.py walk_type:=[on_spot|walk_20] {pattern_generator|path_folder_of_the_reference_trajectories}")
    print("By giving only the walk_type the script starts using the default file trajectories")
    raise ValueError("Bad options")

if not pattern_generator:
    runCommandClient('folder = "' + folder + '"')
    runCommandClient('walk_type = "' + walk_type + '"')
    print("Starting script whith inverse_dyn_balance_controller main_sim_walk_torque.py")
    run_test(pkg_path + '/../../lib/'+PKG_NAME+'/main_sim_walk_torque.py')
else:
    runCommandClient('walk_type = "' + walk_type + '"')
    print("Starting script whith inverse_dyn_balance_controller main_sim_walk_torque_online.py")
    run_test(pkg_path + '/../../lib/'+PKG_NAME+'/main_sim_walk_torque_online.py')

#initializing node to read ros time
rospy.init_node('test_torque_walking', anonymous=True)

#input("Waiting before writing the graph")
time.sleep(5)
runCommandClient("from dynamic_graph import writeGraph")

print("WriteGraph in /tmp/sot_talos_tsid.dot")
runCommandClient("writeGraph('/tmp/sot_talos_tsid_walk.dot')")
print("Convert graph to PDF in /tmp/sot_talos_tsid_walk.pdf")
proc3 = subprocess.Popen(["dot", "-Tpdf", "/tmp/sot_talos_tsid_walk.dot", "-o", "/tmp/sot_talos_tsid_walk_torque.pdf"])

if pattern_generator:
    #input("Waiting before setting gains")
    time.sleep(3)
    print("Setting gains")
    runCommandClient("robot.inv_dyn.kp_feet.value = 6*(250,)")
    #runCommandClient("robot.inv_dyn.kp_com.value = 3*(400,)")
    runCommandClient("robot.inv_dyn.kd_feet.value = 6*(12,)")
    #runCommandClient("robot.inv_dyn.kd_com.value = 3*(12,)")
    #input("Waiting before playing trajectories")
    time.sleep(2)
    runCommandClient('robot.triggerPG.sin.value = 1')
    rospy.sleep(70) #arbitrary value
    #input("Wait before stopping the trajectory")
    runCommandClient('robot.pg.velocitydes.value=(0.0,0.0,0.0)')
else:
    #input("Waiting before setting gains")
    time.sleep(3)
    print("Setting gains")
    runCommandClient("robot.inv_dyn.kp_feet.value = 6*(200,)")
    # runCommandClient("robot.inv_dyn.kp_com.value = 3*(250,)")
    runCommandClient("robot.inv_dyn.kd_feet.value = 6*(12,)")
    # runCommandClient("robot.inv_dyn.kd_com.value = 3*(12,)")
    #input("Waiting before playing trajectories")
    time.sleep(2)
    print("Playing trajectories")
    runCommandClient("robot.traj_sync.turnOn()")
    rospy.sleep(64) #64 seconds is the time necessary for the on_spot trajectory (the longest one)
    #input("Waiting before stopping the trajectories")
    print("Stop trajectories")
    runCommandClient("robot.traj_sync.turnOff()")

time.sleep(2.0)
#input("Wait before going to halfSitting")
runCommandClient("go_to_position(robot.traj_gen, robot.halfSitting[6:], 5.0)")
time.sleep(5.0)
print("The robot is back in position!")
runCommandClient('dump_tracer(robot.tracer)')
