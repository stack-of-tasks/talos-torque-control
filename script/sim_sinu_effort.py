#!/usr/bin/python
import sys
import rospy
import subprocess, time
from std_srvs.srv import *
from dynamic_graph_bridge.srv import *
from dynamic_graph_bridge_msgs.srv import *
from run_test_utils import *


# proc1 = subprocess.Popen(["xfce4-terminal", "--title", "Launch Gazebo Pose","--command", "roslaunch talos_gazebo talos_gazebo_pose.launch", "--hold"])
# time.sleep(10)
# proc2 = subprocess.Popen(["xfce4-terminal", "--title", "Launch Sot Effort", "--command", "roslaunch roscontrol_sot_talos sot_talos_controller_gazebo_effort.launch", "--hold"])
# time.sleep(20)

# run_test("test_effort.py")

# Waiting for services
try:
    rospy.loginfo("Waiting for run_command")
    rospy.wait_for_service('/run_command')
    rospy.loginfo("...ok")

    rospy.loginfo("Waiting for start_dynamic_graph")
    rospy.wait_for_service('/start_dynamic_graph')
    rospy.loginfo("...ok")

    runCommandClient = rospy.ServiceProxy('run_command', RunCommand)
    runCommandStartDynamicGraph = rospy.ServiceProxy('start_dynamic_graph', Empty)

    raw_input("Waiting before launching the graph")
    runCommandClient("from dynamic_graph import *")
    #runCommandClient("from dynamic_graph.sot.torque_control.talos.main_sim_talos import *")
    runCommandClient("from dynamic_graph.sot.torque_control.talos.main_sim_talos_sinusoid import *")
    # runCommandClient("from dynamic_graph.sot.torque_control.talos.main_talos import *")

    # print("Initialize ddp_actuator simulation (Dynamic Graph)")
    # runCommandClient("test_ddp_actuator(robot, startSoT=False)")
    print("Initialize sinusoid simulation (Dynamic Graph)")
    # runCommandClient("robot.initializeRobot()")
    runCommandClient("main_sim_v3(robot, startSoT=False, go_half_sitting=True)")#, startSoT=False, go_half_sitting=True)")


    print("WriteGraph in /tmp/sot_talos_tsid_effort.dot")
    runCommandClient("writeGraph('/tmp/sot_talos_tsid_effort.dot')")
    print("Convert graph to PDF in /tmp/sot_talos_tsid_effort.pdf")
    proc3 = subprocess.Popen(["dot", "-Tpdf", "/tmp/sot_talos_tsid_effort.dot", "-o", "/tmp/sot_talos_tsid_effort.pdf"])

    raw_input("Wait before starting the dynamic graph (SoT)")
    runCommandClient("start_sot()")

    # print("Go to sinusoid pose")
    # runCommandClient("go_to_position_sinusoid(robot)")

    # print("Start Sinusoid move")
    # runCommandClient("start_movement_sinusoid(robot)") 
    # print("Stop Sinusoid move")
    # runCommandClient("stop_movement_sinusoid(robot)")
    


except rospy.ServiceException, e:
    rospy.logerr("Service call failed: %s" % e)


