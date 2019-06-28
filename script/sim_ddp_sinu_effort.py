#!/usr/bin/python
import sys
import rospy
import subprocess, time
from std_srvs.srv import *
from dynamic_graph_bridge.srv import *
from dynamic_graph_bridge_msgs.srv import *
from run_test_utils import *

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
    runCommandClient("from dynamic_graph.sot.torque_control.talos.main_sim_ddp_talos import *")

    print("Initialize DDP sinusoid simulation (Dynamic Graph)")
    runCommandClient("ddp_actuator(robot, startSoT=True, go_half_sitting=True)")

    print("WriteGraph in /tmp/sot_ddp_talos_tsid_effort.dot")
    runCommandClient("writeGraph('/tmp/sot_ddp_talos_tsid_effort.dot')")
    print("Convert graph to PDF in /tmp/sot_ddp_talos_tsid_effort.pdf")
    proc3 = subprocess.Popen(["dot", "-Tpdf", "/tmp/sot_ddp_talos_tsid_effort.dot", "-o", "/tmp/sot_ddp_talos_tsid_effort.pdf"])

    raw_input("Wait before starting the dynamic graph (SoT)")
    runCommandClient("start_sot()")

    raw_input("Waiting before going to sinusoid pose")
    print("Go to sinusoid pose")
    runCommandClient("go_to_position_sinusoid(robot)")

    raw_input("Waiting before starting sinusoid move")
    print("Start Sinusoid move")
    runCommandClient("start_movement_sinusoid(robot)") 

    raw_input("Waiting before stopping sinusoid move")
    print("Stop Sinusoid move")
    runCommandClient("stop_movement_sinusoid(robot)")
    


except rospy.ServiceException, e:
    rospy.logerr("Service call failed: %s" % e)


