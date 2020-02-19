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
    runCommandClient("from dynamic_graph import writeGraph")
    runCommandClient("from dynamic_graph.sot.torque_control.talos.main_sim_com_vel import *")

    print("Initialize sinusoid simulation (Dynamic Graph)")
    runCommandClient("main_com(robot, use_real_base_state=True)")

    print("WriteGraph in /tmp/sot_talos_tsid_effort.dot")
    runCommandClient("writeGraph('/tmp/sot_talos_tsid_com.dot')")
    print("Convert graph to PDF in /tmp/sot_talos_tsid_com.pdf")
    proc3 = subprocess.Popen(["dot", "-Tpdf", "/tmp/sot_talos_tsid_com.dot", "-o", "/tmp/sot_talos_tsid_com.pdf"])

    raw_input("Wait before starting the dynamic graph (SoT)")
    runCommandClient("start_sot()")
    # runCommandClient("robot.com_traj_gen.move(0,-0.005,0.50)")

    raw_input("Wait before going to halfSitting")
    runCommandClient("go_to_position(robot.traj_gen, robot.halfSitting[6:], 10.0)")    

    raw_input("Waiting before going to sinusoid pose")
    print("Go to sinusoid pose")
    runCommandClient("robot.com_traj_gen.move(1,-0.025,1.0)")

    raw_input("Waiting before starting sinusoid")
    print("Start sinusoid")
    runCommandClient("robot.com_traj_gen.startSinusoid(1,0.025,2.0)")

    raw_input("Waiting before stopping sinusoid")
    print("Stop Sinusoid")
    runCommandClient("robot.com_traj_gen.stop(1)")
    time.sleep(1.0)
    print("Putting the robot back...")
    runCommandClient('robot.com_traj_gen.move(1,0.0,1.0)')
    time.sleep(1.0)
    print("The robot is back in position!")
    


except rospy.ServiceException, e:
    rospy.logerr("Service call failed: %s" % e)


