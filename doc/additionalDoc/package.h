/** \mainpage
\section sec_intro Introduction

This repository contains a set of dynamic-graph entities for the implementation of sensor-feedback control of the robot Talos. 
It is based on the _sot-torque-control_ repository, you can find a description of the entities in the following wiki:
https://github.com/stack-of-tasks/sot-torque-control/wiki

\section sec_Requirements Requirements 

This package should be compiled and installed with cmake.

This project depends on several packages:
 \li [dynamic-graph] (https://github.com/jrl-umi3218/dynamic-graph) >= 3.0.0
 \li [dynamic-graph-python] (https://github.com/stack-of-tasks/dynamic-graph-python) >= 3.0.0
 \li [sot-core] (https://github.com/stack-of-tasks/sot-core) >= 3.0.0
 \li [pinocchio] (https://github.com/stack-of-tasks/pinocchio) >= 1.2
 \li [sot-torque-control] (https://github.com/stack-of-tasks/sot-torque-control)
 \li [PinInvDyn] (https://github.com/stack-of-tasks/invdyn)
 \li [parametric-curves] (https://github.com/stack-of-tasks/parametric-curves)
 \li [jrl-mal] (https://github.com/jrl-umi3218/jrl-mal)
 \li lapack
 \li boost
 \li pthread

Most of these packages can be installed through [robotpkg](http://robotpkg.openrobots.org/).
In particular, you can find them in [robotpkg-wip](http://robotpkg.openrobots.org/robotpkg-wip.html) (work in progress), a subset of robotpkg.
Pay attention not to install ROS using robotpkg though, because it would install the latest version, which may not be what you need.
Their presence will be checked with the use of pkg-config.

\section sec_install Installation

    git clone --recursive git@gepgitlab.laas.fr:nramuzat/talos-torque-control.git
    cd talos-torque-control
    git checkout devel_ddp
    mkdir _build-RELEASE
    cd _build-RELEASE
    cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=$DEVEL_DIR/openrobots
    make install

In Ubuntu 14.04 you may need to add `-DCMAKE_CXX_FLAGS="-std=c++11"`.

\section sec_gist Example of use

If you want to launch a simulation you have to follow the next steps:
    
    roslaunch talos_gazebo talos_gazebo_pose.launch
    roslaunch roscontrol_sot_talos sot_talos_controller_gazebo_effort.launch
    rosrun dynamic_graph_bridge run_command
    

And then start to execute the program you want, for instance with the _main_sim_talos_sinusoid_ :

    from dynamic_graph.sot.torque_control.talos.main_sim_talos_sinusoid import *
    main_sim_v3(robot, startSoT=False, go_half_sitting=True)
    start_sot()


If you want to save the dynamic graph created, use the following code (in the _rosrun_):

    from dynamic_graph import *
    writeGraph('/PATH/NAME_OF_THE_GRAPH.dot')
    

Instead of using _rosrun_ you can also use the script _sim_simu_effort.py_ (which execute the same bunch of code).
*/

