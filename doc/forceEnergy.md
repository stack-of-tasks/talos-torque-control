# Pyrene realizing a contact-force task in torque control with passivity

In this simulation the robot creates a contact between a cylinder, hold in its left hand, and a block. The robot applies a 30N force along the z-axis on it before the block is removed. This set-up is meant to represent a first step toward complex operations with contacts. It exposes the problem of unexpected broken contact that can be due to slippage or disturbances.

In the following, we demonstrate how to run this simulation with  <a href="https://github.com/stack-of-tasks/sot-torque-control">sot-torque-control</a>, <a href="https://github.com/loco-3d/sot-talos-balance">sot-talos-balance</a> and talos-torque-control.

## Installation procedure

The installation procedure is different than for the other simulations because it requires specific branches on some packages:

* [sot-talos-balance](https://github.com/NoelieRamuzat/sot-talos-balance/tree/topic/fix_ctrl_manager): branch topic/fix_ctrl_manager
* [tsid](https://github.com/NoelieRamuzat/tsid/tree/topic/task_energy_contact): topic/task_energy_contact
* [sot-torque-control](https://github.com/NoelieRamuzat/sot-torque-control/tree/topic/task_contact_force_energy): branch topic/task_contact_force_energy

You will need to clone the previous packages, switch to the appropriate branches and build them as explained in the <a href="md_doc_installation.html">installation page</a>.
And on [talos-torque-control](https://github.com/NoelieRamuzat/talos-torque-control/tree/topic/task_energy) you will need to switch to the branch topic/task_energy:

```
git checkout topic/task_energy
cd _build-RELEASE
make install
```


## Start the simulation

Go into the talos-torque-control package once successfully installed.
```
cd talos-torque-control
```

Start the simulation with the robot in the half-sitting position, a cylinder tool in its left gripper and a block:
```
python talos-torque-control/python/start_talos_block_gazebo.py
```

## Start the SoT in torque mode

To start the SoT in simulation in torque mode, open a new terminal, then: 
```
roslaunch roscontrol_sot_talos sot_talos_controller_gazebo_effort.launch
```

## Run the test

First of all, you need to go to the folder where the script is, assuming you are in the root directory (open a new terminal):

```
cd script
```

Then, you can just run the test for the contact-force task in torque control with passivity:

```
python sim_wrist_force.py
```

This will launch the simulation, some information will appear in the terminal following the beginning of the simulation.

```
Starting script whith inverse_dyn_balance_controller main_sim_wrist_force.py
initialize SoT:
```

Press the "enter" key to continue.
```
command: robot.inv_dyn = create_balance_controller(robot, conf.balance_ctrl,conf.motor_params, dt, controlType="torque")
standardoutput: WARNING: Could not connect dv_des from BalanceController to ForceTorqueEstimator
WARNING: Could not connect rf/lf_force_traj_gen to f_ref_right/left_foot
```

Do not pay attention to the warning, it is a normal procedure.

```
Wait before starting the dynamic graph
```
Press the "enter" key to continue. The controller will start after that and the robot will lower its CoM.
```
Waiting before writing the graph
```
Press the "enter" key to continue. This command save the entity graph built by the controller.
```
WriteGraph in /tmp/sot_talos_tsid.dot
Convert graph to PDF in /tmp/sot_talos_tsid_walk.pdf
```
It then convert the graph to PDF in the /tmp directory.

```
Wait before going to contact pose
```
Press the "enter" key to continue. The robot will put its left arm in position on the top of the block to be prepared to create a contact.

```
Calibrate wrist force sensors? [y/N] 
Wait before running the calibration
```
Press the "y" and then the "enter" keys to calibrate the sensor.

```
Calibrating sensors...
Sensors are calibrated!
Wait before adding EnergyTask
```
Press the "enter" key to continue. It add the energy tank and constraint to the controller.  
The controller scheme is now the following (if not displayed correctly see [here](pictures/torque_scheme_energy.png)):

\image html torque_scheme_energy.png 

```
Wait before adding TaskLeftHandContact
```
Press the "enter" key to continue. The robot will create the contact between the tool and the block and try to maintain 0N force (thus there are some oscillations).

```
Wait before setting 10N force
```
Press the "enter" key to continue. The robot will then apply the desired force following 2 stages:

* The desired force is first set to 10N to stabilize the contact: there is some slippage between the cylinder and the block until the force applied on the block is enough

* Then it is increased to 30N to have a meaningful force application

Finally after a delay of 15s the following message appears:

```
REMOVE BLOCK ! 
```

You will have to clic on the block in gazebo and the press the "Del" key to delete it (the block is removed). 

Without the passivity constraint on the system, the robot quickly falls after the removal of the block. Whereas with the implemented solution, the force task in the controller is penalized by the energy tank and thus the tracking highly deteriorated, leading to a slow motion of the hand toward the ground. Thus, the controller has the time to remove the dangerous task, ensuring the safety of the system.


## Other

More information on how to use the SoT and how to work on Talos can be found <a href="https://wiki.laas.fr/robots/Pyrene">in the robot wiki page</a> (you need LAAS permissions to access this).
