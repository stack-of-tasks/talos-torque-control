This package implements torque control on the TALOS humanoid robot of the LAAS-CNRS: Pyrene.
It uses the following packages:

* [tsid](https://github.com/stack-of-tasks/tsid): The Task Space Inverse Dynamics library 
* [sot-torque-control](https://github.com/stack-of-tasks/sot-torque-control): The wrapper of TSID in the Stack-of-Task framework

One can find the documentation of the package in the ```/doc``` directory (doxygen documentation that is built when calling ```make install```) :
* [The overview of the package](doc/Overview.md)
* [The installation procedure](doc/installation.md)
* [Instructions for running a simulation of Pyrene executing a CoM sinusoid in position or torque control](doc/running.md)
* [Instructions for running a simulation or an experiment using the DDP on the right elbow of Pyrene](doc/ddpRun.md)
* [Instructions for running a simulation of Pyrene executing a foot sinusoid in the air in torque control](doc/bellStepRun.md)
* [Instructions for running a simulation of Pyrene walking in torque control](doc/walkRun.md)
* [Instructions for running a simulation of Pyrene realizing a contact-force task in torque control with passivity](doc/forceEnergy.md)