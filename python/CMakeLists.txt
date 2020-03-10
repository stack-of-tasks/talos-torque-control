SET(${PROJECT_NAME}_PYTHON
  __init__.py
  admittance_ctrl_conf.py
  balance_ctrl_conf.py
  balance_ctrl_sim_conf.py
  base_estimator_conf.py
  base_estimator_sim_conf.py
  control_manager_conf.py
  control_manager_sim_conf.py
  create_entities_utils_talos.py
  create_entities_utils_talos_sinusoid.py
  current_controller_conf.py
  current_controller_sim_conf.py
  force_torque_estimator_conf.py
  joint_pos_ctrl_gains.py
  joint_pos_ctrl_gains_sim.py
  joint_torque_controller_conf.py
  joint_torque_controller_sim_conf.py
  motors_parameters.py
  motors_parameters_sim.py
  sot_utils_talos.py
  ddp_controller_conf.py
  main_ddp_talos.py
  main_sim_com_torque.py
  main_sim_com_vel.py
  )

FOREACH(file ${${PROJECT_NAME}_PYTHON})
  PYTHON_INSTALL_ON_SITE(dynamic_graph/sot/torque_control/talos ${file})
ENDFOREACH(file ${${PROJECT_NAME}_PYTHON})
