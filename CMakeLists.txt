# Copyright 2014, Andrea Del Prete, LAAS/CNRS

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_ORG pyrene-dev)
SET(PROJECT_NAME talos-torque-control)
SET(PROJECT_DESCRIPTION "Collection of dynamic-graph entities aiming at the implementation of torque control on TALOS.")
SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

SET(CUSTOM_HEADER_DIR "sot/torque-control")

SET(DOXYGEN_USE_MATHJAX YES)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)
#add_compile_options(-std=c++11) # CMake 2.8.12 or newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SETUP_PROJECT()

OPTION (INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" ON)
IF(INITIALIZE_WITH_NAN)
  MESSAGE(STATUS "Initialize with NaN all the Eigen entries.")
  ADD_DEFINITIONS(-DEIGEN_INITIALIZE_MATRICES_BY_NAN)
ENDIF(INITIALIZE_WITH_NAN)

PKG_CONFIG_APPEND_LIBS("sot-torque-control")

# Search for dependencies.
# Boost
SET(BOOST_COMPONENTS thread filesystem program_options unit_test_framework system regex )
SEARCH_FOR_BOOST()

SEARCH_FOR_EIGEN()

ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 1.0.0")
ADD_REQUIRED_DEPENDENCY("dynamic-graph-python")
ADD_REQUIRED_DEPENDENCY("sot-core")
ADD_REQUIRED_DEPENDENCY("pinocchio >= 1.2")
ADD_REQUIRED_DEPENDENCY("tsid")
ADD_REQUIRED_DEPENDENCY("sot-torque-control")

include_directories(include)


SET(TALOS_LIBRARY_NAME ${PROJECT_NAME})

SET(${TALOS_LIBRARY_NAME}_HEADERS
        include/sot/torque_control/talos-common.hh
  )

#INSTALL(FILES ${${TALOS_LIBRARY_NAME}_HEADERS}
#  DESTINATION include/sot/torque-control
#  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

SET(${TALOS_LIBRARY_NAME}_SOURCES ${${TALOS_LIBRARY_NAME}_HEADERS}
    src/talos-common.cpp
        )
SET(${TALOS_LIBRARY_NAME}_PYTHON_FILES python/*.py)

ADD_LIBRARY(${TALOS_LIBRARY_NAME} SHARED ${${TALOS_LIBRARY_NAME}_SOURCES})

SET_TARGET_PROPERTIES(${TALOS_LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

MESSAGE(STATUS "talos_library_name: ${TALOS_LIBRARY_NAME}")
PKG_CONFIG_USE_DEPENDENCY(${TALOS_LIBRARY_NAME} dynamic-graph)
PKG_CONFIG_USE_DEPENDENCY(${TALOS_LIBRARY_NAME} dynamic-graph-python)
PKG_CONFIG_USE_DEPENDENCY(${TALOS_LIBRARY_NAME} sot-core)
PKG_CONFIG_USE_DEPENDENCY(${TALOS_LIBRARY_NAME} pinocchio)
PKG_CONFIG_USE_DEPENDENCY(${TALOS_LIBRARY_NAME} tsid)


IF(UNIX)
  TARGET_LINK_LIBRARIES(${TALOS_LIBRARY_NAME} ${CMAKE_DL_LIBS})
ENDIF(UNIX)

IF(UNIX AND NOT APPLE)
  TARGET_LINK_LIBRARIES(${TALOS_LIBRARY_NAME} pthread)
ENDIF(UNIX AND NOT APPLE)

TARGET_LINK_LIBRARIES(${TALOS_LIBRARY_NAME} ${Boost_LIBRARIES})

INSTALL(TARGETS ${TALOS_LIBRARY_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

FINDPYTHON()

INSTALL(FILES python/dynamic_graph/sot/torque_control/tests/__init__.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/torque_control/tests)

INSTALL(FILES python/dynamic_graph/sot/torque_control/talos/__init__.py
              python/dynamic_graph/sot/torque_control/talos/admittance_ctrl_conf.py
              python/dynamic_graph/sot/torque_control/talos/balance_ctrl_conf.py
              python/dynamic_graph/sot/torque_control/talos/balance_ctrl_sim_conf.py
              python/dynamic_graph/sot/torque_control/talos/base_estimator_conf.py
              python/dynamic_graph/sot/torque_control/talos/base_estimator_sim_conf.py
              python/dynamic_graph/sot/torque_control/talos/control_manager_conf.py
              python/dynamic_graph/sot/torque_control/talos/control_manager_sim_conf.py
              python/dynamic_graph/sot/torque_control/talos/create_entities_utils_talos.py
              python/dynamic_graph/sot/torque_control/talos/create_entities_utils_talos_sinusoid.py
              python/dynamic_graph/sot/torque_control/talos/current_controller_conf.py
              python/dynamic_graph/sot/torque_control/talos/current_controller_sim_conf.py
              python/dynamic_graph/sot/torque_control/talos/force_torque_estimator_conf.py
              python/dynamic_graph/sot/torque_control/talos/joint_pos_ctrl_gains.py
              python/dynamic_graph/sot/torque_control/talos/joint_pos_ctrl_gains_sim.py
              python/dynamic_graph/sot/torque_control/talos/joint_torque_controller_conf.py
              python/dynamic_graph/sot/torque_control/talos/joint_torque_controller_sim_conf.py
              python/dynamic_graph/sot/torque_control/talos/motors_parameters.py
              python/dynamic_graph/sot/torque_control/talos/motors_parameters_sim.py
              python/dynamic_graph/sot/torque_control/talos/sot_utils_talos.py
              python/dynamic_graph/sot/torque_control/talos/ddp_controller_conf.py
              python/dynamic_graph/sot/torque_control/talos/main_ddp_talos.py
              python/dynamic_graph/sot/torque_control/talos/main_sim_com_torque.py
              python/dynamic_graph/sot/torque_control/talos/main_sim_com_vel.py
              script/run_test_utils.py
              script/test_ddp_sinu_effort.py
              script/sim_com.py
DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/torque_control/talos)

ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
