# Copyright 2014, 2020, Andrea Del Prete, Guilhem Saurel, LAAS-CNRS

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

# Project properties
SET(PROJECT_ORG pyrene-dev)
SET(PROJECT_NAME talos-torque-control)
SET(PROJECT_DESCRIPTION "Collection of dynamic-graph entities aiming at the implementation of torque control on TALOS.")
SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
OPTION(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
OPTION(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python bindings" OFF)
OPTION(SUFFIX_SO_VERSION "Suffix library name with its version" ON)
OPTION(INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" ON)

# Project configuration
IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  SET(PROJECT_USE_CMAKE_EXPORT TRUE)
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
SET(CUSTOM_HEADER_DIR "sot/torque-control")
SET(CXX_DISABLE_WERROR TRUE)
SET(DOXYGEN_USE_MATHJAX YES)

# JRL-cmakemodule setup
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/sphinx.cmake)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})
CHECK_MINIMAL_CXX_STANDARD(11 ENFORCE)

IF(INITIALIZE_WITH_NAN)
  MESSAGE(STATUS "Initialize with NaN all the Eigen entries.")
  ADD_DEFINITIONS(-DEIGEN_INITIALIZE_MATRICES_BY_NAN)
ENDIF(INITIALIZE_WITH_NAN)

# Project dependencies
ADD_PROJECT_DEPENDENCY(sot-torque-control REQUIRED PKG_CONFIG_REQUIRES sot-torque-control)

IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  ADD_PROJECT_DEPENDENCY(dynamic-graph-python 4.0.0 REQUIRED
    PKG_CONFIG_REQUIRES dynamic-graph-python)
ENDIF(BUILD_PYTHON_INTERFACE)

# Main Library
SET(${PROJECT_NAME}_HEADERS
  include/${CUSTOM_HEADER_DIR}/talos-common.hh
  )

SET(${PROJECT_NAME}_SOURCES
  src/talos-common.cpp
  )

ADD_LIBRARY(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  sot-torque-control::sot-torque-control)

IF(SUFFIX_SO_VERSION)
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
ENDIF(SUFFIX_SO_VERSION)

IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

IF(BUILD_PYTHON_INTERFACE)
  ADD_SUBDIRECTORY(python)
ENDIF(BUILD_PYTHON_INTERFACE)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})
IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

# Install scripts
find_package(catkin REQUIRED)

catkin_package()
catkin_install_python(PROGRAMS
                      script/sim_walk_torque.py
                      script/run_test_utils.py
                      script/sim_walk_vel.py
                      script/test_ddp_sinu_effort.py
                      script/integ_sim_walk_torque.py
                      script/integ_run_test_utils.py
                      script/integ_sim_walk_vel.py
                      python/dynamic_graph/sot/torque_control/talos/main_sim_walk_torque.py
                      python/dynamic_graph/sot/torque_control/talos/main_sim_walk_torque_online.py
                      python/dynamic_graph/sot/torque_control/talos/main_sim_walk_vel.py
                      python/dynamic_graph/sot/torque_control/talos/main_sim_walk_vel_online.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

FOREACH(dir python traj_multicontact_api)
  INSTALL(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
ENDFOREACH(dir)