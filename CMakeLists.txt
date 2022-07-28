# Copyright 2014, 2020, Andrea Del Prete, Guilhem Saurel, LAAS-CNRS

cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME talos-torque-control)
set(PROJECT_DESCRIPTION
    "Collection of dynamic-graph entities aiming at the implementation of torque control on TALOS."
)
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)
option(INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" ON)

# Project configuration
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CUSTOM_HEADER_DIR "sot/torque-control")
set(CXX_DISABLE_WERROR TRUE)
set(DOXYGEN_USE_MATHJAX YES)

# JRL-cmakemodule setup
include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/sphinx.cmake)

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})
check_minimal_cxx_standard(11 ENFORCE)

if(INITIALIZE_WITH_NAN)
  message(STATUS "Initialize with NaN all the Eigen entries.")
  add_definitions(-DEIGEN_INITIALIZE_MATRICES_BY_NAN)
endif(INITIALIZE_WITH_NAN)

# Project dependencies
add_project_dependency(sot-torque-control REQUIRED PKG_CONFIG_REQUIRES
                       sot-torque-control)
add_project_dependency(dynamic-graph-python 4.0.6 REQUIRED)

# Main Library
set(${PROJECT_NAME}_HEADERS include/${CUSTOM_HEADER_DIR}/talos-common.hh)

set(${PROJECT_NAME}_SOURCES src/talos-common.cpp)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} sot-torque-control::sot-torque-control)

if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif(SUFFIX_SO_VERSION)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

add_subdirectory(python)

pkg_config_append_libs(${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
