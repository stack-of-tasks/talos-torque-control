SET(plugins
)


#Plugins compilation, link, and installation
#Compiles a plugin. The plugin library is ${LIBRARY_NAME}
FOREACH(plugin ${plugins})
  #retrieve plugin name
  GET_FILENAME_COMPONENT(LIBRARY_NAME ${plugin} NAME)
  
  # only one source file per plugin
  ADD_LIBRARY(${LIBRARY_NAME}
    SHARED
    ${plugin}.cpp)

  #remove the "lib" prefix from the plugin output name
  SET_TARGET_PROPERTIES(${LIBRARY_NAME}
    PROPERTIES
    PREFIX "")

  SET_TARGET_PROPERTIES(${LIBRARY_NAME}
    PROPERTIES
    SOVERSION ${PROJECT_VERSION}
    INSTALL_RPATH ${DYNAMIC_GRAPH_PLUGINDIR})

  # Link with sot-torque-control library
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${TALOS_LIBRARY_NAME})
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-torque-control)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${Boost_LIBRARIES})

  ADD_DEPENDENCIES(${LIBRARY_NAME} ${TALOS_LIBRARY_NAME})

  IF(ADDITIONAL_${LIBRARY_NAME}_LIBS)
    ADD_DEPENDENCIES(${LIBRARY_NAME} ${ADDITIONAL_${LIBRARY_NAME}_LIBS})
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${ADDITIONAL_${LIBRARY_NAME}_LIBS})
  ENDIF(ADDITIONAL_${LIBRARY_NAME}_LIBS)	
  
  # Linux dynamic loading library flags
  IF(UNIX)
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${CMAKE_DL_LIBS})
  ENDIF(UNIX)

  IF(UNIX AND NOT APPLE)
    TARGET_LINK_LIBRARIES(${LIBRARY_NAME} dl pthread)
  ENDIF(UNIX AND NOT APPLE)

  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-core)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} pinocchio)

  # build python submodule
  # (replace minus with underscore to make Python happy)
  STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})
  DYNAMIC_GRAPH_PYTHON_MODULE("sot/torque_control/${PYTHON_LIBRARY_NAME}"
    ${LIBRARY_NAME}
    sot-torque-control-${PYTHON_LIBRARY_NAME}-wrap
    )
  # Install plugins
  INSTALL(TARGETS ${LIBRARY_NAME}
    DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})
ENDFOREACH(plugin)
