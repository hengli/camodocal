###################### InstallProjectConfig  ###########################

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${PROJECT_CMAKE_DIR}/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKE_DIR}
  )

install(EXPORT ${PROJECT_NAME}Targets 
  DESTINATION ${INSTALL_CMAKE_DIR}
  )

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION ${INSTALL_CMAKE_DIR}
  COMPONENT Devel
  )
