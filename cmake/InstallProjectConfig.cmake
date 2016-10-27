###################### InstallProjectConfig  ###########################

include(CMakePackageConfigHelpers)

# Configuration
set(version_config "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake")

# Configure '<PROJECT-NAME>ConfigVersion.cmake'
# Note: PROJECT_VERSION is used as a VERSION
write_basic_package_version_file(
    "${version_config}" COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  ${PROJECT_CMAKE_DIR}/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKE_DIR}
  )

install(EXPORT ${PROJECT_NAME}Targets 
  DESTINATION ${INSTALL_CMAKE_DIR}
  )

install(
  FILES 
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${INSTALL_CMAKE_DIR}
  COMPONENT Devel
  )

export(PACKAGE ${PROJECT_NAME})


###################### Add uninstall target ############################
if("${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")
    ADD_CUSTOM_TARGET(uninstall
      COMMAND ${CMAKE_COMMAND} -P ${PROJECT_CMAKE_DIR}/cmake_uninstall.cmake)
endif()