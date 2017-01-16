
if(OpenCV_FOUND)
include_directories(
  ${CERES_INCLUDE_DIRECTORIES}
)

camodocal_library(camodocal_camera_systems SHARED
  CameraSystem.cc
)

camodocal_link_libraries(camodocal_camera_systems
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  camodocal_camera_models
  camodocal_pugixml
)

camodocal_install(camodocal_camera_systems)
endif()
