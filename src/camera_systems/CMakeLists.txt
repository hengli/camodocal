include_directories(
  ../ceres-solver/include
)

camodocal_library(camodocal_camera_systems
  CameraSystem.cc
)

camodocal_link_libraries(camodocal_camera_systems
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  camodocal_camera_models
  camodocal_pugixml
)
