include_directories(
  ${CERES_INCLUDE_DIRECTORIES}
)

if(GLOG_FOUND AND OpenCV_FOUND)
camodocal_library(camodocal_camera_models SHARED
  Camera.cc
  CameraFactory.cc
  CataCamera.cc
  CostFunctionFactory.cc
  EquidistantCamera.cc
  PinholeCamera.cc
  ScaramuzzaCamera.cc
)

camodocal_link_libraries(camodocal_camera_models
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${OpenCV_LIBS}
  camodocal_gpl
  ceres
  ${GLOG_LIBRARIES}
)

camodocal_install(camodocal_camera_models)

camodocal_test(CataCamera)
camodocal_link_libraries(CataCamera_test camodocal_camera_models)

camodocal_test(EquidistantCamera)
camodocal_link_libraries(EquidistantCamera_test camodocal_camera_models)

camodocal_test(PinholeCamera)
camodocal_link_libraries(PinholeCamera_test camodocal_camera_models)

endif(GLOG_FOUND AND OpenCV_FOUND)
