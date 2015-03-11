include_directories(
  ../ceres-solver/include
)

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
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_IMGPROC_LIBRARY}
  ${OPENCV_CALIB3D_LIBRARY}
  camodocal_gpl
  ceres
)

camodocal_test(CataCamera)
camodocal_link_libraries(CataCamera_test camodocal_camera_models)

camodocal_test(EquidistantCamera)
camodocal_link_libraries(EquidistantCamera_test camodocal_camera_models)

camodocal_test(PinholeCamera)
camodocal_link_libraries(PinholeCamera_test camodocal_camera_models)
