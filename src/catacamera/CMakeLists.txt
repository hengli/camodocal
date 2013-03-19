include_directories(
  ../ceres-solver/include
)

camodocal_library(catacamera SHARED
  CameraRigExtrinsics.cc
  CataCamera.cc
  CataCameraCalibration.cc
)

camodocal_link_libraries(catacamera
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_IMGPROC_LIBRARY}
  ${OPENCV_CALIB3D_LIBRARY}
  ceres
  gpl
)
