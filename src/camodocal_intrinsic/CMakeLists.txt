camodocal_library(camodocal_intrinsic
  Chessboard.cc
)

camodocal_link_libraries(camodocal_intrinsic
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_CALIB3D_LIBRARY}
  ${OPENCV_IMGPROC_LIBRARY}
  catacamera
)
