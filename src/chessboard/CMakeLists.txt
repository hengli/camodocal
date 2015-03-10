camodocal_library(camodocal_chessboard SHARED
  Chessboard.cc
)

camodocal_link_libraries(camodocal_chessboard
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_CALIB3D_LIBRARY}
  ${OPENCV_IMGPROC_LIBRARY}
)
