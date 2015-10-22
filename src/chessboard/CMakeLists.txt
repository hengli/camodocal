if(OPENCV_FOUND)
camodocal_library(camodocal_chessboard SHARED
  Chessboard.cc
)

camodocal_link_libraries(camodocal_chessboard
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_CALIB3D_LIBRARY}
  ${OPENCV_IMGPROC_LIBRARY}
)

camodocal_install(camodocal_chessboard)
endif(OPENCV_FOUND)
