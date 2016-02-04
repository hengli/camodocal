if(OpenCV_FOUND)
camodocal_library(camodocal_chessboard SHARED
  Chessboard.cc
)

camodocal_link_libraries(camodocal_chessboard
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${OpenCV_LIBS}
)

camodocal_install(camodocal_chessboard)
endif(OpenCV_FOUND)
