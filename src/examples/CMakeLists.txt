if(CAMODOCAL_CALIB_FOUND)

camodocal_executable(intrinsic_calib
  intrinsic_calib.cc
)

camodocal_link_libraries(intrinsic_calib
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_HIGHGUI_LIBRARY}
  camodocal_calib
  camodocal_chessboard
)

camodocal_executable(stereo_calib
  stereo_calib.cc
)

camodocal_link_libraries(stereo_calib
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_HIGHGUI_LIBRARY}
  camodocal_calib
  camodocal_chessboard
)

camodocal_executable(extrinsic_calib
  extrinsic_calib.cc
)

camodocal_link_libraries(extrinsic_calib
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  camodocal_calib
)

endif(CAMODOCAL_CALIB_FOUND)
