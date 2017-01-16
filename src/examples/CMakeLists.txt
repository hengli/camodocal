
if(CAMODOCAL_CALIB_FOUND AND OpenCV_FOUND AND HAVE_OPENCV_XFEATURES2D_NONFREE)

camodocal_executable(intrinsic_calib
  intrinsic_calib.cc
)

camodocal_link_libraries(intrinsic_calib
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${OpenCV_LIBS}
  camodocal_calib
  camodocal_chessboard
)

camodocal_executable(stereo_calib
  stereo_calib.cc
)

camodocal_link_libraries(stereo_calib
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${OpenCV_LIBS}
  camodocal_calib
  camodocal_chessboard
)

camodocal_executable(extrinsic_calib
  extrinsic_calib.cc
)

camodocal_link_libraries(extrinsic_calib
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  camodocal_calib
)

endif(CAMODOCAL_CALIB_FOUND AND OpenCV_FOUND AND HAVE_OPENCV_XFEATURES2D_NONFREE)

if(OpenCV_FOUND)
include_directories(
  ../dbow2/DBoW2
  ../dbow2/DUtils
  ../dbow2/DUtils
  ../dbow2/DUtilsCV
  ../dbow2/DVision
)

camodocal_executable(train_voctree
  train_voctree.cc
)

camodocal_link_libraries(train_voctree
  camodocal_dbow2
)
endif()
