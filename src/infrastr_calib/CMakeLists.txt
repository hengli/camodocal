include_directories(
  ../ceres-solver/include
  ../dbow2/DBoW2
  ../dbow2/DUtils
  ../dbow2/DUtils
  ../dbow2/DUtilsCV
  ../dbow2/DVision
)

camodocal_library(camodocal_infrastr_calib SHARED
  InfrastructureCalibration.cc
)

camodocal_link_libraries(camodocal_infrastr_calib
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_THREAD_LIBRARY}
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_IMGPROC_LIBRARY}
  ${OPENCV_CALIB3D_LIBRARY}
  ${OPENCV_VERSION_SPECIFIC_LIBRARIES}
  camodocal_camera_systems
  camodocal_features2d
  camodocal_gpl
  camodocal_fivepoint
  camodocal_pose_estimation
  camodocal_pose_graph
  camodocal_sparse_graph
  ceres
)

if(VCHARGE_VIZ)
camodocal_link_libraries(camodocal_infrastr_calib
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  overlay
)

camodocal_install(camodocal_infrastr_calib)

endif(VCHARGE_VIZ)
