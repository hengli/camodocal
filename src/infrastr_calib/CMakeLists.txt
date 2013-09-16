if(CERES_FOUND)

include_directories(
  ../ceres-solver/include
  ../dbow2/DBoW2
  ../dbow2/DUtils
  ../dbow2/DUtils
  ../dbow2/DUtilsCV
  ../dbow2/DVision
)

camodocal_library(camodocal_infrastr_calib
  InfrastructureCalibration.cc
  LocationRecognition.cc
)

camodocal_link_libraries(camodocal_infrastr_calib
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_THREAD_LIBRARY}
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_IMGPROC_LIBRARY}
  ${OPENCV_CALIB3D_LIBRARY}
  camodocal_camera_models
  camodocal_camera_systems
  camodocal_DBoW2
  camodocal_DUtilsCV
  camodocal_DUtils
  camodocal_DVision
  camodocal_features2d
  camodocal_gpl
  camodocal_fivepoint
  camodocal_sparse_graph
  ceres
)

if(VCHARGE_VIZ)
camodocal_link_libraries(camodocal_infrastr_calib
  overlay
)
endif(VCHARGE_VIZ)

endif(CERES_FOUND)
