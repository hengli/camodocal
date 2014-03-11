if(CAMODOCAL_CAMERA_MODELS_FOUND)
if(CAMODOCAL_FEATURES2D_FOUND)

include_directories(
  ../ceres-solver/include
  ../brisk/thirdparty/agast/include
)

camodocal_library(camodocal_visual_odometry SHARED
  FeatureTracker.cc
  SlidingWindowBA.cc
)

camodocal_link_libraries(camodocal_visual_odometry
  ${OPENCV_FEATURES2D_LIBRARY}
  ${OPENCV_FLANN_LIBRARY}
  ${OPENCV_HIGHGUI_LIBRARY}
  ${OPENCV_NONFREE_LIBRARY}
  ${OPENCV_GPU_LIBRARY}
  ${GLIBMM2_LIBRARY}
  ${SIGC++_LIBRARY}
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
  camodocal_brisk
  camodocal_camera_models
  camodocal_features2d
  camodocal_fivepoint
  camodocal_gpl
  camodocal_pose_estimation
  camodocal_sparse_graph
  ceres
)

camodocal_test(SlidingWindowBA)
camodocal_link_libraries(SlidingWindowBA_test camodocal_gpl camodocal_visual_odometry)

if(VCHARGE_VIZ)

camodocal_executable(SlidingWindowBA_sim
  SlidingWindowBA_sim.cc
)

camodocal_link_libraries(SlidingWindowBA_sim
  camodocal_visual_odometry
  middleware
  overlay
) 

endif(VCHARGE_VIZ)

endif(CAMODOCAL_FEATURES2D_FOUND)
endif(CAMODOCAL_CAMERA_MODELS_FOUND)
