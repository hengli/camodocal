if(EXISTS ${OPENCV_GPU_LIBRARY})

include_directories(
  ../brisk/thirdparty/agast/include
  ../ceres-solver/include
)

camodocal_library(visual_odometry
  FeatureTracker.cc
  ORBGPU.cc
  SlidingWindowBA.cc
  SurfGPU.cc
)

camodocal_link_libraries(visual_odometry
  ${OPENCV_FEATURES2D_LIBRARY}
  ${OPENCV_FLANN_LIBRARY}
  ${OPENCV_HIGHGUI_LIBRARY}
  ${OPENCV_NONFREE_LIBRARY}
  ${OPENCV_GPU_LIBRARY}
  ${GLIBMM2_LIBRARY}
  ${SIGC++_LIBRARY}
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
  brisk
  catacamera
  fivepoint
  gpl
  sparse_graph
)

endif(EXISTS ${OPENCV_GPU_LIBRARY})
