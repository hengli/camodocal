if(CAMODOCAL_CAMERA_MODELS_FOUND AND CAMODOCAL_FEATURES2D_FOUND AND HAVE_OPENCV_XFEATURES2D_NONFREE)

include_directories(
  ${CERES_INCLUDE_DIRECTORIES}
  ../brisk/thirdparty/agast/include
)

camodocal_library(camodocal_visual_odometry SHARED
  FeatureTracker.cc
  SlidingWindowBA.cc
)

camodocal_link_libraries(camodocal_visual_odometry
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${Boost_THREAD_LIBRARY}
  ${OpenCV_LIBS}
  camodocal_brisk
  camodocal_camera_models
  camodocal_features2d
  camodocal_fivepoint
  camodocal_gpl
  camodocal_pose_estimation
  camodocal_sparse_graph
  ${CERES_LIBRARIES}
)

camodocal_test(SlidingWindowBA)
camodocal_link_libraries(SlidingWindowBA_test ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES} camodocal_gpl camodocal_visual_odometry)

if(VCHARGE_VIZ)

camodocal_executable(SlidingWindowBA_sim
  SlidingWindowBA_sim.cc
)

camodocal_link_libraries(SlidingWindowBA_sim
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}    
  camodocal_visual_odometry
  middleware
  overlay
) 

camodocal_install(SlidingWindowBA_sim)

endif(VCHARGE_VIZ)

camodocal_install(camodocal_visual_odometry)

endif()
