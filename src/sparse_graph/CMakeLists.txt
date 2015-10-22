if(OPENCV_FOUND)
camodocal_library(camodocal_sparse_graph SHARED
  Odometry.cc
  Pose.cc
  SparseGraph.cc
  SparseGraphUtils.cc
  Transform.cc
)

camodocal_link_libraries(camodocal_sparse_graph
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_FEATURES2D_LIBRARY}
  ${OPENCV_HIGHGUI_LIBRARY}
  ${OPENCV_VERSION_SPECIFIC_LIBRARIES}
)

camodocal_install(camodocal_sparse_graph)
endif(OPENCV_FOUND)