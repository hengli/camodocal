if(OpenCV_FOUND)
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
  ${OpenCV_LIBS}
)

camodocal_install(camodocal_sparse_graph)
endif(OpenCV_FOUND)
