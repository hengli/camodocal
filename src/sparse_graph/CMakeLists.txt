camodocal_library(sparse_graph SHARED
  Odometer.cc
  SparseGraph.cc
)

camodocal_link_libraries(sparse_graph
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_HIGHGUI_LIBRARY}
  pugixml
)
