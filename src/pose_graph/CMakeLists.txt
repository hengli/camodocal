if(OpenCV_FOUND)
include_directories(
  ${CERES_INCLUDE_DIRECTORIES}
  ../dbow2/DBoW2
  ../dbow2/DUtils
  ../dbow2/DUtilsCV
  ../dbow2/DVision
)

camodocal_library(camodocal_pose_graph SHARED
  PoseGraph.cc
)

camodocal_link_libraries(camodocal_pose_graph
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${Boost_THREAD_LIBRARY}
  camodocal_camera_systems
  camodocal_location_recognition
  camodocal_pose_estimation
  ceres
  ${CERES_LIBRARIES}
)

camodocal_install(camodocal_pose_graph)
endif()
