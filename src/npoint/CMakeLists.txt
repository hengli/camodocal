camodocal_library(camodocal_fivepoint SHARED
  five-point/five-point.cpp
  five-point/modelest.cpp
)

camodocal_link_libraries(camodocal_fivepoint
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_CALIB3D_LIBRARY}
)
