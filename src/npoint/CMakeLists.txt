camodocal_library(fivepoint SHARED
  five-point/five-point.cpp
  five-point/modelest.cpp
)

camodocal_link_libraries(fivepoint
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_CALIB3D_LIBRARY}
)
