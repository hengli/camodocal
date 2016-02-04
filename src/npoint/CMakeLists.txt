if(OpenCV_FOUND)
camodocal_library(camodocal_fivepoint SHARED
  five-point/five-point.cpp
  five-point/modelest.cpp
)

camodocal_link_libraries(camodocal_fivepoint
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${OpenCV_LIBS}
)

camodocal_install(camodocal_fivepoint)
endif(OpenCV_FOUND)
