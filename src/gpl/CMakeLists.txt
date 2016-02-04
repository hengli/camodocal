if(OpenCV_FOUND)
include_directories(
  ${CERES_INCLUDE_DIRECTORIES}
)

set_source_files(GPL_SRC_FILES
  CubicSpline.cc
  EigenQuaternionParameterization.cc
  gpl.cc
)

camodocal_library(camodocal_gpl SHARED ${GPL_SRC_FILES})
camodocal_link_libraries(camodocal_gpl
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${OpenCV_LIBS}
  ceres
  ${CERES_LIBRARIES}
)

camodocal_install(camodocal_gpl)
endif(OpenCV_FOUND)
