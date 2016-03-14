if(OpenCV_FOUND)
include_directories(
  ../dbow2/DBoW2
  ../dbow2/DUtils
  ../dbow2/DUtilsCV
  ../dbow2/DVision
)

camodocal_library(camodocal_location_recognition SHARED
  LocationRecognition.cc
)

camodocal_link_libraries(camodocal_location_recognition
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  camodocal_dbow2
  camodocal_dutilscv
  camodocal_dutils
  camodocal_dvision
  camodocal_sparse_graph
)

camodocal_install(camodocal_location_recognition)
endif(OpenCV_FOUND)