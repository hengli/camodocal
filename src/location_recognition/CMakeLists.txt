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
  camodocal_DBoW2
  camodocal_DUtilsCV
  camodocal_DUtils
  camodocal_DVision
  camodocal_sparse_graph
)
