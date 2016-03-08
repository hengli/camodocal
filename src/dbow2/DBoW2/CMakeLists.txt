include_directories(
  ../DUtils
  ../DVision
)

camodocal_library(camodocal_DBoW2 SHARED
  BowVector.cpp
  FBrief.cpp
  FeatureVector.cpp
  FOrb.cpp
  FSurf64.cpp
  QueryResults.cpp
  ScoringObject.cpp
)

camodocal_link_libraries(camodocal_DBoW2
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  camodocal_dutils
  camodocal_dutilscv
  camodocal_dvision
)

camodocal_install(camodocal_DBoW2)