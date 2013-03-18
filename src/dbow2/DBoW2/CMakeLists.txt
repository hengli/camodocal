include_directories(
  ../DUtils
  ../DVision
)

camodocal_library(DBoW2 SHARED
  BowVector.cpp
  FBrief.cpp
  FeatureVector.cpp
  FOrb.cpp
  FSurf64.cpp
  QueryResults.cpp
  ScoringObject.cpp
)

camodocal_link_libraries(DBoW2
  DUtils
  DUtilsCV
  DVision
)
