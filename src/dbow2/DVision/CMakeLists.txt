include_directories(
  ../DUtils
  ../DUtilsCV
)

camodocal_library(DVision SHARED
  BRIEF.cpp
  BundleCamera.cpp
  FSolver.cpp
  HSolver.cpp
  ImageFunctions.cpp
  Matches.cpp
  ORB.cpp
  PatchFile.cpp
  PixelPointFile.cpp
  PLYFile.cpp
  PMVSCamera.cpp
  SurfSet.cpp
)

camodocal_link_libraries(DVision
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_FEATURES2D_LIBRARY}
  ${OPENCV_FLANN_LIBRARY}
  ${OPENCV_IMGPROC_LIBRARY} 
  ${OPENCV_LEGACY_LIBRARY}
  ${OPENCV_NONFREE_LIBRARY}
  DUtils
)
