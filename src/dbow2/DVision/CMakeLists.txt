include_directories(
  ../DUtils
  ../DUtilsCV
)

if(OpenCV_FOUND)
    
  if(${OpenCV_VERSION} VERSION_LESS 3.0.0)
    # outdated components that don't work in opencv3
    set(DVision_COMPONENTS
      SurfSet.cpp
      Matches.cpp
    )
  endif()
endif()

camodocal_library(camodocal_DVision SHARED
  BRIEF.cpp
  BundleCamera.cpp
  FSolver.cpp
  HSolver.cpp
  ImageFunctions.cpp
  ORB.cpp
  PatchFile.cpp
  PixelPointFile.cpp
  PLYFile.cpp
  PMVSCamera.cpp
  ${DVision_COMPONENTS}
)

camodocal_link_libraries(camodocal_DVision
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_FEATURES2D_LIBRARY}
  ${OPENCV_FLANN_LIBRARY}
  ${OPENCV_IMGPROC_LIBRARY} 
  ${OPENCV_LEGACY_LIBRARY}
  ${OPENCV_NONFREE_LIBRARY}
  camodocal_DUtils
  ${OPENCV_VERSION_SPECIFIC_LIBRARIES}
)
