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

camodocal_library(camodocal_dvision SHARED
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

camodocal_link_libraries(camodocal_dvision
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${OpenCV_LIBS}
  camodocal_dutils
)

camodocal_install(camodocal_dvision)
