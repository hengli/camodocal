if(OPENCV_FOUND)
camodocal_library(camodocal_features2d SHARED
  ORBGPU.cc
  SurfGPU.cc
)


camodocal_link_libraries(camodocal_features2d
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_VERSION_SPECIFIC_LIBRARIES}
  ${OPENCV_LIBRARIES}
  ${OPENCV_FEATURES2D_LIBRARY}
  ${Boost_SYSTEM_LIBRARY_RELEASE}
  ${CUDA_CUDART_LIBRARY_OPTIONAL}
  ${Boost_LIBRARIES}
)

camodocal_install(camodocal_features2d)

endif(OPENCV_FOUND)