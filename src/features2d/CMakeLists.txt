if(OpenCV_FOUND AND HAVE_OPENCV_XFEATURES2D_NONFREE)
camodocal_library(camodocal_features2d SHARED
  ORBGPU.cc
  SurfGPU.cc
)


camodocal_link_libraries(camodocal_features2d
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_SYSTEM_LIBRARY_RELEASE}
  ${CUDA_CUDART_LIBRARY_OPTIONAL}
  ${Boost_LIBRARIES}
)

camodocal_install(camodocal_features2d)

endif(OpenCV_FOUND AND HAVE_OPENCV_XFEATURES2D_NONFREE)
