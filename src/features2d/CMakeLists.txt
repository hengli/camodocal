if(EXISTS ${OPENCV_GPU_LIBRARY})

camodocal_library(camodocal_features2d SHARED
  ORBGPU.cc
  SurfGPU.cc
)

camodocal_link_libraries(camodocal_features2d
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_NONFREE_LIBRARY}
  ${OPENCV_GPU_LIBRARY}
)

endif(EXISTS ${OPENCV_GPU_LIBRARY})
