camodocal_library(gpl SHARED
  gpl.cc
)

set_target_properties(gpl PROPERTIES COMPILE_FLAGS "-D_REENTRANT")

camodocal_link_libraries(gpl
  ${OPENCV_CORE_LIBRARY}
  rt
)
