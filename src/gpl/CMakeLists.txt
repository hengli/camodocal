set_source_files(GPL_SRC_FILES
  gpl.cc
)

camodocal_library(gpl SHARED ${GPL_SRC_FILES})
set_target_properties(gpl PROPERTIES COMPILE_FLAGS "-D_REENTRANT")
camodocal_link_libraries(gpl
  ${OPENCV_CORE_LIBRARY}
  rt
)
