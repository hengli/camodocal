if(OpenCV_FOUND)
add_subdirectory(DBoW2)
add_subdirectory(DUtils)
add_subdirectory(DUtilsCV)
add_subdirectory(DVision)

include_directories(
  DBoW2
  DUtils
  DUtilsCV
  DVision
)

camodocal_executable(dbow2_demo
  demo.cpp
)

camodocal_link_libraries(dbow2_demo
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  camodocal_dbow2
  camodocal_dutils
  camodocal_dutilscv
  camodocal_dvision
)
endif()
