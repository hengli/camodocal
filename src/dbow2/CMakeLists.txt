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
  camodocal_DBoW2
  camodocal_DUtils
  camodocal_DUtilsCV
  camodocal_DVision
)
