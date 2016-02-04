camodocal_library(camodocal_DUtilsCV SHARED
  Drawing.cpp
  Geometry.cpp
  GUI.cpp
  IO.cpp
  Mat.cpp
  Transformations.cpp
  Types.cpp
)

camodocal_link_libraries(camodocal_DUtilsCV
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${OpenCV_LIBS}
)

camodocal_install(camodocal_DUtilsCV)
