if(OpenCV_FOUND)
camodocal_library(camodocal_dutilscv SHARED
  Drawing.cpp
  Geometry.cpp
  GUI.cpp
  IO.cpp
  Mat.cpp
  Transformations.cpp
  Types.cpp
)

camodocal_link_libraries(camodocal_dutilscv
  ${CAMODOCAL_PLATFORM_UNIX_LIBRARIES}
  ${OpenCV_LIBS}
)

camodocal_install(camodocal_dutilscv)
endif()
