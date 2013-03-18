cmake_minimum_required(VERSION 2.4.6)

file(GLOB AGAST_SOURCE_FILES  "src/*.cc")

file(GLOB AGAST_HEADER_FILES  "include/agast/*.h")

INCLUDE_DIRECTORIES(include/agast)

#build the library dynamic and static versions
    add_library(agast SHARED ${AGAST_SOURCE_FILES} ${AGAST_HEADER_FILES})
    target_link_libraries(agast ${OPENCV_CORE_LIBRARY}
			${OPENCV_FEATURES2D_LIBRARY}
			${OPENCV_IMGPROC_LIBRARY})
