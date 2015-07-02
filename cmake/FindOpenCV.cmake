include(FindPackageHandleStandardArgs)
include(HandleLibraryTypes)

# @TODO: Consider using standard find_package(OpenCV) because this will support a broader range of versions.

set(OpenCV_IncludeSearchPaths
  /usr/include/
  /usr/include/opencv-2.3.1
  /usr/local/include/
  /opt/local/include/
  $ENV{OpenCV_DIR}/include
)

set(OpenCV_LibrarySearchPaths
  /usr/lib/
  /usr/local/lib/
  /opt/local/lib/
  $ENV{OpenCV_DIR}/x64/vc10/lib
)

find_path(OPENCV_INCLUDE_DIRS opencv2/opencv.hpp
  PATHS ${OpenCV_IncludeSearchPaths}
)

find_file(OPENCV_CONFIG_VERSION_CMAKE NAMES OpenCVConfig-version.cmake
  PATHS
  /usr/local/share/OpenCV/
  /usr/share/OpenCV/
  /opt/local/share/OpenCV/
  $ENV{OpenCV_DIR}/share/OpenCV
)

if(EXISTS ${OPENCV_CONFIG_VERSION_CMAKE})
    include(${OPENCV_CONFIG_VERSION_CMAKE})
endif()

if(WIN32)
    # this is the suffix of the library in windows
    # @TODO may want to change this to work for more than opencv 2.4.1
    set(OPENCV_LIB_STRING 241)
endif()

find_library(OPENCV_CALIB3D_LIBRARY
  NAMES opencv_calib3d${OPENCV_LIB_STRING}
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_CORE_LIBRARY
  NAMES opencv_core${OPENCV_LIB_STRING}
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_FEATURES2D_LIBRARY
  NAMES opencv_features2d${OPENCV_LIB_STRING}
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_FLANN_LIBRARY
  NAMES opencv_flann${OPENCV_LIB_STRING}
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_GPU_LIBRARY
  NAMES opencv_gpu${OPENCV_LIB_STRING}
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_HIGHGUI_LIBRARY
  NAMES opencv_highgui${OPENCV_LIB_STRING}
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_IMGPROC_LIBRARY
  NAMES opencv_imgproc${OPENCV_LIB_STRING}
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_ML_LIBRARY
  NAMES opencv_ml${OPENCV_LIB_STRING}
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_OBJDETECT_LIBRARY
  NAMES opencv_objdetect${OPENCV_LIB_STRING}
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_VIDEO_LIBRARY
  NAMES opencv_video${OPENCV_LIB_STRING}
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_IMGCODECS_LIBRARY
  NAMES opencv_imgcodecs
  PATHS ${OpenCV_LibrarySearchPaths}
)



# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
find_package_handle_standard_args(OpenCV "Could NOT find opencv_calib3d (OpenCV)"
  OPENCV_CALIB3D_LIBRARY
)
find_package_handle_standard_args(OpenCV "Could NOT find opencv_core (OpenCV)"
  OPENCV_INCLUDE_DIRS
  OPENCV_CORE_LIBRARY
)
find_package_handle_standard_args(OpenCV "Could NOT find opencv_highgui (OpenCV)"
  OPENCV_HIGHGUI_LIBRARY
)
find_package_handle_standard_args(OpenCV "Could NOT find opencv_imgproc (OpenCV)"
  OPENCV_IMGPROC_LIBRARY
)
find_package_handle_standard_args(OpenCV "Could NOT find opencv_ml (OpenCV)"
  OPENCV_ML_LIBRARY
)

message(STATUS "OPENCV_VERSION: ${OpenCV_VERSION} VERSION_LESS 3.0.0")
if(${OpenCV_VERSION} VERSION_LESS 3.0.0)
    # OpenCV2
    message(STATUS "OPENCV_VERSION: ${OpenCV_VERSION} VERSION_LESS 3.0.0 IS_LESS")
    find_library(OPENCV_NONFREE_LIBRARY
      NAMES opencv_nonfree${OPENCV_LIB_STRING}
      PATHS ${OpenCV_LibrarySearchPaths}
    )

    find_library(OPENCV_LEGACY_LIBRARY
      NAMES opencv_legacy${OPENCV_LIB_STRING}
      PATHS ${OpenCV_LibrarySearchPaths}
    )

    find_library(OPENCV_CONTRIB_LIBRARY
      NAMES opencv_contrib${OPENCV_LIB_STRING}
      PATHS ${OpenCV_LibrarySearchPaths}
    )
    
    # Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
    find_package_handle_standard_args(OpenCV "Could NOT find opencv_gpu (OpenCV)"
      OPENCV_GPU_LIBRARY
    )
    find_package_handle_standard_args(OpenCV "Could NOT find opencv_gpu (OpenCV)"
      OPENCV_NONFREE_LIBRARY
    )
    find_package_handle_standard_args(OpenCV "Could NOT find opencv_contrib (OpenCV)"
      OPENCV_CONTRIB_LIBRARY
    )
    find_package_handle_standard_args(OpenCV "Could NOT find opencv_legacy (OpenCV)"
      OPENCV_LEGACY_LIBRARY
    )
    
    # Collect optimized and debug libraries
    handle_library_types(OPENCV_LEGACY_LIBRARY)
    handle_library_types(OPENCV_NONFREE_LIBRARY)
    handle_library_types(OPENCV_CONTRIB_LIBRARY)
    handle_library_types(OPENCV_GPU_LIBRARY)
    
    set(OPENCV_VERSION_SPECIFIC_LIBRARIES 
        ${OPENCV_LEGACY_LIBRARY} 
        ${OPENCV_NONFREE_LIBRARY} 
        ${OPENCV_CONTRIB_LIBRARY} 
        ${OPENCV_GPU_LIBRARY}
    )
else()
    # OpenCV3
    message(STATUS "OPENCV_VERSION: ${OpenCV_VERSION} VERSION_LESS 3.0.0 NOT_LESS")
    find_library(OPENCV_IMGCODECS_LIBRARY
      NAMES opencv_imgcodecs${OPENCV_LIB_STRING}
      PATHS ${OpenCV_LibrarySearchPaths}
    )
    find_library(OPENCV_XFEATURES2D_LIBRARY
      NAMES opencv_xfeatures2d${OPENCV_LIB_STRING}
      PATHS ${OpenCV_LibrarySearchPaths}
    )
    find_library(OPENCV_FEATURES2D_LIBRARY
      NAMES opencv_features2d${OPENCV_LIB_STRING}
      PATHS ${OpenCV_LibrarySearchPaths}
    )
    find_library(OPENCV_CUDAFEATURES2D_LIBRARY
      NAMES opencv_cudafeatures2d${OPENCV_LIB_STRING}
      PATHS ${OpenCV_LibrarySearchPaths}
    )
    
    # Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
    find_package_handle_standard_args(OpenCV "Could NOT find opencv_imgcodecs (OpenCV)"
      OPENCV_IMGCODECS_LIBRARY
    )
    find_package_handle_standard_args(OpenCV "Could NOT find opencv_xfeatures2d (OpenCV)"
      OPENCV_XFEATURES2D_LIBRARY
    )
    find_package_handle_standard_args(OpenCV "Could NOT find opencv_features2d (OpenCV)"
      OPENCV_FEATURES2D_LIBRARY
    )
    find_package_handle_standard_args(OpenCV "Could NOT find opencv_cudafeatures2d (OpenCV)"
      OPENCV_CUDAFEATURES2D_LIBRARY
    )
    
    # Collect optimized and debug libraries
    handle_library_types(OPENCV_IMGCODECS_LIBRARY  )
    handle_library_types(OPENCV_XFEATURES2D_LIBRARY)
    handle_library_types(OPENCV_FEATURES2D_LIBRARY )
    handle_library_types(OPENCV_CUDAFEATURES2D_LIBRARY )
    
    set(OPENCV_VERSION_SPECIFIC_LIBRARIES 
        ${OPENCV_IMGCODECS_LIBRARY}
        ${OPENCV_XFEATURES2D_LIBRARY}
        ${OPENCV_FEATURES2D_LIBRARY}
        ${OPENCV_CUDAFEATURES2D_LIBRARY}
    )
    
endif()

# Collect optimized and debug libraries
handle_library_types(OPENCV_CORE_LIBRARY)
handle_library_types(OPENCV_FEATURES2D_LIBRARY)
handle_library_types(OPENCV_FLANN_LIBRARY)
handle_library_types(OPENCV_HIGHGUI_LIBRARY)
handle_library_types(OPENCV_IMGPROC_LIBRARY)
handle_library_types(OPENCV_ML_LIBRARY)
handle_library_types(OPENCV_OBJDETECT_LIBRARY)
handle_library_types(OPENCV_VIDEO_LIBRARY)

mark_as_advanced(
  OPENCV_INCLUDE_DIRS

  OPENCV_CONTRIB_LIBRARY
  OPENCV_CORE_LIBRARY
  OPENCV_FEATURES2D_LIBRARY
  OPENCV_FLANN_LIBRARY
  OPENCV_GPU_LIBRARY
  OPENCV_HIGHGUI_LIBRARY
  OPENCV_IMGPROC_LIBRARY
  OPENCV_LEGACY_LIBRARY
  OPENCV_ML_LIBRARY
  OPENCV_OBJDETECT_LIBRARY
  OPENCV_VIDEO_LIBRARY
  OPENCV_IMGCODECS_LIBRARY
)

