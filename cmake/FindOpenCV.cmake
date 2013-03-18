include(FindPackageHandleStandardArgs)
include(HandleLibraryTypes)

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

if(WIN32)

find_library(OPENCV_CALIB3D_LIBRARY
  NAMES opencv_calib3d241
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_CONTRIB_LIBRARY
  NAMES opencv_contrib241
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_CORE_LIBRARY
  NAMES opencv_core241
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_FEATURES2D_LIBRARY
  NAMES opencv_features2d241
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_FLANN_LIBRARY
  NAMES opencv_flann241
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_GPU_LIBRARY
  NAMES opencv_gpu241
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_HIGHGUI_LIBRARY
  NAMES opencv_highgui241
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_IMGPROC_LIBRARY
  NAMES opencv_imgproc241
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_LEGACY_LIBRARY
  NAMES opencv_legacy241
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_ML_LIBRARY
  NAMES opencv_ml241
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_OBJDETECT_LIBRARY
  NAMES opencv_objdetect241
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_VIDEO_LIBRARY
  NAMES opencv_video241
  PATHS ${OpenCV_LibrarySearchPaths}
)

else(WIN32)

find_library(OPENCV_CALIB3D_LIBRARY
  NAMES opencv_calib3d
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_CONTRIB_LIBRARY
  NAMES opencv_contrib
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_CORE_LIBRARY
  NAMES opencv_core
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_FEATURES2D_LIBRARY
  NAMES opencv_features2d
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_FLANN_LIBRARY
  NAMES opencv_flann
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_GPU_LIBRARY
  NAMES opencv_gpu
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_HIGHGUI_LIBRARY
  NAMES opencv_highgui
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_IMGPROC_LIBRARY
  NAMES opencv_imgproc
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_LEGACY_LIBRARY
  NAMES opencv_legacy
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_ML_LIBRARY
  NAMES opencv_ml
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_OBJDETECT_LIBRARY
  NAMES opencv_objdetect
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_VIDEO_LIBRARY
  NAMES opencv_video
  PATHS ${OpenCV_LibrarySearchPaths}
)

find_library(OPENCV_NONFREE_LIBRARY
  NAMES opencv_nonfree
  PATHS ${OpenCV_LibrarySearchPaths}
)

endif(WIN32)

# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
find_package_handle_standard_args(OpenCV "Could NOT find opencv_calib3d (OpenCV)"
  OPENCV_CALIB3D_LIBRARY
)
find_package_handle_standard_args(OpenCV "Could NOT find opencv_contrib (OpenCV)"
  OPENCV_CONTRIB_LIBRARY
)
find_package_handle_standard_args(OpenCV "Could NOT find opencv_core (OpenCV)"
  OPENCV_INCLUDE_DIRS
  OPENCV_CORE_LIBRARY
)
find_package_handle_standard_args(OpenCV "Could NOT find opencv_gpu (OpenCV)"
  OPENCV_GPU_LIBRARY
)
find_package_handle_standard_args(OpenCV "Could NOT find opencv_highgui (OpenCV)"
  OPENCV_HIGHGUI_LIBRARY
)
find_package_handle_standard_args(OpenCV "Could NOT find opencv_imgproc (OpenCV)"
  OPENCV_IMGPROC_LIBRARY
)
find_package_handle_standard_args(OpenCV "Could NOT find opencv_legacy (OpenCV)"
  OPENCV_LEGACY_LIBRARY
)
find_package_handle_standard_args(OpenCV "Could NOT find opencv_ml (OpenCV)"
  OPENCV_ML_LIBRARY
)


# Collect optimized and debug libraries
handle_library_types(OPENCV_CONTRIB_LIBRARY)
handle_library_types(OPENCV_CORE_LIBRARY)
handle_library_types(OPENCV_FEATURES2D_LIBRARY)
handle_library_types(OPENCV_FLANN_LIBRARY)
handle_library_types(OPENCV_GPU_LIBRARY)
handle_library_types(OPENCV_HIGHGUI_LIBRARY)
handle_library_types(OPENCV_IMGPROC_LIBRARY)
handle_library_types(OPENCV_LEGACY_LIBRARY)
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
)

