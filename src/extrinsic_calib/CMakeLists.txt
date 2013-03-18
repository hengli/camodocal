include_directories(
  ../ceres-solver/include
  ../dbow2/DBoW2
  ../dbow2/DLoopDetector
  ../dbow2/DUtils
  ../dbow2/DUtilsCV
  ../dbow2/DVision
)

if(CATACAMERA_FOUND)
if(GLUT_FOUND)
if(VISUAL_ODOMETRY_FOUND)

include_directories(
  ${GLUT_INCLUDE_DIR}
)

camodocal_library(cam_odo_cal
  CameraOdometerCalibration.cc
)

camodocal_link_libraries(cam_odo_cal
  ${OPENCV_CORE_LIBRARY}
  catacamera
  ceres
)

camodocal_executable(camera_odometer_calibration
  camera_odometer_calibration.cc
)

camodocal_link_libraries(camera_odometer_calibration
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  cam_odo_cal
)

camodocal_executable(extrinsic_calib
  extrinsic_calib.cc
  CalibrationWindow.cc
  CameraRigBA.cc
  CameraRigCalibration.cc
  utils.cc
)

camodocal_link_libraries(extrinsic_calib
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${OPENCV_FEATURES2D_LIBRARY}
  ${OPENCV_HIGHGUI_LIBRARY}
  ${GLIBMM2_LIBRARY}
  ${GLUT_glut_LIBRARY}
  ${GLUT_Xmu_LIBRARY}
  ${GLUT_Xi_LIBRARY} 
  ${SIGC++_LIBRARY}
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
  ${OPENGL_gl_LIBRARY}
  ${OPENGL_glu_LIBRARY}
  catacamera
  ceres
  DBoW2
  DUtils
  DUtilsCV
  DVision
  feature_tracker
  fivepoint
  gpl
  cam_odo_cal
  overlay
  sparse_graph
  visual_odometry
)

endif(VISUAL_ODOMETRY_FOUND)
endif(GLUT_FOUND)
endif(CATACAMERA_FOUND)
