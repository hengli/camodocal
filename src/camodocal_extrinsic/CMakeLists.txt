if(VISUAL_ODOMETRY_FOUND)

include_directories(
  ${GLUT_INCLUDE_DIR}
  ../ceres-solver/include
  ../dbow2/DBoW2
  ../dbow2/DLoopDetector
  ../dbow2/DUtils
  ../dbow2/DUtilsCV
  ../dbow2/DVision
)

camodocal_library(cam_odo_calibration
  CamOdoCalibration.cc
)

camodocal_link_libraries(cam_odo_calibration
  ${OPENCV_CORE_LIBRARY}
  catacamera
  ceres
)

camodocal_library(camodocal_extrinsic
  CameraRigBA.cc
  CamRigOdoCalibration.cc
  utils.cc
)

camodocal_link_libraries(camodocal_extrinsic
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
  cam_odo_calibration
  catacamera
  ceres
  DBoW2
  DUtils
  DUtilsCV
  DVision
  fivepoint
  gpl
  sparse_graph
  visual_odometry
)

endif(VISUAL_ODOMETRY_FOUND)
