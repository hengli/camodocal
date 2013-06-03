if(CERES_FOUND)

if(VCHARGE_VIZ)
if(GLUT_FOUND)
include_directories(
  ${GLUT_INCLUDE_DIR}
)

endif(GLUT_FOUND)
endif(VCHARGE_VIZ)

include_directories(
  ../ceres-solver/include
  ../dbow2/DBoW2
  ../dbow2/DUtils
  ../dbow2/DUtils
  ../dbow2/DUtilsCV
  ../dbow2/DVision
)

set(SRCS
  CameraCalibration.cc
  CameraRigBA.cc
  CamOdoCalibration.cc
  CamRigOdoCalibration.cc
  HandEyeCalibration.cc
  utils.cc
)

if(VCHARGE_VIZ)
if(GLUT_FOUND)
set(SRCS ${SRCS} CalibrationWindow.cc)
endif(GLUT_FOUND)
endif(VCHARGE_VIZ)

camodocal_library(camodocal_calib SHARED ${SRCS})

camodocal_link_libraries(camodocal_calib
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${OPENCV_CORE_LIBRARY}
  ${OPENCV_IMGPROC_LIBRARY}
  ${OPENCV_CALIB3D_LIBRARY}
  camodocal_camera_models
  camodocal_camera_systems
  camodocal_DBoW2
  camodocal_DUtilsCV
  camodocal_DUtils
  camodocal_DVision
  camodocal_gpl
  camodocal_sparse_graph
  camodocal_visual_odometry
  ceres
)

if(VCHARGE_VIZ)
if(GLUT_FOUND)
camodocal_link_libraries(camodocal_calib
  ${GLUT_glut_LIBRARY}
  ${GLUT_Xmu_LIBRARY}
  ${GLUT_Xi_LIBRARY} 
  ${OPENGL_gl_LIBRARY}
  ${OPENGL_glu_LIBRARY}
  overlay
)
endif(GLUT_FOUND)
endif(VCHARGE_VIZ)

camodocal_test(CamOdoCalibration)
camodocal_link_libraries(CamOdoCalibration_test camodocal_calib)

camodocal_test(HandEyeCalibration)
camodocal_link_libraries(HandEyeCalibration_test camodocal_calib)

endif(CERES_FOUND)
