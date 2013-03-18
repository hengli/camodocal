# - Try to find Glibmm-2.4
# Once done this will define
#
#  GLIBMM2_FOUND - system has glibmm2
#  GLIBMM2_INCLUDE_DIR - the glibmm2 include directory
#  GLIBMM2_LIBRARY - glibmm2 library

if(GLIBMM2_INCLUDE_DIR AND GLIBMM2_LIBRARIES)
    # Already in cache, be silent
    set(GLIBMM2_FIND_QUIETLY TRUE)
endif(GLIBMM2_INCLUDE_DIR AND GLIBMM2_LIBRARIES)

if(WIN32)
    set(_LibGLIBMM2IncDir $ENV{GTKMM64_BASEPATH}/include)
    set(_LibGLIBMM2LinkDir $ENV{GTKMM64_BASEPATH}/lib)
else(WIN32)
    include(UsePkgConfig)
    pkgconfig(glibmm-2.4 _LibGLIBMM2IncDir _LibGLIBMM2LinkDir _LibGLIBMM2LinkFlags _LibGLIBMM2Cflags)
endif(WIN32)

find_path(GLIBMM2_MAIN_INCLUDE_DIR glibmm.h
          PATH_SUFFIXES glibmm-2.4
          PATHS ${_LibGLIBMM2IncDir} )

# search the glibconfig.h include dir under the same root where the library is found

if(WIN32)
    set(GLIBMM2_LIBRARY_NAME glibmm-vc100-2_4)
else(WIN32)
    set(GLIBMM2_LIBRARY_NAME glibmm-2.4)
endif(WIN32)

find_library(GLIBMM2_LIBRARY 
             NAMES ${GLIBMM2_LIBRARY_NAME}
             PATHS ${_LibGLIBMM2LinkDir} )

get_filename_component(glibmm2LibDir "${GLIBMM2_LIBRARY}" PATH)

find_path(GLIBMM2_INTERNAL_INCLUDE_DIR glibmmconfig.h
          PATH_SUFFIXES glibmm-2.4/include
          PATHS ${_LibGLIBMM2IncDir} ${glibmm2LibDir} ${CMAKE_SYSTEM_LIBRARY_PATH})

set(GLIBMM2_INCLUDE_DIR "${GLIBMM2_MAIN_INCLUDE_DIR}")

# not sure if this include dir is optional or required
# for now it is optional
if(GLIBMM2_INTERNAL_INCLUDE_DIR)
  set(GLIBMM2_INCLUDE_DIR ${GLIBMM2_INCLUDE_DIR} "${GLIBMM2_INTERNAL_INCLUDE_DIR}")
endif(GLIBMM2_INTERNAL_INCLUDE_DIR)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLIBMM2  DEFAULT_MSG  GLIBMM2_LIBRARY GLIBMM2_MAIN_INCLUDE_DIR)

mark_as_advanced(GLIBMM2_INCLUDE_DIR GLIBMM2_LIBRARY)
