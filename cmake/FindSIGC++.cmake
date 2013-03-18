# - Try to find SigC++-2.0
# Once done this will define
#
#  SIGC++_FOUND - system has SigC++
#  SIGC++_INCLUDE_DIR - the SigC++ include directory
#  SIGC++_LIBRARY - SigC++ library

if(SIGC++_INCLUDE_DIR AND SIGC++_LIBRARIES)
    # Already in cache, be silent
    set(SIGC++_FIND_QUIETLY TRUE)
endif(SIGC++_INCLUDE_DIR AND SIGC++_LIBRARIES)

if(WIN32)
    set(_LibSIGC++IncDir $ENV{GTKMM64_BASEPATH}/include)
    set(_LibSIGC++LinkDir $ENV{GTKMM64_BASEPATH}/lib)
else(WIN32)
    include(UsePkgConfig)
    pkgconfig(sigc++-2.0 _LibSIGC++IncDir _LibSIGC++LinkDir _LibSIGC++LinkFlags _LibSIGC++Cflags)
endif(WIN32)

find_path(SIGC++_INCLUDE_DIR sigc++/sigc++.h
          PATH_SUFFIXES sigc++-2.0
          PATHS ${_LibSIGC++IncDir} )

if(WIN32)
    set(SIGC++_LIBRARY_NAME sigc-vc100-2_0)
else(WIN32)
    set(SIGC++_LIBRARY_NAME sigc-2.0)
endif(WIN32)
		  
find_library(SIGC++_LIBRARY 
             NAMES ${SIGC++_LIBRARY_NAME}
             PATHS ${_LibSIGC++LinkDir} )

get_filename_component(sigc++LibDir "${SIGC++_LIBRARY}" PATH)

# Glib-related libraries also use a separate config header, which is in lib dir
find_path(SIGC++_CONFIG_INCLUDE_DIR
  NAMES sigc++config.h
  PATHS ${_LibSIGC++IncDir} ${sigc++LibDir} ${CMAKE_SYSTEM_LIBRARY_PATH} /usr
  PATH_SUFFIXES sigc++-2.0/include lib/sigc++-2.0/include lib/x86_64-linux-gnu/sigc++-2.0/include
)

# not sure if this include dir is optional or required
# for now it is optional
if(SIGC++_CONFIG_INCLUDE_DIR)
  set(SIGC++_INCLUDE_DIR ${SIGC++_INCLUDE_DIR} "${SIGC++_CONFIG_INCLUDE_DIR}" CACHE INTERNAL "List of SigC++-2.0 libraries")
endif(SIGC++_CONFIG_INCLUDE_DIR)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SIGC++  DEFAULT_MSG  SIGC++_LIBRARY SIGC++_INCLUDE_DIR)

mark_as_advanced(SIGC++_INCLUDE_DIR SIGC++_LIBRARY)
