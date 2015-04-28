# - Find libm
# Find the native libm includes and library
#
#  LIBM_LIBRARIES    - List of libraries when using libm.
#  LIBM_FOUND        - True if libm found.

find_library( LIBM_LIBRARY NAMES m libm )
mark_as_advanced( LIBM_LIBRARY )

# handle the QUIETLY and REQUIRED arguments and set LIBM_FOUND to TRUE if
# all listed variables are TRUE
include( FindPackageHandleStandardArgs )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( LIBM DEFAULT_MSG LIBM_LIBRARY )

if( LIBM_FOUND )
	set( LIBM_LIBRARIES ${LIBM_LIBRARY} )
endif( LIBM_FOUND )