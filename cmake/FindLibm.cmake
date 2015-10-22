# - Find libm
# Find the native libm includes and library
#
#  LIBM_LIBRARIES    - List of libraries when using libm.
#  LIBM_FOUND        - True if libm found.
#
# The following variables control the behaviour of this module:
#
# LIBM_DIR:         Specify a custom directory where suitesparse is located
#                          libraries and headers will be searched for in
#                          ${LIBM_DIR}/include and ${LIBM_DIR}/lib

find_library( LIBM_LIBRARY 
                PATHS 
                   ${LIBM_DIR}/lib
                    ~/.linuxbrew/lib/
        	        /usr/local/lib64/
        	        /usr/local/lib/
        	        /usr/lib/i386-linux-gnu/
        	        /usr/lib/x86_64-linux-gnu/
        	        /usr/lib64/
        	        /usr/lib/
                NAMES m libm )
mark_as_advanced( LIBM_LIBRARY )

# handle the QUIETLY and REQUIRED arguments and set LIBM_FOUND to TRUE if
# all listed variables are TRUE
include( FindPackageHandleStandardArgs )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( LIBM DEFAULT_MSG LIBM_LIBRARY )

if( LIBM_FOUND )
	set( LIBM_LIBRARIES ${LIBM_LIBRARY} )
endif( LIBM_FOUND )