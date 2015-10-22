# - Find libnsl
# Find the native libnsl includes and library
#
#  LIBNSL_LIBRARIES    - List of libraries when using libnsl.
#  LIBNSL_FOUND        - True if libnsl found.
#
# The following variables control the behaviour of this module:
#
# LIBNSL_DIR:         Specify a custom directory where suitesparse is located
#                     libraries and headers will be searched for in
#                     ${LIBNSL_DIR}/include and ${LIBNSL_DIR}/lib

find_library( LIBNSL_LIBRARY NAMES libnsl nsl 
        PATHS
	        ${LIBNSL_DIR}/lib/
            ~/.linuxbrew/lib/
	        /usr/local/lib64/
	        /usr/local/lib/
	        /usr/lib/i386-linux-gnu/
	        /usr/lib/x86_64-linux-gnu/
	        /usr/lib64/
	        /usr/lib/
    )
mark_as_advanced( LIBNSL_LIBRARY )

# handle the QUIETLY and REQUIRED arguments and set LIBNSL_FOUND to TRUE if
# all listed variables are TRUE
include( FindPackageHandleStandardArgs )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( LIBNSL DEFAULT_MSG LIBNSL_LIBRARY )

if( LIBNSL_FOUND )
	set( LIBNSL_LIBRARIES ${LIBNSL_LIBRARY} )
endif( LIBNSL_FOUND )