# - Find libnsl
# Find the native libnsl includes and library
#
#  LIBNSL_LIBRARIES    - List of libraries when using libnsl.
#  LIBNSL_FOUND        - True if libnsl found.

find_library( LIBNSL_LIBRARY NAMES libnsl nsl )
mark_as_advanced( LIBNSL_LIBRARY )

# handle the QUIETLY and REQUIRED arguments and set LIBNSL_FOUND to TRUE if
# all listed variables are TRUE
include( FindPackageHandleStandardArgs )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( LIBNSL DEFAULT_MSG LIBNSL_LIBRARY )

if( LIBNSL_FOUND )
	set( LIBNSL_LIBRARIES ${LIBNSL_LIBRARY} )
endif( LIBNSL_FOUND )