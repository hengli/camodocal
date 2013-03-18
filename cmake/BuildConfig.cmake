################# Misc Options ##################

# Standard path suffixes, might not hold everywhere though
set(DEFAULT_RUNTIME_PATH bin)
set(DEFAULT_LIBRARY_PATH lib)
set(DEFAULT_ARCHIVE_PATH lib)
set(DEFAULT_DOC_PATH     doc)

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${DEFAULT_RUNTIME_PATH})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${DEFAULT_LIBRARY_PATH})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${DEFAULT_ARCHIVE_PATH})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})

# Check for SSE extensions
include(CheckCXXSourceRuns)
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_REQUIRED_FLAGS "-msse4.1")
  check_cxx_source_runs("
    #include <smmintrin.h>
  
    int main()
    {
       __m128d a, b;
       double vals[2] = {0};
       a = _mm_loadu_pd(vals);
       b = _mm_hadd_pd(a,a);
       _mm_storeu_pd(vals, b);
       return 0;
    }"
    SUPPORTS_SSE41)

  set(CMAKE_REQUIRED_FLAGS "-msse3")
  check_cxx_source_runs("
    #include <pmmintrin.h>
  
    int main()
    {
       __m128d a, b;
       double vals[2] = {0};
       a = _mm_loadu_pd(vals);
       b = _mm_hadd_pd(a,a);
       _mm_storeu_pd(vals, b);
       return 0;
    }"
    SUPPORTS_SSE3)
  
  set(CMAKE_REQUIRED_FLAGS "-msse2")
  check_cxx_source_runs("
    #include <emmintrin.h>
  
    int main()
    {
        __m128d a, b;
        double vals[2] = {0};
        a = _mm_loadu_pd(vals);
        b = _mm_add_pd(a,a);
        _mm_storeu_pd(vals,b);
        return 0;
     }"
     SUPPORTS_SSE2)
  
   set(CMAKE_REQUIRED_FLAGS "-msse")
   check_cxx_source_runs("
    #include <xmmintrin.h>
    int main()
    {
        __m128 a, b;
        float vals[4] = {0};
        a = _mm_loadu_ps(vals);
        b = a;
        b = _mm_add_ps(a,b);
        _mm_storeu_ps(vals,b);
        return 0;
    }"
    SUPPORTS_SSE1)
  
   set(CMAKE_REQUIRED_FLAGS)
endif()

set(SUPPORTS_SSE1 CACHE BOOL "supports SSE1" ${SUPPORTS_SSE1})
set(SUPPORTS_SSE2 CACHE BOOL "supports SSE2" ${SUPPORTS_SSE2})
set(SUPPORTS_SSE3 CACHE BOOL "supports SSE3" ${SUPPORTS_SSE3})
set(SUPPORTS_SSE41 CACHE BOOL "supports SSE4.1" ${SUPPORTS_SSE41})

# Set Debug build to default when not having multi-config generator like msvc
if(NOT CMAKE_CONFIGURATION_TYPES)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING
        "Build types are: Debug, Release, MinSizeRel, RelWithDebInfo" FORCE)
  endif()
  mark_as_advanced(CLEAR CMAKE_BUILD_TYPE)

  message(STATUS "*** Build type is ${CMAKE_BUILD_TYPE} ***")
else()
  if(CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE CACHE STRING FORCE)
  endif()
  mark_as_advanced(CMAKE_BUILD_TYPE)
endif()


################ Compiler Config ################

option(EXTRA_COMPILER_WARNINGS "Enable some extra warnings (heavily pollutes the output)" FALSE)

include(FlagUtilities)

# Configure the compiler specific build options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_GNUC)
  include(BuildConfigGCC)
else()
  message(STATUS "Warning: Your compiler is not officially supported.")
endif()

set(USER_SCRIPT_BUILD_CONFIG "" CACHE FILEPATH
    "Specify a CMake script if you wish to write your own build config.
     See BuildConfigGCC.cmake or BuildConfigMSVC.cmake for examples.")
if(USER_SCRIPT_BUILD_CONFIG)
  if(EXISTS ${CMAKE_MODULE_PATH}/${USER_SCRIPT_BUILD_CONFIG}.cmake)
    include(${USER_SCRIPT_BUILD_CONFIG})
  elseif(EXISTS ${USER_SCRIPT_BUILD_CONFIG})
    include(${USER_SCRIPT_BUILD_CONFIG})
  elseif(EXISTS ${CMAKE_MODULE_PATH}/${USER_SCRIPT_BUILD_CONFIG})
    include(${CMAKE_MODULE_PATH}/${USER_SCRIPT_BUILD_CONFIG})
  endif()
endif(USER_SCRIPT_BUILD_CONFIG)


################# Test options ##################


############# Installation Settings #############

set(_info_text "Puts all installed files in subfolders of the install prefix path. That root folder can then be moved, copied and renamed as you wish. The executable will not write to folders like ~/.camodocal or \"Application Data\"")
if(UNIX)
  option(INSTALL_COPYABLE "${_info_text}" FALSE)
else()
  option(INSTALL_COPYABLE "${_info_text}" TRUE)
endif()

if(INSTALL_COPYABLE)
  # Note the relative paths. They will be resolved at runtime.
  # For CMake operations CMAKE_INSTALL_PREFIX is always appended.
  set(CAMODOCAL_RUNTIME_INSTALL_PATH ${DEFAULT_RUNTIME_PATH})
  set(CAMODOCAL_LIBRARY_INSTALL_PATH ${DEFAULT_LIBRARY_PATH})
  set(CAMODOCAL_ARCHIVE_INSTALL_PATH ${DEFAULT_ARCHIVE_PATH})
  SET(CAMODOCAL_DOC_INSTALL_PATH     ${DEFAULT_DOC_PATH})

elseif(UNIX) # Apple too?
  # Using absolute paths
  set(CAMODOCAL_RUNTIME_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/bin)
  set(CAMODOCAL_LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib/camodocal)
  set(CAMODOCAL_ARCHIVE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib/camodocal/static)
  set(CAMODOCAL_DOC_INSTALL_PATH     ${CMAKE_INSTALL_PREFIX}/share/doc/camodocal)

elseif(WIN32)
  set(CAMODOCAL_RUNTIME_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/${DEFAULT_RUNTIME_PATH})
  set(CAMODOCAL_LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/${DEFAULT_LIBRARY_PATH})
  set(CAMODOCAL_ARCHIVE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/${DEFAULT_ARCHIVE_PATH})
  set(CAMODOCAL_DOC_INSTALL_PATH     ${CMAKE_INSTALL_PREFIX}/${DEFAULT_DOC_PATH})
endif()


################## Unix rpath ###################

# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

# the RPATH to be used when installing
set(CMAKE_INSTALL_RPATH ${CAMODOCAL_LIBRARY_INSTALL_PATH})

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
