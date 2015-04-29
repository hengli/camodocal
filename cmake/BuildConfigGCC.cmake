# Determine compiler version
execute_process(
  COMMAND ${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_ARG1} -dumpversion
  OUTPUT_VARIABLE GCC_VERSION
)

# GCC may not support #pragma GCC system_header correctly when using
# templates. According to Bugzilla, it was fixed March 07 but tests
# have confirmed that GCC 4.0.0 does not pose a problem for our cases.
include(CompareVersionStrings)
compare_version_strings("${GCC_VERSION}" "4.0.0" _compare_result)
if(_compare_result LESS 0)
#if("${GCC_VERSION}" VERSION_LESS "4.0.0")
  set(GCC_NO_SYSTEM_HEADER_SUPPORT TRUE)
endif()

# Also include environment flags. Could cause conflicts though
set_compiler_flags("$ENV{CXXFLAGS}" CXX CACHE)
set_compiler_flags("$ENV{CFLAGS}"   C   CACHE)

# Include supported SSE extensions
if(SUPPORTS_SSE41)
  set(SSE_FLAGS "-msse4.1 -mfpmath=sse")
  message(STATUS "Found SSE4.1 extensions, using flags: ${SSE_FLAGS}")
elseif(SUPPORTS_SSE3)
  set(SSE_FLAGS "-msse3 -mfpmath=sse")
  message(STATUS "Found SSE3 extensions, using flags: ${SSE_FLAGS}")
elseif(SUPPORTS_SSE2)
  set(SSE_FLAGS "-msse2 -mfpmath=sse")
  message(STATUS "Found SSE2 extensions, using flags: ${SSE_FLAGS}")
elseif(SUPPORTS_SSE1)
  set(SSE_FLAGS "-msse -mfpmath=sse")
  message(STATUS "Found SSE1 extensions, using flags: ${SSE_FLAGS}")
endif()

add_compiler_flags("${SSE_FLAGS}" CACHE)

# These flags get added to the flags above
if(APPLE)
add_compiler_flags("    -g -ggdb -D_DEBUG -march=core2" Debug          CACHE)
add_compiler_flags("             -DNDEBUG -march=core2" ReleaseAll     CACHE)
add_compiler_flags("-O2                   -march=core2" Release        CACHE)
add_compiler_flags("-O2 -g -ggdb          -march=core2" RelWithDebInfo CACHE)
add_compiler_flags("-Os                   -march=core2" MinSizeRel     CACHE)
else(APPLE)
add_compiler_flags("    -g -ggdb -D_DEBUG -march=native" Debug          CACHE)
add_compiler_flags("             -DNDEBUG -march=native" ReleaseAll     CACHE)
add_compiler_flags("-O2                   -march=native" Release        CACHE)
add_compiler_flags("-O2 -g -ggdb          -march=native" RelWithDebInfo CACHE)
add_compiler_flags("-Os                   -march=native" MinSizeRel     CACHE)
endif(APPLE)

add_compiler_flags("-fpermissive" CACHE)
add_compiler_flags("-std=c++11" CACHE)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "i686" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
# add_compiler_flags("-mtune=pentium4 -march=pentium4 -ftree-vectorize -msse2 -ffast-math -fexpensive-optimizations -fomit-frame-pointer -funroll-loops" Release CACHE)
  add_compiler_flags("-ftree-vectorize -ftree-vectorizer-verbose=0" Release CACHE)
  
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
  add_compiler_flags("-mtune=cortex-a8 -march=armv7-a -ftree-vectorize -mfpu=neon -mfloat-abi=softfp -fexpensive-optimizations -fomit-frame-pointer -funroll-loops -ftree-vectorizer-verbose=1" Release CACHE)
endif()

# CMake doesn't seem to set the PIC flags right on certain 64 bit systems
if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  add_compiler_flags("-fPIC" CACHE)
endif()

# We have some uncoformant code, disable an optimisation feature
#remove_compiler_flags("-fno-strict-aliasing" CACHE)

# For GCC older than version 4, do not display sign compare warnings
# because of boost::filesystem (which creates about a hundred per include)
#add_compiler_flags("-Wno-sign-compare" GCC_NO_SYSTEM_HEADER_SUPPORT CACHE)

# For newer GCC (4.3 and above), don't display hundreds of annoying deprecated
# messages. Other versions don't seem to show any such warnings at all.
add_compiler_flags("-Wno-deprecated" CXX CACHE)

# avoid g++ internal errors which often occur with a high number of inlines
add_compiler_flags("-finline-limit=400" CXX CACHE)

# Increase warning level if requested
if(EXTRA_COMPILER_WARNINGS)
  add_compiler_flags("-Wall -Wextra -Wno-unused-parameter" CACHE)
else()
  remove_compiler_flags("-Wextra -Wno-unused-parameter" CACHE)
  add_compiler_flags("-Wall" CACHE)
endif()
