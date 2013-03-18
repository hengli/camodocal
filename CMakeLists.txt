cmake_minimum_required(VERSION 2.8.6 FATAL_ERROR)

project(CAMODOCAL C CXX)

# This sets where to look for modules (e.g. "Find*.cmake" files)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# Library finding
include(LibraryConfig)

# General build and compiler options and configurations
include(BuildConfig)

# PRINT macro
macro(print)
  message(STATUS ${ARGN})
endmacro(print)

# Last but not least: Try to make doxygen target
add_subdirectory(cmake/doc)

# Output
set(_output_blanks "                              ")

print("")
print("---------------------------------------------------")
print("DEPENDENCY:                 Required:    Found:")
print("---------------------------------------------------")
foreach(_name ${__required_dependencies})
  string(TOUPPER ${_name} _name_upper)
  string(LENGTH ${_name} _name_length)
  math(EXPR _name_length "30 - ${_name_length}")
  string(SUBSTRING ${_output_blanks} 0 ${_name_length} _blanks)
  if(${_name_upper}_FOUND OR
     ${_name_upper}_INCLUDE_DIR OR
     ${_name_upper}_INCLUDE_DIRS OR
     ${_name_upper}_LIBRARY OR
     ${_name_upper}_LIBRARY_OPTIMIZED OR
     ${_name_upper}_LIBRARIES OR
     ${_name_upper}_LIBRARIES_OPTIMIZED)
    set(_found " yes")
  else()
    set(_found " no")
  endif()
  print("${_name}${_blanks} yes        ${_found}")
endforeach(_name)
foreach(_name ${__optional_dependencies})
  string(TOUPPER ${_name} _name_upper)
  string(LENGTH ${_name} _name_length)
  math(EXPR _name_length "30 - ${_name_length}")
  string(SUBSTRING ${_output_blanks} 0 ${_name_length} _blanks)
  if(${_name_upper}_FOUND OR
     ${_name_upper}_INCLUDE_DIR OR
     ${_name_upper}_INCLUDE_DIRS OR
     ${_name_upper}_LIBRARY OR
     ${_name_upper}_LIBRARY_OPTIMIZED OR
     ${_name_upper}_LIBRARIES OR
     ${_name_upper}_LIBRARIES_OPTIMIZED)
    set(_found " yes")
  else()
    set(_found " no")
  endif()
  print("${_name}${_blanks} no         ${_found}")
endforeach(_name)

print("")
print("---------------------------------------------------")
print("Library:                      Build:    Condition:")
print("---------------------------------------------------")
foreach(_name ${__libraries})
  string(TOUPPER ${_name} _name_upper)
  string(LENGTH ${_name} _name_length)
  math(EXPR _name_length "30 - ${_name_length}")
  string(SUBSTRING ${_output_blanks} 0 ${_name_length} _blanks)
  if(NOT ${_name_upper}_BUILD)
    set(_condition ${${_name_upper}_CONDITION})
  else()
    set(_condition " (satisfied)")
  endif()
  print("${_name}${_blanks}${${_name_upper}_BUILD}     ${_condition}")
endforeach(_name)

print("")
print("---------------------------------------------------")
print("Executable:                   Build:    Condition:")
print("---------------------------------------------------")
foreach(_name ${__executables})
  string(TOUPPER ${_name} _name_upper)
  string(LENGTH ${_name} _name_length)
  math(EXPR _name_length "30 - ${_name_length}")
  string(SUBSTRING ${_output_blanks} 0 ${_name_length} _blanks)
  if(NOT ${_name_upper}_BUILD)
    set(_condition ${${_name_upper}_CONDITION})
  else()
    set(_condition " (satisfied)")
  endif()
  print("${_name}${_blanks}${${_name_upper}_BUILD}     ${_condition}")
endforeach(_name)

print("")
print("---------------------------------------------------")
print("Test:                         Build:    Condition:")
print("---------------------------------------------------")
foreach(_name ${__tests})
  string(TOUPPER ${_name} _name_upper)
  string(LENGTH ${_name} _name_length)
  math(EXPR _name_length "30 - ${_name_length}")
  string(SUBSTRING ${_output_blanks} 0 ${_name_length} _blanks)
  if(NOT ${_name_upper}_BUILD)
    set(_condition ${${_name_upper}_CONDITION})
  else()
    set(_condition " (satisfied)")
  endif()
  print("${_name}${_blanks}${${_name_upper}_BUILD}     ${_condition}")
endforeach(_name)

print("")

################ Various Options ################

# various macro includes
include(FlagUtilities)
include(InstallUtilities)
include(SourceFileUtilities)
include(BinaryUtilities)

############## CamOdoCalConfig.h ##############

# Check endianness
include(TestBigEndian)
test_big_endian(CAMODOCAL_BIG_ENDIAN)
if(NOT CAMODOCAL_BIG_ENDIAN)
  set(CAMODOCAL_LITTLE_ENDIAN TRUE)
endif()

# 32/64 bit system check
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(CAMODOCAL_ARCH_64 TRUE)
else()
  set(CAMODOCAL_ARCH_32 TRUE)
endif()

# Platforms
set(CAMODOCAL_PLATFORM_WINDOWS ${WIN32})
set(CAMODOCAL_PLATFORM_APPLE ${APPLE})
set(CAMODOCAL_PLATFORM_UNIX ${UNIX})
if(UNIX AND NOT APPLE)
  set(CAMODOCAL_PLATFORM_LINUX TRUE)
endif()

set(BUILD_SHARED_LIBS ON)

# Check __forceinline
if(MSVC)
  include(CheckCXXSourceCompiles)
  set(_source "int main() { return 0; } __forceinline void test() { return; }")
  check_cxx_source_compiles("${_source}" HAVE_FORCEINLINE)
endif(MSVC)

# Check ciso646 include (literal operators)
include(CheckIncludeFileCXX)
check_include_file_cxx(iso646.h HAVE_ISO646_H)

# XCode and Visual Studio support multiple configurations. In order to tell the
# which one we have to define the macros separately for each configuration
add_compiler_flags("-DCMAKE_BUILD_TYPE=Debug"          Debug)
add_compiler_flags("-DCMAKE_BUILD_TYPE=Release"        Release)
add_compiler_flags("-DCMAKE_BUILD_TYPE=RelWithDebInfo" RelWithDebInfo)
add_compiler_flags("-DCMAKE_BUILD_TYPE=MinSizeRel"     MinSizeRel)

set(GENERATED_FILE_COMMENT
   "DO NOT EDIT THIS FILE!
    It has been automatically generated by CMake from CamOdoCalConfig.h.in")
# Copy and configure CamOdoCalConfig which gets included in every file
configure_file(CamOdoCalConfig.h.in ${CMAKE_CURRENT_BINARY_DIR}/CamOdoCalConfig.h)
configure_file(CamOdoCalPathConfig.h.in ${CMAKE_CURRENT_BINARY_DIR}/CamOdoCalPathConfig.h)

############## Include Directories ##############

# Set the search paths for include files
include_directories(
  # External
  ${EIGEN3_INCLUDE_DIR}
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GLIBMM2_MAIN_INCLUDE_DIR}
  ${GLIBMM2_INTERNAL_INCLUDE_DIR}
  ${SIGC++_INCLUDE_DIR}
  ${OPENCV_INCLUDE_DIRS}
  ${RTI_INCLUDE_DIR}
  ${RTI_INCLUDE_DIR}/ndds
  external/adtf/include

  # All library includes are prefixed with the path to avoid conflicts
  ${CMAKE_CURRENT_SOURCE_DIR}

  # testing
  ${GTEST_INCLUDE_DIRS}

  # CamOdoCalConfig.h
  ${CMAKE_CURRENT_BINARY_DIR}
)

if(BUILD_SHARED_LIBS)
  add_definitions(-DCAMODOCAL_EXPORTS)
  if(CAMODOCAL_PLATFORM_WINDOWS)
    add_definitions(-DNDDS_USER_DLL_EXPORT)
  endif(CAMODOCAL_PLATFORM_WINDOWS)
endif(BUILD_SHARED_LIBS)

if(CAMODOCAL_PLATFORM_WINDOWS)
  add_definitions(-DRTI_WIN32 -DNDDS_DLL_VARIABLE)
  # Windows 64-bit
  set(CMAKE_CXX_FLAGS "/MD")
elseif(CAMODOCAL_PLATFORM_UNIX)
  add_definitions(-DRTI_UNIX)
  if(CAMODOCAL_ARCH_32)
    # Linux 32-bit
    set(CMAKE_CXX_FLAGS "-m32")
  else(CAMODOCAL_ARCH_32)
    # Linux 64-bit
    set(CMAKE_CXX_FLAGS "-m64")
  endif(CAMODOCAL_ARCH_32)
endif()

# General linker flags
if(MSVC)
  if(NOT CMAKE_CXX_FLAGS MATCHES "/EHsc")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc") 
  endif()
endif(MSVC)
if(CAMODOCAL_PLATFORM_UNIX)
  set_linker_flags("-Wl,--no-undefined -ldl -lnsl -lm -pthread -lrt" CACHE)
endif(CAMODOCAL_PLATFORM_UNIX)
  
if(PROTOBUF_FOUND)
  include_directories(${PROTOBUF_INCLUDE_DIR})
  add_definitions(-DPROTOBUF_FOUND)
endif()

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
  add_definitions(-DBOOST_ALL_DYN_LINK)
endif()

link_directories(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

# testing
enable_testing()

################ Sub Directories ################

# Global includes
include_directories(
  include
)

# CamOdoCal code
add_subdirectory(src)
