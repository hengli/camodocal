# Google Logging
find_library(GLOG_LIB NAMES glog PATHS "/usr/local/lib")

find_path(GLOG_INCLUDE NAMES glog/logging.h PATHS "/usr/local/include")

if(EXISTS ${GLOG_INCLUDE} AND EXISTS ${GLOG_LIB})

option(SCHUR_SPECIALIZATIONS
       "Enable fixed-size schur specializations."
       ON)

if(NOT ${SCHUR_SPECIALIZATIONS})
  add_definitions(-DCERES_RESTRICT_SCHUR_SPECIALIZATION)
  message("-- Disabling Schur specializations (faster compiles)")
endif(NOT ${SCHUR_SPECIALIZATIONS})

if(${OPENMP_FOUND})
  message("-- Found OpenMP.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  add_definitions(-DCERES_USE_OPENMP)
else(${OPENMP_FOUND})
  message("-- Can't find OpenMP. Continuing without it.")
endif(${OPENMP_FOUND})

#if(${PROTOBUF_FOUND})
#  include_directories(${PROTOBUF_INCLUDE_DIRS})
#  include_directories(${CMAKE_CURRENT_BINARY_DIR}/internal)
#else(${PROTOBUF_FOUND})
  add_definitions(-DCERES_NO_PROTOCOL_BUFFERS)
#endif(${PROTOBUF_FOUND})

if(${UNIX})
  # At least on linux, we need pthreads to be enabled for mutex to
  # compile. This may not work on windows or android.
  add_definitions(-DCERES_HAVE_PTHREAD)
  add_definitions(-DCERES_HAVE_RWLOCK)
endif(${UNIX})

# Use the std namespace for the hash<> and related templates. This may vary by
# system.
add_definitions("\"-DCERES_HASH_NAMESPACE_START=namespace std { namespace tr1 {\"")
add_definitions("\"-DCERES_HASH_NAMESPACE_END=}}\"")

include_directories(
  include
  internal
  internal/ceres
  ${GLOG_INCLUDE}
  ${CHOLMOD_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
)

add_subdirectory(internal/ceres)

endif(EXISTS ${GLOG_INCLUDE} AND EXISTS ${GLOG_LIB})
