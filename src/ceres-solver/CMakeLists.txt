if(NOT CERES_FOUND)
if(GLOG_FOUND)
    # Google Logging
    set(GLOG_LIB ${GLOG_LIBRARIES})
    set(GLOG_INCLUDE ${GLOG_INCLUDE_DIRS})
    
    INCLUDE(CheckIncludeFileCXX)
    CHECK_INCLUDE_FILE_CXX(unordered_map HAVE_STD_UNORDERED_MAP_HEADER)
    IF (HAVE_STD_UNORDERED_MAP_HEADER)
      # Finding the unordered_map header doesn't mean that unordered_map
      # is in std namespace.
      #
      # In particular, MSVC 2008 has unordered_map declared in std::tr1.
      # In order to support this, we do an extra check to see which
      # namespace should be used.
      INCLUDE(CheckCXXSourceCompiles)
      CHECK_CXX_SOURCE_COMPILES("#include <unordered_map>
                                 int main() {
                                   std::unordered_map<int, int> map;
                                   return 0;
                                 }"
                                HAVE_UNORDERED_MAP_IN_STD_NAMESPACE)
      IF (HAVE_UNORDERED_MAP_IN_STD_NAMESPACE)
        LIST(APPEND CERES_COMPILE_OPTIONS CERES_STD_UNORDERED_MAP)
        MESSAGE("-- Found unordered_map/set in std namespace.")
      ELSE (HAVE_UNORDERED_MAP_IN_STD_NAMESPACE)
        CHECK_CXX_SOURCE_COMPILES("#include <unordered_map>
                                   int main() {
                                     std::tr1::unordered_map<int, int> map;
                                     return 0;
                                   }"
                                  HAVE_UNORDERED_MAP_IN_TR1_NAMESPACE)
        IF (HAVE_UNORDERED_MAP_IN_TR1_NAMESPACE)
          LIST(APPEND CERES_COMPILE_OPTIONS CERES_STD_UNORDERED_MAP_IN_TR1_NAMESPACE)
          MESSAGE("-- Found unordered_map/set in std::tr1 namespace.")
        ELSE (HAVE_UNORDERED_MAP_IN_TR1_NAMESPACE)
          MESSAGE("-- Found <unordered_map> but cannot find either std::unordered_map "
                  "or std::tr1::unordered_map.")
          MESSAGE("-- Replacing unordered_map/set with map/set (warning: slower!)")
          LIST(APPEND CERES_COMPILE_OPTIONS CERES_NO_UNORDERED_MAP)
        ENDIF (HAVE_UNORDERED_MAP_IN_TR1_NAMESPACE)
      ENDIF (HAVE_UNORDERED_MAP_IN_STD_NAMESPACE)
    ELSE (HAVE_STD_UNORDERED_MAP_HEADER)
      CHECK_INCLUDE_FILE_CXX("tr1/unordered_map" HAVE_TR1_UNORDERED_MAP_HEADER)
      IF (HAVE_TR1_UNORDERED_MAP_HEADER)
        LIST(APPEND CERES_COMPILE_OPTIONS CERES_TR1_UNORDERED_MAP)
        MESSAGE("-- Found tr1/unordered_map/set in std::tr1 namespace.")
      ELSE (HAVE_TR1_UNORDERED_MAP_HEADE)
        MESSAGE("-- Unable to find <unordered_map> or <tr1/unordered_map>. ")
        MESSAGE("-- Replacing unordered_map/set with map/set (warning: slower!)")
        LIST(APPEND CERES_COMPILE_OPTIONS CERES_NO_UNORDERED_MAP)
      ENDIF (HAVE_TR1_UNORDERED_MAP_HEADER)
    ENDIF (HAVE_STD_UNORDERED_MAP_HEADER)

    foreach(CURRENT_CERES_OPTION IN LISTS CERES_COMPILE_OPTIONS)
        add_definitions(-D${CURRENT_CERES_OPTION})
    endforeach()

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

if(${TBB_FOUND})
    set(TBB_LBRARIES_OPTIONAL ${TBB_LIBRARIES})
endif()

# Use the std namespace for the hash<> and related templates. This may vary by
# system.
#add_definitions("\"-DCERES_HASH_NAMESPACE_START=namespace std { namespace tr1 {\"")
#add_definitions("\"-DCERES_HASH_NAMESPACE_END=}}\"")

include_directories(
  include
  internal
  internal/ceres
  ${GLOG_INCLUDE_DIRS}
  ${CHOLMOD_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
)

add_subdirectory(internal/ceres)

endif(GLOG_FOUND)
endif(NOT CERES_FOUND)
