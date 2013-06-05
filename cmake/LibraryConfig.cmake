include(CompareVersionStrings)
include(DependencyUtilities)
include(FindPackageHandleStandardArgs)

# Prevent CMake from finding libraries in the installation folder on Windows.
# There might already be an installation from another compiler
if(DEPENDENCY_PACKAGE_ENABLE)
  list(REMOVE_ITEM CMAKE_SYSTEM_PREFIX_PATH  "${CMAKE_INSTALL_PREFIX}")
  list(REMOVE_ITEM CMAKE_SYSTEM_LIBRARY_PATH "${CMAKE_INSTALL_PREFIX}/bin")
endif()

############### Library finding #################
# Performs the search and sets the variables    #
camodocal_required_dependency(BLAS)
camodocal_required_dependency(CUDA)
camodocal_required_dependency(Eigen3)
camodocal_required_dependency(GLIB2)
camodocal_required_dependency(GLIBMM2)
camodocal_required_dependency(GTHREAD2)
camodocal_required_dependency(LAPACK)
camodocal_required_dependency(OpenCV)
camodocal_required_dependency(SIGC++)
camodocal_required_dependency(SuiteSparse)

camodocal_optional_dependency(GTest)
camodocal_optional_dependency(OpenMP)

##### Boost #####
# Expand the next statement if newer boost versions than 1.40.0 are released
set(Boost_ADDITIONAL_VERSIONS "1.40" "1.40.0" "1.49" "1.49.0")

find_package(Boost 1.40 REQUIRED COMPONENTS filesystem program_options serialization system thread)

# MSVC seems to be the only compiler requiring date_time
if(MSVC)
  find_package(Boost 1.40 REQUIRED date_time)
endif(MSVC)

# No auto linking, so this option is useless anyway
mark_as_advanced(Boost_LIB_DIAGNOSTIC_DEFINITIONS)
