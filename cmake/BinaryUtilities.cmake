include (GenerateExportHeader)

set(__libraries CACHE INTERNAL "list of all libraries")
set(__executables CACHE INTERNAL "list of all executables")
set(__tests CACHE INTERNAL "list of all tests")

function(_camodocal_library_intern _build _name)
  if(_build)
    set(_build TRUE)
  else()
    set(_build FALSE)
  endif()

  if(_build)
    add_library(${_name} STATIC ${ARGN})

	if(CAMODOCAL_PLATFORM_WINDOWS AND BUILD_SHARED_LIBS)
	  # generate corresponding .lib for .dll
	  generate_export_header(${_name}
                             BASE_NAME ${_name}
                             EXPORT_MACRO_NAME ${_name}_EXPORT
                             EXPORT_FILE_NAME ${_name}_Export.h
                             STATIC_DEFINE ${_name}_BUILT_AS_STATIC
	  )
	endif(CAMODOCAL_PLATFORM_WINDOWS AND BUILD_SHARED_LIBS)
	
    if(NOT BUILD_SHARED_LIBS)
      camodocal_install(${_name})
    endif()
  endif()

  string(TOUPPER ${_name} _name_upper)
  set(${_name_upper}_FOUND ${_build} CACHE INTERNAL "TRUE if the library ${_name} will be built.")
  set(${_name_upper}_BUILD ${_build} CACHE INTERNAL "TRUE if the library ${_name} will be built.")

  list(APPEND __libraries ${_name})
  set(__libraries ${__libraries} CACHE INTERNAL "list of all libraries")
endfunction(_camodocal_library_intern)

function(_camodocal_executable_intern _build _name)
  if(_build)
    set(_build TRUE)
  else()
    set(_build FALSE)
  endif()

  if(_build)
    add_executable(${_name} ${ARGN})
    camodocal_install(${_name})
  endif()

  string(TOUPPER ${_name} _name_upper)
  set(${_name_upper}_BUILD ${_build} CACHE INTERNAL "TRUE if the executable ${_name} will be built.")

  list(APPEND __executables ${_name})
  set(__executables ${__executables} CACHE INTERNAL "list of all executables")
endfunction(_camodocal_executable_intern)

function(_camodocal_test_intern _build _name)
  if(_build AND GTEST_FOUND)
    set(_build TRUE)
  else()
    set(_build FALSE)
  endif()

  if(_build)
    add_executable(${_name}_test ${_name}_test.cc ${ARGN})
    target_link_libraries(${_name}_test gtest gtest_main)
    add_test(NAME ${_name}_test
             COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${_name}_test)
    camodocal_install(${_name}_test)
  endif()

  string(TOUPPER ${_name}_test _name_upper)
  set(${_name_upper}_BUILD ${_build} CACHE INTERNAL "TRUE if the test ${_name}_test will be built.")

  list(APPEND __tests ${_name}_test)
  set(__tests ${__tests} CACHE INTERNAL "list of all tests")
endfunction(_camodocal_test_intern)


function(camodocal_library _name)
  _camodocal_library_intern(TRUE ${_name} ${ARGN})
endfunction(camodocal_library)

function(camodocal_executable _name)
  _camodocal_executable_intern(TRUE ${_name} ${ARGN})
endfunction(camodocal_executable)

function(camodocal_test _name)
  _camodocal_test_intern(TRUE ${_name} ${ARGN})
endfunction(camodocal_test)

function(camodocal_link_libraries _name)
  STRING(TOUPPER ${_name} _name_upper)
  if (${_name_upper}_BUILD)
    TARGET_LINK_LIBRARIES(${_name} ${ARGN})
  endif ()
endfunction(camodocal_link_libraries)

function(_camodocal_get_files_and_conditions _conditionvar _filesvar _name)
  set(_firstarg 1)

  set(_condition)
  set(_files)

  set(_parsing_condition FALSE)
  set(_parsing_files FALSE)

  foreach(_arg ${ARGN})
    string(TOUPPER ${_arg} _arg_upper)

    if(_firstarg EQUAL 1 AND NOT _arg_upper STREQUAL "CONDITION" AND NOT _arg_upper STREQUAL "FILES")
      set(_parsing_files TRUE)
    endif()
    set(_firstarg 0)
    if(_arg_upper STREQUAL "FILES")
      set(_parsing_condition FALSE)
    endif()

    if(_parsing_condition)
      if(_arg_upper STREQUAL "TRUE")
        list(APPEND _condition 1)
      elseif(_arg_upper STREQUAL "FALSE")
        list(APPEND _condition 0)
      else()
        list(APPEND _condition ${_arg})
      endif()
    elseif(_parsing_files)
      list(APPEND _files ${_arg})
    endif()

    if(_arg_upper STREQUAL "CONDITION")
      set(_condition)
      set(_parsing_condition TRUE)
    endif()
    if(_arg_upper STREQUAL "FILES")
      set(_files)
      set(_parsing_files TRUE)
    endif()
  endforeach(_arg)

  if(NOT DEFINED _condition)
    set(_condition 1)
  endif()

  set(${_filesvar} ${_files} PARENT_SCOPE)

  if(${_condition})
     set(${_conditionvar} 1 PARENT_SCOPE)
  else()
     set(${_conditionvar} 0 PARENT_SCOPE)
  endif()   

  string(TOUPPER ${_name} _name_upper)
  set(${_name_upper}_CONDITION ${_condition} CACHE INTERNAL "The condition of the binary ${_name}.")
endfunction(_camodocal_get_files_and_conditions)

function(camodocal_library_conditional _name)
  _camodocal_get_files_and_conditions(_condition _files ${_name} ${ARGN})
  _camodocal_library_intern(${_condition} ${_name} ${_files})
endfunction(camodocal_library_conditional _name)

function(camodocal_executable_conditional _name)
  _camodocal_get_files_and_conditions(_condition _files ${_name} ${ARGN})
  _camodocal_EXECUTABLE_INTERN(${_condition} ${_name} ${_files})
endfunction(camodocal_executable_conditional _name)

