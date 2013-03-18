function(handle_library_types _name)
  # Additional libraries can be added as additional arguments
  if(${_name}_LIBRARY_DEBUG AND ${_name}_LIBRARY_OPTIMIZED)
    set(${_name}_LIBRARY
      optimized ${${_name}_LIBRARY_OPTIMIZED} ${ARGN}
      debug     ${${_name}_LIBRARY_DEBUG}     ${ARGN}
      PARENT_SCOPE
    )
  else()
    set(${_name}_LIBRARY
      ${${_name}_LIBRARY_OPTIMIZED} ${ARGN}
      PARENT_SCOPE
     )
  endif()
endfunction(handle_library_types)
