 #  Description:
 #    Several functions that help organising the source tree.
 #    [add/set]_source_files - Writes source files to the cache by force and
 #                             adds the current directory.
 #    get_all_header_files- Finds all header files recursively.
 #    generate_source_groups - Set Visual Studio source groups.
 #

# Adds source files with the full path to a list
function(add_source_files _varname)
  # Prefix the full path
  set(_fullpath_sources)
  foreach(_file ${ARGN})
    get_source_file_property(_filepath ${_file} LOCATION)
    list(APPEND _fullpath_sources ${_filepath})
  endforeach(_file)
  # Write into the cache to avoid variable scoping in subdirs
  set(${_varname} ${${_varname}} ${_fullpath_sources} CACHE INTERNAL "Do not edit")
endfunction(add_source_files)


# Sets source files with the full path
function(set_source_files _varname)
  # Prefix the full path
  set(_fullpath_sources)
  foreach(_file ${ARGN})
    get_source_file_property(_filepath ${_file} LOCATION)
    list(APPEND _fullpath_sources ${_filepath})
  endforeach(_file)
  # Write into the cache to avoid variable scoping in subdirs
  set(${_varname} ${_fullpath_sources} CACHE INTERNAL "Do not edit")
endfunction(set_source_files)


# Search the entire directory tree for header files and add them to a variable
macro(get_all_header_files _target_varname)
  file(GLOB_RECURSE ${_target_varname} ${CMAKE_CURRENT_SOURCE_DIR} "*.h")
endmacro(get_all_header_files)


# Generate source groups according to the directory structure
function(generate_source_groups)

  foreach(_file ${ARGN})
    get_source_file_property(_full_filepath ${_file} LOCATION)
    file(RELATIVE_PATH _relative_path ${CMAKE_CURRENT_SOURCE_DIR} ${_full_filepath})
    get_filename_component(_relative_path ${_relative_path} PATH)
    string(REPLACE "/" "\\\\" _group_path "${_relative_path}")
    source_group("Source\\${_group_path}" FILES ${_file})
  endforeach(_file)

endfunction(generate_source_groups)
