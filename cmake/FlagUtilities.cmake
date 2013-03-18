# Write to the cache by force, but only if the user didn't edit the value
# Additional argument is the value (may also be a list)
macro(set_cache _varname _type _docstring)
  set(_value ${ARGN})
  if(NOT "${_type}" MATCHES "^(STRING|BOOL|PATH|FILEPATH)$")
    message(FATAL_ERROR "${_type} is not a valid CACHE entry type")
  endif()

  if(NOT DEFINED _INTERNAL_${_varname} OR "${_INTERNAL_${_varname}}" STREQUAL "${${_varname}}")
    set(${_varname} "${_value}" CACHE ${_type} "${_docstring}" FORCE)
    set(_INTERNAL_${_varname} "${_value}" CACHE INTERNAL "Do not edit in any case!")
  endif()
endmacro(set_cache)

# Visual studio (esp. IntelliSense) doesn't like dashes to specify arguments
# Always use foward slashes instead
if(MSVC)
  set(ARGUMENT_STARTER "/")
else()
  set(ARGUMENT_STARTER "-")
endif(MSVC)

# Separates a string of flags. " -" or " /" denotes the start of a flag.
# The same sequence inside double quotation marks is ignored.
# Spaces not within quotes are cleaned meaningfully.
# This macro cannot cope with semicolons in the flag string!
macro(separate_flags _flags _output_variable)
  set(_flags_prep " ${_flags} -")
  string(REPLACE " " " ;" _flag_chunks ${_flags_prep}) # Max loop iterations
  set(_flag_string)
  set(_parsed_flags)
  # Loop is necessary because the regex engine is greedy
  foreach(_chunk ${_flag_chunks})
    set(_flag_string "${_flag_string}${_chunk}")
    # Replace all " -" and " /" inside quotation marks
    string(REGEX REPLACE "^(([^\"]*\"[^\"]*\")*[^\"]*\"[^\"]*) [/-]([^\"]*\")"
           "\\1@39535493@\\3" _flag_string "${_flag_string}")
    # Extract one flag if possible
    set(_flag)
    string(REGEX REPLACE "^.* [/-](.+)( [/-].*$)" "${ARGUMENT_STARTER}\\1" _flag "${_flag_string}")
    string(REGEX REPLACE "^.* [/-](.+)( [/-].*$)" "\\2"  _flag_string "${_flag_string}")
    if(NOT _flag STREQUAL _flag_string)
      list(APPEND _parsed_flags "${_flag}")
    endif(NOT _flag STREQUAL _flag_string)
  endforeach(_chunk)

  # Re-replace all " -" and " /" inside quotation marks
  string(REGEX REPLACE "@39535493@" " -" ${_output_variable} "${_parsed_flags}")
endmacro(separate_flags)

# Internal macro, do not use
# Modifies the flags according to the mode: set, add or remove
# Also sets flags according to the CACHE and FORCE parameter.
# If only CACHE is specified, set_CACHE() is used.
macro(_internal_parse_flags _mode _flags _varname _write_to_cache _force)
  separate_flags("${_flags}" _arg_flag_list)

  if("${_mode}" STREQUAL "set")
    # set
    set(_flag_list "${_arg_flag_list}")
  else()
    # ADD or REMOVE
    separate_flags("${${_varname}}" _flag_list)
    foreach(_flag ${_arg_flag_list})
      list(${_mode} _flag_list "${_flag}")
    endforeach(_flag)
  endif()

  list(REMOVE_DUPLICATES _flag_list)
  list(SORT _flag_list)
  string(REPLACE ";" " " _flag_list "${_flag_list}")

  if(_write_to_cache)
    if(_force)
      set(${_varname} "${_flag_list}" CACHE STRING "${${_varname}}" FORCE)
      set(${_varname} "${_flag_list}" CACHE STRING "${${_varname}}" FORCE)
    else()
      set_cache(${_varname} STRING "${${_varname}}" "${_flag_list}")
    endif()
  else()
    set(${_varname} "${_flag_list}")
  endif()
endmacro(_internal_parse_flags)

# Internal macro, do not use
# Parses the given additional arguments and sets the flags to the
# corresponding variables.
macro(_internal_parse_flags_args _mode _keys _key_postfix _flags)
  set(_langs)
  set(_build_types)
  set(_cond TRUE)
  set(_invert_condition FALSE)
  set(_write_to_cache FALSE)
  set(_force FALSE)
  string(REPLACE ";" "|" _key_regex "${_keys}")
  set(_key_regex "^(${_key_regex})$")

  foreach(_arg ${ARGN})
    if(_arg MATCHES "${_key_regex}")
      list(APPEND _langs "${_arg}")
    elseif(   _arg MATCHES "^(Debug|Release|MinSizeRel|RelWithDebInfo)$"
           OR _arg MATCHES "^(DEBUG|RELEASE|MINSIZEREL|RELWITHDEBINFO)$")
      string(TOUPPER "${_arg}" _upper_arg)
      list(APPEND _build_types ${_upper_arg})
    elseif(_arg STREQUAL "ReleaseAll")
      list(APPEND _build_types RELEASE MINSIZEREL RELWITHDEBINFO)
    elseif(_arg STREQUAL "CACHE")
      set(_write_to_cache TRUE)
    elseif(_arg STREQUAL "FORCE")
      set(_force TRUE)
    elseif(_arg MATCHES "^[Nn][Oo][Tt]$")
      set(_invert_condition TRUE)
    else()
      if(_invert_condition)
        set(_invert_condition FALSE)
    if(${_arg})
          set(_arg_cond FALSE)
    else()
          set(_arg_cond TRUE)
    endif()
      else()
        set(_arg_cond ${${_arg}})
      endif()
      if(_cond AND _arg_cond)
        set(_cond TRUE)
      else()
        set(_cond FALSE)
      endif()
    endif()
  endforeach(_arg)

  # No language specified, use all: C and CXX or EXE, SHARED and MODULE
  if(NOT DEFINED _langs)
    set(_langs ${_keys})
  endif()

  if(_cond)
    foreach(_lang ${_langs})
      set(_varname "CMAKE_${_lang}${_key_postfix}_FLAGS")
      if(DEFINED _build_types)
        foreach(_build_type ${_build_types})
          _INTERNAL_PARSE_FLAGS(${_mode} "${_flags}" ${_varname}_${_build_type} ${_write_to_cache} ${_force})
        endforeach(_build_type)
      else()
        _INTERNAL_PARSE_FLAGS(${_mode} "${_flags}" ${_varname} ${_write_to_cache} ${_force})
      endif()
    endforeach(_lang ${_langs})
  endif(_cond)
endmacro(_internal_parse_flags_args)


# Compiler flags, additional arguments:
# C, CXX: Specify a language, default is both
macro(set_compiler_flags _flags)
  _internal_parse_flags_args(set "C;CXX" "" "${_flags}" "${ARGN}")
endmacro(set_compiler_flags)
# Add flags (flags don't get added twice)
macro(add_compiler_flags _flags)
  _internal_parse_flags_args(APPEND "C;CXX" "" "${_flags}" "${ARGN}")
endmacro(add_compiler_flags)
# Remove flags
macro(remove_compiler_flags _flags)
  _internal_parse_flags_args(REMOVE_ITEM "C;CXX" "" "${_flags}" "${ARGN}")
endmacro(remove_compiler_flags)

# Linker flags, additional arguments:
# EXE, SHARED, MODULE: Specify a linker mode, default is all three
macro(set_linker_flags _flags)
  _internal_parse_flags_args(set "EXE;SHARED;MODULE" "_LINKER" "${_flags}" "${ARGN}")
endmacro(set_linker_flags)
# Add flags (flags don't get added twice)
macro(add_linker_flags _flags)
  _internal_parse_flags_args(APPEND "EXE;SHARED;MODULE" "_LINKER" "${_flags}" "${ARGN}")
endmacro(add_linker_flags)
# Remove flags
macro(remove_linker_flags _flags)
  _internal_parse_flags_args(REMOVE_ITEM "EXE;SHARED;MODULE" "_LINKER" "${_flags}" "${ARGN}")
endmacro(remove_linker_flags)
