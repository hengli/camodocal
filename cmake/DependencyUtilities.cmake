set(__required_dependencies CACHE INTERNAL "list of all required dependencies")
set(__optional_dependencies CACHE INTERNAL "list of all optional dependencies")

function(camodocal_required_dependency _name)
  find_package(${_name} REQUIRED)

  list(APPEND __required_dependencies ${_name})
  set(__required_dependencies ${__required_dependencies} CACHE INTERNAL "list of all required dependencies")
endfunction(camodocal_required_dependency _name)

function(camodocal_optional_dependency _name)
  find_package(${_name})

  list(APPEND __optional_dependencies ${_name})
  set(__optional_dependencies ${__optional_dependencies} CACHE INTERNAL "list of all optional dependencies")
endfunction(camodocal_optional_dependency _name)

