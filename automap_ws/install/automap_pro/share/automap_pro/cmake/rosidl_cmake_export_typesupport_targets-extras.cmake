# generated from
# rosidl_cmake/cmake/template/rosidl_cmake_export_typesupport_targets.cmake.in

set(_exported_typesupport_targets
  "__rosidl_generator_c:automap_pro__rosidl_generator_c;__rosidl_typesupport_fastrtps_c:automap_pro__rosidl_typesupport_fastrtps_c;__rosidl_typesupport_introspection_c:automap_pro__rosidl_typesupport_introspection_c;__rosidl_typesupport_c:automap_pro__rosidl_typesupport_c;__rosidl_generator_cpp:automap_pro__rosidl_generator_cpp;__rosidl_typesupport_fastrtps_cpp:automap_pro__rosidl_typesupport_fastrtps_cpp;__rosidl_typesupport_introspection_cpp:automap_pro__rosidl_typesupport_introspection_cpp;__rosidl_typesupport_cpp:automap_pro__rosidl_typesupport_cpp;__rosidl_generator_py:automap_pro__rosidl_generator_py")

# populate automap_pro_TARGETS_<suffix>
if(NOT _exported_typesupport_targets STREQUAL "")
  # loop over typesupport targets
  foreach(_tuple ${_exported_typesupport_targets})
    string(REPLACE ":" ";" _tuple "${_tuple}")
    list(GET _tuple 0 _suffix)
    list(GET _tuple 1 _target)

    set(_target "automap_pro::${_target}")
    if(NOT TARGET "${_target}")
      # the exported target must exist
      message(WARNING "Package 'automap_pro' exports the typesupport target '${_target}' which doesn't exist")
    else()
      list(APPEND automap_pro_TARGETS${_suffix} "${_target}")
    endif()
  endforeach()
endif()
