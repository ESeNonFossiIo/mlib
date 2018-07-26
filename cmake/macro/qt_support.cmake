function(target_qt_support target)
  if (NOT TARGET ${target})
    message(FATAL_ERROR "No ${target}")
  endif()

  set (target_for_ide "${target}_ide_support")
  if (NOT TARGET ${target_for_ide})
      file(GLOB_RECURSE target_for_ide_srcs "*.h" "*.hpp" "*.hxx" "*.c" "*.cpp" "*.cxx")
      add_executable(${target_for_ide} ${target_for_ide_srcs})
      set_target_properties(${target_for_ide} PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)
  endif()

  get_target_property(dirs ${target} INCLUDE_DIRECTORIES)
  target_include_directories(${target_for_ide} PRIVATE ${dirs})

endfunction(target_qt_support)
