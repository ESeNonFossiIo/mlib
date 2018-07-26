IF(PCL_WITH_VTK)
  find_package(PCL 1.7 QUIET REQUIRED common filters segmentation io)
ELSE(PCL_WITH_VTK)
  find_package(PCL 1.7 QUIET REQUIRED common filters segmentation)
ENDIF(PCL_WITH_VTK)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
