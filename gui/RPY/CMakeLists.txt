

SET(_MAIN_SRC "RPY.cc")

FILE(GLOB_RECURSE _SOURCES       "src/*.cc")
FILE(GLOB_RECURSE _MOC_HEADERS   "include/*.h")
FILE(GLOB_RECURSE _UI_SOURCES    "*.ui")


# Qt5:
################################################################################
find_package(Qt5Core      REQUIRED)
find_package(Qt5Widgets   REQUIRED)
find_package(Qt5Gui       REQUIRED)
find_package(Qt5OpenGL    REQUIRED)

set(QT_INCLUDE_DIRS
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
)

add_definitions(
  ${Qt5Core_DEFINITIONS}
  ${Qt5Widgets_DEFINITIONS}
  ${Qt5Gui_DEFINITIONS}
  ${Qt5OpenGL_DEFINITIONS}
)

set(QT_LIBRARIES
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  Qt5::OpenGL
)

QT5_WRAP_UI( UI_SOURCES ${_UI_SOURCES} )
QT5_WRAP_CPP( MOC_HEADERS ${_MOC_HEADERS} )

INCLUDE_DIRECTORIES (
  ${CMAKE_CURRENT_BINARY_DIR}
  ${QT_INCLUDE_DIRS}
  "include"
)

GET_FILENAME_COMPONENT(_main_name ${_MAIN_SRC} NAME_WE)

ADD_EXECUTABLE( ${_main_name} MACOSX_BUNDLE WIN32
  ${_MAIN_SRC}
  ${_SOURCES}
  ${MOC_HEADERS}
  ${UI_SOURCES}
  ${PRY_GEN_RCS}
)

TARGET_LINK_LIBRARIES ( ${_main_name}
  ${QT_LIBRARIES}
  ${_project_lib}
)
