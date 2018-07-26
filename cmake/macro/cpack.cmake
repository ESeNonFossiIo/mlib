
MESSAGE(STATUS "Configuring CPack")

INCLUDE(InstallRequiredSystemLibraries)

SET(CPACK_PACKAGE_VENDOR "MLib")
SET(CPACK_PACKAGE_CONTACT "ESeNonFossiIo <esenonfossiio@gmail.com")
SET(HOMEPAGE "")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY)

SET(CPACK_PACKAGE_DESCRIPTION_FILE "${PROJECT_SOURCE_DIR}/README.md")
SET(CPACK_RESOURCE_FILE_LICENSE 	 "${PROJECT_SOURCE_DIR}/LICENSE")

FILE (READ "VERSION" _version)

STRING(REGEX REPLACE "Version ............" " " _version "${_version}")
STRING(REPLACE "\n" ";" _version ${_version})
LIST(GET _version 0 _version)
STRING(REPLACE " " "" _version ${_version})
STRING(REPLACE "." ";" _version ${_version})

LIST(GET _version 0 CPACK_PACKAGE_VERSION_MAJOR)
LIST(GET _version 1 CPACK_PACKAGE_VERSION_MINOR)
LIST(GET _version 2 CPACK_PACKAGE_VERSION_PATCH)

SET(CPACK_PACKAGE_INSTALL_DIRECTORY "${CMAKE_PROJECT_NAME}")
SET(CPACK_PACKAGE_DIRECTORY "${CMAKE_BINARY_DIR}/package")

SET(CPACK_PACKAGE_EXECUTABLES  "${_project}" "mlib - MLIB Library")
SET(CPACK_CREATE_DESKTOP_LINKS "${_project}")
set(CPACK_PACKAGE_NAME 				 "${_project}")
set(CPACK_BUNDLE_NAME  				 "${_project}")

SET(CPACK_STRIP_FILES TRUE)
SET(CPACK_SET_DESTDIR TRUE)

SET(CPACK_SET_DESTDIR TRUE)
SET(CPACK_INSTALL_PREFIX "/")

IF(APPLE)
	  # Apple specific
	  SET(CPACK_GENERATOR "DragNDrop")
	  SET(CPACK_PACKAGING_INSTALL_PREFIX "/")

	  SET(MACOSX_BUNDLE_BUNDLE_NAME "${_project}")
	  SET(MACOSX_BUNDLE_BUNDLE_GUI_IDENTIFIER "com.${_project}.${_project}")
	  SET(MACOSX_BUNDLE_ICON_FILE ${PROJECT_SOURCE_DIR}/packaging/MLIB.icns)
	  SET(MACOSX_BUNDLE_INFO_PLIST ${PROJECT_SOURCE_DIR}/packaging/MacOSXBundleInfo.plist.in)
		SET_SOURCE_FILES_PROPERTIES(${PROJECT_SOURCE_DIR}/packaging/MLIB.icns
				PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")

	  SET(CPACK_DMG_VOLUME_NAME "${_project}")
	  SET(CPACK_DMG_DS_STORE_SETUP_SCRIPT "${PROJECT_SOURCE_DIR}/packaging/DS_Store.scpt")
	  SET(CPACK_DMG_BACKGROUND_IMAGE "${PROJECT_SOURCE_DIR}/packaging/m_white.png")
	  SET(CPACK_OSX_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}")
ENDIF(APPLE)

IF(WIN32)
#   #--------------------------------------------------------------------------
#   # Windows specific
  SET(CPACK_GENERATOR "STGZ;ZIP")

  # # NSIS windows installer
  # find_program(NSIS_PATH nsis PATH_SUFFIXES nsis)
  # if(NSIS_PATH)
  #   set(CPACK_GENERATOR "${CPACK_GENERATOR};NSIS")
  #   message(STATUS "   + NSIS                                 YES ")
  #   # Note: There is a bug in NSI that does not handle full unix paths properly. Make
  #   # sure there is at least one set of four (4) backlasshes.
  #   set(CPACK_NSIS_DISPLAY_NAME "mlib")
  #   # Icon of the installer
  #   set(CPACK_NSIS_MUI_ICON "${CMAKE_CURRENT_SOURCE_DIR}\\\\package\\\\MLIB.ico")
  #   # set(CPACK_NSIS_HELP_LINK "http:\\\\\\\\www.my-project-home-page.org")
  #   # set(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\www.my-personal-home-page.com")
  #   set(CPACK_NSIS_CONTACT "${CPACK_PACKAGE_CONTACT}")
  #   set(CPACK_NSIS_MODIFY_PATH ON)
  # else()
  #   message(STATUS "   + NSIS                                 NO ")
  # endif()

  set(CPACK_PACKAGE_ICON "${CMAKE_CURRENT_SOURCE_DIR}\\\\package\\\\MLIB.png")
ENDIF(WIN32)

INCLUDE(CPack)

# TODO: Linux

#   # Linux specific
#
#   # SET(CPACK_GENERATOR "DEB")
# SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Mauro Bardelloni") #required
#   set(CPACK_GENERATOR "DEB;TBZ2;TXZ")
#   message(STATUS "Package generation - UNIX")
#   message(STATUS "   + DEB                                  YES ")
#   message(STATUS "   + TBZ2                                 YES ")
#   message(STATUS "   + TXZ                                  YES ")
#
#   find_program(RPMBUILD_PATH rpmbuild)
#   if(RPMBUILD_PATH)
#     message(STATUS "   + RPM                                  YES ")
#     set(CPACK_GENERATOR "${CPACK_GENERATOR};RPM")
#     set(CPACK_RPM_PACKAGE_LICENSE "MIT")
#     # set(CPACK_RPM_PACKAGE_REQUIRES "gtkmm30")
#     # exclude folders which clash with default ones
#     set(CPACK_RPM_EXCLUDE_FROM_AUTO_FILELIST
#       ${CPACK_RPM_EXCLUDE_FROM_AUTO_FILELIST}
#       /usr
#       /usr/bin
#       /usr/share
#       /usr/share/applications
#       /usr/share/doc
#       /usr/share/icons
#       /usr/share/icons/hicolor
#       /usr/share/icons/hicolor/256x256
#       /usr/share/icons/hicolor/256x256/apps
#       /usr/share/icons/gnome
#       /usr/share/icons/gnome/256x256
#     /usr/share/icons/gnome/256x256/apps)
#   else()
#     message(STATUS "   + RPM                                  NO ")
#   endif()
#
#   # TODO do this better
#   set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
#   set(CPACK_DEBIAN_PACKAGE_CONTROL_STRICT_PERMISSION TRUE)
#   set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "${HOMEPAGE}")
#   # set(CPACK_DEBIAN_COMPRESSION_TYPE "xz")
#   # set(CPACK_DEBIAN_PACKAGE_DEPENDS "libgtkmm-3.0")
#
#
#
#   # Icon and app shortcut for Linux systems
#   # Note: .desktop file must have same name as executable
#   install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/exampleApp.desktop
#     DESTINATION share/applications/
#     PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
#   )
#   install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/exampleApp.png
#     DESTINATION share/icons/hicolor/256x256/apps/
#     PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
#   )
#   install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/exampleApp.png
#     DESTINATION share/icons/gnome/256x256/apps/
#     PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
#   )
#   # License file
#   install(FILES ${PROJECT_SOURCE_DIR}/LICENSE
#     DESTINATION share/doc/${PROJECT_NAME}/
#     PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
#   RENAME copyright)
#   # set package icon
#   #     set(CPACK_PACKAGE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/main.png")
# endif()

#------------------------------------------------------------------------------
# and INCLUDE CPack, so we get target for packages
