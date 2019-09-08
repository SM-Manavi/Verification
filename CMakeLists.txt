# This is the CMake script for compiling a CGAL application.

project(Graphic_Connectivity)

cmake_minimum_required(VERSION 3.1)
if(NOT POLICY CMP0070 AND POLICY CMP0053)
  # Only set CMP0053 to OLD with CMake<3.10, otherwise there is a warning.
  cmake_policy(SET CMP0053 OLD)
endif()

find_package(CGAL COMPONENTS Core Qt5 )
include( ${CGAL_USE_FILE} )

find_package(Boost COMPONENTS filesystem serialization REQUIRED)
#find_package(Boost QUIET)

find_package( Qt5 QUIET COMPONENTS Script Widgets )

if ( CGAL_FOUND AND CGAL_Qt5_FOUND AND Qt5_FOUND AND CGAL_Core_FOUND)
  include_directories( Heder_files )
  include_directories( Algorithm )
#  include_directories( ./ )

  add_definitions(-DQT_NO_KEYWORDS)

#  qt5_wrap_ui( Connectivity_uis  GEnvironment.ui )

#  qt5_wrap_cpp( CGAL_Qt5_MOC_FILES  GEnvironment.h)

  add_subdirectory(Lib)
  list(APPEND EXTRA_LIBS Lib)


#   add_library(lib
#       Robot.cpp
#       Environment.cpp
#       Arrangement.cpp
#       Event.cpp
#       dispatch.cpp)

   add_executable( main
     main.cpp
 #   ${CGAL_Qt5_MOC_FILES}
 #   ${Connectivity_uis}
   )
#  target_include_directories(main PUBLIC Heder_files)
  #qt5_use_modules(main Widgets Script)
  target_link_libraries( main
    ${EXTRA_LIBS}
    ${CGAL_LIBRARIES}
    ${CGAL_3RD_PARTY_LIBRARIES}
    ${QT_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
	${Boost_SERIALIZATION_LIBRARY}
  )

  add_to_cached_list( CGAL_EXECUTABLE_TARGETS main )

else()

  message(STATUS "NOTICE: This demo requires CGAL, CGAL-Core and Qt5, and will not be compiled.")

endif()

# add_subdirectory( tests )
