cmake_minimum_required(VERSION 2.8.3)
project(fovis)
find_package(catkin REQUIRED COMPONENTS
)

find_package(PkgConfig REQUIRED)

find_package(Eigen3 REQUIRED)

pkg_check_modules(GLIB2 glib-2.0)

catkin_package(
  INCLUDE_DIRS ../../libfovis/libfovis ${GLIB2_INCLUDE_DIRS}
  LIBRARIES fovis
#  CATKIN_DEPENDS bot_core_lcmtypes
#  DEPENDS GLIB2
#  CFG_EXTRAS bot-core-includes.cmake
)

file(GLOB c_files ../../libfovis/libfovis/*.c*)
file(GLOB h_files ../../libfovis/libfovis/*.h*)

include_directories(../../libfovis/libfovis ${GLIB2_INCLUDE_DIRS} 
	${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

add_library(fovis SHARED ${c_files})
target_link_libraries(fovis ${catkin_LIBRARIES} ${GLIB2_LIBRARIES} m)
set_property(TARGET fovis APPEND_STRING PROPERTY COMPILE_FLAGS -Wno-unused-result )

install(TARGETS fovis
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES ${h_files}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})