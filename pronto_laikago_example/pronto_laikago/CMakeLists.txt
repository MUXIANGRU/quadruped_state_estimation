cmake_minimum_required(VERSION 2.8.3)
project(pronto_laikago)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        pronto_ros
                                        pronto_quadruped_ros
                                        laikago_commons
                                        sensor_msgs)


catkin_package()

include_directories(include
                    ${catkin_INCLUDE_DIRS})

set(NODE_NAME ${PROJECT_NAME}_node)


add_executable(${NODE_NAME} src/pronto_laikago_node.cpp)
target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES})
add_executable(laikago_state src/laikago_state.cpp)
target_link_libraries(laikago_state ${catkin_LIBRARIES})
add_executable(pose_means src/pose_means.cpp)
target_link_libraries(pose_means ${catkin_LIBRARIES})
#add_executable(test_pronto src/test_pronto.cpp)
#target_link_libraries(test_pronto ${catkin_LIBRARIES})
add_executable(laikago_test src/laikago_test.cpp)
target_link_libraries(laikago_test ${catkin_LIBRARIES})

install(TARGETS ${NODE_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

