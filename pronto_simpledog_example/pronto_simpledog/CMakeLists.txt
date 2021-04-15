cmake_minimum_required(VERSION 2.8.3)
project(pronto_simpledog)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        pronto_ros
                                        pronto_quadruped_ros
                                        simpledog_commons
                                        sensor_msgs)


catkin_package()

include_directories(include
                    ${catkin_INCLUDE_DIRS})

set(NODE_NAME ${PROJECT_NAME}_node)


add_executable(${NODE_NAME} src/pronto_simpledog_node.cpp)
target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES})
add_executable(contact_force_test src/contact_force_test.cpp)
target_link_libraries(contact_force_test ${catkin_LIBRARIES})
add_executable(test_pronto src/test_pronto.cpp)
target_link_libraries(test_pronto ${catkin_LIBRARIES})
add_executable(real_contact_force src/real_contact_force.cpp)
target_link_libraries(real_contact_force ${catkin_LIBRARIES})

install(TARGETS ${NODE_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

