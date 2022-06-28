cmake_minimum_required(VERSION 3.0.2)
project(dip_pkg)
set(CMAKE_CXX_STANDARD 11)

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        cv_bridge
        tf
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

catkin_package()


add_executable(expr1_node src/expr1.cpp include/dip_pkg/expr1.hpp)
target_link_libraries(expr1_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(expr1_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(quaternion_test src/quaternion_test.cpp)
target_link_libraries(quaternion_test ${catkin_LIBRARIES})
add_dependencies(quaternion_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(expr2_node src/expr2.cpp include/dip_pkg/expr2.hpp)
target_link_libraries(expr2_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(expr2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(expr3_node src/expr3.cpp include/dip_pkg/expr3.hpp)
target_link_libraries(expr3_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(expr3_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dip8_node src/dip8.cpp)
target_link_libraries(dip8_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(dip8_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(expr4_node src/expr4.cpp include/dip_pkg/expr4.hpp)
target_link_libraries(expr4_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(expr4_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})