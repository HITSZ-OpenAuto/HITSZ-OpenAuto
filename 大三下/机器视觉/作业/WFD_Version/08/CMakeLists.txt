cmake_minimum_required(VERSION 2.8.3)
project(lineit)

set(CMAKE_CXX_STANDARD 11)

# change the path below to your opencv directory
set(OpenCV_DIR /home/sieroy/桌面/lib/opencv-4.x/build)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} src/lineit.cpp)
target_Link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})