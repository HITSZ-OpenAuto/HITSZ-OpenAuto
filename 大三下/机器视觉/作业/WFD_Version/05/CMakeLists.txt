cmake_minimum_required(VERSION 2.8)
project( imfilter )

# change the path below to your opencv directoy
set(OpenCV_DIR /home/sieroy/Library/opencv/build)

find_package( OpenCV REQUIRED )
add_executable( ${PROJECT_NAME} 
    src/imfilter.cpp
)
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )

install(TARGETS ${PROJECT_NAME} DESTINATION ..)