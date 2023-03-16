cmake_minimum_required(VERSION 2.8)
project( octncc )

# change the path below to your opencv directory!
set(OpenCV_DIR /home/yuki/library/opencv-4.x/build)

find_package( OpenCV REQUIRED )
add_executable( ${PROJECT_NAME} 
    octncc.cpp
)
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )

install(TARGETS ${PROJECT_NAME} DESTINATION ..)