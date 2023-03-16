cmake_minimum_required(VERSION 2.8)
project( imncc )

# please change the path below to your opencv directory!
set(OpenCV_DIR /home/sieroy/Library/opencv/build)

find_package( OpenCV REQUIRED )
add_executable( ${PROJECT_NAME} 
    imncc.cpp
)
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )

install(TARGETS ${PROJECT_NAME} DESTINATION ..)