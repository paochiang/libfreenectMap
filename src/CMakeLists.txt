cmake_minimum_required(VERSION 2.8)  
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
project(ColorMap)

#OPENCV
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS} )
link_directories(${OPENCV_LIBRARY_DIRS})
add_definitions(${OPENCV_DEFINITIONS})

include_directories("H:/libfreenectMap/3rd/include")
link_directories("H:/libfreenectMap/3rd/lib")

add_executable(ColorMap main.cpp libfreenect2opencv.cpp libfreenect2opencv.h)
set (LINK_LIB optimized freenect2 debug freenect2d)
TARGET_LINK_LIBRARIES(ColorMap ${OpenCV_LIBS} ${LINK_LIB})