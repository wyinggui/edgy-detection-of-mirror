SET(PROJECT_NAME line_detection_LSD)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(${PROJECT_NAME})
#set(CMAKE_CXX_FLAGS "-std=c++11")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
SET(CMAKE_BUILD_TYPE Debug) # Release, RelWithDebInfo

find_package(OpenCV 3.4.0 REQUIRED)
include(/home/wyinggui/opencv-3.4.0/build/OpenCVConfig.cmake)



SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")





find_package(catkin REQUIRED COMPONENTS
  roscpp roslib cmake_modules cv_bridge image_transport pcl_ros sensor_msgs geometry_msgs)
catkin_package(
    DEPENDS OpenCV 
    CATKIN_DEPENDS roscpp roslib image_transport pcl_ros
    #LIBRARIES efficient_map efficient_map_cuda
)


include_directories(
  ${OpenCV_INCLUDE_DIRS}

)

# Set link libraries
list(APPEND LINK_LIBS
  ${OpenCV_LIB_COMPONENTS}
  ${OpenCV_LIBS}

)

include_directories(${catkin_INCLUDE_DIRS})
list(APPEND LINK_LIBS ${catkin_LIBRARIES})

add_executable(line_detection_LSD
    src/line_detection_LSD.cpp
    src/line_find.cpp
    )
target_link_libraries(line_detection_LSD
${catkin_LIBRARIES} ${OpenCV_LIBS} 
)
