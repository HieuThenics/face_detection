cmake_minimum_required(VERSION 3.0.2)
project(gscam)
set (CMAKE_CXX_STANDARD 17) 


#change path to libtorch /home/user/catkin_ws/src/dependencies/libtorh
set(CMAKE_PREFIX_PATH /home/hieuthenics/catkin_ws/src/dependencies/libtorch)
find_package(Torch REQUIRED)


find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp image_transport sensor_msgs)
find_package(PkgConfig)

pkg_check_modules(GSTREAMER QUIET gstreamer-0.10)
if(NOT GSTREAMER_FOUND)
  set(GSTREAMER_VERSION_1_x TRUE)
endif()
if(GSTREAMER_VERSION_1_x)
  message(STATUS "gst 1.0")
  pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
  pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
else()
  message(STATUS "gst 0.1")
  pkg_check_modules(GSTREAMER REQUIRED gstreamer-0.10)
  pkg_check_modules(GST_APP REQUIRED gstreamer-app-0.10)
endif()

find_package( OpenCV REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gscam
  CATKIN_DEPENDS cv_bridge roscpp image_transport sensor_msgs
  DEPENDS GSTREAMER GST_APP OpenCV
)

include_directories(
  include
  ${TORCH_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GLIB_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME} src/gst_pipeline.cpp src/utils.cpp)
target_link_libraries(${PROJECT_NAME} ${TORCH_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
    ${GST_APP_LIBRARIES}  
    ${OPENCV_LIBRARIES}  
    ${TORCH_LIBRARIES}
    )


## Declare a cpp executable
add_executable(ros_node src/ros_node.cpp)
target_link_libraries(ros_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GST_APP_LIBRARIES}  
  ${OPENCV_LIBRARIES}  
  ${TORCH_LIBRARIES}
  )

add_executable(my_subscriber src/my_subscriber.cpp)
target_link_libraries(my_subscriber 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
    ${GST_APP_LIBRARIES}  
    ${OPENCV_LIBRARIES} )

add_executable(tst src/tst.cpp)
target_link_libraries(tst
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GST_APP_LIBRARIES}  
  ${OPENCV_LIBRARIES}  
  ${TORCH_LIBRARIES}
  )




install(TARGETS ${PROJECT_NAME} ros_node 
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
PATTERN ".svn" EXCLUDE)

