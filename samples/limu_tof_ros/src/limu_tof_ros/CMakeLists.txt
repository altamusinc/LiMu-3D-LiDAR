cmake_minimum_required(VERSION 3.0.2)
project(limu_tof_ros)

add_compile_options(-std=c++14)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

message( STATUS "Architecture: ${ARCHITECTURE}" )

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
)

find_package(OpenCV REQUIRED)

message("OpenCV version: ${OpenCV_VERSION}")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package (Threads)

find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(cfg/limu_tof_ros.cfg)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES limu_tof_ros
  CATKIN_DEPENDS roscpp dynamic_reconfigure message_runtime sensor_msgs
)

add_definitions(${PCL_DEFINITIONS})

include_directories( 
  ../../../../driver/include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ../../../../driver/lib/${ARCHITECTURE}
  ${PCL_LIBRARY_DIRS}
  {CMAKE_THREAD_LIBS_INIT}
)

add_executable(limu_camera_node src/limu_camera_node.cpp
              )

add_dependencies(limu_camera_node ${PROJECT_NAME}_gencfg)

add_dependencies(limu_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(limu_camera_node ${catkin_LIBRARIES})
target_link_libraries(limu_camera_node ${OpenCV_LIBS} ${PCL_LIBRARIES} limu_driver)

install(TARGETS limu_camera_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
