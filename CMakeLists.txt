cmake_minimum_required(VERSION 3.5)
project(road_recognizer)

SET(CMAKE_CXX_FLAGS "-std=c++14 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  message_generation
  cv_bridge
  amsl_navigation_msgs
)

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(Eigen3 REQUIRED COMPONENTS system)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
	FILES
	OtsuBinary.msg
	Analysis.msg
	Intensity.msg
	Distribution.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES road_recognizer
	CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(peak_detector src/peak_detector.cpp)
add_dependencies(peak_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(road_cloud_publisher src/road_cloud_publisher_node.cpp src/road_cloud_publisher.cpp src/intensity_partition_node.cpp)
add_executable(road_point_cloud_storer src/road_point_cloud_storer_node.cpp src/road_point_cloud_storer.cpp)
add_executable(road_recognizer src/road_recognizer_node.cpp src/road_recognizer.cpp)
add_executable(intensity_partition src/intensity_partition.cpp)
add_executable(old_intensity_partition src/old_intensity_partition.cpp)
add_executable(make_image_node src/make_image_node.cpp src/make_image.cpp)
add_executable(intersection_detector src/intersection_detector.cpp src/intersection_detector_node.cpp)
add_executable(road_shape_estimator src/road_shape_estimator_node.cpp src/road_shape_estimator.cpp)
add_executable(road_boundary_detector src/road_boundary_detector_node.cpp src/road_boundary_detector.cpp)

add_dependencies(road_cloud_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(road_point_cloud_storer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(road_recognizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(intensity_partition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(old_intensity_partition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(make_image_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(intersection_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(road_shape_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(road_boundary_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(road_cloud_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(road_point_cloud_storer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(road_recognizer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(intensity_partition
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(old_intensity_partition
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(make_image_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(intersection_detector
  ${catkin_LIBRARIES}
  peak_detector
)
target_link_libraries(road_shape_estimator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  peak_detector
)
target_link_libraries(road_boundary_detector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()

