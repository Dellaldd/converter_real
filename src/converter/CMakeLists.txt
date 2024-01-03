cmake_minimum_required(VERSION 3.0.2)
project(converter)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  eigen_conversions
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS 
 LIBRARIES converter eigen_conversions
 CATKIN_DEPENDS roscpp std_msgs cv_bridge image_transport
 DEPENDS OpenCV EIGEN3
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(imu src/imu_data.cpp)


target_link_libraries(imu
  ${catkin_LIBRARIES}
)
