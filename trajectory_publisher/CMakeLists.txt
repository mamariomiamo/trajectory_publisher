cmake_minimum_required(VERSION 3.0.2)
project(trajectory_publisher)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  roscpp
  trajectory_msgs
  quadrotor_msgs
  tf
  eigen_conversions
)


find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_publisher
  CATKIN_DEPENDS mavros_msgs roscpp trajectory_msgs quadrotor_msgs tf
  DEPENDS Eigen3
)

add_library(${PROJECT_NAME} src/trajectory_publisher.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include/trajectory_publisher
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen ${catkin_LIBRARIES})

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
# include_directories(
# include
# ${catkin_INCLUDE_DIRS}
# ${EIGEN3_INCLUDE_DIR}
# ${PROJECT_SOURCE_DIR}/include
# )

# # Declare a C++ library
# add_library(${PROJECT_NAME}
# src/${PROJECT_NAME}/trajectory_publisher.cpp
# )
# add_executable(${PROJECT_NAME}_node
# src/main.cpp
# )

# target_link_libraries(${PROJECT_NAME}_node
# ${catkin_LIBRARIES}
# ${PROJECT_NAME}
# )
add_executable(main
  src/main.cpp
)

target_link_libraries(main
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)