cmake_minimum_required(VERSION 3.10.2)
project(tkp_gazebo_simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV 4 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    image_transport
    cv_bridge
    gazebo_ros
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(UAV_Simulation
    src/uav_simulation.cpp
    src/uav_controller.cpp
    src/vehicle_controller.cpp
    src/data_recorder.cpp
    src/parameters.cpp
)

target_link_libraries(UAV_Simulation
    ${catkin_LIBRARIES}
)

# add_executable(UGV_Simulation
#     src/gazebo_ugv_simulation.cpp
#     src/vehicle_controller.cpp
#     src/data_recorder.cpp
#     src/parameters.cpp  
# )

# target_link_libraries(UGV_Simulation
#     ${catkin_LIBRARIES}
# )
