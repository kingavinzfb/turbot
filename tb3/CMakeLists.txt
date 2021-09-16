cmake_minimum_required(VERSION 3.0.2)
project(tb3)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    tf
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs nav_msgs tf geometry_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

