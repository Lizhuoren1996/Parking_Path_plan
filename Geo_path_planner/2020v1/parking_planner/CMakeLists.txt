
project(parking_planner)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 3.0)

aux_source_directory(./ SRC_LIST)

add_executable(parking_planner ${SRC_LIST})

target_link_libraries(parking_planner glog)