project(ICPMatching)

cmake_minimum_required(VERSION 2.8)

find_package(PCL REQUIRED)

include_directories(
  include ${PCL_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

aux_source_directory(. DIR_SRCS)

add_executable(main ${DIR_SRCS})

target_link_libraries(main ${PCL_LIBRARIES})
