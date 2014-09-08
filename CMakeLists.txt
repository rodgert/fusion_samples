cmake_minimum_required(VERSION 2.8)
project(fusion-samples)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DAZIOMQ_ENABLE_HANDLER_TRACKING")
find_package(Boost 1.55 COMPONENTS system date_time REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

enable_testing()
add_subdirectory(src)
