cmake_minimum_required(VERSION 3.16.3)

project(SFM)

add_executable(sfm src/main.cpp src/SocialForce.cpp)

include_directories(${CMAKE_SOURCE_DIR}/include)
