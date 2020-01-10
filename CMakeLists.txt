cmake_minimum_required( VERSION 2.8 )
project(useEigen)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-O3" )
set( CMAKE_CXX_FLAGS "-std=c++11")

# 添加Eigen头文件
include_directories("/usr/include/eigen3")
#include_directories("/usr/include/Eigen")

add_executable(useEigen main.cpp)

find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_NAME} ${EIGEN3_LIBS})

