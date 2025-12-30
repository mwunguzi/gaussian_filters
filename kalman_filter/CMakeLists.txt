
cmake_minimum_required(VERSION 3.22)
project(kalman_filter)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(kalman_filter SHARED ${PROJECT_SOURCE_DIR}/src/kalman_filter.cpp)
target_include_directories(kalman_filter PUBLIC ${PROJECT_SOURCE_DIR}/include)
set_target_properties(kalman_filter PROPERTIES
	              ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib"
	              LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib"
	              RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib")
