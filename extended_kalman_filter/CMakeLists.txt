
cmake_minimum_required(VERSION 3.22)
project(extened_kalman_filter)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(extended_kalman_filter SHARED ${PROJECT_SOURCE_DIR}/src/extended_kalman_filter.cpp)
target_include_directories(extended_kalman_filter PUBLIC ${PROJECT_SOURCE_DIR}/include)
set_target_properties(extended_kalman_filter PROPERTIES
	              ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib"
	              LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib"
	              RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib")
