cmake_minimum_required(VERSION 3.10)


project(RmdApiSerial VERSION 0.0.1)


set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall --std=c++11 -O3 -fPIC" )
set( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wall -O3 -fPIC" )
link_directories(${CMAKE_SOURCE_DIR}/rmd-sdk/lib/x86)
include_directories("./rmd-sdk/include")

find_library(LIBRARY_NAME_FOUND Rmd_Library PATHS "./rmd-sdk/lib/x86")

add_subdirectory(rmd-sdk)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

#test case
#add_executable(test test/main.cpp)
#target_link_libraries(test rmd CppLinuxSerial)

#add_executable(test1 test/main1.cpp)
#target_link_libraries(test1 rmd CppLinuxSerial)

message("cmake_prefix: ${CMAKE_INSTALL_PREFIX}")
#/////////////////////////////////////////////////

# #Build library



