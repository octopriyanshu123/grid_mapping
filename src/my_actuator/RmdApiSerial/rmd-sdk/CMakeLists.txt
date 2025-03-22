

cmake_minimum_required(VERSION 3.10)
project(rmd-sdk)



set(CMAKE_PREFIX_PATH /home/ws/Octo_crawler/src/my_actuator/RmdSerialApi/rmd-sdk/lib/x86)
# Specify library directories
link_directories(/home/ws/Octo_crawler/src/my_actuator/RmdSerialApi/rmd-sdk/lib/x86)

# Find the Rmd_Library
find_library(RMD_LIBRARY_FOUND Rmd_Library PATHS /home/ws/Octo_crawler/src/my_actuator/RmdSerialApi/rmd-sdk/lib/x86)

if(NOT RMD_LIBRARY_FOUND)
    message(FATAL_ERROR "Rmd_Library not found.")
endif()



