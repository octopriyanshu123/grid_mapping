



cmake_minimum_required(VERSION 3.10)
project(rmd_sdk_example)

add_compile_options(-std=c++14)
add_compile_options(-fPIC)

# Include the directories where the headers are located
include_directories(${CMAKE_SOURCE_DIR}/../include)

# Add the directory where the library is located
link_directories(${CMAKE_SOURCE_DIR}/../lib/x86)

# Define the source files
set(SOURCES ex2_single.cpp    
            )

# Define the executable
add_executable(ex1 ${SOURCES})

target_link_libraries( ex1 Rmd_Library )

