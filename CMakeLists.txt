cmake_minimum_required(VERSION 3.22)

project(test)

set(CMAKE_BUILD_TYPE "Debug")

include_directories(/usr/local/include)

link_directories(/usr/local/lib)

add_executable(excutable src/main.c)

target_link_libraries(excutable ethercat pthread)




