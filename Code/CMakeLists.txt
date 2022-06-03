cmake_minimum_required(VERSION 3.13)

project(RidesharingUniversality)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "-O3 -Wall")

include_directories("include")
file(GLOB SOURCES "src/*.cpp")

add_executable(ridesharing_universality ${SOURCES})