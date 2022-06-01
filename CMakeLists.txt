cmake_minimum_required(VERSION 3.13)

project(RidesharingUniversality)


include_directories(include)
file(GLOB SOURCES "src/*.cpp")

add_executable(ridesharing_universality ${SOURCES})

if(MSVC)
    target_compile_options(ridesharing_universality PRIVATE /W4)
else()
    target_compile_options(ridesharing_universality PRIVATE -Wall -Wextra -Wpedantic)
endif()
