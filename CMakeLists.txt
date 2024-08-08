cmake_minimum_required(VERSION 3.12)
project(RRTZ LANGUAGES CXX)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

SET(CMAKE_EXE_LINKER_FLAGS 
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,${CMAKE_SOURCE_DIR}/build")

# set the g++ compiler flags
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_LOADED)
    message(STATUS "Compiler Path: ${CMAKE_CXX_COMPILER}")
    message(STATUS "Compiler ID: ${CMAKE_CXX_COMPILER_ID}")
    message(STATUS "Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}")
endif()

# include header dirs
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

# find headers
file(GLOB HEADER_FILES_HPP ${CMAKE_SOURCE_DIR}/include/*.hpp)
message(STATUS "Header Files: ${HEADER_FILES_HPP}")

set(SOURCES
    ${CMAKE_SOURCE_DIR}/src/obs.cpp
    ${CMAKE_SOURCE_DIR}/src/rrtz.cpp
    ${CMAKE_SOURCE_DIR}/src/utils.cpp
)

add_executable(main ${SOURCES} ${CMAKE_SOURCE_DIR}/src/main.cpp)