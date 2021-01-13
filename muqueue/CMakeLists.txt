project(muqueue)

set(VERSION 0.2.2)
set(SOVERSION 0.2)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(CheckIncludeFile)
include(CheckSymbolExists)
include(CheckFunctionExists)

option(BREAK_EV_LOOP_EXPERIMENTAL "Exit threads holding event loop. Use with caution" ON)

set(BUILD_SHARED_LIBS ON)

set(HEADER_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output/include)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output/bin)

file(MAKE_DIRECTORY ${HEADER_OUTPUT_PATH})
file(MAKE_DIRECTORY ${LIBRARY_OUTPUT_PATH})
file(MAKE_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_INSTALL_PREFIX}/include)
link_directories(${CMAKE_INSTALL_PREFIX}/lib)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)

find_package(Threads REQUIRED)

find_package(LibEV REQUIRED)

find_package(Boost REQUIRED
    thread
    chrono
    serialization
    date_time
    atomic
    iostreams
    system
    filesystem
    regex
    program_options
    unit_test_framework
    )

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

add_subdirectory(src)

add_dependencies(muqueue muconfig)
