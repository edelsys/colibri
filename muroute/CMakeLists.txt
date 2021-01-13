project(muroute)

set(VERSION 0.2.2)
set(SOVERSION 0.2)

#set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(CheckIncludeFile)
include(CheckSymbolExists)
include(CheckFunctionExists)

set(BUILD_SHARED_LIBS ON)

set(HEADER_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output/include)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output/bin)
set(FFLOW_OUTPUT ${CMAKE_BINARY_DIR}/output)

file(MAKE_DIRECTORY ${HEADER_OUTPUT_PATH}/functionflow)
file(MAKE_DIRECTORY ${LIBRARY_OUTPUT_PATH})
file(MAKE_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_INSTALL_PREFIX}/include)
link_directories(${CMAKE_INSTALL_PREFIX}/lib)

include_directories(${HEADER_OUTPUT_PATH})
link_directories(${LIBRARY_OUTPUT_PATH})

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
find_package(Threads REQUIRED)

find_package(Glog)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(Boost REQUIRED thread chrono regex filesystem
    iostreams system serialization date_time atomic
    program_options unit_test_framework)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

configure_file(muroute_config.h.in ${PROJECT_SOURCE_DIR}/include/muroute/muroute.h)

add_subdirectory(src)

add_dependencies(muroute muqueue)
