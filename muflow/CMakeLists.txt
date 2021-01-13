PROJECT(muflow C CXX)

set(VERSION 0.2.2)
set(SOVERSION 0.2)

set(CPACK_PACKAGE_NAME lib${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(CheckIncludeFile)
include(CheckSymbolExists)
include(CheckFunctionExists)

set(BUILD_SHARED_LIBS ON)

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output/bin)

set(FFLOW_OUTPUT ${CMAKE_BINARY_DIR}/output)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/output/include/functionflow)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/output/bin)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_INSTALL_PREFIX}/include)
link_directories(${CMAKE_INSTALL_PREFIX}/lib)

include_directories(${CMAKE_BINARY_DIR}/output/include)
link_directories(${CMAKE_BINARY_DIR}/output/lib )

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
find_package(Threads REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(Glog)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(Boost REQUIRED thread chrono regex filesystem
    iostreams system serialization date_time atomic
    program_options unit_test_framework
    )
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

configure_file(muflow_config.h.in ${PROJECT_SOURCE_DIR}/include/muflow/muflow.h)

add_subdirectory(src)
add_subdirectory(tests)

add_dependencies(muflow muqueue muroute)