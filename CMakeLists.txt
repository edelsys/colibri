project(colibri)

cmake_minimum_required(VERSION 3.1)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(CheckIncludeFile)
include(CheckSymbolExists)
include(CheckFunctionExists)
include(CTest)
include(format)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -latomic")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -gdwarf-3 -O0 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} \
    -Wall -g -Wextra -Wno-deprecated-declarations -Wold-style-cast")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -latomic")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -gdwarf-3 -O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} \
    -Wall -g -Wextra -Wno-deprecated-declarations -Wold-style-cast")

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output/bin)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/output/include)
file(MAKE_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
file(MAKE_DIRECTORY ${LIBRARY_OUTPUT_PATH})

link_directories(${LIBRARY_OUTPUT_PATH})

include_directories( ${CMAKE_BINARY_DIR}/output/include)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/muconfig/include)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/muqueue/include)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/muflow/include)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/muroute/include)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/mutelemetry/include)

######################################
######################################
##### CPACK GENERAL INFO #############
#####
configure_file("LICENSE" "LICENSE" COPYONLY)
configure_file("README.md" "README.md" COPYONLY)

set(GLOB_PACKAGE_NAME libcolibri)

set(CPACK_PACKAGE_NAME lib${PROJECT_NAME})
set(CPACK_SET_DESTDIR OFF)
#set(CPACK_COMPONENTS_GROUPING ONE_PER_GROUP)

set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CPACK_RESOURCE_FILE_README}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Async task queues and event handling library")
set(CPACK_PACKAGE_VENDOR "EDEL LLC")
set(CPACK_PACKAGE_CONTACT "info@edelsys.com")
set(CPACK_PACKAGE_VERSION ${VERSION})

set(CPACK_STRIP_FILES "lib/*")
set(CPACK_SOURCE_STRIP_FILES "")

######################################
##### CPACK DEBIAN INFO ##############
#####

set(CPACK_GENERATOR DEB)
set(CPACK_DEB_COMPONENT_INSTALL 1)
set(CPACK_DEBIAN_ENABLE_COMPONENT_DEPENDS ON)
set(CPACK_DEBIAN_PACKAGE_GENERATE_SHLIBS ON)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "info@edelsys.com")
set(CPACK_OUTPUT_FILE_PREFIX packages)
#set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}.amd64")
#set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")

#####
######################################
######################################

add_subdirectory(muconfig)
add_subdirectory(muqueue)
add_subdirectory(muroute)
add_subdirectory(muflow)
add_subdirectory(mutelemetry)
add_subdirectory(examples)

include(CMakePackageConfigHelpers)

configure_package_config_file(
  "Config.cmake.in"
  "${GLOB_PACKAGE_NAME}Config.cmake"
  INSTALL_DESTINATION lib/cmake/lib${PROJECT_NAME}
#  PATH_VARS  lib
  )

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${GLOB_PACKAGE_NAME}ConfigVersion.cmake
  VERSION 1.0.0
  COMPATIBILITY SameMajorVersion
  )

### Install Config and ConfigVersion files
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/${GLOB_PACKAGE_NAME}Config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/${GLOB_PACKAGE_NAME}ConfigVersion.cmake
  DESTINATION lib/cmake/${GLOB_PACKAGE_NAME}
  COMPONENT all-dev
  )

include(CPack)
