find_program( CLANG_FORMAT clang-format-3.6 )
if(NOT CLANG_FORMAT )
    find_program( CLANG_FORMAT clang-format )
endif()

if (CLANG_FORMAT )
    file(GLOB_RECURSE SOURCES ${CMAKE_SOURCE_DIR}/*.cc
        ${CMAKE_SOURCE_DIR}/*.cpp ${CMAKE_SOURCE_DIR}/*.h ${CMAKE_SOURCE_DIR}/*.hpp)
    add_custom_target(taskeng_format ALL)
    add_custom_command(TARGET taskeng_format PRE_BUILD COMMAND
        ${CLANG_FORMAT} -style=Google ${SOURCES} -i)
else()
    if( NOT "$ENV{BUILDNUM}" )
        message( FATAL_ERROR "Unable to find clang-format" )
    endif() 
endif()
