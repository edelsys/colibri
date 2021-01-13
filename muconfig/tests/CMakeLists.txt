link_directories(${LIBRARY_OUTPUT_PATH})
include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(test_muconfig test_muconfig.cc)
target_link_libraries(test_muconfig muconfig)
