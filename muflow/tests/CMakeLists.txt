link_directories(${CMAKE_BINARY_DIR}/output/lib)
include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(testloader testloader.cc)
target_link_libraries(testloader
    muflow
    dl
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    )

add_library(testplugin SHARED testplugin.cc)
target_link_libraries(testplugin
    muflow
    ${CMAKE_THREAD_LIBS_INIT}
    )

add_library(testplugin2 SHARED testplugin2.cc)
target_link_libraries(testplugin2
    muflow
    ${CMAKE_THREAD_LIBS_INIT}
    )

add_executable(testsparsemem testsparsemem.cc)
target_link_libraries(testsparsemem
    muflow
    dl
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    )
