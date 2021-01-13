find_package(Boost REQUIRED thread chrono regex filesystem
    iostreams system serialization date_time atomic
    program_options unit_test_framework
    )

link_directories(${CMAKE_BINARY_DIR}/output/lib)

# Creating custom protocol
add_executable(customproto customproto.cc)
target_link_libraries(customproto
    muqueue muroute
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    ${LIBEV_LIBRARIES}
    ${Boost_LIBRARIES}
    )

# Mavlink router
add_executable(mavlink-router mavlink-router.cc)
target_link_libraries(mavlink-router
    muqueue muroute
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    ${LIBEV_LIBRARIES}
    ${Boost_LIBRARIES}
    )

# Telemetry examples
add_executable(mutelemetry_example mutelemetry_example.cc)
target_link_libraries(mutelemetry_example
    mutelemetry muroute
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    )

add_executable(mutelemetry_client mutelemetry_client.cc)
target_link_libraries(mutelemetry_client
    mutelemetry muroute
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    )

# Config file examples
add_executable(muconfig_example muconfig_example.cc)
target_link_libraries(muconfig_example muconfig)

########## EV WRAPPERS #############
# Examples of using ev wrapers
add_executable(file_watcher_example file_watcher_example.cc)
target_link_libraries(file_watcher_example
muqueue
${CMAKE_THREAD_LIBS_INIT}
${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)

add_executable(io_example io_example.cc)
target_link_libraries(io_example
muqueue
${CMAKE_THREAD_LIBS_INIT}
${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)

add_executable(periodic_example periodic_example.cc)
target_link_libraries(periodic_example
muqueue
${CMAKE_THREAD_LIBS_INIT}
${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)

add_executable(signal_example signal_example.cc)
target_link_libraries(signal_example
muqueue
${CMAKE_THREAD_LIBS_INIT}
${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)

add_executable(timer1_example timer1_example.cc)
target_link_libraries(timer1_example
muqueue
${CMAKE_THREAD_LIBS_INIT}
${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)

add_executable(timer2_example timer2_example.cc)
target_link_libraries(timer2_example
muqueue
${CMAKE_THREAD_LIBS_INIT}
${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)


