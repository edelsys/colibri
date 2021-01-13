set(TARGETNAME muqueue)

set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/${TARGETNAME})
set(MUQUEUE_HEADER
    ${INCLUDE_DIR}/asynccontroller.h
    ${INCLUDE_DIR}/asyncsubsys.h
    ${INCLUDE_DIR}/detachedfunction.h
    ${INCLUDE_DIR}/erqio.h
    ${INCLUDE_DIR}/erqperiodic.h
    ${INCLUDE_DIR}/erqtimer.h
    ${INCLUDE_DIR}/erqsignal.h
    ${INCLUDE_DIR}/erqfstat.h
    ${INCLUDE_DIR}/muqueue.h
    ${INCLUDE_DIR}/scheduler.h
    )
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SOURCE)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${LIBEV_INCLUDE_DIR}/include)

# add main library
add_library(${TARGETNAME} ${SOURCE} ${MUQUEUE_HEADER})
target_include_directories(${TARGETNAME} INTERFACE ${CMAKE_INSTALL_PREFIX}/include)

target_link_libraries(${TARGETNAME}
    muconfig
    ${LIBEV_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
)

if (BREAK_EV_LOOP_EXPERIMENTAL)
target_compile_definitions(${TARGETNAME} PUBLIC -DBREAK_EV_LOOP_EXPERIMENTAL)
endif()

set_target_properties(${TARGETNAME} PROPERTIES PUBLIC_HEADER "${MUQUEUE_HEADER}")
set_property(TARGET ${TARGETNAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
set_target_properties(${TARGETNAME} PROPERTIES VERSION ${VERSION} SOVERSION ${SOVERSION})

install(TARGETS muqueue EXPORT ${TARGETNAME}-targets
    RUNTIME DESTINATION bin COMPONENT ${TARGETNAME}
    LIBRARY DESTINATION lib COMPONENT ${TARGETNAME}
    ARCHIVE DESTINATION lib COMPONENT ${TARGETNAME}
    PUBLIC_HEADER DESTINATION include/${TARGETNAME} COMPONENT ${TARGETNAME}-dev
#    INCLUDES DESTINATION include/${TARGETNAME} COMPONENT ${TARGETNAME}-dev
    )


install(EXPORT ${TARGETNAME}-targets
  FILE "${GLOB_PACKAGE_NAME}-${TARGETNAME}-targets.cmake"
  NAMESPACE ${GLOB_PACKAGE_NAME}::
  DESTINATION lib/cmake/${GLOB_PACKAGE_NAME}
  COMPONENT ${TARGETNAME}-dev
)

if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/../cmake/uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_BINARY_DIR}/cmake/uninstall.cmake)
endif()
