set(TARGETNAME mutelemetry)

set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/${TARGETNAME})
set(MUTELEMETRY_HEADER
    ${INCLUDE_DIR}/mutelemetry.h
    ${INCLUDE_DIR}/mutelemetry_ulog.h
    ${INCLUDE_DIR}/mutelemetry_parse.h
    ${INCLUDE_DIR}/mutelemetry_logger.h
    ${INCLUDE_DIR}/mutelemetry_network.h
    ${INCLUDE_DIR}/mutelemetry_tools.h
    )
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SOURCE)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_library(${TARGETNAME} ${SOURCE} ${MUTELEMETRY_HEADER})

target_link_libraries(${TARGETNAME}
    muroute muconfig memsparse
    ${Boost_LIBRARIES}
    ${GLOG_LIBRARIES}
    )

target_include_directories(${TARGETNAME} INTERFACE ${CMAKE_INSTALL_PREFIX}/include)

set_target_properties(${TARGETNAME} PROPERTIES PUBLIC_HEADER "${MUTELEMETRY_HEADER}")
set_property(TARGET ${TARGETNAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
set_target_properties(${TARGETNAME} PROPERTIES VERSION ${VERSION} SOVERSION ${SOVERSION})

install(TARGETS ${TARGETNAME} EXPORT ${TARGETNAME}-targets
    PUBLIC_HEADER DESTINATION include/${TARGETNAME} COMPONENT ${TARGETNAME}-dev
    RUNTIME DESTINATION bin COMPONENT ${TARGETNAME}
    LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT ${TARGETNAME}
    ARCHIVE DESTINATION lib COMPONENT ${TARGETNAME})


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
