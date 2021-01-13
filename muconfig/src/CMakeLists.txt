set(TARGETNAME muconfig)

set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/${TARGETNAME})
set(MUCONFIG_HEADER ${INCLUDE_DIR}/muconfig.h)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SOURCE)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(../contrib/cpptoml/include)

add_library(${TARGETNAME} ${SOURCE} ${MUCONFIG_HEADER})

target_include_directories(${TARGETNAME} INTERFACE ${CMAKE_INSTALL_PREFIX}/include)

set_target_properties(${TARGETNAME} PROPERTIES PUBLIC_HEADER "${MUCONFIG_HEADER}")
set_property(TARGET ${TARGETNAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
set_target_properties(${TARGETNAME} PROPERTIES VERSION ${VERSION} SOVERSION ${SOVERSION})

install(TARGETS ${TARGETNAME} EXPORT ${TARGETNAME}-targets
    RUNTIME DESTINATION bin COMPONENT ${TARGETNAME}
    LIBRARY DESTINATION lib COMPONENT ${TARGETNAME}
    ARCHIVE DESTINATION lib COMPONENT ${TARGETNAME}
    PUBLIC_HEADER DESTINATION include/${TARGETNAME} COMPONENT ${TARGETNAME}-dev
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
