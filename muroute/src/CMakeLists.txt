set(TARGETNAME muroute)

set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/${TARGETNAME})

file(COPY ${PROJECT_SOURCE_DIR}/mavlink2 DESTINATION ${INCLUDE_DIR})

set(MUROUTE_HEADER
    ${INCLUDE_DIR}/funcan.h
    ${INCLUDE_DIR}/funserial.h
    ${INCLUDE_DIR}/funtcpclient.h
    ${INCLUDE_DIR}/funtcpserver.h
    ${INCLUDE_DIR}/funudp.h
    ${INCLUDE_DIR}/proto.h
    ${INCLUDE_DIR}/subsystem.h
    ${INCLUDE_DIR}/muroute.h
    ${INCLUDE_DIR}/concqueue.h
    ${INCLUDE_DIR}/mavparamproto.h
    ${INCLUDE_DIR}/componentbus.h
    ${INCLUDE_DIR}/mavparams.h
    )
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SOURCE)

include_directories(${CMAKE_SOURCE_DIR}/include)

# add main library
add_library(${TARGETNAME} ${SOURCE} ${MUROUTE_HEADER0})
set_property(TARGET ${TARGETNAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${TARGETNAME}
    muqueue  memsparse
    ${CMAKE_THREAD_LIBS_INIT}
    ${GLOG_LIBRARIES}
)
target_include_directories(${TARGETNAME} INTERFACE ${CMAKE_INSTALL_PREFIX}/include)
target_include_directories(${TARGETNAME} INTERFACE ${CMAKE_INSTALL_PREFIX}/include/mavlink2)

set_target_properties(${TARGETNAME} PROPERTIES PUBLIC_HEADER "${MUROUTE_HEADER}")
set_target_properties(${TARGETNAME} PROPERTIES VERSION ${VERSION} SOVERSION ${SOVERSION})

add_subdirectory(components)

install(DIRECTORY ${INCLUDE_DIR}/mavlink2
    DESTINATION include/${TARGETNAME}
    COMPONENT ${TARGETNAME}-dev)

install(TARGETS ${TARGETNAME} EXPORT ${TARGETNAME}-targets
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

