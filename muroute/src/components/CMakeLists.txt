set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/components)

set(MUROUTE_COMP_HEADERS
    ${INCLUDE_DIR}/streamcomp.h
    )
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SOURCE)

include_directories(${CMAKE_SOURCE_DIR}/include)

# add main library
add_library(components streamcomp.cc ${MUROUTE_HEADER0})
set_property(TARGET components PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(components
    muroute
    ${CMAKE_THREAD_LIBS_INIT}
    ${GLOG_LIBRARIES}
)
target_include_directories(components INTERFACE ${CMAKE_INSTALL_PREFIX}/include)

set_target_properties(components PROPERTIES PUBLIC_HEADER "${MUROUTE_COMP_HEADERS}")
set_target_properties(components PROPERTIES VERSION ${VERSION} SOVERSION ${SOVERSION})

install(TARGETS components EXPORT components-targets
    RUNTIME DESTINATION bin COMPONENT components
    LIBRARY DESTINATION lib COMPONENT components
    ARCHIVE DESTINATION lib COMPONENT components
    PUBLIC_HEADER DESTINATION include/components COMPONENT components-dev
    )

install(EXPORT components-targets
  FILE "${GLOB_PACKAGE_NAME}-components-targets.cmake"
  NAMESPACE ${GLOB_PACKAGE_NAME}::
  DESTINATION lib/cmake/${GLOB_PACKAGE_NAME}
  COMPONENT components-dev
)

