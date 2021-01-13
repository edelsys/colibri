set(TARGETNAME muflow)

set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/${TARGETNAME})
set(FUNFLOW_HEADER
    ${INCLUDE_DIR}/sparsememory.h
    ${INCLUDE_DIR}/sparseutils.h
    ${INCLUDE_DIR}/vertex.h
    ${INCLUDE_DIR}/loopclosing.h
    ${INCLUDE_DIR}/functiontable.h
    ${INCLUDE_DIR}/dynamicloader.h
    ${INCLUDE_DIR}/hashing.h
    ${INCLUDE_DIR}/muflow.h
    )
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SOURCE)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(/usr/include)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include )

# add main library
set(SPARSE_MEM_SOURCES
    sparsememory.cc
    sparseutils.cc
    functiontable.cc
    vertex.cc
    loopclosing.cc
    ../include/muflow/sparsememory.h
    ../include/muflow/sparseutils.h
    ../include/muflow/vertex.h
    ../include/muflow/loopclosing.h
    ../include/muflow/functiontable.h
    )

# add main library
set(FUNCTBL_SOURCES
    dynamicloader.cc
    hashing.cc
    ../include/muflow/dynamicloader.h
    ../include/muflow/hashing.h
    )

add_library(memsparse ${SPARSE_MEM_SOURCES})
target_link_libraries(memsparse
    muqueue
    ${LIBDL_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
)
set_target_properties(memsparse PROPERTIES VERSION ${VERSION} SOVERSION ${SOVERSION})

add_library(${TARGETNAME} ${FUNCTBL_SOURCES})
target_link_libraries(${TARGETNAME}
    muroute memsparse
    ${LIBDL_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
)

target_include_directories(${TARGETNAME} INTERFACE ${CMAKE_INSTALL_PREFIX}/include)

set_target_properties(${TARGETNAME} PROPERTIES PUBLIC_HEADER "${FUNFLOW_HEADER}")
set_property(TARGET ${TARGETNAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
set_target_properties(${TARGETNAME} PROPERTIES VERSION ${VERSION} SOVERSION ${SOVERSION})
set_target_properties(${TARGETNAME} PROPERTIES VERSION ${VERSION} SOVERSION ${SOVERSION})

install(TARGETS ${TARGETNAME} memsparse EXPORT ${TARGETNAME}-targets
    RUNTIME DESTINATION bin COMPONENT ${TARGETNAME}
    LIBRARY DESTINATION lib COMPONENT ${TARGETNAME}
    ARCHIVE DESTINATION lib COMPONENT ${TARGETNAME}
    PUBLIC_HEADER DESTINATION include/${TARGETNAME}/ COMPONENT ${TARGETNAME}-dev
)

install(EXPORT ${TARGETNAME}-targets
    FILE "${GLOB_PACKAGE_NAME}-${TARGETNAME}-targets.cmake"
  NAMESPACE ${GLOB_PACKAGE_NAME}::
  DESTINATION lib/cmake/${GLOB_PACKAGE_NAME}
  COMPONENT ${TARGETNAME}-dev
)
