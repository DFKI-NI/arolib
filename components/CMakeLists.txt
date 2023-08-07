set(AROLIB_LIB_NAME "${AROLIB_LIB_PREFIX}${AROLIB_LIB_COMPONENTS}")

add_library(${AROLIB_LIB_NAME} SHARED
    src/fieldprovider.cpp
    src/fieldgeometryprocessor.cpp
    src/infieldbaseroutesplanner.cpp
    src/headlandbaseroutesplanner.cpp
    src/baseroutesplanner.cpp
    src/fieldprocessplanner.cpp
    src/generalrouteplanner.cpp
    src/machinedb.cpp
    src/planpostprocessor.cpp
    src/workedareamanager.cpp
    src/drivenmassareamanager.cpp
    src/gpssimulator.cpp
    src/componentparametershelper.cpp
    src/baseroutesprocessor.cpp
    src/graphprocessor.cpp
    include/arolib/components/fieldprovider.h
    include/arolib/components/fieldgeometryprocessor.h
    include/arolib/components/infieldbaseroutesplanner.h
    include/arolib/components/headlandbaseroutesplanner.h
    include/arolib/components/baseroutesplanner.h
    include/arolib/components/fieldprocessplanner.h
    include/arolib/components/generalrouteplanner.h
    include/arolib/components/machinedb.h
    include/arolib/components/planpostprocessor.h
    include/arolib/components/workedareamanager.h
    include/arolib/components/drivenmassareamanager.h
    include/arolib/components/gpssimulator.h
    include/arolib/components/componentparametershelper.h
    include/arolib/components/baseroutesprocessor.h
    include/arolib/components/graphprocessor.h
)

#set_target_properties(${AROLIB_LIB_NAME} PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})

target_include_directories(${AROLIB_LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${AROLIB_LIB_NAME}
    PUBLIC
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_TYPES}
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_MISC}
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_GEOMETRY}
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_CARTOGRAPHY}
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_PLANNING}
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_IO}
)

install(TARGETS ${AROLIB_LIB_NAME}
        EXPORT ${AROLIB_LIB_NAME}_targets
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib)

add_subdirectory(test)
