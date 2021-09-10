set(AROLIB_LIB_NAME "${AROLIB_LIB_PREFIX}${AROLIB_LIB_COMPONENTS}")

add_library(${AROLIB_LIB_NAME} SHARED
    src/fieldprovider.cpp
    src/baseroutesinfieldplanner.cpp
    src/headlandplanner.cpp
    src/fieldprocessplanner.cpp
    src/generalrouteplanner.cpp
    src/machinedb.cpp
    src/planpostprocessor.cpp
    src/tracksgenerator.cpp
    src/workedareamanager.cpp
    src/gpssimulator.cpp
    src/componentparametershelper.cpp
    src/baseroutesprocessor.cpp
    src/graphprocessor.cpp
    include/arolib/components/fieldprovider.h
    include/arolib/components/baseroutesinfieldplanner.h
    include/arolib/components/headlandplanner.h
    include/arolib/components/fieldprocessplanner.h
    include/arolib/components/generalrouteplanner.h
    include/arolib/components/machinedb.h
    include/arolib/components/planpostprocessor.h
    include/arolib/components/tracksgenerator.h
    include/arolib/components/workedareamanager.h
    include/arolib/components/gpssimulator.h
    include/arolib/components/componentparametershelper.h
    include/arolib/components/baseroutesprocessor.h
    include/arolib/components/graphprocessor.h
)

#set_property(TARGET ${AROLIB_LIB_NAME} PROPERTY CXX_STANDARD 11)  # 98, 11 or 14
#set_property(TARGET ${AROLIB_LIB_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)



target_include_directories(${AROLIB_LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${AROLIB_LIB_NAME}
    INTERFACE
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
