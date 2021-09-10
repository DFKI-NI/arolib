set(AROLIB_LIB_NAME "${AROLIB_LIB_PREFIX}${AROLIB_LIB_ANALYSIS}")

add_library(${AROLIB_LIB_NAME} SHARED
    src/plananalyserbase.cpp
    src/harvesterplananalyser.cpp
    src/olvplananalyser.cpp
    include/arolib/analysis/plananalyserbase.hpp
    include/arolib/analysis/harvesterplananalyser.hpp
    include/arolib/analysis/olvplananalyser.hpp
)

#set_property(TARGET ${AROLIB_LIB_CARTOGRAPHY} PROPERTY CXX_STANDARD 11)  # 98, 11 or 14
#set_property(TARGET ${AROLIB_LIB_CARTOGRAPHY} PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(${AROLIB_LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
    INTERFACE
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${AROLIB_LIB_NAME}
    INTERFACE
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_TYPES}
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_GEOMETRY}
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_CARTOGRAPHY}
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_IO}
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_MISC}
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_PLANNING}
)

install(TARGETS ${AROLIB_LIB_NAME}
        EXPORT ${AROLIB_LIB_NAME}_targets
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib)


