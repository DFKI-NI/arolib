set(AROLIB_LIB_NAME "${AROLIB_LIB_PREFIX}${AROLIB_LIB_ANALYSIS}")

add_library(${AROLIB_LIB_NAME} SHARED
    src/plananalyserbase.cpp
    src/harvesterplananalyser.cpp
    src/olvplananalyser.cpp
    include/arolib/analysis/plananalyserbase.hpp
    include/arolib/analysis/harvesterplananalyser.hpp
    include/arolib/analysis/olvplananalyser.hpp
)

#set_target_properties(${AROLIB_LIB_NAME} PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})

target_include_directories(${AROLIB_LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
    INTERFACE
    $<INSTALL_INTERFACE:include>
)

#target_link_libraries(${AROLIB_LIB_NAME}
#    INTERFACE
#      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_TYPES}$<$<CONFIG:Debug>:${CMAKE_DEBUG_POSTFIX}>
#      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_GEOMETRY}$<$<CONFIG:Debug>:${CMAKE_DEBUG_POSTFIX}>
#      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_CARTOGRAPHY}$<$<CONFIG:Debug>:${CMAKE_DEBUG_POSTFIX}>
#      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_IO}$<$<CONFIG:Debug>:${CMAKE_DEBUG_POSTFIX}>
#      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_MISC}$<$<CONFIG:Debug>:${CMAKE_DEBUG_POSTFIX}>
#      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_PLANNING}$<$<CONFIG:Debug>:${CMAKE_DEBUG_POSTFIX}>
#)

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


