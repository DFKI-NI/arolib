set(AROLIB_LIB_NAME "${AROLIB_LIB_PREFIX}${AROLIB_LIB_PROCESSING}")

add_library(${AROLIB_LIB_NAME} SHARED
  src/route_smoother.cpp
  src/overrideMapProcessor.cpp
  include/arolib/processing/route_smoother.hpp
  include/arolib/processing/overrideMapProcessor.hpp
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
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_CARTOGRAPHY}
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_GEOMETRY}
      ${AROLIB_LIB_PREFIX}${AROLIB_LIB_MISC}
)

install(TARGETS ${AROLIB_LIB_NAME}
        EXPORT ${AROLIB_LIB_NAME}_targets
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib)

add_subdirectory(test)
