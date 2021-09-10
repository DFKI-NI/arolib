set(AROLIB_LIB_NAME "${AROLIB_LIB_PREFIX}${AROLIB_LIB_PROCESSING}")

add_library(${AROLIB_LIB_NAME} SHARED
  src/route_smoother.cpp
  src/overrideMapProcessor.cpp
  include/arolib/processing/route_smoother.hpp
  include/arolib/processing/overrideMapProcessor.hpp
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
