set(AROLIB_LIB_NAME "${AROLIB_LIB_PREFIX}${AROLIB_LIB_GEOMETRY}")

add_library(${AROLIB_LIB_NAME} SHARED
  src/boost_geometries_wrapper.cpp
  src/geometry_helper.cpp
  src/field_geometry_processing.cpp
  src/curves_helper.cpp
  src/pathsmoother.cpp
  src//3rdParty/dubins/dubins.cpp
  src//3rdParty/clipper/clipper.cpp
  include/arolib/geometry/geometry.hpp
  include/arolib/geometry/boost_geometries_wrapper.hpp
  include/arolib/geometry/geometry_helper.hpp
  include/arolib/geometry/impl/geometry_helper.tcc
  include/arolib/geometry/field_geometry_processing.hpp
  include/arolib/geometry/curves_helper.hpp
  include/arolib/geometry/pathsmoother.hpp
  include/arolib/geometry/3rdParty/dubins/dubins.h
  include/arolib/geometry/3rdParty/bezier/bezier.h
  include/arolib/geometry/3rdParty/clipper/clipper.hpp
  include/arolib/geometry/3rdParty/spline/splines.hpp
  include/arolib/geometry/3rdParty/spline/splines.inl
)

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
)

install(TARGETS ${AROLIB_LIB_NAME}
        EXPORT ${AROLIB_LIB_NAME}_targets
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib)

add_subdirectory(test)
