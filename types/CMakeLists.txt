set(AROLIB_LIB_NAME "${AROLIB_LIB_PREFIX}${AROLIB_LIB_TYPES}")

add_library(${AROLIB_LIB_NAME} SHARED
  src/coordtransformer.cpp
  src/field.cpp
  src/headland.cpp
  src/point.cpp
  src/pose2D.cpp
  src/route_point.cpp
  src/route.cpp
  src/machine.cpp
  src/resourcepoint.cpp
  src/fieldaccesspoint.cpp
  src/outfieldinfo.cpp
  src/headlandroute.cpp
  src/track.cpp
  src/units.cpp
  include/arolib/types/coordtransformer.hpp
  include/arolib/types/field.hpp
  include/arolib/types/linestring.hpp
  include/arolib/types/machine.hpp
  include/arolib/types/machinedynamicinfo.hpp
  include/arolib/types/point.hpp
  include/arolib/types/pose2D.hpp
  include/arolib/types/resourcepoint.hpp
  include/arolib/types/fieldaccesspoint.hpp
  include/arolib/types/polygon.hpp
  include/arolib/types/subfield.hpp
  include/arolib/types/headland.hpp
  include/arolib/types/headlandpoint.hpp
  include/arolib/types/route_point.hpp
  include/arolib/types/route.hpp
  include/arolib/types/headlandroute.hpp
  include/arolib/types/obstacle.hpp
  include/arolib/types/outfieldinfo.hpp
  include/arolib/types/units.hpp
  include/arolib/types/track.hpp
)

#set_property(TARGET AROLIB_types PROPERTY CXX_STANDARD 11)  # 98, 11 or 14
#set_property(TARGET AROLIB_types PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(${AROLIB_LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${AROLIB_LIB_NAME}
    INTERFACE gdal
)

install(TARGETS ${AROLIB_LIB_NAME}
        EXPORT ${AROLIB_LIB_NAME}_targets
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib)

add_subdirectory(test)
