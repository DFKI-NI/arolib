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
  src/machinedynamicinfo.cpp
  src/resourcepoint.cpp
  src/fieldaccesspoint.cpp
  src/outfieldinfo.cpp
  src/track.cpp
  src/units.cpp
  src/materialFlowType.cpp
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
  include/arolib/types/route_point.hpp
  include/arolib/types/route.hpp
  include/arolib/types/obstacle.hpp
  include/arolib/types/outfieldinfo.hpp
  include/arolib/types/units.hpp
  include/arolib/types/track.hpp
  include/arolib/types/materialFlowType.hpp
  include/arolib/types/types.hpp
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
    gdal 
    ${Boost_LIBRARIES} 
    ${AROLIB_LIB_PREFIX}${AROLIB_LIB_MISC}
  # PRIVATE libAROLIB_misc.so
)
# INTERFACE 
install(TARGETS ${AROLIB_LIB_NAME}
        EXPORT ${AROLIB_LIB_NAME}_targets
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib)

add_subdirectory(test)
