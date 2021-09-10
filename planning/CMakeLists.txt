set(AROLIB_LIB_NAME "${AROLIB_LIB_PREFIX}${AROLIB_LIB_PLANNING}")

add_library(${AROLIB_LIB_NAME} SHARED
  src/directedgraph.cpp
  src/graph_builder_tracks_based.cpp
  src/graph_builder_routes_based.cpp
  src/graphhelper.cpp
  src/graph_building_info.cpp
  src/simpletracksequencer.cpp
  src/astar.cpp
  src/astarPlan.cpp
  src/simpleBaseRoutesPlanner.cpp
  src/olvPlan.cpp
  src/multiharvplanner.cpp
  src/multiolvplanner.cpp
  src/overloadactivitiesplanner.cpp
  src/aro_functions.cpp
  src/generalplanningparameters.cpp
  src/edgeMassCalculator.cpp
  src/edgeSpeedCalculator.cpp
  src/edgeCostCalculator.cpp
  src/routeassembler.cpp
  src/headlandrouteassembler.cpp
  src/planningworkspace.cpp
  src/planinfo.cpp
  src/roundtripplanner.cpp
  src/activitiesswitchingplanner.cpp
  src/infieldtracksconnector.cpp
  include/arolib/planning/directedgraph.hpp
  include/arolib/planning/graph_builder_tracks_based.hpp
  include/arolib/planning/graph_builder_routes_based.hpp
  include/arolib/planning/graphhelper.hpp
  include/arolib/planning/graph_building_info.hpp
  include/arolib/planning/simpletracksequencer.hpp
  include/arolib/planning/astar.hpp
  include/arolib/planning/astarPlan.hpp
  include/arolib/planning/simpleBaseRoutesPlanner.hpp
  include/arolib/planning/olvPlan.hpp
  include/arolib/planning/multiharvplanner.h
  include/arolib/planning/multiolvplanner.h
  include/arolib/planning/overloadactivitiesplanner.h
  include/arolib/planning/planningException.hpp
  include/arolib/planning/aro_functions.hpp
  include/arolib/planning/generalplanningparameters.hpp
  include/arolib/planning/edgeMassCalculator.hpp
  include/arolib/planning/edgeSpeedCalculator.hpp
  include/arolib/planning/edgeCostCalculator.hpp
  include/arolib/planning/routeassembler.hpp
  include/arolib/planning/headlandrouteassembler.hpp
  include/arolib/planning/planningworkspace.h
  include/arolib/planning/planinfo.h
  include/arolib/planning/roundtripplanner.hpp
  include/arolib/planning/activitiesswitchingplanner.hpp
  include/arolib/planning/infieldtracksconnector.hpp
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
