set(AROLIB_LIB_NAME "${AROLIB_LIB_PREFIX}${AROLIB_LIB_PLANNING}")

add_library(${AROLIB_LIB_NAME} SHARED
  src/simpleBaseRoutesPlanner.cpp
  src/olvPlan.cpp
  src/route_planner_standalone_machines.cpp
  src/multiolvplanner.cpp
  src/overloadactivitiesplanner.cpp
  src/aro_functions.cpp
  src/generalplanningparameters.cpp
  src/routeassembler.cpp
  src/planningworkspace.cpp
  src/planinfo.cpp
  src/roundtripplanner.cpp
  src/activitiesswitchingplanner.cpp
  src/transit_restrictions.cpp
  src/edge_calculators/edgeMassCalculator.cpp
  src/edge_calculators/edgeSpeedCalculator.cpp
  src/edge_calculators/edgeCostCalculator.cpp
  src/edge_calculators/edgeCostCalculatorSoil.cpp
  src/path_search/astar.cpp
  src/path_search/astarPlan.cpp
  src/path_search/astar_successor_checkers.cpp
  src/path_search/directedgraph.cpp
  src/path_search/graph_builder_tracks_based.cpp
  src/path_search/graphhelper.cpp
  src/path_search/graph_building_info.cpp
  src/track_connectors/infieldtracksconnector.cpp
  src/track_connectors/infieldtracksconnectordef.cpp
  src/track_connectors/infieldtracksconnectordubins.cpp
  src/track_connectors/infieldtracksconnectorgraphbased.cpp
  src/track_connectors/infieldtracksconnectordubinsorgraphbased.cpp
  src/track_sequencing/tracksequencer.cpp
  src/track_sequencing/simpletracksequencer.cpp
  src/track_sequencing/tracksequencerclosestnext.cpp
  src/track_sequencing/tracksequenceradjacentnext.cpp
  src/track_sequencing/tracksequencerspiral.cpp
  include/arolib/planning/simpleBaseRoutesPlanner.hpp
  include/arolib/planning/olvPlan.hpp
  include/arolib/planning/route_planner_standalone_machines.hpp
  include/arolib/planning/multiolvplanner.h
  include/arolib/planning/overloadactivitiesplanner.h
  include/arolib/planning/planningException.hpp
  include/arolib/planning/aro_functions.hpp
  include/arolib/planning/generalplanningparameters.hpp
  include/arolib/planning/routeassembler.hpp
  include/arolib/planning/planningworkspace.h
  include/arolib/planning/planinfo.h
  include/arolib/planning/roundtripplanner.hpp
  include/arolib/planning/activitiesswitchingplanner.hpp
  include/arolib/planning/transit_restrictions.hpp
  include/arolib/planning/edge_calculators/edgeMassCalculator.hpp
  include/arolib/planning/edge_calculators/edgeSpeedCalculator.hpp
  include/arolib/planning/edge_calculators/edgeCostCalculator.hpp
  include/arolib/planning/edge_calculators/edgeCostCalculatorSoil.hpp
  include/arolib/planning/path_search/astar.hpp
  include/arolib/planning/path_search/astarPlan.hpp
  include/arolib/planning/path_search/astar_successor_checkers.hpp
  include/arolib/planning/path_search/directedgraph.hpp
  include/arolib/planning/path_search/graph_builder_tracks_based.hpp
  include/arolib/planning/path_search/graphhelper.hpp
  include/arolib/planning/path_search/graph_building_info.hpp
  include/arolib/planning/track_connectors/infieldtracksconnector.hpp
  include/arolib/planning/track_connectors/infieldtracksconnectordef.hpp
  include/arolib/planning/track_connectors/infieldtracksconnectordubins.hpp
  include/arolib/planning/track_connectors/infieldtracksconnectorgraphbased.hpp
  include/arolib/planning/track_connectors/infieldtracksconnectordubinsorgraphbased.hpp
  include/arolib/planning/track_sequencing/tracksequencer.hpp
  include/arolib/planning/track_sequencing/simpletracksequencer.hpp
  include/arolib/planning/track_sequencing/tracksequencerclosestnext.hpp
  include/arolib/planning/track_sequencing/tracksequenceradjacentnext.hpp
  include/arolib/planning/track_sequencing/tracksequencerspiral.hpp
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
