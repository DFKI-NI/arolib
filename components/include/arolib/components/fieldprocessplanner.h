/*
 * Copyright 2023  DFKI GmbH
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License
*/
 
#ifndef AROLIB_FIELDPROCESSPLANNER_H
#define AROLIB_FIELDPROCESSPLANNER_H

#include <unistd.h>
#include <iostream>
#include <math.h>
#include <string>
#include <memory>

#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/container_helper.h"
#include "arolib/misc/basic_responses.h"
#include "arolib/types/machine.hpp"
#include "arolib/types/machinedynamicinfo.hpp"
#include "arolib/geometry/field_geometry_processing.hpp"

#include "arolib/planning/generalplanningparameters.hpp"
#include "arolib/planning/planinfo.h"
#include "arolib/planning/multiolvplanner.h"
#include "arolib/planning/route_planner_standalone_machines.hpp"
#include "arolib/planning/path_search/graphhelper.hpp"
#include "arolib/planning/track_connectors/infieldtracksconnectordef.hpp"
#include "arolib/planning/path_search/astar_successor_checkers.hpp"

namespace arolib {

/**
 * @brief Class used to generate the routes/plans for all machines in the harvesting scenario, taking (innitially planned) harvester routes and generating OLV routes based on them
 */
class FieldProcessPlanner : public LoggingComponent, protected PlanningWorkspaceAccessor
{
public:

    /**
     * @brief Planner parameters
     *
     * Inherits from MultiOLVPlanner::PlannerSettings
     * @sa MultiOLVPlanner::PlannerSettings
     * @warning For MultiOLVPlanner::PlannerSettings: if harvestedMassLimit if < 0 and numOverloadActivities <= 0, an optimal value will be calculated
     * @sa MultiOLVPlanner::PlannerSettings
     */
    struct PlannerParameters
            : public virtual FieldGeneralParameters,
              public virtual GridComputationSettings,
              public virtual RoutePlannerStandaloneMachines::PlannerSettings,
              public virtual MultiOLVPlanner::PlannerSettings{
        double unloading_offset = 300; /**< Default time a machine spends at the resource point (e.g., unloading crop in the silo) */
        double workingWidth = 0; /**< Working width */
        /**
         * @brief Parse the parameters from a string map, starting from a default PlannerParameters
         * @param [out] param Parameters
         * @param map String map containing the parameter values
         * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the default values will remain)
         * @return True on success
         */
        static bool parseFromStringMap( PlannerParameters& params, const std::map<std::string, std::string>& map, bool strict = false);

        /**
         * @brief Parse the parameters to a string map
         * @param param Parameters
         * @return String map containing the parameter values
         */
        static std::map<std::string, std::string> parseToStringMap( const PlannerParameters& params);
    };

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit FieldProcessPlanner(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Generate the machine routes (complete plan) based on initial base routes for infield and headland.
     *
     * @param [in/out] graph Graph
     * @param subfield Processed subfield
     * @param baseRoutes Processed base routes (must have increasingly monotonic timestamps)
     * @param [out] plannedRoutes Generated routes for all machines
     * @param machines Machines used for planning
     * @param outFieldInfo Out-of-field information (inc. arrival times, transport times, etc.)
     * @param machineCurrentStates Map containing the current states of the machines
     * @param plannerParameters Planner parameters
     * @param yieldmap Yield-proportion map/grid (values in t/ha)
     * @param remainingAreaMap Remaining (unharvested) -area map/grid
     * @param edgeCostCalculator Edge Cost Calculator. Temporary: if = nullptr, uses internal astar functions
     * @param [out] pPlanInfo Pointer to where the information of the final plan will be saved (if = nullptr, it is not saved)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp planSubfield(DirectedGraph::Graph& graph,
                         const Subfield& subfield,
                         const std::vector<arolib::Route> &baseRoutes_,
                         std::vector<arolib::Route> &plannedRoutes,
                         const std::vector<arolib::Machine>& machines,
                         const OutFieldInfo &outFieldInfo,
                         const std::map<MachineId_t, arolib::MachineDynamicInfo>& machineCurrentStates,
                         const PlannerParameters &_plannerParameters,
                         const ArolibGrid_t &yieldmap,
                         const ArolibGrid_t &remainingAreaMap,
                         std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                         PlanGeneralInfo *pPlanInfo = nullptr);

    /**
     * @brief Set the output files for debug and analysis
     * @param foldername_planData Folder where the planning (search) information will be stored (if empty-string, no data will be saved)
     * @param foldername_visitSchedule Folder where the initial and final visit schedules will be stored (if empty-string, no data will be saved)
     */
    void setOutputFiles(const std::string& foldername_planData,
                        const std::string& foldername_visitSchedule);

protected:
    /**
     * @brief Do route planning for the scenario where only (capacitated) primary machines are participating
     * @param baseRoutes Base routes
     * @param machines Machines
     * @param machineCurrentStates Current states of the machines
     * @param plannerParameters Planner parameters
     * @param edgeCostCalculator Edge Cost Calculator.
     * @param materialFlowType Material flow type
     * @param transitRestriction Transit restriction inside field
     * @param logger Logger
     * @param [out] out_mainRoutes Planned routes
     * @param [out] out_updatedGraph Updated graph
     * @param [out] out_pPlanInfo Pointer to where the information of the final plan will be saved (if = nullptr, it is not saved)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp do_planningForStandaloneMachines(const DirectedGraph::Graph originalGraph,
                                             const Polygon& boundary,
                                             const std::vector<Route> &baseRoutes,
                                             const std::vector<Machine> &machines,
                                             const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                             const RoutePlannerStandaloneMachines::PlannerSettings& plannerParameters,
                                             const std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                             MaterialFlowType materialFlowType,
                                             TransitRestriction transitRestriction,
                                             Logger& logger,
                                             std::vector<Route>& out_mainRoutes,
                                             DirectedGraph::Graph &out_updatedGraph,
                                             PlanGeneralInfo *out_pPlanInfo = nullptr);

    /**
     * @brief Do route planning for the scenario where (non-capacitated) primary machines and overload vehicles are participating
     * @param baseRoutes Base routes of the primary machines
     * @param machines Machines
     * @param machineCurrentStates Current states of the machines
     * @param plannerParameters Planner parameters
     * @param edgeCostCalculator Edge Cost Calculator. Temporary: if = nullptr, uses internal astar functions
     * @param logger Logger
     * @param [out] out_harvRoutes Planned routes of the primary machines
     * @param [out] out_olvRoutes Planned routes of the overload vehicles
     * @param [out] out_updatedGraph Updated graph
     * @param [out] out_pPlanInfo Pointer to where the information of the final plan will be saved (if = nullptr, it is not saved)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp do_multiOLVPlanning(const DirectedGraph::Graph originalGraph,
                                const std::vector<Route> &baseRoutes, 
                                const std::vector<Machine> &machines, 
                                const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates, 
                                const MultiOLVPlanner::PlannerSettings& plannerParameters, 
                                const std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                Logger& logger, 
                                std::vector<Route>& out_harvRoutes, 
                                std::vector<Route>& out_olvRoutes, 
                                DirectedGraph::Graph &out_updatedGraph, 
                                PlanGeneralInfo *out_pPlanInfo = nullptr);

    /**
     * @brief Check if the given machines are valid based on their type
     * @param machines Machines
     * @param [out] workingMachines Working machines
     * @param [out] numOLVs Number of OLVs
     * @param [out] materialFlowType Based on the working machines' types
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp checkMachineTypes(const std::vector<Machine> &machines, std::vector<Machine> &workingMachines, size_t &numOlvs,
                              MaterialFlowType& materialFlowType, TransitRestriction& transitRestriction);

    /**
     * @brief Adjust the arrival times of the out-of-field information
     *
     * The arrival-times of the olvs are adjusted so that they do not arrive to the field before the first (base) machine arrives
     * Might be deprecated and removed in the future
     * @param outFieldInfo_in Out-of-field information containing the original arrival times
     * @param fieldAccessPoints Field access points
     * @param machines Machines used for planning
     * @param baseRoutes_infield Inner-field base routes (before planning process routes)
     * @param baseRoutes_headland Headland base routes (before planning process routes)
     * @param machineCurrentStates Map containing the current states of the machines
     * @param subfield Processed subfield
     * @param [out] deltaTime Time needed by the (base) machines to arrive
     * @return Out-of-field information containing the adjusted arrival times
     */
    OutFieldInfo adjustOutFieldInfoArrivalTimes(const OutFieldInfo& outFieldInfo_in,
                                                const std::vector<FieldAccessPoint> &fieldAccessPoints,
                                                const std::vector<Machine> &machines,
                                                const std::vector<arolib::Route> &baseRoutes_infield,
                                                const std::vector<arolib::Route> &baseRoutes_headland,
                                                const std::map<MachineId_t, arolib::MachineDynamicInfo>& machineCurrentStates,
                                                const Subfield &subfield,
                                                double &deltaTime);

    /**
     * @brief Pre-process the initial headland and inner-field base routes before planning
     *
     * Sets the timestamp of the harvested route points to -1 (the points in the route, starting by the firts one, until there is something to harvest), based on yield to harvest and remainin-area map
     * If there is nothing to harvest in a route, the route is removed
     * @param [in/out] baseRoutes_infield Inner-field base routes (before planning process routes) to be processed.
     * @param [in/out] baseRoutes_headland Headland base routes (before planning process routes) to be processed.
     * @param subfield Processed subfield
     * @param machines Machines used for planning
     * @param outFieldInfo Out-of-field information (inc. arrival times, transport times, etc.)
     * @param machineCurrentStates Map containing the current states of the machines
     * @param remainingAreaMap Remaining (unharvested) -area map/grid
     * @param calcRemainingAreaPrecisely Perform map/grid operations precisely (ONLY for remaining-area map)
     */
    void preProcessBaseRoutes(std::vector<arolib::Route> &baseRoutes_infield,
                                   std::vector<arolib::Route> &baseRoutes_headland,
                                   const Subfield& subfield,
                                   const std::vector<Machine> &machines,
                                   const OutFieldInfo& outFieldInfo,
                                   const std::map<MachineId_t, arolib::MachineDynamicInfo>& machineCurrentStates,
                                   const ArolibGrid_t &remainingAreaMap,
                                   bool calcRemainingAreaPrecisely);

    /**
     * @brief Connect the (initial, pre-processed) headland and inner-field base routes
     *
     * The timestamps and other route-point parameters are adjusted correspondingly (also in baseRoutes_headland and baseRoutes_infield)
     * @param [in/out] baseRoutes_infield Inner-field harvester routes (before planning process routes) to be processed.
     * @param [in/out] baseRoutes_headland Headland harvester routes (before planning process routes) to be processed.
     * @param subfield Processed subfield
     * @param machines Machines used for planning
     * @return Connected and adjusted base routes
     */
    std::vector<Route> connectBaseRoutes(std::vector<Route> &baseRoutes_infield,
                                              std::vector<Route> &baseRoutes_headland,
                                              const Subfield& subfield,
                                              const std::vector<Machine> &machines);

    /**
     * @brief Check if the planned OLV routes concurr with the planned harvester routes
     * @param machines Machines used for planning
     * @param harvesterRoutes Harvester routes after overload planning.
     * @param OLVRoutes OLV routes after overload planning.
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp checkOLVRoutes(const std::vector<Machine>& machines,
                                 const std::vector<Route> &harvesterRoutes,
                                 const std::vector<Route> &OLVRoutes);

    /**
     * @brief Estimate the initial entry point used during initial harvester-route planning
     *
     * The estimation is done based on the distance of the access points to the first route point
     * Might be deprecated and removed in the future
     * @param outFieldInfo Out-of-field information (inc. arrival times)
     * @param fieldAccessPoints Field access points
     * @param workingGroup Machines used for planning
     * @param baseRoutes_infield Inner-field base routes (before planning process routes)
     * @param baseRoutes_headland Headland base routes (before planning process routes)
     * @param [out] initialFAP Estimated initial field access point
     * @param [out] maxArrivalTime Maximum arrival time of the (base) machines to the estimated field access point
     * @return True on success
     */
    bool getInitialEntryPoint(const OutFieldInfo& outFieldInfo,
                              const std::vector<FieldAccessPoint> &fieldAccessPoints,
                              const std::vector<Machine> &workingGroup,
                              const std::vector<Route> &baseRoutes_infield,
                              const std::vector<Route> &baseRoutes_headland,
                              FieldAccessPoint & initialFAP,
                              double &maxArrivalTime);

    /**
     * @brief Connect the (planned) main (working machine) routes to the best exit points using astar search (unless useSearch = false)
     *
     * If useSearch = false, or if the search for the final route-segments is unsuccessfull, it calculates the final route-segments geometrically
     * @param [in/out] graph Updated graph (to be updated with final final route-segments, if applicable)
     * @param [in/out] mainRoutes Harvester routes (after process planning) to be connected to the exit points
     * @param subfield Processed subfield
     * @param machines Machines used for planning
     * @param useSearch Should the final route-segments be calculated using astar search i and the given graph?
     */
    void sendWorkingMachinesToExitPoints(DirectedGraph::Graph & graph,
                                         std::vector<arolib::Route> &mainRoutes,
                                         const Subfield& subfield,
                                         const std::vector<Machine> &machines,
                                         const PlannerParameters& plannerParameters,
                                         std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                         bool useSearch = false);

    /**
     * @brief Connect a (planned) working-machine route to the best exit point using the geometry of the field
     * @param [in/out] route Working-machine route (after process planning) to be connected to the exit point
     * @param subfield Processed subfield
     * @param machine Machine for the given route
     */
    void sendWorkingMachinesToExitPoint(Route &route,
                                        const Subfield &subfield,
                                        const Machine &machine);


    /**
     * @brief Obtain (geometrically) the best path connecting the (working) machine initial point/location with the first (valid) working point of its headland route
     * @param route Route
     * @param subfield Processed subfield
     * @param initialPoint Initial working point / location
     * @param machine Machine for the given route
     * @return Resulting path points
     */
    std::vector<Point> getBestPathToFirstWorkingPoint_hl(const Route& route,
                                                      const Subfield& subfield,
                                                      const Point &initialPoint,
                                                      const Machine &machine);

    /**
     * @brief Obtain (geometrically) the best path connecting the (working) machine initial point/location with the first (valid) working point of its inner-field route
     * @param route Route
     * @param subfield Processed subfield
     * @param initialPoint Initial working point / location
     * @param machine Machine for the given route
     * @return Resulting path points
     */
    std::vector<Point> getBestPathToFirstWorkingPoint_if(const Route& route,
                                                         const Subfield& subfield,
                                                         const Point &initialPoint,
                                                         const Machine &machine);

    /**
     * @brief Make initial adjustments to base routes and graph based on the machineCurrentStates
     *
     * Removes initial segments
     * Adjust timestamps of the base route points and corresponding vertices
     * @param [in/out] graph Updated graph (to be updated)
     * @param [in/out] baseRoutes Base/main (working-machine) routes. Their parameters (timestamps, etc.) are updated on success
     * @param machineCurrentStates Map containing the current states of the machines
     */
    void adjustBaseRoutes(DirectedGraph::Graph &graph,
                          std::vector<Route> &baseRoutes,
                          const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates);

    /**
     * @brief Obtain (using a-star search) the best route-segments connecting the (working) machines' initial points/locations with the first (valid) working point of their routes
     * @param [in/out] graph Updated graph (to be updated with final initial route-segments)
     * @param [in/out] baseRoutes Base/main (working-machine) routes. Their parameters (timestamps, etc.) are updated on success
     * @param subfield Processed subfield
     * @param machines Machines used for planning
     * @param machineCurrentStates Map containing the current states of the machines
     * @param plannerParameters Planner parameters
     * @return Map with the resulting initial route-segments (one per machine)
     */
    std::map<MachineId_t, Route> getInitialPathToWorkingRoutes(DirectedGraph::Graph & graph,
                                                               std::vector<arolib::Route> &baseRoutes,
                                                               const Subfield& subfield,
                                                               const std::vector<Machine> &machines,
                                                               const std::map<MachineId_t, arolib::MachineDynamicInfo>& machineCurrentStates,
                                                               const PlannerParameters& plannerParameters,
                                                               std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator);
    /**
     * @brief Adds the visit periods of the working-machines (main routes) to the coppesponding vertices
     * @param [in/out] graph Updated graph (to be updated with final initial route-segments)
     * @param wmRoutes Processed routes.
     * @param machines Machines.
     */
    void addWMVisitPeriods(DirectedGraph::Graph & graph,
                           const std::vector<arolib::Route> &mainRoutes,
                           const std::vector<Machine> &machines);

    /**
     * @brief Get the set of exclude vertices to be used during the search of best route-segments connecting the working-machines' initial points/locations with the first (valid) working point of their routes
     * @param graph Updated graph
     * @param goal_vt Goal vertex
     * @return Resulting set of exclude vertices
     */
    std::set<DirectedGraph::vertex_t> getExcludeVertices_initMainRoute(DirectedGraph::Graph & graph, DirectedGraph::vertex_t goal_vt);

    /**
     * @brief Adds the initial route-segments to the working-machine routes
     * @param routesInit Map with the initial route-segments
     * @param [in/out] mainRoutes Routes to which the initial route-segments will be appended
     */
    void addInitialPathToMainRoutes(const std::map<MachineId_t, Route> & routesInit,
                                         std::vector<arolib::Route> &mainRoutes);

    /**
     * @brief Adds a given time to the timestamps of the routes (shifts the routes in time)
     * @param [in/out] mainRoutes Main (workig-machine) routes to be updated
     * @param [in/out] secRoutes Secondary (OLV, ...) routes to be updated
     * @param deltaTime Time to be shifted
     */
    void shiftRoutesInTime(std::vector<Route> &mainRoutes,
                           std::vector<Route> &secRoutes,
                           double deltaTime);

    /**
     * @brief If applicable, adds an INITIAL_POSITION route point at the begining of the main routes
     * @param [in/out] mainRoutes Mai (working machine) routes to be updated
     * @param machineCurrentStates Map containing the current states of the machines
     */
    void addInitialPointsToMainRoutes(std::vector<Route> &mainRoutes,
                                          const std::map<MachineId_t, MachineDynamicInfo>& machineCurrentStates);

    /**
     * @brief Check if at least one of the working machines is outside the given boundary
     * @param machines Machines
     * @param machineCurrentStates Map containing the current states of the machines
     * @param field_Boundary Field boundary
     * @return True if at least one of the machines is outside the given boundary
     */
    bool areWorkingMachinesOutOfField(const std::vector<Machine> &machines,
                                      const std::map<MachineId_t, MachineDynamicInfo>& machineCurrentStates,
                                      const Polygon &field_Boundary);

    /**
     * @brief Remove the initial points of the main routes where there is no work to be done (e.g. is harvested)
     * @param [in/out] mainRoutes Main (base) routes from working machines to be updated
     * @return Amount of poits removed from all routes
     */
    size_t removeInitialUselessPoints(std::vector<Route> &mainRoutes);

    /**
     * @brief Obtain the points (path) between the given indexes (excludes point at indEnd). The path stops when the timestamp of a point in the range is > timeLimit
     *
     * Allows reverse order (indBegin > indEnd).
     * @param routePoints Points from which the path will be obtained
     * @param indBegin Index of the first point
     * @param indEnd Index of the last point (excluded from the resulting path)
     * @param timeLimit Time used to stop the path (the path stops when the timestamp of a point in the range is > timeLimit)
     * @return Resulting path
     */
    std::vector<Point> getPathInTrack(const std::vector<RoutePoint> &routePoints,
                                      int indBegin,
                                      int indEnd,
                                      double timeLimit = -0.0001);


    /**
     * @brief Calculates the optimal parameter 'harvestedMassLimit' based on the fields yield proportion and the already harvested area
     *
     * If the overload activity surpaces this value (harvestedMassLimit), no more overload activities will be processed
     * Used in overload planning
     * @param subfield Processed subfield
     * @param machines Machines to be used for planning
     * @param plannerParameters Planner parameters
     * @param yieldmap Yield-proportion map/grid (values in t/ha)
     * @param remainingAreaMap Remaining (unharvested) -area map/grid
     * @return Calculated mass limit [Kg]; -1 if there is no limit
     */
    double getHarvestedMassLimit(const Subfield& subfield,
                                 const std::vector<arolib::Machine>& machines,
                                 const PlannerParameters& plannerParameters,
                                 const ArolibGrid_t &yieldmap,
                                 const ArolibGrid_t &remainingAreaMap);
    /**
     * @brief Add edge overruns to graph for given routes
     * @param graph [in/out] graph
     * @param machines machines
     * @param routes routes
     * @return Map of the machines (working group)
     */
    void addRoutesOverruns(DirectedGraph::Graph & graph,
                           const std::vector<arolib::Machine>& machines,
                           const std::vector<arolib::Route>& routes);

    /**
     * @brief Saves the visit schedule in m_foldername_visitSchedule with the given filename (iif m_foldername_visitSchedule is set)
     * @param graph Updated graph
     * @param filename Name of the file (.csv)
     */
    void saveVisitSchedule( const DirectedGraph::Graph & graph, const std::string& filename );


protected:

    std::string m_foldername_planData = "";/**< Folder where the planning (search) information will be stored (if empty-string, no data will be saved) */
    std::string m_foldername_visitSchedule = "";/**< Folder where the initial and final visit schedules will be stored (if empty-string, no data will be saved) */

    const double m_partialPlanAreaThreshold = 50000;/**< Area-threshold to decide whether a partial-plan must be carried out, or the plan for the complete field is to be done */
};

}

#endif // AROLIB_FIELDPROCESSPLANNER_H
