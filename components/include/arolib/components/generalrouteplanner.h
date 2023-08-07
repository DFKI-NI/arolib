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
 
#ifndef AROLIB_GENERALROUTEPLANNER_H
#define AROLIB_GENERALROUTEPLANNER_H

#include <unistd.h>
#include <iostream>
#include <math.h>
#include <string>
#include <chrono>

#include "arolib/types/machine.hpp"
#include "arolib/types/machinedynamicinfo.hpp"

#include "arolib/misc/container_helper.h"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/field_geometry_processing.hpp"

#include "arolib/planning/path_search/directedgraph.hpp"
#include "arolib/planning/planningworkspace.h"
#include "arolib/planning/planinfo.h"
#include "arolib/planning/edge_calculators/edgeCostCalculator.hpp"
#include "arolib/planning/path_search/astar.hpp"
#include "arolib/planning/path_search/astar_successor_checkers.hpp"

#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/logger.h"
#include "arolib/misc/basic_responses.h"


namespace arolib {

/**
 * @brief Class used for simple route planning (e.g. send a machine to an exit point)
 */
class GeneralRoutePlanner : public LoggingComponent, protected PlanningWorkspaceAccessor
{
public:

    /**
     * @brief Plan parameters
     *
     */
    struct PlanParameters : public Astar::AStarSettings{
        double max_time_visit = -1; /**< If a vertex corresponding to an (unprocessed) processing route-point has a (process) timestamp >= 0 higher than this value, this vertex will be excluded from the search. Disregarded if < 0 */
        double max_time_goal = -1;  /**< Maximum time(-stamp) in which the goal must be reached*/
        std::set<DirectedGraph::vertex_t> exclude = {}; /**< Set of vertices to be excluded from the search path */
        bool includeWaitInCost = true; /**< Should we include the time the machine has to wait to drive over an edge in the edge's cost calculation? */
        std::set<MachineId_t> restrictedMachineIds = {Machine::AllMachineIds};/**< Ids of the machines for which the max_time_visit applies */
    };

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit GeneralRoutePlanner(const LogLevel& logLevel = LogLevel::INFO);


    /**
     * @brief Generate the route for the given machine from its current location to the given location inside the subfield.
     * @param [in/out] graph Graph
     * @param [out] route Planned route
     * @param subfield Processed subfield
     * @param machineCurrentStates Map containing the current states of the machines
     * @param edgeCostCalculator Edge Cost Calculator. Temporary: if = nullptr, uses internal astar functions
     * @param [out] pPlanInfo Pointer to where the information of the final plan will be saved (if = nullptr, it is not saved)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp planRouteToPointInSubfield(DirectedGraph::Graph& graph,
                                       Route& route,
                                       const Subfield& subfield,
                                       const Machine& machine,
                                       const DirectedGraph::vertex_t& goal_vt,
                                       const PlanParameters& planParameters,
                                       const std::map<MachineId_t, MachineDynamicInfo>& machineCurrentStates,
                                       std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                       PlanGeneralInfo *pPlanInfo = nullptr);

    /**
     * @brief Generate the route for the given machine from its current location to the given location inside the subfield.
     * @param [in/out] graph Graph
     * @param [out] route Planned route
     * @param [in/out] pw Planning workspace containing the necessary data (subfield, initial harvester routes, etc.) for planning, as well as the resulting harvester and olv planned routes.
     * @param subfieldIdx Index of the subfield (in pw) that will be planned
     * @param machineId Id of the machine to be sent to the point
     * @param edgeCostCalculator Edge Cost Calculator. Temporary: if = nullptr, uses internal astar functions
     * @param [out] pPlanInfo Pointer to where the information of the final plan will be saved (if = nullptr, it is not saved)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp planRouteToPointInSubfield(DirectedGraph::Graph& graph,
                                        Route& route,
                                        PlanningWorkspace &pw,
                                        size_t subfieldIdx,
                                        const MachineId_t& machineId,
                                        const DirectedGraph::vertex_t& goal_vt,
                                        const PlanParameters& planParameters,
                                        std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                        PlanGeneralInfo *pPlanInfo = nullptr );


    /**
     * @brief Generate the route for the given machine from its current location to the given location inside the subfield.
     * @param [in/out] graph Graph
     * @param [out] route Planned route
     * @param subfield Processed subfield
     * @param machine Machine whose route will be planned
     * @param goal Goal location
     * @param searchRadius Radious around the goal location where the valid (goal) vertex can be located (if < 0 --> computed automatically; if = 0 --> no limit)
     * @param machineCurrentStates Map containing the current states of the machines
     * @param edgeCostCalculator Edge Cost Calculator. Temporary: if = nullptr, uses internal astar functions
     * @param [out] pPlanInfo Pointer to where the information of the final plan will be saved (if = nullptr, it is not saved)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp planRouteToPointInSubfield(DirectedGraph::Graph& graph,
                                       Route& route,
                                       const Subfield& subfield,
                                       const Machine& machine,
                                       const Point& goal,
                                       const PlanParameters& planParameters,
                                       const std::map<MachineId_t, MachineDynamicInfo>& machineCurrentStates,
                                       std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                       double searchRadius = -1,
                                       PlanGeneralInfo *pPlanInfo = nullptr);

    /**
     * @brief Generate the route for the given machine from its current location to the given location inside the subfield.
     * @param [in/out] graph Graph
     * @param [out] route Planned route
     * @param [in/out] pw Planning workspace containing the necessary data (subfield, initial harvester routes, etc.) for planning, as well as the resulting harvester and olv planned routes.
     * @param subfieldIdx Index of the subfield (in pw) that will be planned
     * @param machineId Id of the machine to be sent to the point
     * @param edgeCostCalculator Edge Cost Calculator. Temporary: if = nullptr, uses internal astar functions
     * @param [out] pPlanInfo Pointer to where the information of the final plan will be saved (if = nullptr, it is not saved)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp planRouteToPointInSubfield(DirectedGraph::Graph& graph,
                                       Route& route,
                                       PlanningWorkspace &pw,
                                       size_t subfieldIdx,
                                       const MachineId_t& machineId,
                                       const Point& goal,
                                       const PlanParameters& planParameters,
                                       std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                       double searchRadius = -1,
                                       PlanGeneralInfo *pPlanInfo = nullptr );


    /**
     * @brief Generate route for the given machine(s) from their current location (inside the subfield) to an exit point.
     * @param [in/out] graph Graph
     * @param [out] routes Planned routes
     * @param subfield Processed subfield
     * @param machines Machines to be sent to the exit point
     * @param fapIds Ids of the field access points to be included in the search (if empty, all are included)
     * @param machineCurrentStates Map containing the current states of the machines
     * @param edgeCostCalculator Edge Cost Calculator. Temporary: if = nullptr, uses internal astar functions
     * @param [out] pPlanInfo Pointer to where the information of the final plan will be saved (if = nullptr, it is not saved)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp planRouteToExitPoint(DirectedGraph::Graph& graph,
                                 std::vector<Route>& routes,
                                 const Subfield& subfield,
                                 const std::vector<Machine> &machines,
                                 const std::set<FieldAccessPointId_t> &fapIds,
                                 const PlanParameters& planParameters,
                                 const std::map<MachineId_t, MachineDynamicInfo>& machineCurrentStates,
                                 std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                 PlanGeneralInfo *pPlanInfo = nullptr);

    /**
     * @brief Generate route for the given machine(s) from their current location (inside the subfield) to an exit point.
     * @param [in/out] graph Graph
     * @param [out] routes Planned routes
     * @param [in/out] pw Planning workspace containing the necessary data (subfield, initial harvester routes, etc.) for planning, as well as the resulting harvester and olv planned routes.
     * @param subfieldIdx Index of the subfield (in pw) that will be planned
     * @param machineIds Ids of the machines to be sent to the exit point
     * @param fapIds Ids of the field access points to be included in the search (if empty, all are included)
     * @param edgeCostCalculator Edge Cost Calculator. Temporary: if = nullptr, uses internal astar functions
     * @param [out] pPlanInfo Pointer to where the information of the final plan will be saved (if = nullptr, it is not saved)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp planRouteToExitPoint( DirectedGraph::Graph& graph,
                                  std::vector<Route>& routes,
                                  PlanningWorkspace &pw,
                                  size_t subfieldIdx,
                                  const std::set<MachineId_t> &machineIds,
                                  const std::set<FieldAccessPointId_t> &fapIds,
                                  const PlanParameters& planParameters,
                                  std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                  PlanGeneralInfo *pPlanInfo = nullptr );


    /**
     * @brief Generate route for the given machine(s) from their current location (inside the subfield) to a resource point.
     * @param [in/out] graph Graph
     * @param [out] routes Planned routes
     * @param subfield Processed subfield
     * @param machines Machines to be sent to the exit point
     * @param resourcePointIds Ids of the resource points to be included in the search (if empty, all are included)
     * @param resourceTypes Types of resource points to be included in the search
     * @param machineCurrentStates Map containing the current states of the machines
     * @param edgeCostCalculator Edge Cost Calculator. Temporary: if = nullptr, uses internal astar functions
     * @param [out] pPlanInfo Pointer to where the information of the final plan will be saved (if = nullptr, it is not saved)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp planRouteToResourcePoint(DirectedGraph::Graph& graph,
                                     std::vector<Route>& routes,
                                     const Subfield& subfield,
                                     const std::vector< Machine >& machines,
                                     const PlanParameters& planParameters,
                                     const std::set<ResourcePointId_t> &resourcePointIds,
                                     const std::set<ResourcePoint::ResourceType> &resourceTypes,
                                     const std::map<MachineId_t, MachineDynamicInfo>& machineCurrentStates,
                                     std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                     PlanGeneralInfo *pPlanInfo = nullptr);

    /**
     * @brief Generate route for the given machine(s) from their current location (inside the subfield) to an exit point.
     * @param [in/out] graph Graph
     * @param [out] routes Planned routes
     * @param [in/out] pw Planning workspace containing the necessary data (subfield, initial harvester routes, etc.) for planning, as well as the resulting harvester and olv planned routes.
     * @param subfieldIdx Index of the subfield (in pw) that will be planned
     * @param machineIds Ids of the machines to be sent to the exit point
     * @param resourcePointIds Ids of the resource points to be included in the search (if empty, all are included)
     * @param resourceTypes Types of resource points to be included in the search
     * @param edgeCostCalculator Edge Cost Calculator. Temporary: if = nullptr, uses internal astar functions
     * @param [out] pPlanInfo Pointer to where the information of the final plan will be saved (if = nullptr, it is not saved)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp planRouteToResourcePoint(DirectedGraph::Graph& graph,
                                      std::vector<Route>& routes,
                                      PlanningWorkspace &pw,
                                      size_t subfieldIdx,
                                      const std::set<MachineId_t> &machineIds,
                                      const PlanParameters& planParameters,
                                      const std::set<ResourcePointId_t> &resourcePointIds,
                                      const std::set<ResourcePoint::ResourceType> &resourceTypes,
                                      std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                      PlanGeneralInfo *pPlanInfo = nullptr );

    /**
     * @brief Set the output files for debug and analysis
     * @param filename_graphBuilding File name/path where the gragh-building information will be stored (if empty-string, no data will be saved)
     * @param foldername_planData Folder where the planning (search) information will be stored (if empty-string, no data will be saved)
     * @param foldername_visitSchedule Folder where the initial and final visit schedules will be stored (if empty-string, no data will be saved)
     */
    void setOutputFiles(const std::string& foldername_planData,
                        const std::string& foldername_visitSchedule);

protected:



protected:

    PlanningWorkspace* m_planningWorkspace = nullptr;/**< Pointer to the planning workspace (if NULL, no methods with a planning workspace as parameter were called) */
    size_t m_pw_subfieldIdx; /**< Index of the subfield to be planned (in case the planning workspace is being used (CALC_PW)) */

    std::string m_foldername_planData = "";/**< Folder where the planning (search) information will be stored (if empty-string, no data will be saved) */
    std::string m_foldername_visitSchedule = "";/**< Folder where the initial and final visit schedules will be stored (if empty-string, no data will be saved) */

};

}

#endif // AROLIB_GENERALROUTEPLANNER_H
