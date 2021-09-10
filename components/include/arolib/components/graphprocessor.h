/*
 * Copyright 2021  DFKI GmbH
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
 
#ifndef AROLIBGRAPHPROCESSOR_H
#define AROLIBGRAPHPROCESSOR_H

#include <unistd.h>
#include <iostream>
#include <math.h>
#include <string>

#include "arolib/misc/loggingcomponent.h"

#include "arolib/planning/graph_builder_tracks_based.hpp"
#include "arolib/planning/graph_builder_routes_based.hpp"
#include "arolib/planning/planningworkspace.h"
#include "arolib/misc/basic_responses.h"


namespace arolib {

/**
 * @brief Class used to generate the routes/plans for all machines in the harvesting scenario, processing (innitially planned) harvester routes and generating OLV routes based on them
 */
class GraphProcessor : public LoggingComponent, protected PlanningWorkspaceAccessor
{
public:
    /**
     * @brief Settings
     */
    struct Settings{
        double workingWidth = 0; /**< Working width (inner field) */
        double workingWidthHL = 0; /**< Working width (headland) */
        bool bePreciseWithMaps = true; /**< Perform map/grid operations precisely (except for remaining-area map) */
        bool incVisitPeriods = false; /**< Include the visiting periods of the base routes */
        Settings() = default;
    };

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit GraphProcessor(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Generates a graph based on the (processed) base routes and other information.
     * @param subfield Processed subfield
     * @param baseRoutes Base routes of the working (primary) machines
     * @param machines Machines used for planning
     * @param settings Settings
     * @param outFieldInfo Out-of-field information (inc. arrival times, transport times, etc.)
     * @param machineCurrentStates Map containing the current states of the machines
     * @param [out] graph Resulting graph
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp createGraph( const Subfield& subfield,
                         const std::vector<arolib::Route> &baseRoutes,
                         const std::vector<arolib::Machine>& machines,
                         const Settings& settings,
                         OutFieldInfo &outFieldInfo,
                         const std::map<MachineId_t, arolib::MachineDynamicInfo>& machineCurrentStates,
                         DirectedGraph::Graph& graph);

    /**
     * @brief Generates a graph based on the (processed) base routes and other information.
     * @param [in/out] pw Planning workspace containing the necessary data (subfield, initial base routes, etc.) for planning, as well as the resulting harvester and olv planned routes.
     * @param subfieldIdx Index of the subfield (in pw) that will be planned
     * @param settings Settings
     * @param [out] graph Resulting graph
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp createGraph(PlanningWorkspace &pw, size_t subfieldIdx, const Settings& settings, DirectedGraph::Graph &graph);

    /**
     * @brief Generates a graph based on the (processed) base routes and other information.
     * @param subfield Processed subfield
     * @param baseRoutes_headland Headland processed base routes
     * @param baseRoutes_infield Inner-field processed base routes
     * @param machines Machines used for planning
     * @param settings Settings
     * @param outFieldInfo Out-of-field information (inc. arrival times, transport times, etc.)
     * @param machineCurrentStates Map containing the current states of the machines
     * @param [out] graph Resulting graph
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp createGraph_old( const Subfield& subfield,
                         const std::vector<arolib::HeadlandRoute> &baseRoutes_headland,
                         const std::vector<arolib::Route> &baseRoutes_infield,
                         const std::vector<arolib::Machine>& machines,
                         const Settings& settings,
                         OutFieldInfo &outFieldInfo,
                         const std::map<MachineId_t, arolib::MachineDynamicInfo>& machineCurrentStates,
                         DirectedGraph::Graph& graph);

    /**
     * @brief Generates a graph based on the (processed) base routes and other information.
     * @param [in/out] pw Planning workspace containing the necessary data (subfield, base routes routes, etc.).
     * @param subfieldIdx Index of the subfield (in pw)
     * @param settings Settings
     * @param [out] graph Resulting graph
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp createGraph_old( PlanningWorkspace &pw,
                         size_t subfieldIdx,
                         const Settings& settings,
                         DirectedGraph::Graph& graph);

    /**
     * @brief Generates a graph based on the (processed) subfield and worked area map.
     *
     * The track vertex-points will have a timestamp = -1 if it is processed (available), and very high it it is not
     * @param subfield Processed subfield
     * @param settings Settings
     * @param outFieldInfo Out-of-field information (inc. arrival times, transport times, etc.)
     * @param machineCurrentStates Map containing the current states of the machines
     * @param soilmap Soil-state map/grid
     * @param remainingAreaMap Remaining-area map map/grid
     * @param [out] graph Resulting graph
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp createSimpleGraph( const Subfield& subfield,
                               const std::vector<arolib::Machine>& machines,
                               const Settings& settings,
                               OutFieldInfo &outFieldInfo,
                               const std::map<MachineId_t, arolib::MachineDynamicInfo>& machineCurrentStates,
                               const ArolibGrid_t &remainingAreaMap,
                               DirectedGraph::Graph& graph);

    /**
     * @brief Generates a graph based on the (processed) subfield and worked area map.
     * @param [in/out] pw Planning workspace containing the necessary data (subfield, maps, etc.).
     * @param subfieldIdx Index of the subfield (in pw)
     * @param settings Settings
     * @param [out] graph Resulting graph
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp createSimpleGraph( PlanningWorkspace &pw,
                               size_t subfieldIdx,
                               const Settings& settings,
                               DirectedGraph::Graph& graph);


    /**
     * @brief Adds (or replace if existent) the vertices and corresponding edges corresponding to the initial (current) locations of the machines.
     *
     * @param [in/out] graph Graph to be edited
     * @param subfield Processed subfield
     * @param settings Settings
     * @param outFieldInfo Out-of-field information (inc. arrival times, transport times, etc.)
     * @param machineCurrentStates Map containing the current states of the machines
     * @param soilmap Soil-state map/grid
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp addInitialPositons(DirectedGraph::Graph& graph,
                               const Subfield& subfield,
                               const std::vector<arolib::Machine>& machines,
                               OutFieldInfo &outFieldInfo,
                               const std::map<MachineId_t, arolib::MachineDynamicInfo>& machineCurrentStates,
                               const Polygon &field_bounday,
                               bool locateMissingMachinesAtFAP = false);


    /**
     * @brief Set the output files for debug and analysis
     * @param filename_graphBuilding File name/path where the gragh-building information will be stored (if empty-string, no data will be saved)
     */
    void setOutputFiles(const std::string& filename_graphBuilding);

protected:
    /**
     * @brief Inserts an edge(s) connecting two vertices
     * @note In the case of headland harvesting (headland subgraph), the headland vertices correspond to the (field) boundary points.
     * @param [in/out] graph Graph to be expanded/updated
     * @param distance Edge distance.
     * @param yield Yield-mass under the edge [Kg].
     * @param edgeType Edge type.
     * @param vt1 First vertex to be connected.
     * @param vt2 Second vertex to be connected.
     * @param rp1 Route-point corresponding to the first vertex to be connected (vt1).
     * @param rp2 Route-point corresponding to the second vertex to be connected (vt2).
     * @param width Edge width.
     * @param addedTag Tag indicating 'when' was the edge inserted (for debug and analysis).
     * @param bidirectional Is the connection bidirectionsl? If true, two edges will be created: one from u to v, and another one from v to u.
     * @param onlyIfNotExisting If the edge exists, it is unchanged
     * @param overwrite (if the edge exists) if overwrite=true, the edge_prop is replaced; otherwise a new edge is created;
     * @return true if added;
     */
    bool addEdge(DirectedGraph::Graph &graph,
                 DirectedGraph::EdgeType edgeType,
                 const DirectedGraph::vertex_t &vt1,
                 const DirectedGraph::vertex_t &vt2,
                 const RoutePoint &rp1,
                 const RoutePoint &rp2,
                 double distance,
                 double width,
                 bool bidirectional = true,
                 bool onlyIfNotExisting = true,
                 bool overwrite = false);

    /**
     * @brief Generate some pseudo base routes based on the subfield geometry and worked area
     * @param subfield subfield
     * @param settings settings
     * @param remainingAreaMap Remaining-area map map/grid
     * @param [out] Generated headland routes
     * @param [out] Generated inner-field routes
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generatePseudoRoutes(const Subfield& subfield,
                                 const Settings& settings,
                                 const ArolibGrid_t &remainingAreaMap,
                                 arolib::HeadlandRoute& headlandRoute,
                                 arolib::Route& infieldRoute) const;

protected:
    mutable PlanningWorkspace* m_planningWorkspace = nullptr;/**< Pointer to the planning workspace (if NULL, no methods with a planning workspace as parameter were called) */
    mutable size_t m_pw_subfieldIdx; /**< Index of the subfield to be planned (in case the planning workspace is being used */

    std::string m_filename_graphBuilding = "";/**< File name/path where the gragh-building information will be stored (if empty-string, no data will be saved) */
};

}

#endif // AROLIBGRAPHPROCESSOR_H
