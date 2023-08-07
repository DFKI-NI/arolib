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
 
#ifndef AROLIBGRAPHPROCESSOR_H
#define AROLIBGRAPHPROCESSOR_H

#include <unistd.h>
#include <iostream>
#include <math.h>
#include <string>

#include "arolib/misc/basic_responses.h"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/cartography/sharedgridsmanager.hpp"
#include "arolib/planning/path_search/graph_builder_tracks_based.hpp"
#include "arolib/planning/planningworkspace.h"


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

        /**
         * @brief Parse the parameters from a string map, starting from a default Settings
         * @param [out] param Settings
         * @param map String map containing the parameter values
         * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the default values will remain)
         * @return True on success
         */
        static bool parseFromStringMap( Settings& params, const std::map<std::string, std::string>& map, bool strict = false);

        /**
         * @brief Parse the parameters to a string map
         * @param param Parameters
         * @return String map containing the parameter values
         */
        static std::map<std::string, std::string> parseToStringMap(const Settings &params);
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

    /**
     * @brief Edit the vertex information of vertices corresponding to points of the ẃorking routes (incl. timestamp) in a graph where no routes info has been added before
     * @param [in/out] graph graph
     * @param machines Machines corresponding to the routes
     * @param baseWorkingRoutes Routes
     * @param includeVisitPeriods Include visit periods of the base routes? (including them might not be necesary if no collition avoidance will be carried out)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp addBaseRouteInformationToGraph(DirectedGraph::Graph & graph,
                                           const std::vector<Machine> &machines,
                                           const std::vector<Route> &baseWorkingRoutes,
                                           bool includeVisitPeriods = true) const;

    /**
     * @brief Edit the vertex information of vertices corresponding to points of the ẃorking routes (incl. timestamp) in a graph where no routes info has been added before
     * @param [in/out] graph graph
     * @param remainingAreaMap Remaining-area gridmap
     * @param resetIfWorked The timestamps of the vertices considered worked (true) or not worked (false) will be reset
     * @param cim (optinal) CellsInfoManager
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp resetVertexTimestampsFromWorkedArea(DirectedGraph::Graph & graph,
                                                std::shared_ptr<const ArolibGrid_t> remainingAreaMap,
                                                bool resetIfWorked,
                                                std::shared_ptr<gridmap::GridCellsInfoManager> cim = nullptr) const;

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

protected:
    std::string m_filename_graphBuilding = "";/**< File name/path where the gragh-building information will be stored (if empty-string, no data will be saved) */
};

}

#endif // AROLIBGRAPHPROCESSOR_H
