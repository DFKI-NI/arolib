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
 
#ifndef AROLIB_OLVPLAN_HPP
#define AROLIB_OLVPLAN_HPP

#include <ctime>

#include "arolib/planning/path_search/directedgraph.hpp"
#include "arolib/planning/path_search/astar.hpp"
#include "planningException.hpp"
#include "arolib/planning/path_search/graphhelper.hpp"
#include "arolib/planning/path_search/astar_successor_checkers.hpp"

#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/filesystem_helper.h"


namespace arolib{

/**
 * @brief Class to compute plans for the OLV (plan route segments for overload activities) and to manage the plans for subsequent overload planning
 */
class OLVPlan : public LoggingComponent{

public:

    /**
     * @brief Holds the relevant information about the overloading activities/windows
     */
    struct OverloadInfo {
        int start_index; /**< Index in the harvester route points corresponding to the start of overloading */
        int end_index; /**< Index in the harvester route points corresponding to the end of overloading */
        Machine machine; /**< Assigned overloading vehicle */
        bool toResourcePointFirst; /**< If true, a trip to a resource point is needed before the overload is started */
    };

    /**
     * @brief Planner parameters/settings
     *
     * Inherits from Astar::AStarSettings
     * @sa Astar::AStarSettings
     */
    struct PlannerSettings : public virtual Astar::AStarSettings{
        bool includeCostOfOverload = false;

        /**
         * @brief Parse the parameters from a string map, starting from a default PlannerSettings
         * @param [out] param Parameters
         * @param map String map containing the parameter values
         * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the default values will remain)
         * @return True on success
         */
        static bool parseFromStringMap( PlannerSettings& params, const std::map<std::string, std::string>& map, bool strict = false);

        /**
         * @brief Parse the parameters to a string map
         * @param param Parameters
         * @return String map containing the parameter values
         */
        static std::map<std::string, std::string> parseToStringMap( const PlannerSettings& params);
    };

    /**
     * @brief Constructor
     * @param olv OLV
     * @param resource_vertices Resource/Silo vertices
     * @param accessPoints_vertices Field access points' vertices
     * @param settings Planner parameters/settings
     * @param initialBunkerMass OLV's initial bunker mass
     * @param edgeCostCalculator Edge Cost Calculator. Temporary: if = nullptr, uses internal astar functions
     * @param outputFolder Folder where the planning (search) information will be stored (if empty-string, no data will be saved)
     * @param logLevel Log level
     */
    explicit OLVPlan(const Machine &olv,
                     const std::vector<DirectedGraph::vertex_t> &resource_vertices,
                     const std::vector<DirectedGraph::vertex_t> &accessPoints_vertices,
                     const PlannerSettings &settings,
                     const MachineDynamicInfo& initialState,
                     std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                     const std::string& outputFolder = "",
                     LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Plan next overload activity (from current machine state to overload end)
     *
     * For the first overload plan: -> resource point (if necesary) -> overload start -> overload (follow harvester) -> overload end
     * For subsequent overload plans: -> resource point -> overload start -> overload (follow harvester) -> overload end
     * @param [in/out] graph Current graph, updated after planning
     * @param [in/out] harvester_route Current harvester route, updated after planning
     * @param harv Harvester (if = null, not used)
     * @param overload_info Info about the overloading activity to be planned
     * @param maxWaitingTime The maximum amount of time [s] that the harvester may wait for the olv to arrive to the overload-start. If < 0, there is no limit
     * @param is_overloading (deprecated) Is the OLV overloading?
     * @return True on success
     */
   bool planOverload(DirectedGraph::Graph &graph,
                     Route &harvester_route,
                     const std::shared_ptr<Machine> harv,
                     const OverloadInfo &overload_info,
                     const std::vector< std::pair<MachineId_t, double> >& overload_info_prev = {},
                     double maxWaitingTime = 0,
                     bool is_overloading = false);//is_overloading is useless in actual state

    /**
     * @brief Finishes the plan for the OLV (i.e. computes the last route segments to, for intance, send the olv to a resource point)
     * @param [in/out] graph Current graph, updated after planning
     * @param sendToResourcePointIfNotFull If true, the OLV will be sent to a resource point even if it is not full
     * @param Route Current harvester route
     * @return True on success
     */
    bool finishPlan(DirectedGraph::Graph &graph,
                    bool sendToResourcePointIfNotFull,
                    const Route &harvester_route,
                    const std::vector< std::pair<MachineId_t, double> >& overload_info_prev);

    /**
     * @brief Adds a delay to a harvester route starting from the given route-point index
     * @param [in/out] graph Current graph, updated after applying delay
     * @param [in/out] route Current harvester route, updated after applying delay
     * @param indStart Route-point index from which to start to apply the delay
     * @param delay Delay [s]
     */
    static void addDelayToHarvRoute(DirectedGraph::Graph &graph,
                                    Route &harvester_route,
                                    size_t indStart,
                                    double delay);

    /**
     * @brief Get a copy of the planned route at a given planning state (can retrieve previous routes, i.e. corresponding to previous overload plans)
     * @param routeId Id to be set to the route
     * @param stateIndex Index of the state to be retrieved (if < 0 or >= numStates, it uses the last (current) state, i.e. the current planned route)
     * @sa getStatesMemorySize
     * @return Planned route at the given planning state
     */
    Route getRoute(int routeId, int stateIndex = -1) const;

    /**
     * @brief Get the timestamp of the last route-point of the planned route at a given planning state (can retrieve previous routes, i.e. corresponding to previous overload plans)
     * @param stateIndex Index of the state to be retrieved (if < 0 or >= numStates, it uses the last (current) state, i.e. the current planned route)
     * @sa getStatesMemorySize
     * @return Timestamp of the last route-point of the planned route at the given planning state
     */
    double getRouteLastTimestamp(int stateIndex = -1) const;

    /**
     * @brief Get the planned route segments of the current plan (i.e. the results of each search (subplan))
     * @return Planned route segments
     */
    const std::vector<Route> &getRouteSegments() const {return m_planned_routes;}

    /**
     * @brief Get the total cost at a given planning state (can retrieve previous costs, i.e. corresponding to previous overload plans)
     * @param stateIndex Index of the state to be retrieved (if < 0 or >= numStates, it uses the last (current) state, i.e. the current plan)
     * @sa getStatesMemorySize
     * @return Total cost at the given planning state
     */
    double getCost(int stateIndex = -1) const;

    /**
     * @brief Get the total (harvester) delay at a given planning state (can retrieve previous costs, i.e. corresponding to previous overload plans)
     * @param stateIndex Index of the state to be retrieved (if < 0 or >= numStates, it uses the last (current) state, i.e. the current plan)
     * @sa getStatesMemorySize
     * @return Total (harvester) delay [s] at the given planning state
     */
    double getDelay(int stateIndex = -1) const;

    /**
     * @brief Get the total amount of CROSS edges used at a given planning state (can retrieve previous costs, i.e. corresponding to previous overload plans)
     * @param [out] numCrossings Total amount of inner-field-harvesting CROSS edges at the given planning state
     * @param [out] numCrossings_HL Total amount of headland-harvesting CROSS edges at the given planning state
     * @param stateIndex Index of the state to be retrieved (if < 0 or >= numStates, it uses the last (current) state, i.e. the current plan)
     * @sa getStatesMemorySize
     * @return Total amount of CROSS edges at the given planning state (numCrossings + numCrossings_HL)
     */
    size_t getNumCrossings(size_t& numCrossings, size_t& numCrossings_HL ,int stateIndex = -1) const;

    /**
     * @brief Get the current amount of planning states
     * @return Current amount of planning states
     */
    size_t getStatesMemorySize() const {return m_statesMemory.size();}

    /**
     * @brief Check if a plan was found for the last overload activity
     * @return True if a plan was found
     */
    bool foundPlan() const {return m_found_plan;}

    static const double OlvMaxCapacityMultiplier; /**< Multiplier used to decide whether an OLV is 'full' or not (used max bunker capacity = machine max capacity * OlvMaxCapacityMultiplier) */

private:

    /**
     * @brief Location of the OLV
     */
    enum LocationType{
        LOC_UNKNOWN, /**< Unknown location (normally before planning any overload) */
        LOC_INIT, /**< Initial (known) location of the OLV (i.e. corresponding to the position of th OLV) */
        LOC_RESOURCE_IN, /**< At a resource point, before downloading */
        LOC_RESOURCE_OUT, /**< At a resource point, after downloading */
        LOC_HARVESTER_IN, /**< At a overload-start point */
        LOC_HARVESTER_OUT, /**< At a overload-end point */
        LOC_EXIT /**< At a field exit point */
    };

    /**
     * @brief Download side
     */
    enum DownloadSide{
        DS_RIGHT, /**< Right */
        DS_LEFT, /**< Left */
        DS_BEHIND, /**< Previous harvester route point to the switching point */
        DS_SWITCHING_POINT_BEHIND, /**< At the switching point, but following in the back */
        DS_SWITCHING_POINT, /**< At the switching point */
    };

    /**
     * @brief Class holding the relevant information of a planning state
     */
    struct PlanState{
        LocationType lastLocation = LOC_UNKNOWN; /**< Last location of the OLV */
        DirectedGraph::vertex_t olv_last_vt = -1; /**< Last vertex visited by the OLV */
        double plan_cost = 0.0; /**< Total plan cost (including the costs of previous overload plans) */
        double olv_time = 0.0; /**< Total duration [s] of the OLV planned routes (including the routes of previous overload plans) */
        double olv_bunker_mass = 0.0; /**< Last OLV bunker mass [Kg] */
        int route_id = 0; /**< Route Id */
        std::set<DirectedGraph::vertex_t> excludeVts; /**< Set containing the vertices to be excluded in the search  */
        Route route; /**< (complete) planned route (including the costs of previous overload plans) */
        std::vector<RoutePoint> route_points_prev; /**< Route points (previous state) */
        double planningDuration_toHarv = 0; /**< Time [s] needed to plan the route segment to arrive to the overload-start of the corresponding overload activity */
        double planningDuration_toRP = 0; /**< Time [s] needed to plan the route segment to arrive to the resource point (from an overloading-end) of the corresponding overload activity */
        double planningDuration_toRP_init = 0; /**< Time [s] needed to plan the route segment to arrive to the resource point (from the initial location) of the corresponding overload activity */
        double planningDuration_overloading = 0; /**< Time [s] needed to plan the overloading route segment */
        double planCost_toHarv = 0; /**< Cost of the planned route segment to arrive to the overload-start of the corresponding overload activity */
        double planCost_toRP = 0; /**< Cost of the planned route segment to arrive to the resource point (from an overloading-end) of the corresponding overload activity */
        double planCost_toRP_init = 0; /**< Cost of the planned route segment to arrive to the resource point (from the initial location) of the corresponding overload activity */
        double planCost_overloading = 0; /**< Cost of the overloading route segment (follow harvester) of the corresponding overload activity */
        double planCost_unloading = 0; /**< Cost of the unloading route segment (at the resource point) of the corresponding overload activity */
        double delay = 0; /**< (Harvester) delay for the corresponding overload activity (without including the ones of previous overload plans) */
        size_t prev_overloading_end_index = -1; /**< Route-point index of the harvester route corresponding to the last/previous overload-end */
        size_t numCrossings = 0; /**< Total amount of inner-field-harvesting CROSS edges used so far (including the ones of previous overload plans) */
        size_t numCrossings_HL = 0; /**< Total amount of headland-harvesting CROSS edges used so far (including the ones of previous overload plans) */
        std::pair<DirectedGraph::vertex_t, DownloadSide> adjVertexInfo = std::make_pair(-1, DownloadSide::DS_SWITCHING_POINT); /**< Holds the information about the goal vertex used to go to the (OL start) switching point */
        bool leaveOverloadingFromClosestVt = false;//for cases where the olv finishes the overloading in an unknown vertex
        RoutePoint lastRoutePoint;//Last route point of the previous overload
    };

    PlanState m_state; /**< Current working planning state */
    std::map<size_t, PlanState> m_states; /**< Current working planning states (one per resource point) */
    std::vector<PlanState> m_statesMemory; /**< Previous planning states */
    PlannerSettings m_settings; /**< Planner parameters/settings */
    bool m_start_at_resource = true; /**< Flag to know if the OLV has to start at a resource point */
    DirectedGraph::vertex_t m_olv_initial_vt = -1; /**< Initial vertex of the OLV (i.e. the one corresponding to the initial position of the OLV before planning) */
    Machine m_olv; /**< OLV machine */
    std::vector<DirectedGraph::vertex_t> m_resource_vertices; /**< Resource/silo vertices */
    std::vector<DirectedGraph::vertex_t> m_accessPoints_vertices; /**< Field access points' vertices */
    MachineDynamicInfo m_olv_initial_state; /**< Initial OLV state */
    bool m_found_plan = false; /**< Flag indicating if a plan was found for the last overload activity */
    std::shared_ptr<IEdgeCostCalculator> m_edgeCostCalculator = nullptr; /**< Edge cost calculator */

    std::vector<Route> m_planned_routes;  /**< Planned route segments so far */

    int m_prev_overloading_end_index = -1;  /**< (deprecated) Route-point index of the harvester route corresponding to the last/previous overload-end */

    std::string m_outputFolder = ""; /**< Folder where the planning (search) information will be stored (if empty-string, no data will be saved) */
    size_t m_countPlans = 0; /**< Counter of route-segment plans */

    /**
     * @brief Restore the necessary data if an error happened
     * @return (not implemented yet)
     */
    bool restoreInError();

    /**
     * @brief Initializes the current planning state (m_state), copying the necessary dat from the previous/last state
     */
    void initState();

    /**
     * @brief Updates the current planning state (m_state) and the graph with the given plan results
     * @param plan Plan results
     * @param [out] graph Graph to be updated with the plan results
     * @param newLocation New OLV location
     */
    void updateState(const AstarPlan& plan, DirectedGraph::Graph &graph, LocationType newLocation);

    /**
     * @brief Find the best resource-point/access-point vertex to be used as the initial OLV vertex (in cases when the real initial position of the OLV is unknown)
     * @param graph Graph
     * @param harvester_route Harvester route
     * @param [out] vt Best resource-point/access-point vertex to be used as the initial OLV vertex
     * @param [out] isEntryPoint Set to true if the output vertex 'vt' is a field entry point vertex
     * @param includeEntryPoints If true, resource and field-entry vertices will be checked; otherwise, only resource vertices are checked
     * @return True on success
     */
    bool getBestInitialVertex(const DirectedGraph::Graph &graph, const Route &harvester_route, DirectedGraph::vertex_t &vt, bool &isEntryPoint, bool includeEntryPoints);


    /**
     * @brief Plan the route-segment from the current vertex to a valid point adjacent to the switching (overload-start) vertex
     * @param [in/out] graph Current graph, updated after planning
     * @param [out] plan Plan results
     * @param switching_vt Switching (overload-start) vertex
     * @param switching_time (original) Switching timestamp (without delays). i.e. timestamp when the harvester is at the switching point
     * @param machine_speed OLV speed [m/s]
     * @param harvester_route Harvester route
     * @param harvester_rp_index Index of the route point (of the harvester route) corresponding to the switching (overload-start) point
     * @param harv Harverster
     * @param maxDelay Maximum (harvester) delay (waiting time) [s] allowed so that the OLV reaches the switching (overload-start) point
     * @param [out] adjVertexInfo info about the selected adjacent vertex
     * @param [out] olvWaitingTime Time the olv has to wait for the harvester to arrive to the switching point
     * @return True on success
     */
    bool planPathToAdjacentPoint(DirectedGraph::Graph &graph,
                                 AstarPlan &plan,
                                 const DirectedGraph::vertex_t &switching_vt,
                                 double switching_time,
                                 double machine_speed,
                                 const Route &harvester_route,
                                 int harvester_rp_index,
                                 const Machine& harv,
                                 double maxDelay,
                                 std::pair<DirectedGraph::vertex_t, DownloadSide> &adjVertexInfo,
                                 double &olvWaitingTime,
                                 const OverloadInfo &overload_info,
                                 const std::vector< std::pair<MachineId_t, double> >& overload_info_prev = {});

    /**
     * @brief Plan the route-segment from the current vertex to a valid point adjacent to the switching (overload-start) vertex
     * @param [in/out] graph Current graph, updated after planning
     * @param [out] plan Plan results
     * @param switching_vt Switching (overload-start) vertex
     * @param switching_time (original) Switching timestamp (without delays). i.e. timestamp when the harvester is at the switching point
     * @param machine_speed OLV speed [m/s]
     * @param harvester_route Harvester route
     * @param harvester_rp_index Index of the route point (of the harvester route) corresponding to the switching (overload-start) point
     * @param maxDelay Maximum (harvester) delay (waiting time) [s] allowed so that the OLV reaches the switching (overload-start) point
     * @param [out] olvWaitingTime Time the olv has to wait for the harvester to arrive to the switching point
     * @return True on success
     */
    bool planPathToAdjacentPoint_2(DirectedGraph::Graph &graph,
                                   AstarPlan &plan,
                                   const DirectedGraph::vertex_t &switching_vt,
                                   double switching_time,
                                   double machine_speed,
                                   const Route &harvester_route,
                                   int harvester_rp_index,
                                   double clearanceDist,
                                   double maxDelay,
                                   double &olvWaitingTime,
                                   const OverloadInfo &overload_info,
                                   const std::vector< std::pair<MachineId_t, double> >& overload_info_prev = {});

    /**
     * @brief Plan the route-segment from the current vertex to the switching (overload-start) vertex
     * @param [in/out] graph Current graph, updated after planning
     * @param [out] plan Plan results
     * @param switching_vt Switching (overload-start) vertex
     * @param switching_time (original) Switching timestamp (without delays). i.e. timestamp when the harvester is at the switching point
     * @param machine_speed OLV speed [m/s]
     * @param maxDelay Maximum (harvester) delay (waiting time) [s] allowed so that the OLV reaches the switching (overload-start) point
     * @param [out] olvWaitingTime Time the olv has to wait for the harvester to arrive to the switching point
     * @return True on success
     */
    bool planPathToSwitchingPoint(DirectedGraph::Graph &graph,
                                  AstarPlan &plan,
                                  const DirectedGraph::vertex_t &switching_vt,
                                  double switching_time,
                                  double machine_speed,
                                  double maxDelay,
                                  const Route &harvester_route,
                                  int harvester_rp_index,
                                  double clearanceDist,
                                  double &olvWaitingTime,
                                  const OverloadInfo &overload_info,
                                  const std::vector< std::pair<MachineId_t, double> >& overload_info_prev = {});

    /**
     * @brief Plan the route-segment from the current vertex to a valid vertex to (re)start overloading
     * @param [in/out] graph Current graph, updated after planning
     * @param [out] plan Plan results
     * @param [in/out] harvester_route Current harvester route, updated after planning
     * @param harvester_rp_index Index of the route point (of the harvester route) corresponding to the switching (overload-start) point
     * @param harv Harverster (if = null, not used)
     * @param maxDelay Maximum (harvester) delay (waiting time) [s] allowed so that the OLV reaches the switching (overload-start) point
     * @return True on success
     */
    bool planPathToHarvester(DirectedGraph::Graph &graph,
                             AstarPlan &plan,
                             Route &harvester_route,
                             int harvester_rp_index,
                             const std::shared_ptr<Machine> harv,
                             double maxDelay,
                             const OverloadInfo &overload_info,
                             const std::vector< std::pair<MachineId_t, double> >& overload_info_prev = {});

    /**
     * @brief Plan the route-segment from the current vertex to the best resource/silo vertex
     *
     * About nextSwitchingPointTimestamp: (if >= 0) this timestamp will be used as the A* max_visit_time. If the machine will be sent to an overloading-start after visiting the resource/silo vertex, this value should correspond to the timestamp of that overloading-start.
     * @sa: Astar
     * @param [in/out] graph Current graph, updated after planning
     * @param [out] plan Plan results
     * @param nextSwitchingPointTimestamp Timestamp of the (harvesting) route point corresponding to the next switching (OL-start) point. Disregarded if < 0;
     * @param maxTimeGoal Maximum time(-stamp) in which the goal (resource/silo vertex) must be reached (A* max_time_goal). If < 0, no limit is applied
     * @return True on success
     */

    bool planPathToResource(DirectedGraph::Graph &graph,
                            DirectedGraph::vertex_t *pResVt,
                            AstarPlan &plan,
                            double nextSwitchingPointTimestamp,
                            const Route &harvester_route,
                            double maxTimeGoal = std::numeric_limits<double>::max());

    /**
     * @brief Plan the route-segment from the current vertex to the best field-exit vertex
     * @param [in/out] graph Current graph, updated after planning
     * @param [out] plan Plan results
     * @param harvester_route (optional) Current harvester route
     * @param harvester_rp_index Index of the route point (of the harvester route) corresponding to the NEXT switching (overload-start) point
     * @param maxDelay Maximum (harvester) delay (waiting time) [s] allowed so that the OLV reaches the switching (overload-start) point. If the are no more overloading activities, set harvester_route = {}
     * @return True on success
     */
    bool planPathToExitPoint(DirectedGraph::Graph &graph,
                             AstarPlan &plan,
                             double lastOverloadTimestamp,
                             const Route &harvester_route);

    /**
     * @brief Plan the route-segment related to overloading following the harvester behind
     * @param [in/out] graph Current graph, updated after planning
     * @param [in/out] harvester_route Current harvester route, updated after planning
     * @param overload_info Overload information
     * @param harv Harverster (if = null, not used)
     * @return True on success
     */
    bool planOverloadingPath_behind(DirectedGraph::Graph &graph,
                                    Route &harvester_route,
                                    const OverloadInfo &overload_info,
                                    const std::shared_ptr<Machine> harv);

    /**
     * @brief Plan the route-segment related to overloading following the harvester in the side(s)
     * @param [in/out] graph Current graph, updated after planning
     * @param [in/out] harvester_route Current harvester route, updated after planning
     * @param overload_info Overload information
     * @param harv Harverster (if = null, not used)
     * @return True on success
     */
    bool planOverloadingPath_alongside(DirectedGraph::Graph &graph,
                                       Route &harvester_route,
                                       const OverloadInfo &overload_info,
                                       const std::shared_ptr<Machine> harv);

    /**
     * @brief Check if overloading a the given side is doable
     * @param graph Current graph, updated after planning
     * @param harvester_route Current harvester route, updated after planning
     * @param startInd Start route-point index index (harvester_route)
     * @param endInd End route-point index index (harvester_route)
     * @param downloadSide Side
     * @param harv Harverster (if = null, not used)
     * @return True on success
     */
    bool checkSideOverloading(const DirectedGraph::Graph &graph,
                              const Route &harvester_route,
                              size_t startInd, size_t endInd,
                              DownloadSide &downloadSide,
                              const std::shared_ptr<Machine> harv);
    /**
     * @brief Plan inter-track connection during overloading
     * @param [in/out] graph Current graph, updated after planning
     * @param [in/out] harvester_route Current harvester route, updated after planning
     * @param overload_info Overload information
     * @param startVt Start vertex
     * @param excludeVts Vertices to be excluded
     * @param bunker_mass Current OLV bunker mass
     * @param startTime Start timestamp
     * @param harv Harverster (if = null, not used)
     * @param plan Resulting plan
     * @return True on success
     */
    bool planConnectionDuringOverloading(DirectedGraph::Graph &graph,
                                         Route &harvester_route,
                                         const OverloadInfo &overload_info,
                                         DirectedGraph::vertex_t startVt,
                                         std::set<DirectedGraph::vertex_t> excludeVts,
                                         double bunker_mass,
                                         double startTime,
                                         const std::shared_ptr<Machine> harv,
                                         DirectedGraph::vertex_t &endVt,
                                         DownloadSide &downloadSide,
                                         AstarPlan &plan);

    /**
     * @brief Get the points of the route-segment corresponding to overloading (follow harvester from overload-start to overload-end)
     * @param [in/out] graph Current graph, updated after planning
     * @param info Overload activity data
     * @param harvester_route Current harvester route
     * @param initial_harv_mass Harvested mass at the moment of switching
     * @param initial_bunker_mass OLV bunker mass at the moment of starting the overloading
     * @param clearanceTime Time applied to the olv route not to overlap withn the harvester
     * @return Route points of the route-segment corresponding to overloading
     */
    std::vector<RoutePoint> followHarvester(DirectedGraph::Graph &graph,
                                            const OverloadInfo &info,
                                            const Route &harvester_route,
                                            const std::shared_ptr<Machine> harv,
                                            double initial_bunker_mass,
                                            double clearanceTime);
    std::vector<RoutePoint> followHarvesterNoLength(DirectedGraph::Graph &graph,
                                                    const OverloadInfo &info,
                                                    const Route &harvester_route,
                                                    const std::shared_ptr<Machine> harv,
                                                    double initial_bunker_mass,
                                                    double clearanceTime);

    /**
     * @brief Get the points of the route-segment corresponding to overloading (follow harvester alonside)
     *
     * It should be used to compute overloading segments separatelly, i.e. first the segment with no track changes, then the headland-connection segments, then the next track-segment
     * @param [in/out] graph Current graph, updated after planning
     * @param info Overload activity data
     * @param harvester_route Current harvester route
     * @param initial_bunker_mass OLV bunker mass at the moment of starting the overloading
     * @param offset If true, the route points will be obtained by offseting the harvester points; if false, a translation operation is done instead
     * @param harv Harverster (if = null, not used)
     * @return Route points of the route-segment corresponding to overloading
     */
    std::vector<RoutePoint> followHarvesterAlongside(DirectedGraph::Graph &graph,
                                                     const OverloadInfo &info,
                                                     const Route &harvester_route,
                                                     double initial_bunker_mass,
                                                     bool offset,
                                                     const std::shared_ptr<Machine> harv);

    /**
     * @brief Update costs, edge properties (incl. overruns) and visit periods from a route segment
     * @param [in/out] graph Current graph, updated after planning
     * @param route_points Route point
     * @param harv Harverster (if = null, not used)
     * @param ind_from Index of the first route point in the segment
     * @param ind_to Index of the last route point in the segment (if <0 -> last route point)
     */
    void updateFromRouteSegment( DirectedGraph::Graph &graph,
                                 const std::vector<RoutePoint> route_points,
                                 bool addCosts,
                                 const std::shared_ptr<Machine> harv,
                                 size_t ind_from = 0,
                                 int ind_to = -1 );

    /**
     * @brief Removed the points from the state route that cause spikes in the initial segments of the route after planning to a resource point or an exit
     */
    void removeSpikePointsFromTransitRoute();

    /**
     * @brief Get the set of vertices under a route segment
     * @param [in/out] graph Current graph, updated after planning
     * @param route_points Route point
     * @param width Machine width
     * @param ind_from Index of the first route point in the segment
     * @param ind_to Index of the last route point in the segment (if <0 -> last route point)
     */
    static std::set<DirectedGraph::vertex_t> getVerticesUnderRouteSegment(const DirectedGraph::Graph &graph,
                                                                          const std::vector<RoutePoint> route_points,
                                                                          double width,
                                                                          size_t ind_from = 0,
                                                                          int ind_to = -1 );

    /**
     * @brief Get the adjacent vertices to the switching point depending on the available download sides of the harvester
     * @param graph Current graph
     * @param harvester_route Current harvester route
     * @param harvester_rp_index Index of the route point
     * @param harv Harvester
     * @return Vector containing the adjacent vertices and the location (right, left, back, front) with respect to the harvester
     */
    std::vector<std::pair<DirectedGraph::vertex_t, DownloadSide> > getAdjacentVertices(DirectedGraph::Graph &graph,
                                                                                       const Route &harvester_route,
                                                                                       int harvester_rp_index,
                                                                                       const Machine &harv);

    /**
     * @brief Insert into the given list the vertices adjascent to a given route point (located behind it)
     * @param graph Current graph
     * @param harvester_route Current harvester route
     * @param harvester_rp_index Index of the route point
     * @param harv Harvester
     * @param [in/out] adjacent_vts Vector where the vertices will be inserted
     */
    void addAdjacentVertices_back(DirectedGraph::Graph &graph,
                                  const Route &harvester_route,
                                  int harvester_rp_index,
                                  const Machine &harv,
                                  std::vector<std::pair<DirectedGraph::vertex_t, DownloadSide> >& adjacent_vts);

    /**
     * @brief Insert into the given list the vertices adjascent to a given route point (located besides it)
     * @param graph Current graph
     * @param harvester_route Current harvester route
     * @param harvester_rp_index Index of the route point
     * @param harv Harvester
     * @param [in/out] adjacent_vts Vector where the vertices will be inserted
     */
    void addAdjacentVertices_sides(DirectedGraph::Graph &graph,
                                  const Route &harvester_route,
                                  int harvester_rp_index,
                                  const Machine &harv,
                                  std::vector<std::pair<DirectedGraph::vertex_t, DownloadSide> >& adjacent_vts);

    std::set<DirectedGraph::vertex_t> addSuccCheckerForReverseDriving_start(std::vector<std::shared_ptr<const Astar::ISuccesorChecker>>& succCheckers,
                                                                            const DirectedGraph::Graph &graph,
                                                                            double angTH = 45);
    std::set<DirectedGraph::edge_t> replaceSuccCheckerForReverseDriving_OLStart(std::vector<std::shared_ptr<const Astar::ISuccesorChecker>>& succCheckers,
                                                                                size_t indSC,
                                                                                const DirectedGraph::Graph &graph,
                                                                                const Route &harvester_route,
                                                                                const OverloadInfo &overload_info, const DirectedGraph::vertex_t &vt_goal, const DirectedGraph::vertex_property &vt_prop_goal,
                                                                                double angTH = 100);
    void addSuccCheckerForPrevOLEnd(std::vector<std::shared_ptr<const Astar::ISuccesorChecker>>& succCheckers,
                                    const DirectedGraph::Graph& graph,
                                    const Route &harvester_route);

    /**
     * @brief Update the set of vertices to be excluded during the search, adding more necessary vertices when necessary
     * @param graph Current graph
     * @param [in/out] exclude Set of exclude vertices to be updated
     */
    void updateExcludeSet(const DirectedGraph::Graph &graph,
                          std::set<DirectedGraph::vertex_t>& exclude) const;


    /**
     * @brief Compute the time an olv hast to wait to start following a harvester
     * @param harvester_route Current harvester route
     * @param harvester_rp_index Index of the route point (of the harvester route) corresponding to the NEXT switching (overload-start) point
     * @param clearanceDist Clearance distance between the harvester and the olv
     * @return Waiting time
     */
    double getOlvWaitingTimeToFollowHarv(const Route &harvester_route,
                                         int harvester_rp_index,
                                         double clearanceDist);

    /**
     * @brief Get the clearance distance when following the harvester behind
     * @param harv Harvester
     * @return Clearance distance
     */
    double getClearanceDistBehindHarv(const Machine &harv);


    /**
     * @brief Updates the current olv vertex with the one that is valid and closest to the last route point.
     */
    bool updateCurrentVertexWithClosest(DirectedGraph::Graph &graph, const Route &harvester_route);
    bool updateCurrentVertexWithClosest(size_t key, DirectedGraph::Graph &graph, const Route &harvester_route);

    /**
     * @brief Get the list of vertices adjascent (i.e. connected) closest to a given vertex, from the previous and next tracks, and (if applicable) the headland
     * @param graph Graph
     * @param allowBoundaryVtsIfNeeded If Not enough vertices are found and the vt connects to the boundary vts, take into account the boundary vts
     * @return List of resulting vertices
     */
    std::set<DirectedGraph::vertex_t> getAdjacentVerticesForSideOverloading(const DirectedGraph::Graph &graph,
                                                                               const Route &harvester_route,
                                                                               const DirectedGraph::vertex_t switching_vt,
                                                                               int harvester_rp_index,
                                                                               bool allowBoundaryVtsIfNeeded = false) const;


    /**
     * @brief Get the folder where the planning (search) information will be stored depending on the plan type
     * @param planType Plan type corresponding to the search within a overload plan (e.g. "to_adj", "to_harv", etc)
     */
    std::string getOutputFolder(const std::string& planType);


    class AstarSuccessorChecker_prevOLEnd: public Astar::ISuccesorChecker{
    public:
        /**
         * @brief Constructor.
         */
        AstarSuccessorChecker_prevOLEnd(double timestamp_nextOLStart, const DirectedGraph::vertex_t& vt_prevOLEnd);

        /**
         * @brief Check if a sucesor is valid during AStar search
         * @sa Astar::ISuccesorChecker::isSuccessorValid
        */
        virtual Astar::ISuccesorChecker::Validity isSuccessorValid(const Astar::ISuccesorChecker::IsSuccessorValidParams&) const override {return Astar::ISuccesorChecker::VALID;}

        /**
         * @brief Get the minimum duration [s] for a machine planning to go over the edge
         * @param params Input parameters
         * @return Minimum duration [s]. If < 0, no minimum duration applies (disregarded during search)
        */
        virtual double getMinDurationAtEdge(const GetMinDurationAtEdgeParams& params) const override;

    protected:
        double m_timestamp_nextOLStart;
        DirectedGraph::vertex_t m_vt_prevOLEnd;
    };
};

}

#endif // AROLIB_OLVPLAN_HPP
