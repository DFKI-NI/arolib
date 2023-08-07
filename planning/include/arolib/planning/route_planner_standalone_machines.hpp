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
 
#ifndef AROLIB_ROUTE_PLANNER_STANDALONE_MACHINES_HPP
#define AROLIB_ROUTE_PLANNER_STANDALONE_MACHINES_HPP

#include <ctime>
#include <sys/stat.h>

#include "arolib/planning/path_search/directedgraph.hpp"
#include "arolib/types/route.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/types/materialFlowType.hpp"
#include "arolib/planning/roundtripplanner.hpp"
#include "arolib/planning/activitiesswitchingplanner.hpp"
#include "arolib/planning/transit_restrictions.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/logger.h"
#include "arolib/misc/filesystem_helper.h"

namespace arolib{

/**
 * @brief Planner to compute routes for processes where only standalone capacitated machines perform the work in the field, i.e. the machines do not need a service unit (e.g. a transport vehicle) to do the work, they do both the working in the field and the transportation of material.
 */
class RoutePlannerStandaloneMachines : public LoggingComponent
{
public:

    /**
     * @brief Holds the relevant data of each plan
     */
    struct PlanData{
        friend class RoutePlannerStandaloneMachines;

        /**
         * @brief Holds the information of a working window
         */
        struct WorkingWindowInfo{
            size_t indStart; /**< Index of the route-point where the windows starts */
            size_t indFinish; /**< Index of the route-point where the windows finishes */
        };

        DirectedGraph::Graph graph; /**< Updated graph */
        std::vector<Route> routes; /**< Map holding all planned routes (including transportation to resource points) */
        std::vector<double> planCosts; /**< Plan costs for each route */
        std::map<size_t, std::vector<WorkingWindowInfo>> workingWindows;  /**< Map containing the information of the working windows for each route (key := route index) */
        double planOverallCost = 0; /**< Overall plan cost (for all harvester routes) */
        bool planOK = false; /**< Flag to know if the plan is OK */
    protected:

        /**
         * @brief Initialized the data
         * @param _graph Current graph
         * @param _routes Current routes
         */
        void init(const DirectedGraph::Graph &_graph,
                  const std::vector<Route> &_routes);

        /**
         * @brief Cumpute and update the overall plan cost (for all routes) using the plan costs for each one of the routes
         */
        void updateOverallCost();

        /**
         * @brief Updates the working windows information by adding a delta value to their indexes
         * @param indRoute Index of the routes whose working windows will be updated
         * @param indRef only the windows with indStart >= indRef are updated
         * @param deltaInd Delta value
         */
        void updateWorkingWindows(size_t indRoute, size_t indRef, int deltaInd);

        /**
         * @brief Updates the working windows information by adding a delta value to their indexes
         * @param workingWindows Working windows to be updated
         * @param indRoute Index of the routes whose harvesting windows will be updated
         * @param indRef only the windows with indStart >= indRef are updated
         * @param deltaInd Delta value
         */
        static void updateWorkingWindows(std::map<size_t, std::vector<WorkingWindowInfo>> &workingWindows, size_t indRoute, size_t indRef, int deltaInd);
    };

    /**
     * @brief Planner settings
     *
     * Inherits from RoundtripPlanner::PlannerSetting
     * @sa RoundtripPlanner::PlannerSetting
     */
    struct PlannerSettings : public virtual RoundtripPlanner::PlannerSettings, public virtual ASP_GeneralSettings{
        double maxPlanningTime = 60;/**< planning timeout [s] */
        bool finishAtResourcePoint = true;  /**< Should the last machine working the field be sent to the rosource point even if the bunker is not at the limit (full/empty)? */

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
     * @brief Constructor.
     *
     * @param graph (Initial) graph
     * @param baseRoutes (processed) base routes without transportation (must have increasingly monotonic timestamps)
     * @param machines Machines
     * @param machineCurrentStates Map containing the current states of the machines (inc. current location, bunker mass, etc.)
     * @param settings Planner parameters/settings
     * @param edgeCostCalculator Edge Cost Calculator. Temporary: if = nullptr, uses internal astar functions
     * @param outputFolder Folder where the planning (search) information will be stored (if empty-string, no data will be saved)
     * @param logLevel Log level
     */
    explicit RoutePlannerStandaloneMachines(const DirectedGraph::Graph &graph,
                                            const std::vector<Route> &baseRoutes,
                                            const std::vector<Machine> &machines,
                                            const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                            const Polygon& boundary,
                                            const PlannerSettings& settings,
                                            std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                            const std::string& outputFolder = "",
                                            LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Reset/reinitialize
     */
    void reset();

    /**
     * @brief Compute routes for all machines
     * @param materialFlowType Material flow type
     * @param transitRestriction Type of restriction for transit to- and from- the field
     * @return Error message (ok := empty string)
     */
    std::string planAll(MaterialFlowType materialFlowType,
                        TransitRestriction transitRestriction);

    /**
     * @brief Retrieve the data from the (best) computed plan
     * @return Data from the (best) computed plan
     */
    const PlanData& getPlanData() const {return m_bestPlan;}

    /**
     * @brief Get a copy of all planned routes
     * @return Copy of all planned routes
     */
    std::vector<Route> getPlannedRoutes();

protected:

    /**
     * @brief Add edge overruns for the routes
     * @return Map of the machines (working group)
     */
    void addOverruns( size_t routeIndex, size_t ind0, size_t ind1);

    /**
     * @brief Compute the information of the working windows for all routes
     * @param materialFlowType Material-flow type
     * @param plan Plan holding the routes and working windows
     * @return Error message (ok := empty string)
     */
    std::string calcWorkingWindows(MaterialFlowType materialFlowType, PlanData& plan);

    /**
     * @brief Initializes some the route points bunker masses
     *
     * All route points before and including the first working window indStart will have the current machine's bunker mass. If the routes have no working windows, this will apply to all route points.
     * All other route points' bunker masses will be set based on the working windows and worked masses (assumning that after unload the bunker is empty, and after load the bunker mass is full)
     * It is assumed that the windows are ordered in the vectors.
     * @param plan Plan holding the routes and working windows
     * @return Error message (ok := empty string)
     */
    std::string initRoutesBunkerMasses(PlanData& plan, MaterialFlowType materialFlowType);

    /**
     * @brief Add edge overruns for the routes
     * @param plan Plan holding the routes and working windows
     */
    void addOverruns(PlanData& plan);

    /**
     * @brief Add edge overruns for the routes (segments)
     * @param graph Graph to be updated
     * @param machine Machine
     * @param route Route
     * @param ind0 Start route-point index
     * @param ind0 End route-point index
     */
    void addOverruns(DirectedGraph::Graph& graph, const Machine &machine, const Route& route, size_t ind0, size_t ind1);

    /**
     * @brief Add the initial visit periods of the routes
     *
     * Adds the visit periods from rotues before working the field. If there are no working windows for the route, it adds the periods for all route points
     * @param plan Plan holding the routes and working windows
     */
    void addInitialVisitPeriods(PlanData& plan);

    /**
     * @brief Add the final visit periods of the routes
     *
     * Adds the visit periods from routes after working the field.
     * @param plan Plan holding the routes and working windows
     */
    void addFinalVisitPeriods(PlanData& plan);

    /**
     * @brief Add the visit periods of the route segment
     * @param graph Graph to be updated
     * @param machine Machine
     * @param route Route
     * @param ind0 Start route-point index
     * @param ind0 End route-point index
     */
    void addVisitPeriods(DirectedGraph::Graph& graph, const Machine &machine, const Route& route, size_t ind0, size_t ind1);

    /**
     * @brief Compute the updated routes (with unload trips) for all machines
     * @param plan Plan holding the routes and working windows
     * @param materialFlowType Material flow type
     * @param transitRestriction Type of restriction for transit to- and from- the field
     * @return Error message (ok := empty string)
     */
    std::string planTrips(PlanData& plan,
                          MaterialFlowType materialFlowType,
                          TransitRestriction transitRestriction);


    /**
     * @brief Make initial adjustments to base routes and graph based on the machines initial timestamp
     *
     * Removes initial segments
     * Adjust timestamps of the base route points and corresponding vertices
     * @param [in/out] plan containing graph and routes (to be updated)
     */
    void adjustBaseRoutesTimestamps(PlanData &plan);

    /**
     * @brief Plan initial trips from the initial location to the first route point (via resource point if necessary)
     * @param workingWindows[in/out] Working windows
     * @param resource_vts Resource point vertices
     * @return Error message (ok := empty string)
     */
    std::string planInitialTrips(PlanData& plan,
                                  std::map<size_t, std::vector<PlanData::WorkingWindowInfo>> &workingWindows,
                                  const std::vector<DirectedGraph::vertex_t>& resource_vts,
                                  MaterialFlowType materialFlowType,
                                  TransitRestriction transitRestriction);

    /**
     * @brief Plan initial trips from the initial location to the first route point
     * @param indRoutes Index of the routes to be connected
     * @param initRoutes [out] Planned route segments
     * @return Error message (ok := empty string)
     */
    std::string planInitialSegmentDirectly(PlanData& plan,
                                           const std::multimap<double, size_t>& indRoutes,
                                           TransitRestriction transitRestriction,
                                           std::map<size_t, AstarPlan>& initPlans);

    /**
     * @brief Plan initial trips from the initial location to the first route point via resource point
     * @param indRoutes Index of the routes to be connected
     * @param resource_vts Resource point vertices
     * @param initRoutes [out] Planned route segments
     * @return Error message (ok := empty string)
     */
    std::string planInitialSegmentViaResource(PlanData& plan,
                                              const std::multimap<double, size_t>& indRoutes,
                                              const std::vector<DirectedGraph::vertex_t>& resource_vts,
                                              MaterialFlowType materialFlowType,
                                              TransitRestriction transitRestriction,
                                              std::map<size_t, AstarPlan>& initPlans);

    /**
     * @brief Get the set of exclude vertices to be used during the search of best route-segments connecting the working-machines' initial points/locations with the first (valid) working point of their routes
     * @param graph Updated graph
     * @param goal_vt Goal vertex
     * @return Resulting set of exclude vertices
     */
    std::set<DirectedGraph::vertex_t> getExcludeVertices_initialRoutes(DirectedGraph::Graph &graph, DirectedGraph::vertex_t goal_vt);

    /**
     * @brief Gets the information about the next transportation planning to be done
     * @param plan Plan holding the routes
     * @param workingWindows Map of working windows to be used in the analysis
     * @param indRoute[out] Index of the route for which we have to plan.
     * @param indRP[out] Index of the route point corresponding to the moment of transportation.
     * @param machine[out] Machine of the corresponding route.
     * @return True if there is something to plan
     */
    bool getNextTransportationInfo(const PlanData& plan,
                                   const std::map<size_t, std::vector<PlanData::WorkingWindowInfo>> &workingWindows,
                                   size_t& indRoute, size_t& indRP, size_t& indRP_ret, Machine& machine);

    /**
     * @brief Gets the bunker state for a machine after visiting the resource point
     * @param machine Machine
     * @param materialFlowType Material flow type
     * @param remainingMass Mass remaining to be worked in the field
     * @param remainingVol Volume remaining to be worked in the field
     * @param [out] bunker_mass Mass remaining to be worked in the field
     * @param [out] bunker_volume Volume remaining to be worked in the field
     */
    static void getMachineBunkerStateAfterResourcePoint(const Machine& machine,
                                                        MaterialFlowType materialFlowType,
                                                        double remainingMass, double remainingVol,
                                                        double &bunker_mass, double &bunker_volume);

protected:

    DirectedGraph::Graph m_graph; /**< Initial graph */
    std::vector<Route> m_baseRoutes; /**< Base (initial) routes (with no transportation) */
    std::map<MachineId_t, Machine> m_machines; /**< Machines */
    std::map<MachineId_t, MachineDynamicInfo> m_machineInitialStates; /**< Map containing the current states of the machines (inc. current location, bunker mass, etc.) */
    Polygon m_boundary; /**< Field boundary */
    PlannerSettings m_settings; /**< Planner parameters/settings */
    std::shared_ptr<IEdgeCostCalculator> m_edgeCostCalculator = nullptr; /**< Edge cost calculator */

    PlanData m_bestPlan; /**< Data of the current BEST plan */
    PlanData m_currentPlan; /**< Data of the current plan */
    std::string m_outputFolder = ""; /**< Folder where the planning (search) information of all permutations will be stored (if empty-string, no data will be saved) */

    static const double MachineMaxCapacityMultiplier_OutputFlow; /**< Multiplier used to decide whether a machine is 'full' or not in output material flow operations (used max bunker capacity = machine max capacity * MachineMaxCapacityMultiplier) */
    static const double MachineMaxCapacityMultiplier_InputFlow; /**< Multiplier used to decide whether a machine is 'empty' or not in input material flow operations (used max bunker capacity = machine max capacity * MachineMaxCapacityMultiplier) */
};

}

#endif // AROLIB_ROUTE_PLANNER_STANDALONE_MACHINES_HPP
