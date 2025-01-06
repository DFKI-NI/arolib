/*
 * Copyright 2021-2025 DFKI GmbH
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

#include "arolib/types/machinedynamicinfo.hpp"
#include "arolib/types/resourcepointstate.hpp"
#include "arolib/types/materialFlowType.hpp"
#include "arolib/planning/roundtripplanner.hpp"
#include "arolib/planning/activitiesswitchingplanner.hpp"
#include "arolib/planning/transit_restrictions.hpp"

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
            float nextMinRequiredCapacityMass = 0; /**< Required mass capacity for the next working window */
            float nextMinRequiredCapacityVol = 0; /**< Required volume capacity for the next working window  */
        };

        DirectedGraph::Graph graph; /**< Updated graph */
        std::vector<Route> routes; /**< Map holding all planned routes (including transportation to resource points) */
        std::vector<double> planCosts; /**< Plan costs for each route */
        std::map<size_t, std::vector<WorkingWindowInfo>> workingWindows;  /**< Map containing the information of the working windows for each route (key := route index) */
        std::map<size_t, WorkingWindowInfo> nextWorkingWindows;  /**< Map containing the information of the next working windows for each route (key := route index) */
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
    };

    /**
     * @brief Enum holding the options for the last location of the machies after finishing working
     */
    enum FinishPointOption{
        FINISH_AT_FIELD_EXIT = 0, /**< Send the machine to a field exit when it finished working */
        FINISH_AT_RESOURCE_POINT, /**< Send the machine to a resource point when it finished working */
        FINISH_IN_FIELD /**< Leave the machine in the field at the last working point */
    };

    /**
      * @brief Get the FinishPoint (enum) from its int value
      * @brief value Int value
      * @return FinishPoint
      */
    static FinishPointOption intToFinishPointOption(int value);

    /**
     * @brief Planner settings
     *
     * Inherits from RoundtripPlanner::PlannerSetting
     * @sa RoundtripPlanner::PlannerSetting
     */
    struct PlannerSettings : public virtual RoundtripPlanner::PlannerSettings, public virtual ASP_GeneralSettings{
        double maxPlanningTime = 60;/**< planning timeout [s] */
        FinishPointOption finishPointOption = FinishPointOption::FINISH_AT_RESOURCE_POINT;  /**< Should the last machine working the field be sent an exit point, a rosource point, or stay at the location of the last working point? */

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
     * @param resourcePointCurrentStates Current states of the resource points (inc. current capacities)
     * @param settings Planner parameters/settings
     * @param edgeCostCalculator Edge Cost Calculator. Temporary: if = nullptr, uses internal astar functions
     * @param outputFolder Folder where the planning (search) information will be stored (if empty-string, no data will be saved)
     * @param logLevel Log level
     */
    explicit RoutePlannerStandaloneMachines(const DirectedGraph::Graph &graph,
                                            const std::vector<Route> &baseRoutes,
                                            const std::vector<Machine> &machines,
                                            const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                            const std::map<ResourcePointId_t, ResourcePointState> &resourcePointCurrentStates,
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
     * @brief Compute the next working window
     * @param materialFlowType Material-flow type
     * @param route Route
     * @param indPtFrom Index of the route point from where to start checking
     * @param bunker_mass Current bunker mass
     * @param bunker_vol Current bunker volume
     * @param [out] workingWindow Resulting working window (iff finished == false)
     * @param [out] finished True if no working window was computed
     * @return Error message (ok := empty string)
     */
    std::string getNextWorkingWindow(MaterialFlowType materialFlowType, const Route &route, size_t indPtFrom, double bunker_mass, double bunker_vol, PlanData::WorkingWindowInfo& workingWindow, bool &finished);

    /**
     * @brief Compute the information of the initial working windows for all routes
     * @param materialFlowType Material-flow type
     * @param plan Plan holding the routes and working windows
     * @return Error message (ok := empty string)
     */
    std::string initWorkingWindows(MaterialFlowType materialFlowType, PlanData& plan);

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
     * @brief Add edge overruns for the routes until the first working windows
     * @param plan Plan holding the routes and working windows
     */
    void addInitialOverruns(PlanData& plan);

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
     * @param resourcePointMassCapacities Map holding the resource point vertices and their current capacities <mass, volume>
     * @return Error message (ok := empty string)
     */
    std::string planInitialTrips(PlanData& plan,
                                  std::map<DirectedGraph::vertex_t, std::pair<double, double>>& resourcePointCapacities,
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
     * @param resourcePointCapacities Map holding the resource point vertices and their current capacities <mass, volume>
     * @param initRoutes [out] Planned route segments
     * @return Error message (ok := empty string)
     */
    std::string planInitialSegmentViaResource(PlanData& plan,
                                              const std::multimap<double, size_t>& indRoutes,
                                              std::map<DirectedGraph::vertex_t, std::pair<double, double>>& resourcePointCapacities,
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
     * @param materialFlowType Material flow type
     * @param [out] indRouteIndex of the route for which we have to plan.
     * @param [out] indRPIndex of the route point corresponding to the end of transportation.
     * @param [out] machine Machine of the corresponding route.
     * @param [out] currentWorkingWindow Current working window.
     * @return True if there is something to plan
     */
    bool getNextTransportationInfo(const PlanData& plan, MaterialFlowType materialFlowType,
                                   size_t& indRoute, size_t& indRP, size_t& indRP_ret, Machine& machine,
                                   PlanData::WorkingWindowInfo &nextWorkingWindow);


    /**
     * @brief Gets the bunker state for a machine after visiting the resource point
     * @param machine Machine
     * @param materialFlowType Material flow type
     * @param resourcePointMassCapacity Mass capacity of the resource point
     * @param resourcePointVolumeCapacity Mass capacity of the resource point
     * @param remainingMass Mass remaining to be worked in the field
     * @param remainingVol Volume remaining to be worked in the field
     * @param [in, out] bunker_mass Current bunker mass (updated to the resulting one)
     * @param [in, out] bunker_volume Current bunker volume (updated to the resulting one)
     * @return Reuturn the percentage [0, 1] of the requested capacity (e.g., if == 1, all requested capacity was supplied by the resource point)
     */
    static float getMachineBunkerStateAfterResourcePoint(const Machine& machine,
                                                        MaterialFlowType materialFlowType,
                                                        double resourcePointMassCapacity, double resourcePointVolumeCapacity,
                                                        double remainingMass, double remainingVol,
                                                        double &bunker_mass, double &bunker_volume);

protected:

    DirectedGraph::Graph m_graph; /**< Initial graph */
    std::vector<Route> m_baseRoutes; /**< Base (initial) routes (with no transportation) */
    std::map<MachineId_t, Machine> m_machines; /**< Machines */
    std::map<MachineId_t, MachineDynamicInfo> m_machineInitialStates; /**< Map containing the current states of the machines (inc. current location, bunker mass, etc.) */
    std::map<ResourcePointId_t, ResourcePointState> m_resourcePointCurrentStates; /**< Map containing the current states of the resource points (inc. capacities) */
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
