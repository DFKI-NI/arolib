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
 
#ifndef AROLIB_MULTIHARVPLANNER_H
#define AROLIB_MULTIHARVPLANNER_H

#include <ctime>
#include <sys/stat.h>

#include "directedgraph.hpp"
#include "arolib/types/route.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/planning/roundtripplanner.hpp"
#include "arolib/planning/activitiesswitchingplanner.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/logger.h"
#include "arolib/misc/filesystem_helper.h"

namespace arolib{

/**
 * @brief Planner to compute (multiple) harvester routes when no olvs make part of the process and the harvesters must transport the yield to the resource points
 */
class MultiHarvPlanner : public LoggingComponent
{
public:

    /**
     * @brief Holds the relevant data of each plan
     */
    struct PlanData{
        friend class MultiHarvPlanner;

        /**
         * @brief Holds the information of a working window
         */
        struct HarvestingWindowInfo{
            size_t indStart; /**< Index of the route-point where the windows starts */
            size_t indFinish; /**< Index of the route-point where the windows finishes */
        };

        DirectedGraph::Graph graph; /**< Updated graph */
        std::vector<Route> routes; /**< Map holding all planned harvester rutes (including transportation to resource points) */
        std::vector<double> planCosts; /**< Plan costs for the harvesting of each harvester route */
        std::map<size_t, std::vector<HarvestingWindowInfo>> harvestingWindows;  /**< Map containing the information of the harvesting windows for each route (key := route index) */
        double planOverallCost = 0; /**< Overall plan cost (for all harvester routes) */
        bool planOK = false; /**< Flag to know if the plan is OK */
    protected:

        /**
         * @brief Initialized the data
         * @param _graph Current graph
         * @param harv_routes Current harvester routes
         */
        void init(const DirectedGraph::Graph &_graph,
                  const std::vector<Route> &harv_routes);

        /**
         * @brief Cumpute and update the overall plan cost (for all harvester routes) using the plan costs for each one of the harvester routes
         */
        void updateOverallCost();

        /**
         * @brief Updates the harvesting windows information by adding a delta value to their indexes
         * @param indRoute Index of the routes whose harvesting windows will be updated
         * @param indRef only the windows with indStart >= indRef are updated
         * @param deltaInd Delta value
         */
        void updateHarvestingWindows(size_t indRoute, size_t indRef, int deltaInd);

        /**
         * @brief Updates the harvesting windows information by adding a delta value to their indexes
         * @param harvestingWindows Harvesting windows to be updated
         * @param indRoute Index of the routes whose harvesting windows will be updated
         * @param indRef only the windows with indStart >= indRef are updated
         * @param deltaInd Delta value
         */
        static void updateHarvestingWindows(std::map<size_t, std::vector<HarvestingWindowInfo>> &harvestingWindows, size_t indRoute, size_t indRef, int deltaInd);
    };

    /**
     * @brief Planner settings
     *
     * Inherits from RoundtripPlanner::PlannerSetting
     * @sa HarvPlan::PlannerSetting
     */
    struct PlannerSettings : public virtual RoundtripPlanner::PlannerSettings, public virtual ASP_GeneralSettings{
        double maxPlanningTime = 20;/**< planning timeout [s] */
        bool finishAtResourcePoint = true;  /**< Should the last olv overloading a harvester route be sent to the rosource point to download even if it is not full? */

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
     * @param graph (Initial) graph
     * @param harv_routes (processed) initial harvester routes without transportation
     * @param machines Machines
     * @param machineCurrentStates Map containing the current states of the machines (inc. current location, bunker mass, etc.)
     * @param settings Planner parameters/settings
     * @param edgeCostCalculator Edge Cost Calculator. Temporary: if = nullptr, uses internal astar functions
     * @param outputFolder Folder where the planning (search) information will be stored (if empty-string, no data will be saved)
     * @param logLevel Log level
     */
    explicit MultiHarvPlanner(const DirectedGraph::Graph &graph,
                              const std::vector<Route> &harv_routes,
                              const std::vector<Machine> &machines,
                              const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                              const PlannerSettings& settings,
                              std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                              const std::string& outputFolder = "",
                              LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Reset/reinitialize
     */
    void reset();

    /**
     * @brief Compute plans for all harvester routes
     * @return Error message (ok := empty string)
     */
    std::string planAll();

    /**
     * @brief Retrieve the data from the (best) computed plan
     * @return Data from the (best) computed plan
     */
    const PlanData& getPlanData() const {return m_bestPlan;}

    /**
     * @brief Get a copy of all planned harvester routes
     * @return Copy of all planned harvester routes
     */
    std::vector<Route> getPlannedRoutes();

protected:

    /**
     * @brief Add edge overruns fro the harvester routes
     * @return Map of the machines (working group)
     */
    void addHarvesterSoilValues( size_t routeIndex, size_t ind0, size_t ind1);

    /**
     * @brief Compute the information of the harvesting windows for all routes
     * @param plan Plan holding the routes and harvesting windows
     * @return Error message (ok := empty string)
     */
    std::string calcHarvestingWindows(PlanData& plan);

    /**
     * @brief Initializes some the route points bunker masses
     *
     * All route points before and including the first harvesting windown indStart will have the current machine's bunker mass. If the routes have no harvesting windows, this will apply to all route points.
     * All other route points' bunker masses will be set based on the harvesting windows and harvested masses (assumning that after unload the bunker is empty)
     * It is assumed that the windows are ordered in the vectors.
     * @param plan Plan holding the routes and harvesting windows
     * @return Error message (ok := empty string)
     */
    std::string initRoutesBunkerMasses(PlanData& plan);
    /**
     * @brief Add edge overruns fro the harvester routes
     * @param plan Plan holding the routes and harvesting windows
     */
    void addHarvesterSoilValues(PlanData& plan);

    /**
     * @brief Add edge overruns for the harvester routes (segments)
     * @param graph Graph to be updated
     * @param machine Machine
     * @param route Route
     * @param ind0 Start route-point index
     * @param ind0 End route-point index
     */
    void addHarvesterSoilValues(DirectedGraph::Graph& graph, const Machine &machine, const Route& route, size_t ind0, size_t ind1);

    /**
     * @brief Add the initial visit periods of the harvester routes
     *
     * Adds the visit periods from rotues before harvesting. If there are no harvesting windows for the route, it adds the periods for all route points
     * @param plan Plan holding the routes and harvesting windows
     */
    void addInitialVisitPeriods(PlanData& plan);

    /**
     * @brief Add the final visit periods of the harvester routes
     *
     * Adds the visit periods from rotues after harvesting.
     * @param plan Plan holding the routes and harvesting windows
     */
    void addFinalVisitPeriods(PlanData& plan);

    /**
     * @brief Add the visit periods of the harvester route segment
     * @param graph Graph to be updated
     * @param machine Machine
     * @param route Route
     * @param ind0 Start route-point index
     * @param ind0 End route-point index
     */
    void addVisitPeriods(DirectedGraph::Graph& graph, const Machine &machine, const Route& route, size_t ind0, size_t ind1);

    /**
     * @brief Compute the updated routes (with unload trips) for all machines
     * @param plan Plan holding the routes and harvesting windows
     * @return Error message (ok := empty string)
     */
    std::string planUnloadTrips(PlanData& plan);

    /**
     * @brief Gets the information about the next transportation planning to be done
     * @param plan Plan holding the routes
     * @param harvestingWindows Map of harvesting windows to be used in the analysis
     * @param indRoute[out] Index of the harvester route for which we have to plan.
     * @param indRP[out] Index of the route point corresponding to the moment of transportation.
     * @param harv[out] Harvester of the corresponding route.
     * @return True if there is something to plan
     */
    bool getNextTransportationInfo(const PlanData& plan,
                                   const std::map<size_t, std::vector<PlanData::HarvestingWindowInfo>> &harvestingWindows,
                                   size_t& indRoute, size_t& indRP, size_t& indRP_ret, Machine& harv);

protected:

    DirectedGraph::Graph m_graph; /**< Initial graph */
    std::vector<Route> m_harvesterRoutes; /**< Initial harvester routes (with no transportation) */
    std::map<MachineId_t, Machine> m_harvesters; /**< Harvesters */
    std::map<MachineId_t, MachineDynamicInfo> m_machineInitialStates; /**< Map containing the current states of the machines (inc. current location, bunker mass, etc.) */
    PlannerSettings m_settings; /**< Planner parameters/settings */
    std::shared_ptr<IEdgeCostCalculator> m_edgeCostCalculator = nullptr; /**< Edge cost calculator */

    PlanData m_bestPlan; /**< Data of the current BEST plan */
    PlanData m_currentPlan; /**< Data of the current plan */
    std::string m_outputFolder = ""; /**< Folder where the planning (search) information of all permutations will be stored (if empty-string, no data will be saved) */

    static const double HarvMaxCapacityMultiplier; /**< Multiplier used to decide whether a machine is 'full' or not (used max bunker capacity = machine max capacity * HarvMaxCapacityMultiplier) */
};

}

#endif // AROLIB_MULTIHARVPLANNER_H
