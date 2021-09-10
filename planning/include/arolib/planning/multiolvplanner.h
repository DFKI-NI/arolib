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
 
#ifndef AROLIB_MULTIOLVPLANNER_H
#define AROLIB_MULTIOLVPLANNER_H

#include <future>
#include <mutex>
#include <ctime>
#include <chrono>
#include <sys/stat.h>

#include "directedgraph.hpp"
#include "arolib/types/route.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/planning/olvPlan.hpp"
#include "arolib/planning/overloadactivitiesplanner.h"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/logger.h"
#include "arolib/misc/filesystem_helper.h"

namespace arolib{

/**
 * @brief Planner to compute (multiple) OLV routes for the harvester routes
 */
class MultiOLVPlanner : public LoggingComponent
{
public:

    /**
     * @brief Strategy to order the OLVS to compute the overloading activities
     *
     * The order in which the OLVs will be used will directly affect the calculation of the overloading activities/windows.
     * If the strategy calls to generate different OL activities and plans (e.g. CHECK_ALL_PERMUTATIONS, CHECK_ALL_PERMUTATIONS__START_FOLLOWING_OLVS_ORDER) and PlannerSettings.numFixedInitalOlvsInOrder = n > 0, the first n olvs present in the assigned working group will be fixed, and only the remaining OLV will be permuted to check for the best plan
     */
    enum OlvOrderStrategy{
        FOLLOW_OLVS_ORDER, /**< Compute the overload activities using the same order in which the olvs appear in the (assigned) working group vector (only this permutation will be planned) */
        ESTIMATE_OPTIMAL_ORDER, /**< The optimal order of olvs will be estimated based on the current states of the machines (location, bunker mass,...), respecting numFixedInitalOlvsInOrder, and only this permutation will be planned*/
        CHECK_ALL_PERMUTATIONS, /**< Check the all options (permutations) of ordering the OLVs, starting with the estimated optimal order, and select the plan with the permutation that generates the lowest cost */
        CHECK_ALL_PERMUTATIONS__START_FOLLOWING_OLVS_ORDER /**< Check the all options (permutations) of ordering the OLVs, starting with the order given by the assigned working group vector (this should be the one expected to generate the lowest costs), and select the plan with the permutation that generates the lowest cost */
    };
    /**
      * @brief Get the OlvOrderStrategy (enum) from its int value
      */
    static OlvOrderStrategy intToOlvOrderStrategy(int value);

    /**
     * @brief Strategy to assign OLVs to the harvester routes
     */
    enum OlvAssignmentStrategy{
        HARVESTER_EXCLUSIVE, /**< A group of OLVs is selected for each harvester route exclusivelly (each OLV will overload only from a harverset) */
        SHARED_OLVS /**< (Not implemented yet) The OLVs can overload from different harvesters */
    };
    /**
      * @brief Get the OlvAssignmentStrategy (enum) from its int value
      */
    static OlvAssignmentStrategy intToOlvAssignmentStrategy(int value);

    /**
     * @brief Option regarding the number of threads used in planning
     */
    enum ThreadsOption{
        SINGLE_THREAD, /**< Compute all in the same thread */
        MULTIPLE_THREADS /**< Compute in multiple threads if possible */
    };
    /**
      * @brief Get the ThreadsOption (enum) from its int value
      */
    static ThreadsOption intToThreadsOption(int value);

    /**
     * @brief Holds the relevant data of each plan
     */
    struct PlanData{
        friend class MultiOLVPlanner;
        DirectedGraph::Graph graph; /**< Updated graph */
        std::vector<int> lastHarvIndexes; /**< Indexes corresponding to the last route-point of the last overload activity that was planned (i.e. used to know the route point of the next overload start to be planned) */
        std::map<MachineId_t, Route> plannedRoutes; /**< Map holding all planned OLV rutes */
        std::vector<Route> harvesterRoutes; /**< Current (updated) hasvester routes */
        std::vector<double> overallDelays; /**< Holds the delays of all harvesters (w.r.t. their original planned routes) */
        std::vector<double> planCosts; /**< Plan costs for the overloading of each harvester route */
        double planOverallCost = 0; /**< Overall plan cost (for all harvester routes) */
        double subplanCost_exclusive = std::numeric_limits<double>::max(); /**< (deprecated) Cost of the plan for a sub-plan (route segment) for OlvAssignmentStrategy=HARVESTER_EXCLUSIVE */
        bool planOK = false; /**< Flag to know if the plan is OK */
        int routeIdDelta = 100; /**< Used to set the id of the generated OLV routes so that they are not repeated */
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
    };

    /**
     * @brief Planner settings
     *
     * Inherits from OverloadActivitiesPlanner::PlannerSettings and OLVPlan::PlannerSetting
     * Regarding OLVs order and OLV assignment to harvester routes (for OlvAssignmentStrategy::HARVESTER_EXCLUSIVE):
     *    If there is more than one harvester route, the OLVs will be assigned one by one to each harvester route in the order given by the user.
     *    For example, if there are 2 harvester routes and the working group given by the user is <HARV1, OLV1, HARV2, OLV2, OLV3, OLV4, OLV5>, the assignment will be:
     *       First harvester route: <OLV1, OLV3, OLV5>
     *       Second harvester route: <OLV2, OLV4>
     * Regarding OLVs order in the assigned working group:
     *    The overload activities/windows are computed depending on the order of the working group (OLVs) assigned to the harvester route (the order might not be the same as the order given  by the user)
     *    If numFixedInitalOlvsInOrder = n > 0 (if < 0, this value will be recalculated to be >= 0 based on the machine current states), the order of the first n OLVs in the assigned working group will be kept fixed. The OLV assignement operation (explained above) must be taken into account.
     *    Depending on the olvOrderStrategy, the optimal order of the OLVs can be estimated and set, or plans for each one of the possible permutations will be generated (selecting the plan with the lowest cost), BUT respecting the given numFixedInitalOlvsInOrder.
     *    If, for example, the OLV working group assigned to the route of HARV1 is <OLV5, OLV1, OLV3>, and numFixedInitalOlvsInOrder = 1 and olvOrderStrategy = CHECK_ALL_PERMUTATIONS, the following plans (permutations) will be generated and compared: <OLV5, OLV1, OLV3>, <OLV5, OLV3, OLV1>
     *    If, for example, the OLV working group assigned to the route of HARV1 is <OLV5, OLV1, OLV3>, and numFixedInitalOlvsInOrder = 1 and olvOrderStrategy = ESTIMATE_OPTIMAL_ORDER, it will estimate which of the possible remaining combinations ( <OLV5, OLV1, OLV3>, <OLV5, OLV3, OLV1> ) would generate the lowest plan (without planning for all to compare actual costs) and generate a plan only for that combination
     * @sa OverloadActivitiesPlanner::PlannerSettings
     * @sa OLVPlan::PlannerSetting
     */
    struct PlannerSettings : public virtual OverloadActivitiesPlanner::PlannerSettings, public virtual OLVPlan::PlannerSettings{
        double max_waiting_time = 10000; /**< The maximum amount of time [s] that the harvester may wait for the next olv (overload activity). */
        bool sendLastOlvToResourcePoint = true;  /**< Should the last olv overloading a harvester route be sent to the rosource point to download even if it is not full? */
        double harvestedMassLimit = 0; /**< if an overload activity surpaces this value, no more overload activities will be processed (i.e. partial plans will be generated, for only a part of the harvester route). Disregarded if <= 0 (warning: in the OverloadPlanner, if < 0, an optimal value will be calculated) */
        int numOLActivitiesPerSubplan = -1; /**< (not implemented) Subplans will be planned every numOLActivitiesPerSubplan, instead of a single plan with all overload activities. disregarded if <= 0 */
        OlvOrderStrategy olvOrderStrategy = OlvOrderStrategy::CHECK_ALL_PERMUTATIONS;/**< Strategy to order the OLVS to compute the overloading activities */
        double max_planning_time = 20;/**< planning timeout [s]. Current plan been executed will be calcelled and no more plans (for other olv-order permulations) will be executed. The best plan until this moment (if any) will be set as result */
        int numFixedInitalOlvsInOrder = 0;/**< The order of the first n olvs in the assigned working group will be kept fixed (no permutations will be tested in these machines), whereas the rest will be either calculated to be optimal or permuted to get the best plan. If < 0, the optimal number of fixed OLVs will be estimated and used. */
        OlvAssignmentStrategy olvAssignmentStrategy = OlvAssignmentStrategy::HARVESTER_EXCLUSIVE; /**< Strategy to assign OLVs to the harvester routes (at the moment, only HARVESTER_EXCLUSIVE is supported) */
        ThreadsOption threadsOption = ThreadsOption::MULTIPLE_THREADS;/**< Option regarding the number of threads used in planning */

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
     * @param harv_routes (processed) harvester routes
     * @param machines Machines (the order might important, depending on 'settings')
     * @param machineCurrentStates Map containing the current states of the machines (inc. current location, bunker mass, etc.)
     * @param settings Planner parameters/settings
     * @param edgeCostCalculator Edge Cost Calculator. Temporary: if = nullptr, uses internal astar functions
     * @param addHarvRoutesSoilValues If true, the soil values (edge overruns)
     * @param outputFolder Folder where the planning (search) information of all permutations will be stored (if empty-string, no data will be saved)
     * @param logLevel Log level
     */
    explicit MultiOLVPlanner(const DirectedGraph::Graph &graph,
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
     * @brief Compute plans (harvester and olv routes) for all harvester routes folllowing the given OlvAssignmentStrategy setting
     */
    std::string planAll();

    /**
     * @brief Retrieve the data from the (best) computed plan
     * @return Data from the (best) computed plan
     */
    const PlanData& getPlanData() const {return m_bestPlan;}

    /**
     * @brief Get a copy of all planned OLV routes
     * @return Copy of all planned OLV routes
     */
    std::vector<Route> getPlannedRoutes();

private:
    /**
     * @brief Add edge overruns fro the harvester routes
     * @return Map of the machines (working group)
     */
    void addHarvesterSoilValues( const std::map<MachineId_t, Machine>& harvesters);

    /**
     * @brief Compute plans (harvester and olv routes) for all harvester routes following OlvAssignmentStrategy:HARVESTER_EXCLUSIVE
     * @sa OlvAssignmentStrategy
     * @return Error message (empty string := OK)
     */
    std::string planAll_exclusive();

    /**
     * @brief Compute plans (harvester and olv routes) for all harvester routes following OlvAssignmentStrategy:HARVESTER_EXCLUSIVE (in a single thread)
     * @sa OlvAssignmentStrategy
     * @return Error message (empty string := OK)
     */
    std::string planAll_exclusive__singleThread();

    /**
     * @brief Compute plan (harvester and olv routes) for all harvester routes following OlvAssignmentStrategy:HARVESTER_EXCLUSIVE, without checking different permutations (only the one given by the input parameter 'olvs')
     * @param olvs Vector containing the assigned olvs to each harvester route (the index of the first vector of 'olvs' corresponds to the index of the assigned harvester route in the harvester-routes' (internal) vector
     * @sa OlvAssignmentStrategy
     * @return Error message (empty string := OK)
     */
    std::string planAll_exclusive_singlePlan(const std::vector<std::vector<Machine> > &olvs);

    /**
     * @brief (not implemented yet) Compute plan (harvester and olv routes) for all harvester routes following OlvAssignmentStrategy:HARVESTER_SHARED_OLVS
     * @sa OlvAssignmentStrategy
     * @return Error message (empty string := OK)
     */
    std::string planAll_shared();


    /**
     * @brief Compute a single plan (harvester and olv routes) for one harvester route following OlvAssignmentStrategy:HARVESTER_EXCLUSIVE
     * @param indHarvRoute Index of the harvester route
     * @param olvs Olvs assigned to the harvester route
     * @param max_planning_time Maximum planning time (disregarded if <= 0)
     * @param folderName base folder name to save planning debug info (disregarded if empty)
     * @param numThreads Number of threads used in the planning
     * @sa OlvAssignmentStrategy
     * @return Error message (empty string := OK)
     */
    std::string planSingle_exclusive(size_t indHarvRoute, std::vector<Machine> olvs, double max_planning_time, const std::string &folderName, bool disableFurtherLogging);


    /**
     * @brief Update the costs of the plan and check if they are higher than the costs of the current BEST plan
     * @param plan Plan data
     * @param olvplan_map (sub)Plans (one per machine) to be analized
     * @param delay Overall harvester delays for the current plan
     * @param delayLocation Location of the delay
     * @param indHarvRoute Index of the corresponding harvester route
     * @param harv Corresponding harvester
     * @return True iif the costs of the current plan are lower than the costs of the curren BEST plan
     */
    bool updatePlanCost(PlanData &plan, const std::map<int, OLVPlan> &olvplan_map, double delay, const Point &delayLocation, size_t indHarvRoute, const std::shared_ptr<Machine> harv) const;

    /**
     * @brief Updates the current best plan
     * @param newPlan Plan used to update the best plan.
     * @param indHarvRoute Index of the harvester route for which we are planning.
     * @param plannedRoutesRefIndex The route points of this machine will be updated starting from this index. Used in case of shared olvs. If a machine is not in the map, its route will remain untouched
     * @param checkCost If true, it will update the best plan iif the cost of the new plan is lower that the one from the best plan
     */
    std::string updateBestPlan(const PlanData& newPlan, size_t indHarvRoute, std::map<MachineId_t, size_t> plannedRoutesRefIndex, bool checkCost);

    /**
     * @brief Reorders the overload machines based on the machines' current states and the assigned harvester route.
     * @param _overloadMachines Assigned OLVs in the original order.
     * @param harvesterRoute Harvester route
     * @param machineCurrentStates Map containing the current states of the machines (inc. current location, bunker mass, etc.)
     * @param switching_strategy Strategy to stwitch from one overloading activity/window to the next
     * @param numFixedInitalOlvsInOrder The order of the first n olvs in '_overloadMachines' will be kept fixed, whereas the rest will be either checked to estimate the optimal order of the remaining machines
     * @param harvestedMassLimit If an overload activity surpaces this value, no more overload activities will be processed/checked. Disregarded if <= 0.
     * @param _logger (optional) Pointer to the parent logger (to use the parent's logLevel). If NULL, it creates a default logger.
     * @return Machines in the estimated best order.
     */
    static std::vector<Machine> reorderWorkingGroup(const std::vector<Machine>& _overloadMachines,
                                                    const Route &harvesterRoute,
                                                    const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                    const OverloadActivitiesPlanner::PlannerSettings &activitiesPlannerSettings,
                                                    const size_t &numFixedInitalOlvsInOrder = 0,
                                                    double harvestedMassLimit = -1,
                                                    Logger *_logger = nullptr);

    /**
     * @brief Estimate how many of the first OLVs in irder should be kept in that position for the calculation of OLVs.
     *
     * For example, if the firs olv is overloading at the moment of (re)planning, and the second is following closelly the harvester and the first OLV waiting for its overloading window to start, this two OLVs should be kept in that order, whereas the remaining ones can be checked for the best permutation.
     * @param harvesterRoute Harvester route
     * @param olv_machines Assigned OLVs in order.
     * @param machineCurrentStates Map containing the current states of the machines (inc. current location, bunker mass, etc.)
     * @param _logger (optional) Pointer to the parent logger (to use the parent's logLevel). If NULL, it creates a default logger.
     * @return Estimated number of initial fixed OLVs.
     */
    static size_t estimateNumFixedInitalOlvsInOrder(const Route &harv_route,
                                                    const std::vector<Machine> &olv_machines,
                                                    const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                    Logger *_logger = nullptr);


    /**
     * @brief Update the machine-relations in the route points of the harvester routes with their corresponding planned OLV routes
     */
    void updateHarvesterRouteRelations();

protected:

    DirectedGraph::Graph m_graph; /**< Initial graph */
    std::vector<Route> m_harvesterRoutes; /**< Initial harvester routes */
    std::vector<Machine> m_olvs; /**< OLVs */
    std::map<MachineId_t, Machine> m_harvesters; /**< Harvesters */
    std::map<MachineId_t, MachineDynamicInfo> m_machineInitialStates; /**< Map containing the current states of the machines (inc. current location, bunker mass, etc.) */
    PlannerSettings m_settings; /**< Planner parameters/settings */
    std::shared_ptr<IEdgeCostCalculator> m_edgeCostCalculator = nullptr; /**< Edge cost calculator */

    mutable std::mutex m_mutex_bestPlan; /**< Mutex used in updates of the best plan */

    PlanData m_bestPlan; /**< Data of the current BEST plan */
    PlanData m_currentPlan; /**< Data of the current plan */
    std::string m_outputFolder = ""; /**< Folder where the planning (search) information of all permutations will be stored (if empty-string, no data will be saved) */
    std::string m_outputFolderEd = ""; /**< Folder where the planning (search) information of the current permutation will be stored (if empty-string, no data will be saved) */
};

}

#endif // AROLIB_MULTIOLVPLANNER_H
