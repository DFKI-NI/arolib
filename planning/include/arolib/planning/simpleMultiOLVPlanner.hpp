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
 
#ifndef SIMPLEMULTIOLVPLANNER_HPP
#define SIMPLEMULTIOLVPLANNER_HPP

#include <ctime>

#include "directedgraph.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/types/route.hpp"
#include "arolib/planning/olvPlan.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/logger.h"


namespace arolib{

/**
 * @brief Depecated. Used together with the deprecated OverloadPlannerHelper
 */

class SimpleMultiOLVPlanner : public LoggingComponent {

public:

    enum OLVSwitchingStrategy{
        INFIELD,
        HEADLAND_ONLY,
    };

    enum ReplanningStrategy{
        DELAY_HARVESTER,
        DELAY_HARVESTER_IN_HEADLAND,
        REPLAN_OLV_ACITIVITIES
    };

    struct PlannerSettings : OLVPlan::PlannerSettings{
        SimpleMultiOLVPlanner::OLVSwitchingStrategy olvSwitchingStrategy = SimpleMultiOLVPlanner::OLVSwitchingStrategy::INFIELD; //Strategy for switching the olvs. In the field vs only in the headland.
        double planning_time_limit = std::numeric_limits<double>::max(); //
        double waiting_inc = 100.0; // time in seconds by which the waiting time of the harvester is increased in each iteration if no plan is found for the overloading activity
        double max_waiting_time = 10000; //The maximum amount of time in seconds that the harvester may wait for the next olv (overload activity).
        ReplanningStrategy replanning_strategy = ReplanningStrategy::DELAY_HARVESTER; //
        bool sendLastOlvToResourcePoint = true; //
        bool estimateOptimalOlvOrder = false; //
        double harvestedMassLimit = -1; //if the overload activity surpaces this value, no more overload activities will be processed. disregarded if <= 0
        int numOLActivitiesPerSubplan = -1; //subplans will be planned every numOLActivitiesPerSubplan, instead of a single plan with all overload activities. disregarded if <= 0
    };

    /**
     * @param graph Graph of the field.
     * @param harvester_route Route of the harvester.
     * @param overloadMachines Available OLVs
     * @param machineCurrentStates machine current states.
     */
    explicit SimpleMultiOLVPlanner(const DirectedGraph::Graph &graph,
                                   const Route &harvester_route,
                                   const std::vector<Machine> &_overloadMachines,
                                   const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                   const PlannerSettings& settings,
                                   const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Plan routes of multiple olvs for one harvester route.
     */
    std::string plan(double max_plan_cost = std::numeric_limits<double>::max(),
                     bool estimateOptimalOlvOrder = false);

    virtual ~SimpleMultiOLVPlanner(){}

    std::vector<Route> getRoutes() const {return m_planned_routes;}

    Route getHarvesterRoute() const {return m_harvester_route;}

    double getCost() const {return m_plan_cost;}

    const DirectedGraph::Graph& getGraph() const {return m_graph;}

    /**
     * @brief Reorders the overload machines based on the machines' current states and the harvester route.
     * @param _overloadMachines Machines in the original order.
     * @param harvesterRoute harvester route.
     * @param harvesterRoute machines' current states.
     * @return Machines in the estimated best order.
     */
    static std::vector<Machine> reorderWorkingGroup(const DirectedGraph::Graph& graph,
                                                    const std::vector<Machine>& _overloadMachines,
                                                    const Route &harvesterRoute,
                                                    const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                    OLVSwitchingStrategy switching_strategy,
                                                    const size_t &numFixedInitalOlvsInOrder = 0,
                                                    double harvestedMassLimit = -1,
                                                    Logger *_logger = nullptr);


    static OLVSwitchingStrategy intToOLVSwitchingStrategy(int value);
    static ReplanningStrategy intToReplanningStrategy(int value);

private:
    int m_routeId_count;

    DirectedGraph::Graph m_graph;

    Route m_harvester_route;
    std::vector<Machine> m_overloadMachines;
    std::map<MachineId_t, MachineDynamicInfo> m_machineInitialStates;

    double m_plan_cost = 0;
    double m_overallDelay = 0;
    PlannerSettings m_settings;
    int lastHarvIndex = -1;

    std::vector<Route> m_planned_routes; // planned routes of all olvs

    static std::vector<OLVPlan::OverloadInfo> computeFieldOverloadPoints(const DirectedGraph::Graph& graph,
                                                                         const Route &harvester_route,
                                                                         const std::vector<Machine> &overloadMachines,
                                                                         const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                                         double harvestedMassLimit = -1,
                                                                         Logger *_logger = nullptr);

    static std::vector<OLVPlan::OverloadInfo> computeTracksEndsOverloadPoints(const DirectedGraph::Graph& graph,
                                                                              const Route &harvester_route,
                                                                              std::vector<Machine> overloadMachines,
                                                                              const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                                              double harvestedMassLimit = -1,
                                                                              Logger *_logger = nullptr);

    static int run_to_next_type(const Route &harvester_route, int start, RoutePoint::RoutePointType type);

    bool updateCurrentPlanCost(const std::map<int, OLVPlan> &olvplan_map,
                               const double &delay,
                               const Astar::AStarSettings &aStarSettings,
                               const double &max_cost);

    void updateHarvesterRouteRelations();


    void saveOverloadActivities(const std::vector<OLVPlan::OverloadInfo>& overloadActivities,
                                const std::string& folder = "/tmp");

    static const double InitialOlvMaxCapacityMultiplier;

};

}

#endif // MULTIOLVPLANNER_HPP
