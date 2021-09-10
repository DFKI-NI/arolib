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
 
#ifndef OVERLOADPLANNERHELPER_H
#define OVERLOADPLANNERHELPER_H

#include <iostream>
#include <fstream>
#include "directedgraph.hpp"
#include "astar.hpp"
#include "simpleMultiOLVPlanner.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/types/machine.hpp"
#include "arolib/types/headlandroute.hpp"
#include "arolib/types/route.hpp"
#include "arolib/planning/planningworkspace.h"

#include <boost/geometry/geometry.hpp>


namespace arolib{

/**
 * @brief Depecated. Used together with the deprecated SimpleMultiOLVPlanner
 */
class OverloadPlannerHelper : public LoggingComponent{
public:

    enum OlvOrderStrategy{
        KEEP_ORIGINAL_ORDER,
        ESTIMATE_OPTIMAL_ORDER,
        CHECK_ALL_PERMUTATIONS,
        CHECK_ALL_PERMUTATIONS__START_WITH_ORIGINAL
    };
    static OlvOrderStrategy intToOlvOrderStrategy(int value);

    struct PlannerSettings : SimpleMultiOLVPlanner::PlannerSettings{
        OlvOrderStrategy olvOrderStrategy = OlvOrderStrategy::CHECK_ALL_PERMUTATIONS;
        double max_planning_time = 20;
        int numFixedInitalOlvsInOrder = 0;
    protected:
        using SimpleMultiOLVPlanner::PlannerSettings::planning_time_limit;
    };

    explicit OverloadPlannerHelper(const LogLevel& logLevel = LogLevel::INFO);
    virtual ~OverloadPlannerHelper(){}

    /**
      * @brief Creates the graph from the routes and headland_points
      *
      */
    void build_directGraph(const Subfield &sf,
                           const std::vector<Route> &harvInfieldRoutes,
                           const std::vector<HeadlandRoute> &harvHeadlandRoutes,
                           const OutFieldInfo &outFieldInfo,
                           const std::vector<Machine> &machines,
                           const std::map<MachineId_t, MachineDynamicInfo> &machineInitialStates,
                           double defYield, //in Kg/m²
                           const ArolibGrid_t& yieldmap,
                           const ArolibGrid_t& soilmap,
                           double machineWidthIF,
                           double machineWidthHL,
                           bool bePreciseWithMaps);

    /**
      * @brief Creates the graph from the routes and headland_points
      *
      */
    void build_directGraph(PlanningWorkspace& pw,
                           size_t subfieldIdx,
                           const std::vector<Route> &harvInfieldRoutes,
                           const std::vector<HeadlandRoute> &harvHeadlandRoutes,
                           const OutFieldInfo &outFieldInfo,
                           double defYield,
                           double machineWidthIF,
                           double machineWidthHL,
                           bool bePreciseWithMaps);


   void insert_olv_route_into_graph(const Route &olv_route, const Machine &olv);

   /**
    * @brief Update the routepoint's timestamp in the graph according to the route that has already been harvested.
    * @param harv_route The route that has already been harvested.
    * @param set_to_zero True if the timestamp should be set to 0.0, otherwise it will be updated according to harv_route
    */
   void update_graph_rps_timestamp(const Route &harv_route, bool set_to_zero);

   DirectedGraph::Graph getGraph() const;

   std::vector<Route> getHarvesterRoutes() const {return m_harvester_routes;}

   /**
    * @brief plan
    * @param harv_routes The routes of all harvesters.
    * @param machines Available machines.
    * @param unloading_duration Time in seconds that the olv needs to unload the crop at the resource point.
    * @param optimizeOLVOrder Try different permutations of the order of olvs?
    * @param max_planning_time Maximum time in seconds that the planner may need for plan generation.
    * @param switching_strategy Should the olv switch only in the headland or is it allowed to switch to the next olv on the field?
    * @param waiting_inc The timediff in seconds that the waiting time will be increased in each iteration.
    * @param max_waiting_time The maximum amount of time in seconds that the harvester may wait for the next olv (for each turn).
    * @param numFixedInitalOlvsInOrder If checking all permutations or estimated optimal order, the given number of initial olvs will be fixed, hence not permuting the n-first olvs. If negative, the decition will be made automatically
    * @return
    */
   std::vector<Route> plan(const std::vector<Route> &harv_routes,
                           const std::vector<Machine> &machines,
                           const std::map<MachineId_t, arolib::MachineDynamicInfo>& machineCurrentStates,
                           const OutFieldInfo &outFieldInfo,
                           const PlannerSettings& settings);
   

protected:

    void updateGraph(double current_time);
    size_t estimateNumFixedInitalOlvsInOrder(const Route &harv_route,
                                             const std::vector<Machine> &olv_machines,
                                             const std::map<MachineId_t, arolib::MachineDynamicInfo>& machineCurrentStates);

protected:
    DirectedGraph::Graph m_graph;
    std::vector<Route> m_harvester_routes;

//    double extractHarvesterSwitchingTime(DirectedGraphBuilder::vertex_t goal_vt);

};

}



#endif //OVERLOADPLANNERHELPER_H
