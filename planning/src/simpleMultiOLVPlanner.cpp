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
 
#include "arolib/planning/simpleMultiOLVPlanner.hpp"

#include <map>
#include <sstream>

namespace arolib{

const double SimpleMultiOLVPlanner::InitialOlvMaxCapacityMultiplier = 0.7;

SimpleMultiOLVPlanner::SimpleMultiOLVPlanner(const DirectedGraph::Graph &graph,
                                 const Route &harvester_route,
                                 const std::vector<Machine> &_overloadMachines,
                                 const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                 const PlannerSettings& settings,
                                 const LogLevel& logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_routeId_count( harvester_route.route_id * 100 ),
    m_graph(graph),
    m_harvester_route(harvester_route),
    m_overloadMachines (_overloadMachines),
    m_machineInitialStates(machineCurrentStates),
    m_settings(settings){

    m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "planHarvesterRoute route_id: " + std::to_string( m_harvester_route.route_id )
                                                    + " route.machine_id: " + std::to_string( m_harvester_route.machine_id )
                                                    + " route size: " + std::to_string( m_harvester_route.route_points.size() ) );
    m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "overloadMachines.size(): " + std::to_string( _overloadMachines.size() ) );
}

std::string SimpleMultiOLVPlanner::plan(double max_plan_cost, bool estimateOptimalOlvOrder)
{
    if (m_overloadMachines.size() == 0) {
        m_logger.printOut(LogLevel::CRITIC, __FUNCTION__, "No overloading machines assigened.");
        return "No overloading machines assigened.";
    }
    if (m_harvester_route.route_points.size() == 0) {
        m_logger.printOut(LogLevel::CRITIC, __FUNCTION__, "Route does not contain any points.");
        return "Route does not contain any points.";
    }

    std::vector<Machine> overloadMachines = m_overloadMachines;

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Resource points: ");

    std::vector<DirectedGraph::vertex_t> resource_vertices;
    std::vector<DirectedGraph::vertex_t> accessPoint_vertices;
    for(DirectedGraph::vertex_iter vp = vertices(m_graph); vp.first != vp.second; vp.first++){
        RoutePoint rp = m_graph[*vp.first].route_point;
        if (rp.type == RoutePoint::RoutePointType::RESOURCE_POINT) {
            resource_vertices.push_back(*vp.first);
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "\t\t\tRessource Point: RoutePoint: " + rp.point.toString() );
        }
        else if (rp.type == RoutePoint::RoutePointType::FIELD_ENTRY || rp.type == RoutePoint::RoutePointType::FIELD_EXIT) {
            accessPoint_vertices.push_back(*vp.first);
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "\t\t\tField Access Point: RoutePoint: " + rp.point.toString() );
        }
    }

    if(estimateOptimalOlvOrder && m_overloadMachines.size() > 1)
        overloadMachines = reorderWorkingGroup(m_graph, overloadMachines, m_harvester_route, m_machineInitialStates, m_settings.olvSwitchingStrategy);

//        // first compute points where we change overload vehicles
    double first_olv_bunker_level = 0.0;
    auto it0 = m_machineInitialStates.find(overloadMachines.at(0).id);
    if (it0 != m_machineInitialStates.end()) {
        first_olv_bunker_level = it0->second.bunkerMass;
    }
    std::vector<OLVPlan::OverloadInfo> overloadActivities;

    if (m_settings.olvSwitchingStrategy == HEADLAND_ONLY) {
        overloadActivities = computeTracksEndsOverloadPoints(m_graph,
                                                             m_harvester_route,
                                                             overloadMachines,
                                                             m_machineInitialStates,
                                                             m_settings.harvestedMassLimit,
                                                             &m_logger);
    } else {
        overloadActivities = computeFieldOverloadPoints(m_graph,
                                                        m_harvester_route,
                                                        overloadMachines,
                                                        m_machineInitialStates,
                                                        m_settings.harvestedMassLimit,
                                                        &m_logger);
    }


    std::set<MachineId_t> olvsInDutty;
    for(auto &a : overloadActivities){
        if( olvsInDutty.find(a.machine.id) != olvsInDutty.end() )
            break;
        olvsInDutty.insert(a.machine.id);
    }

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Computed " + std::to_string( overloadActivities.size() ) + " overloadActivities (" + (m_settings.olvSwitchingStrategy == HEADLAND_ONLY?"HEADLAND_ONLY":"IN_FIELD") + ")");

    //saveOverloadActivities(overloadActivities);

    std::map<int, OLVPlan> olvplan_map;
    // init new olvplan objects (no planning involved)
    for (int i = 0; i < overloadMachines.size(); ++i){
        if(olvsInDutty.find(overloadMachines.at(i).id) != olvsInDutty.end()){
            double olv_bunker_level = 0;
            auto it0 = m_machineInitialStates.find(overloadMachines.at(i).id);
            if (it0 != m_machineInitialStates.end()) {
                olv_bunker_level = it0->second.bunkerMass;
            }
            auto it_plan = olvplan_map.insert(std::make_pair(overloadMachines.at(i).id, OLVPlan (overloadMachines.at(i),
                                                                                                 resource_vertices,
                                                                                                 accessPoint_vertices,
                                                                                                 m_settings,
                                                                                                 olv_bunker_level,
                                                                                                 "",
                                                                                                 m_logger.logLevel())));
            if(it_plan.second)
                it_plan.first->second.logger().setParent(&m_logger);
        }
    }

    m_overallDelay = 0;
    clock_t time_start = clock();

    int numOLActivitiesPerSubplan = m_settings.numOLActivitiesPerSubplan;
    if(numOLActivitiesPerSubplan <= 0)
        numOLActivitiesPerSubplan = overloadActivities.size();

    // plan each olv activity
    for (auto it = overloadActivities.begin() ; it != overloadActivities.end()  ; ++it) {

        bool result = true;
        double delay = 0.0;
        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Planning overload for olv " + std::to_string( it->machine.id ) + "..." );

        do{
            double duration = (clock() - time_start) / (double) CLOCKS_PER_SEC;
            if(m_settings.planning_time_limit > 0 && duration > m_settings.planning_time_limit){
                m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Planning aborted: maximum planning time reached" );
                throw PlanningException("Planning aborted: maximum planning time reached" );
            }

            if(!result){
                m_overallDelay += m_settings.waiting_inc;
                delay += m_settings.waiting_inc;
                m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Could not find a plan for olv " + std::to_string(it->machine.id)
                                                                   + ". Increasing wait time for harvester by " + std::to_string(m_settings.waiting_inc) + " seconds, delay: " + std::to_string(delay) + "\n");

                if(!updateCurrentPlanCost(olvplan_map,
                                          m_overallDelay,
                                          m_settings,
                                          max_plan_cost)){
                    m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Planning aborted: current plan cost " + std::to_string( m_plan_cost ) + " is higher than the limit cost " + std::to_string( max_plan_cost ) );
                    return "Planning aborted: current plan cost " + std::to_string( m_plan_cost ) + " is higher than the limit cost " + std::to_string( max_plan_cost );
                }

                // add delay to harvester route and update the graph accordingly:
                for (int j = it->start_index; j < m_harvester_route.route_points.size(); ++j) {
                    m_harvester_route.route_points.at(j).time_stamp += m_settings.waiting_inc;
                    DirectedGraph::vertex_t v = m_graph.routepoint_vertex_map()[m_harvester_route.route_points.at(j)];
                    m_graph[v].route_point.time_stamp = m_harvester_route.route_points.at(j).time_stamp;
                }
            }

            result = olvplan_map.at(it->machine.id).planOverload(m_graph,
                                                                 m_harvester_route,
                                                                 *it,
                                                                 0);  // if a plan is found, m_graph is updated with the overruns


        }while (!result && delay < m_settings.max_waiting_time);

        if (result)
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Found overload plan for olv " + std::to_string( it->machine.id ) );
        else {
            m_logger.printOut(LogLevel::CRITIC, __FUNCTION__, "Could not find overload plan for olv " + std::to_string( it->machine.id )
                                                              + " with max waiting time " + std::to_string( m_settings.max_waiting_time ) );
            return "Could not find overload plan for olv " + std::to_string( it->machine.id )
                    + " with max waiting time" + std::to_string( m_settings.max_waiting_time );
        }

        if(!updateCurrentPlanCost(olvplan_map,
                                  m_overallDelay,
                                  m_settings,
                                  max_plan_cost)){
            m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Planning aborted: current plan cost " + std::to_string( m_plan_cost ) + " is higher than the limit cost " + std::to_string( max_plan_cost )  );
            return "Planning aborted: current plan cost " + std::to_string( m_plan_cost ) + " is higher than the limit cost " + std::to_string( max_plan_cost );
        }

    }

    for (std::map<int, OLVPlan>::iterator it = olvplan_map.begin(); it != olvplan_map.end(); ++it) {
        it->second.finishPlan(m_graph, m_settings.sendLastOlvToResourcePoint, m_harvester_route);
        if (it->second.foundPlan()) {
            m_planned_routes.push_back(it->second.getRoute(m_routeId_count++));
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Successfully finished the plan for olv " + std::to_string( it->first ) + " with cost " + std::to_string(it->second.getCost()) );

            if(!updateCurrentPlanCost(olvplan_map,
                                      m_overallDelay,
                                      m_settings,
                                      max_plan_cost)){
                m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Planning aborted: current plan cost " + std::to_string( m_plan_cost ) + " is higher than the limit cost " + std::to_string( max_plan_cost )  );
                return "Planning aborted: current plan cost " + std::to_string( m_plan_cost ) + " is higher than the limit cost " + std::to_string( max_plan_cost );
            }
        }
        else {
            m_logger.printOut(LogLevel::CRITIC, __FUNCTION__, "Could not finish the plan for olv " + std::to_string( it->first ));
            return "Could not finish the plan for olv " + std::to_string( it->first );
        }
    }

    if(!updateCurrentPlanCost(olvplan_map,
                              m_overallDelay,
                              m_settings,
                              max_plan_cost)){
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Planning aborted: current plan cost " + std::to_string( m_plan_cost ) + " is higher than the limit cost " + std::to_string( max_plan_cost )  );
        return "Planning aborted: current plan cost " + std::to_string( m_plan_cost ) + " is higher than the limit cost " + std::to_string( max_plan_cost );
    }

    updateHarvesterRouteRelations();

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Successfully finished the plan for olvs with a total cost " + std::to_string(m_plan_cost)
                                                    + " and a total delay of " + std::to_string(m_overallDelay) );

    return "";

}

std::vector<Machine> SimpleMultiOLVPlanner::reorderWorkingGroup(const DirectedGraph::Graph &graph,
                                                          const std::vector<Machine> &_overloadMachines,
                                                          const Route &harvesterRoute,
                                                          const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                          SimpleMultiOLVPlanner::OLVSwitchingStrategy switching_strategy,
                                                          const size_t &numFixedInitalOlvsInOrder,
                                                          double harvestedMassLimit,
                                                          Logger *_logger)
{
    std::vector<Machine> orderedWorkingGroup;
    std::vector<OLVPlan::OverloadInfo> overloadActivities;

    if(_overloadMachines.size() < 2)
        return _overloadMachines;

    std::vector<Machine> overloadMachines, fullMachines, rejectedMachines;
    double first_olv_bunker_level = 0.0;

    for(size_t i = 0 ; i < _overloadMachines.size() ; ++i){
        if(i < numFixedInitalOlvsInOrder){
            orderedWorkingGroup.push_back(_overloadMachines.at(i));
            continue;
        }
        auto it0 = machineCurrentStates.find(_overloadMachines.at(i).id);
        if (it0 != machineCurrentStates.end()){
            if(it0->second.bunkerMass >= _overloadMachines.at(i).bunker_mass * OLVPlan::OlvMaxCapacityMultiplier){
                fullMachines.push_back(_overloadMachines.at(i));
                continue;
            }
        }
        overloadMachines.push_back(_overloadMachines.at(i));
    }


    std::vector<Machine> overloadMachinesTmp = orderedWorkingGroup;
    overloadMachinesTmp.insert( overloadMachinesTmp.end(), overloadMachines.begin(), overloadMachines.end() );
    overloadMachinesTmp.insert( overloadMachinesTmp.end(), fullMachines.begin(), fullMachines.end() );

    if (switching_strategy == HEADLAND_ONLY) {
        overloadActivities = computeTracksEndsOverloadPoints(graph,
                                                             harvesterRoute,
                                                             overloadMachinesTmp,
                                                             machineCurrentStates,
                                                             harvestedMassLimit,
                                                             _logger);
    } else {
        overloadActivities = computeFieldOverloadPoints(graph,
                                                        harvesterRoute,
                                                        overloadMachinesTmp,
                                                        machineCurrentStates,
                                                        harvestedMassLimit,
                                                        _logger);
    }
    if(overloadActivities.empty())
        return _overloadMachines;

    size_t indCurrentActivity = 0;

    while (!overloadMachines.empty() || !fullMachines.empty()){

        Point startPoint = harvesterRoute.route_points.at( overloadActivities.at(indCurrentActivity).start_index ).point;

        if(!overloadMachines.empty()){
            double minDist = std::numeric_limits<double>::max();
            int minDistInd = -1;
            for(size_t i = 0 ; i < overloadMachines.size() ; i++){
                Point machinePos = startPoint;
                double bunkerMass = 0;
                auto it0 = machineCurrentStates.find(overloadMachines.at(i).id);
                if (it0 != machineCurrentStates.end()){
                    machinePos = it0->second.position;
                    bunkerMass = it0->second.bunkerMass;
                }
                double dist = arolib::geometry::calc_dist(startPoint, machinePos);
                dist += (1-bunkerMass/overloadMachines.at(i).bunker_mass) * 5;//if two machines are very close, choose the one that is more loaded
                if( minDist > dist ){
                    minDist = dist;
                    minDistInd = i;
                }
            }

            orderedWorkingGroup.push_back( overloadMachines.at(minDistInd) );
            overloadMachines.erase(overloadMachines.begin() + minDistInd);
        }
        else{
            double maxDist = std::numeric_limits<double>::lowest();
            int maxDistInd = -1;
            for(size_t i = 0 ; i < fullMachines.size() ; i++){
                Point machinePos(0,0);
                auto it0 = machineCurrentStates.find(fullMachines.at(i).id);
                if (it0 != machineCurrentStates.end())
                    machinePos = it0->second.position;
                double dist = arolib::geometry::calc_dist(startPoint, machinePos);
                if( maxDist < dist ){
                    maxDist = dist;
                    maxDistInd = i;
                }
            }

            orderedWorkingGroup.push_back( fullMachines.at(maxDistInd) );
            fullMachines.erase(fullMachines.begin() + maxDistInd);
        }

        overloadMachinesTmp = orderedWorkingGroup;
        overloadMachinesTmp.insert( overloadMachinesTmp.end(), overloadMachines.begin(), overloadMachines.end() );
        overloadMachinesTmp.insert( overloadMachinesTmp.end(), fullMachines.begin(), fullMachines.end() );

        if(orderedWorkingGroup.size() == 1){
            auto it0 = machineCurrentStates.find(orderedWorkingGroup.front().id);
            if (it0 != machineCurrentStates.end()) {
                first_olv_bunker_level = it0->second.bunkerMass;
            }
        }

        if (switching_strategy == HEADLAND_ONLY) {
            overloadActivities = computeTracksEndsOverloadPoints(graph,
                                                                 harvesterRoute,
                                                                 overloadMachinesTmp,
                                                                 machineCurrentStates,
                                                                 harvestedMassLimit,
                                                                 _logger);
        } else {
            overloadActivities = computeFieldOverloadPoints(graph,
                                                            harvesterRoute,
                                                            overloadMachinesTmp,
                                                            machineCurrentStates,
                                                            harvestedMassLimit,
                                                            _logger);
        }

        if( overloadActivities.at(indCurrentActivity).machine.id != orderedWorkingGroup.back().id
                || overloadActivities.at(indCurrentActivity).toResourcePointFirst ){
            rejectedMachines.push_back( orderedWorkingGroup.back() );
            orderedWorkingGroup.pop_back();
        }
        else
            indCurrentActivity++;

        if(indCurrentActivity >= overloadActivities.size())
            break;

    }

    if(orderedWorkingGroup.size() < overloadActivities.size())
        orderedWorkingGroup.insert( orderedWorkingGroup.end(), rejectedMachines.begin(), rejectedMachines.end() );

    return orderedWorkingGroup;

}

SimpleMultiOLVPlanner::OLVSwitchingStrategy SimpleMultiOLVPlanner::intToOLVSwitchingStrategy(int value)
{
    if(value == OLVSwitchingStrategy::INFIELD)
        return OLVSwitchingStrategy::INFIELD;
    else if(value == OLVSwitchingStrategy::HEADLAND_ONLY)
        return OLVSwitchingStrategy::HEADLAND_ONLY;

    throw std::invalid_argument( "The given value does not correspond to any MultiOLVPlanner::OLVSwitchingStrategy" );
}

SimpleMultiOLVPlanner::ReplanningStrategy SimpleMultiOLVPlanner::intToReplanningStrategy(int value)
{
    if(value == ReplanningStrategy::DELAY_HARVESTER)
        return ReplanningStrategy::DELAY_HARVESTER;
    else if(value == ReplanningStrategy::DELAY_HARVESTER_IN_HEADLAND)
        return ReplanningStrategy::DELAY_HARVESTER_IN_HEADLAND;
    else if(value == ReplanningStrategy::REPLAN_OLV_ACITIVITIES)
        return ReplanningStrategy::REPLAN_OLV_ACITIVITIES;

    throw std::invalid_argument( "The given value does not correspond to any MultiOLVPlanner::ReplanningStrategy" );
}

std::vector<OLVPlan::OverloadInfo> SimpleMultiOLVPlanner::computeFieldOverloadPoints(const DirectedGraph::Graph &graph,
                                                                               const Route &harvester_route,
                                                                               const std::vector<Machine> &_overloadMachines,
                                                                               const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                                               double harvestedMassLimit,
                                                                               Logger *_logger)
{
    Logger logger(LogLevel::CRITIC, __FUNCTION__);
    logger.setParent(_logger);
    std::vector<OLVPlan::OverloadInfo> ret;

    std::vector<Machine> overloadMachines = _overloadMachines;
    // we need the first routepoint that is on the track to get the initial harvested_mass
    // this is required in case of replanning
    int first_non_headland_index = 0;
    RoutePoint first_rp = harvester_route.route_points.at(first_non_headland_index);
    while (first_rp.type == RoutePoint::RoutePointType::HEADLAND || first_rp.time_stamp < -0.0001) {
        first_non_headland_index++;
        if(first_non_headland_index >= harvester_route.route_points.size())
            return ret;
        first_rp = harvester_route.route_points.at(first_non_headland_index);
    }

    RoutePoint second_rp = first_rp;
    if(first_non_headland_index+1 < harvester_route.route_points.size())
        second_rp = harvester_route.route_points.at(first_non_headland_index+1);
    double distDisregardInitialLimit = arolib::geometry::calc_dist(first_rp.point, second_rp.point) ;
    distDisregardInitialLimit = std::max( 25.0, std::min(50.0, distDisregardInitialLimit*5) );

    std::vector<double> olvCurrentBunkerLevels( overloadMachines.size() );
    for(size_t i = 0 ; i < overloadMachines.size() ; ++i){
        auto it_m = machineCurrentStates.find(overloadMachines.at(i).id);
        if(it_m != machineCurrentStates.end()){
            double dist = arolib::geometry::calc_dist(it_m->second.position, first_rp.point);
            bool disregardInitialLimit = (i == 0 && dist < distDisregardInitialLimit);
            if(disregardInitialLimit
                    || it_m->second.bunkerMass < overloadMachines.at(i).bunker_mass * InitialOlvMaxCapacityMultiplier)
                olvCurrentBunkerLevels.at(i) = it_m->second.bunkerMass;
            else
                olvCurrentBunkerLevels.at(i) = overloadMachines.at(i).bunker_mass * 2;//make them go tho a resource point first
        }
        else
            olvCurrentBunkerLevels.at(i) = 0;
    }

    int olv_index = 0;
    Machine current_olv = overloadMachines.at(olv_index);
    double current_harvester_mass = graph.at(first_rp).harvested_mass;
    double max_harvested_mass = current_harvester_mass + current_olv.bunker_mass * OLVPlan::OlvMaxCapacityMultiplier - olvCurrentBunkerLevels.at(olv_index);  /// harvester_mass when the next olv will be full
    int overload_start_index = first_non_headland_index;
    int overload_end_index = first_non_headland_index;

    bool toResourcePointFirst = false;
    bool olvCurrentBunkerLevel_prev;
    bool updateCurrentMass = true;
    bool updateLastIndex = true;

    for (int i = first_non_headland_index; i < harvester_route.route_points.size()-1; ++i) {
        RoutePoint next_rp = harvester_route.route_points.at(i+1);
        if (next_rp.type != RoutePoint::RoutePointType::HEADLAND
                && next_rp.type != RoutePoint::RoutePointType::TRACK_START) {

            DirectedGraph::vertex_t next_harvester_vt = graph.routepoint_vertex_map().at(next_rp);
            double next_harvester_mass = graph[next_harvester_vt].harvested_mass;

            if (next_harvester_mass > max_harvested_mass) {

                logger.printOut( LogLevel::DEBUG,
                                  __FUNCTION__,
                                  "*****IF****** Overload info :: start_index = " + std::to_string(overload_start_index)
                                  + "  -  end_index = " + std::to_string(overload_end_index)
                                  + "  -  track_id(in) = " + std::to_string( harvester_route.route_points.at(overload_start_index).track_id )
                                  + "  -  track_id(out) = " +std::to_string( harvester_route.route_points.at(i).track_id ) );

                if(overload_start_index < overload_end_index){
                    OLVPlan::OverloadInfo olvinfo;
                    olvinfo.start_index = overload_start_index;
                    olvinfo.end_index = overload_end_index;
                    olvinfo.machine = current_olv;
                    olvinfo.toResourcePointFirst = toResourcePointFirst;
                    toResourcePointFirst = false;
                    ret.push_back(olvinfo);

                    if(harvestedMassLimit > 0
                            && harvester_route.route_points.at(olvinfo.end_index).harvested_mass > harvestedMassLimit)
                        return ret;

                    if(updateLastIndex)
                        overload_start_index = i+1;
                    else
                        overload_start_index = i;

                    olvCurrentBunkerLevel_prev = olvCurrentBunkerLevels.at(olv_index);
                    olvCurrentBunkerLevels.at(olv_index) = 0;

                    // next olv machine:
                    olv_index = (olv_index + 1) % overloadMachines.size();
                    current_olv = overloadMachines.at(olv_index);
                }
                else if(!toResourcePointFirst){
                    toResourcePointFirst = true;
                    olvCurrentBunkerLevels.at(olv_index) = 0;
                    i--;
                    updateCurrentMass = false;
                }
                else{
                    toResourcePointFirst = false;
                    olvCurrentBunkerLevels.at(olv_index) = olvCurrentBunkerLevel_prev;

                    // next olv machine:
                    olv_index = (olv_index + 1) % overloadMachines.size();
                    current_olv = overloadMachines.at(olv_index);
                }


                /// next maximum
                max_harvested_mass = current_harvester_mass + current_olv.bunker_mass * OLVPlan::OlvMaxCapacityMultiplier - olvCurrentBunkerLevels.at(olv_index);

                //logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Current olv: " << std::to_string( current_olv.id ) + " bunker_volume: " + std::to_string( current_olv.bunker_volume ) );
                //logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Found next overload switching point: next_bunker_sum: " + std::to_string( max_harvested_mass ) + " prev mass: " + std::to_string( current_harvester_mass ) );

            }
            else if(updateLastIndex)
                overload_end_index = i+1;

            // update variables for next iteration
            if(updateCurrentMass)
                current_harvester_mass = next_harvester_mass;
            else
                updateCurrentMass = true;

            updateLastIndex = true;
        }
        else
            updateLastIndex = false;
    }


    /// add last overloading activity
    OLVPlan::OverloadInfo olvinfo;
    olvinfo.start_index = overload_start_index;
    olvinfo.end_index = harvester_route.route_points.size()-1;
    olvinfo.machine = current_olv;
    olvinfo.toResourcePointFirst = toResourcePointFirst;
    ret.push_back(olvinfo);

    return ret;

}


std::vector<OLVPlan::OverloadInfo> SimpleMultiOLVPlanner::computeTracksEndsOverloadPoints(const DirectedGraph::Graph &graph,
                                                                                    const Route &harvester_route,
                                                                                    std::vector<Machine> overloadMachines,
                                                                                    const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                                                    double harvestedMassLimit,
                                                                                    Logger *_logger)
{
    Logger logger(LogLevel::CRITIC, __FUNCTION__);
    logger.setParent(_logger);
    std::vector<OLVPlan::OverloadInfo> ret;
    int olv_index = 0;  /// index of the current overloading vehicle in overloadMachines
    int overload_start_index = 0;
    int old_index = 0;  // index of the previous track end

    int first_non_headland_index = 0;
    RoutePoint first_rp = harvester_route.route_points.at(first_non_headland_index);
    while (first_rp.type == RoutePoint::RoutePointType::HEADLAND || first_rp.time_stamp >= -0.0001) {
        first_non_headland_index++;
        if(first_non_headland_index >= harvester_route.route_points.size())
            return ret;
        first_rp = harvester_route.route_points.at(first_non_headland_index);
    }

    RoutePoint second_rp = first_rp;
    if(first_non_headland_index+1 < harvester_route.route_points.size())
        second_rp = harvester_route.route_points.at(first_non_headland_index+1);
    double distDisregardInitialLimit = arolib::geometry::calc_dist(first_rp.point, second_rp.point) ;
    distDisregardInitialLimit = std::max( 25.0, std::min(50.0, distDisregardInitialLimit*5) );

    std::vector<double> olvCurrentBunkerLevels( overloadMachines.size() );
    for(size_t i = 0 ; i < overloadMachines.size() ; ++i){
        auto it_m = machineCurrentStates.find(overloadMachines.at(i).id);
        if(it_m != machineCurrentStates.end()){
            double dist = arolib::geometry::calc_dist(it_m->second.position, first_rp.point);
            bool disregardInitialLimit = (i == 0 && dist < distDisregardInitialLimit);
            if(disregardInitialLimit
                    || it_m->second.bunkerMass < overloadMachines.at(i).bunker_mass * InitialOlvMaxCapacityMultiplier)
                olvCurrentBunkerLevels.at(i) = it_m->second.bunkerMass;
            else
                olvCurrentBunkerLevels.at(i) = overloadMachines.at(i).bunker_mass * 2;//make them go tho a resource point first
        }
        else
            olvCurrentBunkerLevels.at(i) = 0;
    }

    double max_harvested_mass = graph.at(first_rp).harvested_mass + overloadMachines.at(olv_index).bunker_mass * OLVPlan::OlvMaxCapacityMultiplier - olvCurrentBunkerLevels.at(olv_index);  /// harvester_mass when the next olv will be full
    while (old_index < harvester_route.route_points.size()-1) {
        int next_end_id = run_to_next_type(harvester_route, old_index+1, RoutePoint::RoutePointType::TRACK_END);
        RoutePoint next_end_rp = harvester_route.route_points.at(old_index);
        const DirectedGraph::vertex_t next_end_vt = graph.routepoint_vertex_map().at(next_end_rp);
        double next_end_harvester_mass = graph[next_end_vt].harvested_mass;
        if (next_end_harvester_mass > max_harvested_mass) {

//            logger.printOut( LogLevel::DEBUG,
//                              __FUNCTION__,
//                              "*****HP****** Overload info :: start_index = " + std::to_string(overload_start_index)
//                              + "  -  end_index = " + std::to_string(old_index)
//                              + "  -  track_id(in) = " + std::to_string( m_harvester_route.route_points.at(overload_start_index).track_id )
//                              + "  -  track_id(out) = " +std::to_string( m_harvester_route.route_points.at(old_index).track_id ) );

            OLVPlan::OverloadInfo olvinfo;
            olvinfo.start_index = overload_start_index;
            olvinfo.end_index = old_index;
            olvinfo.machine = overloadMachines.at(olv_index);
            ret.push_back(olvinfo);

            if(harvestedMassLimit > 0
                    && harvester_route.route_points.at(olvinfo.end_index).harvested_mass > harvestedMassLimit)
                return ret;

            /// next start:
            overload_start_index = run_to_next_type(harvester_route, old_index, RoutePoint::RoutePointType::TRACK_START);
            /// next maximum:
            RoutePoint start_rp = harvester_route.route_points.at(overload_start_index);

            olvCurrentBunkerLevels.at(olv_index) = 0;

            /// next olv machine:
            olv_index = (olv_index + 1) % overloadMachines.size();
            max_harvested_mass = graph.at(start_rp).harvested_mass + overloadMachines.at(olv_index).bunker_mass * OLVPlan::OlvMaxCapacityMultiplier - olvCurrentBunkerLevels.at(olv_index); /// TODO should not be completely filled (maybe 90%)
            logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Found next overload switching point: next_bunker_sum: " + std::to_string(max_harvested_mass));
        }
        old_index = next_end_id;
    }

    /// add last overloading activity
    OLVPlan::OverloadInfo olvinfo;
    olvinfo.start_index = overload_start_index;
    olvinfo.end_index = harvester_route.route_points.size()-1;
    olvinfo.machine = overloadMachines.at(olv_index);
    ret.push_back(olvinfo);
    return ret;

}


int SimpleMultiOLVPlanner::run_to_next_type(const Route &harvester_route, int start, RoutePoint::RoutePointType type) {
    for (int i = start; i < harvester_route.route_points.size(); ++i) {
        RoutePoint rp = harvester_route.route_points.at(i);
        if (rp.type == type) {
            return i;
        }
    }
    return harvester_route.route_points.size()-1;  //TODO or -1???
}

bool SimpleMultiOLVPlanner::updateCurrentPlanCost(const std::map<int, OLVPlan> &olvplan_map,
                                                  const double& delay,
                                                  const Astar::AStarSettings& aStarSettings,
                                                  const double &max_cost)
{
    auto aStarSettingsTmp = aStarSettings;
    aStarSettingsTmp.crossCostMult = 0;
    aStarSettingsTmp.soilOpt_soilValueCoef = 0;

    m_plan_cost = 0;
    for(auto &it : olvplan_map)
        m_plan_cost += it.second.getCost();
    m_plan_cost += calcEdgeCost(aStarSettingsTmp,
                                delay,
                                0,
                                0);
    if(std::isnan(m_plan_cost))
        m_logger.printOut(LogLevel::ERROR, "m_plan_cost is NAN!");
    return m_plan_cost <= max_cost;
}

void SimpleMultiOLVPlanner::updateHarvesterRouteRelations()
{
    for(auto &route : m_planned_routes){
        for(size_t i = 0 ; i < route.route_points.size() ; ++i){
            for(auto &mir : route.route_points.at(i).machineRelations){
                if( m_harvester_route.machine_id == mir.machine_id &&
                      m_harvester_route.route_id == mir.route_id &&
                      mir.routePointIndex < m_harvester_route.route_points.size() ){
                    RoutePoint::MachineRelationInfo mir_harv;
                    mir_harv.machine_id = route.machine_id;
                    mir_harv.route_id = route.route_id;
                    mir_harv.routePointIndex = i;
                    mir_harv.routePointType = route.route_points.at(i).type;
                    m_harvester_route.route_points.at( mir.routePointIndex ).machineRelations.push_back(mir_harv);
                }
            }
        }
    }
}


void SimpleMultiOLVPlanner::saveOverloadActivities(const std::vector<OLVPlan::OverloadInfo> &overloadActivities, const std::string &folder) {

    std::string filename = folder;
    if (folder.empty())
        filename = "/";
    if ( filename.at( filename.size()-1 ) != '/' )
        filename += "/";
    filename += "overloadActivities_HARV";
    filename += std::to_string( m_harvester_route.machine_id  );
    filename += ".txt";

    std::ofstream out(filename.c_str());
    for (int i=0; i < overloadActivities.size(); i++) {
        int j = overloadActivities.at(i).start_index;
        Point point_wgs;
        Point& point_utm = m_harvester_route.route_points.at(j).point;
        arolib::CoordTransformer::GetInstance().convert_to_geodetic(point_utm, point_wgs, point_utm.utm_zone, point_utm.utm_designator);
        out << std::setprecision(12);
        out << point_wgs.x << " "
            << point_wgs.y << " ";
        out << m_harvester_route.machine_id << " "
            << overloadActivities.at(i).machine.id << " "
            << m_harvester_route.route_points.at(j).time_stamp
            << std::endl;
        if (i == overloadActivities.size()-1){
            j = overloadActivities.at(i).end_index;
            arolib::CoordTransformer::GetInstance().convert_to_geodetic(point_utm, point_wgs, point_utm.utm_zone, point_utm.utm_designator);
            out << std::setprecision(12);
            out << point_wgs.x << " "
                << point_wgs.y << " ";
            out << m_harvester_route.machine_id << " "
                << overloadActivities.at(i).machine.id << " "
                << m_harvester_route.route_points.at(j).time_stamp
                << std::endl;
        }
    }


    out.close();

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "OverloadActivities saved to " + filename );
}


}
