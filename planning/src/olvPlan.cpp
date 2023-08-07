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
 
#include "arolib/planning/olvPlan.hpp"
#include "arolib/planning/path_search/graphhelper.hpp"
#include "arolib/planning/path_search/astar.hpp"

namespace arolib{

const double OLVPlan::OlvMaxCapacityMultiplier = 0.95;

bool OLVPlan::PlannerSettings::parseFromStringMap(OLVPlan::PlannerSettings &params, const std::map<std::string, std::string> &map, bool strict)
{
    OLVPlan::PlannerSettings tmp;

    if( !AStarSettings::parseFromStringMap(tmp, map, strict) )
        return false;

    std::map<std::string, bool*> bMap = { {"includeCostOfOverload" , &tmp.includeCostOfOverload} };

    if( !setValuesFromStringMap( map, bMap, strict) )
        return false;

    params = tmp;
    return true;
}

std::map<std::string, std::string> OLVPlan::PlannerSettings::parseToStringMap(const OLVPlan::PlannerSettings &params)
{
    std::map<std::string, std::string> ret, subMap;
    subMap = AStarSettings::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );
    ret["includeCostOfOverload"] = std::to_string( params.includeCostOfOverload );
    return ret;
}

OLVPlan::OLVPlan(const Machine &olv,
                 const std::vector<DirectedGraph::vertex_t> &resource_vertices,
                 const std::vector<DirectedGraph::vertex_t> &accessPoints_vertices,
                 const PlannerSettings & settings,
                 const MachineDynamicInfo &initialState,
                 std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                 const std::string &outputFolder,
                 LogLevel logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_settings(settings),
    m_olv(olv),
    m_resource_vertices(resource_vertices),
    m_accessPoints_vertices(accessPoints_vertices),
    m_edgeCostCalculator(edgeCostCalculator),
    m_olv_initial_state(initialState),
    m_outputFolder(outputFolder)
{
    if(std::isnan(m_state.plan_cost))
        logger().printOut(LogLevel::ERROR, "m_state.plan_cost is NAN!");

    if(!m_outputFolder.empty() && m_outputFolder.back() != '/')
        m_outputFolder += "/";

}

bool OLVPlan::planOverload(DirectedGraph::Graph &graph,
                           Route &harvester_route,
                           const std::shared_ptr<Machine> harv,
                           const OverloadInfo &overload_info,
                           const std::vector< std::pair<MachineId_t, double> >& overload_info_prev,
                           double maxWaitingTime,
                           bool is_overloading) {
    if(!m_edgeCostCalculator){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return false;
    }

    if(maxWaitingTime < -1e-6)
        maxWaitingTime = std::numeric_limits<double>::max();

    initState();

    if(graph.resourcepoint_vertex_map().empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "There are no resource vertices");
        return restoreInError();
    }

    if(harvester_route.route_points.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Harvester route is empty");
        return restoreInError();
    }

    Point initialPoint;

    if(m_resource_vertices.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "There are no resource vertices");
        return restoreInError();
    }


    DirectedGraph::Graph graph_minCost;
    PlanState state_0 = m_state, best_state;
    Route best_harvester_route;
    int indBestPlan = -1;

    for(size_t indResVt = 0 ; indResVt < m_resource_vertices.size() ; ++indResVt){

        DirectedGraph::Graph graph_tmp = graph;//in case the overload plan fails
        auto harvester_route_tmp = harvester_route;//in case the overload plan fails
        m_state = state_0;

        std::chrono::steady_clock::time_point time_start;

        bool overloading = false;

        if( m_state.lastLocation == LOC_HARVESTER_OUT ){//send to a resource point (continue planning)
            AstarPlan plan;
            double nextSwitchingPointTimestamp = harvester_route_tmp.route_points.at(overload_info.start_index).time_stamp;
            time_start = std::chrono::steady_clock::now();

            if(m_state.leaveOverloadingFromClosestVt){
                if(!updateCurrentVertexWithClosest(graph_tmp, harvester_route_tmp)){
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error obtaining current olv vertex");
                    continue;
                }
                m_state.leaveOverloadingFromClosestVt = false;
            }

            //add vertex corresponding to the overload start point to the vertices to be excluded
            auto it_rp = graph.routepoint_vertex_map().find( harvester_route_tmp.route_points.at( overload_info.start_index) );
            if( it_rp != graph.routepoint_vertex_map().end() )
                m_state.excludeVts.insert( it_rp->second );

            if( overload_info.start_index > 0 ){//add vertex corresponding to the previous point to the overload start to the vertices to be excluded
                it_rp = graph.routepoint_vertex_map().find( harvester_route_tmp.route_points.at( overload_info.start_index - 1) );
                if( it_rp != graph.routepoint_vertex_map().end() )
                    m_state.excludeVts.insert( it_rp->second );
            }

            if( !planPathToResource(graph_tmp,
                                    &m_resource_vertices.at(indResVt),
                                    plan,
                                    nextSwitchingPointTimestamp,//the A* max_visit_time is the timestamp of the overloading-start of this overload activity
                                    harvester_route_tmp,
                                    nextSwitchingPointTimestamp + maxWaitingTime)//the machine must not get to the resource point after this time (it would be to late to go travel to the next OL-start on time). It practice, should be less than this value (the machine cannot teleport to the next OL-start), but this value can be used as a best-case scenario limit.
                    ){
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error calling planPathToResource");
                continue;
            }
            m_state.planningDuration_toRP = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
            m_state.planCost_toRP = plan.plan_cost_total;
            if(std::isnan(m_state.plan_cost))
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "plan_cost is NAN (planPathToResource)!");


            if(indBestPlan >= 0 && best_state.plan_cost < m_state.plan_cost)
                continue;
        }

        if(m_state.lastLocation == LOC_UNKNOWN){//start of planning
            m_state.olv_bunker_mass = m_olv_initial_state.bunkerMass;
            m_state.olv_time = m_olv_initial_state.timestamp;

            auto it_m = graph_tmp.initialpoint_vertex_map().find(m_olv.id);
            if(it_m != graph_tmp.initialpoint_vertex_map().end()){//the current location of the OLV is in its initial/current-position vertex
                m_state.lastLocation = LOC_INIT;
                m_olv_initial_vt = it_m->second;
                m_state.olv_last_vt = it_m->second;
                m_start_at_resource = false;
                initialPoint = graph_tmp[m_olv_initial_vt].route_point.point();
            }
            else{//the location is really unknown --> locate the OLV at a suitable location
                if( overload_info.start_index != 0 //the OLV is not assigned to the first overloading window (virgin field)
                        || overload_info.toResourcePointFirst //this OLV has to be sent to a resource point befor going to the overload-start
                        || (overload_info.start_index == 0 && harvester_route_tmp.route_points.front().time_stamp > 1e-3) ){//the OLV is assigned to the first overloading window but the harvester does not start directly at the overload-start (the first route-point of the harvester route has a timestamp > 0)

                    //locate the OLV directly in the most convenient resource/silo or field-entry vertex

                    bool isEntryPoint;
                    if( !getBestInitialVertex(graph_tmp,
                                              harvester_route_tmp,
                                              m_state.olv_last_vt,
                                              isEntryPoint,
                                              !overload_info.toResourcePointFirst && m_state.olv_bunker_mass < 1e-5) ){
                        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error obtaining initial resource point");
                        continue;
                    }
                    m_start_at_resource = !isEntryPoint;

                    if(isEntryPoint){//start in the selected entry-point vertex
                        m_state.lastLocation = LOC_INIT;
                    }
                    else{//start in the selected resource/silo vertex
                        if(m_state.olv_bunker_mass > 1e-5)//it still needs to unload
                            m_state.lastLocation = LOC_RESOURCE_IN;
                        else
                            m_state.lastLocation = LOC_RESOURCE_OUT;
                    }
                }
                else{//the OLV is assigned to the first overloading window (virgin field) --> locate the OLV directly in the overload-start position
                    m_state.olv_last_vt = graph_tmp.routepoint_vertex_map().at( harvester_route_tmp.route_points.front() );
                    m_state.lastLocation = LOC_HARVESTER_IN;
                }
            }
        }

        if( m_state.lastLocation == LOC_INIT ){//check if the machine is currently overloading, if it has to go first to a resource point, or none of the previous
            bool readyToOverload = (m_state.olv_bunker_mass < m_olv.bunker_mass * OlvMaxCapacityMultiplier /*&& is_overloading*/)
                    && !overload_info.toResourcePointFirst;//check if the machine is ready to start/continue overloading, or if it needs to go first to unload in a silo

            if (readyToOverload) { // for replanning, the first olv, that starts at harvester should not unload first, but directly continue at harvester

                RoutePoint overloadPoint = harvester_route_tmp.route_points.at(overload_info.start_index);
                RoutePoint overloadPoint2;

                int harvester_next_rp_index = geometry::getNextNonRepeatedPointIndex(harvester_route_tmp.route_points, overload_info.start_index, -1, -1);
                int harvester_prev_rp_index = geometry::getPrevNonRepeatedPointIndex(harvester_route_tmp.route_points, overload_info.start_index, -1, -1);

                if(harvester_prev_rp_index >= 0 || harvester_next_rp_index >= 0){
                    if(harvester_prev_rp_index >= 0)
                        overloadPoint2 = harvester_route_tmp.route_points.at(harvester_prev_rp_index);
                    else
                        overloadPoint2 = harvester_route_tmp.route_points.at(harvester_next_rp_index);

                    double distToOverloadStart = arolib::geometry::calc_dist(initialPoint, overloadPoint);

                    DownloadSide side = DS_SWITCHING_POINT;

                    if(harvester_next_rp_index >= 0){
                        const auto& nextOverloadingPoint = harvester_route_tmp.route_points.at( harvester_next_rp_index );
                        float angle = geometry::get_angle(initialPoint, overloadPoint, nextOverloadingPoint, true);

                        if( std::fabs( angle ) > 150
                                || graph.outermostTrackIds_HL().find(nextOverloadingPoint.track_id) != graph.outermostTrackIds_HL().end() ){
                            double distComp = std::max( m_olv.workingRadius(), m_olv.length );
                            if(harv)
                                distComp = std::max( {distComp, harv->workingRadius(), harv->length });
                            distComp *= 1.2;
                            if(distToOverloadStart < distComp)
                                side = DS_BEHIND;
                        }
                        else if( std::fabs( angle ) <= 150 && std::fabs( angle ) >= 30 ){
                            double distComp = std::max( m_olv.width, m_olv.working_width );
                            if(harv)
                                distComp = std::max( {distComp, harv->working_width, harv->width });
                            distComp *= 1.2;
                            if(distToOverloadStart < distComp)
                                side = angle > 0 ? DS_RIGHT : DS_LEFT;
                        }

                    }



                    if( distToOverloadStart <= arolib::geometry::calc_dist(overloadPoint, overloadPoint2)
                            || side != DS_SWITCHING_POINT){// the machine is very close to the overload start --> go directly to follow the harvester

                        overloading = true;

                        //connect the first route point with the first overload-start
                        RoutePoint rp = graph_tmp[m_olv_initial_vt].route_point;
                        rp.bunker_mass = m_state.olv_bunker_mass;
                        rp.time_stamp = m_state.olv_time;
                        rp.type = RoutePoint::INITIAL_POSITION;
                        m_state.route.route_points.clear();
                        m_state.route.route_points.push_back(rp);


                        m_state.lastLocation = LOC_HARVESTER_IN;

                        m_state.adjVertexInfo.first = m_olv_initial_vt;
                        m_state.adjVertexInfo.second = side;

                    }
                }

            }
            else{//needs to go to the silo first
                AstarPlan plan;
                double nextSwitchingPointTimestamp = harvester_route_tmp.route_points.at(overload_info.start_index).time_stamp;
                time_start = std::chrono::steady_clock::now();
                if( !planPathToResource(graph_tmp,
                                        &m_resource_vertices.at(indResVt),
                                        plan,
                                        nextSwitchingPointTimestamp,//the A* max_visit_time is the timestamp of the overloading-start of this overload activity
                                        harvester_route_tmp,
                                        nextSwitchingPointTimestamp + maxWaitingTime)//the machine must not get to the resource point after this time (it would be to late to go travel to the next OL-start on time). It practice, should be less than this value (the machine cannot teleport to the next OL-start), but this value can be used as a best-case scenario limit.
                        ){
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error calling planPathToResource (init)");
                    continue;
                }
                m_state.planningDuration_toRP_init = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
                m_state.planCost_toRP_init = plan.plan_cost_total;

                if(std::isnan(m_state.plan_cost))
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, "plan_cost is NAN (planPathToResource - init)!");

                if(indBestPlan >= 0 && best_state.plan_cost < m_state.plan_cost)
                    continue;
            }
        }

        if( m_state.lastLocation == LOC_INIT
                || m_state.lastLocation == LOC_RESOURCE_IN
                || m_state.lastLocation == LOC_RESOURCE_OUT ){

            if( m_state.lastLocation == LOC_RESOURCE_IN ){//unload at the resource point
                double ul_time = 60;//initial default
                DirectedGraph::vertex_property last_vt_property = graph_tmp[m_state.olv_last_vt];

                if(m_olv.unloading_speed_mass > 1e-9)
                    ul_time = m_state.olv_bunker_mass / m_olv.unloading_speed_mass;
                else{
                    OutFieldInfo::UnloadingCosts ulc;
                    if( OutFieldInfo::getUnloadingCosts(last_vt_property.unloadingCosts, m_olv.id, ulc) ){
                        if(ulc.time_per_kg > 1e-9)
                            ul_time = m_state.olv_bunker_mass * ulc.time_per_kg;
                        else if(ulc.time > 1e-9)
                            ul_time = ulc.time;
                    }
                }
                ul_time = std::max(0.0, ul_time);

                double cost = m_edgeCostCalculator->calcCost(m_olv,
                                                             last_vt_property.route_point,
                                                             last_vt_property.route_point,
                                                             ul_time,
                                                             0,
                                                             0.5 * m_state.olv_bunker_mass,
                                                             {});//compute simple costs related to the unloading time

                m_state.plan_cost += cost;
                m_state.planCost_unloading = cost;
                m_state.olv_time += ul_time;
                m_state.olv_bunker_mass = 0;
                m_state.lastLocation = LOC_RESOURCE_OUT;

                if(std::isnan(m_state.plan_cost))
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, "plan_cost is NAN (unloading in RP)!");
            }

            //send to the overloading-start
            AstarPlan plan;
            std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
            if( !planPathToHarvester(graph_tmp,
                                     plan,
                                     harvester_route_tmp,
                                     overload_info.start_index,
                                     harv,
                                     maxWaitingTime,
                                     overload_info,
                                     overload_info_prev) ){
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error calling planPathToHarvester");
                continue;
            }
            m_state.planningDuration_toHarv = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
            m_state.planCost_toHarv = plan.plan_cost_total;

            if(std::isnan(m_state.plan_cost))
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "plan_cost is NAN (planPathToHarvester)!");
        }

        if(indBestPlan < 0 || m_state.plan_cost < best_state.plan_cost){
            indBestPlan = indResVt;
            best_state = m_state;
            graph_minCost = graph_tmp;
            best_harvester_route = harvester_route_tmp;
        }
    }

    if(indBestPlan < 0){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "No valid plan was found to any of the resource vertices");
        return restoreInError();
    }

    m_state = best_state;

    if( m_state.lastLocation == LOC_HARVESTER_IN ){// overloading --> follow harvester

        std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
        if(m_state.adjVertexInfo.second == DS_BEHIND
                || m_state.adjVertexInfo.second == DS_SWITCHING_POINT_BEHIND
                || m_state.adjVertexInfo.second == DS_SWITCHING_POINT){

            if(!planOverloadingPath_behind(graph_minCost, best_harvester_route, overload_info, harv)){
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error calling planOverloadingPath (behind)");
                return restoreInError();
            }
        }
        else{
            if(!planOverloadingPath_alongside(graph_minCost, best_harvester_route, overload_info, harv)){
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error calling planOverloadingPath (alongside)");
                return restoreInError();
            }
        }
        m_state.planningDuration_overloading = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();

        m_state.lastLocation = LOC_HARVESTER_OUT;
        m_prev_overloading_end_index = overload_info.end_index;
    }

    std::swap(graph, graph_minCost);
    std::swap(harvester_route, best_harvester_route);
    m_found_plan = true;
    if(!m_state.route.route_points.empty())
        m_state.lastRoutePoint = m_state.route.route_points.back();
    m_statesMemory.emplace_back(m_state);


    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Planning duration = " + std::to_string(m_state.planningDuration_toRP_init) + "[INIT->RP]"
                      + " + " + std::to_string(m_state.planningDuration_toRP) + "[Harv->RP]"
                      + " + " + std::to_string(m_state.planningDuration_toHarv) + "[RP->Harv]"
                      + " + " + std::to_string(m_state.planningDuration_overloading) + "[OL]"
                      + " = " + std::to_string(m_state.planningDuration_toRP_init + m_state.planningDuration_toHarv + m_state.planningDuration_toRP + m_state.planningDuration_overloading) + " seconds" );

    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Planning cost = " + std::to_string(m_state.planCost_toRP_init) + "[INIT->RP]"
                      + " + " + std::to_string(m_state.planCost_toRP) + "[Harv->RP]"
                      + " + " + std::to_string(m_state.planCost_unloading) + "[UL]"
                      + " + " + std::to_string(m_state.planCost_toHarv) + "[RP->Harv]"
                      + " + " + std::to_string(m_state.planCost_overloading) + "[OL]"
                      + " = " + std::to_string(m_state.planCost_toRP_init + m_state.planCost_toRP + m_state.planCost_unloading + m_state.planCost_toHarv + m_state.planCost_overloading) );


    return true;
}

bool OLVPlan::finishPlan(DirectedGraph::Graph &graph,
                         bool sendToResourcePointIfNotFull,
                         const Route &harvester_route,
                         const std::vector< std::pair<MachineId_t, double> >& overload_info_prev) {
    if(!m_edgeCostCalculator){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return false;
    }

    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Finishing plan...");

    initState();

    if(m_state.lastLocation == LOC_UNKNOWN){//no activity was scheduled for this OLV, but it might still need to leave the field
        m_state.olv_bunker_mass = m_olv_initial_state.bunkerMass;
        m_state.olv_time = m_olv_initial_state.timestamp;

        //get the curent position of the machine and locate the machine there
        auto it_m = graph.initialpoint_vertex_map().find(m_olv.id);
        if(it_m == graph.initialpoint_vertex_map().end()){//location really unknown --> do nothing
            logger().printOut(LogLevel::WARNING, __FUNCTION__, "The location on the current OLVPlan state is UNKNOWN and not initial vertex was found for the olv");
            return true;
        }
        m_state.lastLocation = LOC_INIT;
        m_olv_initial_vt = it_m->second;
        m_state.olv_last_vt = it_m->second;
        m_start_at_resource = false;

        //ckeck if the machine is inside the field
        bool insideField = false;
        for( auto out_edges = boost::out_edges(m_state.olv_last_vt, graph) ;
             out_edges.first != out_edges.second; out_edges.first++){

            auto successor = target(*out_edges.first, graph);
            DirectedGraph::vertex_property successor_prop = graph[successor];

            if(successor_prop.route_point.type != RoutePoint::FIELD_ENTRY
                    && successor_prop.route_point.type != RoutePoint::FIELD_EXIT
                    && successor_prop.route_point.type != RoutePoint::RESOURCE_POINT){
                insideField = true;
                break;
            }
        }

        if (!insideField)//outside the field --> do nothing
            return true;
    }


    if(m_state.lastLocation == LOC_RESOURCE_IN || m_state.lastLocation == LOC_RESOURCE_OUT)//machine is at resource/silo point --> do nothing
        return true;

    DirectedGraph::Graph graph_tmp = graph;//in case the plan fails

    //check if the machine has to be sent to a resource point
    bool toResourcePoint = sendToResourcePointIfNotFull || m_accessPoints_vertices.empty();
    if(!toResourcePoint){
        if(m_state.olv_bunker_mass > 0.85 * m_olv.bunker_mass )
            toResourcePoint = true;
    }

    AstarPlan plan;

    std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

    if(m_state.leaveOverloadingFromClosestVt){
        if(!updateCurrentVertexWithClosest(graph_tmp, harvester_route)){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error obtaining current olv vertex");
            return restoreInError();
        }
        m_state.leaveOverloadingFromClosestVt = false;
    }

    double lastTimestamp = -1;//timestamp of the last overload end
    for(auto& ts_pair : overload_info_prev)
        lastTimestamp = std::max(lastTimestamp, ts_pair.second);

    if(toResourcePoint){//send to resource/silo vertex
        if(graph.resourcepoint_vertex_map().empty()){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "There are no resource vertices");
            return restoreInError();
        }

        if( !planPathToResource(graph_tmp,
                                nullptr,
                                plan,
                                lastTimestamp,//there is no next switching point
                                harvester_route,
                                -1 ) //no limit, just get there
           ){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error calling planPathToResource");
            return restoreInError();
        }
        m_state.planningDuration_toRP = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
        if(std::isnan(m_state.plan_cost))
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "plan_cost is NAN (planPathToResource)!");
    }
    else{//try to send to a field-exit vertex
        if(graph.accesspoint_vertex_map().empty()){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "There are no access point vertices");
            return restoreInError();
        }

        if( !planPathToExitPoint(graph_tmp,
                                 plan,
                                 lastTimestamp,
                                 harvester_route) ){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error calling planPathToExitPoint");
            return restoreInError();
        }
        m_state.planningDuration_toRP = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
        if(std::isnan(m_state.plan_cost))
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "plan_cost is NAN (planPathToExitPoint)!");
    }

    graph = graph_tmp;
    m_found_plan = true;
    m_statesMemory.emplace_back(m_state);

    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Planning duration = " +  std::to_string(m_state.planningDuration_toRP) + "[Harv->RP/EP] seconds" );

    return true;

}

void OLVPlan::addDelayToHarvRoute(DirectedGraph::Graph &graph, Route &harvester_route, size_t indStart, double delay)
{
    if(indStart >= harvester_route.route_points.size())
        return;

    //update graph using original base route (before changingthe route)
    updateTimestampsFromBaseRoute(graph, harvester_route, delay, indStart, -1, harvester_route.route_points.at(indStart).time_stamp);

    double timestamp_old = harvester_route.route_points.at(indStart).time_stamp;//used to update the visit periods of the vertices (they must be changed from this timestamp on)

    // add delay to harvester route (starting from the given route-point index) and update the vertex_properties accordingly
    for (int j = indStart; j < harvester_route.route_points.size(); ++j){
        //add delay to the route point of the harvester route
        auto& rp = harvester_route.route_points.at(j);
        if(rp.time_stamp > -1e-6)
            rp.time_stamp += delay;
    }

    //update all visit windows for the machine from given timestamp
    for(DirectedGraph::vertex_iter vp = vertices(graph); vp.first != vp.second; vp.first++){
        DirectedGraph::vertex_property& v_prop = graph[*vp.first];
        updateVisitPeriods(v_prop.visitPeriods,
                           harvester_route.machine_id,
                           timestamp_old - 1e-3,
                           delay,
                           [timestamp_old](const DirectedGraph::VisitPeriod & vp)->bool{
                               return std::fabs( vp.timestamp - timestamp_old) < 1e-3;
                           });
    }

}

Route OLVPlan::getRoute(int routeId, int stateIndex) const
{
    Route route;
    route.machine_id = m_olv.id;
    route.route_id = routeId;

    if(m_statesMemory.empty())
        return route;

    if( stateIndex < 0 || stateIndex >= m_statesMemory.size() )
        stateIndex = m_statesMemory.size() - 1;

    for(size_t i = 0 ; i <= stateIndex ; ++i){
        const auto& route_points = m_statesMemory.at(i).route.route_points;
        route.route_points.insert( route.route_points.end(), route_points.begin(), route_points.end() );
    }

    return route;
}

double OLVPlan::getRouteLastTimestamp(int stateIndex) const
{
    if(m_statesMemory.empty())
        return -1;

    if( stateIndex < 0 || stateIndex >= m_statesMemory.size() )
        stateIndex = m_statesMemory.size() - 1;

    if( m_statesMemory.at(stateIndex).route.route_points.empty() )
        return -1;
    return m_statesMemory.at(stateIndex).route.route_points.back().time_stamp;
}

double OLVPlan::getCost(int stateIndex) const {

    if(m_statesMemory.empty())
        return 0;
    if( stateIndex < 0 || stateIndex >= m_statesMemory.size() )
        stateIndex = m_statesMemory.size() - 1;
    return m_statesMemory.at(stateIndex).plan_cost;
}

double OLVPlan::getDelay(int stateIndex) const
{
    if(m_statesMemory.empty())
        return 0;
    if( stateIndex < 0 || stateIndex >= m_statesMemory.size() )
        stateIndex = m_statesMemory.size() - 1;
    return m_statesMemory.at(stateIndex).delay;

}

size_t OLVPlan::getNumCrossings(size_t &numCrossings, size_t &numCrossings_HL, int stateIndex) const
{
    numCrossings = 0;
    numCrossings_HL = 0;
    if(m_statesMemory.empty())
        return 0;
    if( stateIndex < 0 || stateIndex >= m_statesMemory.size() )
        stateIndex = m_statesMemory.size() - 1;

    numCrossings = m_statesMemory.at(stateIndex).numCrossings;
    numCrossings_HL = m_statesMemory.at(stateIndex).numCrossings_HL;
    return numCrossings + numCrossings_HL;
}

bool OLVPlan::restoreInError()
{
    m_found_plan = false;
    return false;
}

void OLVPlan::initState()
{
    if(m_statesMemory.empty())//add the initial state (default)
        m_statesMemory.emplace_back(PlanState());

    m_state = m_statesMemory.back();//retrieve the data from the last planning state

    //save the last x route points from the route of the previous state
    std::swap(m_state.route_points_prev, m_state.route.route_points);
    if(m_state.route_points_prev.size() > 10)
        pop_front(m_state.route_points_prev, m_state.route_points_prev.size()-10);

    //initialize the data corresponding (only) to the current overload activity
    m_state.route.machine_id = m_olv.id;
    m_state.route.route_id = m_state.route_id++;//id of the current overload activity route SEGMENT
    m_state.route.route_points.clear();//route points of the current overload activity route SEGMENT
    m_state.delay = 0;//delay corresponding to the planning of the current overload activity route SEGMENT
    m_state.adjVertexInfo = std::make_pair(-1, DownloadSide::DS_SWITCHING_POINT);
    m_state.excludeVts.clear();
}

void OLVPlan::updateState(const AstarPlan &plan, DirectedGraph::Graph &graph, OLVPlan::LocationType newLocation)
{
    m_state.route.route_points.insert(m_state.route.route_points.end(), plan.route_points_.begin(), plan.route_points_.end());

    plan.insertIntoGraph(graph);

    if(!plan.route_points_.empty()){
        m_state.plan_cost += plan.plan_cost_total;
        m_state.olv_time = plan.route_points_.back().time_stamp;
        m_state.numCrossings += plan.numCrossings;
        m_state.numCrossings_HL += plan.numCrossings_HL;
    }

    m_state.lastLocation = newLocation;
}

bool OLVPlan::getBestInitialVertex(const DirectedGraph::Graph &graph,
                                   const Route &harvester_route,
                                   DirectedGraph::vertex_t &vt,
                                   bool &isEntryPoint,
                                   bool includeEntryPoints)
{
    double minCost = std::numeric_limits<double>::max();
    bool ok = false;
    vt = -1;
    isEntryPoint = false;

    //get the first working route-point
    auto rp0_it = harvester_route.route_points.end();
    for(auto it = harvester_route.route_points.begin() ; it != harvester_route.route_points.end() ; ++it){
        if(it->isOfTypeWorking_InTrack() || it->type == RoutePoint::TRACK_START){//@TODO might be deprecated. Maybe we need to check as well if the timestamp of the corresponding vertex_prop is >= 0.
            rp0_it = it;
            break;
        }
    }

    if(includeEntryPoints){
        //search for the vertex that is closest to the first working route-point in the field-entry vertices' list
        for(auto &ap_vt : m_accessPoints_vertices){

            //check if it is a field-entry-point vertex
            bool validVertex = false;
            auto out_edges = boost::out_edges(ap_vt, graph);
            for(;out_edges.first != out_edges.second; out_edges.first++){
                DirectedGraph::vertex_t successor = target(*out_edges.first, graph);
                DirectedGraph::vertex_property successor_prop = graph[successor];
                if(successor_prop.route_point.type != RoutePoint::INITIAL_POSITION
                        && successor_prop.route_point.type != RoutePoint::RESOURCE_POINT ){//there is an edge connecting to the field --> field entry point
                    if (rp0_it == harvester_route.route_points.end()){//strange case, just select any entry vertex
                        vt = ap_vt;
                        isEntryPoint = true;
                        return true;
                    }
                    validVertex = true;
                    break;
                }
            }
            if(!validVertex)
                continue;

            //check if this vertex is the closest to the first working route-point (update if so)
            DirectedGraph::vertex_property vt_prop = graph[ap_vt];
            double dist = arolib::geometry::calc_dist( vt_prop.route_point, rp0_it->point() );
            if( minCost > dist ){
                vt = ap_vt;
                minCost = dist;
                isEntryPoint = true;
                ok = true;
            }
        }
    }

    //search for the vertex that is closest to the first working route-point in the resource-point vertices' list
    for(auto &rp_vt : m_resource_vertices){

        //check if it is a resource vertex connected to the field (should be, but just in case)
        bool validVertex = false;
        auto out_edges = boost::out_edges(rp_vt, graph);
        for(;out_edges.first != out_edges.second; out_edges.first++){
            DirectedGraph::vertex_t successor = target(*out_edges.first, graph);
            DirectedGraph::vertex_property successor_prop = graph[successor];
            if(successor_prop.route_point.type != RoutePoint::INITIAL_POSITION
                    && successor_prop.route_point.type != RoutePoint::RESOURCE_POINT ){//just in case the vertex is not connected to the field
                if (rp0_it == harvester_route.route_points.end()){//strange case, just select any resource vertex
                    vt = rp_vt;
                    isEntryPoint = false;
                    return true;
                }
                validVertex = true;
                break;
            }
        }
        if(!validVertex)
            continue;

        //check if this vertex is the closest to the first working route-point (update if so)
        DirectedGraph::vertex_property vt_prop = graph[rp_vt];
        double dist = arolib::geometry::calc_dist( vt_prop.route_point, rp0_it->point() );
        if( minCost > dist ){
            vt = rp_vt;
            minCost = dist;
            isEntryPoint = false;
            ok = true;
        }
    }

    return ok;
}

bool OLVPlan::planPathToAdjacentPoint(DirectedGraph::Graph &graph,
                                      AstarPlan &plan,
                                      const DirectedGraph::vertex_t &switching_vt,
                                      double switching_time,
                                      double machine_speed,
                                      const Route &harvester_route,
                                      int harvester_rp_index,
                                      const Machine &harv,
                                      double maxDelay,
                                      std::pair<DirectedGraph::vertex_t, DownloadSide>& adjVertexInfo,
                                      double &olvWaitingTime,
                                      const OverloadInfo &overload_info,
                                      const std::vector< std::pair<MachineId_t, double> >& overload_info_prev)
{

    //for now (only do the sides part if there is no change from headland to infield or viceversa)

    bool doSides = true;

    std::vector<RoutePoint> segment( harvester_route.route_points.begin()+overload_info.start_index, harvester_route.route_points.begin()+overload_info.end_index+1 );
    if(harvester_route.route_points.size() < 2)
        doSides = false;
    else{
        int track_ref = -1;
        for(size_t i = 0 ; i < segment.size() ; ++i){
            if(segment.at(i).track_id >= 0){
                track_ref = segment.at(i).track_id;
                break;
            }
        }
        if(track_ref >= 0 ){
            for(size_t i = 0 ; i < segment.size() ; ++i){
                auto trackId = r_at(segment, i).track_id;
                if(segment.at(i).track_id >= 0){
                    doSides = Track::isInfieldTrack(trackId) == Track::isInfieldTrack(track_ref);
                    break;
                }
            }

            //do not do sides if working in partial headland
            if(Track::isHeadlandTrack(track_ref) && graph.hasPartialHeadlands())
                doSides = false;
        }
    }

    if(!doSides){
        adjVertexInfo = std::make_pair(switching_vt, DownloadSide::DS_SWITCHING_POINT);
        return planPathToAdjacentPoint_2(graph,
                                         plan,
                                         switching_vt,
                                         switching_time,
                                         machine_speed,
                                         harvester_route,
                                         harvester_rp_index,
                                         harv.workingRadius() * 1.2,
                                         maxDelay,
                                         olvWaitingTime,
                                         overload_info,
                                         overload_info_prev);
    }
    //for now

    olvWaitingTime = 0;

    std::vector< std::pair<DirectedGraph::vertex_t, DownloadSide>> adjacents  = getAdjacentVertices(graph,
                                                                                                    harvester_route,
                                                                                                    harvester_rp_index,
                                                                                                    harv);//get the (valid) closest vertices where a connecting edge exists

    if ( adjacents.empty() ){
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Unable to find adjacent vertices. (Switching point track id = " + std::to_string( graph[switching_vt].route_point.track_id ) + ")");
        return false;
    }


    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "adjacents.size() = " + std::to_string( adjacents.size() ) +
                                                     " (Switching point track id = " + std::to_string( graph[switching_vt].route_point.track_id ) + ")");

    DirectedGraph::vertex_property &switching_vt_prop = graph[switching_vt];
    bool planOK = false;
    double minCost = std::numeric_limits<double>::max();
    size_t best_adjacent_vt_ind;

    Astar::PlanParameters astarParams;
    astarParams.start_vt = m_state.olv_last_vt;
    astarParams.start_time = m_state.olv_time;
    astarParams.machine = m_olv;
    astarParams.machine_speed = machine_speed;
    astarParams.initial_bunker_mass = m_state.olv_bunker_mass;
    astarParams.includeWaitInCost = false;//false, for the cases where going behind the harvester with delays (better) has the same time-cost as going around covering more distance (worse). The waitTime will be added to the cost afterwards.


    std::set<MachineId_t> restrictedMachineIds;
    std::map<MachineId_t, double> restrictedMachineIds_futureVisits;
    if(graph.outermostTrackIds_HL().find(harvester_route.route_points.at(harvester_rp_index).track_id) != graph.outermostTrackIds_HL().end())
        restrictedMachineIds.insert(Machine::AllMachineIds);
    else
        restrictedMachineIds.insert(harvester_route.machine_id);

    for(auto &oi : overload_info_prev)
        restrictedMachineIds_futureVisits[oi.first] = oi.second;

    astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_SuccessorTimestamp>(std::max(0.0, switching_time), //do not drive over any of the vertices that are harvested after the swtiching-vertex
                                                                                                           AstarSuccessorChecker_SuccessorTimestamp::INVALID_IF_LOWER_THAN_SUCC_TIMESPAMP,
                                                                                                           restrictedMachineIds) );

    addSuccCheckerForReverseDriving_start(astarParams.successorCheckers, graph, 30);

    size_t indSCGoalMaxTime = astarParams.successorCheckers.size();
    astarParams.successorCheckers.emplace_back( nullptr );//to be updated later
    size_t indSCVertexExcludeSet = astarParams.successorCheckers.size();
    astarParams.successorCheckers.emplace_back( nullptr );//to be updated later
    astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_FutureVisits>(restrictedMachineIds_futureVisits) );
    size_t indSCLastEdge = astarParams.successorCheckers.size();
    astarParams.successorCheckers.emplace_back( nullptr );//to be updated later


    //get a plan to each one of the valid adjacent vertices and select the one with the lowest costs
    for (size_t av_ind = 0; av_ind < adjacents.size(); av_ind++) {
        DirectedGraph::vertex_t adjacent_vt = adjacents[av_ind].first;
        DownloadSide av_side = adjacents[av_ind].second;

        DirectedGraph::vertex_property adjVertexProp = graph[adjacent_vt];

        std::set<DirectedGraph::vertex_t> exclude;

        if( adjacent_vt != switching_vt ){

            if (adjVertexProp.route_point.time_stamp >= switching_time
                    && adjVertexProp.harvester_id == harvester_route.machine_id)//adjacent vertex is not valid (hasn't been harvested)
                continue;

            exclude = {switching_vt};//avoid reaching the adjacent vertex through the switching vertex
        }

        astarParams.goal_vt = adjacent_vt;

        int harvester_next_rp_index = geometry::getNextNonRepeatedPointIndex(harvester_route.route_points, harvester_rp_index, -1, -1);
        if(harvester_next_rp_index >= 0){//avoid reaching the adjacent vertex in oposite direction of driving
            for(auto in_edges = boost::in_edges(adjacent_vt, graph) ; in_edges.first != in_edges.second ; in_edges.first++){
                DirectedGraph::vertex_t predecessor = source(*in_edges.first, graph);
                const RoutePoint& predecessorRP = graph[predecessor].route_point;
                double ang = geometry::get_angle( harvester_route.route_points.at(harvester_rp_index), harvester_route.route_points.at(harvester_next_rp_index),
                                                  adjVertexProp.route_point, predecessorRP);
                if( std::fabs(ang) < deg2rad(15) )
                    exclude.insert(predecessor);
            }
        }

        for(auto rv : m_resource_vertices)
            exclude.insert(rv);

        DirectedGraph::adj_iterator vi, vi_end;
        std::vector<std::pair< DirectedGraph::vertex_t, DirectedGraph::vertex_property> > neighbor_vertices;  /// neighbors of the adjacent vertex
        for (boost::tie(vi, vi_end) = adjacent_vertices(adjacent_vt, graph); vi != vi_end; ++vi) {
            DirectedGraph::vertex_t vtTmp = *vi;
            DirectedGraph::vertex_property vpTmp = graph[vtTmp];

            if(  vtTmp != switching_vt &&  //is not the switching vertex and ...
                 vtTmp != m_state.olv_last_vt && //is not the vertex where the machine currently is and ...
                 vpTmp.route_point.time_stamp <= switching_time && //has to be already harvested and ...
                 (
                     vpTmp.route_point.isOfTypeWorking() || //is a working vertex or...
                     ( vpTmp.route_point.type == RoutePoint::HEADLAND
                       && vpTmp.graph_location == DirectedGraph::vertex_property::HEADLAND ) //is a headland-boundary vertex (headland working)
                  )
               ){
                neighbor_vertices.push_back( std::make_pair(vtTmp, vpTmp) );
            }
        }

        if(neighbor_vertices.size() > 1){

            //check for vertices to exclude specifically for headland working
            if( adjVertexProp.route_point.type == RoutePoint::HEADLAND
                    && switching_vt_prop.route_point.type == RoutePoint::TRACK_START
                    && Track::isHeadlandTrack(switching_vt_prop.route_point.track_id)){//if it is headland working, the switching point is a TRACK_START and the adjacent vertex is a headland-boundary vertex...

                //get the last working routpoint in the headland track of the switching point (i.e. before the TRACK_END of that track)
                RoutePoint last_default_rp_in_track;
                bool last_default_rp_in_track_found = false;
                for(size_t hrp = harvester_rp_index ; hrp+1 < harvester_route.route_points.size() ; ++hrp){
                    if(harvester_route.route_points.at(hrp+1).type == RoutePoint::TRACK_END
                            && hrp != harvester_rp_index){
                        last_default_rp_in_track = harvester_route.route_points.at(hrp+1);
                        last_default_rp_in_track_found = true;
                        break;
                    }

                }

                if(last_default_rp_in_track_found){
                    std::vector<DirectedGraph::vertex_t> neighbor_vertices_tmp;
                    for(size_t nv = 0 ; nv < neighbor_vertices.size() ; ++nv){
                        RoutePoint &rp_neighbor = neighbor_vertices.at(nv).second.route_point;
                        if( arolib::geometry::calc_dist(rp_neighbor, last_default_rp_in_track)
                                < arolib::geometry::calc_dist(harvester_route.route_points.at(harvester_rp_index), last_default_rp_in_track))
                            neighbor_vertices_tmp.push_back(neighbor_vertices.at(nv).first);
                    }

                    if(!neighbor_vertices_tmp.empty()){

                        auto it_vt = graph.routepoint_vertex_map().find(last_default_rp_in_track);
                        if(it_vt != graph.routepoint_vertex_map().end()){
                            DirectedGraph::vertex_t exclude_vt = calcNearestVertexToPoint(graph,
                                                                                          graph[it_vt->second].route_point,
                                                                                          neighbor_vertices_tmp);
                            exclude.insert(exclude_vt);
                        }
                    }
                }
            }
        }
        updateExcludeSet(graph, exclude);//add other exclude vertices related to the planning state

        double max_time_goal;
        if(adjacent_vt == switching_vt)
            max_time_goal = std::max(0.0, switching_time - m_settings.clearanceTime + maxDelay);
        else
            max_time_goal = std::max(0.0, switching_time + maxDelay);//the maximum timestamp to reach the ADJACENT vertex

        astarParams.successorCheckers.at(indSCGoalMaxTime) = std::make_shared<AstarSuccessorChecker_GoalMaxTime>(max_time_goal);
        astarParams.successorCheckers.at(indSCVertexExcludeSet) = std::make_shared<AstarSuccessorChecker_VertexExcludeSet_Exceptions1>(exclude);
        replaceSuccCheckerForReverseDriving_OLStart(astarParams.successorCheckers,
                                                    indSCLastEdge,
                                                    graph,
                                                    harvester_route,
                                                    overload_info,
                                                    adjacent_vt,
                                                    adjVertexProp);

        std::string searchType = "to_adj";
        if(av_side == DownloadSide::DS_BEHIND)
            searchType += "_b";
        else if(av_side == DownloadSide::DS_RIGHT)
            searchType += "_r";
        else if(av_side == DownloadSide::DS_LEFT)
            searchType += "_l";
        else if(av_side == DownloadSide::DS_SWITCHING_POINT)
            searchType += "_s";
        else if(av_side == DownloadSide::DS_SWITCHING_POINT_BEHIND)
            searchType += "_sb";

        if(m_state.lastLocation == LOC_HARVESTER_IN)
            searchType += "_OL";

        Astar adjacent_astar(astarParams,
                             m_settings,
                             RoutePoint::TRANSIT,
                             getOutputFolder(searchType),
                             loggerPtr());

        if( !adjacent_astar.plan(graph, m_edgeCostCalculator) ){
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Error calling adjacent_astar on adjacent vertex " + std::to_string(av_ind));
            continue;
        }

        //compare costs with previous plans and update best plan if necessary
        if(minCost > adjacent_astar.getPlan().plan_cost_total){
            minCost = adjacent_astar.getPlan().plan_cost_total;
            plan = adjacent_astar.getPlan();
            best_adjacent_vt_ind = av_ind;
            planOK = true;
        }
    }

    if(!planOK){
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Could not find any plan to adjacent point");
        return false;
    }

    adjVertexInfo = adjacents.at(best_adjacent_vt_ind);

    if(!plan.route_points_.empty()){

        if(adjVertexInfo.first == switching_vt){
            plan.route_points_.back().type = RoutePoint::OVERLOADING_START;
            double timeDiff = plan.route_points_.back().time_stamp - switching_time;
            double clearanceTime = getOlvWaitingTimeToFollowHarv(harvester_route, harvester_rp_index, harv.workingRadius() * 1.2);
            if( timeDiff < clearanceTime ){
                plan.route_points_.back().time_stamp += ( clearanceTime - timeDiff );
                m_state.delay = 0;
            }
            else{
                m_state.delay = std::max(0.0, timeDiff - std::max(0.0, clearanceTime) );
            }


        }
        else{
            m_state.delay = plan.route_points_.back().time_stamp - switching_time;
            m_state.delay = std::max(0.0, m_state.delay);
            olvWaitingTime = std::max(0.0, switching_time - plan.route_points_.back().time_stamp);


            if(olvWaitingTime > 1e-3){//add the costs related to the time the olv has to wait at the switching/adjacent node for the harvester to arrive
                double lastCost = m_edgeCostCalculator->calcCost(m_olv,
                                                                 plan.route_points_.back(),
                                                                 plan.route_points_.back(),
                                                                 olvWaitingTime,
                                                                 olvWaitingTime,
                                                                 m_state.olv_bunker_mass,
                                                                 {});
                plan.plan_cost_ += lastCost;
                plan.plan_cost_total += lastCost;
                logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Adding a final cost = " + std::to_string(lastCost) + " (edge time = " + std::to_string(olvWaitingTime) + " s)");
            }
        }
    }

    m_state.olv_last_vt = adjVertexInfo.first;
    plan.adjustAccessPoints(false);
    updateState(plan, graph, LOC_HARVESTER_IN);

    return true;

}


bool OLVPlan::planPathToAdjacentPoint_2(DirectedGraph::Graph &graph,
                                        AstarPlan & plan,
                                        const DirectedGraph::vertex_t &switching_vt,
                                        double switching_time,
                                        double machine_speed,
                                        const Route &harvester_route,
                                        int harvester_rp_index,
                                        double clearanceDist,
                                        double maxDelay,
                                        double &olvWaitingTime,
                                        const OverloadInfo &overload_info,
                                        const std::vector< std::pair<MachineId_t, double> >& overload_info_prev){
    olvWaitingTime = 0;

    std::vector<std::set<DirectedGraph::vertex_t>> adjacents(2);

    if(harvester_rp_index > 0){
        for(int i = harvester_rp_index-1; i >= 0 ; --i){
            const auto& rpHarvPrev = harvester_route.route_points.at(i);
            if( geometry::calc_dist(harvester_route.route_points.at(harvester_rp_index), rpHarvPrev) < 1e-3 )
                continue;
            DirectedGraph::vertex_t vtPrev;
            if( graph.getClosestVertexInRadius( rpHarvPrev, 1e-3, vtPrev ) )
                adjacents.front().insert(vtPrev);
            break;
        }
    }

    adjacents.at(1) = getAdjacentVerticesForSideOverloading(graph, harvester_route, switching_vt, harvester_rp_index, false);//get the (valid) closest vertices where a connecting edge exists

    if ( adjacents.front().empty() && adjacents.at(1).empty() )
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Unable to find adjacent vertices. (Switching point track id = " + std::to_string( graph[switching_vt].route_point.track_id ) + ")");


    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "adjacents.size() = " + std::to_string( adjacents.front().size() + adjacents.at(1).size() ) +
                                                     " (Switching point track id = " + std::to_string( graph[switching_vt].route_point.track_id ) + ")");

    DirectedGraph::vertex_property &switching_vt_prop = graph[switching_vt];
    bool planOK = false;
    double minCost = std::numeric_limits<double>::max();
    double bestDTime = 0;
    DirectedGraph::vertex_t best_adjacent_vt;
    double bestLastDuration = 0;
    double bestDelay = 0;
    double bestOlvWaitingTime = 0;
    double clearanceTime = getOlvWaitingTimeToFollowHarv(harvester_route, harvester_rp_index, clearanceDist);


    Astar::PlanParameters astarParams;
    astarParams.start_vt = m_state.olv_last_vt;
    astarParams.start_time = m_state.olv_time;
    astarParams.machine = m_olv;
    astarParams.machine_speed = machine_speed;
    astarParams.initial_bunker_mass = m_state.olv_bunker_mass;
    astarParams.includeWaitInCost = false;//false, for the cases where going behind the harvester with delays (better) has the same time-cost as going around covering more distance (worse). The waitTime will be added to the cost afterwards.


    std::set<MachineId_t> restrictedMachineIds;
    std::map<MachineId_t, double> restrictedMachineIds_futureVisits;
    if(graph.outermostTrackIds_HL().find(harvester_route.route_points.at(harvester_rp_index).track_id) != graph.outermostTrackIds_HL().end())
        restrictedMachineIds.insert(Machine::AllMachineIds);
    else
        restrictedMachineIds.insert(harvester_route.machine_id);

    for(auto &oi : overload_info_prev)
        restrictedMachineIds_futureVisits[oi.first] = oi.second;

    //successor checkers
    astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_SuccessorTimestamp>(std::max(0.0, switching_time), //do not drive over any of the vertices that are harvested after the swtiching-vertex
                                                                                                          AstarSuccessorChecker_SuccessorTimestamp::INVALID_IF_LOWER_THAN_SUCC_TIMESPAMP,
                                                                                                          restrictedMachineIds) );

    addSuccCheckerForReverseDriving_start(astarParams.successorCheckers, graph, 30);

    size_t indSCGoalMaxTime = astarParams.successorCheckers.size();
    astarParams.successorCheckers.emplace_back( nullptr );//to be updated later
    size_t indSCVertexExcludeSet = astarParams.successorCheckers.size();
    astarParams.successorCheckers.emplace_back( nullptr );//to be updated later
    astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_FutureVisits>(restrictedMachineIds_futureVisits) );
    size_t indSCLastEdge = astarParams.successorCheckers.size();
    astarParams.successorCheckers.emplace_back( nullptr );//to be updated later

    for(size_t i = 0 ; i < adjacents.size() && !planOK ; ++i){

        // adjacents[0] : vertex of the harv route point before ol start (priority)
        // adjacents[1] : side adjacents

        //get a plan to each one of the valid adjacent vertices and select the one with the lowest costs (if a plan is found for adjacents[0] (i.e. the route point behind), select that one)
        for(auto adjacent_vt : adjacents.at(i)){

            DirectedGraph::vertex_property adjVertexProp = graph[adjacent_vt];

            if (adjVertexProp.route_point.time_stamp >= switching_time
                    && adjVertexProp.harvester_id == harvester_route.machine_id)//adjacent vertex is not valid (hasn't been harvested)
                continue;

            if(graph.boundary_vts().find(adjacent_vt) != graph.boundary_vts().end())
                continue;

            astarParams.goal_vt = adjacent_vt;

            std::set<DirectedGraph::vertex_t> exclude = {switching_vt};//avoid reaching the adjacent vertex through the switching vertex

            int harvester_next_rp_index = geometry::getNextNonRepeatedPointIndex(harvester_route.route_points, harvester_rp_index, -1, -1);
            if(harvester_next_rp_index >= 0){//avoid reaching the adjacent vertex in oposite direction of driving
                auto& rp0 = harvester_route.route_points.at(harvester_rp_index);
                auto& rp1 = harvester_route.route_points.at(harvester_next_rp_index);
                for(auto in_edges = boost::in_edges(adjacent_vt, graph) ; in_edges.first != in_edges.second ; in_edges.first++){
                    DirectedGraph::vertex_t predecessor = source(*in_edges.first, graph);
                    const RoutePoint& predecessorRP = graph[predecessor].route_point;
                    double ang = geometry::get_angle( rp0, rp1, adjVertexProp.route_point, predecessorRP);
                    if( std::fabs(ang) < deg2rad(15) )
                        exclude.insert(predecessor);
                }
            }

            for(auto rv : m_resource_vertices)
                exclude.insert(rv);

            DirectedGraph::adj_iterator vi, vi_end;
            std::vector<std::pair< DirectedGraph::vertex_t, DirectedGraph::vertex_property> > neighbor_vertices;  /// neighbors of the adjacent vertex
            for (boost::tie(vi, vi_end) = adjacent_vertices(adjacent_vt, graph); vi != vi_end; ++vi) {
                DirectedGraph::vertex_t vtTmp = *vi;
                DirectedGraph::vertex_property vpTmp = graph[vtTmp];

                if(  vtTmp != switching_vt &&  //is not the switching vertex and ...
                     vtTmp != m_state.olv_last_vt && //is not the vertex where the machine currently is and ...
                     vpTmp.route_point.time_stamp <= switching_time && //has to be already harvested and ...
                     (
                         vpTmp.route_point.isOfTypeWorking() || //is a working vertex or...
                         ( vpTmp.route_point.type == RoutePoint::HEADLAND
                           && vpTmp.graph_location == DirectedGraph::vertex_property::HEADLAND ) //is a headland-boundary vertex (headland working)
                      )
                   ){
                    neighbor_vertices.push_back( std::make_pair(vtTmp, vpTmp) );
                }
            }

            if(neighbor_vertices.size() > 1){

                //check for vertices to exclude specifically for headland working
                if( adjVertexProp.route_point.type == RoutePoint::HEADLAND
                        && switching_vt_prop.route_point.type == RoutePoint::TRACK_START
                        && Track::isHeadlandTrack(switching_vt_prop.route_point.track_id)){//if it is headland working, the switching point is a TRACK_START and the adjacent vertex is a headland-boundary vertex...

                    //get the last working routpoint in the headland track of the switching point (i.e. before the TRACK_END of that track)
                    RoutePoint last_default_rp_in_track;
                    bool last_default_rp_in_track_found = false;
                    for(size_t hrp = harvester_rp_index ; hrp+1 < harvester_route.route_points.size() ; ++hrp){
                        if(harvester_route.route_points.at(hrp+1).type == RoutePoint::TRACK_END
                                && hrp != harvester_rp_index){
                            last_default_rp_in_track = harvester_route.route_points.at(hrp+1);
                            last_default_rp_in_track_found = true;
                            break;
                        }

                    }

                    if(last_default_rp_in_track_found){
                        std::vector<DirectedGraph::vertex_t> neighbor_vertices_tmp;
                        for(size_t nv = 0 ; nv < neighbor_vertices.size() ; ++nv){
                            RoutePoint &rp_neighbor = neighbor_vertices.at(nv).second.route_point;
                            if( arolib::geometry::calc_dist(rp_neighbor, last_default_rp_in_track)
                                    < arolib::geometry::calc_dist(harvester_route.route_points.at(harvester_rp_index), last_default_rp_in_track))
                                neighbor_vertices_tmp.push_back(neighbor_vertices.at(nv).first);
                        }

                        if(!neighbor_vertices_tmp.empty()){
                            auto it_vt = graph.routepoint_vertex_map().find(last_default_rp_in_track);
                            if(it_vt != graph.routepoint_vertex_map().end()){
                                DirectedGraph::vertex_t exclude_vt = calcNearestVertexToPoint(graph,
                                                                                              graph[it_vt->second].route_point,
                                                                                              neighbor_vertices_tmp);
                                exclude.insert(exclude_vt);
                            }
                        }
                    }
                }
            }
            updateExcludeSet(graph, exclude);//add other exclude vertices related to the planning state

            //compute the time that the machine needs to go from the adjacent vertex to the switching vertex
            double dTime = 0;
            if(machine_speed > 0){
                double distAdj = arolib::geometry::calc_dist(adjVertexProp.route_point, switching_vt_prop.route_point);
                dTime = distAdj / machine_speed;
            }

            double max_time_goal = std::max(0.0, switching_time - dTime + maxDelay);//the maximum timestamp to reach the ADJACENT vertex

            astarParams.successorCheckers.at(indSCGoalMaxTime) = std::make_shared<AstarSuccessorChecker_GoalMaxTime>(max_time_goal);
            astarParams.successorCheckers.at(indSCVertexExcludeSet) = std::make_shared<AstarSuccessorChecker_VertexExcludeSet_Exceptions1>(exclude);

            if(i == 0)
                replaceSuccCheckerForReverseDriving_OLStart(astarParams.successorCheckers,
                                                            indSCLastEdge,
                                                            graph,
                                                            harvester_route,
                                                            overload_info,
                                                            adjacent_vt,
                                                            adjVertexProp);
            else
                astarParams.successorCheckers.at(indSCLastEdge) = nullptr;

            std::string searchType = "to_adj_2";
            if(i == 0)
                searchType += "_b";
            else
                searchType += "_a";
            if(m_state.lastLocation == LOC_HARVESTER_IN)
                searchType += "_OL";

            Astar adjacent_astar(astarParams,
                                 m_settings,
                                 RoutePoint::TRANSIT,
                                 getOutputFolder(searchType),
                                 loggerPtr());

            if( !adjacent_astar.plan(graph, m_edgeCostCalculator) ){
                logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Error calling adjacent_astar on adjacent vertex " + std::to_string(adjacent_vt));
                continue;
            }

            //compare costs with previous plans and update best plan if necessary

            if(minCost <= adjacent_astar.getPlan().plan_cost_total)
                continue;

            auto planTmp = adjacent_astar.getPlan();

            double lastDuration = 0;
            double delay = 0;
            RoutePoint lastRP;
            lastRP.time_stamp = 0;

            double clearanceTimeTmp = std::max(clearanceTime, dTime);
            double olvWaitingTimeTmp = 0;

            if(!planTmp.route_points_.empty()){
                delay = planTmp.route_points_.back().time_stamp - ( switching_time - dTime );
                delay = std::max(0.0, delay);
                lastRP = planTmp.route_points_.back();
                lastDuration = switching_time + delay + clearanceTimeTmp - planTmp.route_points_.back().time_stamp;

                olvWaitingTimeTmp = std::max(0.0, switching_time - planTmp.route_points_.back().time_stamp);
            }

            //end at the switching point after the clearance time
            lastRP.point() = switching_vt_prop.route_point.point();
            lastRP.time_stamp = switching_time + delay + clearanceTimeTmp;
            lastRP.type = RoutePoint::OVERLOADING_START;
            planTmp.route_points_.emplace_back(lastRP);

            //add the cost of the last connection
            {
                DirectedGraph::edge_t e;
                bool edge_found;
                boost::tie(e, edge_found) = boost::edge(adjacent_vt, switching_vt, graph);
                double lastCost = 0;
                if (edge_found) {
                    lastCost  = m_edgeCostCalculator->calcCost(m_olv,
                                                               e,
                                                               graph[e],
                                                               lastDuration,
                                                               0,
                                                               m_state.olv_bunker_mass);
                }
                else if(planTmp.route_points_.size() > 1){
                    lastCost  = m_edgeCostCalculator->calcCost(m_olv,
                                                               planTmp.route_points_.back(),
                                                               r_at(planTmp.route_points_, 1),
                                                               lastDuration,
                                                               0,
                                                               m_state.olv_bunker_mass,
                                                               {});
                }
                planTmp.plan_cost_ += lastCost;
                planTmp.plan_cost_total += lastCost;
            }

            if(minCost > planTmp.plan_cost_total){
                minCost = planTmp.plan_cost_total;
                std::swap(plan, planTmp);
                best_adjacent_vt = adjacent_vt;
                bestDTime = dTime;
                bestLastDuration = lastDuration;
                bestDelay = delay;
                bestOlvWaitingTime = olvWaitingTimeTmp;
                planOK = true;
            }
        }

    }

    if(!planOK){
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Could not find any plan to adjacent point");
        return false;
    }

    olvWaitingTime = bestOlvWaitingTime;
    m_state.delay = bestDelay;

    //add the overrun of the last connection
    DirectedGraph::overroll_property lastOverrun;
    lastOverrun.machine_id = m_olv.id;
    lastOverrun.duration = bestLastDuration;
    lastOverrun.weight = m_olv.weight + m_state.olv_bunker_mass;
    graph.addOverrun(best_adjacent_vt, switching_vt, lastOverrun);


    //m_state.olv_last_vt = best_adjacent_vt;
    m_state.olv_last_vt = switching_vt;

    plan.adjustAccessPoints(false);

    updateState(plan, graph, LOC_HARVESTER_IN);

    return true;
}

bool OLVPlan::planPathToSwitchingPoint(DirectedGraph::Graph &graph,
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
                                       const std::vector< std::pair<MachineId_t, double> >& overload_info_prev)
{
    olvWaitingTime = 0;

    auto switching_rp = harvester_route.route_points.at(harvester_rp_index);

    double clearanceTime = getOlvWaitingTimeToFollowHarv(harvester_route, harvester_rp_index, clearanceDist);
    clearanceTime = std::max(5.0, clearanceTime);//give at least 5 seconds

    Astar::PlanParameters astarParams;
    astarParams.start_vt = m_state.olv_last_vt;
    astarParams.goal_vt = switching_vt;
    astarParams.start_time = m_state.olv_time;
    astarParams.machine = m_olv;
    astarParams.machine_speed = machine_speed;
    astarParams.initial_bunker_mass = m_state.olv_bunker_mass;
    astarParams.includeWaitInCost = false;//false, for the cases where going behind the harvester with delays (better) has the same time-cost as going around covering more distance (worse). The waitTime will be added to the cost afterwards.



    std::set<MachineId_t> restrictedMachineIds;
    std::map<MachineId_t, double> restrictedMachineIds_futureVisits;
    if(graph.outermostTrackIds_HL().find(switching_rp.track_id) != graph.outermostTrackIds_HL().end())
        restrictedMachineIds.insert(Machine::AllMachineIds);
    else
        restrictedMachineIds.insert(harvester_route.machine_id);

    for(auto &oi : overload_info_prev)
        restrictedMachineIds_futureVisits[oi.first] = oi.second;

    std::set<DirectedGraph::vertex_t> exclude;
    for(auto rv : m_resource_vertices)
        exclude.insert(rv);

    double timestampRestriction = switching_time + 1e-3;
    if(overload_info.start_index + 1 < harvester_route.route_points.size()){
        bool found = false;
        DirectedGraph::vertex_t vt_next;
        const auto& rp_next = harvester_route.route_points.at(overload_info.start_index + 1);
        auto it_vt = graph.routepoint_vertex_map().find(rp_next);
        if(it_vt != graph.routepoint_vertex_map().end()){
            vt_next = it_vt->second;
            found == true;
        }
        else if(graph.getClosestVertexInRadius(rp_next, 0.1, vt_next))
            found == true;
        if(found){
            const DirectedGraph::vertex_property vp_next = graph[vt_next];
            if(vp_next.route_point.time_stamp > -1e-9)
                timestampRestriction = std::min(timestampRestriction, vp_next.route_point.time_stamp);
        }
    }

    //successor checkers
    astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_SuccessorTimestamp>(timestampRestriction, //do not drive over any of the vertices that are harvested after the swtiching-vertex
                                                                                                          AstarSuccessorChecker_SuccessorTimestamp::INVALID_IF_LOWER_THAN_SUCC_TIMESPAMP,
                                                                                                          restrictedMachineIds) );
    astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_GoalMaxTime>(switching_time - clearanceTime + maxDelay) );
    astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_VertexExcludeSet_Exceptions1>(exclude) );
    astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_FutureVisits>(restrictedMachineIds_futureVisits) );

    addSuccCheckerForReverseDriving_start(astarParams.successorCheckers, graph, 30);
    replaceSuccCheckerForReverseDriving_OLStart(astarParams.successorCheckers,
                                                astarParams.successorCheckers.size(),
                                                graph,
                                                harvester_route,
                                                overload_info,
                                                switching_vt,
                                                graph[switching_vt]);

    std::string searchType = "to_swt";
    if(m_state.lastLocation == LOC_HARVESTER_IN)
        searchType += "_OL";

    Astar astar(astarParams,
                m_settings,
                RoutePoint::TRANSIT,
                getOutputFolder(searchType),
                loggerPtr());

    if( !astar.plan(graph, m_edgeCostCalculator) ){
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Error calling astar directly to switching vertex " + std::to_string(switching_vt));
        return false;
    }

    plan = astar.getPlan();

    if(!plan.route_points_.empty()){
        m_state.delay = plan.route_points_.back().time_stamp - (switching_time - clearanceTime);
        m_state.delay = std::max(0.0, m_state.delay);
        olvWaitingTime = std::max(0.0, switching_time - plan.route_points_.back().time_stamp);
    }


    if(olvWaitingTime > 1e-3){//add the costs related to the time the olv has to wait at the switching/adjacent node for the harvester to arrive
        double lastCost = m_edgeCostCalculator->calcCost(m_olv,
                                                         plan.route_points_.back(),
                                                         plan.route_points_.back(),
                                                         olvWaitingTime,
                                                         olvWaitingTime,
                                                         m_state.olv_bunker_mass,
                                                         {});
        plan.plan_cost_ += lastCost;
        plan.plan_cost_total += lastCost;
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Adding a final cost = " + std::to_string(lastCost) + " (edge time = " + std::to_string(olvWaitingTime) + " s)");
    }

    m_state.olv_last_vt = switching_vt;

    plan.adjustAccessPoints(false);
    updateState(plan, graph, LOC_HARVESTER_IN);
    return true;
}

bool OLVPlan::planPathToHarvester(DirectedGraph::Graph &graph,
                                  AstarPlan &plan,
                                  Route &harvester_route,
                                  int harvester_rp_index,
                                  const std::shared_ptr<Machine> harv,
                                  double maxDelay,
                                  const OverloadInfo &overload_info,
                                  const std::vector< std::pair<MachineId_t, double> >& overload_info_prev) {


    const RoutePoint switching_rp = harvester_route.route_points.at(harvester_rp_index);
    std::pair<DirectedGraph::vertex_t, DownloadSide> adjVertexInfo;

    double switching_time = switching_rp.time_stamp;  // time when harvester is at switching point
    if (switching_time < -1e-6) {
        logger().printOut(LogLevel::CRITIC, __FUNCTION__, "Invalid switching route-point timestamp.");
        return false;;
    }


    DirectedGraph::vertex_t switching_vt;
    auto it_vt = graph.routepoint_vertex_map().find(switching_rp);
    if(it_vt != graph.routepoint_vertex_map().end())
        switching_vt = it_vt->second;
    else if(!graph.getClosestVertexInRadius(switching_rp, 0.1, switching_vt)){
        logger().printOut(LogLevel::CRITIC, __FUNCTION__, "No (route point) vertex found for the switching point");
        return false;
    }

    double machine_speed = m_olv.calcSpeed(m_state.olv_bunker_mass);

    double clearanceDist = m_olv.workingRadius() * 1.2;
    if(harv)
        clearanceDist = harv->workingRadius() * 1.2;

    double olvWaitingTime = 0;

    // Try to plan to a vertex adjacent to the swtitching vertex
    bool ok;
    if(harv)//use the harvester download sides
        ok = planPathToAdjacentPoint(graph,
                                     plan,
                                     switching_vt,
                                     switching_time,
                                     machine_speed,
                                     harvester_route,
                                     harvester_rp_index,
                                     *harv,
                                     maxDelay,
                                     adjVertexInfo,
                                     olvWaitingTime,
                                     overload_info,
                                     overload_info_prev);
    else{//old fashion
        adjVertexInfo = std::make_pair(switching_vt, DownloadSide::DS_SWITCHING_POINT);
        ok = planPathToAdjacentPoint_2(graph,
                                       plan,
                                       switching_vt,
                                       switching_time,
                                       machine_speed,
                                       harvester_route,
                                       harvester_rp_index,
                                       maxDelay,
                                       clearanceDist,
                                       olvWaitingTime,
                                       overload_info,
                                       overload_info_prev);
    }


    if(!ok){//failed to plan to an adjacent vertex --> try planning directly to the switching point
        adjVertexInfo = std::make_pair(switching_vt, DownloadSide::DS_SWITCHING_POINT);
        ok = planPathToSwitchingPoint(graph,
                                      plan,
                                      switching_vt,
                                      switching_time,
                                      machine_speed,
                                      maxDelay,
                                      harvester_route,
                                      harvester_rp_index,
                                      clearanceDist,
                                      olvWaitingTime,
                                      overload_info,
                                      overload_info_prev);
    }

    if(ok){
        m_state.adjVertexInfo = adjVertexInfo;
        if(m_state.delay > 1e-6)
            addDelayToHarvRoute(graph, harvester_route, harvester_rp_index, m_state.delay);

    }
    return ok;
}

bool OLVPlan::planPathToResource(DirectedGraph::Graph &graph,
                                 DirectedGraph::vertex_t* pResVt,
                                 AstarPlan &plan,
                                 double nextSwitchingPointTimestamp,
                                 const Route &harvester_route,
                                 double maxTimeGoal)
{
    double min_cost = std::numeric_limits<double>::max();
    bool found_plan = false;

    Astar::PlanParameters astarParams;
    astarParams.start_vt = m_state.olv_last_vt;
    astarParams.start_time = m_state.olv_time;
    astarParams.machine = m_olv;
    astarParams.machine_speed = m_olv.calcSpeed(m_state.olv_bunker_mass);
    astarParams.initial_bunker_mass = m_state.olv_bunker_mass;
    astarParams.includeWaitInCost = true;

    std::set<DirectedGraph::vertex_t> exclude = m_state.excludeVts;
    updateExcludeSet(graph, exclude);

    //(initially) exclude resource points when planning to route (the corresponding search resource point will be included later)
    for(auto& it_resP : graph.resourcepoint_vertex_map())
        exclude.insert(it_resP.second);

    //successor checkers

    if(nextSwitchingPointTimestamp > -1e-6) //if nextSwitchingPointTimestamp < 0 -> no limits (everything must be harvested); otherwise, do not drive over any of the vertices that are harvested after the given swtiching-point
        astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_SuccessorTimestamp>(nextSwitchingPointTimestamp,
                                                                                                              AstarSuccessorChecker_SuccessorTimestamp::INVALID_IF_LOWER_THAN_SUCC_TIMESPAMP,
                                                                                                              std::set<MachineId_t>{harvester_route.machine_id}) );

    if(maxTimeGoal > -1e-6)//no limit if maxTimeGoal < 0
        astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_GoalMaxTime>(maxTimeGoal) );


    addSuccCheckerForReverseDriving_start(astarParams.successorCheckers, graph);
    addSuccCheckerForPrevOLEnd(astarParams.successorCheckers, graph, harvester_route);

    size_t indSCVertexExcludeSet = astarParams.successorCheckers.size();
    astarParams.successorCheckers.emplace_back( nullptr );//to be updated later
    //@todo FutureVisits checker is needed

    DirectedGraph::vertex_t resource_vt;

    //plan to each of the resource vertices and select the plan with lowest costs
    for (auto &it : graph.resourcepoint_vertex_map()) {

        DirectedGraph::vertex_t resource_vt_tmp = it.second;

        if(pResVt && *pResVt != resource_vt_tmp)
            continue;

        astarParams.goal_vt = resource_vt_tmp;

        //remove current resource point from exclude set
        exclude.erase(resource_vt_tmp);

        astarParams.successorCheckers.at(indSCVertexExcludeSet) = std::make_shared<AstarSuccessorChecker_VertexExcludeSet_Exceptions1>(exclude);

        Astar aStar(astarParams,
                    m_settings,
                    RoutePoint::TRANSIT,
                    getOutputFolder("to_res"),
                    loggerPtr());

        //add current resource point again to exclude set
        exclude.insert(resource_vt_tmp);

        if( !aStar.plan(graph, m_edgeCostCalculator) ){
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Error calling aStar for resource vertex " + std::to_string(resource_vt_tmp) + " (id = " + std::to_string(it.first.id) + ")");
            continue;  // no plan found => try next resource point
        }


        if (aStar.getPlan().plan_cost_total > min_cost)
            continue;

        plan = aStar.getPlan();

        resource_vt = resource_vt_tmp;
        min_cost = plan.plan_cost_total;
        found_plan = true;
    }
    if(!found_plan){
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "No plan to any of the resource points was found");
        return false;
    }

    m_state.olv_last_vt = resource_vt;

    //@todo we donnot know if the resource point is inside or outside the field
    if(plan.route_points_.size() > 1 && r_at(plan.route_points_, 1).isFieldAccess() )
        plan.adjustAccessPoints(true);
    else
        plan.adjustAccessPoints(false);

    //remove first point (if it exists already)
    if(!plan.route_points_.empty()){
        if(m_state.lastRoutePoint.point() == plan.route_points_.front().point())
            pop_front(plan.route_points_);
    }

    updateState(plan, graph, LOC_RESOURCE_IN);

    removeSpikePointsFromTransitRoute();

    return true;
}

bool OLVPlan::planPathToExitPoint(DirectedGraph::Graph &graph, AstarPlan &plan, double lastOverloadTimestamp, const Route &harvester_route)
{
    double min_cost = std::numeric_limits<double>::max();
    bool found_plan = false;

    Astar::PlanParameters astarParams;
    astarParams.start_vt = m_state.olv_last_vt;
    astarParams.start_time = m_state.olv_time;
    astarParams.machine = m_olv;
    astarParams.machine_speed = m_olv.calcSpeed(m_state.olv_bunker_mass);
    astarParams.initial_bunker_mass = m_state.olv_bunker_mass;
    astarParams.includeWaitInCost = true;

    std::set<DirectedGraph::vertex_t> exclude = m_state.excludeVts;

    //exclude resource points
    for(auto& it_resP : graph.resourcepoint_vertex_map())
        exclude.insert(it_resP.second);

    updateExcludeSet(graph, exclude);

    //successor checkers

    //No AstarSuccessorChecker_GoalMaxTime: no limit, just get there

    if(lastOverloadTimestamp > -1e-6) //if lastOverloadTimestamp < 0 -> no limits (everything must be harvested); otherwise, do not drive over any of the vertices that are harvested after the given swtiching-point
        astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_SuccessorTimestamp>(lastOverloadTimestamp,
                                                                                                               AstarSuccessorChecker_SuccessorTimestamp::INVALID_IF_LOWER_THAN_SUCC_TIMESPAMP,
                                                                                                               std::set<MachineId_t>{Machine::AllMachineIds}) );


    addSuccCheckerForReverseDriving_start(astarParams.successorCheckers, graph);
    addSuccCheckerForPrevOLEnd(astarParams.successorCheckers, graph, harvester_route);

    astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_VertexExcludeSet_Exceptions1>(exclude) );
    //@todo FutureVisits checker is needed

    DirectedGraph::vertex_t fap_vt;

    //compute a plan to each one of the valid field-exit vertices and select the best one
    for(auto &it : graph.accesspoint_vertex_map()){

        const FieldAccessPoint& fap = it.first;

        //check if it is a valid exit point
        if(fap.accessType == FieldAccessPoint::AccessPointType::AP_ENTRY_ONLY )//not an exit point
            continue;

        DirectedGraph::vertex_t fap_vt_tmp = it.second;

        astarParams.goal_vt = fap_vt_tmp;

        Astar aStar(astarParams,
                    m_settings,
                    RoutePoint::TRANSIT,
                    getOutputFolder("to_ext"),
                    loggerPtr());

        if( !aStar.plan(graph, m_edgeCostCalculator) ){
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Error calling aStar for field exit vertex " + std::to_string(fap_vt_tmp) + " (id = " + std::to_string(it.first.id) + ")");
            continue;  // no plan found => try next exit point
        }

        if (aStar.getPlan().plan_cost_total > min_cost)
            continue;

        plan = aStar.getPlan();

        fap_vt = fap_vt_tmp;
        min_cost = plan.plan_cost_total;

        found_plan = true;
    }
    if(!found_plan){
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "No plan to any of the exit points points was found");
        return false;
    }

    //remove first point (if it exists already)
    if(!plan.route_points_.empty()){
        if(m_state.lastRoutePoint.point() == plan.route_points_.front().point())
            pop_front(plan.route_points_);
    }

    m_state.olv_last_vt = fap_vt;
    plan.adjustAccessPoints(true);
    updateState(plan, graph, LOC_EXIT);

    removeSpikePointsFromTransitRoute();

    return true;

}

bool OLVPlan::planOverloadingPath_behind(DirectedGraph::Graph &graph, Route &harvester_route, const OverloadInfo &overload_info, const std::shared_ptr<Machine> harv)
{
    RoutePoint switching_rp = harvester_route.route_points.at(overload_info.start_index);
    RoutePoint end_rp_harv = harvester_route.route_points.at(overload_info.end_index);
    double initial_bunkermass = m_state.olv_bunker_mass;

    double prevCost = m_state.plan_cost;

    //obtain the route points of the overloading segment, add them to the route, and update plan data

    double clearanceTime = 5;
    if(!m_state.route.route_points.empty()){
        if(m_state.route.route_points.back().type == RoutePoint::OVERLOADING_START)
            clearanceTime = std::max(0.0, m_state.route.route_points.back().time_stamp - harvester_route.route_points.at(overload_info.start_index).time_stamp);
    }
    else if(harv)
        clearanceTime = getOlvWaitingTimeToFollowHarv(harvester_route, overload_info.start_index, harv->workingRadius() * 1.2);

    std::vector<RoutePoint> follow_rps = followHarvester(graph, overload_info, harvester_route, harv, initial_bunkermass, clearanceTime);
    if(follow_rps.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "No route points to follow harvester were obtained.");
        return false;
    }

    //update costs, edges properties and visit periods
    updateFromRouteSegment( graph, follow_rps, m_settings.includeCostOfOverload, harv, 0, -1 );

    if(!m_state.route.route_points.empty() && geometry::calc_dist( m_state.route.route_points.back(), switching_rp ) < 1e-3 )//remove the prevoius point overload start, it will be inserted with the follow_rps
        m_state.route.route_points.pop_back();

    m_state.route.route_points.insert(m_state.route.route_points.end(), follow_rps.begin(), follow_rps.end());
    m_state.olv_bunker_mass = follow_rps.back().bunker_mass;

    m_state.olv_time = m_state.route.route_points.back().time_stamp;
    m_state.prev_overloading_end_index = overload_info.end_index;
    m_state.planCost_overloading = m_state.plan_cost - prevCost;


    if(graph.getClosestVertexInRadius( m_state.route.route_points.back(), 1e-3, m_state.olv_last_vt)){

        if( std::isnan(m_state.planCost_overloading) ){
            logger().printOut(LogLevel::CRITIC, __FUNCTION__, 10,
                              "m_state.planCost_overloading in NAN! \n",
                              "\t prevCost = ", prevCost,
                              "\t m_state.plan_cost = ", m_state.plan_cost);
        }

        //try to finish at an adjacent point and not in the last OL working point
        if(!m_state.route.route_points.empty()){
            auto adjacents = getAdjacentVerticesForSideOverloading(graph, harvester_route, m_state.olv_last_vt, -1, true);
            DirectedGraph::vertex_t bestAdj = -1;
            bool bestAdjFound = false;
            AstarPlan best_plan;
            double min_cost = std::numeric_limits<double>::max();

            std::set<DirectedGraph::vertex_t> adjacentsSet;

            auto out_edges= boost::out_edges(m_state.olv_last_vt, graph);
            for(;out_edges.first != out_edges.second; out_edges.first++)
                adjacentsSet.insert( target(*out_edges.first, graph) );

            adjacentsSet.insert(adjacents.begin(), adjacents.end());

            Astar::PlanParameters astarParams;
            astarParams.start_vt = m_state.olv_last_vt;
            astarParams.start_time = m_state.olv_time;
            astarParams.machine = m_olv;
            astarParams.machine_speed = m_olv.calcSpeed(m_state.olv_bunker_mass);
            astarParams.initial_bunker_mass = m_state.olv_bunker_mass;
            astarParams.includeWaitInCost = m_settings.includeWaitInCost;

            //successor checkers
            astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_SuccessorTimestamp>(std::max(0.0, end_rp_harv.time_stamp/*-clearanceTime*/-1e-3),
                                                                                                                  AstarSuccessorChecker_SuccessorTimestamp::INVALID_IF_LOWER_THAN_SUCC_TIMESPAMP,
                                                                                                                  std::set<MachineId_t>{harvester_route.machine_id}) );

            auto excludeVetRevDriving = addSuccCheckerForReverseDriving_start(astarParams.successorCheckers, graph, 30);

            for(auto vtrd : excludeVetRevDriving)
                adjacentsSet.erase(vtrd);

            size_t indSCVertexExcludeSet = astarParams.successorCheckers.size();
            astarParams.successorCheckers.emplace_back( nullptr );//to be updated later
            //@todo check if a FutureVisits checker is needed

            std::set<DirectedGraph::vertex_t> exclude = adjacentsSet;
            for(auto adj : adjacents ){
                astarParams.goal_vt = adj;
                exclude.erase(adj);//exclude all other adjacent vertices except this one

                astarParams.successorCheckers.at(indSCVertexExcludeSet) = std::make_shared<AstarSuccessorChecker_VertexExcludeSet_Exceptions1>(exclude);

                Astar astar(astarParams, m_settings, RoutePoint::TRANSIT, "", loggerPtr());

                exclude.insert(adj);//insert again

                if(!astar.plan(graph,m_edgeCostCalculator))
                    continue;
                if(min_cost < astar.getPlan().plan_cost_total
                        || astar.getPlan().route_points_.size() != 2)
                    continue;

                min_cost = astar.getPlan().plan_cost_total;
                bestAdj = adj;
                bestAdjFound = true;
                best_plan.adjustAccessPoints(false);
                best_plan = astar.getPlan();
            }
            if(bestAdjFound){//replace last route point and add cost, overrun and visiting period
                auto lastRP_prev = m_state.route.route_points.back();
                if(m_state.route.route_points.size() > 1){
                    double clearanceDist = -1;
                    if(harv)
                        clearanceDist = getClearanceDistBehindHarv(*harv);
                    if(clearanceDist <= 0)
                        clearanceDist = m_olv.workingRadius();
                    double dist_harv = arolib::geometry::calc_dist( lastRP_prev, end_rp_harv );
                    double dist_prev = arolib::geometry::calc_dist( lastRP_prev, r_at(m_state.route.route_points, 1) );
                    if( dist_harv < 1e-3 && dist_prev > 1.1 * clearanceDist )
                        m_state.route.route_points.back().point() = geometry::getPointInLineAtDist(lastRP_prev, r_at(m_state.route.route_points, 1), clearanceDist);
                }

                if(m_settings.includeCostOfOverload)
                    m_state.planCost_overloading += min_cost;
                else
                    best_plan.plan_cost_ = 0;

                if(!best_plan.route_points_.empty())//remove first RP from plan to avoid repeated points
                    pop_front(best_plan.route_points_);

                updateState(best_plan, graph, LocationType::LOC_HARVESTER_OUT);
                m_state.olv_last_vt = bestAdj;

            }
        }


        return true;
    }

//    if(Track::isInfieldTrack(end_rp_harv.track_id)){//search the closest headland point and finish there
//        double rad = 1.5 * ( harv ? harv->workingRadius() : m_olv.workingRadius());
//        bool found = false;
//        double min_dist = std::numeric_limits<double>::max();
//        double min_delay = std::numeric_limits<double>::max();
//        double lastTimestamp;
//        DirectedGraph::vertex_t nearest_vertex;
//        DirectedGraph::vertex_property nearest_vertex_prop;
//        RoutePoint prevRP = m_state.route.route_points.size() > 1 ? r_at(m_state.route.route_points, 1) : m_state.route.route_points.back();
//        double speed = m_olv.max_speed_full;
//        std::set<MachineId_t> olvId = {m_olv.id};
//        graph.getVerticesInRadius(m_state.route.route_points.back(), rad,
//                                  [&min_dist, &min_delay, &nearest_vertex, &nearest_vertex_prop, &found, &end_rp_harv, &prevRP, &speed, &lastTimestamp, &olvId]
//                                  (const DirectedGraph::vertex_t& vt, const DirectedGraph::vertex_property& vertex_prop)->bool{
//            if( vertex_prop.route_point.type == RoutePoint::HEADLAND
//                    || Track::isHeadlandTrack( vertex_prop.route_point.track_id ) ){
//                double dist = geometry::calc_dist(prevRP, vertex_prop.route_point);
//                double time_in = prevRP.time_stamp + ( dist <= 0 || speed <= 0 ? 0 : dist/speed );
//                double delay;
//                bool bTmp;
//                std::vector<MachineId_t> vmTmp;
//                DirectedGraph::VisitPeriod::isBusy(vertex_prop.visitPeriods,
//                                                   time_in,
//                                                   5,//worarround
//                                                   1,
//                                                   delay,
//                                                   -1,
//                                                   bTmp,
//                                                   vmTmp,
//                                                   olvId);

//                if( min_delay - 0.5 > delay ||
//                        ( std::fabs(min_delay-delay) < 0.5 && min_dist > dist )){
//                    found = true;
//                    min_delay = delay;
//                    min_dist = dist;
//                    nearest_vertex = vt;
//                    nearest_vertex_prop = vertex_prop;
//                    lastTimestamp = time_in + min_delay;
//                }
//            }
//            return false;//we dont really nead the list of vertices
//        });

//        if(found){
//            m_state.prev_overloading_end_index = overload_info.end_index;
//            m_state.planCost_overloading = m_state.plan_cost - prevCost;
//            m_state.route.route_points.back().time_stamp = lastTimestamp;
//            m_state.route.route_points.back().point() = nearest_vertex_prop.route_point.point();
//            m_state.route.route_points.back().type = RoutePoint::OVERLOADING_FINISH;
//            m_state.olv_time = lastTimestamp;
//            m_state.olv_last_vt = nearest_vertex;
//            return true;
//        }

//    }
//    logger().printOut(LogLevel::CRITIC, __FUNCTION__, "No (route point) vertex found for the switching (end) point nor a valid vertex nearby");
//    return false;

    m_state.leaveOverloadingFromClosestVt = true;
    return true;

}

bool OLVPlan::planOverloadingPath_alongside(DirectedGraph::Graph &graph,
                                            Route &harvester_route,
                                            const OLVPlan::OverloadInfo &overload_info,
                                            const std::shared_ptr<Machine> harv)
{
    double prevCost = m_state.plan_cost;

    OLVPlan::OverloadInfo info_tpm;
    std::vector< std::pair<RoutePoint::RoutePointType, OLVPlan::OverloadInfo> > subInfos;

    info_tpm.start_index = overload_info.start_index;
    int prevTrackId = harvester_route.route_points.at(info_tpm.start_index).track_id;
    for(size_t i = overload_info.start_index + 1 ; i <= overload_info.end_index ; ++i ){
        auto& rp = harvester_route.route_points.at(i);
        if(prevTrackId >= 0){
            if(rp.track_id >= 0 && i != overload_info.end_index)
                continue;

            //if(i-1 != info_tpm.start_index){
                info_tpm.end_index = i-1;
                subInfos.emplace_back( std::make_pair(RoutePoint::DEFAULT, info_tpm) );
                info_tpm.start_index = i-1;
                prevTrackId = rp.track_id;
                continue;
            //}
        }
        else{
            if(rp.track_id < 0 && i != overload_info.end_index)
                continue;

            //if(i-1 != info_tpm.start_index){
                info_tpm.end_index = i;
                subInfos.emplace_back( std::make_pair(RoutePoint::TRANSIT, info_tpm) );
                info_tpm.start_index = i;
                prevTrackId = rp.track_id;
                continue;
            //}

        }
    }
    if(!subInfos.empty())
        subInfos.back().second.end_index = overload_info.end_index;

    //obtain the route points of the overloading segment, add them to the route, and update plan data

    m_state.prev_overloading_end_index = -1;
    auto prevRoutePoints = m_state.route.route_points;
    auto prevRoutePoints_prev = m_state.route_points_prev;
    if(!prevRoutePoints.empty())
        m_state.route_points_prev = prevRoutePoints;
    m_state.route.route_points.clear();

    std::vector<RoutePoint> &stateRoutePoints = m_state.route.route_points;//the state route points are changed by some methods, so the OL segment will be saved there and later the previous (original) route points have to be added in the front

    for(size_t i = 0 ; i < subInfos.size() ; ++i){
        auto & infoPair = subInfos.at(i);
        if(infoPair.second.start_index == infoPair.second.end_index){//@todo: special case with one point
            //...
        }
        else if(infoPair.first == RoutePoint::DEFAULT){//
            if(m_state.adjVertexInfo.second == DownloadSide::DS_RIGHT || m_state.adjVertexInfo.second == DownloadSide::DS_LEFT){
                std::vector<RoutePoint> follow_rps = followHarvesterAlongside(graph, infoPair.second, harvester_route, m_state.olv_bunker_mass, true, harv);
                if(follow_rps.empty()){
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, "No route points to follow harvester (in track) alonside were obtained.");
                    m_state.route.route_points = prevRoutePoints;
                    return false;
                }

                //update costs, edges properties and visit periods
                updateFromRouteSegment( graph, follow_rps, m_settings.includeCostOfOverload, harv, 0, -1 );

                if(!stateRoutePoints.empty())//remove the last route point obtained from the connection plan
                    stateRoutePoints.pop_back();


                while(!stateRoutePoints.empty()){//remove the points of the last connection that lay in front the first segment of follow_rps or are too close to it
                    double dist = geometry::calc_dist(stateRoutePoints.back(), follow_rps.front());
                    double olvTurningRad = m_olv.getTurningRadius();

                    while( follow_rps.size() > 1 &&
                            follow_rps.front().point() == follow_rps.at(1).point() ){
                        pop_front(follow_rps);
                    }

                    if(follow_rps.size() > 1){
                        double ang = geometry::get_angle( follow_rps.at(1), follow_rps.front(), stateRoutePoints.back(), true );
                        if(std::fabs(ang) < 45 ||
                               ( std::fabs(ang) < 90 && (olvTurningRad < 1e-6 || dist < olvTurningRad) ) ){
                            stateRoutePoints.pop_back();
                            continue;
                        }
                    }
                    if(dist < 0.5 * olvTurningRad){
                        stateRoutePoints.pop_back();
                        continue;
                    }
                    break;
                }

                stateRoutePoints.insert( stateRoutePoints.end(), follow_rps.begin(), follow_rps.end() );
                m_state.olv_time = stateRoutePoints.back().time_stamp;
                m_state.olv_bunker_mass = stateRoutePoints.back().bunker_mass;
                m_state.lastRoutePoint = stateRoutePoints.back();

                bool found = false;
                auto vts = graph.getVerticesInRadius(stateRoutePoints.back(), m_olv.workingRadius());
                if( !vts.empty() ){
                    double min_dist = std::numeric_limits<double>::max();
                    for(auto vt : vts){
                        const Point p_vt = graph[vt].route_point.point();
                        double dist = arolib::geometry::calc_dist(stateRoutePoints.back(), p_vt);
                        if( min_dist > dist ){
                            min_dist = dist;
                            found = true;
                            m_state.olv_last_vt = vt;
                        }
                    }
                    if(!found){
                        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error obtaining proximal vertices (follow in track).");
                        m_state.route.route_points = prevRoutePoints;
                        return false;
                    }
                }
                else{//no proximal vertices were added (maybe they are relativelly far)
                    m_state.leaveOverloadingFromClosestVt = true;
                }
                m_state.prev_overloading_end_index = infoPair.second.end_index;
            }
            else{//do the rest of the overloading following behind
                OLVPlan::OverloadInfo lastOLInfo = infoPair.second;
                lastOLInfo.end_index = subInfos.back().second.end_index;
                if(!planOverloadingPath_behind(graph, harvester_route, lastOLInfo, harv)){
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error planning overload segment (behind harvester).");
                    m_state.route.route_points = prevRoutePoints;
                    return false;
                }
                m_state.leaveOverloadingFromClosestVt = false;
                m_state.prev_overloading_end_index = lastOLInfo.end_index;
                break;
            }
        }
        else{
            DownloadSide nextSideSame = DownloadSide::DS_RIGHT, nextSideChanged = DownloadSide::DS_LEFT;
            if(m_state.adjVertexInfo.second == DownloadSide::DS_LEFT)
                std::swap(nextSideSame, nextSideChanged);

            bool keepSide = checkSideOverloading(graph, harvester_route, infoPair.second.end_index, overload_info.end_index, nextSideSame, harv);
            bool changeSide = checkSideOverloading(graph, harvester_route, infoPair.second.end_index, overload_info.end_index, nextSideChanged, harv);;

            bool planPath = true;
            auto stateRoutePoints2 = stateRoutePoints;
            if(changeSide || keepSide){//if possible to change sides, go for it: this means a shorter hl connection
                std::vector<RoutePoint> follow_rps = followHarvesterAlongside(graph, infoPair.second, harvester_route, m_state.olv_bunker_mass, !changeSide, harv);

                if(!stateRoutePoints.empty()){
                    while(!follow_rps.empty()){//remove the connection points that lay behind the last stateRoutePoint or are too close to it
                        double dist = geometry::calc_dist(stateRoutePoints.back(), follow_rps.front());
                        double olvTurningRad = m_olv.getTurningRadius();
                        if(stateRoutePoints.size() > 1){
                            double ang = geometry::get_angle( r_at(stateRoutePoints, 1), stateRoutePoints.back(), follow_rps.front(), true );
                            if(std::fabs(ang) < 45 ||
                                   ( std::fabs(ang) < 90 && (olvTurningRad < 1e-6 || dist < olvTurningRad) ) ){
                                pop_front(follow_rps);
                                continue;
                            }
                        }
                        if(dist < 0.5 * olvTurningRad){
                            pop_front(follow_rps);
                            continue;
                        }
                        break;
                    }
                }

                if(!follow_rps.empty()){

                    //update costs, edges properties and visit periods
                    updateFromRouteSegment( graph, follow_rps, m_settings.includeCostOfOverload, harv, 0, -1 );

                    stateRoutePoints.insert( stateRoutePoints.end(), follow_rps.begin(), follow_rps.end() );
                    m_state.olv_time = stateRoutePoints.back().time_stamp;
                    m_state.olv_bunker_mass = stateRoutePoints.back().bunker_mass;
                    m_state.lastRoutePoint = stateRoutePoints.back();

                    bool found = false;

                    found = false;
                    auto vts = graph.getVerticesInRadius(stateRoutePoints.back(), m_olv.workingRadius());
                    if( !vts.empty() ){
                        double min_dist = std::numeric_limits<double>::max();
                        for(auto vt : vts){
                            const Point p_vt = graph[vt].route_point.point();
                            double dist = arolib::geometry::calc_dist(stateRoutePoints.back(), p_vt);
                            if( min_dist > dist ){
                                min_dist = dist;
                                found = true;
                                m_state.olv_last_vt = vt;
                            }
                        }
                        if(!found){
                            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error obtaining proximal vertices (follow in headland).");
                            m_state.route.route_points = prevRoutePoints;
                            return false;
                        }
                    }
                    else{//no proximal vertices were added (maybe they are relativelly far)
                        m_state.leaveOverloadingFromClosestVt = true;
                    }
                    m_state.prev_overloading_end_index = infoPair.second.end_index;

                    if(changeSide){
                        if(m_state.adjVertexInfo.second == DownloadSide::DS_LEFT)
                            m_state.adjVertexInfo.second= DownloadSide::DS_RIGHT;
                        else
                            m_state.adjVertexInfo.second= DownloadSide::DS_LEFT;
                    }

                    planPath = false;
                }

            }

            if(planPath){
                stateRoutePoints = stateRoutePoints2;

                //plan the connection with astar
                if(stateRoutePoints.empty()){
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, "Current segment has no route points .");
                    m_state.route.route_points = prevRoutePoints;
                    return false;
                }
                if(i+1 >= subInfos.size()){
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, "Last segment is a connection");
                    m_state.route.route_points = prevRoutePoints;
                    return false;
                }

                if(m_state.leaveOverloadingFromClosestVt){
                    if(!updateCurrentVertexWithClosest(graph, harvester_route)){
                        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Unable to update current vertex");
                        m_state.route.route_points = prevRoutePoints;
                        return false;
                    }
                }

                RoutePoint startRP = graph[m_state.olv_last_vt].route_point;
                RoutePoint lastRP = stateRoutePoints.back();

                double dist = arolib::geometry::calc_dist(lastRP, startRP);
                if(dist < 0.25 * m_olv.width)
                    stateRoutePoints.pop_back();

                double maxDelay = std::numeric_limits<double>::max();
                if(m_settings.clearanceTime > 1e-3)
                    maxDelay = 5 * m_settings.clearanceTime;

                AstarPlan plan;
                if(!planPathToHarvester(graph,
                                        plan,
                                        harvester_route,
                                        subInfos.at(i+1).second.start_index,
                                        harv,
                                        maxDelay,
                                        subInfos.at(i+1).second,
                                        {})){
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error planning path to harvester");
                    m_state.route.route_points = prevRoutePoints;
                    return false;
                }

                m_state.olv_last_vt = m_state.adjVertexInfo.first;
            }

            m_state.olv_time = stateRoutePoints.back().time_stamp;
            m_state.olv_bunker_mass = stateRoutePoints.back().bunker_mass;
            m_state.lastRoutePoint = stateRoutePoints.back();
        }
    }

    if(!stateRoutePoints.empty()){
        stateRoutePoints.front().type = RoutePoint::OVERLOADING_START;
        stateRoutePoints.back().type = RoutePoint::OVERLOADING_FINISH;
        for(size_t i = 1 ; i+1 < stateRoutePoints.size() ; ++i)
            stateRoutePoints.at(i).type = RoutePoint::OVERLOADING;
    }

    while(prevRoutePoints.size() > 1 && !stateRoutePoints.empty()
          && r_at(prevRoutePoints, 1).type == RoutePoint::TRANSIT
          && arolib::geometry::getLocationInLine(prevRoutePoints.back(), r_at(prevRoutePoints, 1), stateRoutePoints.front()) == 0)
        prevRoutePoints.pop_back();

    if(!stateRoutePoints.empty() && !prevRoutePoints.empty()
            && arolib::geometry::calc_dist( prevRoutePoints.back(), stateRoutePoints.front().point() ) < 0.5 * m_olv.width )
        prevRoutePoints.pop_back();

    //restore previous state route points
    stateRoutePoints.insert(stateRoutePoints.begin(), prevRoutePoints.begin(), prevRoutePoints.end());
    m_state.route_points_prev = prevRoutePoints_prev;


    if(m_state.adjVertexInfo.second == DownloadSide::DS_RIGHT || m_state.adjVertexInfo.second == DownloadSide::DS_LEFT){
        m_state.leaveOverloadingFromClosestVt = true;
    }

    m_state.prev_overloading_end_index = overload_info.end_index;
    m_state.planCost_overloading = m_state.plan_cost - prevCost;
    m_state.olv_bunker_mass = m_state.route.route_points.back().bunker_mass;
    if(!m_state.route.route_points.empty())
        m_state.olv_time = m_state.route.route_points.back().time_stamp;
    else
        m_state.olv_time = harvester_route.route_points.at(overload_info.end_index).time_stamp;

    return true;
}

bool OLVPlan::checkSideOverloading(const DirectedGraph::Graph &graph,
                                   const Route &harvester_route,
                                   size_t startInd,
                                   size_t endInd,
                                   DownloadSide &downloadSide,
                                   const std::shared_ptr<Machine> harv)
{
    if(downloadSide != DownloadSide::DS_RIGHT && downloadSide != DownloadSide::DS_LEFT)
        return false;

    if(harv){
        if( downloadSide == DownloadSide::DS_RIGHT && (harv->unload_sides & Machine::WorkingSide_RIGHT) == 0)
            return false;
        if( downloadSide == DownloadSide::DS_LEFT && (harv->unload_sides & Machine::WorkingSide_LEFT) == 0)
            return false;
    }

    for(; startInd < harvester_route.route_points.size(); ++startInd){
        if(harvester_route.route_points.at(startInd).track_id >= 0)
            break;
    }

    for(size_t i = startInd+1 ; i < harvester_route.route_points.size() && i <= endInd ; ++i){
        if(harvester_route.route_points.at(i).track_id < 0){
            endInd = i-1;
            break;
        }
    }

    if(startInd+1 >= harvester_route.route_points.size() || startInd >= endInd)
        return false;

    auto vt_it = graph.routepoint_vertex_map().find(harvester_route.route_points.at(startInd));
    if(vt_it == graph.routepoint_vertex_map().end()){
        logger().printWarning(__FUNCTION__, "Unable to find vertex corresponding to route point " + std::to_string(startInd));
        return false;
    }
    const auto& startTimestamp = graph[vt_it->second].route_point.time_stamp;

    int vtsBoundaryCount = 0;

    for(size_t i = startInd; i+1 < harvester_route.route_points.size() && i <= endInd; ++i){

        int harvester_next_rp_index = geometry::getNextNonRepeatedPointIndex(harvester_route.route_points, i, -1, -1);
        if(harvester_next_rp_index < 0)
            continue;

        const auto& rp0 = harvester_route.route_points.at(i);
        const auto& rp1 = harvester_route.route_points.at(harvester_next_rp_index);
        vt_it = graph.routepoint_vertex_map().find(rp0);
        if(vt_it == graph.routepoint_vertex_map().end()){
            logger().printWarning(__FUNCTION__, "Unable to find vertex corresponding to route point " + std::to_string(i));
            return false;
        }
        const auto& vt_harv = vt_it->second;
        const auto& timestamp_harv = graph[vt_harv].route_point.time_stamp;

        int vtsBoundaryCountTmp = 0;

        auto connectedVts = getConnectedVertices(graph, vt_harv, true);
        for(auto& vt : connectedVts){
            const RoutePoint& rp = graph[vt].route_point;
            if(rp.track_id == rp0.track_id)
                continue;
            if(rp.isOfType({RoutePoint::FIELD_ENTRY, RoutePoint::FIELD_EXIT, RoutePoint::RESOURCE_POINT}))
                continue;
            double ang = geometry::get_angle(rp1, rp0, rp, true);//in degrees
            if(downloadSide == DownloadSide::DS_LEFT){
                if(ang < 5 || ang > 135)
                    continue;
            }
            else{
                if(ang > -5 || ang < -135)
                    continue;
            }
            if(rp.type == RoutePoint::TRANSIT_OF){//--> boundary
                ++vtsBoundaryCountTmp;
                continue;
            }
            if(rp.time_stamp > -1e-9 &&
                    (rp.time_stamp > startTimestamp || rp.time_stamp > timestamp_harv) )//not yet harvested
                return false;
        }
        if(vtsBoundaryCountTmp > 0)
            ++vtsBoundaryCount;
    }

    if(vtsBoundaryCount > 2)
        return false;

    return true;
}

std::vector<RoutePoint> OLVPlan::followHarvester(DirectedGraph::Graph &graph,
                                                 const OverloadInfo &info,
                                                 const Route &harvester_route,
                                                 const std::shared_ptr<Machine> harv,
                                                 double initial_bunker_mass,
                                                 double clearanceTime) {

    std::vector<RoutePoint> ret;
    if(info.end_index-info.start_index < 1)
        return ret;

    ret.reserve( std::max(0, info.end_index - info.start_index + 2) );

    double clearanceDist = -1;
    if(harv)
        clearanceDist = getClearanceDistBehindHarv(*harv);

    if(clearanceDist < 1e-9)
        return followHarvesterNoLength(graph, info, harvester_route, harv, initial_bunker_mass, clearanceTime);


    std::function<bool(size_t, double, double&, double&, double&)> getDataAhead =
            [&getDataAhead, &harvester_route, &info]
            (size_t ind, double rem_length, double& timestamp, double& worked_mass, double& worked_volume)->bool{

        if(ind == info.end_index){
            timestamp = harvester_route.route_points.at(ind).time_stamp;
            worked_mass = harvester_route.route_points.at(ind).worked_mass;
            worked_volume = harvester_route.route_points.at(ind).worked_volume;
            return true;
        }

        double dist = geometry::calc_dist( harvester_route.route_points.at(ind), harvester_route.route_points.at(ind+1) );
        if(dist < rem_length)
            return getDataAhead(ind+1, rem_length - dist, timestamp, worked_mass, worked_volume);

        double dTimeHarv = harvester_route.route_points.at(ind+1).time_stamp - harvester_route.route_points.at(ind).time_stamp;
        double dMassHarv = harvester_route.route_points.at(ind+1).worked_mass - harvester_route.route_points.at(ind).worked_mass;
        double dVolumeHarv = harvester_route.route_points.at(ind+1).worked_volume - harvester_route.route_points.at(ind).worked_volume;

        double eps = std::min(1.0, rem_length / dist);

        bool finished = false;
        if(ind+1 == info.end_index && eps > 0.95){
            eps = 1;
            finished = true;
        }

        timestamp = harvester_route.route_points.at(ind).time_stamp + eps * dTimeHarv;
        worked_mass = harvester_route.route_points.at(ind).worked_mass + eps * dMassHarv;
        worked_volume = harvester_route.route_points.at(ind).worked_volume + eps * dVolumeHarv;

        return finished;
    };

    double worked_mass_prev = harvester_route.route_points.at(info.start_index).worked_mass;
    double worked_volume_prev = harvester_route.route_points.at(info.start_index).worked_volume;
    bool finisheAtEnd = false;
    for (int i = info.start_index; i < info.end_index; ++i) {
        const RoutePoint& harvester_rp = harvester_route.route_points.at(i);
        RoutePoint olv_rp;
        if(ret.empty())
            olv_rp.type = RoutePoint::OVERLOADING_START;
        else
            olv_rp.type = RoutePoint::OVERLOADING;

        olv_rp.point() = harvester_rp.point();
        olv_rp.track_id = harvester_rp.track_id;

        double worked_mass, worked_volume;
        finisheAtEnd = getDataAhead(i, clearanceDist, olv_rp.time_stamp, worked_mass, worked_volume);

        const double dTimeCheck = 0.1; // used check if the given timestamp is lower/before that the last olv timestamp (should not happen, but just in case)
        if(ret.empty()){
            if(!m_state.route.route_points.empty() && olv_rp.time_stamp < m_state.route.route_points.back().time_stamp + dTimeCheck)
                olv_rp.time_stamp = m_state.route.route_points.back().time_stamp + dTimeCheck;
        }
        else{
            if(olv_rp.time_stamp < ret.back().time_stamp + dTimeCheck)
                olv_rp.time_stamp =  ret.back().time_stamp + dTimeCheck;
        }

        if(ret.empty()){
            olv_rp.bunker_mass = worked_mass - worked_mass_prev + initial_bunker_mass;
            //olv_rp.bunker_volume = worked_volume - worked_volume_prev + initial_bunker_volume;
        }
        else{
            olv_rp.bunker_mass = worked_mass - worked_mass_prev + ret.back().bunker_mass;
            olv_rp.bunker_volume = worked_volume - worked_volume_prev + ret.back().bunker_volume;
        }

        worked_mass_prev = worked_mass;
        worked_volume_prev = worked_volume;

        //update the machine-relations info of the olv route points (the relations of the harvested route-points must be updated externally, e.g. by the multi-olv-planner)
        RoutePoint::MachineRelationInfo mri_olv;
        mri_olv.machine_id = harvester_route.machine_id;
        mri_olv.route_id = harvester_route.route_id;
        mri_olv.routePointIndex = i;
        mri_olv.routePointType = harvester_rp.type;
        olv_rp.machineRelations.push_back(mri_olv);

        ret.push_back(olv_rp);
        if(finisheAtEnd){
            ret.back().type = RoutePoint::OVERLOADING_FINISH;
            return ret;
        }
    }

    const auto& rpLastHarv = harvester_route.route_points.at(info.end_index);
    const auto& rpLastOlv = ret.back();
    double distCompare = clearanceDist + 3;

    double last_dist = geometry::calc_dist( rpLastHarv, rpLastOlv );
    if(last_dist > distCompare){//add last route point
        RoutePoint olv_rp;
        olv_rp.type = RoutePoint::OVERLOADING_FINISH;
        olv_rp.point() = geometry::getPointInLineAtDist(rpLastHarv, rpLastOlv, distCompare);
        olv_rp.track_id = rpLastHarv.track_id;
        olv_rp.time_stamp = std::max(rpLastHarv.time_stamp, rpLastOlv.time_stamp);

        RoutePoint::MachineRelationInfo mri_olv;
        mri_olv.machine_id = harvester_route.machine_id;
        mri_olv.route_id = harvester_route.route_id;
        mri_olv.routePointIndex = info.end_index;
        mri_olv.routePointType = rpLastHarv.type;
        olv_rp.machineRelations.push_back(mri_olv);

        ret.push_back(olv_rp);
    }

    ret.back().type = RoutePoint::OVERLOADING_FINISH;
    ret.back().bunker_mass += rpLastHarv.worked_mass + worked_mass_prev;
    ret.back().bunker_volume += rpLastHarv.worked_volume + worked_volume_prev;

    return ret;
}

std::vector<RoutePoint> OLVPlan::followHarvesterNoLength(DirectedGraph::Graph &graph,
                                                         const OverloadInfo &info,
                                                         const Route &harvester_route,
                                                         const std::shared_ptr<Machine> harv,
                                                         double initial_bunker_mass,
                                                         double clearanceTime){
    std::vector<RoutePoint> ret;

    ret.reserve( std::max(0, info.end_index-info.start_index+1) );

    double initial_harv_mass = harvester_route.route_points.at( info.start_index ).worked_mass;

    for (int j = info.start_index; j <= info.end_index; ++j) {
        RoutePoint harvester_rp = harvester_route.route_points.at(j);
        RoutePoint olv_rp;
        if(j == info.start_index)
            olv_rp.type = RoutePoint::OVERLOADING_START;
        else if(j == info.end_index)
            olv_rp.type = RoutePoint::OVERLOADING_FINISH;
        else
            olv_rp.type = RoutePoint::OVERLOADING;
        olv_rp.point() = harvester_rp.point();
        olv_rp.time_stamp = harvester_rp.time_stamp + clearanceTime;
        olv_rp.track_id = harvester_rp.track_id;

        olv_rp.bunker_mass = harvester_rp.worked_mass - initial_harv_mass + initial_bunker_mass;
        //olv_rp.bunker_volume = harvester_rp.worked_volume - initial_harv_volume + initial_bunker_volume;

        //update the machine-relations info of the olv route points (the relations of the harvested route-points must be updated externally, e.g. by the multi-olv-planner)
        RoutePoint::MachineRelationInfo mri_olv;
        mri_olv.machine_id = harvester_route.machine_id;
        mri_olv.route_id = harvester_route.route_id;
        mri_olv.routePointIndex = j;
        mri_olv.routePointType = harvester_rp.type;
        olv_rp.machineRelations.push_back(mri_olv);

        ret.push_back(olv_rp);
    }

    return ret;

}


std::vector<RoutePoint> OLVPlan::followHarvesterAlongside(DirectedGraph::Graph &graph,
                                                          const OLVPlan::OverloadInfo &info,
                                                          const Route &harvester_route,
                                                          double initial_bunker_mass,
                                                          bool offset,
                                                          const std::shared_ptr<Machine> harv)
{
    std::vector<RoutePoint> ret( harvester_route.route_points.begin()+info.start_index, harvester_route.route_points.begin()+info.end_index+1 );

    if(ret.size() < 2){
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Not enough route points to compute a path");
        return {};
    }

    double distToHarv = m_olv.workingRadius();
    if (harv)
        distToHarv = harv->working_width;
    distToHarv = std::max(0.0, distToHarv);

    double prevHarvestedMass = -1;
    double prevHarvestedVol;
    double offsetMult = ( m_state.adjVertexInfo.second == DownloadSide::DS_RIGHT ? -1 : 1 );
    double angleMult = ( m_state.adjVertexInfo.second == DownloadSide::DS_RIGHT ? -1 : 1 );

    for(size_t i = 0 ; i < ret.size() ; ++i){
        size_t i2 = i + info.start_index;
        const auto harvester_rp = harvester_route.route_points.at(i2);
        auto& olv_rp = ret.at(i);

        olv_rp = RoutePoint();
        olv_rp.point() = harvester_rp.point();
        olv_rp.time_stamp = harvester_rp.time_stamp;
        olv_rp.track_id = harvester_rp.track_id;
        olv_rp.type = RoutePoint::OVERLOADING;


        double harvestedMass = -1;
        double harvestedVol = -1;

        if(harvester_rp.isOfTypeWorking()){
            harvestedMass = harvester_rp.worked_mass;
            harvestedVol = harvester_rp.worked_volume;
        }


        //update bunker mass/volume
        if(i == 0){
            olv_rp.bunker_mass = initial_bunker_mass;
            //olv_rp.bunker_volume = initial_bunker_volume;
        }
        else{
            double deltaMass = 0;
            double deltaVol = 0;
            if(harvestedMass >= 0 && prevHarvestedMass >= 0){
                deltaMass = harvestedMass - prevHarvestedMass;
                deltaVol = harvestedVol - prevHarvestedVol;
            }
            olv_rp.bunker_mass = ret.at(i-1).bunker_mass + deltaMass;
            olv_rp.bunker_volume = ret.at(i-1).bunker_volume + deltaVol;
        }

        if(harvestedMass >= 0){
            prevHarvestedMass = harvestedMass;
            prevHarvestedVol = harvestedVol;
        }
        else
            prevHarvestedMass = -1;


        //update the machine-relations info of the olv route points (the relations of the harvested route-points must be updated externally, e.g. by the multi-olv-planner)
        RoutePoint::MachineRelationInfo mri_olv;
        mri_olv.machine_id = harvester_route.machine_id;
        mri_olv.route_id = harvester_route.route_id;
        mri_olv.routePointIndex = i2;
        mri_olv.routePointType = harvester_rp.type;
        olv_rp.machineRelations.push_back(mri_olv);
    }

    if(offset){//offset points
        std::vector<Point> offsetPts;
        std::vector<Point> pts0 = RoutePoint::toPoints(ret);
        double offsetDist = offsetMult * distToHarv;
        if( arolib::geometry::offsetLinestring_boost( pts0,
                                    offsetPts,
                                    offsetDist,
                                    true)
                && !offsetPts.empty() ){

            if(ret.size() == offsetPts.size()){
                for(size_t j = 0 ; j < offsetPts.size() ; ++j)
                    ret.at(j).point() = offsetPts.at(j);
            }
            else if(ret.size() > offsetPts.size()){
                size_t j2 = 0;
                for(auto& p : offsetPts){
                    for( ; j2 < ret.size() ; ++j2){
                        if( std::fabs( arolib::geometry::calc_dist(ret.at(j2), p) - distToHarv ) > 0.001*distToHarv )
                            break;
                        ret.at(j2).point() = p;
                    }
                }
            }
            else{
                size_t k2 = 1;
                ret.front().point() = offsetPts.front();
                for(size_t j = 1 ; j+1 < ret.size() ; ++j){
                    auto& rp = ret.at(j);
                    std::vector<Point> seg;
                    size_t nextRpInd = j+1;

                    while(k2 < offsetPts.size()){
                        double dist1 = arolib::geometry::calc_dist( rp  , offsetPts.at(k2) );
                        bool valid = true;
                        for(size_t jj = j+1 ; jj < ret.size() ; ++jj ){
                            const auto& rp2 = ret.at(jj);
                            double dist2 = arolib::geometry::calc_dist( rp2 , offsetPts.at(k2) );
                            if( dist1 > dist2 ){
                                nextRpInd = jj;
                                valid = false;
                                break;
                            }
                        }
                        if(!valid)
                            break;

                        seg.emplace_back( offsetPts.at(k2) );
                        ++k2;
                    }


                    if(seg.empty()){//shouldn't happen except when a spike goes inwards
                        ret.erase( ret.begin() + j );
                        j--;
                        continue;
                    }

                    rp.point() = seg.front();
                    if(seg.size() == 1)
                        continue;

                    double finalTimestamp = ret.at(nextRpInd-1).time_stamp;
                    double deltaTime = finalTimestamp - ret.at(j-1).time_stamp;
                    if(seg.size() == 2){
                        double dist1 = arolib::geometry::calc_dist(ret.at(j-1), seg.front());
                        double dist2 = arolib::geometry::calc_dist(seg.front(), seg.back());
                        double distSeg = dist1 + dist2;
                        if(distSeg == 0)
                            continue;
                        auto extraRP1 = rp;
                        extraRP1.time_stamp = finalTimestamp;
                        extraRP1.point() = seg.back();
                        rp.time_stamp -= ( dist2 * deltaTime / distSeg );
                        ret.insert( ret.begin()+j+1, extraRP1 );
                        ++j;
                        continue;
                    }

                    auto extraRP1 = rp;
                    auto extraRP2 = rp;
                    extraRP1.point() = seg.at( seg.size() * 0.5 );
                    extraRP2.point() = seg.back();

                    double dist1 = arolib::geometry::calc_dist(ret.at(j-1), seg.front());
                    double dist2 = arolib::geometry::calc_dist(seg.front(), extraRP1);
                    double dist3 = arolib::geometry::calc_dist(extraRP1, seg.back());
                    double distSeg = dist1 + dist2 + dist3;
                    if(distSeg == 0)
                        continue;

                    rp.time_stamp = finalTimestamp - ( (dist2+dist3) * deltaTime / distSeg);
                    extraRP1.time_stamp = finalTimestamp - ( dist3 * deltaTime / distSeg);
                    extraRP2.time_stamp = finalTimestamp;

                    ret.insert( ret.begin()+j+1, extraRP1 );
                    ret.insert( ret.begin()+j+2, extraRP2 );
                    j+=2;
                }
                ret.back().point() = offsetPts.back();
            }
        }
        else{
            for(size_t i = 0 ; i < ret.size() ; ++i){
                size_t i2 = i + info.start_index;
                const auto harvester_rp = harvester_route.route_points.at(i2);
                auto& olv_rp = ret.at(i);
                if(i+1 < ret.size())
                    olv_rp.point() = arolib::geometry::rotate(harvester_rp, harvester_route.route_points.at(i2+1), angleMult*M_PI_2);
                else
                    olv_rp.point() = arolib::geometry::rotate(harvester_rp, harvester_route.route_points.at(i2-1), -angleMult*M_PI_2);
                arolib::geometry::setVectorLength(harvester_rp.point(), olv_rp.point(), distToHarv);
            }
        }
    }
    else{//translate points
        //@todo check what happens when the harv route has repated points

        size_t i1 = info.start_index;
        size_t i2 = info.end_index;
        Point vec1 = arolib::geometry::rotate(harvester_route.route_points.at(i1), harvester_route.route_points.at(i1+1), angleMult*M_PI_2);
        Point vec2;

        double angleMult2 = angleMult;
        if(i2+1 < harvester_route.route_points.size()){
            if( std::fabs( arolib::geometry::get_angle(harvester_route.route_points.at(i1), harvester_route.route_points.at(i1+1),
                                     harvester_route.route_points.at(i2), harvester_route.route_points.at(i2+1)) )
                    > M_PI_2)//change direction
                angleMult2 *= -1;
            vec2 = arolib::geometry::rotate(harvester_route.route_points.at(i2), harvester_route.route_points.at(i2+1), angleMult2*M_PI_2);
        }
        else{
            if( std::fabs( arolib::geometry::get_angle(harvester_route.route_points.at(i1), harvester_route.route_points.at(i1+1),
                                     harvester_route.route_points.at(i2-1), harvester_route.route_points.at(i2)) )
                    > M_PI_2)//change direction
                angleMult2 *= -1;
            vec2 = arolib::geometry::rotate(harvester_route.route_points.at(i2), harvester_route.route_points.at(i2-1), -angleMult2*M_PI_2);
        }
        vec1 = vec1 - harvester_route.route_points.at(i1).point();
        vec2 = vec2 - harvester_route.route_points.at(i2).point();

        arolib::geometry::setVectorLength(vec1, distToHarv);
        arolib::geometry::setVectorLength(vec2, distToHarv);

        auto segmentPoints = RoutePoint::toPoints(ret);
        double segmentLength = arolib::geometry::getGeometryLength(segmentPoints);

        ret.front().point() = ret.front().point() + vec1;
        ret.back().point() = ret.back().point() + vec2;

        double currentLength = 0;
        if(segmentLength > 0){
            for(size_t i = 1 ; i+1 < ret.size() ; ++i){
                i1 = i + info.start_index;
                currentLength += arolib::geometry::calc_dist( harvester_route.route_points.at(i1), harvester_route.route_points.at(i1-1) );
                double mult = currentLength / segmentLength;
                Point vec1_tmp = vec1;
                Point vec2_tmp = vec2;
                arolib::geometry::setVectorLength(vec1_tmp, distToHarv * (1-mult));
                arolib::geometry::setVectorLength(vec2_tmp, distToHarv * mult);
                ret.at(i).point() =  ret.at(i).point() + vec1_tmp + vec2_tmp;
            }
        }

    }

    return ret;

}

void OLVPlan::updateFromRouteSegment(DirectedGraph::Graph &graph, const std::vector<RoutePoint> route_points, bool addCosts, const std::shared_ptr<Machine> harv, size_t ind_from, int ind_to)
{
    if(ind_to < 0)
        ind_to = route_points.size()-1;

    ind_to = std::min( ind_to, (int)route_points.size()-1 );

    if(ind_from > ind_to || route_points.empty())
        return;

    if(m_settings.collisionAvoidanceOption != Astar::WITHOUT_COLLISION_AVOIDANCE)//update visit periods
        updateVisitPeriods(graph,
                           m_olv,
                           route_points,
                           ind_from,
                           ind_to);

    double olvWidth = m_olv.width;
    if (olvWidth < 1e-6){//invalid radius --> use radius of harvester
        if( harv )
            olvWidth = harv->width;
        else
            olvWidth = 1;//ok, use 1 m
    }

    auto proximalVts = getVerticesUnderRouteSegment(graph, route_points, olvWidth, ind_from, ind_to);


    //check what edges connected to the proximal vetices overlap with the route (sub) segments. add overruns and update cost
    std::set<DirectedGraph::edge_t> edges;
    for(auto& vt : proximalVts){
        auto out_edges = boost::out_edges(vt, graph);
        for(;out_edges.first != out_edges.second; out_edges.first++)
            edges.insert(*out_edges.first);
        auto in_edges = boost::in_edges(vt, graph);
        for(;in_edges.first != in_edges.second; in_edges.first++)
            edges.insert(*in_edges.first);
    }


    for(size_t i = ind_from ; i < ind_to ; ++i){
        const auto& rp0 = route_points.at(i);
        const auto& rp1 = route_points.at(i+1);
        Polygon linePoly = arolib::geometry::createRectangleFromLine(rp0, rp1, olvWidth);

        double dist_intersection_all = 0;
        std::vector< std::pair< double, std::vector< DirectedGraph::overroll_property > > > overruns_all;
        std::set<DirectedGraph::edge_t> revEdges;

        for(auto& edge : edges){
            if(revEdges.find(edge) != revEdges.end())//rev edge info added already
                continue;

            DirectedGraph::edge_property edge_prop = graph[edge];

            if(edge_prop.bidirectional)
                revEdges.insert(edge_prop.revEdge);

            DirectedGraph::vertex_t vt0 = source(edge, graph);
            DirectedGraph::vertex_t vt1 = target(edge, graph);

            const RoutePoint& vt0_rp = graph[vt0].route_point;
            const RoutePoint& vt1_rp = graph[vt1].route_point;

            double edge_dist = arolib::geometry::calc_dist(vt0_rp, vt1_rp);
            if(edge_dist < 1e-5)
                continue;

            auto intersection = arolib::geometry::get_intersection(vt0_rp, vt1_rp, linePoly);
            if (intersection.size() < 2)
                continue;


            double dist_intersection = arolib::geometry::calc_dist( intersection.front(), intersection.back() );
            double mult = dist_intersection / edge_dist;

            if(mult < 1e-3)
                continue;

            dist_intersection_all += dist_intersection;
            overruns_all.emplace_back( std::make_pair(dist_intersection, edge_prop.overruns) );

            DirectedGraph::overroll_property overroll;
            overroll.machine_id = m_olv.id;
            overroll.duration = mult * (route_points.at(i+1).time_stamp - route_points.at(i).time_stamp);
            overroll.weight = std::max( 0.0, mult * 0.5 * ( route_points.at(i+1).bunker_mass + route_points.at(i).bunker_mass ) ) + m_olv.weight;

            graph.addOverrun(vt0, vt1, overroll);

        }

        if(addCosts){

            std::vector< DirectedGraph::overroll_property > overruns;//estimated overruns in the pseudo edge
            if(dist_intersection_all > 0){
                for( auto& overrun_pair : overruns_all){
                    double mult = overrun_pair.first / dist_intersection_all;
                    for(auto& overrun : overrun_pair.second){
                        overruns.emplace_back(overrun);
                        overruns.back().duration *= mult;
                        overruns.back().weight *= mult;
                    }
                }
            }
            double cost = m_edgeCostCalculator->calcCost(m_olv,
                                                         route_points.at(i),
                                                         route_points.at(i+1),
                                                         route_points.at(i+1).time_stamp - route_points.at(i).time_stamp,
                                                         0,
                                                         route_points.at(i+1).bunker_mass - route_points.at(i).bunker_mass,
                                                         overruns);
            m_state.plan_cost += cost;

        }

    }

}

void OLVPlan::removeSpikePointsFromTransitRoute()
{
    if(!m_state.route_points_prev.empty()){
        while(m_state.route.route_points.size() > 1){
            auto ptsTmp = Point::toPoints(m_state.route.route_points);
            geometry::unsample_linestring(ptsTmp);
            const auto& rpPrev = m_state.route_points_prev.back();
            const auto& rp0 = ptsTmp.front();
            const auto& rp1 = ptsTmp.at(1);

            if( geometry::getLocationInLine(rp0, rp1, rpPrev) != 0 )
                break;

            pop_front(m_state.route.route_points);
        }
    }

}

std::set<DirectedGraph::vertex_t> OLVPlan::getVerticesUnderRouteSegment(const DirectedGraph::Graph &graph, const std::vector<RoutePoint> route_points, double width, size_t ind_from, int ind_to)
{
    std::set<DirectedGraph::vertex_t> ret;

    if(ind_to < 0)
        ind_to = route_points.size()-1;

    ind_to = std::min( ind_to, (int)route_points.size()-1 );

    if(ind_from > ind_to || route_points.empty())
        return ret;

    for(size_t i = ind_from ; i < ind_to ; ++i){
        auto vts_tmp = graph.getVerticesInRectangle( route_points.at(i), route_points.at(i+1), width );
        ret.insert( vts_tmp.begin(), vts_tmp.end() );
    }

    return ret;
}



std::vector<std::pair<DirectedGraph::vertex_t, OLVPlan::DownloadSide> > OLVPlan::getAdjacentVertices(DirectedGraph::Graph &graph,
                                                                                                     const Route &harvester_route,
                                                                                                     int harvester_rp_index,
                                                                                                     const Machine &harv)
{
    std::vector<std::pair<DirectedGraph::vertex_t, DownloadSide> > ret;

    if( (harv.unload_sides & Machine::WorkingSide_BACK) == 0
            || graph.outermostTrackIds_HL().find(harvester_route.route_points.at(harvester_rp_index).track_id) == graph.outermostTrackIds_HL().end() )
        addAdjacentVertices_sides(graph, harvester_route, harvester_rp_index, harv, ret);
    if(harv.unload_sides & Machine::WorkingSide_BACK)
        addAdjacentVertices_back(graph, harvester_route, harvester_rp_index, harv, ret);

    return ret;
}

void OLVPlan::addAdjacentVertices_back(DirectedGraph::Graph &graph,
                                       const Route &harvester_route,
                                       int harvester_rp_index,
                                       const Machine &harv,
                                       std::vector<std::pair<DirectedGraph::vertex_t, OLVPlan::DownloadSide> > &adjacent_vts)
{
    const auto& points = harvester_route.route_points;
    const auto switchingPoint = points.at(harvester_rp_index);

    int indPrevPoint = -1;
    if(harvester_rp_index > 0){
        for(int i = harvester_rp_index-1 ; i >= 0; --i){
            if( geometry::calc_dist( points.at(harvester_rp_index), points.at(i) ) > 1e-3 ){
                indPrevPoint = i;
                break;
            }
        }
    }

    if(indPrevPoint > 0){
        const auto prevPoint = points.at(indPrevPoint);
        if(prevPoint.isOfTypeWorking()){
            auto it_rp = graph.routepoint_vertex_map().find( prevPoint );
            if(it_rp != graph.routepoint_vertex_map().end()){
                adjacent_vts.emplace_back( std::make_pair(it_rp->second, DownloadSide::DS_BEHIND) );
                return;
            }
        }
        else if( Track::isInfieldTrack(switchingPoint.track_id)
                 && switchingPoint.track_id > 0
                 && (prevPoint.type == RoutePoint::HEADLAND
                     || prevPoint.type == RoutePoint::TRANSIT)//@todo: should the type really be included here?
                ){//search for the closest vertex
            double rad = 1.5 * harv.workingRadius();
            bool found = false;
            double min_dist = std::numeric_limits<double>::max();
            DirectedGraph::vertex_t nearest_vertex;

            //try searching by radius
            graph.getVerticesInRadius(prevPoint, rad, [&min_dist, &nearest_vertex, &found, &prevPoint](const DirectedGraph::vertex_t& vt, const DirectedGraph::vertex_property& vertex_prop)->bool{
                if( vertex_prop.route_point.type == RoutePoint::HEADLAND
                        || Track::isHeadlandTrack( vertex_prop.route_point.track_id ) ){

                    double dist = arolib::geometry::calc_dist(prevPoint, vertex_prop.route_point);
                    if (dist < min_dist) {
                        min_dist = dist;
                        nearest_vertex = vt;
                        found = true;
                    }
                }
                return false;//we dont really need the list of vertices
            });

            if(found){
                adjacent_vts.emplace_back( std::make_pair(nearest_vertex, DownloadSide::DS_BEHIND) );
                return;
            }
        }
    }

    if(points.size() > 1){
        Point pRef;
        double angMin, angMax;
        if(indPrevPoint > 0){
            pRef = points.at(indPrevPoint).point();
            angMin = 0;
            angMax = deg2rad(15);
        }
        else {
            int harvester_next_rp_index = geometry::getNextNonRepeatedPointIndex(harvester_route.route_points, harvester_rp_index, -1, -1);
            pRef = points.at( harvester_next_rp_index ).point();
            angMin = deg2rad(165);
            angMax = deg2rad(180);
        }


        double rad = 1.5 * harv.workingRadius();
        double minAllowedDist = 0.5 * harv.workingRadius();
        double minDist = std::numeric_limits<double>::max();
        auto vts = graph.getVerticesInRadius(switchingPoint, rad,
                                             [switchingPoint, pRef, minAllowedDist, minDist, angMin, angMax](const DirectedGraph::vertex_t& vt, const DirectedGraph::vertex_property& vertex_prop)->bool{
            double dist = geometry::calc_dist(switchingPoint, vertex_prop.route_point);
            if( dist < minAllowedDist || dist > minDist || vertex_prop.route_point.track_id < 0 )
                return false;
            double angle = geometry::get_angle( vertex_prop.route_point, switchingPoint, pRef );
            if(angle < angMin || angle > angMax)
                return false;
            return true;
        });

        if(!vts.empty()){
            for(auto& vt : vts)
                adjacent_vts.emplace_back( std::make_pair(vt, DownloadSide::DS_BEHIND) );
            return;
        }
    }


    auto it_rp = graph.routepoint_vertex_map().find( switchingPoint );
    if(it_rp != graph.routepoint_vertex_map().end())
        adjacent_vts.emplace_back( std::make_pair(it_rp->second, DownloadSide::DS_SWITCHING_POINT_BEHIND) );
    return;

}

void OLVPlan::addAdjacentVertices_sides(DirectedGraph::Graph &graph,
                                       const Route &harvester_route,
                                       int harvester_rp_index,
                                       const Machine &harv,
                                       std::vector<std::pair<DirectedGraph::vertex_t, OLVPlan::DownloadSide> > &adjacent_vts)
{

    if( (harv.unload_sides & Machine::WorkingSide_RIGHT) == 0
            && (harv.unload_sides & Machine::WorkingSide_LEFT) == 0)
        return;

    const auto& points = harvester_route.route_points;
    const auto switchingPoint = points.at(harvester_rp_index);

    int harvester_next_rp_index = geometry::getNextNonRepeatedPointIndex(harvester_route.route_points, harvester_rp_index, -1, -1);

    if(harvester_next_rp_index < 0)
        return;

    const auto nextPoint = points.at(harvester_next_rp_index);

    Point pRight = arolib::geometry::rotate(switchingPoint, nextPoint, -M_PI_2);
    Point pLeft = arolib::geometry::rotate(switchingPoint, nextPoint, M_PI_2);

    auto it_rp = graph.routepoint_vertex_map().find( switchingPoint );
    if(it_rp == graph.routepoint_vertex_map().end())
        return;

    auto switching_vt = it_rp->second;

    std::set<DirectedGraph::vertex_t> adjacentsTmp = getAdjacentVerticesForSideOverloading(graph, harvester_route, switching_vt, harvester_rp_index, false);//get the closest vertices where a connecting edge exists

    for(auto& adj : adjacentsTmp){
        DirectedGraph::vertex_property adj_prop = graph[adj];
        const auto adjPoint = adj_prop.route_point;

        DownloadSide dl_side = DownloadSide::DS_LEFT;
        if( arolib::geometry::calc_dist(adjPoint, pRight) < arolib::geometry::calc_dist(adjPoint, pLeft) )
            dl_side = DownloadSide::DS_RIGHT;

        if(adjPoint.time_stamp > -1e-5
                && adjPoint.time_stamp > switchingPoint.time_stamp
                /*&& adj_prop.harvester_id == harvester_route.machine_id*/)
            continue;

        if(graph.boundary_vts().find(adj) != graph.boundary_vts().end())
            continue;

        if( Track::isInfieldTrack(switchingPoint.track_id)
                && graph.extremaTrackIds_IF().find(switchingPoint.track_id) == graph.extremaTrackIds_IF().end()
                && ( Track::isHeadlandTrack(adjPoint.track_id) || adjPoint.track_id < 0 ) ){//if the adjacent vt is in the headland it could connect to a track that hasn't been harvested

            std::vector<DirectedGraph::vertex_t> vts1;
            DirectedGraph::vertex_property v_prop_1;
            auto it_adj = graph.adjacentTrackIds_IF().find(switchingPoint.track_id);
            if(it_adj != graph.adjacentTrackIds_IF().end()){//find the closest RP from the corresponding to the next/previous track to see if it is valid
                const auto& adjIds = it_adj->second;
                for(auto adjId : adjIds){
                    if( searchNearestVerticesOnTrack(graph, switching_vt, adjId, vts1, 1) && !vts1.empty() ){
                        v_prop_1 = graph[vts1.front()];
                        double ang = arolib::geometry::get_angle(switchingPoint, nextPoint, v_prop_1.route_point);
                        if( (dl_side == DownloadSide::DS_RIGHT && ang < 0)
                                || (dl_side == DownloadSide::DS_LEFT && ang > 0) )
                            vts1.clear();
                        else
                            break;
                    }
                    else
                        vts1.clear();
                }
            }

            if(vts1.empty())
                continue;

            if(v_prop_1.route_point.time_stamp > -1e-5
                    && v_prop_1.route_point.time_stamp > switchingPoint.time_stamp
                    /*&& adj_prop.harvester_id == harvester_route.machine_id*/)
                continue;//not valid

        }

        auto isValidAdjVt = [&](double ang, const Point& pRef)->bool{
            return ang < 45 &&
                    ( !adjPoint.isOfTypeWorking()
                      || ( Track::isInfieldTrack(switchingPoint.track_id)
                           && adjPoint.isOfType({RoutePoint::TRACK_START, RoutePoint::TRACK_END})
                           && std::abs(switchingPoint.track_id - adjPoint.track_id) == 1 )
                      || ( Track::isInfieldTrack(switchingPoint.track_id)
                           && Track::isHeadlandTrack(adjPoint.track_id) )
                      || ang < 30 );
        };

        if( dl_side == DownloadSide::DS_RIGHT ){
            if( (harv.unload_sides & Machine::WorkingSide_RIGHT )> 0){
                double ang = std::fabs( arolib::geometry::get_angle(pRight, switchingPoint, adjPoint, true) );
                if ( isValidAdjVt(ang, pRight) )
                    adjacent_vts.emplace_back( std::make_pair(adj, dl_side) );
            }
        }

        else{
            if( (harv.unload_sides & Machine::WorkingSide_LEFT) > 0){
                double ang = std::fabs( arolib::geometry::get_angle(pLeft, switchingPoint, adjPoint, true) );
                if ( isValidAdjVt(ang, pLeft) )
                    adjacent_vts.emplace_back( std::make_pair(adj, dl_side) );
            }
        }

    }

}

std::set<DirectedGraph::vertex_t> OLVPlan::addSuccCheckerForReverseDriving_start(std::vector<std::shared_ptr<const Astar::ISuccesorChecker> > &succCheckers, const DirectedGraph::Graph &graph, double angTH){

    angTH = std::fabs(angTH);

    std::set<DirectedGraph::vertex_t> excludeVts;
    const auto& route_points = m_state.route.route_points.empty() ? m_state.route_points_prev : m_state.route.route_points;
    if(route_points.size() < 2)
        return excludeVts;

    double distTH = 2 * std::max({m_olv.getTurningRadius(), m_olv.working_width, m_olv.width});
    auto connVts = getConnectedVertices(graph, m_state.olv_last_vt, false);

    size_t indLastRev = 0;
    for(; indLastRev+1 < route_points.size() ; ++indLastRev){
        if( r_at(route_points, indLastRev).point() != r_at(route_points, indLastRev+1).point() )
            break;
    }

    const RoutePoint& rp0 = r_at(route_points, indLastRev);
    for(size_t i = indLastRev + 1 ; i < route_points.size() ; ++i){
        const auto& rp = r_at(route_points, i);
        if(i > indLastRev + 1 && geometry::calc_dist(rp0, rp) > distTH)
            break;

        if(rp.point() == rp0.point())
            continue;

        for(auto& vt : connVts){
            const RoutePoint& rpVt = graph[vt].route_point;
            if(rpVt.isFieldAccess() || rpVt.type == RoutePoint::RESOURCE_POINT)
                continue;
            double ang = geometry::get_angle(rp, rp0, rpVt, true);
            if(std::fabs(ang) < angTH)
                excludeVts.insert(vt);
        }

        if( !excludeVts.empty() )
            break;
    }

    if(excludeVts.empty())
        return excludeVts;

    if(excludeVts.size() == connVts.size() && angTH > 20){
        return addSuccCheckerForReverseDriving_start(succCheckers, graph, 0.75 * angTH);
    }

    succCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_VertexExcludeSet>(excludeVts, [](const Astar::ISuccesorChecker::IsSuccessorValidParams& params)->bool{
        return params.vt_from != params.vt_start; //only exclude these vertices if they are trying to be accessed from the starting vt (avoid reverse driving)
    }) );
    return excludeVts;
}

std::set<DirectedGraph::edge_t> OLVPlan::replaceSuccCheckerForReverseDriving_OLStart(std::vector<std::shared_ptr<const Astar::ISuccesorChecker> > &succCheckers,
                                                                                     size_t indSC,
                                                                                     const DirectedGraph::Graph &graph,
                                                                                     const Route &harvester_route,
                                                                                     const OverloadInfo &overload_info,
                                                                                     const DirectedGraph::vertex_t& vt_goal,
                                                                                     const DirectedGraph::vertex_property& vt_prop_goal,
                                                                                     double angTH)
{
    std::set<DirectedGraph::edge_t> excludeEdges;
    if(overload_info.start_index+1 >= harvester_route.route_points.size())
        return excludeEdges;
    Point harvRP = harvester_route.route_points.at(overload_info.start_index).point();


    int harvester_next_rp_index = geometry::getNextNonRepeatedPointIndex(harvester_route.route_points, overload_info.start_index, -1, 1e-3);
    if(harvester_next_rp_index < 0)
        return excludeEdges;

    Point harvRPNext = harvester_route.route_points.at(harvester_next_rp_index).point();

    for(auto edges = boost::in_edges(vt_goal, graph); edges.first != edges.second; edges.first++){
        DirectedGraph::vertex_t vt = source(*edges.first, graph);
        const RoutePoint& rpVt = graph[vt].route_point;
        if(rpVt.isFieldAccess() || rpVt.type == RoutePoint::RESOURCE_POINT)
            continue;
        double ang = geometry::get_angle(harvRP, harvRPNext, vt_prop_goal.route_point, rpVt, true);
        if(std::fabs(ang) < angTH)
            excludeEdges.insert(*edges.first);
    }

    if(excludeEdges.empty())
        return excludeEdges;

    if(indSC >= succCheckers.size())
        succCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_EdgeExcludeSet>(excludeEdges) );
    else
        succCheckers.at(indSC) = std::make_shared<AstarSuccessorChecker_EdgeExcludeSet>(excludeEdges);

    return excludeEdges;
}

void OLVPlan::addSuccCheckerForPrevOLEnd(std::vector<std::shared_ptr<const Astar::ISuccesorChecker> > &succCheckers, const DirectedGraph::Graph &graph, const Route &harvester_route)
{
    if(m_prev_overloading_end_index < 0 || m_prev_overloading_end_index+1 >= harvester_route.route_points.size())
        return;

    const auto& rp_next = harvester_route.route_points.at(m_prev_overloading_end_index+1);
    if(!rp_next.isOfTypeWorking())
        return;

    DirectedGraph::vertex_t vt_ol_end;
    if(!graph.getClosestVertexInRadius(harvester_route.route_points.at(m_prev_overloading_end_index), 1e-3, vt_ol_end, [](const DirectedGraph::vertex_t&, const DirectedGraph::vertex_property& vertex_prop)->bool{
          return vertex_prop.route_point.time_stamp > -1e-3;
    })){
        return;
    }

    succCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_prevOLEnd>(rp_next.time_stamp, vt_ol_end) );

}

void OLVPlan::updateExcludeSet(const DirectedGraph::Graph &graph, std::set<DirectedGraph::vertex_t> &exclude) const
{
    std::set<DirectedGraph::vertex_t> exclude_plus;
    for(auto& e : exclude){
        DirectedGraph::vertex_property v_prop = graph[e];

        //add the start/end_track points in HL corresponding the the end/start_track exclude vertices (because they are "redundant")
        if(Track::isHeadlandTrack(v_prop.route_point.track_id)
                && ( v_prop.route_point.type == RoutePoint::TRACK_START || v_prop.route_point.type == RoutePoint::TRACK_END )){
            auto excludeTmp = graph.getVerticesInRadius( v_prop.route_point, 1e-1,
                                                         [&v_prop, e](const DirectedGraph::vertex_t& vt, const DirectedGraph::vertex_property& vertex_prop)->bool{
                if(vertex_prop.route_point.track_id != v_prop.route_point.track_id
                        || ( vertex_prop.route_point.type != RoutePoint::TRACK_START && vertex_prop.route_point.type != RoutePoint::TRACK_END )
                        || e == vt)
                    return false;
                return true;
            });

            exclude_plus.insert(excludeTmp.begin(), excludeTmp.end());
        }

    }
    exclude.insert(exclude_plus.begin(), exclude_plus.end());
}

double OLVPlan::getOlvWaitingTimeToFollowHarv(const Route &harvester_route, int harvester_rp_index, double clearanceDist)
{
    const auto rp0 = harvester_route.route_points.at(harvester_rp_index);

    clearanceDist = std::max(0.0, clearanceDist);

    double distRem = clearanceDist;

    for(size_t i = harvester_rp_index; i+1 < harvester_route.route_points.size(); ++i){
        const auto& rp = harvester_route.route_points.at(i+1);
        double dist = arolib::geometry::calc_dist(rp0, rp);
        if(dist >= clearanceDist){
            double time0 = harvester_route.route_points.at(i).time_stamp - rp0.time_stamp;
            double time1 = rp.time_stamp - harvester_route.route_points.at(i).time_stamp;
            double dist1 = arolib::geometry::calc_dist(harvester_route.route_points.at(i), rp);
            if(time1 > 1e-5 && dist1 > 1e-5){
                double speed_inv = time1/dist;
                time1 = distRem * speed_inv;
            }
            return time0+time1;
        }
        distRem = clearanceDist - dist;
    }
    return m_settings.clearanceTime;
}

double OLVPlan::getClearanceDistBehindHarv(const Machine &harv)
{
    double ret = harv.length;
    if(ret <= 1e-9)
        ret = std::max({harv.working_width, harv.width, harv.turning_radius});
    if(ret <= 0)
        return -1;
    return ret + std::min(2.0, 0.1 * ret);
}

bool OLVPlan::updateCurrentVertexWithClosest(DirectedGraph::Graph &graph, const Route &harvester_route)
{
    const auto& rp = m_state.lastRoutePoint;
    DirectedGraph::vertex_property::GraphLocation graphLoc = DirectedGraph::vertex_property::INFIELD;

    if(m_state.prev_overloading_end_index > 0 && m_state.prev_overloading_end_index < harvester_route.route_points.size()){
        if(Track::isHeadlandTrack(harvester_route.route_points.at(m_state.prev_overloading_end_index).track_id))
            graphLoc = DirectedGraph::vertex_property::HEADLAND;
    }
    else{
        if(Track::isHeadlandTrack(rp.track_id))
            graphLoc = DirectedGraph::vertex_property::HEADLAND;
    }

    const auto& route_points = m_state.route.route_points.empty() ? m_state.route_points_prev : m_state.route.route_points;

    //get proximal vertices
    double rad = m_olv.workingRadius();
    if (rad < 1e-6)
        rad = 1;

    DirectedGraph::Graph::VertexFilterFct isVtValid = [this, graphLoc, &route_points, &rad](const DirectedGraph::vertex_t &, const DirectedGraph::vertex_property &v_prop)->bool{
        if(v_prop.route_point.time_stamp >= 0 && m_state.olv_time < v_prop.route_point.time_stamp )//not valid
            return false;
        if(v_prop.graph_location != DirectedGraph::vertex_property::DEFAULT && v_prop.graph_location != graphLoc )//not valid
            return false;
        if(v_prop.route_point.type == RoutePoint::RESOURCE_POINT)
            return false;
        if(route_points.size() > 1 && geometry::calc_dist(route_points.back(), v_prop.route_point) > 0.05 * rad  ){
            double ang = std::fabs( geometry::get_angle( r_at(route_points, 1), route_points.back(), v_prop.route_point, true ) );
            if(ang < 5)//avoid reverse driving
                return false;
        }
        return true;
    };

    bool found = false;
    double min_dist = std::numeric_limits<double>::max();
    auto proximalVts = graph.getVerticesInRadius(rp, rad, isVtValid);

    for(auto& vt : proximalVts){
        DirectedGraph::vertex_property &v_prop = graph[vt];
        double dist = arolib::geometry::calc_dist( rp, v_prop.route_point );
        if(min_dist > dist){
            min_dist = dist;
            m_state.olv_last_vt = vt;
            found = true;
        }
    }

    if(!found){//check all vertices
        auto closestVt = getClosestValidVertices(graph, rp, isVtValid);
        if(!closestVt.empty()){
            m_state.olv_last_vt = closestVt.front();
            found = true;
            DirectedGraph::vertex_property &v_prop = graph[m_state.olv_last_vt];
            min_dist = arolib::geometry::calc_dist( rp, v_prop.route_point );
        }
    }

    if(!found){
        logger().printError(__FUNCTION__, "No valid vertex found in the proximity");
        return false;
    }

    //update the current olv time
    m_state.olv_time += ( min_dist / m_olv.calcSpeed(rp.bunker_mass) );
    return true;
}

std::set<DirectedGraph::vertex_t> OLVPlan::getAdjacentVerticesForSideOverloading(const DirectedGraph::Graph &graph,
                                                                                    const Route &harvester_route,
                                                                                    const DirectedGraph::vertex_t switching_vt,
                                                                                    int harvester_rp_index,
                                                                                    bool allowBoundaryVtsIfNeeded) const
{
    std::set<DirectedGraph::vertex_t> ret;
    if(harvester_rp_index >= 0 && harvester_rp_index+1 >= harvester_route.route_points.size())
        return ret;

    double tolerance = 0.01 * m_olv.workingRadius();

    auto connectedVertices = getConnectedVertices(graph, switching_vt, true);

    const RoutePoint& vt_rp = graph[switching_vt].route_point;

    auto validityFctAdjTracks = [&vt_rp, &harvester_route, harvester_rp_index](const DirectedGraph::vertex_t&, const DirectedGraph::vertex_property& vertex_prop){
        if(harvester_rp_index < 0)
            return true;

        int harvester_next_rp_index = geometry::getNextNonRepeatedPointIndex(harvester_route.route_points, harvester_rp_index, -1, -1);
        if(harvester_next_rp_index < 0)
            return true;

        double ang = std::fabs( geometry::get_angle(harvester_route.route_points.at(harvester_next_rp_index), vt_rp, vertex_prop.route_point) );
        return( ang > deg2rad(75) && ang < deg2rad(105) );
    };

    std::set<int> adjHL, adjIF;
    auto adj_it = graph.adjacentTrackIds_HL().find(vt_rp.track_id);
    if(adj_it != graph.adjacentTrackIds_HL().end())
        adjHL = adj_it->second;
    adj_it = graph.adjacentTrackIds_IF().find(vt_rp.track_id);
    if(adj_it != graph.adjacentTrackIds_IF().end())
        adjIF = adj_it->second;

    int sides_count = 0;

    for(auto& adj_id : adjHL){
        std::vector<DirectedGraph::vertex_t> vts;
        if( searchNearestVerticesOnTrack(graph, connectedVertices, switching_vt, adj_id, vts, 0, tolerance, validityFctAdjTracks) ){
            ret.insert( vts.begin(), vts.end() );
            sides_count += !vts.empty();
        }
    }

    for(auto& adj_id : adjIF){
        std::vector<DirectedGraph::vertex_t> vts;
        if( searchNearestVerticesOnTrack(graph, connectedVertices, switching_vt, adj_id, vts, 0, tolerance, validityFctAdjTracks) ){
            ret.insert( vts.begin(), vts.end() );
            sides_count += !vts.empty();
        }
    }

    if(ret.empty())
        logger().printWarning("No adjacent points for point in track " + std::to_string(vt_rp.track_id) + " cound be found in the adjacent track vertices");


    if(graph.extremaTrackIds_IF().find(vt_rp.track_id) != graph.extremaTrackIds_IF().end()){//search also in the headland because the point is in the first or last track

        std::vector<DirectedGraph::vertex_t> vts;
        if( searchNearestVertices(graph, connectedVertices, switching_vt, vts, 0, tolerance,
                                  [](const DirectedGraph::vertex_t&, const DirectedGraph::vertex_property& vertex_prop)->bool{
                                  return Track::isHeadlandTrack(vertex_prop.route_point.track_id);
    }) ){
            ret.insert( vts.begin(), vts.end() );
            sides_count += !vts.empty();
        }
    }

    if(sides_count < 2 && allowBoundaryVtsIfNeeded){//search also in the boundary
        std::vector<DirectedGraph::vertex_t> vts;
        if( searchNearestVertices(graph, connectedVertices, switching_vt, vts, 0, tolerance,
                                  [](const DirectedGraph::vertex_t&, const DirectedGraph::vertex_property& vertex_prop)->bool{
                                  return vertex_prop.route_point.track_id < 0
                                  && !vertex_prop.route_point.isOfType({RoutePoint::RESOURCE_POINT,
                                                                       RoutePoint::FIELD_ENTRY,
                                                                       RoutePoint::FIELD_EXIT,
                                                                       RoutePoint::INITIAL_POSITION})
                                  && !vertex_prop.route_point.isOfTypeWorking(true, true);
    }) ){
            ret.insert( vts.begin(), vts.end() );
        }
    }

    return ret;
}

std::string OLVPlan::getOutputFolder(const std::string &planType)
{
    ++m_countPlans;

    if(m_outputFolder.empty())
        return "";

    std::string folder = m_outputFolder;
    folder += ("P" + std::to_string(m_countPlans) + "-" + planType + "/");

    if(!io::create_directory(folder, true))
        return "";
    return folder;
}

OLVPlan::AstarSuccessorChecker_prevOLEnd::AstarSuccessorChecker_prevOLEnd(double timestamp_nextOLStart, const DirectedGraph::vertex_t &vt_prevOLEnd)
    : m_timestamp_nextOLStart(timestamp_nextOLStart), m_vt_prevOLEnd(vt_prevOLEnd)
{

}

double OLVPlan::AstarSuccessorChecker_prevOLEnd::getMinDurationAtEdge(const GetMinDurationAtEdgeParams &params) const
{
    if(params.vt_to != m_vt_prevOLEnd || params.vt_to_prop.route_point.time_stamp < -1e-3 || m_timestamp_nextOLStart < -1e-3 )
        return -1;


    double clearanceTime = std::max(0.0, params.clearance_time);
    return std::max(0.0, m_timestamp_nextOLStart)
            + clearanceTime
            - std::max(0.0, params.current_search_node.time);//minimum time that transversing the edge can take based on whether the route point was worked already (and when) or not (disregarding distance)

}

}
