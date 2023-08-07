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
 
#include "arolib/planning/roundtripplanner.hpp"
#include "arolib/planning/path_search/graphhelper.hpp"
#include "arolib/planning/path_search/astar.hpp"

namespace arolib{

bool RoundtripPlanner::PlannerSettings::parseFromStringMap(RoundtripPlanner::PlannerSettings &params, const std::map<std::string, std::string> &map, bool strict)
{
    return Astar::AStarSettings::parseFromStringMap(params, map, strict);
}

std::map<std::string, std::string> RoundtripPlanner::PlannerSettings::parseToStringMap(const RoundtripPlanner::PlannerSettings &params)
{
    return Astar::AStarSettings::parseToStringMap(params);
}

RoundtripPlanner::RoundtripPlanner(const Machine &machine,
                 const PlannerSettings & settings, std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                 const std::string &outputFolder,
                 LogLevel logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_settings(settings),
    m_machine(machine),
    m_edgeCostCalculator(edgeCostCalculator),
    m_outputFolder(outputFolder)
{
    if(std::isnan(m_state.plan_cost))
        logger().printOut(LogLevel::ERROR, "m_state.plan_cost is NAN!");

    if(!m_outputFolder.empty() && m_outputFolder.back() != '/')
        m_outputFolder += "/";

}

bool RoundtripPlanner::planTrip(const DirectedGraph::Graph &graph,
                                const Route &routeBase,
                                size_t rp_index,
                                size_t rp_ret_index,
                                const std::vector<DirectedGraph::vertex_t> &destinationVertices,
                                std::vector<std::shared_ptr<const Astar::ISuccesorChecker> > successorCheckers_toDest,
                                std::vector<std::shared_ptr<const Astar::ISuccesorChecker> > successorCheckers_toRoute,
                                bool allowReverseDriving,
                                std::function<RoutePoint (const RoutePoint &, const DirectedGraph::vertex_t &)> functAtDest)
{
    if(!m_edgeCostCalculator){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return false;
    }

    m_found_plan = false;
    if(!initState(graph, routeBase, rp_index, rp_ret_index)){
        logger().printError(__FUNCTION__, "Error initializing planner state");
        return false;
    }

    if(routeBase.route_points.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Base route is empty");
        return false;
    }

    if(rp_index >= routeBase.route_points.size()){
        logger().printOut(LogLevel::ERROR, "Invalid route-point index");
        return false;
    }

    if(destinationVertices.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "There are no destination vertices");
        return false;
    }

    //send to the best (intermediate) destination
    AstarPlan planToDestination;
    std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
    if( !planPathToDestination(planToDestination,
                               destinationVertices,
                               successorCheckers_toDest,
                               allowReverseDriving) ){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error calling planPathToDestination");
        return false;
    }
    m_state.planningDuration_toDest = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
    m_state.planCost_toDest = planToDestination.plan_cost_total;
    m_state.updateCost();
    m_state.addSubroute(planToDestination.route_points_);
    planToDestination.insertIntoGraph(m_state.graph);
    m_state.numCrossings += planToDestination.numCrossings;
    m_state.numCrossings_HL += planToDestination.numCrossings_HL;

    if(std::isnan(m_state.plan_cost))
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "plan_cost is NAN (planPathToDestination)!");


    if(!m_state.subRoutes.empty() && !m_state.subRoutes.back().route_points.empty()){//add info 'at (intermediate) destination'
        auto rpLast = m_state.subRoutes.back().route_points.back();
        auto rpAtDest = functAtDest(rpLast, m_state.destVt);
        m_state.subRoutes.back().route_points.emplace_back(rpAtDest);
        m_state.planCost_atDest += m_edgeCostCalculator->calcCost(m_machine,
                                                                  rpAtDest,
                                                                  rpLast,
                                                                  rpAtDest.time_stamp - rpLast.time_stamp,
                                                                  0,
                                                                  0.5 * (rpAtDest.bunker_mass + rpLast.bunker_mass),
                                                                  {});
        m_state.updateCost();
    }

    if(rp_ret_index < routeBase.route_points.size()){//the process is not over, go back to the route point
        AstarPlan planToRoute;
        time_start = std::chrono::steady_clock::now();
        if( !planPathToRoutePoint(planToRoute, successorCheckers_toRoute) ){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error calling planPathToRoutePoint");
            return false;
        }
        m_state.planningDuration_toRoute = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
        m_state.planCost_toRoute = planToRoute.plan_cost_total;
        m_state.updateCost();
        m_state.addSubroute(planToRoute.route_points_);
        planToRoute.insertIntoGraph(m_state.graph);
        m_state.numCrossings += planToRoute.numCrossings;
        m_state.numCrossings_HL += planToRoute.numCrossings_HL;

        if(std::isnan(m_state.plan_cost))
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "plan_cost is NAN (planPathToRoutePoint)!");

    }
    else if (!m_state.subRoutes.empty() && !m_state.subRoutes.back().route_points.empty()){
        RoutePoint p = m_state.subRoutes.back().route_points.back();
        m_state.addSubroute({p});
    }

    m_state.subRoutes.front().route_points.pop_back();//remove the extra point previously calculated with the functAtDest

    updateRoute();

    m_found_plan = true;

    logger().printOut(LogLevel::DEBUG, __FUNCTION__, 10, "Planning duration = ",
                      m_state.planningDuration_toDest, "[Route->RP]",
                      " + ", m_state.planningDuration_toRoute, "[RP->Route]",
                      " = ", m_state.planningDuration_toDest + m_state.planningDuration_toRoute, " seconds" );

    logger().printOut(LogLevel::DEBUG, __FUNCTION__, 10, "Planning cost = ",
                      m_state.planCost_toDest,  "[Route->RP]",
                      " + ", m_state.planCost_atDest, "[atRP]",
                      " + ", m_state.planCost_toRoute, "[RP->Route]",
                      " = ", m_state.plan_cost );


    return true;

}

const DirectedGraph::Graph &RoundtripPlanner::getGraph() const
{
    return m_state.graph;
}

const Route &RoundtripPlanner::getPlannedRoute() const
{
    return m_state.routeUpdated;
}

bool RoundtripPlanner::getPlannedRouteIndexRanges(size_t &indStart_toDest, size_t &indEnd_toDest, size_t &indStart_toRoute, size_t &indEnd_toRoute) const
{
    if(m_state.subRoutes.empty() || m_state.subRoutes.front().route_points.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "No planned routes");
        return false;
    }
    indStart_toDest = m_state.rp_index + 1;
    indEnd_toDest = m_state.rp_index + m_state.subRoutes.front().route_points.size();
    if(m_state.subRoutes.size() == 1 || m_state.subRoutes.at(1).route_points.empty()){
        indStart_toRoute = indEnd_toRoute = m_state.routeUpdated.route_points.size();
        return true;
    }
    indStart_toRoute = indEnd_toDest + 1;
    indEnd_toRoute = indEnd_toDest + m_state.subRoutes.at(1).route_points.size();
    return true;
}

Route RoundtripPlanner::getUpdatedBaseRoute()
{
    Route ret;
    m_state.routeBase.copyToWithoutPoints(ret, true);
    ret.route_points.insert( ret.route_points.end(), m_state.routeUpdated.route_points.begin(), m_state.routeUpdated.route_points.begin() + m_state.rp_index + 1 );
    ret.route_points.insert( ret.route_points.end(), m_state.routeUpdated.route_points.begin() + m_state.rp_ret_index_new, m_state.routeUpdated.route_points.end() );
    return ret;
}

double RoundtripPlanner::getCost() const
{
    return m_state.plan_cost;
}

bool RoundtripPlanner::foundPlan() const {return m_found_plan;}

bool RoundtripPlanner::initState(const DirectedGraph::Graph& graph, const Route& routeBase, size_t rp_index, size_t rp_ret_index)
{
    m_state = PlanState();

    m_state.graph = graph;
    m_state.routeBase = routeBase;
    m_state.rp_index = rp_index;
    m_state.rp_ret_index = rp_ret_index;
    if(rp_index < routeBase.route_points.size()){
        m_state.routePt = m_state.routeBase.route_points.at(rp_index);
        auto it_vt = graph.routepoint_vertex_map().find( m_state.routePt );
        if(it_vt == graph.routepoint_vertex_map().end()){
            logger().printError(__FUNCTION__, "Vertex corresponding to start route point not found in graph's map");
            return false;
        }

        m_state.routePtVt = it_vt->second;
        m_state.removedVisitPeriods = m_state.graph.removeVisitPeriodsAfterTimestamp({m_machine.id}, m_state.routePt.time_stamp);
    }

    if(rp_ret_index == rp_index){
        m_state.routePt_ret = m_state.routePt;
        m_state.routePtVt_ret = m_state.routePtVt;
    }
    else if(rp_ret_index < routeBase.route_points.size()){
        m_state.routePt_ret = m_state.routeBase.route_points.at(rp_ret_index);
        auto it_vt = graph.routepoint_vertex_map().find( m_state.routePt_ret );
        if(it_vt == graph.routepoint_vertex_map().end()){
            logger().printError(__FUNCTION__, "Vertex corresponding to return route point not found in graph's map");

            return false;
        }

        m_state.routePtVt_ret = it_vt->second;

        for(auto &vt_it : m_state.removedVisitPeriods){//remove for good the visiting periods of the connection between rp_index and rp_ret_index
            for(size_t i = 0 ; i < vt_it.second.size() ; ++i){
                DirectedGraph::VisitPeriod& vp = vt_it.second.at(i);
                if(vp.timestamp < m_state.routePt_ret.time_stamp - 1e-6){
                    vt_it.second.erase(vt_it.second.begin()+i);
                    --i;
                }
            }
        }
    }
    else
        m_state.routePt_ret = m_state.routePt;//to avoid possible errors

    routeBase.copyToWithoutPoints(m_state.routeTrip, true);
    routeBase.copyToWithoutPoints(m_state.routeUpdated, true);

    return true;

}

bool RoundtripPlanner::planPathToDestination(AstarPlan &plan, const std::vector<DirectedGraph::vertex_t> &destinationVertices, std::vector<std::shared_ptr<const Astar::ISuccesorChecker> > successorCheckers, bool allowReverseDriving)
{
    double min_cost = std::numeric_limits<double>::max();
    bool found_plan = false;

    Astar::PlanParameters astarParams;
    astarParams.start_vt = m_state.routePtVt;
    astarParams.start_time = m_state.routePt.time_stamp;
    astarParams.machine = m_machine;
    astarParams.machine_speed = m_machine.calcSpeed(m_state.routePt.bunker_mass);
    astarParams.initial_bunker_mass = m_state.routePt.bunker_mass;
    astarParams.includeWaitInCost = true;
    astarParams.successorCheckers = successorCheckers;

    DirectedGraph::vertex_t destination_vt;

    std::set<DirectedGraph::vertex_t> exclude;
    updateExcludeSet_toDest(exclude, allowReverseDriving);

    //additional successor checkers
    astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_VertexExcludeSet_Exceptions1>(exclude) );
    size_t indSCEdgeExcludeSet = astarParams.successorCheckers.size();
    astarParams.successorCheckers.emplace_back( nullptr );
    //@todo check if a FutureVisits checker is needed


    std::map<DirectedGraph::vertex_t, DirectedGraph::edge_t> successorVts;
    std::set<DirectedGraph::edge_t> successorEdges;
    for(auto out_edges = boost::out_edges(m_state.routePtVt, m_state.graph); out_edges.first != out_edges.second; out_edges.first++){
        successorVts[ target(*out_edges.first, m_state.graph) ] = *out_edges.first;
        successorEdges.insert(*out_edges.first);
    }

    for(size_t i = 0 ; i < 2 ; ++i){
        std::set<DirectedGraph::edge_t> allowedEdges;

        if(i == 0){//try first leaving via a vertex that is in "front" if the start_rp is an IF track_end
            if( allowReverseDriving
                    || m_state.routePt.type != RoutePoint::TRACK_END
                    || !Track::isInfieldTrack(m_state.routePt.track_id) )
                continue;


            int indPrev = geometry::getPrevNonRepeatedPointIndex(m_state.routeBase.route_points, m_state.rp_index, -1, -1);
            if(indPrev < 0)
                continue;
            RoutePoint routePt_prev = m_state.routeBase.route_points.at(indPrev);

            for(auto& edge_it : successorVts){
                auto successor_vt = edge_it.first;
                DirectedGraph::vertex_property successor_prop = m_state.graph[successor_vt];

                auto angle = geometry::get_angle(routePt_prev, m_state.routePt, successor_prop.route_point, true);//in deg!
                if( std::fabs(angle) > 115 )
                    allowedEdges.insert( edge_it.second );
            }

        }

        if(!allowedEdges.empty())
            astarParams.successorCheckers.at(indSCEdgeExcludeSet) = std::make_shared<AstarSuccessorChecker_EdgeExcludeSet>(successorEdges,
                                                                                                                           [&allowedEdges](const Astar::ISuccesorChecker::IsSuccessorValidParams& params)->bool{
                                                                                                                                return allowedEdges.find( params.edge ) != allowedEdges.end();
                                                                                                                           });
        else if(i < 1)
            continue;
        else
            astarParams.successorCheckers.at(indSCEdgeExcludeSet) = nullptr;

        //plan to each of the destination vertices and select the plan with lowest costs
        for (auto &dest_vt_tmp : destinationVertices) {
            astarParams.goal_vt = dest_vt_tmp;

            Astar aStar(astarParams,
                        m_settings,
                        RoutePoint::TRANSIT,
                        getOutputFolder("to_dest"),
                        loggerPtr());

            if( !aStar.plan(m_state.graph, m_edgeCostCalculator) ){
                logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Error calling aStar for destination vertex " + std::to_string(dest_vt_tmp));
                continue;  // no plan found => try next destination point
            }


            if (aStar.getPlan().plan_cost_total > min_cost)
                continue;

            plan = aStar.getPlan();

            destination_vt = dest_vt_tmp;
            min_cost = plan.plan_cost_total;

            found_plan = true;
        }

        if(found_plan)
            break;
    }

    if(!found_plan){
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "No plan to any of the destination points was found");
        return false;
    }

    //@todo we donnot know if the destination point is inside or outside the field
    if(plan.route_points_.size() > 1 && r_at(plan.route_points_, 1).isFieldAccess() )
        plan.adjustAccessPoints(true);
    else
        plan.adjustAccessPoints(false);

    m_state.destVt = destination_vt;

//    for(auto& rp : plan.route_points_){
//        if( !rp.isOfType({RoutePoint::FIELD_ENTRY,
//                          RoutePoint::FIELD_EXIT,
//                          RoutePoint::RESOURCE_POINT}) )
//            rp.type = RoutePoint::DEFAULT;
//    }

    return true;

}

bool RoundtripPlanner::planPathToRoutePoint(AstarPlan &plan, std::vector<std::shared_ptr<const Astar::ISuccesorChecker> > successorCheckers)
{
    bool found_plan = false;

    if(m_state.subRoutes.empty() || m_state.subRoutes.back().route_points.empty()){
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "No planned route points to the destination point");
        return false;
    }

    auto& prevRoute = m_state.subRoutes.back();

    Astar::PlanParameters astarParams;
    astarParams.start_vt = m_state.destVt;
    astarParams.goal_vt = m_state.routePtVt_ret;
    astarParams.start_time = prevRoute.route_points.back().time_stamp;
    astarParams.machine = m_machine;
    astarParams.machine_speed = m_machine.calcSpeed(m_state.routePt.bunker_mass);
    astarParams.initial_bunker_mass = prevRoute.route_points.back().bunker_mass;
    astarParams.includeWaitInCost = true;
    astarParams.successorCheckers = successorCheckers;


    std::set<DirectedGraph::vertex_t> exclude;
    if(m_state.rp_ret_index+1 < m_state.routeBase.route_points.size()){//exclude the vertex corresponding to the next working route point to avoid reaching the return route point in this direction
        const auto& routePt_next = m_state.routeBase.route_points.at(m_state.rp_ret_index+1);
        auto it_vt = m_state.graph.routepoint_vertex_map().find(routePt_next);
        if(it_vt != m_state.graph.routepoint_vertex_map().end())
            exclude.insert(it_vt->second);
    }

    //additional successor checkers
    astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_VertexExcludeSet_Exceptions1>(exclude) );
    size_t indSCEdgeExcludeSet = astarParams.successorCheckers.size();
    astarParams.successorCheckers.emplace_back( nullptr );

    std::map<DirectedGraph::vertex_t, DirectedGraph::edge_t> predecessorVts;
    std::set<DirectedGraph::edge_t> predecessorEdges;
    for(auto in_edges = boost::in_edges(m_state.routePtVt_ret, m_state.graph); in_edges.first != in_edges.second; in_edges.first++){
        predecessorVts[ source(*in_edges.first, m_state.graph) ] = *in_edges.first;
        predecessorEdges.insert(*in_edges.first);
    }

    for(size_t i = 0 ; i < 3 ; ++i){
        std::set<DirectedGraph::edge_t> allowedEdges;
        if(i == 0){//try planning via the previous route point
            if(m_state.rp_ret_index > 0){//check for vertex of previous route point (at the moment, only harvesting points are checked)
                int indPrev = geometry::getPrevNonRepeatedPointIndex(m_state.routeBase.route_points, m_state.rp_ret_index, -1, -1);
                if(indPrev < 0)
                    continue;
                RoutePoint routePt_prev = m_state.routeBase.route_points.at(indPrev);
                auto it_vt = m_state.graph.routepoint_vertex_map().find(routePt_prev);
                if(it_vt == m_state.graph.routepoint_vertex_map().end())
                    continue;

                auto edge_it = predecessorVts.find(it_vt->second);
                if(edge_it == predecessorVts.end())
                    continue;
                allowedEdges.insert( edge_it->second );
            }
        }
        else if(i == 1){//try planning via vertices that are "back"
            int indNext = geometry::getNextNonRepeatedPointIndex(m_state.routeBase.route_points, m_state.rp_ret_index, -1, -1);
            if(indNext < 0)
                continue;
            RoutePoint routePt_next = m_state.routeBase.route_points.at(indNext);

            for(auto& edge_it : predecessorVts){
                auto predecessor_vt = edge_it.first;
                DirectedGraph::vertex_property predecessor_prop = m_state.graph[predecessor_vt];

                auto angle = geometry::get_angle(routePt_next, m_state.routePt_ret, predecessor_prop.route_point, true);//in deg!
                if( std::fabs(angle) > 115 )
                    allowedEdges.insert( edge_it.second );
            }

        }

        if(!allowedEdges.empty())
            astarParams.successorCheckers.at(indSCEdgeExcludeSet) = std::make_shared<AstarSuccessorChecker_EdgeExcludeSet>(predecessorEdges,
                                                                                                                           [&allowedEdges](const Astar::ISuccesorChecker::IsSuccessorValidParams& params)->bool{
                                                                                                                                return allowedEdges.find( params.edge ) != allowedEdges.end();
                                                                                                                           });
        else if(i < 2)
            continue;
        else
            astarParams.successorCheckers.at(indSCEdgeExcludeSet) = nullptr;

        if(i == 0)
            logger().printOut(LogLevel::INFO, __FUNCTION__, "Planning via previous route point...");
        else if (i == 1)
            logger().printOut(LogLevel::INFO, __FUNCTION__, "Planning via back-vertices...");
        else
            logger().printOut(LogLevel::INFO, __FUNCTION__, "Planning directly to given route point");

        logger().printOut(LogLevel::INFO, __FUNCTION__, "Planning directly to given route point");

        astarParams.goal_vt = m_state.routePtVt_ret;
        exclude.erase(m_state.routePtVt_ret);

        Astar aStar(astarParams,
                    m_settings,
                    RoutePoint::TRANSIT,
                    getOutputFolder("to_route"),
                    loggerPtr());

        if( !aStar.plan(m_state.graph, m_edgeCostCalculator) ){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "...Error calling aStar.");
            continue;
        }

        plan = aStar.getPlan();

        found_plan = true;
        break;

    }

    if(!found_plan){
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "No plan was found");
        return false;
    }

    plan.adjustAccessPoints(false);

//    for(auto& rp : plan.route_points_){
//        if( !rp.isOfType({RoutePoint::FIELD_ENTRY,
//                          RoutePoint::FIELD_EXIT,
//                          RoutePoint::RESOURCE_POINT}) )
//            rp.type = RoutePoint::DEFAULT;
//    }

    return true;
}

bool RoundtripPlanner::updateRoute()
{
    if( m_state.subRoutes.size() != 2 ){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Planned subroutes count different than 2");
        return false;
    }

    auto& routePointsBase = m_state.routeBase.route_points;
    auto& routePointsUpdated = m_state.routeUpdated.route_points;

    routePointsUpdated.insert( routePointsUpdated.end(), routePointsBase.begin(), routePointsBase.begin()+m_state.rp_index+1 );

    for(auto &r : m_state.subRoutes)
        routePointsUpdated.insert( routePointsUpdated.end(), r.route_points.begin(), r.route_points.end() );

    m_state.travel_time = routePointsUpdated.back().time_stamp - routePointsBase.at(m_state.rp_index).time_stamp;

    m_state.rp_ret_index_new = routePointsUpdated.size();
    routePointsUpdated.insert( routePointsUpdated.end(), routePointsBase.begin()+m_state.rp_ret_index/*+1*/, routePointsBase.end() );

    double deltaTime = m_state.travel_time + m_state.routePt.time_stamp - m_state.routePt_ret.time_stamp;
    for(size_t i = m_state.rp_ret_index_new ; i < routePointsUpdated.size() ; ++i){
        auto & rp = routePointsUpdated.at(i);
        rp.time_stamp += deltaTime;

//        auto it_vt = m_state.graph.routepoint_vertex_map().find(rp);
//        if(it_vt != m_state.graph.routepoint_vertex_map().end()){//update the vertex (harvesting) timestamp
//            DirectedGraph::vertex_property &v_prop = m_state.graph[it_vt->second];
//            if(v_prop.route_point.time_stamp >= m_state.routePt_ret.time_stamp)
//                v_prop.route_point.time_stamp = rp.time_stamp;//important: set the timestamp of the route-point of harvester route (instead of adding the delay to the vertex timestamp), in case the vertex timestamp is outdated
//        }
    }
    updateTimestampsFromBaseRoute(m_state.graph, m_state.routeUpdated, 0.0, m_state.rp_ret_index_new, -1, m_state.routePt_ret.time_stamp);

    restoreVisitPeriods();

    return true;

}

void RoundtripPlanner::updateExcludeSet_toDest(std::set<DirectedGraph::vertex_t> &exclude,
                                             bool allowReverseDriving) const
{
    if(!allowReverseDriving){//don't drive in reverse
        if(m_state.rp_index > 0){
            int indPrev = geometry::getPrevNonRepeatedPointIndex(m_state.routeBase.route_points, m_state.rp_index, -1, -1);
            if(indPrev >= 0){
                const auto& prevRP = m_state.routeBase.route_points.at(indPrev);
                auto it_vt = m_state.graph.routepoint_vertex_map().find(prevRP);
                if(it_vt != m_state.graph.routepoint_vertex_map().end())
                    exclude.insert(it_vt->second);
            }
        }
    }

}

void RoundtripPlanner::restoreVisitPeriods()
{
    double deltaTime = m_state.travel_time + m_state.routePt.time_stamp - m_state.routePt_ret.time_stamp;
    for(auto & vt_it : m_state.removedVisitPeriods){
        DirectedGraph::vertex_property &vt_prop = m_state.graph[vt_it.first];
        for(auto &vp : vt_it.second){
            DirectedGraph::VisitPeriod vp_new = vp;
            vp_new.time_in += deltaTime;
            vp_new.time_out += deltaTime;
            vp_new.timestamp += deltaTime;
            vt_prop.visitPeriods.insert( std::make_pair(vp_new.time_in, vp) );
        }
    }

}


std::string RoundtripPlanner::getOutputFolder(const std::string &planType)
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

void RoundtripPlanner::PlanState::updateCost()
{
    plan_cost = planCost_toDest + planCost_atDest + planCost_toRoute;
}

void RoundtripPlanner::PlanState::addSubroute(const std::vector<RoutePoint> &routePoints)
{
    subRoutes.emplace_back(Route());
    subRoutes.back().machine_id = routeBase.machine_id;
    subRoutes.back().route_id = routeBase.route_id;
    subRoutes.back().route_points= routePoints;
}


}
