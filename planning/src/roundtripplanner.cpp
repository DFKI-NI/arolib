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
 
#include "arolib/planning/roundtripplanner.hpp"
#include "arolib/planning/graphhelper.hpp"
#include "arolib/planning/astar.hpp"

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
        m_logger.printOut(LogLevel::ERROR, "m_state.plan_cost is NAN!");

    if(!m_outputFolder.empty() && m_outputFolder.back() != '/')
        m_outputFolder += "/";

}

bool RoundtripPlanner::planTrip(const DirectedGraph::Graph &graph,
                                const Route &routeBase,
                                size_t rp_index,
                                size_t rp_ret_index,
                                const std::vector<DirectedGraph::vertex_t> &destinationVertices,
                                const std::set<DirectedGraph::vertex_t> &excludeVts_toDest,
                                const std::set<DirectedGraph::vertex_t> &excludeVts_toRoute,
                                double maxVisitTime_toDest,
                                double maxVisitTime_toRoute,
                                const std::set<MachineId_t> &restrictedMachineIds,
                                bool allowReverseDriving,
                                RoutePoint functAtDest(const RoutePoint &) ) // para ver como cambia el route point una vez esta en el destino. tambien se podria incluir aca el tiempo que gasta ahi
{
    if(!m_edgeCostCalculator){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return false;
    }

    m_found_plan = false;
    initState(graph, routeBase, rp_index, rp_ret_index);

    if(routeBase.route_points.empty()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Harvester route is empty");
        return false;
    }

    if(rp_index >= routeBase.route_points.size()){
        m_logger.printOut(LogLevel::ERROR, "Invalid route-point index");
        return false;
    }

    if(destinationVertices.empty()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "There are no destination vertices");
        return false;
    }

    //send to the best (intermediate) destination
    AstarPlan planToDestination;
    std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
    if( !planPathToDestination(planToDestination,
                               destinationVertices,
                               maxVisitTime_toDest,
                               restrictedMachineIds,
                               excludeVts_toDest,
                               allowReverseDriving) ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error calling planPathToDestination");
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
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "plan_cost is NAN (planPathToDestination)!");


    if(!m_state.subRoutes.empty() && !m_state.subRoutes.back().route_points.empty()){//add info 'at (intermediate) destination'
        auto rpLast = m_state.subRoutes.back().route_points.back();
        auto rpAtDest = functAtDest(rpLast);
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
        if( !planPathToRoutePoint(planToRoute, maxVisitTime_toRoute, restrictedMachineIds, excludeVts_toRoute) ){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error calling planPathToRoutePoint");
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
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "plan_cost is NAN (planPathToRoutePoint)!");

    }
    else if (!m_state.subRoutes.empty() && !m_state.subRoutes.back().route_points.empty()){
        RoutePoint p = m_state.subRoutes.back().route_points.back();
        m_state.addSubroute({p});
    }

    m_state.subRoutes.front().route_points.pop_back();//remove the extra point previously calculated with the functAtDest

    updateRoute();

    m_found_plan = true;

    m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, 10, "Planning duration = ",
                      m_state.planningDuration_toDest, "[Route->RP]",
                      " + ", m_state.planningDuration_toRoute, "[RP->Route]",
                      " = ", m_state.planningDuration_toDest + m_state.planningDuration_toRoute, " seconds" );

    m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, 10, "Planning cost = ",
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
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No planned routes");
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

void RoundtripPlanner::initState(const DirectedGraph::Graph& graph, const Route& routeBase, size_t rp_index, size_t rp_ret_index)
{
    m_state = PlanState();

    m_state.graph = graph;
    m_state.routeBase = routeBase;
    m_state.rp_index = rp_index;
    m_state.rp_ret_index = rp_ret_index;
    if(rp_index < routeBase.route_points.size()){
        m_state.routePt = m_state.routeBase.route_points.at(rp_index);
        auto it_vt = graph.routepoint_vertex_map().find( m_state.routePt );
        if(it_vt != graph.routepoint_vertex_map().end())
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
        if(it_vt != graph.routepoint_vertex_map().end())
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

}

bool RoundtripPlanner::planPathToDestination(AstarPlan &plan,
                                             const std::vector<DirectedGraph::vertex_t> &destinationVertices,
                                             double maxVisitTime,
                                             const std::set<MachineId_t> &restrictedMachineIds,
                                             const std::set<DirectedGraph::vertex_t> &excludeVts,
                                             bool allowReverseDriving)
{

    double min_cost = std::numeric_limits<double>::max();
    bool found_plan = false;

    Astar::PlanParameters astarParams;
    astarParams.start_vt = m_state.routePtVt;
    astarParams.start_time = m_state.routePt.time_stamp;
    astarParams.max_time_visit = maxVisitTime;
    astarParams.max_time_goal = std::numeric_limits<double>::max();
    astarParams.machine = m_machine;
    astarParams.machine_speed = m_machine.calcSpeed(m_state.routePt.bunker_mass);
    astarParams.initial_bunker_mass = m_state.routePt.bunker_mass;
    astarParams.overload = false;
    astarParams.includeWaitInCost = true;
    astarParams.restrictedMachineIds = restrictedMachineIds;
    astarParams.restrictedMachineIds_futureVisits = {};//@todo check

    DirectedGraph::vertex_t destination_vt;

    astarParams.exclude = excludeVts;
    updateExcludeSet_toRP(astarParams.exclude, allowReverseDriving);

    //plan to each of the destination vertices and select the plan with lowest costs
    for (auto &dest_vt_tmp : destinationVertices) {
        astarParams.goal_vt = dest_vt_tmp;

        Astar aStar(astarParams,
                    m_settings,
                    RoutePoint::TRANSIT,
                    getOutputFolder("to_dest"),
                    &m_logger);

        if( !aStar.plan(m_state.graph, m_edgeCostCalculator) ){
            m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Error calling aStar for destination vertex " + std::to_string(dest_vt_tmp));
            continue;  // no plan found => try next destination point
        }


        if (aStar.getPlan().plan_cost_total > min_cost)
            continue;

        plan = aStar.getPlan();

        destination_vt = dest_vt_tmp;
        min_cost = plan.plan_cost_total;

        found_plan = true;
    }
    if(!found_plan){
        m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "No plan to any of the destination points was found");
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

bool RoundtripPlanner::planPathToRoutePoint(AstarPlan &plan,
                                            double maxVisitTime,
                                            const std::set<MachineId_t> &restrictedMachineIds,
                                            const std::set<DirectedGraph::vertex_t> &excludeVts)
{
    bool found_plan = false;

    if(m_state.subRoutes.empty() || m_state.subRoutes.back().route_points.empty()){
        m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "No planned route points to the destination point");
        return false;
    }

    auto& prevRoute = m_state.subRoutes.back();

    Astar::PlanParameters astarParams;
    astarParams.start_vt = m_state.destVt;
    astarParams.start_time = prevRoute.route_points.back().time_stamp;
    astarParams.max_time_visit = maxVisitTime;
    astarParams.max_time_goal = std::numeric_limits<double>::max();
    astarParams.machine = m_machine;
    astarParams.machine_speed = m_machine.calcSpeed(m_state.routePt.bunker_mass);
    astarParams.initial_bunker_mass = prevRoute.route_points.back().bunker_mass;
    astarParams.overload = false;
    astarParams.includeWaitInCost = true;
    astarParams.restrictedMachineIds = restrictedMachineIds;
    astarParams.restrictedMachineIds_futureVisits = {};//@todo check

    astarParams.exclude = excludeVts;
    astarParams.exclude.insert(m_state.routePtVt_ret);

    DirectedGraph::vertex_t routeVt = -1;
    RoutePoint routePt_prev;

    if(m_state.rp_ret_index > 0){//check for vertex of previous route point (at the moment, only harvesting points are checked)
        routePt_prev = m_state.routeBase.route_points.at(m_state.rp_ret_index-1);
        auto it_vt = m_state.graph.routepoint_vertex_map().find(routePt_prev);
        if(it_vt != m_state.graph.routepoint_vertex_map().end())
            routeVt = it_vt->second;
    }
    if(routeVt != -1){//try planning to the previous route point

        astarParams.goal_vt = routeVt;

        Astar aStar(astarParams,
                    m_settings,
                    RoutePoint::TRANSIT,
                    getOutputFolder("to_route"),
                    &m_logger);

        if( !aStar.plan(m_state.graph, m_edgeCostCalculator) )
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error calling aStar for previous route point vertex " + std::to_string(routeVt));

        else{
            plan = aStar.getPlan();

            auto p = m_state.routePt;
            p.type = RoutePoint::TRANSIT;
            if(!plan.route_points_.empty()){
                double deltaTime = m_state.routePt.time_stamp - routePt_prev.time_stamp;
                double speed = m_machine.calcSpeed(0);
                if(speed > 0)
                    deltaTime = arolib::geometry::calc_dist( m_state.routePt, routePt_prev ) / speed;
                p.time_stamp = plan.route_points_.back().time_stamp + deltaTime;
                p.bunker_mass = 0;
                p.bunker_volume = 0;
            }

            plan.route_points_.emplace_back(p);

            found_plan = true;
        }
    }

    if(!found_plan){
        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Planning directly to given route point");

        astarParams.goal_vt = m_state.routePtVt_ret;
        astarParams.exclude.erase(m_state.routePtVt_ret);

        Astar aStar(astarParams,
                    m_settings,
                    RoutePoint::TRANSIT,
                    getOutputFolder("to_route"),
                    &m_logger);

        if( !aStar.plan(m_state.graph, m_edgeCostCalculator) ){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error calling aStar for route point vertex " + std::to_string(routeVt));
            return false;
        }

        plan = aStar.getPlan();

        found_plan = true;
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
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Planned subroutes count different than 2");
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

    for(size_t i = m_state.rp_ret_index_new ; i < routePointsUpdated.size() ; ++i){
        auto & rp = routePointsUpdated.at(i);
        double deltaTime = m_state.travel_time + m_state.routePt.time_stamp - m_state.routePt_ret.time_stamp;
        rp.time_stamp += deltaTime;

        auto it_vt = m_state.graph.routepoint_vertex_map().find(rp);
        if(it_vt != m_state.graph.routepoint_vertex_map().end()){//update the vertex (harvesting) timestamp
            DirectedGraph::vertex_property &v_prop = m_state.graph[it_vt->second];
            v_prop.route_point.time_stamp = rp.time_stamp;//important: set the timestamp of the route-point of harvester route (instead of adding the delay to the vertex timestamp), in case the vertex timestamp is outdated
        }
    }

    restoreVisitPeriods();

    return true;

}

void RoundtripPlanner::updateExcludeSet_toRP(std::set<DirectedGraph::vertex_t> &exclude,
                                             bool allowReverseDriving) const
{
    if(!allowReverseDriving){//don't drive in reverse
        if(m_state.rp_index > 0 && m_state.rp_index < m_state.routeBase.route_points.size()){
            const auto& prevRP = m_state.routeBase.route_points.at(m_state.rp_index-1);
            auto it_vt = m_state.graph.routepoint_vertex_map().find(prevRP);
            if(it_vt != m_state.graph.routepoint_vertex_map().end())
                exclude.insert(it_vt->second);
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
