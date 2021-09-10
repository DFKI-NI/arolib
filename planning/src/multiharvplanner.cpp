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
 
#include "arolib/planning/multiharvplanner.h"

namespace arolib{

namespace{
int factorial(int n)
{
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}
}

const double MultiHarvPlanner::HarvMaxCapacityMultiplier = 0.95;


void MultiHarvPlanner::PlanData::init(const DirectedGraph::Graph &_graph, const std::vector<Route> &harv_routes)
{
    graph = _graph;

    planOverallCost = 0;
    planOK = false;

    routes = harv_routes;
    planCosts = std::vector<double>(routes.size(), 0);
    harvestingWindows.clear();

}

void MultiHarvPlanner::PlanData::updateOverallCost()
{
    planOverallCost = 0;
    for(auto &c : planCosts)
        planOverallCost += c;
}

void MultiHarvPlanner::PlanData::updateHarvestingWindows(size_t indRoute, size_t indRef, int deltaInd)
{
    updateHarvestingWindows(harvestingWindows, indRoute, indRef, deltaInd);
}

void MultiHarvPlanner::PlanData::updateHarvestingWindows(std::map<size_t, std::vector<MultiHarvPlanner::PlanData::HarvestingWindowInfo> > &harvestingWindows, size_t indRoute, size_t indRef, int deltaInd)
{
    auto it_r = harvestingWindows.find(indRoute);
    if(it_r == harvestingWindows.end())
        return;

    std::vector<HarvestingWindowInfo> &uws = it_r->second;
    for(auto& uw : uws){
        if(uw.indStart >= indRef){
            uw.indStart += deltaInd;
            uw.indFinish += deltaInd;
        }
    }

}

bool MultiHarvPlanner::PlannerSettings::parseFromStringMap(MultiHarvPlanner::PlannerSettings &params, const std::map<std::string, std::string> &map, bool strict)
{
    MultiHarvPlanner::PlannerSettings tmp;

    try{

        if( !RoundtripPlanner::PlannerSettings::parseFromStringMap(tmp, map, strict) )
            return false;

        if( !ASP_GeneralSettings::parseFromStringMap(tmp, map, strict) )
            return false;

        std::map<std::string, double*> dMap = { {"maxPlanningTime" , &tmp.maxPlanningTime} };
        std::map<std::string, bool*> bMap = { {"finishAtResourcePoint" , &tmp.finishAtResourcePoint} };

        if( !setValuesFromStringMap( map, dMap, strict)
                || !setValuesFromStringMap( map, bMap, strict) )
            return false;

    } catch(...){ return false; }

    params = tmp;

    return true;

}

std::map<std::string, std::string> MultiHarvPlanner::PlannerSettings::parseToStringMap(const MultiHarvPlanner::PlannerSettings &params)
{
    std::map<std::string, std::string> ret, subMap;
    subMap = RoundtripPlanner::PlannerSettings::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );
    subMap = ASP_GeneralSettings::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );

    ret["maxPlanningTime"] = double2string( params.maxPlanningTime );
    ret["switchOnlyAtTrackEnd"] = std::to_string( params.switchOnlyAtTrackEnd );

    return ret;
}

MultiHarvPlanner::MultiHarvPlanner(const DirectedGraph::Graph &graph,
                                   const std::vector<Route> &harv_routes,
                                   const std::vector<Machine> &machines,
                                   const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                   const MultiHarvPlanner::PlannerSettings &settings,
                                   std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                   const std::string &outputFolder,
                                   LogLevel logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_graph(graph),
    m_harvesterRoutes(harv_routes),
    m_machineInitialStates(machineCurrentStates),
    m_settings(settings),
    m_edgeCostCalculator(edgeCostCalculator),
    m_outputFolder(outputFolder)
{
    if(!m_outputFolder.empty() && m_outputFolder.back() != '/')
        m_outputFolder += "/";

    for(auto& m : machines){
        if(!m.isOfWorkingType(true))
            continue;
        m_harvesters[m.id] = m;
    }
}

void MultiHarvPlanner::reset()
{
    m_bestPlan.init(m_graph, m_harvesterRoutes);
    m_currentPlan.init(m_graph, m_harvesterRoutes);
}

std::string MultiHarvPlanner::planAll()
{
    if(!m_edgeCostCalculator){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return "Invalid edgeCostCalculator";
    }
    if(m_harvesterRoutes.empty()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No harvester routes were given");
        return "No harvester routes were given";
    }
    for(auto &r : m_harvesterRoutes){
        if(r.route_points.size() < 2){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "One or more harvester routes are invalid");
            return "One or more harvester routes are invalid";
        }
    }

    if(m_harvesters.empty()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No harvester machines were given");
        return "No harvester machines were given";
    }

    reset();
    std::string resp = calcHarvestingWindows(m_currentPlan);
    if(!resp.empty())
        return "Error computing the harvesting windows: " + resp;

    initRoutesBunkerMasses(m_currentPlan);

    addHarvesterSoilValues(m_currentPlan);

    if(m_settings.collisionAvoidanceOption != Astar::WITHOUT_COLLISION_AVOIDANCE)
        addInitialVisitPeriods(m_currentPlan);

    //plan transportation
    resp = planUnloadTrips(m_currentPlan);
    if(!resp.empty())
        return "Error computing the unload trips: " + resp;


    if(m_settings.collisionAvoidanceOption != Astar::WITHOUT_COLLISION_AVOIDANCE)
        addFinalVisitPeriods(m_currentPlan);


    m_currentPlan.planOK = true;
    m_bestPlan = m_currentPlan;

    return "";//ok

}

std::vector<Route> MultiHarvPlanner::getPlannedRoutes()
{
    return m_bestPlan.routes;
}

std::string MultiHarvPlanner::calcHarvestingWindows(PlanData &plan)
{
    //the routes are already initialized with the original (base) routes

    plan.harvestingWindows.clear();

    for(size_t i = 0 ; i < m_harvesterRoutes.size() ; ++i){
        auto& r = m_harvesterRoutes.at(i);
        auto& route_points = r.route_points;
        auto it_m = m_harvesters.find(r.machine_id);
        if(it_m == m_harvesters.end()){
            reset();
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, 10, "Machine with id " , r.machine_id, " was not given");
            return "Machine with id " + std::to_string(r.machine_id) + " was not given";
        }

        Machine &machine = it_m->second;

        if(machine.bunker_mass < 1e-5)//if the harvester has no bunker, don't plan transportation
            break;

        auto it_mdi = m_machineInitialStates.find(r.machine_id);
        double currentBunkerMass = 0;
        if(it_mdi != m_machineInitialStates.end())
            currentBunkerMass = it_mdi->second.bunkerMass;

        size_t indStartSearch = 0;
        while (1){

            size_t ind0 = route_points.size();
            for(size_t j = indStartSearch ; j < r.route_points.size() ; ++j){
                if(route_points.at(j).isOfTypeWorking_InTrack(true, true) || route_points.at(j).type == RoutePoint::TRACK_START ){
                    ind0 = j;
                    break;
                }
            }
            if(ind0+1 >= route_points.size())//nothing else to harvest
                break;

            auto rp0 = route_points.at(ind0);

            size_t ind1;
            for(ind1 = ind0+1 ; ind1 < route_points.size() ; ++ind1){
                if( route_points.at(ind1).harvested_mass - rp0.harvested_mass > HarvMaxCapacityMultiplier * machine.bunker_mass ){

                    bool firstRound = currentBunkerMass > 1e-6;//for cases when the machine has to go directly to unload
                    if(firstRound || ind1-1 > ind0)
                        --ind1;

                    while(ind1 > ind0
                          && !( route_points.at(ind1).isOfTypeWorking_InTrack(true) || route_points.at(ind1).type == RoutePoint::TRACK_END )){
                          --ind1;
                    }

                    if(ind1 + firstRound <= ind0)
                        ind1 = ind0+1;

                    if(m_settings.switchOnlyAtTrackEnd && Track::isInfieldTrack(route_points.at(ind1).track_id)){//only for infield harvesting
                        auto indTmp = ind1;
                        while(indTmp > ind0 && route_points.at(indTmp).type != RoutePoint::TRACK_END)
                            --indTmp;
                        if(indTmp + firstRound > ind0)
                            ind1 = indTmp;
                    }
                    break;
                }
            }

            if(ind1 == route_points.size())
                --ind1;

            PlanData::HarvestingWindowInfo unloadInfo;
            unloadInfo.indStart = ind0;
            unloadInfo.indFinish = ind1;

            plan.harvestingWindows[i].emplace_back(unloadInfo);

            indStartSearch = ind1;
        }

    }
    return ""; //ok
}

std::string MultiHarvPlanner::initRoutesBunkerMasses(MultiHarvPlanner::PlanData &plan)
{
    //the routes are already initialized with the original (base) routes
    for(size_t i = 0 ; i < plan.routes.size() ; ++i){
        auto& route = plan.routes.at(i);

        auto it_mdi = m_machineInitialStates.find(route.machine_id);
        double currentBunkerMass = 0;
        if(it_mdi != m_machineInitialStates.end())
            currentBunkerMass = it_mdi->second.bunkerMass;

        int ind = route.route_points.size() - 1;
        auto it_r = plan.harvestingWindows.find(i);
        if(it_r != plan.harvestingWindows.end() && !it_r->second.empty())
            ind = it_r->second.front().indStart;
        for(size_t j = 0 ; j <= ind ; ++j)
            route.route_points.at(j).bunker_mass = currentBunkerMass;

        if(ind+1 >= route.route_points.size())
            continue;

        //update bunker during harvesting windows
        for(size_t j = 0 ; j < it_r->second.size() ; ++j){
            PlanData::HarvestingWindowInfo& hw = it_r->second.at(j);
            bool continuesWindows = false;

            if(j > 0){
                PlanData::HarvestingWindowInfo& hw_prev = it_r->second.at(j-1);
                continuesWindows = hw.indStart == hw_prev.indFinish;
            }

            double harvestedMass0 = route.route_points.at(hw.indStart).harvested_mass;
            for(size_t k = hw.indStart + continuesWindows; k <= hw.indFinish; ++k){
                route.route_points.at(k).bunker_mass = route.route_points.at(k).harvested_mass - harvestedMass0;
            }

        }

        //update bunker after harvesting
        PlanData::HarvestingWindowInfo& hw_last = it_r->second.back();
        double bunkerMass_last = route.route_points.at(hw_last.indFinish).bunker_mass;
        for(size_t j = hw_last.indFinish+1; j < route.route_points.size(); ++j)
            route.route_points.at(j).bunker_mass = bunkerMass_last;

    }
    return ""; //ok
}

void MultiHarvPlanner::addHarvesterSoilValues(MultiHarvPlanner::PlanData &plan)
{
    for(size_t i = 0 ; i < plan.routes.size() ; ++i){
        auto& route = plan.routes.at(i);

        auto it_m = m_harvesters.find(route.machine_id);
        if(it_m == m_harvesters.end())
            continue;

        Machine& machine = it_m->second;

        int ind = route.route_points.size() - 1;
        auto it_r = plan.harvestingWindows.find(i);
        if(it_r != plan.harvestingWindows.end() && !it_r->second.empty())
            ind = it_r->second.front().indStart;

        if(ind > 0)
            addHarvesterSoilValues(plan.graph, machine, route, 0, ind);

        if(ind+1 >= route.route_points.size())
            continue;

        //update values during harvesting windows
        for(size_t j = 0 ; j < it_r->second.size() ; ++j){
            PlanData::HarvestingWindowInfo& hw = it_r->second.at(j);
            bool continuesWindows = false;

            if(j+1 < it_r->second.size()){
                PlanData::HarvestingWindowInfo& hw_next = it_r->second.at(j+1);
                continuesWindows = hw.indFinish == hw_next.indStart;
            }

            auto& rpStart = route.route_points.at(hw.indStart);
            auto rpTmp = rpStart;

            if(continuesWindows)//temporarilly set the bunkers to 0
                rpStart.bunker_mass = rpStart.bunker_volume = 0;

            if(hw.indStart < hw.indFinish)
                addHarvesterSoilValues(plan.graph, machine, route, hw.indStart, hw.indFinish);

            rpStart = rpTmp;
        }

        //update values after harvesting
        PlanData::HarvestingWindowInfo& hw_last = it_r->second.back();
        if(hw_last.indFinish+1 < route.route_points.size())
            addHarvesterSoilValues(plan.graph, machine, route, hw_last.indFinish, route.route_points.size()-1);

    }

}

void MultiHarvPlanner::addHarvesterSoilValues(DirectedGraph::Graph &graph, const Machine& machine, const Route &route, size_t ind0, size_t ind1)
{
    ind1 = std::min(ind1, route.route_points.size()-1);
    for(size_t i = ind0 ; i < ind1 ; ++i){
        auto & rp0 = route.route_points.at(i);
        auto & rp1 = route.route_points.at(i+1);
        auto it_0 = graph.routepoint_vertex_map().find( rp0 );
        if(it_0 == graph.routepoint_vertex_map().end())
            continue;
        auto it_1 = graph.routepoint_vertex_map().find( rp1 );
        if(it_1 == graph.routepoint_vertex_map().end())
            continue;

        DirectedGraph::overroll_property overrun;
        overrun.duration = rp1.time_stamp - rp0.time_stamp;
        overrun.machine_id = machine.id;
        overrun.weight = machine.weight + 0.5 * ( rp0.bunker_mass + rp1.bunker_mass );//average weight

        graph.addOverrun( it_0->second, it_1->second, overrun );
    }

}

void MultiHarvPlanner::addInitialVisitPeriods(MultiHarvPlanner::PlanData &plan)
{
    for(size_t i = 0 ; i < plan.routes.size() ; ++i){
        auto& route = plan.routes.at(i);

        auto it_m = m_harvesters.find(route.machine_id);
        if(it_m == m_harvesters.end())
            continue;

        Machine& machine = it_m->second;

        int ind = route.route_points.size() - 1;
        auto it_r = plan.harvestingWindows.find(i);
        if(it_r != plan.harvestingWindows.end() && !it_r->second.empty())
            ind = it_r->second.front().indStart - 1;

        if(ind > 0)
            addVisitPeriods(plan.graph, machine, route, 0, ind);

    }

}

void MultiHarvPlanner::addFinalVisitPeriods(MultiHarvPlanner::PlanData &plan)
{
    for(size_t i = 0 ; i < plan.routes.size() ; ++i){
        auto& route = plan.routes.at(i);

        auto it_m = m_harvesters.find(route.machine_id);
        if(it_m == m_harvesters.end())
            continue;
        Machine& machine = it_m->second;

        auto it_r = plan.harvestingWindows.find(i);
        if(it_r == plan.harvestingWindows.end() || it_r->second.empty())
            continue;

        PlanData::HarvestingWindowInfo& hw_last = it_r->second.back();
        if(hw_last.indFinish+1 < route.route_points.size())
            addVisitPeriods(plan.graph, machine, route, hw_last.indFinish+1, route.route_points.size()-1);

    }

}

void MultiHarvPlanner::addVisitPeriods(DirectedGraph::Graph &graph, const Machine &machine, const Route &route, size_t ind0, size_t ind1)
{
    ind1 = std::min(ind1, route.route_points.size()-1);
    double r = machine.workingRadius();

    for(size_t i = ind0 ; i <= ind1 ; ++i ){
        auto &rp = route.route_points.at(i);
        auto vp = getVisitPeriod(route.machine_id, route.route_points, r, i, &graph, &m_logger);

        bool addedToVertex = false;

        if(rp.isOfTypeWorking(true)){//add visit period to corresponding vertex

            auto it = graph.routepoint_vertex_map().find(rp);
            if(it != graph.routepoint_vertex_map().end()){
                DirectedGraph::vertex_property &v_prop = graph[it->second];
                v_prop.visitPeriods.insert( std::make_pair(vp.time_in, vp) );
                addedToVertex = true;
            }

        }

        if(!addedToVertex){//add visit period to proximal vertices
            auto vts = graph.getVerticesInRadius(rp, r);
            if( !vts.empty() ){
                addedToVertex = true;

                for(auto vt : vts){
                    DirectedGraph::vertex_property &v_prop = graph[vt];
                    if(vp.next_vt.empty()){//add all proximal vertices as next_vt of the visit period
                        vp.next_vt.emplace_back(std::make_pair(vt, v_prop.route_point));
                    }
                    v_prop.visitPeriods.insert( std::make_pair(vp.time_in, vp) );
                }
            }
        }
        if(!addedToVertex){
            m_logger.printOut(LogLevel::WARNING, __FUNCTION__, 10,
                              "No visit period was added corresponding to point [", i, "] of the route of machine ", route.machine_id);
        }
    }

}


std::string MultiHarvPlanner::planUnloadTrips(MultiHarvPlanner::PlanData &plan)
{
    std::string sError;
    auto harvWindows = plan.harvestingWindows;

    size_t countTrips = 0;

    std::set<MachineId_t> createdFolders;

    Machine nextHarv;
    size_t indNextRoute, indNextRP, indNextRP_ret;

    std::vector<DirectedGraph::vertex_t> resource_vts;
    for(const auto& it1 : plan.graph.resourcepoint_vertex_map()){
        const ResourcePoint& resPoint = it1.first;
        if(resPoint.resourceTypes.find(ResourcePoint::ResourceType_UNLOADING) != resPoint.resourceTypes.end()){
            resource_vts.push_back(it1.second);
            m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "\t\t\tAdded vertex to search for resource point " + std::to_string(resPoint.id) + ": " + resPoint.toString(10) );
        }
    }

    std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

    while(getNextTransportationInfo(plan, harvWindows, indNextRoute, indNextRP, indNextRP_ret, nextHarv)){
        auto& route = plan.routes.at(indNextRoute);

        //update the folder where the planning (search) information of the current machine will be stored
        std::string folderName = m_outputFolder;
        if(!folderName.empty()){
            folderName += ( "M" + std::to_string(route.machine_id) + "/Trip" + std::to_string(countTrips++) + "/" );
            if (!io::create_directory(folderName,
                                      createdFolders.find(route.machine_id) == createdFolders.end())){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error creating output folder '" + folderName + "'" );
                folderName.clear();
            }
        }

        //plan to resource (if indNextRP_ret != r.route_points.size()-1, plan route back to indNextRP)
        RoundtripPlanner rtp (nextHarv,
                              m_settings,
                              m_edgeCostCalculator,
                              folderName,
                              m_logger.logLevel());


        int attempt = 1;
        while(1){

            double duration = 0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count();
            m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "##### elapsed time since start of planning: " + std::to_string(duration) + " seconds\n" );
            if(m_settings.maxPlanningTime > 1e-5 && duration > m_settings.maxPlanningTime){
                sError = "Max. planning time reached while planning trip for machine " + std::to_string(route.machine_id);
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, sError );
                return sError;
            }

            double maxVisitTime_toDest = route.route_points.at(indNextRP).time_stamp;
            double maxVisitTime_toRoute = 0;
            if(indNextRP_ret < route.route_points.size())
                maxVisitTime_toRoute = route.route_points.at(indNextRP_ret).time_stamp;
            std::set<MachineId_t> restrictedMachineIds = {nextHarv.id};
            std::set<DirectedGraph::vertex_t> excludeVts_toDest, excludeVts_toRoute;

            bool allowReverseDriving = false;
            if(attempt == 2){//explesetily exclude all remaining harvesting vertices from this harvester and allowReverseDriving
                const auto& route_points = route.route_points;
                for(size_t i = indNextRP+1 ; i < route_points.size() ; ++i){
                    if(!route_points.at(i).isOfTypeWorking(true))
                        continue;
                    auto it_v = plan.graph.routepoint_vertex_map().find(route_points.at(i));
                    if(it_v == plan.graph.routepoint_vertex_map().end())
                        continue;
                    excludeVts_toDest.insert(it_v->second);
                }
                maxVisitTime_toDest = std::numeric_limits<double>::max();
                maxVisitTime_toRoute = std::numeric_limits<double>::max();
                restrictedMachineIds.clear();
                allowReverseDriving = true;
            }

            excludeVts_toRoute = excludeVts_toDest;

            //exclude resource points when planning to route
            for(auto& it_resP : plan.graph.resourcepoint_vertex_map())
                excludeVts_toRoute.insert(it_resP.second);

            if(!rtp.planTrip(plan.graph,
                             route,
                             indNextRP,
                             indNextRP_ret,
                             resource_vts,
                             excludeVts_toDest,
                             excludeVts_toRoute,
                             maxVisitTime_toDest,
                             maxVisitTime_toRoute,
                             restrictedMachineIds,
                             allowReverseDriving,
                             [](const RoutePoint& rp) -> RoutePoint {
                                RoutePoint rpNew = rp;
                                rpNew.time_stamp += 60;//@todo unload time = 60 s is fixed at the moment
                                rpNew.bunker_mass = rpNew.bunker_volume = 0;
                                return rpNew;
                             })){
                m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error planning trip for machine " + std::to_string(route.machine_id) +
                                                                   " for route points indexes [" + std::to_string(indNextRP) + ", " + std::to_string(indNextRP_ret) + "]" +
                                                                   " (attempt " + std::to_string(attempt) + ")");

                if(attempt >= 2){
                    sError = "Error planning trip for machine " + std::to_string(route.machine_id) +
                            " for route points indexes [" + std::to_string(indNextRP) + ", " + std::to_string(indNextRP_ret) + "]";
                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, sError );
                    return sError;
                }
                ++attempt;
            }
            else
                break;

        }

        int deltaInd = route.route_points.size();
        route = rtp.getPlannedRoute();
        plan.graph = rtp.getGraph();
        plan.planCosts.at(indNextRoute) += rtp.getCost();
        plan.updateOverallCost();

        deltaInd = route.route_points.size() - deltaInd;

        size_t indStart_toDest, indEnd_toDest, indStart_toRoute, indEnd_toRoute;
        if( rtp.getPlannedRouteIndexRanges(indStart_toDest, indEnd_toDest, indStart_toRoute, indEnd_toRoute) ){
//            route.route_points.at(indStart_toDest).type = RoutePoint::OVERLOADING_START;
//            for(size_t i = indStart_toDest+1 ; i <= indEnd_toDest ; ++i){
//                auto &rp = route.route_points.at(i);
//                if( !rp.isOfType({RoutePoint::FIELD_ENTRY,
//                                  RoutePoint::FIELD_EXIT,
//                                  RoutePoint::RESOURCE_POINT}) )
//                    rp.type = RoutePoint::OVERLOADING;
//            }
//            if(indStart_toRoute < route.route_points.size()){
//                for(size_t i = indStart_toRoute ; i < indEnd_toRoute ; ++i){
//                    auto &rp = route.route_points.at(i);
//                    if( !rp.isOfType({RoutePoint::FIELD_ENTRY,
//                                      RoutePoint::FIELD_EXIT,
//                                      RoutePoint::RESOURCE_POINT}) )
//                        rp.type = RoutePoint::OVERLOADING;
//                }
//                route.route_points.at(indEnd_toRoute).type = RoutePoint::OVERLOADING_FINISH;
//            }

            //update bunker from first route point after return trip (in case the windows were continues)
            if(indEnd_toRoute+1 < route.route_points.size()){
                route.route_points.at(indEnd_toRoute+1).bunker_mass = 0;
                route.route_points.at(indEnd_toRoute+1).bunker_volume = 0;
            }

            //update harvested_mass in trip routepoints
            for(size_t i = indStart_toDest ; i <= indEnd_toRoute && i < route.route_points.size() ; ++i){
                route.route_points.at(i).harvested_mass = route.route_points.at(indNextRP).harvested_mass;
                route.route_points.at(i).harvested_volume = route.route_points.at(indNextRP).harvested_volume;
            }
        }

        //update the harvester windows

        auto &windows = harvWindows.at(indNextRoute);
        pop_front(windows);
        if(windows.empty())
            harvWindows.erase(indNextRoute);

        PlanData::updateHarvestingWindows(harvWindows, indNextRoute, indNextRP_ret, deltaInd);
        plan.updateHarvestingWindows(indNextRoute, indNextRP_ret, deltaInd);
    }

    return sError; //ok

}

bool MultiHarvPlanner::getNextTransportationInfo(const MultiHarvPlanner::PlanData &plan,
                                                 const std::map<size_t, std::vector<PlanData::HarvestingWindowInfo> > &harvestingWindows,
                                                 size_t &indRoute, size_t &indRP, size_t &indRP_ret, Machine &harv)
{

    bool ret = false;
    double minTime = std::numeric_limits<double>::max();
    for(const auto& it_hw : harvestingWindows){
        const std::vector<PlanData::HarvestingWindowInfo> & hws = it_hw.second;
        if(hws.empty())
            continue;

        if(hws.size() == 1 && !m_settings.finishAtResourcePoint)
            continue;

        size_t ind = hws.front().indFinish;
        const auto& route = plan.routes.at(it_hw.first);
        if(ind >= route.route_points.size())
            continue;

        auto rp = route.route_points.at(ind);

        if(rp.time_stamp < minTime){
            ret = true;
            minTime = rp.time_stamp;
            indRoute = it_hw.first;
            harv = m_harvesters.at( route.machine_id );
            indRP = indRP_ret = ind;
            if(hws.size() > 1)
                indRP_ret = hws.at(1).indStart;
            else if(indRP_ret+1 >= route.route_points.size())//set it to size so that no return trip is computed
                indRP_ret = route.route_points.size();
        }
    }

    return ret;
}



}
