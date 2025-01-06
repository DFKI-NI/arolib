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
 
#include "arolib/planning/route_planner_standalone_machines.hpp"

#include "arolib/misc/filesystem_helper.h"
#include "arolib/planning/path_search/graphhelper.hpp"

namespace arolib{

namespace{
int factorial(int n)
{
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}
}

const double RoutePlannerStandaloneMachines::MachineMaxCapacityMultiplier_OutputFlow = 0.95;
const double RoutePlannerStandaloneMachines::MachineMaxCapacityMultiplier_InputFlow = 0.999;


void RoutePlannerStandaloneMachines::PlanData::init(const DirectedGraph::Graph &_graph, const std::vector<Route> &_routes)
{
    graph = _graph;

    planOverallCost = 0;
    planOK = false;

    routes = _routes;
    planCosts = std::vector<double>(routes.size(), 0);
    workingWindows.clear();
}

void RoutePlannerStandaloneMachines::PlanData::updateOverallCost()
{
    planOverallCost = 0;
    for(auto &c : planCosts)
        planOverallCost += c;
}

bool RoutePlannerStandaloneMachines::PlannerSettings::parseFromStringMap(RoutePlannerStandaloneMachines::PlannerSettings &params, const std::map<std::string, std::string> &map, bool strict)
{
    RoutePlannerStandaloneMachines::PlannerSettings tmp;

    if( !RoundtripPlanner::PlannerSettings::parseFromStringMap(tmp, map, strict) )
        return false;

    if( !ASP_GeneralSettings::parseFromStringMap(tmp, map, strict) )
        return false;

    int finishPointOption = tmp.finishPointOption;

    std::map<std::string, double*> dMap = { {"maxPlanningTime" , &tmp.maxPlanningTime} };

    std::map<std::string, int*> enumMap = { {"finishPointOption" , &finishPointOption} };

    if( !setValuesFromStringMap( map, dMap, strict)
            || !setValuesFromStringMap( map, enumMap, strict))
        return false;

    tmp.finishPointOption = RoutePlannerStandaloneMachines::intToFinishPointOption( finishPointOption );

    params = tmp;
    return true;
}

std::map<std::string, std::string> RoutePlannerStandaloneMachines::PlannerSettings::parseToStringMap(const RoutePlannerStandaloneMachines::PlannerSettings &params)
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

RoutePlannerStandaloneMachines::FinishPointOption RoutePlannerStandaloneMachines::intToFinishPointOption(int value)
{
    if(value == FinishPointOption::FINISH_AT_FIELD_EXIT)
        return FinishPointOption::FINISH_AT_FIELD_EXIT;
    else if(value == FinishPointOption::FINISH_AT_RESOURCE_POINT)
        return FinishPointOption::FINISH_AT_RESOURCE_POINT;
    else if(value == FinishPointOption::FINISH_IN_FIELD)
        return FinishPointOption::FINISH_IN_FIELD;

    throw std::invalid_argument( "The given value does not correspond to any RoutePlannerStandaloneMachines::FinishPoint" );
}

RoutePlannerStandaloneMachines::RoutePlannerStandaloneMachines(const DirectedGraph::Graph &graph,
                                                               const std::vector<Route> &baseRoutes,
                                                               const std::vector<Machine> &machines,
                                                               const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                               const std::map<ResourcePointId_t, ResourcePointState> &resourcePointCurrentStates,
                                                               const Polygon &boundary,
                                                               const RoutePlannerStandaloneMachines::PlannerSettings &settings,
                                                               std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                                               const std::string &outputFolder,
                                                               LogLevel logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_graph(graph),
    m_machineInitialStates(machineCurrentStates),
    m_resourcePointCurrentStates(resourcePointCurrentStates),
    m_boundary(boundary),
    m_settings(settings),
    m_edgeCostCalculator(edgeCostCalculator),
    m_outputFolder(outputFolder)
{
    if(!m_outputFolder.empty() && m_outputFolder.back() != '/')
        m_outputFolder += "/";

    for(auto& m : machines){
        if(!m.isOfWorkingType(true))
            continue;
        m_machines[m.id] = m;
    }

    m_baseRoutes.reserve(baseRoutes.size());
    for(const auto& route : baseRoutes){
        double machineTimestamp = 0;
        auto mdi_it = m_machineInitialStates.find(route.machine_id);
        if(mdi_it != m_machineInitialStates.end())
            machineTimestamp = std::max(0.0, mdi_it->second.timestamp);

        size_t indStart = route.route_points.size();
        for(size_t i = 0 ; i < route.route_points.size() ; ++i){
            if(route.route_points.at(i).time_stamp > -1e-9){
                indStart = i;
                break;
            }
        }

        if(indStart > 0)//set the timestamps of the vertices corresponding to the disregarded segment to "worked"
            resetTimestampsFromBaseRoute(m_graph, route, 0, indStart-1);

        m_baseRoutes.emplace_back(Route());
        route.copyToWithoutPoints(m_baseRoutes.back(), true);
        if(indStart < route.route_points.size()){

            m_baseRoutes.back().route_points.insert(m_baseRoutes.back().route_points.end(),
                                                    route.route_points.begin() + indStart,
                                                    route.route_points.end());
            if(machineTimestamp > 1e-9 && m_baseRoutes.back().route_points.front().time_stamp < machineTimestamp){
                double delta_time = machineTimestamp - m_baseRoutes.back().route_points.front().time_stamp;
                for(auto& rp : m_baseRoutes.back().route_points)
                    rp.time_stamp += delta_time;
            }

            //update the timestamps of the vertices corresponding to the remaining route points
            updateTimestampsFromBaseRoute(m_graph, m_baseRoutes.back(), 0, 0, -1, 0);
        }
    }
}

void RoutePlannerStandaloneMachines::reset()
{
    m_bestPlan.init(m_graph, m_baseRoutes);
    m_currentPlan.init(m_graph, m_baseRoutes);
}

std::string RoutePlannerStandaloneMachines::planAll(MaterialFlowType materialFlowType, TransitRestriction transitRestriction)
{
    if(!m_edgeCostCalculator){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return "Invalid edgeCostCalculator";
    }
    if(m_baseRoutes.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "No base routes were given");
        return "No harvester routes were given";
    }
    for(auto &r : m_baseRoutes){
        if(r.route_points.size() < 2){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "One or more base routes are invalid");
            return "One or more harvester routes are invalid";
        }
    }

    if(m_machines.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "No machines were given");
        return "No harvester machines were given";
    }

    if(!MachineDynamicInfo::isValid(m_machineInitialStates)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid machine initial states");
        return "Invalid machine initial states";
    }

    reset();
    std::string resp = initWorkingWindows(materialFlowType, m_currentPlan);
    if(!resp.empty())
        return "Error computing the initial working windows: " + resp;

    initRoutesBunkerMasses(m_currentPlan, materialFlowType);

    addInitialOverruns(m_currentPlan);

    if(m_settings.collisionAvoidanceOption != Astar::WITHOUT_COLLISION_AVOIDANCE)
        addInitialVisitPeriods(m_currentPlan);

    //plan transportation
    resp = planTrips(m_currentPlan, materialFlowType, transitRestriction);
    if(!resp.empty())
        return "Error computing the unload trips: " + resp;


    m_currentPlan.planOK = true;
    m_bestPlan = m_currentPlan;

    return "";//ok

}

std::vector<Route> RoutePlannerStandaloneMachines::getPlannedRoutes()
{
    return m_bestPlan.routes;
}

std::string RoutePlannerStandaloneMachines::getNextWorkingWindow(MaterialFlowType materialFlowType, const Route& route, size_t indPtFrom, double bunker_mass, double bunker_vol, PlanData::WorkingWindowInfo &workingWindow, bool &finished)
{
    finished = true;

    if(materialFlowType == NEUTRAL_MATERIAL_FLOW)//no transportation needed -> no working windows
        return ""; //ok

    double machineMaxCapacityMultiplier = (materialFlowType == INPUT_MATERIAL_FLOW ? MachineMaxCapacityMultiplier_InputFlow : MachineMaxCapacityMultiplier_OutputFlow);

    auto& route_points = route.route_points;

    if(indPtFrom >= route_points.size())
        return ""; //ok

    auto it_m = m_machines.find(route.machine_id);
    if(it_m == m_machines.end()){
        reset();
        logger().printOut(LogLevel::ERROR, __FUNCTION__, 10, "Machine with id " , route.machine_id, " was not given");
        return "Machine with id " + std::to_string(route.machine_id) + " was not given";
    }

    Machine &machine = it_m->second;

    bool checkMass = bunker_mass > -1e-6 && machine.bunker_mass > -1e-6;
    bool checkVol = bunker_vol > -1e-6 && machine.bunker_volume > -1e-6;

    if(machine.bunker_mass < 1e-6 && machine.bunker_volume < 1e-6){//if the machine has no bunker capacity, don't plan transportation
        return ""; //ok  (or should we return error?)
        //return "The machine has no valid bunker capacity"; //ok  (or should we return error?)
    }

    if(!checkMass && !checkVol)
        return "Invalid bunker capacities";


    double usedBunkerMassCapacity = 0, usedBunkerVolumeCapacity = 0; //Note: for input material flow operations, the 'used' level is the empty part of the bunker, hence the complement operation
    if(checkMass)
        usedBunkerMassCapacity = ( materialFlowType == MaterialFlowType::INPUT_MATERIAL_FLOW ? machine.bunker_mass - bunker_mass : bunker_mass);
    if(checkVol)
        usedBunkerVolumeCapacity = ( materialFlowType == MaterialFlowType::INPUT_MATERIAL_FLOW ? machine.bunker_volume - bunker_vol : bunker_vol);

    size_t maxIndPt = route.route_points.size()-1;

    bool firstRound = indPtFrom == 0;//for cases when the machine has to go directly to unload    //or should we receive that as parameter in case the first round does not start with pt index == 0?


    size_t ind0 = maxIndPt+1;
    for(size_t j = indPtFrom ; j <= maxIndPt ; ++j){
        if(j+1 <= maxIndPt && route_points.at(j).point() == route_points.at(j+1).point())//check for repeated points corresponding to turning times
            continue;
        if(route_points.at(j).isOfTypeWorking_InTrack(true, true) || route_points.at(j).type == RoutePoint::TRACK_START ){
            ind0 = j;
            break;
        }
    }
    if(ind0+1 > maxIndPt)//nothing else to work
        return ""; //ok;

    finished = false;

    auto rp0 = route_points.at(ind0);

    size_t ind1;
    for(ind1 = ind0+1 ; ind1 <= maxIndPt ; ++ind1){
        if( route_points.at(ind1).worked_mass - rp0.worked_mass > machineMaxCapacityMultiplier * machine.bunker_mass - usedBunkerMassCapacity ){

            if(firstRound || ind1-1 > ind0)
                --ind1;

            while(ind1 > ind0
                  && !( route_points.at(ind1).isOfTypeWorking_InTrack(true) || route_points.at(ind1).type == RoutePoint::TRACK_END )){
                  --ind1;
            }

            if(ind1 + firstRound <= ind0)
                ind1 = ind0+1;

            if( (m_settings.switchOnlyAtTrackEndHL && Track::isHeadlandTrack(route_points.at(ind1).track_id)) ||
                    (m_settings.switchOnlyAtTrackEnd && Track::isInfieldTrack(route_points.at(ind1).track_id))){
                auto indTmp = ind1;
                while(indTmp > ind0 && route_points.at(indTmp).type != RoutePoint::TRACK_END)
                    --indTmp;
                if(indTmp + firstRound > ind0)
                    ind1 = indTmp;
            }
            break;
        }
    }

    if(ind1 == maxIndPt+1)
        --ind1;

    workingWindow = PlanData::WorkingWindowInfo();
    workingWindow.indStart = ind0;
    workingWindow.indFinish = ind1;

    return ""; //ok
}


std::string RoutePlannerStandaloneMachines::initWorkingWindows(MaterialFlowType materialFlowType, PlanData &plan)
{
    //the routes in plan are already initialized with the original (base) routes

    plan.nextWorkingWindows.clear();
    if(materialFlowType == NEUTRAL_MATERIAL_FLOW){//no transportation needed -> working windows

//        for(size_t i = 0 ; i < m_baseRoutes.size() ; ++i){
//            auto& r = m_baseRoutes.at(i);
//            auto& route_points = r.route_points;

//            PlanData::WorkingWindowInfo& ww = plan.workingWindowsNew[i];
//            ww.indStart = 0;
//            ww.indFinish = route_points.size()-1;
//        }
        return ""; //ok
    }

    for(size_t i = 0 ; i < plan.routes.size() ; ++i){
        auto& route = plan.routes.at(i);
        auto it_m = m_machines.find(route.machine_id);
        if(it_m == m_machines.end()){
            reset();
            logger().printOut(LogLevel::ERROR, __FUNCTION__, 10, "Machine with id " , route.machine_id, " was not given");
            return "Machine with id " + std::to_string(route.machine_id) + " was not given";
        }

        Machine &machine = it_m->second;

        auto it_mdi = m_machineInitialStates.find(route.machine_id);

        //default bunker levels: empty for OUTPUT_MATERIAL_FLOW, full for INPUT_MATERIAL_FLOW
        double bunker_mass = ( materialFlowType == MaterialFlowType::INPUT_MATERIAL_FLOW ? machine.bunker_mass : 0);
        double bunker_vol = ( materialFlowType == MaterialFlowType::INPUT_MATERIAL_FLOW ? machine.bunker_volume : 0);
        if(it_mdi != m_machineInitialStates.end()
                && ( it_mdi->second.bunkerMass > -1e-6 || it_mdi->second.bunkerVolume > -1e-6 ) ){
            bunker_mass = it_mdi->second.bunkerMass;
            bunker_vol = it_mdi->second.bunkerVolume;
        }

        PlanData::WorkingWindowInfo ww;
        bool finished;
        auto error = getNextWorkingWindow(materialFlowType, route, 0, bunker_mass, bunker_vol, ww, finished);
        if(!error.empty())
            return "Error obtaining initial window for route of machine with id " + std::to_string(route.machine_id) + ": " + error;

        if(!finished)
            plan.nextWorkingWindows[i] = ww;

    }
    return ""; //ok
}

std::string RoutePlannerStandaloneMachines::initRoutesBunkerMasses(RoutePlannerStandaloneMachines::PlanData &plan, MaterialFlowType materialFlowType)
{
    //the routes are already initialized with the original (base) routes
    for(size_t i = 0 ; i < plan.routes.size() ; ++i){
        auto& route = plan.routes.at(i);
        const Machine &machine = m_machines.at(route.machine_id);

        auto it_mdi = m_machineInitialStates.find(route.machine_id);
        double currentBunkerMass, currentBunkerVolume;
        if(it_mdi != m_machineInitialStates.end()){
            currentBunkerMass = it_mdi->second.bunkerMass;
            currentBunkerVolume = it_mdi->second.bunkerVolume;
        }
        else{//default
            if(materialFlowType == MaterialFlowType::INPUT_MATERIAL_FLOW){
                currentBunkerMass = machine.bunker_mass;
                currentBunkerVolume = machine.bunker_volume;
            }
            else
                currentBunkerMass = currentBunkerVolume = 0;
        }

        //set the bunker_mass of the routepoints before the first window to the current bunker mass in the machine
        int ind = route.route_points.size() - 1;
        auto it_ww = plan.workingWindows.find(i);
        if(it_ww != plan.workingWindows.end() && !it_ww->second.empty())
            ind = it_ww->second.front().indStart;
        for(size_t j = 0 ; j <= ind ; ++j){
            route.route_points.at(j).bunker_mass = currentBunkerMass;
            route.route_points.at(j).bunker_volume = currentBunkerVolume;
        }
    }
    return ""; //ok
}

void RoutePlannerStandaloneMachines::addInitialOverruns(RoutePlannerStandaloneMachines::PlanData &plan)
{
    for(size_t i = 0 ; i < plan.routes.size() ; ++i){
        auto& route = plan.routes.at(i);

        auto it_m = m_machines.find(route.machine_id);
        if(it_m == m_machines.end())
            continue;

        Machine& machine = it_m->second;

        int ind = route.route_points.size() - 1;
        auto it_r = plan.nextWorkingWindows.find(i);
        if(it_r != plan.nextWorkingWindows.end())
            ind = it_r->second.indStart;

        if(ind > 0)
            addOverruns(plan.graph, machine, route, 0, ind);
    }

}

void RoutePlannerStandaloneMachines::addOverruns(DirectedGraph::Graph &graph, const Machine& machine, const Route &route, size_t ind0, size_t ind1)
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

void RoutePlannerStandaloneMachines::addInitialVisitPeriods(RoutePlannerStandaloneMachines::PlanData &plan)
{
    for(size_t i = 0 ; i < plan.routes.size() ; ++i){
        auto& route = plan.routes.at(i);

        auto it_m = m_machines.find(route.machine_id);
        if(it_m == m_machines.end())
            continue;

        Machine& machine = it_m->second;

        int ind = route.route_points.size() - 1;
        auto it_r = plan.nextWorkingWindows.find(i);
        if(it_r != plan.nextWorkingWindows.end())
            ind = it_r->second.indStart - 1;

        if(ind > 0)
            addVisitPeriods(plan.graph, machine, route, 0, ind);

    }

}


void RoutePlannerStandaloneMachines::addVisitPeriods(DirectedGraph::Graph &graph, const Machine &machine, const Route &route, size_t ind0, size_t ind1)
{
    ind1 = std::min(ind1, route.route_points.size()-1);
    double r = machine.workingRadius();

    for(size_t i = ind0 ; i <= ind1 ; ++i ){
        auto &rp = route.route_points.at(i);
        auto vp = getVisitPeriod(route.machine_id, route.route_points, r, i, &graph, loggerPtr());

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
            logger().printOut(LogLevel::WARNING, __FUNCTION__, 10,
                              "No visit period was added corresponding to point [", i, "] of the route of machine ", route.machine_id);
        }
    }

}


std::string RoutePlannerStandaloneMachines::planTrips(RoutePlannerStandaloneMachines::PlanData &plan, MaterialFlowType materialFlowType, TransitRestriction transitRestriction)
{
    struct NextBunkerCapInfo{
        double mass;
        double vol;
        float K;
    };

    std::string sError;

    size_t countTrips = 0;

    std::set<MachineId_t> createdFolders;

    Machine nextMachine;
    size_t indNextRoute, indNextRP, indNextRP_ret;
    PlanData::WorkingWindowInfo nextWorkingWindow;

    std::map<DirectedGraph::vertex_t, std::pair<double, double>> resourcePointCapacities;

    ResourcePoint::ResourceType resType = materialFlowType == MaterialFlowType::INPUT_MATERIAL_FLOW ? ResourcePoint::ResourceType_LOADING : ResourcePoint::ResourceType_UNLOADING;
    for(const auto& it1 : plan.graph.resourcepoint_vertex_map()){
        const ResourcePoint& resPoint = it1.first;
        if( materialFlowType != MaterialFlowType::NEUTRAL_MATERIAL_FLOW &&
                ( std::fabs(resPoint.massCapacity) < 1e-6 || std::fabs(resPoint.volumeCapacity) < 1e-6 ) )
            continue;
        if(resPoint.resourceTypes.find(resType) != resPoint.resourceTypes.end()){
            auto itRPCap = m_resourcePointCurrentStates.find(resPoint.id);
            double massCap = resPoint.massCapacity > -1e-7 ? resPoint.massCapacity : std::numeric_limits<double>::max();
            double volCap = resPoint.volumeCapacity > -1e-7? resPoint.volumeCapacity : std::numeric_limits<double>::max();
            if( itRPCap != m_resourcePointCurrentStates.end() ){
                const ResourcePointState& rpState = itRPCap->second;
                if( !rpState.enabled )
                    continue;
                if( !std::isnan(rpState.capacityMass) )
                    massCap = rpState.capacityMass;
                if( !std::isnan(rpState.capacityVolume) )
                    volCap = rpState.capacityVolume;
            }

            resourcePointCapacities[it1.second] = std::make_pair(massCap, volCap);
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "\t\t\tAdded vertex to search for resource point " + std::to_string(resPoint.id) + ": " + resPoint.toString(10) );
        }
    }

    std::vector<DirectedGraph::vertex_t> exit_vts;
    if(m_settings.finishPointOption == FinishPointOption::FINISH_AT_FIELD_EXIT){
        for(const auto& it1 : plan.graph.accesspoint_vertex_map()){
            const FieldAccessPoint& fap = it1.first;
            if(fap.accessType != FieldAccessPoint::AP_ENTRY_ONLY){
                exit_vts.push_back(it1.second);
                logger().printOut(LogLevel::DEBUG, __FUNCTION__, "\t\t\tAdded vertex to search for exit point " + std::to_string(fap.id) + ": " + fap.toString(10) );
            }
        }
    }


    std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

    adjustBaseRoutesTimestamps(plan);

    //plan to resource point first if needed
    sError = planInitialTrips(plan, resourcePointCapacities, materialFlowType, transitRestriction);
    if(!sError.empty())
        return sError;

    //add overruns and visit periods of current next working windows
    for(auto& it : plan.nextWorkingWindows){
        auto& route = plan.routes.at(it.first);
        auto& machine = m_machines.at(route.machine_id);
        auto& ww = it.second;
        addOverruns(plan.graph, machine, route, ww.indStart, ww.indFinish);
        if(m_settings.collisionAvoidanceOption != Astar::WITHOUT_COLLISION_AVOIDANCE)
            addVisitPeriods(plan.graph, machine, route, ww.indStart, ww.indFinish);
    }

    while(getNextTransportationInfo(plan, materialFlowType, indNextRoute, indNextRP, indNextRP_ret, nextMachine, nextWorkingWindow)){
        plan.workingWindows[indNextRoute].push_back(nextWorkingWindow);

        auto& route = plan.routes.at(indNextRoute);

        //update bunker mass/volume for the working window
        size_t indFinish = std::min(nextWorkingWindow.indFinish, route.route_points.size()-1);
        float multCap = materialFlowType == MaterialFlowType::OUTPUT_MATERIAL_FLOW ? 1 : -1;
        for(size_t i = nextWorkingWindow.indStart+1 ; i <= indFinish ; ++i ){
            route.route_points.at(i).bunker_mass = route.route_points.at(i-1).bunker_mass + multCap * (route.route_points.at(i).worked_mass - route.route_points.at(i-1).worked_mass);
            route.route_points.at(i).bunker_volume = route.route_points.at(i-1).bunker_volume + multCap * (route.route_points.at(i).worked_volume - route.route_points.at(i-1).worked_volume);
        }

        //update the folder where the planning (search) information of the current machine will be stored
        std::string folderName = m_outputFolder;
        if(!folderName.empty()){
            folderName += ( "M" + std::to_string(route.machine_id) + "/Trip" + std::to_string(countTrips++) + "/" );
            if (!io::create_directory(folderName,
                                      createdFolders.find(route.machine_id) == createdFolders.end())){
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating output folder '" + folderName + "'" );
                folderName.clear();
            }
        }


        std::map<DirectedGraph::vertex_t, NextBunkerCapInfo> nextBunkerCapacities;
        std::vector<DirectedGraph::vertex_t> resource_vts;
        double remainingMass = route.route_points.back().worked_mass - route.route_points.at(indNextRP).worked_mass;
        double remainingVol = route.route_points.back().worked_volume - route.route_points.at(indNextRP).worked_volume;
        for(auto& it_cap : resourcePointCapacities){
            NextBunkerCapInfo capInfo;
            capInfo.mass = route.route_points.at(indNextRP).bunker_mass;
            capInfo.vol = route.route_points.at(indNextRP).bunker_volume;
            capInfo.K = getMachineBunkerStateAfterResourcePoint(nextMachine, materialFlowType,
                                                                it_cap.second.first, it_cap.second.second,
                                                                remainingMass, remainingVol, capInfo.mass, capInfo.vol);

            if(capInfo.K > 0.5){//@todo check the next bunkercapacity with respect to the capacity needed for the next window
                resource_vts.emplace_back(it_cap.first);
                nextBunkerCapacities[it_cap.first] = capInfo;
            }

        }

        //plan to resource or exit (if indNextRP_ret != r.route_points.size()-1, plan route back to indNextRP)
        RoundtripPlanner rtp (nextMachine,
                              m_settings,
                              m_edgeCostCalculator,
                              folderName,
                              logger().logLevel());


        int attempt = 1;
        while(1){

            double duration = 0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count();
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "##### elapsed time since start of planning: " + std::to_string(duration) + " seconds\n" );
            if(m_settings.maxPlanningTime > 1e-5 && duration > m_settings.maxPlanningTime){
                sError = "Max. planning time reached while planning trip for machine " + std::to_string(route.machine_id);
                logger().printOut(LogLevel::ERROR, __FUNCTION__, sError );
                return sError;
            }


            double maxVisitTime_toDest = route.route_points.at(indNextRP).time_stamp;
            double maxVisitTime_toRoute = 0;
            bool toExitPoint = false;
            if(indNextRP_ret < route.route_points.size())
                maxVisitTime_toRoute = route.route_points.at(indNextRP_ret).time_stamp;
            else
                toExitPoint = ( m_settings.finishPointOption == FinishPointOption::FINISH_AT_FIELD_EXIT );

            std::set<MachineId_t> restrictedMachineIds = {nextMachine.id};
            std::set<DirectedGraph::vertex_t> excludeVts_toDest, excludeVts_toRoute;

            bool allowReverseDriving = false;
            if(attempt == 2){//explicitly exclude all remaining working vertices from this machine and allowReverseDriving
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

            //remove the return vt from the excludeVts_toRoute
            if(indNextRP_ret < route.route_points.size()){
                auto it_v = plan.graph.routepoint_vertex_map().find(route.route_points.at(indNextRP_ret));
                if(it_v != plan.graph.routepoint_vertex_map().end())
                    excludeVts_toRoute.erase(it_v->second);
            }


            //successor checkers
            //@todo: we need to add SuccessorTimestamp checkers for the other machines. Problem: we do not know how the timestamps of the following machines will change, hence we need to use the current timestamps
            std::vector<std::shared_ptr<const Astar::ISuccesorChecker> > successorCheckers_toDest, successorCheckers_toRoute;
            if(transitRestriction == TransitRestriction::TRANSIT_ONLY_OVER_WORKED_AREAS){
                successorCheckers_toDest.emplace_back( std::make_shared<AstarSuccessorChecker_SuccessorTimestamp>(maxVisitTime_toDest,
                                                                                                                  AstarSuccessorChecker_SuccessorTimestamp::INVALID_IF_LOWER_THAN_SUCC_TIMESPAMP,
                                                                                                                  restrictedMachineIds) );
                successorCheckers_toRoute.emplace_back( std::make_shared<AstarSuccessorChecker_SuccessorTimestamp>(maxVisitTime_toRoute,
                                                                                                                   AstarSuccessorChecker_SuccessorTimestamp::INVALID_IF_LOWER_THAN_SUCC_TIMESPAMP,
                                                                                                                   restrictedMachineIds) );
            }
            else if(transitRestriction == TransitRestriction::TRANSIT_ONLY_OVER_UNWORKED_AREAS){
                successorCheckers_toDest.emplace_back( std::make_shared<AstarSuccessorChecker_SuccessorTimestamp>(maxVisitTime_toDest,
                                                                                                                  AstarSuccessorChecker_SuccessorTimestamp::INVALID_IF_HIGHER_THAN_SUCC_TIMESPAMP,
                                                                                                                  restrictedMachineIds) );
                successorCheckers_toRoute.emplace_back( std::make_shared<AstarSuccessorChecker_SuccessorTimestamp>(maxVisitTime_toRoute,
                                                                                                                   AstarSuccessorChecker_SuccessorTimestamp::INVALID_IF_HIGHER_THAN_SUCC_TIMESPAMP,
                                                                                                                   restrictedMachineIds) );
            }
            successorCheckers_toDest.emplace_back( std::make_shared<AstarSuccessorChecker_VertexExcludeSet_Exceptions1>(excludeVts_toDest) );
            successorCheckers_toRoute.emplace_back( std::make_shared<AstarSuccessorChecker_VertexExcludeSet_Exceptions1>(excludeVts_toRoute) );


            if(!rtp.planTrip(plan.graph,
                             route,
                             indNextRP,
                             indNextRP_ret,
                             toExitPoint ? exit_vts : resource_vts,
                             successorCheckers_toDest,
                             successorCheckers_toRoute,
                             allowReverseDriving,
                             [&nextMachine, &plan, toExitPoint, &nextBunkerCapacities, &route]
                             (const RoutePoint& rp, const DirectedGraph::vertex_t & vt_resP) -> RoutePoint {
                                if(toExitPoint)
                                    return RoutePoint(Point::invalidPoint());
                                auto it_cap = nextBunkerCapacities.find(vt_resP);
                                if(it_cap == nextBunkerCapacities.end())
                                   return RoutePoint(Point::invalidPoint());

                                double bunker_mass_after = it_cap->second.mass;
                                double bunker_volume_after = it_cap->second.vol;

                                RoutePoint rpNew = rp;
                                double timeAtResource = 60; //initial default
                                if(nextMachine.unloading_speed_mass > 1e-6)
                                    timeAtResource = std::fabs( rp.bunker_mass - bunker_mass_after ) / nextMachine.unloading_speed_mass;
                                else if(nextMachine.unloading_speed_volume > 1e-6)
                                    timeAtResource = std::fabs( rp.bunker_volume - bunker_volume_after ) / nextMachine.unloading_speed_volume;
                                else{
                                    DirectedGraph::vertex_property vt_prop_resP = plan.graph[vt_resP];
                                    OutFieldInfo::UnloadingCosts ulc;
                                    if( OutFieldInfo::getUnloadingCosts(vt_prop_resP.unloadingCosts, nextMachine.id, ulc) ){
                                        if(ulc.time_per_kg > 1e-9)
                                            timeAtResource = std::fabs( rp.bunker_mass - bunker_mass_after ) * ulc.time_per_kg;
                                        else if(ulc.time > 1e-9)
                                            timeAtResource = ulc.time;
                                    }
                                }
                                rpNew.time_stamp += std::max(0.0, timeAtResource);
                                rpNew.bunker_mass = bunker_mass_after;
                                rpNew.bunker_volume = bunker_volume_after;
                                return rpNew;
                             })){
                    logger().printOut(LogLevel::WARNING, __FUNCTION__, "Error planning trip for machine " + std::to_string(route.machine_id) +
                                                                   " for route points indexes [" + std::to_string(indNextRP) + ", " + std::to_string(indNextRP_ret) + "]" +
                                                                   " (attempt " + std::to_string(attempt) + ")");

                if(attempt >= 2){
                    sError = "Error planning trip for machine " + std::to_string(route.machine_id) +
                            " for route points indexes [" + std::to_string(indNextRP) + ", " + std::to_string(indNextRP_ret) + "]";
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, sError );
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
        if( !rtp.getPlannedRouteIndexRanges(indStart_toDest, indEnd_toDest, indStart_toRoute, indEnd_toRoute) ){
            sError = "Error obtaining planned route indexes ranges";
            return sError;
        }

        //update worked_mass in trip routepoints
        for(size_t i = indStart_toDest ; i <= indEnd_toRoute && i < route.route_points.size() ; ++i){
            route.route_points.at(i).worked_mass = route.route_points.at(indNextRP).worked_mass;
            route.route_points.at(i).worked_volume = route.route_points.at(indNextRP).worked_volume;
        }

        double massPrev = route.route_points.at(indStart_toDest).bunker_mass;
        double volPrev = route.route_points.at(indStart_toDest).bunker_volume;
        double massNew = massPrev;
        double volNew = volPrev;


        if( materialFlowType != MaterialFlowType::NEUTRAL_MATERIAL_FLOW){
            if(indEnd_toRoute < route.route_points.size()){

                massNew = route.route_points.at(indEnd_toRoute).bunker_mass;
                volNew = route.route_points.at(indEnd_toRoute).bunker_volume;

                //update currrent capacity of resource points
                auto it_caps = resourcePointCapacities.find(rtp.getPlannedDestinationVt());
                if(it_caps != resourcePointCapacities.end()){
                    it_caps->second.first -= std::fabs( massNew - massPrev );
                    it_caps->second.second -= std::fabs( volNew - volPrev );
                }
            }

            //update bunker from first route point after return trip (in case the windows were continues)
            if(indEnd_toRoute+1 < route.route_points.size()){
                route.route_points.at(indEnd_toRoute+1).bunker_mass = massNew;
                route.route_points.at(indEnd_toRoute+1).bunker_volume = volNew;
            }


            if(indEnd_toRoute < route.route_points.size()){// get next working window of this route/machine
                PlanData::WorkingWindowInfo ww_next;
                bool finished;
                auto error = getNextWorkingWindow(materialFlowType, route, indEnd_toRoute, massNew, volNew, ww_next, finished);
                if(!error.empty()){
                    sError = "Error obtaining next window for route of machine with id " + std::to_string(route.machine_id) + ": " + error;
                    return sError;
                }

                if(finished){
                    plan.nextWorkingWindows.erase(indNextRoute);
                    continue;
                }

                //update working window for next check
                plan.nextWorkingWindows[indNextRoute] = ww_next;

                //update overruns and visit periods of new working window
                addOverruns(plan.graph, nextMachine, route, ww_next.indStart, ww_next.indFinish);
                if(m_settings.collisionAvoidanceOption != Astar::WITHOUT_COLLISION_AVOIDANCE)
                    addVisitPeriods(plan.graph, nextMachine, route, ww_next.indStart, ww_next.indFinish);
            }
        }

    }

    return sError; //ok

}


void RoutePlannerStandaloneMachines::adjustBaseRoutesTimestamps(PlanData& plan)
{
    for(auto& route : plan.routes){
        double machineTimestamp = 0;
        auto mdi_it = m_machineInitialStates.find(route.machine_id);
        if(mdi_it != m_machineInitialStates.end())
            machineTimestamp = std::max(0.0, mdi_it->second.timestamp);

        size_t indStart = route.route_points.size();
        for(size_t i = 0 ; i < route.route_points.size() ; ++i){
            if(route.route_points.at(i).time_stamp > -1e-9){
                indStart = i;
                break;
            }
        }

        if(indStart > 0)//set the timestamps of the vertices corresponding to the disregarded segment to "worked"
            resetTimestampsFromBaseRoute(plan.graph, route, 0, indStart-1);

        route.route_points.erase(route.route_points.begin(), route.route_points.begin()+indStart);
        if(!route.route_points.empty()){
            if(machineTimestamp > 1e-9 && route.route_points.front().time_stamp < machineTimestamp){
                double delta_time = machineTimestamp - route.route_points.front().time_stamp;
                for(auto& rp : route.route_points)
                    rp.time_stamp += delta_time;
            }

            //update the timestamps of the vertices corresponding to the remaining route points
            updateTimestampsFromBaseRoute(plan.graph, route, 0, 0, -1, 0);
        }
    }
}

std::string RoutePlannerStandaloneMachines::planInitialTrips(PlanData &plan,
                                                             std::map<DirectedGraph::vertex_t, std::pair<double, double>> &resourcePointCapacities,
                                                             MaterialFlowType materialFlowType,
                                                             TransitRestriction transitRestriction)
{
    std::string sError;

    std::multimap<double, size_t> routesToResource, routesToRoute;
    std::set<size_t> addedRoutes;
    std::vector<size_t> routesToResourceInds;
    for(auto& it_ww : plan.nextWorkingWindows){
        PlanData::WorkingWindowInfo &ww = it_ww.second;
        const Route& route = plan.routes.at(it_ww.first);
        addedRoutes.insert(it_ww.first);
        auto it_mdi = m_machineInitialStates.find(route.machine_id);
        if(it_mdi == m_machineInitialStates.end())
            continue;
//        double dist = geometry::calc_dist(it_mdi->second.position, route.route_points.at(ww.indStart));
//        if(dist < 0.1)
//            continue;
        if(ww.indStart == 0 && ww.indFinish == 0){
            routesToResource.insert( std::make_pair( std::max(0.0, it_mdi->second.timestamp),
                                                     it_ww.first ) );
            routesToResourceInds.push_back(it_ww.first);
            continue;
        }
        routesToRoute.insert( std::make_pair( std::max(0.0, it_mdi->second.timestamp),
                                              it_ww.first ) );
    }

    if(addedRoutes.size() != plan.routes.size()){//there might be routes without working windows
        for(size_t i = 0 ; i < plan.routes.size() ; ++i){
            auto it = addedRoutes.find(i);
            if (it != addedRoutes.end())
                continue;
            auto it_mdi = m_machineInitialStates.find(plan.routes.at(i).machine_id);
            routesToRoute.insert( std::make_pair( std::max(0.0, it_mdi->second.timestamp),
                                                  i ) );
        }
    }

    std::map<size_t, AstarPlan> initPlans;

    //plan the initial segments for the machines that can go directly from their current location to the first working window (routesToRoute)
    sError = planInitialSegmentDirectly(plan, routesToRoute, transitRestriction, initPlans);
    if(!sError.empty())
        return sError;


    //plan the initial segments for the machines that have to go to a resource point before starting the first working window (routesToResource)
    sError = planInitialSegmentViaResource(plan, routesToResource, resourcePointCapacities, materialFlowType, transitRestriction, initPlans);
    if(!sError.empty())
        return sError;


    for(auto& it_plan : initPlans){
        size_t routeInd = it_plan.first;
        auto& route = plan.routes.at(routeInd);
        AstarPlan& initPlan = it_plan.second;
        std::vector<RoutePoint>& initRPs = initPlan.route_points_;

        if(initRPs.empty())
            continue;

        //adjust some of the route points' types
        bool isInside = true;
        initRPs.front().type = RoutePoint::INITIAL_POSITION;
        for(size_t i = 0 ; i+1 < initRPs.size() ; ++i){
            RoutePoint& rp = r_at(initRPs, i);
            if(rp.isFieldAccess() ){
                rp.type = (isInside ? RoutePoint::FIELD_ENTRY : RoutePoint::FIELD_EXIT);
                isInside = !isInside;
            }
            else if(rp.type != RoutePoint::RESOURCE_POINT)
                rp.type = ( isInside ? RoutePoint::TRANSIT : RoutePoint::TRANSIT_OF);
        }


        //update the timestamps of the current route (add the duration of the initial route segment)
        double initTimestamp = initRPs.back().time_stamp;
        double refTimestamp = route.route_points.front().time_stamp;
        for(auto& rp : route.route_points)
            rp.time_stamp = rp.time_stamp - refTimestamp + initTimestamp;

        //update the timestamps of the vertices corresponding to the remaining route points
        updateTimestampsFromBaseRoute(plan.graph, route, 0, 0, -1, 0);

        //update bunker mass and volume from 1st route point
        if(!route.route_points.empty() && !initRPs.empty()){
            route.route_points.front().bunker_mass = initRPs.back().bunker_mass;
            route.route_points.front().bunker_volume = initRPs.back().bunker_volume;
        }

        size_t deltaInd = route.route_points.size();

        //add initial route segments to the beginning of the route
        route.route_points.insert(route.route_points.begin(), initRPs.begin(), initRPs.end()-1);

        //update the working window indexes
        auto it_ww = plan.nextWorkingWindows.find( routeInd );
        if(it_ww != plan.nextWorkingWindows.end()){
            deltaInd = route.route_points.size() - deltaInd;
            it_ww->second.indStart += deltaInd;
            it_ww->second.indFinish += deltaInd;
        }

        plan.planCosts.at(routeInd) += initPlan.plan_cost_total;
        plan.updateOverallCost();
    }


    return sError;

}


std::string RoutePlannerStandaloneMachines::planInitialSegmentDirectly(PlanData &plan, const std::multimap<double, size_t> &indRoutes, TransitRestriction transitRestriction,
                                                                       std::map<size_t, AstarPlan> &initPlans)
{
    std::string sError;
    auto& graph = plan.graph;

    for(auto & it_wws : indRoutes){
        size_t routeInd = it_wws.second;
        Route& route = plan.routes.at(routeInd);

        auto it_m = m_machines.find(route.machine_id);
        auto it_mdi = m_machineInitialStates.find(route.machine_id);

        if(route.route_points.empty()
                || it_m == m_machines.end()
                || it_mdi == m_machineInitialStates.end() ){
            continue;
        }

        const MachineDynamicInfo& mdi = it_mdi->second;
        const Machine &machine = it_m->second;
        double machine_speed = machine.calcSpeed( mdi.bunkerMass );

        //if the working machines is inside the field and very close to the first route point, connect it directly with the first route point
        if(geometry::in_polygon(mdi.position, m_boundary) && route.route_points.size() > 1){
            PointVec seg = {route.route_points.front(), route.route_points.at(1)};
            auto dist = geometry::calc_dist_to_linestring(seg, mdi.position, false);
            if( dist < 2 * machine.getTurningRadius()){
                RoutePoint rp;
                rp.type = RoutePoint::INITIAL_POSITION;
                rp.time_stamp = mdi.timestamp;
                rp.point() = mdi.position;
                rp.bunker_mass = mdi.bunkerMass;
                rp.bunker_volume = mdi.bunkerVolume;

                if(dist < geometry::getGeometryLength(seg)){
                    int ind = geometry::addSampleToGeometryClosestToPoint(seg, mdi.position, 1);
                    if( seg.size() > 2 && ind > 0 ){
                        route.route_points.front().point() = seg.at(ind);
                        dist = geometry::calc_dist(route.route_points.front(), mdi.position);
                    }
                }

                if(dist < 1e-3)
                    continue;

                AstarPlan& initPlan = initPlans[routeInd];
                initPlan.route_points_.emplace_back(rp);
                rp = route.route_points.front();
                rp.time_stamp += ( dist / machine_speed );
                initPlan.route_points_.emplace_back(rp);
                continue;
            }
        }

        auto it_vt = graph.initialpoint_vertex_map().find(route.machine_id);
        if(it_vt == graph.initialpoint_vertex_map().end()){
            sError = "Machine id " + std::to_string(route.machine_id) + " has no initial vertex";
            logger().printOut(LogLevel::ERROR, __FUNCTION__, sError );
            return sError;
        }

        auto it_vt_2 = graph.routepoint_vertex_map().find( route.route_points.front() );
        if(it_vt_2 == graph.routepoint_vertex_map().end()){
            sError = "Vertex corresponding to start route point of machine id " + std::to_string(route.machine_id) + " not found in graph's map";
            logger().printOut(LogLevel::ERROR, __FUNCTION__, sError );
            return sError;
        }

        //connect using AStar
        Astar::PlanParameters astarParams;
        astarParams.start_vt = it_vt->second;
        astarParams.goal_vt = it_vt_2->second;
        astarParams.start_time = mdi.timestamp;
        astarParams.machine = machine;
        astarParams.machine_speed = machine_speed;
        astarParams.initial_bunker_mass = mdi.bunkerMass;
        astarParams.includeWaitInCost = false;

        std::set<MachineId_t> restrictedMachineIds = {machine.id};
        if(transitRestriction == TransitRestriction::TRANSIT_ONLY_OVER_WORKED_AREAS){
            astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_SuccessorTimestamp>(route.route_points.front().time_stamp,
                                                                                                               AstarSuccessorChecker_SuccessorTimestamp::INVALID_IF_LOWER_THAN_SUCC_TIMESPAMP,
                                                                                                               restrictedMachineIds) );
        }
        else if(transitRestriction == TransitRestriction::TRANSIT_ONLY_OVER_UNWORKED_AREAS){
            astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_SuccessorTimestamp>(route.route_points.front().time_stamp,
                                                                                                               AstarSuccessorChecker_SuccessorTimestamp::INVALID_IF_HIGHER_THAN_SUCC_TIMESPAMP,
                                                                                                               restrictedMachineIds) );
        }

        astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_VertexExcludeSet_Exceptions1>( getExcludeVertices_initialRoutes(graph, astarParams.goal_vt) ) );
        //@todo check if a FutureVisits checker is needed

        const Astar::AStarSettings& astarSettings_original = m_settings;
        Astar::AStarSettings astarSettings = astarSettings_original;
        astarSettings.includeWaitInCost = false;

        auto foldername_planData = m_outputFolder;
        if(!foldername_planData.empty()){
            foldername_planData += "M" + std::to_string(route.machine_id) + "/init/to_route/";
            io::create_directory(foldername_planData, true);
        }

        Astar planner (astarParams,
                       astarSettings,
                       RoutePoint::TRANSIT,
                       foldername_planData,
                       loggerPtr());

        if( !planner.plan(graph, m_edgeCostCalculator) ){

            sError = "Error getting initial path for machine " + std::to_string(route.machine_id);
            logger().printOut(LogLevel::ERROR, __FUNCTION__, sError );
            return sError;
        }

        //set again the timestamp of the vertex corresponding to the first route point
        resetTimestampsFromBaseRoute(graph, route, 0, 0, route.route_points.front().time_stamp);

        initPlans[routeInd] = planner.getPlan();
    }


    return sError;
}

std::string RoutePlannerStandaloneMachines::planInitialSegmentViaResource(PlanData &plan,
                                                                          const std::multimap<double, size_t> &indRoutes,
                                                                          std::map<DirectedGraph::vertex_t, std::pair<double, double>> &resourcePointCapacities,
                                                                          MaterialFlowType materialFlowType,
                                                                          TransitRestriction transitRestriction,
                                                                          std::map<size_t, AstarPlan> &initPlans)
{
    std::string sError;
    auto& graph = plan.graph;

    for(auto & it_wws : indRoutes){
        size_t routeInd = it_wws.second;
        const Route& route = plan.routes.at(routeInd);

        auto it_m = m_machines.find(route.machine_id);
        auto it_mdi = m_machineInitialStates.find(route.machine_id);

        if(route.route_points.empty()
                || it_m == m_machines.end()
                || it_mdi == m_machineInitialStates.end() ){
            continue;
        }

        const MachineDynamicInfo& mdi = it_mdi->second;
        const Machine &machine = it_m->second;

        AstarPlan initPlan;
        initPlan.plan_cost_total = std::numeric_limits<double>::max();
        initPlan.isOK = false;

        double remainingMass = route.route_points.back().worked_mass - route.route_points.front().worked_mass;
        double remainingVol = route.route_points.back().worked_volume - route.route_points.front().worked_volume;

        std::set<DirectedGraph::vertex_t> resourceVtsSet;
        for(const auto& it1 : resourcePointCapacities){
            auto caps = it1.second;
            if( materialFlowType == MaterialFlowType::NEUTRAL_MATERIAL_FLOW ||
                    ( caps.first < -1e-6 && caps.second < -1e-6) ){
                resourceVtsSet.insert(it1.first);
                continue;
            }
            if( ( caps.first > -1e-6 && caps.first < 1e-6 ) || ( caps.second > -1e-6 && caps.second < 1e-6 ) )
                continue;

            double bunker_mass_after = mdi.bunkerMass;
            double bunker_volume_after = mdi.bunkerVolume;

            auto capK = getMachineBunkerStateAfterResourcePoint(machine, materialFlowType,
                                                                caps.first, caps.second,
                                                                remainingMass, remainingVol,
                                                                bunker_mass_after, bunker_volume_after);
            if(capK > 0.99)//@todo for now, allow only resource points that can supply/receive most of the material
                resourceVtsSet.insert(it1.first);
        }
        std::set<MachineId_t> restrictedMachineIds = {machine.id};

        float bestPlanCapK;

        for(auto vt : resourceVtsSet){

            DirectedGraph::vertex_property vt_prop = graph[vt];


            //successor checkers
            //@todo: we need to add SuccessorTimestamp checkers for the other machines. Problem: we do not know how the timestamps of the following machines will change, hence we need to use the current timestamps
            std::vector<std::shared_ptr<const Astar::ISuccesorChecker> > successorCheckers;
            if(transitRestriction == TransitRestriction::TRANSIT_ONLY_OVER_WORKED_AREAS){
                successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_SuccessorTimestamp>(route.route_points.front().time_stamp,
                                                                                                                  AstarSuccessorChecker_SuccessorTimestamp::INVALID_IF_LOWER_THAN_SUCC_TIMESPAMP,
                                                                                                                  restrictedMachineIds) );
            }
            else if(transitRestriction == TransitRestriction::TRANSIT_ONLY_OVER_UNWORKED_AREAS){
                successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_SuccessorTimestamp>(route.route_points.front().time_stamp,
                                                                                                                  AstarSuccessorChecker_SuccessorTimestamp::INVALID_IF_HIGHER_THAN_SUCC_TIMESPAMP,
                                                                                                                  restrictedMachineIds) );
            }
            successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_VertexExcludeSet>(resourceVtsSet,
                                                                                                             [&vt](const Astar::ISuccesorChecker::IsSuccessorValidParams& p)->bool{ return p.vt_to == vt; }) );

            AstarPlan plan1;

            //compute plan to resource point

            double machine_speed = machine.calcSpeed( mdi.bunkerMass );

            //if the working machines is very close to the resource point, connect it directly
            auto dist = arolib::geometry::calc_dist(vt_prop.route_point, mdi.position);
            if( dist < 2 * machine.working_width){
                RoutePoint rp;
                rp.type = RoutePoint::INITIAL_POSITION;
                rp.time_stamp = mdi.timestamp;
                rp.point() = mdi.position;
                rp.bunker_mass = std::max(0.0, mdi.bunkerMass);
                rp.bunker_volume = std::max(0.0, mdi.bunkerVolume);
                plan1.route_points_.emplace_back(rp);

                rp.point() = vt_prop.route_point.point();
                rp.time_stamp += ( dist / machine_speed );
                plan1.route_points_.emplace_back(rp);
                plan1.plan_cost_total = 0;
            }
            else{
                auto it_vt = graph.initialpoint_vertex_map().find(route.machine_id);
                if(it_vt == graph.initialpoint_vertex_map().end()){
                    sError = "Machine id " + std::to_string(route.machine_id) + " has no initial vertex";
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, sError );
                    return sError;
                }

                //connect using AStar
                Astar::PlanParameters astarParams;
                astarParams.start_vt = it_vt->second;
                astarParams.goal_vt = vt;
                astarParams.start_time = mdi.timestamp;
                astarParams.machine = machine;
                astarParams.machine_speed = machine_speed;
                astarParams.initial_bunker_mass = mdi.bunkerMass;
                astarParams.includeWaitInCost = false;
                astarParams.successorCheckers = successorCheckers;

                const Astar::AStarSettings& astarSettings_original = m_settings;
                Astar::AStarSettings astarSettings = astarSettings_original;
                astarSettings.includeWaitInCost = false;

                auto foldername_planData = m_outputFolder;
                if(!foldername_planData.empty()){
                    foldername_planData += "M" + std::to_string(route.machine_id) + "/init/to_" + std::to_string(vt) + "/";
                    io::create_directory(foldername_planData, true);
                }

                Astar planner (astarParams,
                               astarSettings,
                               RoutePoint::TRANSIT,
                               foldername_planData,
                               loggerPtr());

                if( !planner.plan(graph, m_edgeCostCalculator)
                        || planner.getPlan().plan_cost_total > initPlan.plan_cost_total
                        || planner.getPlan().route_points_.empty()){
                    continue;
                }

                //set again the timestamp of the vertex corresponding to the first route point
                resetTimestampsFromBaseRoute(graph, route, 0, 0, route.route_points.front().time_stamp);

                plan1 = planner.getPlan();
            }

            plan1.route_points_.back().type = RoutePoint::RESOURCE_POINT;


            //add route point corresponding to load/unload at resource point

            RoutePoint rpAtRes = plan1.route_points_.back();

            auto caps = resourcePointCapacities[vt];
            double bunker_mass_after = rpAtRes.bunker_mass, bunker_volume_after = rpAtRes.bunker_volume;
            double remainingMass = route.route_points.back().worked_mass - route.route_points.front().worked_mass;
            double remainingVol = route.route_points.back().worked_volume - route.route_points.front().worked_volume;
            auto capK = getMachineBunkerStateAfterResourcePoint(machine, materialFlowType,
                                                                caps.first, caps.second,
                                                                remainingMass, remainingVol,
                                                                bunker_mass_after, bunker_volume_after);
            if(machine.unloading_speed_mass > 1e-6)
                rpAtRes.time_stamp += (std::fabs( rpAtRes.bunker_mass - bunker_mass_after ) / machine.unloading_speed_mass);
            else if(machine.unloading_speed_volume > 1e-6)
                rpAtRes.time_stamp += (std::fabs( rpAtRes.bunker_volume - bunker_volume_after ) / machine.unloading_speed_volume);
            else
                rpAtRes.time_stamp += 60;//@todo unload time = 60 s is fixed at the moment
            rpAtRes.bunker_mass = bunker_mass_after;
            rpAtRes.bunker_volume = bunker_volume_after;
            plan1.route_points_.push_back(rpAtRes);


            //compute plan to first route point

            auto it_vt_2 = graph.routepoint_vertex_map().find( route.route_points.front() );
            if(it_vt_2 == graph.routepoint_vertex_map().end()){
                sError = "Vertex corresponding to start route point of machine id " + std::to_string(route.machine_id) + " not found in graph's map";
                logger().printOut(LogLevel::ERROR, __FUNCTION__, sError );
                return sError;
            }

            machine_speed = machine.calcSpeed( plan1.route_points_.back().bunker_mass );

            //connect using AStar
            Astar::PlanParameters astarParams;
            astarParams.start_vt = vt;
            astarParams.goal_vt = it_vt_2->second;
            astarParams.start_time = plan1.route_points_.back().time_stamp;
            astarParams.machine = machine;
            astarParams.machine_speed = machine_speed;
            astarParams.initial_bunker_mass = plan1.route_points_.back().bunker_mass;
            astarParams.includeWaitInCost = false;
            astarParams.successorCheckers = successorCheckers;

            astarParams.successorCheckers.emplace_back( std::make_shared<AstarSuccessorChecker_VertexExcludeSet_Exceptions1>( getExcludeVertices_initialRoutes(graph, astarParams.goal_vt) ) );

            const Astar::AStarSettings& astarSettings_original = m_settings;
            Astar::AStarSettings astarSettings = astarSettings_original;
            astarSettings.includeWaitInCost = false;

            auto foldername_planData = m_outputFolder;
            if(!foldername_planData.empty()){
                foldername_planData += "M" + std::to_string(route.machine_id) + "/init/from_" + std::to_string(vt) + "/";
                io::create_directory(foldername_planData, true);
            }

            Astar planner (astarParams,
                           astarSettings,
                           RoutePoint::TRANSIT,
                           foldername_planData,
                           loggerPtr());

            if( !planner.plan(graph, m_edgeCostCalculator)
                    || planner.getPlan().plan_cost_total + plan1.plan_cost_total > initPlan.plan_cost_total
                    || planner.getPlan().route_points_.empty()){
                continue;
            }

            std::swap(initPlan, plan1);
            initPlan.add(planner.getPlan(), true);
            bestPlanCapK = capK;
        }

        if(!initPlan.isOK){
            sError = "Unable to plan initial route segment via resource point for machine id " + std::to_string(route.machine_id);
            logger().printOut(LogLevel::ERROR, __FUNCTION__, sError );
            return sError;
        }

        //compute next working windows for the machines that needed to go to a resource point first
        if(!initPlan.route_points_.empty()){
            PlanData::WorkingWindowInfo ww;
            bool finished;
            auto error = getNextWorkingWindow(materialFlowType, route, 0,
                                              initPlan.route_points_.back().bunker_mass, initPlan.route_points_.back().bunker_volume,
                                              ww, finished);
            if(!error.empty())
                return "Error obtaining initial window for route of machine with id " + std::to_string(route.machine_id) + ": " + error;

            if(!finished)
                plan.nextWorkingWindows[routeInd] = ww;
        }

        std::swap(initPlans[routeInd], initPlan);

    }


    return sError;
}

std::set<DirectedGraph::vertex_t> RoutePlannerStandaloneMachines::getExcludeVertices_initialRoutes(DirectedGraph::Graph &graph, DirectedGraph::vertex_t goal_vt)
{
    std::set<DirectedGraph::vertex_t> exclude;
    for(auto out_edges = boost::out_edges(goal_vt, graph);
        out_edges.first != out_edges.second; out_edges.first++){

        DirectedGraph::vertex_t successor = target(*out_edges.first, graph);

        DirectedGraph::vertex_property successor_prop = graph[successor];
        DirectedGraph::vertex_property goal_prop = graph[goal_vt];

        if(goal_prop.graph_location != successor_prop.graph_location
                || successor_prop.route_point.track_id < 0)
            continue;

        //exclude the vertices that are connected to the goal vertex and belong to an adjacent track
        if( goal_prop.route_point.type != RoutePoint::TRACK_START
                && goal_prop.route_point.type != RoutePoint::TRACK_END
                && std::abs( goal_prop.route_point.track_id - successor_prop.route_point.track_id ) == 1 )
            exclude.insert(successor);

    }

    for(auto& rv_it : graph.resourcepoint_vertex_map())
        exclude.insert(rv_it.second);

    return exclude;
}


bool RoutePlannerStandaloneMachines::getNextTransportationInfo(const PlanData &plan, MaterialFlowType materialFlowType,
                                                               size_t &indRoute, size_t &indRP, size_t &indRP_ret, Machine &machine, PlanData::WorkingWindowInfo& nextWorkingWindow)
{

    bool ret = false;
    double minTime = std::numeric_limits<double>::max();
    for(const auto& it_hw : plan.nextWorkingWindows){
        const PlanData::WorkingWindowInfo& ww = it_hw.second;


        size_t ind = ww.indFinish;
        const auto& route = plan.routes.at(it_hw.first);
        if(ind >= route.route_points.size())
            continue;

        //check the next working window to see if this window is the last one
        PlanData::WorkingWindowInfo ww_next;
        bool finished;
        const auto& m = m_machines.at( route.machine_id );
        auto error = getNextWorkingWindow(materialFlowType, route, ww.indFinish,
                                          materialFlowType == MaterialFlowType::INPUT_MATERIAL_FLOW ? m.bunker_mass : 0, // use best case scenario just to check if there is something left to work
                                          materialFlowType == MaterialFlowType::INPUT_MATERIAL_FLOW ? m.bunker_volume : 0, // use best case scenario just to check if there is something left to work
                                          ww_next, finished);
        if(!error.empty())
            continue;

        auto rp = route.route_points.at(ind);

        if(rp.time_stamp < minTime){
            ret = true;
            minTime = rp.time_stamp;
            indRoute = it_hw.first;
            machine = m_machines.at( route.machine_id );
            nextWorkingWindow = ww;

            indRP = ind;
            indRP_ret = finished ? route.route_points.size()//set it to size so that no return trip is computed
                                 : ww_next.indStart;
        }
    }

    return ret;
}


float RoutePlannerStandaloneMachines::getMachineBunkerStateAfterResourcePoint(const Machine &machine,
                                                                             MaterialFlowType materialFlowType,
                                                                             double resourcePointMassCapacity, double resourcePointVolumeCapacity,
                                                                             double remainingMass, double remainingVol,
                                                                             double& bunker_mass, double& bunker_volume)
{
    float capK = 1;

    if(materialFlowType == MaterialFlowType::INPUT_MATERIAL_FLOW){
        remainingMass = std::max(0.0, remainingMass);
        remainingVol = std::max(0.0, remainingVol);

        double neededMassForFullBunker = std::max(0.0, machine.bunker_mass - bunker_mass);
        double neededVolumeForFullBunker = std::max(0.0, machine.bunker_volume - bunker_volume);

        double requestedMass = std::min(neededMassForFullBunker, remainingMass);
        double requestedVol = std::min(neededVolumeForFullBunker, remainingVol);

        float massK = 1, volK = 1;

        if(requestedMass > 1e-6){
            double supply = std::min(requestedMass, resourcePointMassCapacity);
            massK = 1 - (requestedMass - supply) / requestedMass;
        }
        if(requestedVol > 1e-6){
            double supply = std::min(requestedVol, resourcePointVolumeCapacity);
            volK = 1 - (requestedVol - supply) / requestedVol;
        }

        capK = std::min(massK, volK);

        bunker_mass += requestedMass * capK;
        bunker_volume += requestedVol * capK;

    }
    else if(materialFlowType == MaterialFlowType::OUTPUT_MATERIAL_FLOW){
        double requestedMass = std::max(0.0, bunker_mass);
        double requestedVol = std::max(0.0, bunker_volume);

        float massK = 1, volK = 1;

        if(requestedMass > 1e-6){
            double supply = std::min(requestedMass, resourcePointMassCapacity);
            massK = 1 - (requestedMass - supply) / requestedMass;
        }
        if(requestedVol > 1e-6){
            double supply = std::min(requestedVol, resourcePointVolumeCapacity);
            volK = 1 - (requestedVol - supply) / requestedVol;
        }

        capK = std::min(massK, volK);

        bunker_mass -= std::max(0.0, requestedMass * capK );
        bunker_volume -= std::max(0.0, requestedVol * capK );
    }

    return capK;

}


}
