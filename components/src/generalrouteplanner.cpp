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
 
#include "arolib/components/generalrouteplanner.h"

namespace arolib {

GeneralRoutePlanner::GeneralRoutePlanner(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{

}

AroResp GeneralRoutePlanner::planRouteToPointInSubfield(DirectedGraph::Graph &graph,
                                                        Route &route,
                                                        const Subfield &subfield,
                                                        const Machine &machine,
                                                        const DirectedGraph::vertex_t &goal_vt,
                                                        const GeneralRoutePlanner::PlanParameters &planParameters,
                                                        const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                        std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                                        PlanGeneralInfo *pPlanInfo)
{
    if(!edgeCostCalculator){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return AroResp(1, "Invalid edgeCostCalculator." );
    }

    double startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    auto it_mdi = machineCurrentStates.find(machine.id);
    if(it_mdi == machineCurrentStates.end()){
        m_logger.printError(__FUNCTION__, "No current state found for machine with id " + std::to_string(machine.id));
        return AroResp(1, "No current state found for machine with id " + std::to_string(machine.id));
    }

    AstarPlan plan;
    const MachineDynamicInfo& mdi = it_mdi->second;

    auto it_initP = graph.initialpoint_vertex_map().find(machine.id);

    if(it_initP == graph.initialpoint_vertex_map().end()){
        m_logger.printError(__FUNCTION__, "No initial vertex found in graph for machine with id " + std::to_string(machine.id));
        return AroResp(1, "No initial vertex found in graph for machine with id " + std::to_string(machine.id));
    }

    const DirectedGraph::vertex_property& v_prop = graph[it_initP->second];
    bool machineInside = arolib::geometry::in_polygon( v_prop.route_point, subfield.boundary_outer );

    Astar::PlanParameters astarParams;
    astarParams.start_vt = it_initP->second;
    astarParams.goal_vt = goal_vt;
    astarParams.start_time = 0;
    astarParams.max_time_visit = planParameters.max_time_visit;
    astarParams.max_time_goal = planParameters.max_time_goal;
    astarParams.machine = machine;
    astarParams.machine_speed = machine.calcSpeed( mdi.bunkerMass );
    astarParams.initial_bunker_mass = mdi.bunkerMass;
    astarParams.overload = false;
    astarParams.includeWaitInCost = false;
    astarParams.exclude = planParameters.exclude;
    astarParams.restrictedMachineIds = planParameters.restrictedMachineIds;
    astarParams.restrictedMachineIds_futureVisits = {};//@todo check

    std::string foldername_planData = m_foldername_planData;
    if(!foldername_planData.empty()){
        if(foldername_planData.back() != '/')
            foldername_planData += "/";
        foldername_planData += ( "to_IFvt/" );
    }

    Astar aStar(astarParams,
                planParameters,
                RoutePoint::TRANSIT,
                foldername_planData,
                &m_logger);

    if( !aStar.plan(graph, edgeCostCalculator) ){
        m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Error planning to vertex " + std::to_string(goal_vt));
        return AroResp(1, "Error planning to vertex " + std::to_string(goal_vt));
    }

    plan = aStar.getPlan();
    plan.adjustAccessPoints(machineInside);

    route.machine_id = machine.id;
    route.route_id = 0;
    route.route_points = plan.route_points_;

    double endTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    if(pPlanInfo){
        pPlanInfo->planningDuration = (endTime - startTime) * 1e-3;

        pPlanInfo->planDuration = std::numeric_limits<double>::lowest();
        if(!route.route_points.empty())
            pPlanInfo->planDuration = std::max(pPlanInfo->planDuration, route.route_points.back().time_stamp);
        pPlanInfo->planDuration = std::max(pPlanInfo->planDuration, 0.0);

        pPlanInfo->planOverallCost = plan.plan_cost_total;
    }

    return AroResp::ok();
}

AroResp GeneralRoutePlanner::planRouteToPointInSubfield(DirectedGraph::Graph &graph, Route &route,
                                                        PlanningWorkspace &pw, size_t subfieldIdx,
                                                        const MachineId_t &machineId,
                                                        const DirectedGraph::vertex_t &goal_vt,
                                                        const GeneralRoutePlanner::PlanParameters &planParameters,
                                                        std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                                        PlanGeneralInfo *pPlanInfo)
{
    if(!edgeCostCalculator){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return AroResp(1, "Invalid edgeCostCalculator." );
    }
    if(subfieldIdx >= getField(pw).subfields.size()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfield index");
        return AroResp(1, "Invalid subfield index");
    }
    const auto &subfield = getField(pw).subfields.at(subfieldIdx);
    const auto &machineCurrentStates = getMachineCurrentStates(pw);
    const auto &allMachines = getMachines(pw);

    bool machineFound = false;
    Machine machine;
    for(const auto& m : allMachines){
        if(m.id == machineId){
            machine = m;
            machineFound = true;
            break;
        }
    }
    if(!machineFound){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Machine not found in working group");
        return AroResp(1, "Machine not found in working group");
    }

    m_planningWorkspace = &pw;
    m_pw_subfieldIdx = subfieldIdx;

    auto ret = planRouteToPointInSubfield( graph,
                                           route,
                                           subfield,
                                           machine,
                                           goal_vt,
                                           planParameters,
                                           machineCurrentStates,
                                           edgeCostCalculator,
                                           pPlanInfo );

    m_planningWorkspace = nullptr;

    return ret;
}

AroResp GeneralRoutePlanner::planRouteToPointInSubfield(DirectedGraph::Graph &graph,
                                                        Route &route,
                                                        const Subfield &subfield,
                                                        const Machine &machine,
                                                        const Point &goal,
                                                        const PlanParameters &planParameters,
                                                        const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                        std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                                        double searchRadius,
                                                        PlanGeneralInfo *pPlanInfo)
{
    if(!edgeCostCalculator){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return AroResp(1, "Invalid edgeCostCalculator." );
    }

    if(searchRadius < -1e-6)
        searchRadius = 1.5 * std::max( {machine.workingRadius(),
                                        graph.workingWidth_HL(),
                                        graph.workingWidth_IF()} );
    else if (searchRadius < 1e-6)
        searchRadius = std::numeric_limits<double>::max();


    double timestampLimit = ( machine.isOfWorkingType(true) ? 1e-3 : -1e-5 );

    auto isVtValid = [&](const DirectedGraph::vertex_t&, const DirectedGraph::vertex_property& v_prop)->bool
    {
        const RoutePoint& rp2 = v_prop.route_point;
        if (rp2.isOfTypeWorking(true, true) && rp2.time_stamp > timestampLimit)
            return false;

        return true;
    };

    auto vts = graph.getVerticesInRadius(goal, searchRadius, isVtValid);
    if(vts.empty()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No valid vertices found around the goal location withing a radius of " + std::to_string(searchRadius));
        return AroResp(1, "No valid vertices found around the goal location withing a radius of " + std::to_string(searchRadius));
    }

    std::sort(vts.begin(), vts.end(), [&](const DirectedGraph::vertex_t& vt1, const DirectedGraph::vertex_t& vt2)->bool{
        const DirectedGraph::vertex_property& vt_prop1 = graph[vt1];
        const DirectedGraph::vertex_property& vt_prop2 = graph[vt2];
        return arolib::geometry::calc_dist(goal, vt_prop1.route_point) < arolib::geometry::calc_dist(goal, vt_prop2.route_point);
    });

    size_t attempt = 1;
    for( const auto& goal_vt : vts ){
        auto folderTmp = m_foldername_planData;
        if(!m_foldername_planData.empty()){
            if(m_foldername_planData.back() != '/')
                m_foldername_planData += "/";
            m_foldername_planData += ( "M" + std::to_string(machine.id) + "_attempt" + std::to_string(attempt++) );
        }
        auto aroResp = planRouteToPointInSubfield( graph,
                                                   route,
                                                   subfield,
                                                   machine,
                                                   goal_vt,
                                                   planParameters,
                                                   machineCurrentStates,
                                                   edgeCostCalculator,
                                                   pPlanInfo );
        m_foldername_planData = folderTmp;

        if(aroResp.isOK())
            return aroResp;

        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "No plan found for goal vertex " + std::to_string(goal_vt) + ". Trying for next closest valid vertex if existent.");

    }

    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No plan found for any of the valid vertices withing a radius of " + std::to_string(searchRadius));
    return AroResp(1, "No plan found for any of the valid vertices withing a radius of " + std::to_string(searchRadius));
}

AroResp GeneralRoutePlanner::planRouteToPointInSubfield(DirectedGraph::Graph &graph,
                                                        Route &route,
                                                        PlanningWorkspace &pw,
                                                        size_t subfieldIdx,
                                                        const MachineId_t &machineId,
                                                        const Point &goal,
                                                        const PlanParameters &planParameters,
                                                        std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                                        double searchRadius,
                                                        PlanGeneralInfo *pPlanInfo)
{
    if(!edgeCostCalculator){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return AroResp(1, "Invalid edgeCostCalculator." );
    }
    if(subfieldIdx >= getField(pw).subfields.size()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfield index");
        return AroResp(1, "Invalid subfield index");
    }
    const auto &subfield = getField(pw).subfields.at(subfieldIdx);
    const auto &machineCurrentStates = getMachineCurrentStates(pw);
    const auto &allMachines = getMachines(pw);

    bool machineFound = false;
    Machine machine;
    for(const auto& m : allMachines){
        if(m.id == machineId){
            machine = m;
            machineFound = true;
            break;
        }
    }
    if(!machineFound){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Machine not found in working group");
        return AroResp(1, "Machine not found in working group");
    }

    m_planningWorkspace = &pw;
    m_pw_subfieldIdx = subfieldIdx;

    auto ret = planRouteToPointInSubfield( graph,
                                           route,
                                           subfield,
                                           machine,
                                           goal,
                                           planParameters,
                                           machineCurrentStates,
                                           edgeCostCalculator,
                                           searchRadius,
                                           pPlanInfo );

    m_planningWorkspace = nullptr;

    return ret;
}

AroResp GeneralRoutePlanner::planRouteToExitPoint(DirectedGraph::Graph &graph,
                                                  std::vector<Route> &routes,
                                                  const Subfield &subfield,
                                                  const std::vector<Machine>& machines,
                                                  const std::set<FieldAccessPointId_t> &fapIds,
                                                  const PlanParameters &planParameters,
                                                  const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                  std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                                  PlanGeneralInfo *pPlanInfo)
{
    if(!edgeCostCalculator){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return AroResp(1, "Invalid edgeCostCalculator." );
    }

    std::map<MachineId_t, Machine> machinesMap;
    std::map<MachineId_t, AstarPlan> plans;
    MachineDynamicInfo mdi;

    double startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    for(auto& m : machines){//check for current state of the machines
        auto it_mdi = machineCurrentStates.find(m.id);
        if(it_mdi != machineCurrentStates.end())
            mdi = it_mdi->second;
//        else{
//            m_logger.printError(__FUNCTION__, "No current state found for machine with id " + std::to_string(m.id));
//            return AroResp(1, "No current state found for machine with id " + std::to_string(m.id));
//        }
        machinesMap[m.id] = m;
    }

    std::vector<DirectedGraph::vertex_t> goalVts;
    for(auto& it_vts : graph.accesspoint_vertex_map()){
        const FieldAccessPoint& fap = it_vts.first;
        if(fap.accessType != FieldAccessPoint::AP_ENTRY_ONLY
                && (fapIds.empty() || fapIds.find(fap.id) != fapIds.end()))
            goalVts.emplace_back(it_vts.second);
    }
    if(goalVts.empty()){
        m_logger.printError(__FUNCTION__, "No valid exit points found in graph");
        return AroResp(1, "No valid exit vertices found in graph");
    }

    for(auto& it_m : machinesMap){
        Machine& m = it_m.second;
        auto it_initP = graph.initialpoint_vertex_map().find(m.id);

        if(it_initP == graph.initialpoint_vertex_map().end()){
            m_logger.printError(__FUNCTION__, "No initial vertex found in graph for machine with id " + std::to_string(m.id));
            return AroResp(1, "No initial vertex found in graph for machine with id " + std::to_string(m.id));
        }

        const DirectedGraph::vertex_property& v_prop = graph[it_initP->second];
        bool machineInside = arolib::geometry::in_polygon( v_prop.route_point, subfield.boundary_outer );

        bool found_plan = false;
        double min_cost = std::numeric_limits<double>::max();
        AstarPlan& plan = plans[m.id];
        for(auto & goal_vt : goalVts){
            Astar::PlanParameters astarParams;
            astarParams.start_vt = it_initP->second;
            astarParams.goal_vt = goal_vt;
            astarParams.start_time = 0;
            astarParams.max_time_visit = std::numeric_limits<double>::max(); //planParameters.max_time_visit;
            astarParams.max_time_goal = planParameters.max_time_goal;
            astarParams.machine = m;
            astarParams.machine_speed = m.calcSpeed( mdi.bunkerMass );
            astarParams.initial_bunker_mass = mdi.bunkerMass;
            astarParams.overload = false;
            astarParams.includeWaitInCost = false;
            astarParams.exclude = planParameters.exclude;
            astarParams.restrictedMachineIds = planParameters.restrictedMachineIds;
            astarParams.restrictedMachineIds_futureVisits = {};//@todo check

            std::string foldername_planData = m_foldername_planData;
            if(!foldername_planData.empty()){
                if(foldername_planData.back() != '/')
                    foldername_planData += "/";
                foldername_planData += ( "M" + std::to_string(m.id) + "/to_fap/" );
            }

            Astar aStar(astarParams,
                        planParameters,
                        RoutePoint::TRANSIT,
                        foldername_planData,
                        &m_logger);

            if( !aStar.plan(graph, edgeCostCalculator) ){
                m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Error calling astar directly to access-point vertex " + std::to_string(goal_vt));
                continue;
            }


            if (aStar.getPlan().plan_cost_total > min_cost)
                continue;

            plan = aStar.getPlan();
            min_cost = plan.plan_cost_total;

            found_plan = true;

        }
        if(!found_plan){
            m_logger.printError(__FUNCTION__, "No plan to any of the resource points was found for machine with id " + std::to_string(m.id));
            return AroResp(1, "No plan to any of the resource points was found for machine with id " + std::to_string(m.id));
        }

        plan.adjustAccessPoints(machineInside);

        routes.emplace_back( Route() );
        routes.back().machine_id = m.id;
        routes.back().route_id = routes.size()-1;
        routes.back().route_points = plan.route_points_;
    }

    double endTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    if(pPlanInfo){
        pPlanInfo->planningDuration = (endTime - startTime) * 1e-3;

        pPlanInfo->planDuration = std::numeric_limits<double>::lowest();
        for(auto& route : routes){
            if(!route.route_points.empty())
                pPlanInfo->planDuration = std::max(pPlanInfo->planDuration, route.route_points.back().time_stamp);
        }
        pPlanInfo->planDuration = std::max(pPlanInfo->planDuration, 0.0);

        pPlanInfo->planOverallCost = 0;
        for(auto& plan : plans)
            pPlanInfo->planOverallCost += plan.second.plan_cost_total;
    }

    return AroResp::ok();
}

AroResp GeneralRoutePlanner::planRouteToExitPoint(DirectedGraph::Graph &graph,
                                                  std::vector<Route> &routes,
                                                  PlanningWorkspace &pw,
                                                  size_t subfieldIdx,
                                                  const std::set<MachineId_t> &machineIds, const std::set<FieldAccessPointId_t> &fapIds,
                                                  const PlanParameters &planParameters,
                                                  std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                                  PlanGeneralInfo *pPlanInfo)
{
    if(!edgeCostCalculator){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return AroResp(1, "Invalid edgeCostCalculator." );
    }
    if(subfieldIdx >= getField(pw).subfields.size()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfield index");
        return AroResp(1, "Invalid subfield index");
    }
    const auto &subfield = getField(pw).subfields.at(subfieldIdx);
    const auto &machineCurrentStates = getMachineCurrentStates(pw);
    const auto &allMachines = getMachines(pw);
    std::vector<Machine> machines;
    for(const auto& m : allMachines){
        if(machineIds.find(m.id) != machineIds.end())
            machines.emplace_back(m);
    }

    m_planningWorkspace = &pw;
    m_pw_subfieldIdx = subfieldIdx;

    auto ret = planRouteToExitPoint( graph,
                                     routes,
                                     subfield,
                                     machines,
                                     fapIds,
                                     planParameters,
                                     machineCurrentStates,
                                     edgeCostCalculator,
                                     pPlanInfo );

    m_planningWorkspace = nullptr;

    return ret;
}

AroResp GeneralRoutePlanner::planRouteToResourcePoint(DirectedGraph::Graph &graph, std::vector<Route> &routes,
                                                      const Subfield &subfield,
                                                      const std::vector<Machine> &machines,
                                                      const PlanParameters &planParameters,
                                                      const std::set<ResourcePointId_t> &resourcePointIds,
                                                      const std::set<ResourcePoint::ResourceType> &resourceTypes,
                                                      const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                      std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                                      PlanGeneralInfo *pPlanInfo)
{
    if(!edgeCostCalculator){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return AroResp(1, "Invalid edgeCostCalculator." );
    }

    std::map<MachineId_t, Machine> machinesMap;
    std::map<MachineId_t, AstarPlan> plans;
    MachineDynamicInfo mdi;

    double startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    for(auto& m : machines){//check for current state of the machines
        auto it_mdi = machineCurrentStates.find(m.id);
        if(it_mdi != machineCurrentStates.end())
            mdi = it_mdi->second;
//        else{
//            m_logger.printError(__FUNCTION__, "No current state found for machine with id " + std::to_string(m.id));
//            return AroResp(1, "No current state found for machine with id " + std::to_string(m.id));
//        }
        machinesMap[m.id] = m;
    }

    std::vector<DirectedGraph::vertex_t> goalVts;
    for(auto& it_vts : graph.resourcepoint_vertex_map()){
        const ResourcePoint& resP = it_vts.first;
        bool typeOK = false;
        if(!resourcePointIds.empty() && resourcePointIds.find(resP.id) == resourcePointIds.end())
            continue;
        for(auto resType : resourceTypes){
            if(resP.resourceTypes.find(resType) != resP.resourceTypes.end()){
                typeOK = true;
                break;
            }
        }
        if(typeOK)
            goalVts.emplace_back(it_vts.second);
    }
    if(goalVts.empty()){
        m_logger.printError(__FUNCTION__, "No valid resource vertices found in graph");
        return AroResp(1, "No valid resource vertices found in graph");
    }

    for(auto& it_m : machinesMap){
        Machine& m = it_m.second;
        auto it_initP = graph.initialpoint_vertex_map().find(m.id);

        if(it_initP == graph.initialpoint_vertex_map().end()){
            m_logger.printError(__FUNCTION__, "No initial vertex found in graph for machine with id " + std::to_string(m.id));
            return AroResp(1, "No initial vertex found in graph for machine with id " + std::to_string(m.id));
        }

        const DirectedGraph::vertex_property& v_prop = graph[it_initP->second];
        bool machineInside = arolib::geometry::in_polygon( v_prop.route_point, subfield.boundary_outer );

        bool found_plan = false;
        double min_cost = std::numeric_limits<double>::max();
        AstarPlan& plan = plans[m.id];
        for(auto & goal_vt : goalVts){
            Astar::PlanParameters astarParams;
            astarParams.start_vt = it_initP->second;
            astarParams.goal_vt = goal_vt;
            astarParams.start_time = 0;
            astarParams.max_time_visit = std::numeric_limits<double>::max(); //planParameters.max_time_visit;
            astarParams.max_time_goal = planParameters.max_time_goal;
            astarParams.machine = m;
            astarParams.machine_speed = m.calcSpeed( mdi.bunkerMass );
            astarParams.initial_bunker_mass = mdi.bunkerMass;
            astarParams.overload = false;
            astarParams.includeWaitInCost = false;
            astarParams.exclude = planParameters.exclude;
            astarParams.restrictedMachineIds = planParameters.restrictedMachineIds;
            astarParams.restrictedMachineIds_futureVisits = {};//@todo check

            std::string foldername_planData = m_foldername_planData;
            if(!foldername_planData.empty()){
                if(foldername_planData.back() != '/')
                    foldername_planData += "/";
                foldername_planData += ( "M" + std::to_string(m.id) + "/to_res/" );
            }

            Astar aStar(astarParams,
                        planParameters,
                        RoutePoint::TRANSIT,
                        foldername_planData,
                        &m_logger);

            if( !aStar.plan(graph, edgeCostCalculator) ){
                m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Error calling astar directly to resource vertex " + std::to_string(goal_vt));
                continue;
            }


            if (aStar.getPlan().plan_cost_total > min_cost)
                continue;

            plan = aStar.getPlan();
            min_cost = plan.plan_cost_total;

            found_plan = true;

        }
        if(!found_plan){
            m_logger.printError(__FUNCTION__, "No plan to any of the resource points was found for machine with id " + std::to_string(m.id));
            return AroResp(1, "No plan to any of the resource points was found for machine with id " + std::to_string(m.id));
        }

        plan.adjustAccessPoints(machineInside);

        routes.emplace_back( Route() );
        routes.back().machine_id = m.id;
        routes.back().route_id = routes.size()-1;
        routes.back().route_points = plan.route_points_;
    }

    double endTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    if(pPlanInfo){
        pPlanInfo->planningDuration = (endTime - startTime) * 1e-3;

        pPlanInfo->planDuration = std::numeric_limits<double>::lowest();
        for(auto& route : routes){
            if(!route.route_points.empty())
                pPlanInfo->planDuration = std::max(pPlanInfo->planDuration, route.route_points.back().time_stamp);
        }
        pPlanInfo->planDuration = std::max(pPlanInfo->planDuration, 0.0);

        pPlanInfo->planOverallCost = 0;
        for(auto& plan : plans)
            pPlanInfo->planOverallCost += plan.second.plan_cost_total;
    }

    return AroResp::ok();
}

AroResp GeneralRoutePlanner::planRouteToResourcePoint(DirectedGraph::Graph &graph,
                                                      std::vector<Route> &routes,
                                                      PlanningWorkspace &pw,
                                                      size_t subfieldIdx,
                                                      const std::set<MachineId_t> &machineIds,
                                                      const PlanParameters &planParameters,
                                                      const std::set<ResourcePointId_t> &resourcePointIds,
                                                      const std::set<ResourcePoint::ResourceType> &resourceTypes,
                                                      std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                                      PlanGeneralInfo *pPlanInfo)
{
    if(!edgeCostCalculator){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return AroResp(1, "Invalid edgeCostCalculator." );
    }
    if(subfieldIdx >= getField(pw).subfields.size()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfield index");
        return AroResp(1, "Invalid subfield index");
    }
    const auto &subfield = getField(pw).subfields.at(subfieldIdx);
    const auto &machineCurrentStates = getMachineCurrentStates(pw);
    const auto &allMachines = getMachines(pw);
    std::vector<Machine> machines;
    for(const auto& m : allMachines){
        if(machineIds.find(m.id) != machineIds.end())
            machines.emplace_back(m);
    }

    m_planningWorkspace = &pw;
    m_pw_subfieldIdx = subfieldIdx;

    auto ret = planRouteToResourcePoint( graph,
                                         routes,
                                         subfield,
                                         machines,
                                         planParameters,
                                         resourcePointIds,
                                         resourceTypes,
                                         machineCurrentStates,
                                         edgeCostCalculator,
                                         pPlanInfo );

    m_planningWorkspace = nullptr;

    return ret;
}

void GeneralRoutePlanner::setOutputFiles(const std::string &foldername_planData,
                                         const std::string &foldername_visitSchedule)
{
    m_foldername_planData = foldername_planData;
    m_foldername_visitSchedule = foldername_visitSchedule;
}


}
