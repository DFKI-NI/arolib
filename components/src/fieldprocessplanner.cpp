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
 
#include "arolib/components/fieldprocessplanner.h"

namespace arolib {

bool FieldProcessPlanner::PlannerParameters::parseFromStringMap(FieldProcessPlanner::PlannerParameters &params, const std::map<std::string, std::string> &map, bool strict)
{
    FieldProcessPlanner::PlannerParameters tmp;

    try{

        if( !FieldGeneralParameters::parseFromStringMap(tmp, map, strict) )
            return false;
        if( !GridComputationSettings::parseFromStringMap(tmp, map, strict) )
            return false;
        if( !MultiOLVPlanner::PlannerSettings::parseFromStringMap(tmp, map, strict) )
            return false;
        if( !MultiHarvPlanner::PlannerSettings::parseFromStringMap(tmp, map, strict) )
            return false;

        std::map<std::string, double*> dMap = { {"unloading_offset" , &tmp.unloading_offset},
                                                {"workingWidth" , &tmp.workingWidth} };
        if( !setValuesFromStringMap( map, dMap, strict) )
            return false;

    } catch(...){ return false; }

    params = tmp;

    return true;

}

std::map<std::string, std::string> FieldProcessPlanner::PlannerParameters::parseToStringMap(const FieldProcessPlanner::PlannerParameters &params)
{
    std::map<std::string, std::string> ret, subMap;
    subMap = FieldGeneralParameters::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );
    subMap = GridComputationSettings::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );
    subMap = MultiOLVPlanner::PlannerSettings::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );
    subMap = MultiHarvPlanner::PlannerSettings::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );

    ret["unloading_offset"] = double2string( params.unloading_offset );
    ret["workingWidth"] = double2string( params.workingWidth );

    return ret;

}

FieldProcessPlanner::FieldProcessPlanner(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{

}

AroResp FieldProcessPlanner::planSubfield(const Subfield &subfield,
                                            const std::vector<Route> &baseRoutes_infield,
                                            const std::vector<HeadlandRoute> &baseRoutes_headland,
                                            std::vector<Route> &plannedRoutes,
                                            const std::vector<Machine> &machines,
                                            OutFieldInfo &outFieldInfo,
                                            const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                            FieldProcessPlanner::PlannerParameters plannerParameters,
                                            const ArolibGrid_t &yieldmap,
                                            const ArolibGrid_t &remainingAreaMap,
                                            std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                            PlanGeneralInfo *pPlanInfo,
                                            DirectedGraph::Graph *pGraph)
{

    if(!edgeCostCalculator){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return AroResp(1, "Invalid edgeCostCalculator." );
    }

    plannedRoutes.clear();
    AroResp compResp;

    if(plannerParameters.harvestedMassLimit < -1e-6)
        plannerParameters.harvestedMassLimit = getHarvestedMassLimit(subfield, machines, plannerParameters, yieldmap, remainingAreaMap);

    LoggingComponent::LoggersHandler lh(true);//will be reset on destruction
    LoggingComponent::setTemporalLoggersParent(lh, *this, yieldmap, remainingAreaMap);
    if(pGraph)
        LoggingComponent::setTemporalLoggersParent(lh, *this, *pGraph);

    try {

        if (subfield.resource_points.empty()) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No ressource points given." );
            return AroResp(1, "No resource points given." );
        }
        if (baseRoutes_infield.empty() && baseRoutes_headland.empty()) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Routes missing, needed by the planner" );
            return AroResp(1, "Routes missing, needed by the planner" );
        }
        if (plannerParameters.workingWidth <= 0) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid working width" );
            return AroResp(1, "Invalid working width" );
        }
        if (plannerParameters.avgMassPerArea < 0) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid average mass/area" );
            return AroResp(1, "Invalid default yield" );
        }

        for(auto &rp : subfield.resource_points){
            OutFieldInfo::UnloadingCosts uc( rp.defaultUnloadingTime, rp.defaultUnloadingTimePerKg );
            if( outFieldInfo.addDefaultUnloadingCosts(rp.id, uc, false) )
                m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Adding default unloading costs for all machines for resource point " + std::to_string(rp.id) );
        }

        std::vector<Route> baseRoutes_infield_ed = baseRoutes_infield;
        std::vector<HeadlandRoute> baseRoutes_headland_ed = baseRoutes_headland;

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Pre-processing base routes...");
        preProcessBaseRoutes(baseRoutes_infield_ed,
                             baseRoutes_headland_ed,
                             subfield,
                             machines,
                             outFieldInfo,
                             machineCurrentStates,
                             remainingAreaMap,
                             plannerParameters.bePreciseWithRemainingAreaMap);

        bool incVisitPeriodsInBuilding = false;//the visitPeriods should be addad once the connected base/main routes (headland -> infield) are available
        //bool incVisitPeriodsInBuilding = plannerParameters.collisionAvoidanceOption != Astar::WITHOUT_COLLISION_AVOIDANCE;
        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Connecting the headland and infield base routes...");
        std::vector<Route> mainRoutes = connectBaseRoutes(baseRoutes_infield_ed,
                                                                    baseRoutes_headland_ed,
                                                                    subfield,
                                                                    machines);

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Removing route points where there is nothing to work on...");
        auto removedWorkedPoints = removeInitialUselessPoints(mainRoutes);

        if(mainRoutes.empty()){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No base/main routes remain after pre-planning processing. Posible reason: there is nothing to harvest.");
            return AroResp(1, "No base/main routes remain after pre-planning processing. Posible reason: there is nothing to harvest." );
        }



        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Building the graph...");
        DirectedGraph::GraphBuilder_TracksBased graphBuilder(m_logger.logLevel());
        graphBuilder.logger().setParent(&m_logger);
        DirectedGraph::Graph* graph;
        DirectedGraph::Graph graphTmp;
        graphTmp.logger().setParent(&m_logger);

        if(pGraph)
            graph = pGraph;
        else
            graph = &graphTmp;

        if(m_planningWorkspace){
            if(m_pw_subfieldIdx >= getField(*m_planningWorkspace).subfields.size())
                throw std::invalid_argument( std::string(__FUNCTION__) + ": Invalid subfield index" );

            const Subfield& sf = getField(*m_planningWorkspace).subfields.at(m_pw_subfieldIdx);

            *graph = graphBuilder.buildGraph(sf,
                                             mainRoutes,
                                             sf.resource_points,
                                             sf.access_points,
                                             outFieldInfo,
                                             getMachines(*m_planningWorkspace),
                                             getMachineCurrentStates(*m_planningWorkspace),
                                             plannerParameters.workingWidth,
                                             plannerParameters.workingWidth,
                                             incVisitPeriodsInBuilding,
                                             m_filename_graphBuilding);
        }
        else{
            *graph = graphBuilder.buildGraph(subfield,
                                             mainRoutes,
                                             subfield.resource_points,
                                             subfield.access_points,
                                             outFieldInfo,
                                             machines,
                                             machineCurrentStates,
                                             plannerParameters.workingWidth,
                                             plannerParameters.workingWidth,
                                             incVisitPeriodsInBuilding,
                                             m_filename_graphBuilding);
        }


        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Adding initial paths to the main routes...");
        auto initMainRoutes = getInitialPathToWorkingRoutes(*graph,
                                                            mainRoutes,
                                                            subfield,
                                                            machines,
                                                            machineCurrentStates,
                                                            plannerParameters,
                                                            edgeCostCalculator);

        std::vector<Route> main_routes, sec_routes;

        if(removedWorkedPoints > 0)//@todo find a better way to check if adding these overruns is necesary, depending on the cost calculator
            addRoutesOverruns(*graph, machines, mainRoutes);

        size_t numOlvs = 0;
        for(auto &m : machines){
            if(m.machinetype == Machine::OLV)
                ++numOlvs;
        }
        DirectedGraph::Graph updated_graph;
        if(numOlvs == 0){
            DirectedGraph::Graph updated_graph;
            auto aroResp = do_multiHarvPlanning(*graph,
                                                mainRoutes,
                                                machines,
                                                machineCurrentStates,
                                                plannerParameters,
                                                edgeCostCalculator,
                                                m_logger,
                                                main_routes,
                                                updated_graph,
                                                pPlanInfo);
            if(!aroResp.isOK())
                return aroResp;
        }
        else{
            if(!incVisitPeriodsInBuilding && plannerParameters.collisionAvoidanceOption != Astar::WITHOUT_COLLISION_AVOIDANCE){
                m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Adding visit periods of working machines to vertices...");
                addWMVisitPeriods(*graph, mainRoutes, machines);
            }

            auto aroResp = this->do_multiOLVPlanning(*graph,
                                                    mainRoutes,
                                                    machines,
                                                    machineCurrentStates,
                                                    plannerParameters,
                                                    edgeCostCalculator,
                                                    m_logger,
                                                    main_routes,
                                                    sec_routes,
                                                    updated_graph,
                                                    pPlanInfo
                                                );
            if(!aroResp.isOK())
                return aroResp;
        }
        *graph = updated_graph;

        //this must be done after the multiOLVPlanner plans, because it assumes that the main (harvester) routes do not contain this initial path!
        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Adding initial paths to the main routes...");
        addInitialPathToMainRoutes(initMainRoutes, main_routes);

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Adding final paths to the main routes...");
        bool useSearchToExit = true;

        sendWorkingMachinesToExitPoints(*graph,
                                        main_routes,
                                        subfield,
                                        machines,
                                        plannerParameters,
                                        edgeCostCalculator,
                                        useSearchToExit);

        plannedRoutes.insert( plannedRoutes.end(), main_routes.begin(), main_routes.end() );
        plannedRoutes.insert( plannedRoutes.end(), sec_routes.begin(), sec_routes.end() );

        if(pPlanInfo){
            double maxTimestamp = std::numeric_limits<double>::lowest();{
                for(auto &route: plannedRoutes){
                    if(!route.route_points.empty())
                        maxTimestamp = std::max(maxTimestamp, route.route_points.back().time_stamp);
                }
            }
            pPlanInfo->planDuration = std::max(maxTimestamp, 0.0);
        }

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Planning finished. Found " + std::to_string(sec_routes.size()) + " olv routes");
        return AroResp(0, "OK" );

    }
    catch (std::exception &e) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what() );
        return AroResp(1, "Exception cought: ", e.what() );
    }


}

AroResp FieldProcessPlanner::planSubfield(PlanningWorkspace &pw, size_t subfieldIdx, const FieldProcessPlanner::PlannerParameters &plannerParameters, std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator, PlanGeneralInfo *pPlanInfo, DirectedGraph::Graph *pGraph)
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
    const auto &machines = getMachines(pw);
    const auto &machineCurrentStates = getMachineCurrentStates(pw);
    const auto &mainRoutes_headland = getBaseRoutes_headland(pw)[subfieldIdx];
    const auto &mainRoutes_infield = getBaseRoutes_infield(pw)[subfieldIdx];
    const auto &yieldmap = getMaps(pw).at( PlanningWorkspace::GridType::YIELD );
    const auto &remainingAreaMap = getMaps(pw).at( PlanningWorkspace::GridType::REMAINING_AREA );
    auto &outFieldInfo = getOutFieldInfo(pw);
    auto &plannedRoutes = getPlannedRoutes(pw)[subfieldIdx];

    m_planningWorkspace = &pw;
    m_pw_subfieldIdx = subfieldIdx;

    auto ret = planSubfield(subfield,
                            mainRoutes_infield,
                            mainRoutes_headland,
                            plannedRoutes,
                            machines,
                            outFieldInfo,
                            machineCurrentStates,
                            plannerParameters,
                            yieldmap,
                            remainingAreaMap,
                            edgeCostCalculator,
                            pPlanInfo,
                            pGraph);

    m_planningWorkspace = nullptr;

    return ret;
}

AroResp FieldProcessPlanner::planSubfield(DirectedGraph::Graph &graph,
                                            const Subfield &subfield,
                                            const std::vector<Route> &baseRoutes_,
                                            std::vector<Route> &plannedRoutes,
                                            const std::vector<Machine> &machines,
                                            const OutFieldInfo &outFieldInfo,
                                            const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                            FieldProcessPlanner::PlannerParameters plannerParameters,
                                            const ArolibGrid_t &yieldmap,
                                            const ArolibGrid_t &remainingAreaMap,
                                            std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                            PlanGeneralInfo *pPlanInfo)
{
    if(!edgeCostCalculator){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return AroResp(1, "Invalid edgeCostCalculator." );
    }
    plannedRoutes.clear();
    AroResp compResp;

    if(plannerParameters.harvestedMassLimit < -1e-6)
        plannerParameters.harvestedMassLimit = getHarvestedMassLimit(subfield, machines, plannerParameters, yieldmap, remainingAreaMap);

    LoggingComponent::LoggersHandler lh(true);//will be reset on destruction
    LoggingComponent::setTemporalLoggersParent(lh, *this, yieldmap, remainingAreaMap, graph);

    try {

        if (subfield.resource_points.empty()) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No ressource points given." );
            return AroResp(1, "No resource points given." );
        }
        if (baseRoutes_.empty()) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Routes missing, needed by the planner" );
            return AroResp(1, "Routes missing, needed by the planner" );
        }

        auto baseRoutes = baseRoutes_;

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Adding initial paths to the base/main routes...");
        auto initBaseRoutes = getInitialPathToWorkingRoutes(graph,
                                                            baseRoutes,
                                                            subfield,
                                                            machines,
                                                            machineCurrentStates,
                                                            plannerParameters,
                                                            edgeCostCalculator);

        std::vector<Route> mainRoutes, sec_routes;

        addRoutesOverruns(graph, machines, baseRoutes);

        size_t numOlvs = 0;
        for(auto &m : machines){
            if(m.machinetype == Machine::OLV)
                ++numOlvs;
        }
        DirectedGraph::Graph updated_graph;
        if(numOlvs == 0){
            auto aroResp = do_multiHarvPlanning(graph,
                                                baseRoutes,
                                                machines,
                                                machineCurrentStates,
                                                plannerParameters,
                                                edgeCostCalculator,
                                                m_logger,
                                                mainRoutes,
                                                updated_graph,
                                                pPlanInfo);
            if(!aroResp.isOK())
                return aroResp;
        }
        else{
            auto aroResp = this->do_multiOLVPlanning(
                graph,
                baseRoutes,
                machines,
                machineCurrentStates,
                plannerParameters,
                edgeCostCalculator,
                m_logger,
                mainRoutes,
                sec_routes,
                updated_graph,
                pPlanInfo
            );
            if(!aroResp.isOK())
                return aroResp;
        }
        graph = updated_graph;

        //this must be done after the multiOLVPlanner plans, because it assumes that the main routes do not contain this initial path!
        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Adding initial paths to the main routes...");
        addInitialPathToMainRoutes(initBaseRoutes, mainRoutes);

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Adding final paths to the main routes...");
        bool useSearchToExit = true;

        sendWorkingMachinesToExitPoints(graph,
                                        mainRoutes,
                                        subfield,
                                        machines,
                                        plannerParameters,
                                        edgeCostCalculator,
                                        useSearchToExit);

        plannedRoutes.insert( plannedRoutes.end(), mainRoutes.begin(), mainRoutes.end() );
        plannedRoutes.insert( plannedRoutes.end(), sec_routes.begin(), sec_routes.end() );

        if(pPlanInfo){
            double maxTimestamp = std::numeric_limits<double>::lowest();{
                for(auto &route: plannedRoutes){
                    if(!route.route_points.empty())
                        maxTimestamp = std::max(maxTimestamp, route.route_points.back().time_stamp);
                }
            }
            pPlanInfo->planDuration = std::max(maxTimestamp, 0.0);
        }

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Planning finished. Found " + std::to_string(sec_routes.size()) + " olv routes");
        return AroResp(0, "OK" );

    }
    catch (std::exception &e) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what() );
        return AroResp(1, "Exception cought: ", e.what() );
    }
}

AroResp FieldProcessPlanner::planSubfield(DirectedGraph::Graph &graph, PlanningWorkspace &pw, size_t subfieldIdx, const FieldProcessPlanner::PlannerParameters &plannerParameters, std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator, PlanGeneralInfo *pPlanInfo)
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
    const auto &machines = getMachines(pw);
    const auto &machineCurrentStates = getMachineCurrentStates(pw);
    const auto &baseRoutes = getConnectedBaseRoutes(pw)[subfieldIdx];
    auto &outFieldInfo = getOutFieldInfo(pw);
    auto &plannedRoutes = getPlannedRoutes(pw)[subfieldIdx];
    const auto &yieldmap = getMaps(pw).at( PlanningWorkspace::GridType::YIELD );
    const auto &remainingAreaMap = getMaps(pw).at( PlanningWorkspace::GridType::REMAINING_AREA );

    m_planningWorkspace = &pw;
    m_pw_subfieldIdx = subfieldIdx;

    auto ret = planSubfield(graph,
                            subfield,
                            baseRoutes,
                            plannedRoutes,
                            machines,
                            outFieldInfo,
                            machineCurrentStates,
                            plannerParameters,
                            yieldmap,
                            remainingAreaMap,
                            edgeCostCalculator,
                            pPlanInfo);

    m_planningWorkspace = nullptr;

    return ret;
}


void FieldProcessPlanner::setOutputFiles(const std::string &filename_graphBuilding, const std::string &foldername_planData, const std::string &foldername_visitSchedule)
{

    m_filename_graphBuilding = filename_graphBuilding;
    m_foldername_planData = foldername_planData;
    if(!m_foldername_planData.empty() && m_foldername_planData.back() != '/')
        m_foldername_planData += "/";
    m_foldername_visitSchedule = foldername_visitSchedule;
    if(!m_foldername_visitSchedule.empty() && m_foldername_visitSchedule.back() != '/')
        m_foldername_visitSchedule += "/";
}

AroResp FieldProcessPlanner::do_multiHarvPlanning(const DirectedGraph::Graph originalGraph,
                                                    const std::vector<Route> &baseRoutes,
                                                    const std::vector<Machine> &machines,
                                                    const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                    const MultiHarvPlanner::PlannerSettings& plannerParameters,
                                                    const std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                                    Logger &logger,
                                                    std::vector<Route>& out_mainRoutes,
                                                    DirectedGraph::Graph &out_updatedGraph,
                                                    PlanGeneralInfo *out_pPlanInfo)
{
    MultiHarvPlanner multiHarvPlanner(originalGraph,
                                        baseRoutes,
                                        machines,
                                        machineCurrentStates,
                                        plannerParameters,
                                        edgeCostCalculator,
                                        m_foldername_planData,
                                        logger.logLevel());
    multiHarvPlanner.logger().setParent(&logger);
    logger.printOut(LogLevel::INFO, __FUNCTION__, "Planning started..." );

    std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

    auto sError = multiHarvPlanner.planAll();
    if(!sError.empty()){
        logger.printOut( LogLevel::ERROR, __FUNCTION__, "Error planning for the working machine(s): " + sError );
        return AroResp( 1, "Error planning for the working machine(s): " + sError );
    }

    double planningDuration = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();

    out_mainRoutes = multiHarvPlanner.getPlannedRoutes();
    out_updatedGraph = multiHarvPlanner.getPlanData().graph;

    if(out_pPlanInfo){
        out_pPlanInfo->overallDelays = std::vector<double>(out_mainRoutes.size(), 0);
        out_pPlanInfo->planOverallCost = multiHarvPlanner.getPlanData().planOverallCost;
        out_pPlanInfo->planningDuration = planningDuration;
    }
    return AroResp();
}

AroResp FieldProcessPlanner::do_multiOLVPlanning(const DirectedGraph::Graph originalGraph,
                                                const std::vector<Route> &baseRoutes,
                                                const std::vector<Machine> &machines,
                                                const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                const MultiOLVPlanner::PlannerSettings& plannerParameters,
                                                const std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                                Logger &logger,
                                                std::vector<Route>& out_harvRoutes,
                                                std::vector<Route>& out_olvRoutes,
                                                DirectedGraph::Graph &out_updatedGraph,
                                                PlanGeneralInfo *out_pPlanInfo)
{
    this->saveVisitSchedule(originalGraph, "graph_visit_schedule__original.csv");

    MultiOLVPlanner multiOLVPlanner (originalGraph,
                                        baseRoutes,
                                        machines,
                                        machineCurrentStates,
                                        plannerParameters,
                                        edgeCostCalculator,
                                        this->m_foldername_planData,
                                        logger.logLevel());
    multiOLVPlanner.logger().setParent(&logger);
    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Planning started..." );
    std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

    auto sError = multiOLVPlanner.planAll();
    if(!sError.empty()){
        logger.printOut( LogLevel::ERROR, __FUNCTION__, "Error planning olvs: " + sError );
        return AroResp( 1, "Error planning olvs: " + sError );
    }

    double planningDuration = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();

    out_olvRoutes = multiOLVPlanner.getPlannedRoutes();
    logger.printOut(LogLevel::INFO, __FUNCTION__, "Checking resulting/planned olv routes...");
    auto compResp = this->checkOLVRoutes(machines, baseRoutes, out_olvRoutes);
    if(compResp.errorID > 0)
        return compResp;

    out_harvRoutes = multiOLVPlanner.getPlanData().harvesterRoutes;

    out_updatedGraph = multiOLVPlanner.getPlanData().graph;
    if(out_pPlanInfo){
        out_pPlanInfo->overallDelays = multiOLVPlanner.getPlanData().overallDelays;
        out_pPlanInfo->planOverallCost = multiOLVPlanner.getPlanData().planOverallCost;
        out_pPlanInfo->planningDuration = planningDuration;
    }

    this->saveVisitSchedule(out_updatedGraph, "graph_visit_schedule__final.csv");
    return AroResp();
}

OutFieldInfo FieldProcessPlanner::adjustOutFieldInfoArrivalTimes(const OutFieldInfo &outFieldInfo_in,
                                                                 const std::vector<FieldAccessPoint> &fieldAccessPoints,
                                                                 const std::vector<Machine> &machines,
                                                                 const std::vector<Route> &baseRoutes_infield,
                                                                 const std::vector<HeadlandRoute> &baseRoutes_headland,
                                                                 const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                                 const Subfield &subfield,
                                                                 double &deltaTime)
{
    deltaTime = 0;
    if(fieldAccessPoints.empty())
        return outFieldInfo_in;

    if ( ! areWorkingMachinesOutOfField(machines,
                                   machineCurrentStates,
                                   subfield.boundary_outer) )
        return outFieldInfo_in;

    OutFieldInfo outFieldInfo_out = outFieldInfo_in;
    outFieldInfo_out.clearArrivalCosts();

    FieldAccessPoint initialFAP;

    if( !getInitialEntryPoint(outFieldInfo_in,
                              fieldAccessPoints,
                              machines,
                              baseRoutes_infield,
                              baseRoutes_headland,
                              initialFAP,
                              deltaTime) ){
        deltaTime = 0;
        return outFieldInfo_in;
    }

    for(auto &it_fap : outFieldInfo_in.mapArrivalCosts()){
        for(auto &it_m : it_fap.second){
            for(auto &it_bs : it_m.second){
                OutFieldInfo::TravelCosts arrivalCosts = it_bs.second;
                arrivalCosts.time -= deltaTime;
                if(arrivalCosts.time < 0)
                    arrivalCosts.time = 0;
                outFieldInfo_out.mapArrivalCosts()[it_fap.first][it_m.first][it_bs.first] = arrivalCosts;
            }
        }
    }

    return outFieldInfo_out;

}

void FieldProcessPlanner::preProcessBaseRoutes(std::vector<Route> &baseRoutes_infield,
                                                std::vector<HeadlandRoute> &baseRoutes_headland,
                                                const Subfield &subfield,
                                                const std::vector<Machine> &machines,
                                                const OutFieldInfo &outFieldInfo,
                                                const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                const ArolibGrid_t &remainingAreaMap,
                                                bool calcRemainingAreaPrecisely)
{
    const double remainingAreaThreshold = 0.5;

    auto machinesMap = Machine::toMachineIdMap(machines);

    std::map<MachineId_t, std::pair< size_t, size_t > > firstGoodIndexMap_HL; //map< machine_id , < good_index, route_index > >
    std::map<MachineId_t, std::pair< size_t, size_t > > firstGoodIndexMap_IF; //map< machine_id , < good_index, route_index > >

    //check if there is anything to be done (work) in the routes. if not, delete the route
    for(size_t r = 0 ; r < baseRoutes_headland.size() ; ++r){

        HeadlandRoute &route = baseRoutes_headland.at(r);
        size_t firstGoodIndex = route.route_points.size();

        //the base/main routes have been preprocessed already, and the timestamps of the initial segment/edge with nothing to work/process is = -1
        for(size_t i = 0 ; i < route.route_points.size() ; ++i){
            if(route.route_points.at(i).time_stamp >= -1e-6){
                firstGoodIndex = i;
                break;
            }
        }

        if(firstGoodIndex == route.route_points.size() ){//nothing to work/proces in this route
            baseRoutes_headland.erase( baseRoutes_headland.begin() + r );
            --r;
            continue;
        }

        if(remainingAreaMap.isAllocated()){

            auto it_m = machinesMap.find(route.machine_id);
            if(it_m == machinesMap.end())
                continue;
            double workingWidth = it_m->second.working_width;
            if(workingWidth <= 0)
                continue;

            //search for the first valid route point (something to work on) based on the remaining (unworked) -area map (i.e. th route point of the first segment/edge that has not being worked/processed)
            size_t firstGoodIndex2 = firstGoodIndex;
            for(size_t i = firstGoodIndex ; i+1 < route.route_points.size() ; ++i){

                bool errorTmp;
                double remainingArea;

                if(calcRemainingAreaPrecisely){
                    //check the average value in the area overlaping with the segment/edge's rectangle
                    if(m_planningWorkspace){
                        //use the planningWorkspace, which might already know what cells correspond to this segment/edge (reduce computation)

                        const ArolibGrid_t &map = m_planningWorkspace->getGrid( PlanningWorkspace::GridType::REMAINING_AREA );
                        const auto & gridCellsInfo = m_planningWorkspace->getEdgeCellsInfo(PlanningWorkspace::GridType::REMAINING_AREA,
                                                                                           PlanningWorkspace::PROCESS_HEADLAND,
                                                                                           route.route_points.at(i),
                                                                                           route.route_points.at(i+1),
                                                                                           workingWidth);
                        remainingArea = map.getCellsComputedValue( gridCellsInfo,
                                                                   ArolibGrid_t::AVERAGE_TOTAL,
                                                                   arolib::geometry::calc_dist(route.route_points.at(i), route.route_points.at(i+1)) * workingWidth,
                                                                   false,
                                                                   &errorTmp );

                    }
                    else
                        remainingArea = remainingAreaMap.getLineComputedValue( route.route_points.at(i),
                                                                               route.route_points.at(i+1),
                                                                               workingWidth,
                                                                               true,
                                                                               ArolibGrid_t::AVERAGE_TOTAL,
                                                                               &errorTmp );
                }
                else{   //check only the cell corresponding to the middle point of the segment
                    Point midPoint;
                    midPoint.x = 0.5 * ( route.route_points.at(i).x + route.route_points.at(i+1).x );
                    midPoint.y = 0.5 * ( route.route_points.at(i).y + route.route_points.at(i+1).y );
                    remainingArea = remainingAreaMap.getValue(midPoint, &errorTmp);
                }

                if (errorTmp || remainingArea < remainingAreaThreshold){    
                    //set the parameters of the 'already worked/processed' route points accordingly (inc. timestamp = -1)
                    firstGoodIndex2 = i+1;
                    route.route_points.at(i).time_stamp = -1;
                    route.route_points.at(i).harvested_mass = 0;
                    route.route_points.at(i).harvested_volume = 0;
                }
                else{
                    if(remainingArea > 0.2 && remainingArea < 0.8){//add new routepoint
                        remainingArea = std::min(1.0, remainingArea * 1.2);//to account for errors
                        int iTmp = i;
                        double time = remainingArea * route.route_points.at(i).time_stamp + (1-remainingArea) * route.route_points.at(i+1).time_stamp;
                        RoutePoint newRP = route.calcPoint2(time, iTmp);
                        route.route_points.insert( route.route_points.begin()+i+1, newRP );
                        ++firstGoodIndex2;
                        route.route_points.at(i).time_stamp = -1;
                        route.route_points.at(i).harvested_mass = 0;
                        route.route_points.at(i).harvested_volume = 0;
                    }
                    break;
                }
            }

            if(firstGoodIndex2 >= route.route_points.size()-1 ){
                baseRoutes_headland.erase( baseRoutes_headland.begin() + r );
                --r;
                continue;
            }

            firstGoodIndex = firstGoodIndex2;
        }

        //adjust timestamps and harvested mass/volume
        auto rpRef = route.route_points.at(firstGoodIndex);
        for(size_t i = firstGoodIndex ; i < route.route_points.size() ; ++i){
            route.route_points.at(i).time_stamp -= rpRef.time_stamp;
            route.route_points.at(i).harvested_mass -= rpRef.harvested_mass;
            route.route_points.at(i).harvested_volume -= rpRef.harvested_volume;
        }

        firstGoodIndexMap_HL[route.machine_id] = std::make_pair(firstGoodIndex, r);

    }
    //check if there is anything to harvest in the routes. if not, delete the route
    for(size_t r = 0 ; r < baseRoutes_infield.size() ; ++r){
        Route &route = baseRoutes_infield.at(r);
        size_t firstGoodIndex = route.route_points.size();

        bool firstGoodIndexInHeadlandRoute = firstGoodIndexMap_HL.find(route.machine_id) != firstGoodIndexMap_HL.end();

        //the main/base routes have been preprocessed already, and the timestamps of the initial segment/edge with nothing to process/work is = -1
        for(size_t i = 0 ; i < route.route_points.size() ; ++i){
            if(route.route_points.at(i).time_stamp >= -0.00001){
                firstGoodIndex = i;
                break;
            }
        }
        if(firstGoodIndex == route.route_points.size() ){
            baseRoutes_infield.erase( baseRoutes_infield.begin() + r );
            --r;
            continue;
        }

        if(remainingAreaMap.isAllocated()){

            auto it_m = machinesMap.find(route.machine_id);
            if(it_m == machinesMap.end())
                continue;
            double workingWidth = it_m->second.working_width;
            if(workingWidth <= 0)
                continue;

            //search for the first valid route point (something to process/work) based on the remaining (unworked) -area map (i.e. th route point of the first segment/edge that has not being worked/processed)
            size_t firstGoodIndex2 = firstGoodIndex;
            for(size_t i = firstGoodIndex ; i+1 < route.route_points.size() ; ++i){
                bool errorTmp;
                double remainingArea;

                if(calcRemainingAreaPrecisely){
                    //check the average value in the area overlaping with the segment's rectangle
                    if(m_planningWorkspace){
                        //use the planningWorkspace, which might already know what cells correspond to this segment/edge (reduce computation)
                        const ArolibGrid_t &map = m_planningWorkspace->getGrid( PlanningWorkspace::GridType::REMAINING_AREA );
                        const auto & gridCellsInfo = m_planningWorkspace->getEdgeCellsInfo(PlanningWorkspace::GridType::REMAINING_AREA,
                                                                                           PlanningWorkspace::PROCESS_INFIELD,
                                                                                           route.route_points.at(i),
                                                                                           route.route_points.at(i+1),
                                                                                           workingWidth);
                        remainingArea = map.getCellsComputedValue( gridCellsInfo,
                                                           ArolibGrid_t::AVERAGE_TOTAL,
                                                           arolib::geometry::calc_dist(route.route_points.at(i), route.route_points.at(i+1)) * workingWidth,
                                                           false,
                                                           &errorTmp );

                    }
                    else
                        remainingArea = remainingAreaMap.getLineComputedValue( route.route_points.at(i),
                                                                               route.route_points.at(i+1),
                                                                               workingWidth,
                                                                               true,
                                                                               ArolibGrid_t::AVERAGE_TOTAL,
                                                                               &errorTmp );
                }
                else{
                    //check only the cell corresponding to the middle point of the segment
                    Point midPoint;
                    midPoint.x = 0.5 * ( route.route_points.at(i).x + route.route_points.at(i+1).x );
                    midPoint.y = 0.5 * ( route.route_points.at(i).y + route.route_points.at(i+1).y );
                    remainingArea = remainingAreaMap.getValue(midPoint, &errorTmp);
                }

                if (errorTmp || remainingArea < remainingAreaThreshold){
                    //set the parameters of the 'already processed/worked' route points accordingly (inc. timestamp = -1)
                    firstGoodIndex2 = i+1;
                    route.route_points.at(i).time_stamp = -1;
                    route.route_points.at(i).harvested_mass = 0;
                    route.route_points.at(i).harvested_volume = 0;
                }
                else{
                    if(remainingArea > 0.2 && remainingArea < 0.8){//add new routepoint
                        remainingArea = std::min(1.0, remainingArea * 1.2);//to account for errors
                        int iTmp = i;
                        double time = remainingArea * route.route_points.at(i).time_stamp + (1-remainingArea) * route.route_points.at(i+1).time_stamp;
                        RoutePoint newRP = route.calcPoint2(time, iTmp);
                        route.route_points.insert( route.route_points.begin()+i+1, newRP );
                        ++firstGoodIndex2;
                        route.route_points.at(i).time_stamp = -1;
                        route.route_points.at(i).harvested_mass = 0;
                        route.route_points.at(i).harvested_volume = 0;
                    }
                    break;
                }
            }

            if(firstGoodIndex2 >= route.route_points.size()-1 ){
                baseRoutes_infield.erase( baseRoutes_infield.begin() + r );
                --r;
                continue;
            }
            firstGoodIndex = firstGoodIndex2;
        }

        //adjust timestamps and worked mass/volume
        auto rpRef = route.route_points.at(firstGoodIndex);
        for(size_t i = firstGoodIndex ; i < route.route_points.size() ; ++i){
            route.route_points.at(i).time_stamp -= rpRef.time_stamp;
            route.route_points.at(i).harvested_mass -= rpRef.harvested_mass;
            route.route_points.at(i).harvested_volume -= rpRef.harvested_volume;
        }

        firstGoodIndexMap_IF[route.machine_id] = std::make_pair(firstGoodIndex, r);
    }
}

std::vector<Route> FieldProcessPlanner::connectBaseRoutes(std::vector<Route> &baseRoutes_infield,
                                                           std::vector<HeadlandRoute> &baseRoutes_headland,
                                                           const Subfield &subfield,
                                                           const std::vector<Machine> &machines)
{
    std::vector<Point> headland_points = subfield.headlands.complete.middle_track.points;
    std::vector<Route> routes;

    std::map<int, Machine> machinesMap;//for easier access
    for(auto &m : machines){
        if(m.isOfWorkingType(true))
            machinesMap[m.id] = m;
    }

    //check if there only exists headland OR inner-field routes (-> no connection is needed)
    if(baseRoutes_headland.empty()){
        routes = baseRoutes_infield;
        return routes;
    }
    if(baseRoutes_infield.empty()){
        routes.resize( baseRoutes_headland.size() );
        for(size_t i = 0 ; i < baseRoutes_headland.size(); ++i){
            routes[i].route_id = baseRoutes_headland[i].route_id;
            routes[i].machine_id = baseRoutes_headland[i].machine_id;
            routes[i].route_points = baseRoutes_headland[i].route_points;
        }
        return routes;
    }

    std::set<int> routeIdsIF, routeIdsIF_added;
    //the base route id will be the ids of the infield routes. this is done so that, if there are headland routes with no corresponding infield routes to be connected, its id does not conflict with other routes (i.e. avoid repeated route ids)
    for(auto &ifr : baseRoutes_infield)
        routeIdsIF.insert(ifr.route_id);

    //connect the headland routes with the inner-field routes of the corresponding machine
    for(auto &hlr : baseRoutes_headland){
        bool connected = false;
        Route route;
        route.route_id = hlr.route_id;
        route.machine_id = hlr.machine_id;
        route.route_points = hlr.route_points;
        auto it_machine = machinesMap.find(route.machine_id);
        if(it_machine != machinesMap.end()){
            Machine machine = it_machine->second;

            //find and connect with the inner-field route of the same machine
            for(auto &ifr : baseRoutes_infield){
                if(ifr.route_points.empty())
                    continue;
                if(route.machine_id == ifr.machine_id){
                    //is the route from the same machine?
                    if(!route.route_points.empty()){
                        //points to be used for the connection (initially, the last point of the headland route and the first point of the infield route)
                        Point cp0 = route.route_points.back().point();
                        Point cpn = ifr.route_points.front().point();

                        //check if some extra points must be added to get a smoother conection (i.e. trying to perserve the working direction before/after the connection path)
                        bool extra0 = false, extran = false;
                        if(route.route_points.size() > 1
                                && route.route_points.back().type == RoutePoint::TRACK_END){
                            //extend the last segment of the headland route so that when the machine finishes working, it keeps the driving direction for a short distance (i.e avoid turns immediatly after finishing working the headland).
                            double extDist = std::min(machine.working_width, arolib::geometry::calc_dist_to_linestring(headland_points, cp0));
                            cp0 = arolib::geometry::extend_line( r_at(route.route_points, 1), cp0, extDist );//update cp0
                            extra0 = true;
                        }
                        if(ifr.route_points.size() > 1){
                            //(reverse) extend the first segment of the infield route so that the last segment of the connection is in the same direction of the first segment in the infield route (i.e. avoid turns immediatly before starting working the infield).
                            double extDist = std::min(machine.working_width, arolib::geometry::calc_dist_to_linestring(headland_points, cpn));
                            cpn = arolib::geometry::extend_line( ifr.route_points.at(1), cpn, extDist );//update cpn
                            extran = true;
                        }

                        // get the shortest connection between cp0 and cpn though the planned headland ('track' in the middle of the headland)
                        std::vector<Point> connection = arolib::geometry::getShortestGeometryPart(headland_points,
                                                                                cp0,
                                                                                cpn,
                                                                                true);

                        // update the start and end of the connections in case extensions had to be performed
                        if(extra0){
                            if (connection.size() > 1 &&
                                    std::fabs( arolib::geometry::get_angle(connection.front(), connection.at(1), connection.front(), cp0) ) < M_PI_2 )
                                connection.front() = cp0;
                            else
                                push_front(connection, cp0);
                        }
                        if(extran){
                            if (connection.size() > 1 &&
                                    std::fabs( arolib::geometry::get_angle(connection.back(), r_at(connection,1), connection.back(), cpn) ) < M_PI_2 )
                                connection.back() = cpn;
                            else
                            connection.emplace_back(cpn);
                        }

                        // build the connection route-segment and add it to both the final (connected) route and the (original) headland route
                        for(auto &cp : connection){
                            RoutePoint rp;
                            rp.point() = cp;
                            rp.type = RoutePoint::TRANSIT;
                            rp.track_id = -1;
                            rp.bunker_mass = route.route_points.back().bunker_mass;
                            rp.bunker_volume = route.route_points.back().bunker_volume;
                            rp.harvested_mass = route.route_points.back().harvested_mass;
                            rp.harvested_volume = route.route_points.back().harvested_volume;
                            rp.time_stamp = route.route_points.back().time_stamp;


                            double machine_speed = machine.calcSpeed(rp.bunker_mass);
                            if(machine_speed > 0){
                                double dist = arolib::geometry::calc_dist(rp, route.route_points.back());
                                rp.time_stamp += dist/machine_speed;
                            }

                            route.route_points.push_back( rp );
                            hlr.route_points.push_back(rp);
                        }

                        // calculate the time that the infield route has to be shifted (i.e. its timestamps)
                        double dTime = 0;
                        {
                            double machine_speed = machine.calcSpeed(ifr.route_points.front().bunker_mass);
                            if(machine_speed > 0){
                                double dist = arolib::geometry::calc_dist(ifr.route_points.front(), route.route_points.back());
                                dTime = route.route_points.back().time_stamp + dist/machine_speed;
                            }
                        }

                        //apply the shift-time to the timestamps of the (original) infield route
                        for(auto &rp : ifr.route_points){
                            rp.time_stamp += dTime;
                            rp.harvested_mass += route.route_points.back().harvested_mass;
                            rp.harvested_volume += route.route_points.back().harvested_volume;
                        }

                        //add the adjusted infield route to the final (connected) route
                        route.route_points.insert( route.route_points.end(), ifr.route_points.begin(), ifr.route_points.end() );

                        route.route_id = ifr.route_id;
                        routeIdsIF_added.insert(ifr.route_id);
                        connected = true;
                    }
                }
            }
        }
        if(!connected){
            //check if the headland route was connected to its corresponding infield route
            while( routeIdsIF.find(route.route_id) != routeIdsIF.end() ){
                //check if the routes id must be changed not to be repeated, and change it if needed
                route.route_id++;
                hlr.route_id++;
            }
        }
        routes.push_back(route);
    }

    // add the remaining IF routes that had no corresponding headland route to be conected with
    for(auto ifr : baseRoutes_infield){
        if(ifr.route_points.empty())
            continue;
        if(routeIdsIF_added.find(ifr.route_id) != routeIdsIF_added.end())
            continue;
        routes.push_back(ifr);
    }

    // sort final routes by route id
    std::sort(routes.begin(), routes.end(), [](const Route & r1, const Route & r2) -> bool { return r1.route_id < r2.route_id; } );

    return routes;
}

AroResp FieldProcessPlanner::checkOLVRoutes(const std::vector<Machine> &machines,
                                             const std::vector<Route> &harvesterRoutes,
                                             const std::vector<Route> &OLVRoutes)
{
    int countHarvesters = harvesterRoutes.size();
    int countOLVs = 0;
    for (auto &m : machines) {
        if (m.machinetype == Machine::OLV)
            ++countOLVs;
    }

    // this should not happen in the real harvesting scenarios (each harvester route must have at least one olv assigned to it),
    // however, if no olvs were selected, or in general less olvs than required were seleceted, we (temporarily) allow it (it is an acceptable special case)
    if(countOLVs < countHarvesters)
        return AroResp(0, "OK");

    bool ok = true;
    std::string errorMsg = "No OLV routes were found for the following harvesters:";//temporary
    for (auto &hr : harvesterRoutes) {
        //@TODO: how stable is this check? can it be possible that the olv route point just went over the track while reaching an overload point of other harvester route?

        // get the first track in the harvester route corresponding to a non-headland track (i.e. corresponding to a harvesting track, either in headland or infield harvesting)
        int harvTrackId = -1;
        for(auto &rp : hr.route_points){
            if (rp.track_id >= 0//@TODO: perhaps it must also be checked that the corresponding segment had something to harvest (to know that indeed an olv was needed)
                  /*&& rp.harvested_mass > 1e-6*/ ){
                harvTrackId = rp.track_id;
                break;
            }
        }

        //check that there is an olv route with a route point with the same track id obtained before
        bool routeFound = false;
        for(auto &route : OLVRoutes){
            for(auto &rp : route.route_points){
                //@TODO: perhaps it must also be checked that the route point type is an overloading one (OL, start or end)
                if (rp.track_id == harvTrackId){
                    routeFound = true;
                    break;
                }
            }
        }
        if(!routeFound){
            ok = false;
            errorMsg += " " + std::to_string(hr.machine_id) + ",";
        }
    }

    errorMsg.pop_back();
    if(!ok)
        return AroResp(1, errorMsg);

    return AroResp(0, "OK");
}

bool FieldProcessPlanner::getInitialEntryPoint(const OutFieldInfo &outFieldInfo,
                                               const std::vector<FieldAccessPoint> &fieldAccessPoints,
                                               const std::vector<Machine> &workingGroup,
                                               const std::vector<Route> &baseRoutes_infield,
                                               const std::vector<HeadlandRoute> &baseRoutes_headland,
                                               FieldAccessPoint &initialFAP,
                                               double &maxArrivalTime)
{
    RoutePoint firstRP;
    firstRP.time_stamp = std::numeric_limits<double>::max();

    if(!baseRoutes_headland.empty()){
        for(auto &route : baseRoutes_headland){
            for(auto &rp : route.route_points){
                if(rp.time_stamp < firstRP.time_stamp)
                    firstRP = rp;
            }
        }
    }
    else{
        for(auto &route : baseRoutes_infield){
            for(auto &rp : route.route_points){
                if(rp.time_stamp < firstRP.time_stamp)
                    firstRP = rp;
            }
        }
    }

    // organice the field access point based on the distance to the first route point (shortest first). The shortest the distance, the more likely it was that this access point was used used during initial harvester-route planning
    std::map<double, size_t> mapDistToFAPx;
    for(size_t i = 0; i < fieldAccessPoints.size(); ++i)
        mapDistToFAPx[arolib::geometry::calc_dist(fieldAccessPoints.at(i), firstRP)] = i;

    maxArrivalTime = std::numeric_limits<double>::lowest();
    bool maxArrivalTime_OK = false;

    // get the maximum arrival time (for the working machines) to the estimated field access point that was used during initial base-route planning (i.e the closest to the first route point).
    for(auto &FAPx : mapDistToFAPx){
        const FieldAccessPoint &fap = fieldAccessPoints.at(FAPx.second);
        if(outFieldInfo.size_arrivalCosts( fap.id ) == 0)
            continue;

        // get the maximum arrival time (for the working machines) to the field access point in queue
        for(auto &m : workingGroup){
            if(!m.isOfWorkingType())
                continue;
            OutFieldInfo::TravelCosts arrivalCosts;
            if(outFieldInfo.getArrivalCost(fap.id, m.id, OutFieldInfo::MACHINE_EMPTY, arrivalCosts)){
                maxArrivalTime = std::max(maxArrivalTime, arrivalCosts.time);
                initialFAP = fap;
                maxArrivalTime_OK = true;
            }
        }

        //stop searching if a maxArrivalTime was obtained for this access point; if no arrival times were obtained for this access point, continue with the next closest to the first route point
        if(maxArrivalTime_OK)
            break;
    }

    if(maxArrivalTime_OK)
        return true;

    // we cannot estimate what access point was used during initial base-route planning (no arrival times could be obtained for any access point)
    maxArrivalTime = 0;
    return false;
}

void FieldProcessPlanner::sendWorkingMachinesToExitPoints(DirectedGraph::Graph &graph,
                                                          std::vector<Route> &mainRoutes,
                                                          const Subfield &subfield,
                                                          const std::vector<Machine> &machines,
                                                          const PlannerParameters &plannerParameters,
                                                          std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                                          bool useSearch)
{

    std::map<MachineId_t, Machine> machinesMap;//for easier access
    for(auto& m : machines){
        if(m.isOfWorkingType(true))
            machinesMap[m.id] = m;
    }

    for(auto &route : mainRoutes){
        if(route.route_points.empty())
            continue;

        if(subfield.access_points.empty())
            continue;

        auto lastRP = route.route_points.back();
        if( lastRP.type == RoutePoint::RESOURCE_POINT &&
                !arolib::geometry::in_polygon(lastRP, subfield.boundary_outer))
            continue;

        auto it_m = machinesMap.find(route.machine_id);
        if(it_m == machinesMap.end())
            continue;

        const Machine& machine = it_m->second;

        if(!useSearch){//do it geometrically
            sendWorkingMachinesToExitPoint(route, subfield, machine);
            continue;
        }

        double machine_speed = machine.calcSpeed(route.route_points.back().bunker_mass);

        //connect using AStar
        double minCost = std::numeric_limits<double>::max();
        AstarPlan bestPlan;
        bool planOK = false;

        Astar::PlanParameters astarParams;
        auto rp_it = graph.routepoint_vertex_map().find(lastRP);
        if(rp_it != graph.routepoint_vertex_map().end())
            astarParams.start_vt = rp_it->second;
        else
            astarParams.start_vt = find_nearest_vertex(lastRP.point(), graph);
        astarParams.start_time = lastRP.time_stamp;
        astarParams.max_time_visit = std::numeric_limits<double>::max();//everything is worked now --> not limits
        astarParams.max_time_goal = std::numeric_limits<double>::max();//not limits, just get there
        astarParams.machine = machine;
        astarParams.machine_speed = machine_speed;
        astarParams.initial_bunker_mass = lastRP.bunker_mass;
        astarParams.overload = false;
        astarParams.includeWaitInCost = true;
        astarParams.restrictedMachineIds = {};
        astarParams.restrictedMachineIds_futureVisits = {};

        //exclude resource points
        for(auto& it_resP : graph.resourcepoint_vertex_map())
            astarParams.exclude.insert(it_resP.second);

        if(route.route_points.size() > 1){
            auto rp_exc = r_at(route.route_points, 1);
            auto it_vt_exc = graph.routepoint_vertex_map().find(rp_exc);
            if(it_vt_exc != graph.routepoint_vertex_map().end()){
                astarParams.exclude.insert(it_vt_exc->second);
            }
        }

        //check the planned route-segments to each (exit) access point to obtain the optimal one
        for(size_t i = 0 ; i < subfield.access_points.size() ; ++i){
            auto &fap = subfield.access_points.at(i);
            if(fap.accessType == FieldAccessPoint::AP_ENTRY_ONLY)
                continue;

            auto it_vt = graph.accesspoint_vertex_map().find(fap);
            if(it_vt == graph.accesspoint_vertex_map().end())
                continue;

            astarParams.goal_vt = it_vt->second;

            const Astar::AStarSettings& astarSettings_original = plannerParameters;
            Astar::AStarSettings astarSettings = astarSettings_original;
            astarSettings.includeWaitInCost = true;

            try{

                auto foldername_planData = m_foldername_planData;
                if(!foldername_planData.empty()){
                    foldername_planData += "finish/M" + std::to_string(route.machine_id) + "/toFAP_" + std::to_string(i) + "/";
                    io::create_directory(foldername_planData, true);
                }

                Astar planner (astarParams,
                               astarSettings,
                               RoutePoint::TRANSIT,
                               foldername_planData,
                               &m_logger);

                if( !planner.plan(graph, edgeCostCalculator) ){
                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, 10, "Error getting final path for machine ", machine.id);
                    continue;
                }

                if(minCost > planner.getPlan().plan_cost_total){
                    bestPlan = planner.getPlan();
                    minCost = bestPlan.plan_cost_total;
                    planOK = true;
                }
            }
            catch(const std::exception& e){
            }
        }

        if(!planOK){
            m_logger.printOut(LogLevel::WARNING, __FUNCTION__, 10, "No plan obtained for final path of machine ", machine.id, " with A*. Getting final path geometrically");
            sendWorkingMachinesToExitPoint(route, subfield, machine);
            continue;
        }

        if(bestPlan.route_points_.empty())
            continue;

        bestPlan.insertIntoGraph(graph);

        for(size_t i = 0 ; i < bestPlan.route_points_.size() ; ++i){
            bestPlan.route_points_.at(i).type = RoutePoint::TRANSIT;
            bestPlan.route_points_.at(i).harvested_mass = lastRP.harvested_mass;
            bestPlan.route_points_.at(i).harvested_volume = lastRP.harvested_volume;
        }
        bestPlan.route_points_.back().type = RoutePoint::FIELD_EXIT;

        route.route_points.insert( route.route_points.end(), bestPlan.route_points_.begin()+1, bestPlan.route_points_.end() );
    }
}

void FieldProcessPlanner::sendWorkingMachinesToExitPoint(Route &route, const Subfield &subfield, const Machine &machine)
{
    std::vector<Point> headland_points = subfield.headlands.complete.middle_track.points;

    if(route.route_points.empty())
        return;

    double machine_speed = machine.calcSpeed(route.route_points.back().bunker_mass);
    double res = machine.working_width * 2;//resolution [m]
    if (machine_speed < 0)
        return;
    if(res <= 1e-3)
        res = 10;

    double minDist = std::numeric_limits<double>::max();
    std::vector<Point> connection;

    Polygon limitBoundary, outerBoundaryTmp;
    arolib::geometry::offsetPolygon( subfield.boundary_outer, limitBoundary, 1e-3, true, 0 );

    //get the path to each (exit) access point and select the shortest one
    for(auto &fap : subfield.access_points){
        if(fap.accessType == FieldAccessPoint::AP_ENTRY_ONLY)
            continue;

        //get a direct connection with the access point iif that connection (line) does not intersect the inner boundary (i.e. the complete path is inside the headland) nor with the outer boundary (i.e. the complete path is inside the field)
        auto intersectionsIB = arolib::geometry::get_intersection(route.route_points.back(), fap, subfield.boundary_inner.points);
        bool intersectsIB = false;
        for(auto &is : intersectionsIB){
            if( arolib::geometry::calc_dist(route.route_points.back(), is) > 0.01 ){//only consider the intersection if it is relativelly far from the last route point
                intersectsIB = true;
                break;
            }
        }
        if( !intersectsIB ){//if the direct connection (line) intersects with the inner field, don't consider it (better use the planned headland to get the connection, even if it is a bit longer( but probably 'smoother'))
            double distToOB = arolib::geometry::calc_dist_to_linestring(subfield.boundary_outer.points, fap) + 0.01;
            auto intersectionsOB = arolib::geometry::get_intersection(route.route_points.back(), fap, subfield.boundary_outer.points);
            bool intersectsOB = false;
            for(auto &is : intersectionsOB){
                if( arolib::geometry::calc_dist(fap, is) > distToOB*1.001 ){//only consider the intersection if it is relativelly far from the access point (the access point could be near the outer boundary)
                    intersectsOB = true;
                    break;
                }
            }
            if( !intersectsOB ){//if the direct connection (line) intersects with the outer field, don't consider it (we need that the path is completelly inside the field)
                double dist = arolib::geometry::calc_dist(route.route_points.back(), fap);
                if(minDist > dist){
                    minDist = dist;
                    connection = {route.route_points.back(), fap};
                    connection = arolib::geometry::sample_geometry( connection, res );
                    pop_front(connection);
                }
            }
        }

        Pose2D pose_start, pose_end;
        double speed_start = 0;
        pose_start.point() = route.route_points.back().point();
        if( route.route_points.size() > 1 ){
            pose_start.angle = arolib::geometry::get_angle( r_at(route.route_points, 1), route.route_points.back() );
            double delta_time = route.route_points.back().time_stamp - r_at(route.route_points, 1).time_stamp;
            if( delta_time > 1e-9 )
                speed_start = arolib::geometry::calc_dist( route.route_points.back(), r_at(route.route_points, 1) ) / delta_time;
        }
        Point fapEd = fap;
        pose_end.angle = -1001;
        if( !subfield.boundary_outer.points.empty() && !arolib::geometry::in_polygon(fap, subfield.boundary_outer) ){
            if(outerBoundaryTmp.points.empty())
                outerBoundaryTmp = subfield.boundary_outer;
            int ind = arolib::geometry::addSampleToGeometryClosestToPoint(outerBoundaryTmp.points, fap, 1);
            if(ind > 0 && ind < outerBoundaryTmp.points.size())
                fapEd = outerBoundaryTmp.points.at(ind);
            else
                fapEd = arolib::geometry::getPointInMinDist( outerBoundaryTmp.points, fap );
            if(fap.point() != fapEd)
                pose_end.angle = arolib::geometry::get_angle( fapEd, fap );
        }
        pose_end.point() = fapEd;
        if(pose_end.angle < -1000)
            pose_end.angle = arolib::geometry::get_angle( route.route_points.back(), fapEd );

        std::shared_ptr<IInfieldTracksConnector> connector = std::make_shared<InfieldTracksConnectorDef>(&logger());
        std::vector<Point> connectionTmp = connector->getConnection(machine,
                                                                    pose_start,
                                                                    pose_end,
                                                                    -1,
                                                                    -1,
                                                                    limitBoundary,
                                                                    subfield.boundary_inner,
                                                                    subfield.headlands,
                                                                    speed_start,
                                                                    -1);


        if(!connectionTmp.empty()){
            if(fap.point() != fapEd)
                connection.push_back(fap.point());
            double dist = arolib::geometry::getGeometryLength(connectionTmp);

            //update the best connection
            if(minDist > dist){
                minDist = dist;
                connection = connectionTmp;
                connection.push_back(fap.point());
            }
        }

    }

    //create the final route-segment from the path
    for(size_t i = 0 ; i < connection.size() ; ++i){
        RoutePoint &prevPoint = route.route_points.back();
        RoutePoint newPoint;
        double dist = arolib::geometry::calc_dist(prevPoint, connection.at(i));
        newPoint.point() = connection.at(i);
        newPoint.time_stamp = prevPoint.time_stamp + dist / machine_speed;
        newPoint.track_id = -1;
        newPoint.track_idx = -1;
        newPoint.copyBasicWorkingValuesFrom(prevPoint);

        if(i+1 == connection.size())
            newPoint.type = RoutePoint::FIELD_EXIT;
        else
            newPoint.type = RoutePoint::TRANSIT;

        route.route_points.push_back(newPoint);
    }
}

std::vector<Point> FieldProcessPlanner::getBestPathToFirstWorkingPoint(const HeadlandRoute &route,
                                                                       const Subfield &subfield,
                                                                       const Point &initialPoint,
                                                                       const Machine& machine)
{
    std::vector<Point> pointsOut;
    pointsOut.push_back(initialPoint);

    int minTrackId = std::numeric_limits<int>::max();
    size_t fistGoodIndex = 0;
    size_t closestIndex = 0;
    double minDistToInitPoint = std::numeric_limits<double>::max();

    //obtain the index of the first valid route point (where there is something to process/work) and of the route point closest to the initial point
    for(size_t i = 0; i < route.route_points.size() ; ++i){
        const RoutePoint &rp = route.route_points.at(i);
        if(rp.track_id >= 0 && minTrackId > rp.track_id)
            minTrackId = rp.track_id;
        double dist = arolib::geometry::calc_dist(initialPoint, rp);
        if(minDistToInitPoint > dist){
            minDistToInitPoint = dist;
            closestIndex = i;
        }
        if(rp.time_stamp >= -0.0001){
            fistGoodIndex = i;
            break;
        }
    }

    //points of interest to build the path
    RoutePoint firstRP = route.route_points.at(fistGoodIndex);//first valid route point (where there is something to process/work)
    RoutePoint closestRP = route.route_points.at(closestIndex);//route point closest to the initial point

    if(fistGoodIndex == closestIndex
            || arolib::geometry::calc_dist(firstRP, initialPoint) <= arolib::geometry::calc_dist(firstRP, closestRP) )
        return pointsOut;//no extra points needed, the initial point is close enough to the first valid point

    if(firstRP.track_id == closestRP.track_id){//points to be connected are on the same track -> build the path through the track
        for(size_t i = closestIndex ; i < fistGoodIndex ; ++i)
            pointsOut.push_back(route.route_points.at(i).point());
        return pointsOut;
    }
    if( std::abs(firstRP.track_id - closestRP.track_id) == 1){//points to be connected are on adjacent tracks
        pointsOut.push_back(closestRP.point());

        double minDistToClosestPoint = std::numeric_limits<double>::max();
        int indClosestToClosest = -1;

        //get the point in the track of firstRP that is closest to closestRP and is before firstRP (i.e. there is nothing to process/work)
        for(int i = fistGoodIndex-1; i >= 0 ; --i){
            const RoutePoint &rp = route.route_points.at(i);
            if(rp.track_id != firstRP.track_id)
                break;
            double dist = arolib::geometry::calc_dist(closestRP, rp);
            if(minDistToClosestPoint > dist){
                minDistToClosestPoint = dist;
                indClosestToClosest = i;
            }
        }
        if(indClosestToClosest >= 0){//if the point is found, build the path through the track of firstRP
            for(size_t i = indClosestToClosest ; i < fistGoodIndex ; ++i)
                pointsOut.push_back(route.route_points.at(i).point());
        }
        return pointsOut;
    }

    // build the path through the track of closestRP and firstRP (the connection between those tracks is a straight line because we are in the headland tracks)
    for(int i = closestIndex; i >= 0 ; --i){
        const RoutePoint &rp = route.route_points.at(i);
        if(rp.track_id != firstRP.track_id)
            break;
        pointsOut.push_back(rp.point());
    }
    std::vector<Point> path2;
    for(int i = fistGoodIndex-1; i >= 0 ; --i){
        const RoutePoint &rp = route.route_points.at(i);
        if(rp.track_id != firstRP.track_id)
            break;
        path2.push_back(rp.point());
    }
    pointsOut.insert( pointsOut.end(), path2.rbegin(), path2.rend() );
    return pointsOut;
}

std::vector<Point> FieldProcessPlanner::getBestPathToFirstWorkingPoint(const Route &route,
                                                                      const Subfield &subfield,
                                                                      const Point &initialPoint,
                                                                      const Machine &machine)
{
    std::vector<Point> pointsOut;
    auto headland = subfield.headlands.complete.middle_track.points;

    int minTrackId = std::numeric_limits<int>::max();
    int maxTrackId = std::numeric_limits<int>::lowest();
    bool fistGoodIndexOK = false;
    size_t fistGoodIndex = 0;
    size_t closestIndex = 0;
    double minDistToInitPoint = std::numeric_limits<double>::max();

    //obtain the index of the first valid route point (where there is something to process/work) and of the (already worked) route point closest to the initial point
    for(size_t i = 0; i < route.route_points.size() ; ++i){
        const RoutePoint &rp = route.route_points.at(i);
        if(rp.track_id >= 0 && minTrackId > rp.track_id)
            minTrackId = rp.track_id;
        if(rp.track_id >= 0 && maxTrackId < rp.track_id)
            maxTrackId = rp.track_id;
        if(!fistGoodIndexOK){//check for the route point closest to the initial point iif it belongs to the initial route-segment where there is nothing to harvest (@TODO: shouldn't we check also for route points of type HEADLAND?)
            double dist = arolib::geometry::calc_dist(initialPoint, rp);
            if(minDistToInitPoint > dist){
                minDistToInitPoint = dist;
                closestIndex = i;
            }
        }
        if(rp.time_stamp >= -0.0001 && !fistGoodIndexOK){
            fistGoodIndex = i;
            fistGoodIndexOK = true;
        }
    }

    //points of interest to build the path
    RoutePoint firstRP = route.route_points.at(fistGoodIndex);//first valid route point (where there is something to work)
    RoutePoint closestRP = route.route_points.at(closestIndex);//route point closest to the initial point belonging to the initial route-segment where there is nothing to harvest

    pointsOut.push_back(initialPoint);

    if( ( fistGoodIndex == closestIndex && arolib::geometry::in_polygon(initialPoint, subfield.boundary_inner) )
            //|| arolib::geometry::calc_dist(firstRP.point, initialPoint) < machine.working_width*1.5 )
            || arolib::geometry::calc_dist(firstRP, initialPoint) <= arolib::geometry::calc_dist(firstRP, closestRP) )
        return pointsOut;//no extra points needed, the initial point is close enough to the first valid point

    std::vector<Point> path2_toHeadland;
    Point path2_pointNearHeadland = firstRP.point();
    if( fistGoodIndex == 0 || firstRP.track_id == route.route_points.at(fistGoodIndex-1).track_id )//fistGoodIndex-1 is not headland
        path2_toHeadland = getPathInTrack(route.route_points, fistGoodIndex-1, -1);
    if(!path2_toHeadland.empty())//fistGoodIndex is a track start/end
        path2_pointNearHeadland = path2_toHeadland.back();

    if(!arolib::geometry::in_polygon(initialPoint, subfield.boundary_inner)){//machine in headland --> connect through headland

        auto pathHeadland = arolib::geometry::getShortestGeometryPart(headland,
                                                    initialPoint,
                                                    path2_pointNearHeadland,
                                                    false);
        pointsOut.insert( pointsOut.end(), pathHeadland.begin(), pathHeadland.end() );
        pointsOut.insert( pointsOut.end(), path2_toHeadland.rbegin(), path2_toHeadland.rend() );
        return pointsOut;
    }

    //in same track
    if(firstRP.track_id == closestRP.track_id){
        for(size_t i = closestIndex ; i < fistGoodIndex ; ++i)
            pointsOut.push_back(route.route_points.at(i).point());
        return pointsOut;
    }

    //in adjacent track
    if( std::abs(firstRP.track_id - closestRP.track_id) == 1){
        pointsOut.push_back(closestRP.point());

        int indTrack_0 = fistGoodIndex;
        int indTrack_1 = closestIndex;
        double minDistInterTrack = std::numeric_limits<double>::max();

        for(int i = fistGoodIndex/*-1*/ ; i >= 0 ; --i){
            const RoutePoint &rp0 = route.route_points.at(i);
            if(rp0.track_id != firstRP.track_id)
                break;
            for(int j = closestIndex-1 ; j >= 0 ; --j){
                const RoutePoint &rp1 = route.route_points.at(j);
                if(rp1.track_id != closestRP.track_id || rp1.time_stamp >= 0.0001)
                    break;
                double dist = arolib::geometry::calc_dist(rp0, rp1);
                if(minDistInterTrack > dist){
                    minDistInterTrack = dist;
                    indTrack_0 = i;
                    indTrack_1 = j;
                }

            }
            for(size_t j = closestIndex ; j < route.route_points.size() ; ++j){
                const RoutePoint &rp1 = route.route_points.at(j);
                if(rp1.track_id != closestRP.track_id || rp1.time_stamp >= 0.0001)
                    break;
                double dist = arolib::geometry::calc_dist(rp0, rp1);
                if(minDistInterTrack > dist){
                    minDistInterTrack = dist;
                    indTrack_0 = i;
                    indTrack_1 = j;
                }
            }
            if(indTrack_1 < (int)closestIndex){
                for(size_t i = indTrack_1 ; i < closestIndex ; ++i)
                    pointsOut.push_back(route.route_points.at(i).point());
            }
            else{
                for(int i = (int)closestIndex+1 ; i <= indTrack_1 ; ++i)
                    pointsOut.push_back(route.route_points.at(i).point());
            }
            for(size_t i = indTrack_0 ; i < fistGoodIndex ; ++i)
                pointsOut.push_back(route.route_points.at(i).point());
            return pointsOut;
        }

        double minDistToClosestPoint = std::numeric_limits<double>::max();
        int indClosestToClosest = -1;
        for(int i = fistGoodIndex-1; i >= 0 ; --i){
            const RoutePoint &rp = route.route_points.at(i);
            if(rp.track_id != firstRP.track_id)
                break;
            double dist = arolib::geometry::calc_dist(closestRP, rp);
            if(minDistToClosestPoint > dist){
                minDistToClosestPoint = dist;
                indClosestToClosest = i;
            }
        }
        if(indClosestToClosest >= 0){
            for(size_t i = indClosestToClosest ; i < fistGoodIndex ; ++i)
                pointsOut.push_back(route.route_points.at(i).point());
        }
        return pointsOut;
    }

    //in different tracks --> connect tracks through headland
    auto path1_a = getPathInTrack(route.route_points, closestIndex, -1);
    auto path1_b = getPathInTrack(route.route_points, closestIndex, route.route_points.size());

    Point path1_a_pointNearHeadland = closestRP.point();
    Point path1_b_pointNearHeadland = closestRP.point();
    if(!path1_a.empty())
        path1_a_pointNearHeadland = path1_a.back();
    if(!path1_b.empty())
        path1_b_pointNearHeadland = path1_b.back();

    auto pathHeadlan_a = arolib::geometry::getShortestGeometryPart(headland,
                                                 path1_a_pointNearHeadland,
                                                 path2_pointNearHeadland,
                                                 false);
    auto pathHeadlan_b = arolib::geometry::getShortestGeometryPart(headland,
                                                 path1_b_pointNearHeadland,
                                                 path2_pointNearHeadland,
                                                 false);

    path1_a.insert( path1_a.end(), pathHeadlan_a.begin(), pathHeadlan_a.end() );
    path1_b.insert( path1_b.end(), pathHeadlan_b.begin(), pathHeadlan_b.end() );

    if( arolib::geometry::getGeometryLength(path1_a) > arolib::geometry::getGeometryLength(path1_b) )
        pointsOut.insert(pointsOut.end(), path1_b.begin(), path1_b.end());
    else
        pointsOut.insert(pointsOut.end(), path1_a.begin(), path1_a.end());

    pointsOut.insert( pointsOut.end(), path2_toHeadland.rbegin(), path2_toHeadland.rend() );

    return pointsOut;

}

std::map<MachineId_t, Route> FieldProcessPlanner::getInitialPathToWorkingRoutes(DirectedGraph::Graph &graph,
                                                                                std::vector<Route> &mainRoutes,
                                                                                const Subfield &subfield,
                                                                                const std::vector<Machine> &machines,
                                                                                const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                                                const PlannerParameters &plannerParameters,
                                                                                std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator)
{
    std::map<MachineId_t, Route> routesInit;

    std::map<MachineId_t, Machine> harvestersMap;//for easier access
    for(auto& m : machines){
        if(m.isOfWorkingType(true))
            harvestersMap[m.id] = m;
    }

    for(auto & route : mainRoutes){
        Route routeInit;
        routeInit.machine_id = route.machine_id;

        auto it_m = harvestersMap.find(route.machine_id);
        auto it_mdi = machineCurrentStates.find(route.machine_id);

        if(route.route_points.empty()
                || it_m == harvestersMap.end()
                || it_mdi == machineCurrentStates.end() ){//if no current location is given, assume it start at the first route point
            routesInit[route.machine_id] = routeInit;
            continue;
        }

        const MachineDynamicInfo& mdi = it_mdi->second;
        const Machine &machine = it_m->second;
        double machine_speed = machine.calcSpeed( mdi.bunkerMass );

        //if the working machines is inside the field and very close to the first route point, connect it directly with the first route point
        if(arolib::geometry::in_polygon(mdi.position, subfield.boundary_outer)){
            auto dist = arolib::geometry::calc_dist(route.route_points.front(), mdi.position);
            if( dist < 2 * machine.working_width){
                RoutePoint rp;
                rp.type = RoutePoint::INITIAL_POSITION;
                rp.time_stamp = 0;
                rp.point() = mdi.position;
                routeInit.route_points.emplace_back(rp);

                rp = route.route_points.front();
                rp.time_stamp += ( dist / machine_speed );
                routeInit.route_points.emplace_back(rp);

                routesInit[route.machine_id] = routeInit;
                continue;
            }
        }

        auto it_vt = graph.initialpoint_vertex_map().find(route.machine_id);
        if(it_vt == graph.initialpoint_vertex_map().end()){//if no initial location (vertex) exists, assume it start at the first route point
            routesInit[route.machine_id] = routeInit;
            continue;
        }

        auto it_vt_2 = graph.routepoint_vertex_map().find( route.route_points.front() );
        if(it_vt_2 == graph.routepoint_vertex_map().end()){
            logger().printError(__FUNCTION__, "Vertex corresponding to start route point not found in graph's map");
            routesInit[route.machine_id] = routeInit;
            continue;
        }

        //connect using AStar
        Astar::PlanParameters astarParams;
        astarParams.start_vt = it_vt->second;
        astarParams.goal_vt = it_vt_2->second;
        astarParams.start_time = 0;
        astarParams.max_time_visit = 0;//do not drive over any of the vertices that will be worked on (except for the first one, which is the goal)
        astarParams.max_time_goal = std::numeric_limits<double>::max();//no limits, just get there
        astarParams.machine = machine;
        astarParams.machine_speed = machine_speed;
        astarParams.initial_bunker_mass = mdi.bunkerMass;
        astarParams.overload = false;
        astarParams.includeWaitInCost = false;
        astarParams.exclude = getExcludeVertices_initMainRoute(graph, astarParams.goal_vt);
        astarParams.restrictedMachineIds = {machine.id};
        astarParams.restrictedMachineIds_futureVisits = {};//@todo check

        const Astar::AStarSettings& astarSettings_original = plannerParameters;
        Astar::AStarSettings astarSettings = astarSettings_original;
        astarSettings.includeWaitInCost = false;

        auto foldername_planData = m_foldername_planData;
        if(!foldername_planData.empty()){
            foldername_planData += "init/M" + std::to_string(route.machine_id) + "/";
            io::create_directory(foldername_planData, true);
        }

        Astar planner (astarParams,
                       astarSettings,
                       RoutePoint::TRANSIT,
                       foldername_planData,
                       &m_logger);

        if( !planner.plan(graph, edgeCostCalculator) ){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, 10, "Error getting initial path for machine ", machine.id);
            routesInit[route.machine_id] = routeInit;
            continue;
        }

        AstarPlan plan = planner.getPlan();

        //remove visit period of last route point (it will be updated with the base routes)
        plan.visit_periods.pop_back();

        plan.insertIntoGraph(graph);
        routeInit.route_points = plan.route_points_;

        //adjust some of the route points' types
        for(size_t i = 0 ; i < routeInit.route_points.size() ; ++i){
            if(i == 0)
                routeInit.route_points.at(i).type = RoutePoint::INITIAL_POSITION;
            else if(i+1 < routeInit.route_points.size()
                    && routeInit.route_points.at(i).type != RoutePoint::FIELD_ENTRY )
                routeInit.route_points.at(i).type = RoutePoint::TRANSIT;
        }
        routesInit[route.machine_id] = routeInit;
    }


    //adjust timestamps of route and vertices
    for(auto & route : mainRoutes){
        auto it_ir = routesInit.find(route.machine_id);
        if(it_ir == routesInit.end())
            continue;

        auto& routeInit = it_ir->second;
        if(routeInit.route_points.empty())
            continue;

        double delta_time = routeInit.route_points.back().time_stamp;

        OLVPlan::addDelayToHarvRoute(graph, route, 0, delta_time);//this also sets the right timestamps to the vertices (until now, the vertices had the timestamps of the initial headland and infield harvester routes)

    }

    return routesInit;
}

void FieldProcessPlanner::addWMVisitPeriods(DirectedGraph::Graph &graph,
                                            const std::vector<Route> &mainRoutes,
                                            const std::vector<Machine> &machines)
{
    std::map<MachineId_t, Machine> workingMachines;
    double r_min = std::numeric_limits<double>::max();
    for(auto& m : machines){
        if(!m.isOfWorkingType(true))
            continue;
        workingMachines[m.id] = m;
        double w = std::numeric_limits<double>::lowest();
        w = std::max(w, m.working_width);
        w = std::max(w, m.width);

        if(m.working_width > 1e-3)
            r_min = std::min(r_min, m.working_width);
        if(m.width > 1e-3)
            r_min = std::min(r_min, m.width);
        if(m.length > 1e-3)
            r_min = std::min(r_min, m.length);
    }
    if(r_min > std::numeric_limits<double>::max() - 1)
        r_min = 0.1;

    for(auto route : mainRoutes){

        double r = 0;
        auto it_m = workingMachines.find(route.machine_id);
        if(it_m != workingMachines.end())
            r = it_m->second.workingRadius();

        for(size_t i = 0 ; i < route.route_points.size() ; ++i ){
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
}

std::set<DirectedGraph::vertex_t> FieldProcessPlanner::getExcludeVertices_initMainRoute(DirectedGraph::Graph &graph, DirectedGraph::vertex_t goal_vt)
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
        if( std::abs( goal_prop.route_point.track_id - successor_prop.route_point.track_id ) == 1 )
            exclude.insert(successor);

    }

    for(auto& rv_it : graph.resourcepoint_vertex_map())
        exclude.insert(rv_it.second);

    return exclude;
}

void FieldProcessPlanner::addInitialPathToMainRoutes(const std::map<MachineId_t, Route> &routesInit, std::vector<Route> &mainRoutes)
{

    for(auto & route : mainRoutes){
        auto it_ir = routesInit.find(route.machine_id);
        if(it_ir == routesInit.end())
            continue;

        auto& routeInit = it_ir->second;
        if(routeInit.route_points.empty())
            continue;

        route.route_points.insert( route.route_points.begin(), routeInit.route_points.begin(), routeInit.route_points.end()-1 );

    }
}

void FieldProcessPlanner::shiftRoutesInTime(std::vector<Route> &mainRoutes, std::vector<Route> &secRoutes, double deltaTime)
{
    for(auto& route : mainRoutes){
        for(auto& rp : route.route_points)
            rp.time_stamp += deltaTime;
    }
    for(auto& route : secRoutes){
        for(size_t i = 1 ; i < route.route_points.size() ; ++i)
            route.route_points.at(i).time_stamp += deltaTime;
    }
}

void FieldProcessPlanner::addInitialPointsToMainRoutes(std::vector<Route> &mainRoutes, const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates)
{
    for(auto& route : mainRoutes){
        if( !route.route_points.empty()
                && route.route_points.front().type == RoutePoint::INITIAL_POSITION )
            continue;
        auto it = machineCurrentStates.find( route.machine_id );
        if(it != machineCurrentStates.end()){
            const MachineDynamicInfo& mdi = it->second;
            RoutePoint rp;
            rp.type = RoutePoint::INITIAL_POSITION;
            rp.time_stamp = 0;
            rp.point() = mdi.position;
            rp.harvested_mass = 0;
            rp.harvested_volume = 0;
            route.route_points.insert( route.route_points.begin(), rp );
        }
    }
}

bool FieldProcessPlanner::areWorkingMachinesOutOfField(const std::vector<Machine> &machines, const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates, const Polygon &field_Boundary)
{
    bool machinesOutside = false;
    for(auto &m : machines){
        if(!m.isOfWorkingType(true))
            continue;
        auto it_m = machineCurrentStates.find(m.id);
        if(it_m != machineCurrentStates.end()){
            if(!arolib::geometry::in_polygon(it_m->second.position, field_Boundary)){
                machinesOutside = true;
                break;
            }
        }
        else{
            machinesOutside = true;
            break;
        }
    }
    return machinesOutside;
}

size_t FieldProcessPlanner::removeInitialUselessPoints(std::vector<Route> &mainRoutes)
{
    size_t count = 0;
    for(auto &route : mainRoutes){
        for(size_t i = 0 ; i < route.route_points.size() ; ++i){
            if(route.route_points.at(i).time_stamp >= -0.0001){
                count += i;
                route.route_points.erase( route.route_points.begin(), route.route_points.begin()+i );
                break;
            }
        }
    }
    return count;
}

std::vector<Point> FieldProcessPlanner::getPathInTrack(const std::vector<RoutePoint> &routePoints,
                                                  int indBegin,
                                                  int indEnd,
                                                  double timeLimit)
{
    std::vector<Point> path;
    if(indBegin >= (int)routePoints.size())
        return path;

    indEnd = std::max( -1 , std::min( (int)routePoints.size(), indEnd ) );

    if(indBegin < indEnd){
        for(int i = indBegin ; i < indEnd ; ++i){
            if( routePoints.at(i).track_id != routePoints.at(indBegin).track_id
                    || routePoints.at(i).time_stamp > timeLimit)
                break;
            path.push_back( routePoints.at(i).point() );
        }
    }
    else{
        for(int i = indBegin ; i > indEnd ; --i){
            if( routePoints.at(i).track_id != routePoints.at(indBegin).track_id
                    || routePoints.at(i).time_stamp > timeLimit )
                break;
            path.push_back( routePoints.at(i).point() );
        }
    }

    return path;
}

double FieldProcessPlanner::getHarvestedMassLimit(const Subfield &subfield,
                                                  const std::vector<Machine> &machines,
                                                  const PlannerParameters &plannerParameters,
                                                  const ArolibGrid_t &yieldmap,
                                                  const ArolibGrid_t &remainingAreaMap)
{
    double fieldArea = arolib::geometry::calc_area(subfield.boundary_outer);
    if(fieldArea <= m_partialPlanAreaThreshold)//small field -> no limit
        return -1;

    double fieldAreaRem = fieldArea;
    if(remainingAreaMap.isAllocated()){
        bool bePrecise = remainingAreaMap.getSizeX() * remainingAreaMap.getSizeY() < 50*50;//be precise only if the resolution of the field is 'low'
        bool errorTmp;
        double rem = remainingAreaMap.getPolygonComputedValue(subfield.boundary_outer,
                                                              ArolibGrid_t::AVERAGE_TOTAL,
                                                              bePrecise,
                                                              &errorTmp);
        if(!errorTmp){
            rem = std::min(1.0, rem+0.05);
            fieldAreaRem *= rem;
        }
    }

    if(fieldAreaRem <= m_partialPlanAreaThreshold)
        return -1;//small unharvested area -> no limit

    double bunkerMassTotal = 0;
    int countOLVs = 0;

    //get the sum of bunker capacities fo the OLVs
    for(auto &m : machines){
        if(m.machinetype == arolib::Machine::OLV){
            bunkerMassTotal += m.bunker_mass;
            ++countOLVs;
        }
    }
    if(countOLVs == 0)
        return -1;//no olvs --> no limit (no overload will be planned, so the mass limit is useless)

    //obtain the total amount of yield in the subfield
    double yieldProportion = plannerParameters.avgMassPerArea;
    if(yieldmap.isAllocated()){
        bool errorTmp;
        auto yp = yieldmap.getPolygonComputedValue(subfield.boundary_outer,
                                                   ArolibGrid_t::AVERAGE_TOTAL,
                                                   true,
                                                   &errorTmp);
        if(!errorTmp)
            yieldProportion = yp;
    }
    double fieldYield = arolib::t_ha2Kg_sqrm(yieldProportion)*fieldArea;

    //calculate the harvested mass limit based on the unharvested area, amount of yield in the subfield and the olvs

    double mult = 1.3 * m_partialPlanAreaThreshold / fieldAreaRem - 0.3;
    auto val = std::max((countOLVs+1)/countOLVs * bunkerMassTotal, fieldYield * mult);

    m_logger.printOut(arolib::LogLevel::INFO, __FUNCTION__, "Using a maximum harvested mass for the OLV planner of " + std::to_string(val)
                                                            + " Kg from a total yield in the subfield of " + std::to_string(fieldYield) + " Kg (" + std::to_string((val/fieldYield)*100) + "%)");

    return val;
}

void FieldProcessPlanner::addRoutesOverruns(DirectedGraph::Graph &graph, const std::vector<Machine> &machines, const std::vector<Route> &routes)
{
    auto machinesMap = Machine::toMachineIdMap(machines);
    for(auto &route : routes){

        auto it_m = machinesMap.find(route.machine_id);
        if(it_m == machinesMap.end())
            continue;

        for(size_t i = 0 ; i+1 < route.route_points.size() ; ++i){
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
            overrun.machine_id = it_m->second.id;
            overrun.weight = it_m->second.weight;

            graph.addOverrun( it_0->second, it_1->second, overrun );
        }

    }

    //@todo add overruns corresponding to non-working route segments

}

void FieldProcessPlanner::saveVisitSchedule(const DirectedGraph::Graph &graph, const std::string &filename)
{
    if(m_foldername_visitSchedule.empty())
        return;

    graph.saveVisitSchedule(m_foldername_visitSchedule + filename);
}

}
