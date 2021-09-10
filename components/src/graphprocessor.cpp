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
 
#include "arolib/components/graphprocessor.h"

namespace arolib {

GraphProcessor::GraphProcessor(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{

}

AroResp GraphProcessor::createGraph(const Subfield &subfield, const std::vector<Route> &baseRoutes, const std::vector<Machine> &machines, const GraphProcessor::Settings &settings, OutFieldInfo &outFieldInfo, const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates, DirectedGraph::Graph &graph)
{
    LoggingComponent::LoggersHandler lh(true);//will be reset on destruction
    LoggingComponent::setTemporalLoggersParent(lh, *this, graph);

    try {

        if (subfield.resource_points.empty()) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No ressource points given." );
            return AroResp(1, "No resource points given." );
        }

        for(auto &rp : subfield.resource_points){
            if( outFieldInfo.mapUnloadingCosts().find(OutFieldInfo::AllResourcePoints) == outFieldInfo.mapUnloadingCosts().end() ){
                OutFieldInfo::UnloadingCosts uc( rp.defaultUnloadingTime, rp.defaultUnloadingTimePerKg );
                if( outFieldInfo.addDefaultUnloadingCosts(rp.id, uc, false) )
                    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Adding default unloading costs for all machines for resource point " + std::to_string(rp.id) );
            }
        }

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Building the graph...");
        DirectedGraph::GraphBuilder_TracksBased graphBuilder(m_logger.logLevel());
        graphBuilder.logger().setParent(&m_logger);
        graph = graphBuilder.buildGraph(subfield,
                                        baseRoutes,
                                        subfield.resource_points,
                                        subfield.access_points,
                                        outFieldInfo,
                                        machines,
                                        machineCurrentStates,
                                        settings.workingWidth,
                                        settings.workingWidthHL,
                                        settings.incVisitPeriods,
                                        m_filename_graphBuilding);

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Graph created!");
        return AroResp(0, "OK" );

    }
    catch (std::exception &e) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what() );
        return AroResp(1, "Exception cought: ", e.what() );
    }

}

AroResp GraphProcessor::createGraph(PlanningWorkspace &pw, size_t subfieldIdx, const Settings& settings, DirectedGraph::Graph &graph)
{
    if(subfieldIdx >= getField(pw).subfields.size()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfield index");
        return AroResp(1, "Invalid subfield index");
    }
    const auto &subfield = getField(pw).subfields.at(subfieldIdx);
    const auto &machines = getMachines(pw);
    const auto &baseRoutes = getConnectedBaseRoutes(pw)[subfieldIdx];
    auto &outFieldInfo = getOutFieldInfo(pw);
    const auto &machineCurrentStates = getMachineCurrentStates(pw);

    m_planningWorkspace = &pw;
    m_pw_subfieldIdx = subfieldIdx;

    auto ret = createGraph(subfield,
                           baseRoutes,
                           machines,
                           settings,
                           outFieldInfo,
                           machineCurrentStates,
                           graph);

    m_planningWorkspace = nullptr;

    return ret;

}

AroResp GraphProcessor::createGraph_old(const Subfield& subfield,
                                          const std::vector<arolib::HeadlandRoute> &baseRoutes_headland,
                                          const std::vector<arolib::Route> &baseRoutes_infield,
                                          const std::vector<arolib::Machine>& machines,
                                          const Settings &settings,
                                          OutFieldInfo &outFieldInfo,
                                          const std::map<MachineId_t, arolib::MachineDynamicInfo>& machineCurrentStates,
                                          DirectedGraph::Graph& graph )
{
    AroResp compResp;

    LoggingComponent::LoggersHandler lh(true);//will be reset on destruction
    LoggingComponent::setTemporalLoggersParent(lh, *this, graph);

    try {

        if (subfield.resource_points.empty()) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No ressource points given." );
            return AroResp(1, "No resource points given." );
        }
        if (baseRoutes_infield.empty() && baseRoutes_headland.empty()) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Routes missing, needed by the planner" );
            return AroResp(1, "Routes missing, needed by the planner" );
        }

        for(auto &rp : subfield.resource_points){
            if( outFieldInfo.mapUnloadingCosts().find(OutFieldInfo::AllResourcePoints) == outFieldInfo.mapUnloadingCosts().end() ){
                OutFieldInfo::UnloadingCosts uc( rp.defaultUnloadingTime, rp.defaultUnloadingTimePerKg );
                if( outFieldInfo.addDefaultUnloadingCosts(rp.id, uc, false) )
                    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Adding default unloading costs for all machines for resource point " + std::to_string(rp.id) );
            }
        }

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Building the graph...");
        DirectedGraph::GraphBuilder_RoutesBased graphBuilder(settings.bePreciseWithMaps, m_logger.logLevel());
        graphBuilder.logger().setParent(&m_logger);
        graphBuilder.setOutputFile(m_filename_graphBuilding);

        if(m_planningWorkspace){
            if(m_pw_subfieldIdx >= getField(*m_planningWorkspace).subfields.size())
                throw std::invalid_argument( std::string(__FUNCTION__) + ": Invalid subfield index" );

            graph = graphBuilder.buildGraph(*m_planningWorkspace,
                                             m_pw_subfieldIdx,
                                             baseRoutes_infield,
                                             baseRoutes_headland,
                                             outFieldInfo,
                                             settings.workingWidth,
                                             settings.workingWidthHL,
                                             settings.incVisitPeriods,
                                             getField(*m_planningWorkspace).subfields.at(m_pw_subfieldIdx).boundary_outer);
        }
        else{
            graph = graphBuilder.buildGraph(subfield,
                                             baseRoutes_infield,
                                             baseRoutes_headland,
                                             subfield.resource_points,
                                             subfield.access_points,
                                             outFieldInfo,
                                             machines,
                                             machineCurrentStates,
                                             settings.workingWidth,
                                             settings.workingWidthHL,
                                             settings.incVisitPeriods,
                                             subfield.boundary_outer);
        }

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Graph created!");
        return AroResp(0, "OK" );

    }
    catch (std::exception &e) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what() );
        return AroResp(1, "Exception cought: ", e.what() );
    }


}

AroResp GraphProcessor::createGraph_old(PlanningWorkspace &pw, size_t subfieldIdx, const Settings& settings, DirectedGraph::Graph &graph)
{
    if(subfieldIdx >= getField(pw).subfields.size()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfield index");
        return AroResp(1, "Invalid subfield index");
    }
    const auto &subfield = getField(pw).subfields.at(subfieldIdx);
    const auto &machines = getMachines(pw);
    const auto &baseRoutes_headland = getBaseRoutesProcessed_headland(pw)[subfieldIdx];
    const auto &baseRoutes_infield = getBaseRoutesProcessed_infield(pw)[subfieldIdx];
    auto &outFieldInfo = getOutFieldInfo(pw);
    const auto &machineCurrentStates = getMachineCurrentStates(pw);

    m_planningWorkspace = &pw;
    m_pw_subfieldIdx = subfieldIdx;

    auto ret = createGraph_old(subfield,
                           baseRoutes_headland,
                           baseRoutes_infield,
                           machines,
                           settings,
                           outFieldInfo,
                           machineCurrentStates,
                           graph);

    m_planningWorkspace = nullptr;

    return ret;

}

AroResp GraphProcessor::createSimpleGraph(const Subfield &subfield,
                                          const std::vector<Machine> &machines,
                                          const Settings &settings,
                                          OutFieldInfo &outFieldInfo,
                                          const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                          const ArolibGrid_t &remainingAreaMap,
                                          DirectedGraph::Graph &graph)
{
    std::vector<arolib::HeadlandRoute> headlandRoute(1);
    std::vector<arolib::Route> infieldRoute(1);
    AroResp resp = generatePseudoRoutes(subfield,
                                        settings,
                                        remainingAreaMap,
                                        headlandRoute.back(),
                                        infieldRoute.back());
    if(resp.isError())
        return resp;

    return createGraph_old(subfield,
                       headlandRoute,
                       infieldRoute,
                       machines,
                       settings,
                       outFieldInfo,
                       machineCurrentStates,
                       graph);

    return AroResp::ok();
}

AroResp GraphProcessor::createSimpleGraph(PlanningWorkspace &pw, size_t subfieldIdx, const GraphProcessor::Settings &settings, DirectedGraph::Graph &graph)
{
    if(subfieldIdx >= getField(pw).subfields.size()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfield index");
        return AroResp(1, "Invalid subfield index");
    }
    const auto &subfield = getField(pw).subfields.at(subfieldIdx);
    const auto &machines = getMachines(pw);
    const auto &remainingAreaMap = getMaps(pw).at( PlanningWorkspace::GridType::REMAINING_AREA );
    auto &outFieldInfo = getOutFieldInfo(pw);
    const auto &machineCurrentStates = getMachineCurrentStates(pw);

    m_planningWorkspace = &pw;
    m_pw_subfieldIdx = subfieldIdx;

    auto ret = createSimpleGraph(subfield,
                                 machines,
                                 settings,
                                 outFieldInfo,
                                 machineCurrentStates,
                                 remainingAreaMap,
                                 graph);

    m_planningWorkspace = nullptr;

    return ret;

}

AroResp GraphProcessor::addInitialPositons(DirectedGraph::Graph &graph, const Subfield &subfield, const std::vector<Machine> &machines,
                                           OutFieldInfo &outFieldInfo, const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                           const Polygon &field_bounday, bool locateMissingMachinesAtFAP)
{
    return insertOrReplaceInitialPositions(graph,
                                           subfield,
                                           machines,
                                           {},//all
                                           machineCurrentStates,
                                           outFieldInfo,
                                           field_bounday,
                                           true,
                                           true,
                                           locateMissingMachinesAtFAP);

}


void GraphProcessor::setOutputFiles(const std::string &filename_graphBuilding)
{
    m_filename_graphBuilding = filename_graphBuilding;
}

bool GraphProcessor::addEdge(DirectedGraph::Graph &graph,
                             DirectedGraph::EdgeType edgeType,
                             const DirectedGraph::vertex_t &vt1, const DirectedGraph::vertex_t &vt2,
                             const RoutePoint &rp1, const RoutePoint &rp2,
                             double distance, double width,
                             bool bidirectional, bool onlyIfNotExisting, bool overwrite)
{
    /// create edge
    DirectedGraph::edge_property edge_prop;
    edge_prop.p0 = rp1.point();
    edge_prop.p1 = rp2.point();
    edge_prop.defWidth = width;
    edge_prop.distance = distance;
    edge_prop.edge_type = edgeType;

    return graph.addEdge(vt1, vt2, edge_prop, bidirectional, onlyIfNotExisting, overwrite);
}

AroResp GraphProcessor::generatePseudoRoutes(const Subfield &subfield, const Settings &settings, const ArolibGrid_t &remainingAreaMap, HeadlandRoute &headlandRoute, Route &infieldRoute) const
{
    const double remainingAreaThreshold = 0.5;
    headlandRoute.route_points.clear();
    infieldRoute.route_points.clear();

    if( subfield.tracks.empty() && subfield.headlands.complete.tracks.empty() )
        return AroResp( 1, "No valid tracks to generate pseudo routes" );

    for(auto& track : subfield.headlands.complete.tracks){
        auto& route = headlandRoute;
        if(track.points.size() < 2)
            continue;
        for(size_t i = 0 ; i < track.points.size() ; ++i){
            auto& p = track.points.at(i);
            route.route_points.push_back(RoutePoint());
            RoutePoint& rp = route.route_points.back();
            rp.point() = p;
            rp.track_id = track.id;
            rp.time_stamp = -1;
            if(remainingAreaMap.isAllocated()){
                bool errorTmp;
                double remainingArea = remainingAreaMap.getValue(p, &errorTmp);

                if (errorTmp || remainingArea > remainingAreaThreshold)
                    rp.time_stamp = std::numeric_limits<double>::max() - 10;
            }
            if(i == 0)
                rp.type = RoutePoint::TRACK_START;
            else if(i+1 == track.points.size())
                rp.type = RoutePoint::TRACK_END;
            else
                rp.type = RoutePoint::HARVESTING;
        }
    }

    for(auto& track : subfield.tracks){
        auto& route = infieldRoute;
        if(track.points.size() < 2)
            continue;
        for(size_t i = 0 ; i < track.points.size() ; ++i){
            auto& p = track.points.at(i);

            if(i==0 && !route.route_points.empty()){//add a pseudo HL point to avoid connection with previout (track end) point
                Point p2 = track.points.at(i+1);
                p2 = arolib::geometry::extend_line( p2, p, arolib::geometry::calc_dist(p, p2) );
                route.route_points.push_back(RoutePoint());
                RoutePoint& rp2 = route.route_points.back();
                rp2.point() = p2;
                rp2.track_id = -1;
                rp2.time_stamp = -1;
                rp2.type = RoutePoint::HEADLAND;
            }

            route.route_points.push_back(RoutePoint());
            RoutePoint& rp = route.route_points.back();
            rp.point() = p;
            rp.track_id = track.id;
            rp.time_stamp = -1;
            if(remainingAreaMap.isAllocated()){
                bool errorTmp;
                double remainingArea = remainingAreaMap.getValue(p, &errorTmp);

                if (errorTmp || remainingArea > remainingAreaThreshold)
                    rp.time_stamp = std::numeric_limits<double>::max() - 10;
            }
            if(i == 0)
                rp.type = RoutePoint::TRACK_START;
            else if(i+1 == track.points.size())
                rp.type = RoutePoint::TRACK_END;
            else
                rp.type = RoutePoint::HARVESTING;
        }
    }

    if( headlandRoute.route_points.empty() && infieldRoute.route_points.empty() )
        return AroResp( 1, "No valid tracks to generate pseudo routes" );

    return AroResp::ok();
}

}
