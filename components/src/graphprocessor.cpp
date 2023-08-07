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
 
#include "arolib/components/graphprocessor.h"

namespace arolib {

bool GraphProcessor::Settings::parseFromStringMap(Settings &params, const std::map<std::string, std::string> &map, bool strict)
{
    GraphProcessor::Settings tmp;

    std::map<std::string, double*> dMap = { {"workingWidth" , &tmp.workingWidth},
                                            {"workingWidthHL" , &tmp.workingWidthHL} };
    std::map<std::string, bool*> bMap = { {"bePreciseWithMaps" , &tmp.bePreciseWithMaps},
                                            {"incVisitPeriods" , &tmp.incVisitPeriods} };

    if( !setValuesFromStringMap( map, dMap, strict)
            || !setValuesFromStringMap( map, bMap, strict) )
        return false;

    params = tmp;
    return true;
}

std::map<std::string, std::string> GraphProcessor::Settings::parseToStringMap(const Settings &params)
{
    std::map<std::string, std::string> ret;

    ret["workingWidth"] = double2string( params.workingWidth );
    ret["workingWidthHL"] = double2string( params.workingWidthHL );
    ret["bePreciseWithMaps"] = std::to_string( params.bePreciseWithMaps );
    ret["incVisitPeriods"] = std::to_string( params.incVisitPeriods );

    return ret;
}

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
//            logger().printOut(LogLevel::ERROR, __FUNCTION__, "No ressource points given." );
//            return AroResp(1, "No resource points given." );

            logger().printOut(LogLevel::WARNING, __FUNCTION__, "No ressource points given." );
        }

        for(auto &rp : subfield.resource_points){
            if( outFieldInfo.mapUnloadingCosts().find(OutFieldInfo::AllResourcePoints) == outFieldInfo.mapUnloadingCosts().end() ){
                OutFieldInfo::UnloadingCosts uc( rp.defaultUnloadingTime, rp.defaultUnloadingTimePerKg );
                if( outFieldInfo.addDefaultUnloadingCosts(rp.id, uc, false) )
                    logger().printOut(LogLevel::INFO, __FUNCTION__, "Adding default unloading costs for all machines for resource point " + std::to_string(rp.id) );
            }
        }

        logger().printOut(LogLevel::INFO, __FUNCTION__, "Building the graph...");
        DirectedGraph::GraphBuilder_TracksBased graphBuilder(logger().logLevel());
        graphBuilder.logger().setParent(loggerPtr());
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

        logger().printOut(LogLevel::INFO, __FUNCTION__, "Graph created!");
        return AroResp(0, "OK" );

    }
    catch (std::exception &e) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what() );
        return AroResp(1, "Exception cought: ", e.what() );
    }

}

AroResp GraphProcessor::createSimpleGraph(const Subfield &subfield,
                                          const std::vector<Machine> &machines,
                                          const Settings &settings,
                                          OutFieldInfo &outFieldInfo,
                                          const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                          const ArolibGrid_t &remainingAreaMap,
                                          DirectedGraph::Graph &graph)
{
    AroResp resp = createGraph(subfield,
                               {},
                               machines,
                               settings,
                               outFieldInfo,
                               machineCurrentStates,
                               graph);
    if(resp.isError())
        return resp;

    if(!remainingAreaMap.isAllocated())
        return AroResp::ok();

    const double remainingAreaThreshold = 0.5;
    for(auto vt_it = boost::vertices(graph); vt_it.first != vt_it.second; vt_it.first++){
        RoutePoint& rp = graph[*vt_it.first].route_point;
        if(rp.track_id >= 0) {
            bool errorTmp = true;
            double remainingArea = 0;
            if(remainingAreaMap.hasValue(rp))
                remainingArea = remainingAreaMap.getValue(rp, &errorTmp);

            if (errorTmp || remainingArea > remainingAreaThreshold){
                rp.time_stamp = std::numeric_limits<double>::max() - 10;
                rp.type = RoutePoint::DEFAULT;
            }
        }
    }


    return AroResp::ok();
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

AroResp GraphProcessor::addBaseRouteInformationToGraph(DirectedGraph::Graph &graph, const std::vector<Machine> &machines, const std::vector<Route> &baseWorkingRoutes, bool includeVisitPeriods) const
{
    LoggingComponent::LoggersHandler lh(true);//will be reset on destruction
    LoggingComponent::setTemporalLoggersParent(lh, *this, graph);

    try {

        logger().printOut(LogLevel::INFO, __FUNCTION__, "Editing the graph...");
        DirectedGraph::GraphBuilder_TracksBased graphBuilder(logger().logLevel());
        graphBuilder.logger().setParent(loggerPtr());
        auto aroResp = graphBuilder.addBaseRouteInformationToGraph(graph,
                                                                   machines,
                                                                   baseWorkingRoutes,
                                                                   includeVisitPeriods);
        if(aroResp.isError())
            return AroResp::LoggingResp(1, "Error adding information to graph: " + aroResp.msg, logger(), LogLevel::ERROR, __FUNCTION__);

        logger().printOut(LogLevel::INFO, __FUNCTION__, "Graph edited!");
        return AroResp(0, "OK" );

    }
    catch (std::exception &e) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what() );
        return AroResp(1, "Exception cought: ", e.what() );
    }

}

AroResp GraphProcessor::resetVertexTimestampsFromWorkedArea(DirectedGraph::Graph &graph, std::shared_ptr<const ArolibGrid_t> remainingAreaMap, bool resetIfWorked, std::shared_ptr<gridmap::GridCellsInfoManager> cim) const
{
    static const std::string RemainingAreaMapName = "RAM";
    static const double ThresholdIsWorked = 0.5;

    if(!remainingAreaMap || !remainingAreaMap->isAllocated()){
        logger().printWarning(__FUNCTION__, "Remaining-area map is not valid");
        return AroResp(0, "OK" );
    }

    gridmap::SharedGridsManager gm;
    gm.setCellsInfoManager(cim);
    gm.addGrid(RemainingAreaMapName, remainingAreaMap);

    auto isSegmentWorked = [&gm](const Point &p0, const Point &p1, double width)->bool{
        bool errorTmp = true;
        double value = 1;
        double area = arolib::geometry::calc_dist(p0, p1) * width;

        if( area > 1e-9 ){
            std::vector<gridmap::GridmapLayout::GridCellOverlap> cellsInfo;
            gm.getCellsInfoUnderLine(RemainingAreaMapName, p0, p1, width, gridmap::SharedGridsManager::PRECISE_ONLY_IF_AVAILABLE, cellsInfo);

            if(!cellsInfo.empty())
                value = gm.getGrid(RemainingAreaMapName)->getCellsComputedValue( cellsInfo,
                                                                                 ArolibGrid_t::AVERAGE_TOTAL,
                                                                                 area,
                                                                                 false,
                                                                                 &errorTmp );
        }
        else{
            arolib::Point p0_1;
            p0_1.x = 0.5*( p0.x + p1.x );
            p0_1.y = 0.5*( p0.y + p1.y );
            if(gm.getGrid(RemainingAreaMapName)->hasValue(p0_1))
                value = gm.getGrid(RemainingAreaMapName)->getValue(p0_1, &errorTmp);
        }

        return (!errorTmp && value < ThresholdIsWorked);
    };

    for(auto it_vt : graph.routepoint_vertex_map()){
        auto vt = it_vt.second;

        DirectedGraph::vertex_property& vt_prop = graph[vt];
        if(vt_prop.route_point.time_stamp < 1e-9)
            continue;

        std::unordered_map<DirectedGraph::vertex_t, double> adjVts;
        for( auto _edges = boost::in_edges(vt, graph) ; _edges.first != _edges.second ; _edges.first++){
            DirectedGraph::edge_property ep = graph[*_edges.first];
            adjVts[ source(*_edges.first, graph) ] = ep.defWidth;
        }
        for( auto _edges = boost::out_edges(vt, graph) ; _edges.first != _edges.second ; _edges.first++){
            DirectedGraph::edge_property ep = graph[*_edges.first];
            adjVts[ target(*_edges.first, graph) ] = ep.defWidth;
        }

        if(adjVts.empty())
            continue;

        Point p1 = vt_prop.route_point.point();
        bool reset = true;
        for(auto it_vt2 : adjVts){
            DirectedGraph::vertex_t vt_adj = it_vt2.first;
            auto width = it_vt2.second;
            const DirectedGraph::vertex_property& vt_prop_adj = graph[vt_adj];
            Point p2 = vt_prop_adj.route_point.point();
            p2 = geometry::getCentroid(p1, p2);
            bool worked = isSegmentWorked(p1, p2, width);
            if(worked != resetIfWorked){
                reset = false;
                break;
            }
        }
        if(reset)
            vt_prop.route_point.time_stamp = -1;
    }

    return AroResp(0, "OK" );
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

}
