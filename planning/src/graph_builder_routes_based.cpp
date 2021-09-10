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
 
#include "arolib/planning/graph_builder_routes_based.hpp"

#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <limits>       // std::numeric_limits

#include <queue>

#define DEBUG_GRAPH

namespace arolib{
namespace DirectedGraph{


/** constructor
*/
GraphBuilder_RoutesBased::GraphBuilder_RoutesBased(const bool &bePreciseWithMaps, const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_bePreciseWithMaps(bePreciseWithMaps),
    m_currentGraphType(GRAPH_IF),
    m_calcGridValueOption(CALC_DIRECT)
{

}

Graph GraphBuilder_RoutesBased::buildGraph(const Subfield &subfield,
                                       const std::vector<Route> &harvInfieldRoutes,
                                       const std::vector<HeadlandRoute> &harvHeadlandRoutes,
                                       const std::vector<ResourcePoint> &resource_points,
                                       const std::vector<FieldAccessPoint> &field_access_points,
                                       const OutFieldInfo &outFieldInfo,
                                       const std::vector<Machine> &machines,
                                       const std::map<MachineId_t, MachineDynamicInfo> &machineInitialStates,
                                       double workingWidth,
                                       double workingWidthHL,
                                       bool includeVisitPeriods,
                                       const Polygon &field_bounday)
{

    Graph graph;

    getGraphData(graph).outFieldInfo = outFieldInfo;

    m_currentWorkedMass.clear();

    double baseTimestampInfield = -1;

    std::map<MachineId_t, Machine> machinesMap;//for easier access to the machines
    for(auto &m : machines)
        machinesMap[m.id] = m;

    if(!harvHeadlandRoutes.empty()){//expand the graph with the headland-harvesting routes and related points (headland sub-graph)
        m_currentGraphType = GRAPH_HL;
        expandGraph( graph,
                     subfield,
                     harvHeadlandRoutes,
                     resource_points,
                     field_access_points,
                     outFieldInfo,
                     workingWidthHL,
                     field_bounday);

        for(auto &route : harvHeadlandRoutes){
            if(!route.route_points.empty())//update the base timestamp for the infield subgraph
                baseTimestampInfield = std::max( baseTimestampInfield, route.route_points.back().time_stamp );
        }
    }

    if(!harvInfieldRoutes.empty()){//expand the graph with the inner-field-harvesting routes and related points (infield sub-graph)
        m_currentGraphType = GRAPH_IF;

        expandGraph( graph,
                     subfield,
                     harvInfieldRoutes,
                     resource_points,
                     field_access_points,
                     outFieldInfo,
                     workingWidth,
                     baseTimestampInfield,
                     field_bounday);

//        if(!harvHeadlandRoutes.empty()){
//            insertInterGraphConnection(graph,
//                                       harvHeadlandRoutes,
//                                       harvInfieldRoutes);
//        }
    }

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Interconnecting field access points...");
    interconnectFieldAccessPoints(graph, outFieldInfo);

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Inserting the initial-positions' vertices/edges...");
    insertInitialPositions(graph,
                           subfield,
                           machines,
                           machineInitialStates,
                           outFieldInfo,
                           field_bounday,
                           !harvHeadlandRoutes.empty(),
                           !harvInfieldRoutes.empty());

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Updating proximity map...");
    addVisitPeriods( graph,
                     machinesMap,
                     harvHeadlandRoutes,
                     harvInfieldRoutes );

    flushOutputBuffer();

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Graph expanded!" );

    return graph;

}

Graph GraphBuilder_RoutesBased::buildGraph(PlanningWorkspace &pw,
                                       size_t subfieldIdx,
                                       const std::vector<Route> &harvInfieldRoutes,
                                       const std::vector<HeadlandRoute> &harvHeadlandRoutes,
                                       const OutFieldInfo &outFieldInfo,
                                       double workingWidth,
                                       double workingWidthHL,
                                       bool includeVisitPeriods,
                                       const Polygon &field_bounday)
{
    m_calcGridValueOption = CALC_PW;

    if(subfieldIdx >= getField(pw).subfields.size())
        throw std::invalid_argument( "DirectedGraphBuilder::buildGraph: Invalid subfield index" );
    const auto &subfield = getField(pw).subfields.at(subfieldIdx);
    const auto &machines = getMachines(pw);
    const auto &resource_points = subfield.resource_points;
    const auto &field_access_points = subfield.access_points;
    const auto &machineInitialStates = getMachineCurrentStates(pw);

    m_planningWorkspace = &pw;

    auto graph = buildGraph(subfield,
                            harvInfieldRoutes,
                            harvHeadlandRoutes,
                            resource_points,
                            field_access_points,
                            outFieldInfo,
                            machines,
                            machineInitialStates,
                            workingWidth,
                            workingWidthHL,
                            includeVisitPeriods,
                            field_bounday);

    m_calcGridValueOption = CALC_DIRECT;
    m_planningWorkspace = nullptr;

    return graph;

}

void GraphBuilder_RoutesBased::setOutputFile(const std::string &filename)
{
    m_outputFile = filename;
    if(m_outputFile.empty())
        return;
    if( m_outputFile.size() <= 4 || m_outputFile.rfind(".txt") != m_outputFile.size() - 4 )
        m_outputFile += ".txt";
}

void GraphBuilder_RoutesBased::expandGraph(Graph &graph,
                                       const Subfield &subfield,
                                       const std::vector<Route> &routes,
                                       const std::vector<ResourcePoint> &resource_points,
                                       const std::vector<FieldAccessPoint> &field_access_points,
                                       const OutFieldInfo &outFieldInfo,
                                       double workingWidth,
                                       double baseTimestampInfield,
                                       const Polygon &field_bounday)
{


    clear();

    getGraphData(graph).workingWidth_IF = std::fabs(workingWidth);

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Inserting routes' nodes/edges..." );
    insertRoutes(graph, subfield, routes);

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Inserting headland points' nodes/edges..." );
    insertHeadlandPoints(graph, subfield, baseTimestampInfield);

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Inserting field access points' nodes/edges..." );
    insertConnectFieldAccessPoints(graph, field_access_points);

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Inserting resource points' nodes/edges..." );
    insertConnectResourcePoints(graph, resource_points, field_access_points, outFieldInfo, field_bounday);

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Inserting edges interconnecting tracks..." );
    interconnectTracks(graph, subfield);

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Graph expanded (infield)!" );

}

void GraphBuilder_RoutesBased::expandGraph(Graph &graph,
                                       const Subfield &subfield,
                                       const std::vector<HeadlandRoute> &routes,
                                       const std::vector<ResourcePoint> &resource_points,
                                       const std::vector<FieldAccessPoint> &field_access_points,
                                       const OutFieldInfo &outFieldInfo,
                                       double workingWidth,
                                       const Polygon &field_bounday)
{
    clear();

    getGraphData(graph).workingWidth_HL = std::fabs(workingWidth);

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Inserting routes' nodes/edges..." );
    insertRoutes(graph, routes);

    std::vector<Point> boundary_points;
    getHeadlandBoundaryPoints(graph, routes, subfield.boundary_outer, boundary_points);
    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Inserting headland boundary points' nodes/edges..." );
    insertHeadlandBoundaryPoints(graph, boundary_points);

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Inserting field access points' nodes/edges..." );
    insertConnectFieldAccessPoints(graph, field_access_points);

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Inserting resource points' nodes/edges..." );
    insertConnectResourcePoints(graph, resource_points, field_access_points, outFieldInfo, field_bounday);

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Inserting edges interconnecting tracks..." );
    interconnectTracksHL(graph);

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Graph expanded (headland)!" );

}

void GraphBuilder_RoutesBased::insertRoutes(Graph &graph,
                                        const Subfield &subfield,
                                        const std::vector<Route> &routes) {

    m_IFVerticesOutsideIF.clear();

    vertex_t predecessor_vertex;

    int minTrackID = std::numeric_limits<int>::max();
    int maxTrackID = std::numeric_limits<int>::lowest();
    for (auto &route : routes) {
        for (auto &rp : route.route_points) {
//                if(rp.track_id < 0)
//                    continue;
            if(rp.type == RoutePoint::RoutePointType::HEADLAND)
                continue;
            if(rp.track_id >= 0 && minTrackID > rp.track_id)
                minTrackID = rp.track_id;
            if(maxTrackID < rp.track_id)
                maxTrackID = rp.track_id;
        }
    }
    getGraphData(graph).maxTrackId_IF = maxTrackID;
    getGraphData(graph).minTrackId_IF = minTrackID;

    double workingWidth = std::max(1e-2, graph.workingWidth_HL() > 0 ? graph.workingWidth_HL() : graph.workingWidth_IF() );

    Polygon innerBoundaryEd;
    if( !arolib::geometry::offsetPolygon(subfield.boundary_inner, innerBoundaryEd, workingWidth, true) ){
        innerBoundaryEd = subfield.boundary_inner;
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error offsetting inner boundary");
    }

    for (unsigned int i = 0; i < routes.size(); i++) {

        const Route& route = routes.at(i);
        bool has_predecessor_on_track = false;
        bool errorTmp = true;

        double mass = 0;
        auto it_mass = m_currentWorkedMass.find(route.machine_id);
        if(it_mass != m_currentWorkedMass.end())
            mass = it_mass->second;

        for (int j = 0; j < route.route_points.size(); j++) {
            RoutePoint rp = route.route_points.at(j);

            vertex_property vertex_prop;
            vertex_prop.graph_location = DirectedGraph::vertex_property::INFIELD;

            double harvestedMassFromRoute = 0;
            if(j > 0)
                harvestedMassFromRoute = rp.harvested_mass - route.route_points.at(j-1).harvested_mass;

            if (rp.type == RoutePoint::HEADLAND) {//only insert points on track (i.e. harvesting points)
                has_predecessor_on_track = false;
                continue;
            }

            if(getGraphData(graph).routepoint_vertex_map.find(rp) != getGraphData(graph).routepoint_vertex_map.end() )
                m_logger.printOut(LogLevel::WARNING, "The route point is already in the routePoint_vertex map. The previous entry will be overwritten!");

            bool pointOutOfIF = !arolib::geometry::in_polygon(rp, innerBoundaryEd); //for the route points that go outside the inner boundary (i.e. the track was interrupted)

            vertex_prop.route_point = rp;
            vertex_prop.harvester_id = route.machine_id;

            vertex_t current_vertex;
            if (!has_predecessor_on_track || pointOutOfIF){//only add the vertex
                current_vertex = graph.addVertex(vertex_prop);
                getGraphData(graph).routepoint_vertex_map[vertex_prop.route_point] = current_vertex;
                if(pointOutOfIF)
                    m_IFVerticesOutsideIF.insert(current_vertex);
            }
            else
            { // add vertex and edge connecting to previous route point (in the same track)
                edge_property edge_prop;
                RoutePoint rp_predecesor = graph[predecessor_vertex].route_point;

                edge_prop.defWidth = getGraphData(graph).workingWidth_IF;
                edge_prop.distance = arolib::geometry::calc_dist(rp_predecesor, rp);

                edge_prop.p0 = rp_predecesor.point();
                edge_prop.p1 = rp.point();

                if( mass > 0.001 ||
                        (rp_predecesor.time_stamp > -0.0001 && rp.time_stamp > -0.0001) ){
                    mass += harvestedMassFromRoute;
                }

                current_vertex = graph.addVertex(vertex_prop);

                getGraphData(graph).routepoint_vertex_map[vertex_prop.route_point] = current_vertex;

                edge_prop.edge_type = EdgeType::DEFAULT;
                if( graph.addEdge(predecessor_vertex, current_vertex, edge_prop, true) )
                    appendToOutputBuffer("IR",
                                         graph[predecessor_vertex].route_point,
                                         graph[current_vertex].route_point,
                                         edge_prop.edge_type,
                                         predecessor_vertex,
                                         current_vertex,
                                         true);

            }

            if (rp.type == RoutePoint::TRACK_END || rp.type == RoutePoint::TRACK_START) {// store first and last point of track to simplify connecting headland points
                m_track_endpoints_map[rp.track_id][current_vertex] = rp.point();

                // TODO ADD POINT!!!
            }
            else if( rp.track_id == minTrackID || rp.track_id == maxTrackID )// store points of the first and last tracks (except TRACK_START and TRACK_END) to simplify connecting headland points
                m_verticesFirstLastTracks.push_back( std::make_pair(current_vertex, rp) );

            predecessor_vertex = current_vertex;
            has_predecessor_on_track = true;
        }

        m_currentWorkedMass[route.machine_id] = mass;

    }
}

void GraphBuilder_RoutesBased::insertRoutes(Graph &graph,
                                        const std::vector<HeadlandRoute> &routes)
{
    vertex_t predecessor_vertex;

    int minTrackID = std::numeric_limits<int>::max();
    int maxTrackID = std::numeric_limits<int>::lowest();
    for (auto &route : routes) {
        for (auto &rp : route.route_points) {
            if(rp.track_id >= 0 && minTrackID > rp.track_id)
                minTrackID = rp.track_id;
            if(maxTrackID < rp.track_id)
                maxTrackID = rp.track_id;
        }
    }
    getGraphData(graph).maxTrackId_HL = maxTrackID;
    getGraphData(graph).minTrackId_HL = minTrackID;

    for (unsigned int i = 0; i < routes.size(); i++) {

        HeadlandRoute route = routes.at(i);
        bool errorTmp = true;
        double mass = 0;
        auto it_mass = m_currentWorkedMass.find(route.machine_id);
        if(it_mass != m_currentWorkedMass.end())
            mass = it_mass->second;

        RoutePoint lastTrackStart;

        for (int j = 0; j < route.route_points.size(); j++) {
            RoutePoint rp = route.route_points.at(j);

            vertex_property vertex_prop;
            vertex_prop.graph_location = DirectedGraph::vertex_property::DEFAULT;//HEADLAND;

            if(!rp.isOfTypeWorking(true, true))
                continue;
            else if(rp.type == RoutePoint::TRACK_START)
                lastTrackStart = rp;

            double harvestedMassFromRoute = 0;
            if(j > 0)
                harvestedMassFromRoute = rp.harvested_mass - route.route_points.at(j-1).harvested_mass;

            if(getGraphData(graph).routepoint_vertex_map.find(rp) != getGraphData(graph).routepoint_vertex_map.end())
                m_logger.printOut(LogLevel::WARNING, "The route point is already in the routePoint_vertex map. The previous entry will be overwritten!");

            vertex_prop.route_point = rp;

            vertex_prop.harvester_id = route.machine_id;

            if(j==0){
                vertex_t current_vertex = graph.addVertex(vertex_prop);
                getGraphData(graph).routepoint_vertex_map[vertex_prop.route_point] = current_vertex;
                if( rp.track_id == minTrackID /*|| rp.track_id == maxTrackID*/ )
                    m_verticesFirstLastTracks.push_back( std::make_pair(current_vertex, rp) );
                predecessor_vertex = current_vertex;
                continue;
            }


            RoutePoint rp_predecesor = graph[predecessor_vertex].route_point;
            double edge_yield = 0;
            if( mass > 0.001 || (rp_predecesor.time_stamp > -0.0001 && rp.time_stamp > -0.0001) )
                edge_yield = harvestedMassFromRoute;

            mass += edge_yield;

            vertex_t current_vertex = graph.addVertex(vertex_prop);
            getGraphData(graph).routepoint_vertex_map[vertex_prop.route_point] = current_vertex;
            getGraphData(graph).HLProcessingVertices.emplace_back(current_vertex);
            if( rp.track_id == minTrackID /*|| rp.track_id == maxTrackID*/ )
                m_verticesFirstLastTracks.push_back( std::make_pair(current_vertex, rp) );

            if(rp.type != RoutePoint::TRACK_START){//do not connect with previous track_end (if there are more than 2 harvesters the connection is undesired)

                edge_property edge_prop;
                edge_prop.defWidth = getGraphData(graph).workingWidth_HL;
                edge_prop.distance = arolib::geometry::calc_dist(rp_predecesor, rp);
                edge_prop.p0 = rp_predecesor.point();
                edge_prop.p1 = rp.point();

                edge_prop.edge_type = EdgeType::DEFAULT;
                if( graph.addEdge(predecessor_vertex, current_vertex, edge_prop, true) )
                    appendToOutputBuffer("IR",
                                         graph[predecessor_vertex].route_point,
                                         graph[current_vertex].route_point,
                                         edge_prop.edge_type,
                                         predecessor_vertex,
                                         current_vertex,
                                         true);

            }


            //connect track start with track end with a virtual edge
            //it could be better to reuse the track start, but it might bring problems regarding harvested mass (among possibly others)
            if(rp.type == RoutePoint::TRACK_END && arolib::geometry::calc_dist(lastTrackStart, rp) < 1e-6){
                auto it0 = getGraphData(graph).routepoint_vertex_map.find(lastTrackStart);
                if(it0 != getGraphData(graph).routepoint_vertex_map.end()){
                    edge_property edge_prop_0;
                    edge_prop_0.defWidth = 0;
                    edge_prop_0.distance = 0;
                    edge_prop_0.edge_type = EdgeType::DEFAULT;

                    const RoutePoint& rp_0 = graph[current_vertex].route_point;
                    const RoutePoint& rp_1 = graph[it0->second].route_point;

                    edge_prop_0.p0 = rp_0.point();
                    edge_prop_0.p1 = rp_1.point();

                    if( graph.addEdge(current_vertex, it0->second, edge_prop_0, true) )
                        appendToOutputBuffer( ( m_currentGraphType == GRAPH_HL ? "IR" : "HL"),
                                              rp_0,
                                              rp_1,
                                              edge_prop_0.edge_type,
                                              current_vertex,
                                              it0->second,
                                              true);

                    //set the timestamp of the track_start vertex
                    const vertex_property &vt_prop_TS = graph[it0->second];
                    vertex_property &vt_prop_TE = graph[current_vertex];
                    vt_prop_TE.route_point.time_stamp = vt_prop_TS.route_point.time_stamp;
                }


            }

            predecessor_vertex = current_vertex;
        }

        m_currentWorkedMass[route.machine_id] = mass;

    }

}
void GraphBuilder_RoutesBased::interconnectTracks(Graph &graph, const Subfield &subfield) {

    std::map< int, std::vector< std::pair<vertex_t, vertex_property> > > tracks_vertices_map;

    struct connectionProp{
        vertex_t vertex;
        RoutePoint p0;
        RoutePoint p1;
    };
    std::map<vertex_t, connectionProp> connectedVertices_prev;
    std::map<vertex_t, RoutePoint> notConnectedVertices_next;

    std::map<vertex_t, std::pair< RoutePoint, std::set<int> > > HLVertices; //pair <route point , connected tracks>

    //fill the tracks-vertices map and the (infield sub-graph) headland-vertices map
    if(!graph.HLProcessingVertices().empty()){
        for(auto& it : graph.routepoint_vertex_map()){

            if(m_IFVerticesOutsideIF.find(it.second) != m_IFVerticesOutsideIF.end())//@todo check if not connecting these vertices causes problems in the path search
                continue;

            int track_id = it.first.track_id;
            if( track_id >= graph.minTrackId_IF() && track_id <= graph.maxTrackId_IF() ){
                const vertex_property& v_prop = graph[it.second];
                tracks_vertices_map[track_id].push_back( std::make_pair(it.second,v_prop) );
            }
        }
        for(auto& vt_hl : graph.HLProcessingVertices()){
            const vertex_property& v_prop = graph[vt_hl];
            const RoutePoint& rp = v_prop.route_point;

            if(rp.track_id != graph.maxTrackId_HL())
                continue; //only check the HL track that is closest to the infield

            std::pair< RoutePoint, std::set<int> > connProp;
            connProp.first = rp;
            auto edges = boost::out_edges(vt_hl, graph);
            for(;edges.first != edges.second; edges.first++){
                DirectedGraph::vertex_t successor = target(*edges.first, graph);
                DirectedGraph::vertex_property successor_prop = graph[successor];
                if(Track::isInfieldTrack(successor_prop.route_point.track_id))
                    connProp.second.insert(successor_prop.route_point.track_id);
            }
            HLVertices[vt_hl] = connProp;
        }
    }
    else{
        for(vertex_iter vp = vertices(graph); vp.first != vp.second; vp.first++){
            const vertex_property& v_prop = graph[*vp.first];
            const RoutePoint rp = v_prop.route_point;
            int track_id = rp.track_id;

            if( track_id >= graph.minTrackId_IF() && track_id <= graph.maxTrackId_IF()
                    && m_IFVerticesOutsideIF.find(*vp.first) == m_IFVerticesOutsideIF.end()//@todo check if not connecting these vertices causes problems in the path search
               ){
                tracks_vertices_map[track_id].push_back( std::make_pair(*vp.first,v_prop) );
            }

            if(rp.type == RoutePoint::HEADLAND && v_prop.graph_location == vertex_property::GraphLocation::INFIELD){
                std::pair< RoutePoint, std::set<int> > connProp;
                connProp.first = rp;
                auto edges = boost::out_edges(*vp.first, graph);
                for(;edges.first != edges.second; edges.first++){
                    DirectedGraph::vertex_t successor = target(*edges.first, graph);
                    DirectedGraph::vertex_property successor_prop = graph[successor];
                    if(Track::isInfieldTrack(successor_prop.route_point.track_id))
                        connProp.second.insert(successor_prop.route_point.track_id);
                }
                HLVertices[*vp.first] = connProp;
            }
        }
    }

    //connect (if possible/suitable) every IF harvesting-vertex (i.e. belonging to an infield track), except TRACK_START and TRACK_END vertices, to the closest vertex in the next and previous track
    int countSwipes = 0;
    while (countSwipes < 2){//swipe first from lower to highest track id, and then reverse for unconnected vertices
        int deltaIt = (countSwipes < 1 ? 1 : -1);
        int itRef = (countSwipes < 1 ? 0 : tracks_vertices_map.size()-1);
        for( ; itRef >= 0 && itRef < tracks_vertices_map.size() ; itRef += deltaIt ){
            auto m_it = std::next(tracks_vertices_map.begin(),itRef);

            if (m_it->first < 0)
                continue;

            auto next_it = tracks_vertices_map.find(m_it->first + deltaIt);
            if (next_it == tracks_vertices_map.end())
                continue;

            for (auto &v_info : m_it->second) {
                vertex_t vt = v_info.first;
                vertex_property v_prop = v_info.second;

                if (!v_prop.route_point.isOfTypeWorking(true, true))
                    continue;

                double min_dist = std::numeric_limits<double>::max();
                vertex_t min_vt;
                arolib::RoutePoint rp_next;

                //get the vertex in next track closest to the current vertex
                for (auto &v_info2 : next_it->second) {
                    vertex_t vt2 = v_info2.first;
                    vertex_property v_prop2 = v_info2.second;
                    arolib::RoutePoint rp = v_prop2.route_point;
                    double d = arolib::geometry::calc_dist(rp, v_prop.route_point);
                    if (d <= min_dist) {
                        min_dist = d;
                        min_vt = vt2;
                        rp_next = rp;
                    }

                }

                if(countSwipes > 0){//check if that same edge was alredy created
                    auto itConnPrev = connectedVertices_prev.find(vt);
                    if(itConnPrev != connectedVertices_prev.end()
                            && itConnPrev->second.vertex == min_vt)
                        continue;//already connected to that vertex in first (fwd) swipe
                }

//                if( ( rp_next.type == RoutePoint::TRACK_START || rp_next.type == RoutePoint::TRACK_END )
//                        && min_dist > 1.5 * graph.workingWidth_IF() ){//connect to TRACK_START and TRACK_END vertices from the next/previous track iif they are not too far away from the vertex. If they are too far, it might be better to connect it to the headland (later on)
                if(min_dist > 1.5 * graph.workingWidth_IF()){ //do not connect if they are far
                    if(countSwipes == 0)
                        notConnectedVertices_next[vt] = v_prop.route_point;
                    continue;
                }

                addEdge( graph,
                         min_dist,
                         EdgeType::CROSS,
                         vt,
                         min_vt,
                         graph[vt].route_point,
                         graph[min_vt].route_point,
                         getGraphData(graph).workingWidth_IF,
                         (countSwipes == 0 ? "CTf" : "CTb") );

                connectionProp connProp;
                if(countSwipes == 0){
                    connProp.vertex = vt;
                    connProp.p0 = rp_next;
                    connProp.p1 = v_prop.route_point;
                    connectedVertices_prev[min_vt] = connProp;
                }
                else{
                    connProp.vertex = min_vt;
                    connProp.p0 = v_prop.route_point;
                    connProp.p1 = rp_next;
                    connectedVertices_prev[vt] = connProp;
                }
            }

        }
        ++countSwipes;
    }

    //try to connect to headland the vertices which were not connected to the next track
    for(auto &v_it : notConnectedVertices_next){
        RoutePoint rp1 = v_it.second;
        RoutePoint rp2;
        RoutePoint rpInTrack;

        double min_dist;
        auto trackVt = tracks_vertices_map.at(rp1.track_id);
        if(!getClosestPointInTrack(rp1, trackVt, rpInTrack, min_dist))//get closest route point in track
            continue;

        //get closest (yet valid) headland vertex
        vertex_t min_vt;
        if(! getClosestValidHeadlandVertex(rp1,
                                           rpInTrack,
                                           rp1.track_id,
                                           HLVertices,
                                           subfield,
                                           getGraphData(graph).workingWidth_IF,
                                           60,//we want a point that is perpendicularish to the track
                                           min_vt,
                                           rp2,
                                           min_dist) ){
            if(! getClosestValidHeadlandVertex(rp1,
                                               rpInTrack,
                                               rp1.track_id,
                                               HLVertices,
                                               subfield,
                                               getGraphData(graph).workingWidth_IF,
                                               0,
                                               min_vt,
                                               rp2,
                                               min_dist) )
                continue;
        }

        //@TODO: check if the edge intersect any of the infield tracks?

        addEdge(graph,
                min_dist,
                EdgeType::DEFAULT,
                v_it.first,
                min_vt,
                rp1,
                rp2,
                getGraphData(graph).workingWidth_IF,
                "CT*");

    }


    //try to connect to headland the vertices which were not connected to the previous track
    for (auto m_it = tracks_vertices_map.begin(); m_it != tracks_vertices_map.end(); ++m_it){
        auto trackId = m_it->first;
        if(trackId <= graph.minTrackId_IF())
            continue;
        for (const auto &vt_info : m_it->second) {
            vertex_t vt = vt_info.first;
            vertex_property v_prop = vt_info.second;

            RoutePoint rp1 = v_prop.route_point;
            RoutePoint rp2;
            RoutePoint rpInTrack;

            if( rp1.type == RoutePoint::TRACK_START || rp1.type == RoutePoint::TRACK_END )
                continue;
            if( connectedVertices_prev.find(vt) != connectedVertices_prev.end() )
                continue;

            double min_dist;
            auto trackVt = tracks_vertices_map.at(rp1.track_id);
            if(!getClosestPointInTrack(rp1, trackVt, rpInTrack, min_dist))//get closest route point in track
                continue;

            //get closest (yet valid) headland vertex
            vertex_t min_vt;

            if(! getClosestValidHeadlandVertex(rp1,
                                               rpInTrack,
                                               rp1.track_id,
                                               HLVertices,
                                               subfield,
                                               getGraphData(graph).workingWidth_IF,
                                               60,//we want a point that is perpendicularish to the track
                                               min_vt,
                                               rp2,
                                               min_dist) ){
                if(! getClosestValidHeadlandVertex(rp1,
                                                   rpInTrack,
                                                   rp1.track_id,
                                                   HLVertices,
                                                   subfield,
                                                   getGraphData(graph).workingWidth_IF,
                                                   0,
                                                   min_vt,
                                                   rp2,
                                                   min_dist) )
                    continue;
            }

            //@TODO: check if the edge intersect any of the infield tracks?

            addEdge(graph,
                    min_dist,
                    EdgeType::DEFAULT,
                    vt,
                    min_vt,
                    rp1,
                    rp2,
                    getGraphData(graph).workingWidth_IF,
                    "CT*");
        }

    }

}

void GraphBuilder_RoutesBased::interconnectTracksHL(Graph &graph)
{

    std::map<int, std::vector<vertex_t>> tracks_vertices_map;

    for(vertex_iter vp = vertices(graph); vp.first != vp.second; vp.first++){
//        RoutePoint rp = graph[*vp.first].route_point;
        int track_id = graph[*vp.first].route_point.track_id;

        if( track_id >= graph.minTrackId_HL() && track_id <= graph.maxTrackId_HL() )
            tracks_vertices_map[track_id].push_back(*vp.first);
    }

    struct connectionProp{
        vertex_t vertex;
        RoutePoint p0;
        RoutePoint p1;
    };
    std::map<vertex_t, connectionProp> connectedVertices_prev;
    int countSwipes = -1;

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Tracks: ");
    while (++countSwipes < 2){//swipe first from lower to highest track id, and then reverse for unconnected vertices
        int deltaIt = (countSwipes < 1 ? 1 : -1);
        int itRef = (countSwipes < 1 ? 0 : tracks_vertices_map.size()-1);
        for( ; itRef >= 0 && itRef < tracks_vertices_map.size() ; itRef += deltaIt ){

            auto m_it = std::next(tracks_vertices_map.begin(),itRef);
            if (m_it->first < 0)
                continue;

            if(countSwipes == 0)
                m_logger.printOut(LogLevel::INFO, __FUNCTION__, "\t" + std::to_string( m_it->first ) + " size: " + std::to_string( m_it->second.size() ) );

            std::map<int, std::vector<vertex_t>>::iterator next_it = tracks_vertices_map.find(m_it->first + deltaIt);
            if (next_it != tracks_vertices_map.end()) {
                for (std::vector<vertex_t>::iterator v_it = m_it->second.begin(); v_it != m_it->second.end(); ++v_it) {

                    vertex_property v_prop = graph[*v_it];
                    if (true/*v_prop.route_point.type != arolib::RoutePoint::RoutePointType::HEADLAND*/) {

                        double min_dist = std::numeric_limits<double>::max();
                        vertex_t min_vt;

                        for (std::vector<vertex_t>::iterator nt_it = next_it->second.begin(); nt_it != next_it->second.end(); ++nt_it) {
                            arolib::RoutePoint rp = graph[*nt_it].route_point;
                            double d = arolib::geometry::calc_dist(rp, v_prop.route_point);
                            if (d <= min_dist) {
                                min_dist = d;
                                min_vt = *nt_it;
                            }
                        }

                        if(countSwipes > 0){//check if that same edge was alredy created
                            auto itConnPrev = connectedVertices_prev.find(*v_it);
                            if(itConnPrev != connectedVertices_prev.end()
                                    && itConnPrev->second.vertex == min_vt)
                                continue;//already connected to that vertex in first (fwd) swipe
                        }

                        /// create edge
                        edge_property edge_prop;
                        edge_prop.defWidth = getGraphData(graph).workingWidth_HL;
                        edge_prop.distance = min_dist;
                        edge_prop.edge_type = EdgeType::CROSS;// = EdgeType::DEFAULT;

                        RoutePoint rp1 = graph[*v_it].route_point;
                        RoutePoint rp2 = graph[min_vt].route_point;

                        edge_prop.p0 = rp1.point();
                        edge_prop.p1 = rp2.point();

                        if( graph.addEdge(*v_it, min_vt, edge_prop, true) ){
                            appendToOutputBuffer("CT",
                                                 rp1,
                                                 rp2,
                                                 edge_prop.edge_type,
                                                 *v_it,
                                                 min_vt,
                                                 true);

                            if(countSwipes == 0){
                                connectionProp connProp;
                                connProp.vertex = *v_it;
                                connProp.p0 = rp2;
                                connProp.p1 = rp1;
                                connectedVertices_prev[min_vt] = connProp;
                            }
                        }

                    }
                }
            }

        }
    }

//    for (std::map<int, std::vector<vertex_t>>::iterator m_it = tracks_vertices_map.begin(); m_it != tracks_vertices_map.end(); ++m_it) {

//        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "\t" + std::to_string( m_it->first ) + " size: " + std::to_string( m_it->second.size() ) );
//        if (m_it->first > -1) {
//            std::map<int, std::vector<vertex_t>>::iterator next_it = tracks_vertices_map.find(m_it->first + 1);
//            if (next_it != tracks_vertices_map.end()) {
//                for (std::vector<vertex_t>::iterator v_it = m_it->second.begin(); v_it != m_it->second.end(); ++v_it) {
//                    vertex_property v_prop = graph[*v_it];
//                    if (true/*v_prop.route_point.type != arolib::RoutePoint::RoutePointType::HEADLAND*/) {

//                        double min_dist = std::numeric_limits<double>::max();
//                        vertex_t min_vt;

//                        for (std::vector<vertex_t>::iterator nt_it = next_it->second.begin(); nt_it != next_it->second.end(); ++nt_it) {
//                            arolib::RoutePoint rp = graph[*nt_it].route_point;
//                            double d = dist(rp.point, v_prop.route_point.point);
//                            if (d <= min_dist) {
//                                min_dist = d;
//                                min_vt = *nt_it;
//                            }
//                        }

//                        /// create edge
//                        edge_property edge_prop;
//                        edge_prop.defWidth = getGraphData(graph).workingWidth_HL;
//                        edge_prop.distance = min_dist;
//                        edge_prop.yield = 0;
//                        edge_prop.edge_type = EdgeType::CROSS;// = EdgeType::DEFAULT;

//                        edge_prop.p0 = rp1.point();
//                        edge_prop.p1 = rp2.point();

//                        RoutePoint rp1 = graph[*v_it].route_point;
//                        RoutePoint rp2 = graph[min_vt].route_point;


//                        if (m_soilmap.isAllocated()){
//                            bool errorTmp;
//                            edge_prop.soilValue = getGridValue(PlanningWorkspace::GridType::SOIL,
//                                                               rp1.point,
//                                                               rp2.point,
//                                                               getGraphData(graph).workingWidth_HL,
//                                                               errorTmp);

//                            if (errorTmp)
//                                edge_prop.soilValue = 0;
//                        }

//                        if( graph.addEdge(*v_it, min_vt, edge_prop, true) )
//                            appendToOutputBuffer("CT",
//                                                 rp1,
//                                                 rp2,
//                                                 edge_prop.edge_type,
//                                                 *v_it,
//                                                 min_vt);

//                    }
//                }
//            }
//        }
//    }

}

void GraphBuilder_RoutesBased::insertConnectFieldAccessPoints(Graph &graph,
                                                          const std::vector<FieldAccessPoint> &field_access_points)
{
    for(auto &fap : field_access_points)
        insertConnectAccessPointOrInfieldResourcePoint(graph, fap, false, OutFieldInfo());

}

void GraphBuilder_RoutesBased::interconnectFieldAccessPoints(Graph &graph, const OutFieldInfo &outFieldInfo)
{
    if( outFieldInfo.mapAccessPoint2AccessPoint().empty() )
        return;

    for(const auto &fap1_it : graph.accesspoint_vertex_map()){
        for(const auto &fap2_it : graph.accesspoint_vertex_map()){
            if(fap1_it == fap2_it)
                continue;
            const FieldAccessPoint& fap1 = fap1_it.first;
            const FieldAccessPoint& fap2 = fap2_it.first;

            edge_property edge_prop;
            edge_prop.defWidth = 0;
            edge_prop.distance = -1;//must be calculated during planning depending on the machine and the travel costs
            edge_prop.bidirectional = false;
            edge_prop.edge_type = EdgeType::FAP_TO_FAP;

            if ( outFieldInfo.size_FAP2FAP(fap1.id, fap2.id) > 0 ){
                if( outFieldInfo.getTravelCost_FAP2FAP(fap1.id, fap2.id, edge_prop.travelCosts) ){
                    if( graph.addEdge(fap1_it.second, fap2_it.second, edge_prop, false) )
                        appendToOutputBuffer("AA",
                                             graph[fap1_it.second].route_point,
                                             graph[fap2_it.second].route_point,
                                             edge_prop.edge_type,
                                             fap1_it.second,
                                             fap2_it.second,
                                             false);
                }
            }
        }
    }
}


vertex_t GraphBuilder_RoutesBased::addHeadlandPoint(Graph &graph, const Point &hp, const double &timestamp) {
    RoutePoint rp;
    rp.type = RoutePoint::RoutePointType::HEADLAND;
    rp.point() = hp;
    rp.track_id = -1;
    rp.track_idx = -1;
    rp.time_stamp = timestamp;
    rp.bunker_mass = -1;
    vertex_property vertex_prop;
    vertex_prop.route_point = rp;
    vertex_prop.graph_location = (m_currentGraphType == GRAPH_IF?
                                      DirectedGraph::vertex_property::INFIELD:
                                      DirectedGraph::vertex_property::HEADLAND);
    return  graph.addVertex(vertex_prop);;
}


void GraphBuilder_RoutesBased::insertHeadlandEdges(Graph &graph,
                                               const vertex_t &u,
                                               const vertex_t &v,
                                               EdgeType edgeType,
                                               bool bidirectional) {
    edge_property edge_prop;
    edge_prop.distance = arolib::geometry::calc_dist(graph[u].route_point, graph[v].route_point);
    edge_prop.edge_type = edgeType;

    RoutePoint rp1 = graph[u].route_point;
    RoutePoint rp2 = graph[v].route_point;

    edge_prop.p0 = rp1.point();
    edge_prop.p1 = rp2.point();

    double workingWidth = (m_currentGraphType == GRAPH_IF? getGraphData(graph).workingWidth_IF : getGraphData(graph).workingWidth_HL);

    edge_prop.defWidth = workingWidth;

    if( graph.addEdge(u, v, edge_prop, bidirectional) ){

        if(bidirectional)
            appendToOutputBuffer("HL",
                                 rp1,
                                 rp2,
                                 edge_prop.edge_type,
                                 u,
                                 v,
                                 true);
        else
            appendToOutputBuffer("HL-",
                                 rp1,
                                 rp2,
                                 edge_prop.edge_type,
                                 u,
                                 v,
                                 false);

    }
}

bool GraphBuilder_RoutesBased::addEdge(DirectedGraph::Graph &graph,
                                   double distance,
                                   EdgeType edgeType,
                                   const vertex_t& vt1,
                                   const vertex_t& vt2,
                                   const RoutePoint& rp1,
                                   const RoutePoint& rp2,
                                   double width,
                                   const std::string& addedTag,
                                   bool bidirectional, bool onlyIfNotExisting, bool overwrite)
{
    /// create edge
    edge_property edge_prop;
    edge_prop.defWidth = width;
    edge_prop.distance = distance;
    edge_prop.edge_type = edgeType;

    edge_prop.p0 = rp1.point();
    edge_prop.p1 = rp2.point();

    bool edgeAdded = graph.addEdge(vt1, vt2, edge_prop, bidirectional, onlyIfNotExisting, overwrite);
    if( edgeAdded )
        appendToOutputBuffer(addedTag,
                             rp1,
                             rp2,
                             edge_prop.edge_type,
                             vt1,
                             vt2,
                             bidirectional);

    return edgeAdded;
}

void GraphBuilder_RoutesBased::insertConnectResourcePoints(Graph &graph,
                                                       const std::vector<arolib::ResourcePoint> &resource_points,
                                                       const std::vector<FieldAccessPoint>& field_access_points,
                                                       const OutFieldInfo &outFieldInfo,
                                                       const Polygon &field_bounday) {


    if (resource_points.empty())
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "No resource points in subfield!" );

    std::set<size_t> infieldResourcePointsIndexes;

    bool treatAllAsInfield = false;
    bool outfieldInfoExists = outFieldInfo.size_FAP2RP() > 0 && outFieldInfo.size_RP2FAP() > 0;

    if(m_currentGraphType == GRAPH_HL){
        if(field_access_points.empty()/* || !outfieldInfoExists*/){
            m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Treating all resource points as infield resource points (no field-access points or travel-costs-map missing)" );
            treatAllAsInfield = true;
        }
    }
    else{
        if(field_access_points.empty()){
            m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Treating all resource points as infield resource points (no field-access points)" );
            treatAllAsInfield = true;
        }

    }

    //add infield resource points
    for(size_t i = 0 ; i < resource_points.size() ; ++i) {
        if( treatAllAsInfield //the resource point should be treated as infield because no access-points or travel costs were given
                || arolib::geometry::in_polygon(resource_points.at(i), field_bounday) //check if the resource point is located inside the field
                //|| !outFieldInfo.resourcePointBidirectionalConnectionExists( resource_points.at(i).id, field_access_points)//the resource point should be treated as infield because no travel costs were given to any access point
          ){
            infieldResourcePointsIndexes.insert(i);
            insertConnectAccessPointOrInfieldResourcePoint(graph, resource_points.at(i), true, outFieldInfo);
        }
    }


    if(!treatAllAsInfield){

        //add out-of-field resource points and connect them with the field access points
        std::vector<ResourcePoint> remaining_resource_points, remaining_resource_points_noOFI;
        for(size_t i = 0 ; i < resource_points.size() ; ++i) {
            if( infieldResourcePointsIndexes.find(i) != infieldResourcePointsIndexes.end() )
                continue;//already added as an infield resource point

            if(!outfieldInfoExists || !outFieldInfo.resourcePointBidirectionalConnectionExists( resource_points.at(i).id, field_access_points))
                remaining_resource_points_noOFI.push_back( resource_points.at(i) );
            else
                remaining_resource_points.push_back( resource_points.at(i) );
        }
        insertConnectOutfieldResourcePoints(graph, remaining_resource_points, field_access_points, outFieldInfo);
        insertConnectOutfieldResourcePoints(graph, remaining_resource_points_noOFI, outFieldInfo.mapUnloadingCosts());
    }

}

void GraphBuilder_RoutesBased::insertConnectAccessPointOrInfieldResourcePoint(Graph &graph,
                                                                          const Point &point,
                                                                          const bool & isResourcePoint,
                                                                          const OutFieldInfo &outFieldInfo)
{


    bool vertexExists = false;
    ResourcePoint resourcePoint;
    FieldAccessPoint fieldAccessPoint;

    RoutePoint rp;
    vertex_property vertex_prop;
    vertex_t point_vt;

    rp.point() = point;
    rp.track_id = -1;
    rp.track_idx = -1;
    rp.time_stamp = -1;
    rp.bunker_mass = -1;

    if(isResourcePoint){
        resourcePoint = dynamic_cast<const ResourcePoint&>(point);

        //check if a vertex for this point already exists in the graph
        auto it = getGraphData(graph).resourcepoint_vertex_map.find(resourcePoint);
        if(it != getGraphData(graph).resourcepoint_vertex_map.end()){
            vertexExists = true;
            point_vt = it->second;
        }
        else{
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Adding new infield resource point: {" + std::to_string( resourcePoint.id ) + "} " + point.toString(10) );
            rp.type = RoutePoint::RoutePointType::RESOURCE_POINT;
            outFieldInfo.getUnloadingCosts( resourcePoint.id,
                                            vertex_prop.unloadingCosts );
            if( vertex_prop.unloadingCosts.empty() ){
                m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Using default unloading costs in resource point with ID " + std::to_string( resourcePoint.id ) + " for all machines (cost map is empty)" );
                vertex_prop.unloadingCosts[OutFieldInfo::AllMachines] = OutFieldInfo::UnloadingCosts(resourcePoint.defaultUnloadingTime,
                                                                                                     resourcePoint.defaultUnloadingTimePerKg);
            }
        }
    }
    else{
        fieldAccessPoint = dynamic_cast<const FieldAccessPoint&>(point);

        //check if a vertex for this point already exists in the graph
        auto it = getGraphData(graph).accesspoint_vertex_map.find(fieldAccessPoint);
        if(it != getGraphData(graph).accesspoint_vertex_map.end()){
            vertexExists = true;
            point_vt = it->second;
        }
        else{
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Adding new field access point: {" + std::to_string( fieldAccessPoint.id ) + "} " + point.toString(10) );
            if(fieldAccessPoint.accessType == FieldAccessPoint::AP_EXIT_ONLY)
                rp.type = RoutePoint::RoutePointType::FIELD_EXIT;
            else
                rp.type = RoutePoint::RoutePointType::FIELD_ENTRY;
        }
    }

    vertex_prop.route_point = rp;

    // search nearest headland point
    double min_dist = std::numeric_limits<double>::max();
    vertex_t min_vt;

    // iif graph_location == HEADLAND, search nearest (harvesting) route point
    double min_dist_2 = std::numeric_limits<double>::max();
    vertex_t min_vt_2;

    for(vertex_iter vp = vertices(graph); vp.first != vp.second; vp.first++){
        vertex_property v_prop = graph[*vp.first];

        if( (m_currentGraphType == GRAPH_IF && v_prop.graph_location == DirectedGraph::vertex_property::HEADLAND)
                ||(m_currentGraphType == GRAPH_HL && v_prop.graph_location == DirectedGraph::vertex_property::INFIELD))
            continue;//the vertex belongs to the other sub-graph

        RoutePoint tmp_rp = v_prop.route_point;
        if(m_currentGraphType == GRAPH_IF
                && !graph.HLProcessingVertices().empty()
                && tmp_rp.track_id == graph.minTrackId_HL()){// search nearest point from outter headland track
            double dist = arolib::geometry::calc_dist(point, tmp_rp);
            if (dist < min_dist) {
                min_dist = dist;
                min_vt = *vp.first;
            }
        }
        else if (tmp_rp.type == RoutePoint::HEADLAND) {// search nearest headland point
            double dist = arolib::geometry::calc_dist(point, tmp_rp);
            if (dist < min_dist) {
                min_dist = dist;
                min_vt = *vp.first;
            }
        }
        else if(m_currentGraphType == GRAPH_HL &&
                ( tmp_rp.isOfTypeWorking(true) ) ){// iif graph_location == HEADLAND, search nearest (working) route point

            double dist = arolib::geometry::calc_dist(point, tmp_rp);
            if (dist < min_dist_2) {
                min_dist_2 = dist;
                min_vt_2 = *vp.first;
            }
        }
    }

    int ref = 0;
    while(ref < 1 + (int)( !isResourcePoint && m_currentGraphType == GRAPH_HL ) ){
        double min_dist_ref = min_dist;
        vertex_t min_vt_ref = min_vt;

        if(ref > 0){
            min_dist_ref = min_dist_2;
            min_vt_ref = min_vt_2;
        }
        if (min_dist_ref < std::numeric_limits<double>::max() - 1) { /// found point -> insert vertex (if it doesn't exist already) and edge
            if(!vertexExists)
                point_vt = graph.addVertex(vertex_prop);

            edge_property edge_prop;
            edge_prop.distance = min_dist_ref;
            edge_prop.edge_type = EdgeType::DEFAULT;

            RoutePoint rp1 = graph[point_vt].route_point;
            RoutePoint rp2 = graph[min_vt_ref].route_point;

            edge_prop.p0 = rp1.point();
            edge_prop.p1 = rp2.point();

            double workingWidth = (m_currentGraphType == GRAPH_IF? getGraphData(graph).workingWidth_IF : getGraphData(graph).workingWidth_HL);

            edge_prop.defWidth = workingWidth;

            if(isResourcePoint){
                if( graph.addEdge(point_vt, min_vt_ref, edge_prop, true) )
                    appendToOutputBuffer("RP",
                                         rp1,
                                         rp2,
                                         edge_prop.edge_type,
                                         point_vt,
                                         min_vt_ref,
                                         true);
                if(!vertexExists)
                    getGraphData(graph).resourcepoint_vertex_map[resourcePoint] = point_vt;
            }
            else{
                if( fieldAccessPoint.accessType == FieldAccessPoint::AP_ENTRY_EXIT
                        || fieldAccessPoint.accessType == FieldAccessPoint::AP_ENTRY_ONLY ){
                    if( graph.addEdge(point_vt, min_vt_ref, edge_prop, false) )
                        appendToOutputBuffer("AI",
                                             rp1,
                                             rp2,
                                             edge_prop.edge_type,
                                             point_vt,
                                             min_vt_ref,
                                             false);
                }

                if( fieldAccessPoint.accessType == FieldAccessPoint::AP_ENTRY_EXIT
                        || fieldAccessPoint.accessType == FieldAccessPoint::AP_EXIT_ONLY ){
                    if( graph.addEdge(min_vt_ref, point_vt, edge_prop, false) )
                        appendToOutputBuffer("AO",
                                             rp2,
                                             rp1,
                                             edge_prop.edge_type,
                                             min_vt_ref,
                                             point_vt,
                                             false);
                }

                if(!vertexExists)
                    getGraphData(graph).accesspoint_vertex_map[fieldAccessPoint] = point_vt;

                vertexExists = true;
            }
        }
        else if(ref == 0){
            m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "No headland points in graph. Could not add"
                              + std::string(isResourcePoint?"resource":"access") + " point and/or connect it to the "
                              + std::string(m_currentGraphType == GRAPH_IF?"headland":"field boundary") + "!" );
        }
        else
            m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "No headland route points in graph. Could not add access point and/or connect it to the closest headland track!" );
        ++ref;
    }
}

void GraphBuilder_RoutesBased::insertConnectOutfieldResourcePoints(Graph &graph, const std::vector<ResourcePoint> &resource_points,
                                                               const std::vector<FieldAccessPoint> &field_access_points,
                                                               const OutFieldInfo &outFieldInfo)
{


    RoutePoint rp_rp;
    rp_rp.type = RoutePoint::RoutePointType::RESOURCE_POINT;
    rp_rp.track_id = -1;
    rp_rp.track_idx = -1;
    rp_rp.time_stamp = -1;
    rp_rp.bunker_mass = -1;

    for(auto &rp : resource_points){
        bool vertexExists = false;

        rp_rp.point() = rp.point();

        for(size_t i = 0 ; i < field_access_points.size() ; ++i){
            const FieldAccessPoint &fap = field_access_points.at(i);
            vertex_t fap_vt;
            auto fap_it = getGraphData(graph).accesspoint_vertex_map.find(fap);
            if(fap_it == getGraphData(graph).accesspoint_vertex_map.end())
                continue;
            fap_vt = fap_it->second;

            vertex_t point_vt;

            auto it_rpvt = getGraphData(graph).resourcepoint_vertex_map.find(rp);
            if(it_rpvt != getGraphData(graph).resourcepoint_vertex_map.end()){
                vertexExists = true;
                point_vt = it_rpvt->second;
            }
            else{
                vertex_property vertex_prop;
                vertex_prop.route_point = rp_rp;
                outFieldInfo.getUnloadingCosts( rp.id,
                                                 vertex_prop.unloadingCosts );
                if( vertex_prop.unloadingCosts.empty() ){
                    m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Using default unloading costs in resource point with ID " + std::to_string( rp.id ) + " for all machines (cost map is empty)" );
                    vertex_prop.unloadingCosts[OutFieldInfo::AllMachines] = OutFieldInfo::UnloadingCosts(rp.defaultUnloadingTime,
                                                                                                           rp.defaultUnloadingTimePerKg);
                }

                point_vt = graph.addVertex(vertex_prop);
                getGraphData(graph).resourcepoint_vertex_map[rp] = point_vt;

            }

            edge_property edge_prop;
            edge_prop.defWidth = 0;
            edge_prop.distance = -1;//must be calculated during planning depending on the machine

            bool addedToOutput = false;
            if ( outFieldInfo.size_FAP2RP(fap.id, rp.id) > 0 ){
                if( outFieldInfo.getTravelCost_FAP2RP(fap.id, rp.id, edge_prop.travelCosts) ){
                    bool edgeExists = false;
                    auto out_edges= boost::out_edges(fap_vt, graph);
                    for(;out_edges.first != out_edges.second; out_edges.first++){
                        if( target(*out_edges.first, graph) == point_vt ){
                            edgeExists = true;
                            break;
                        }
                    }

                    if(!edgeExists){
                        edge_prop.edge_type = EdgeType::FAP_TO_RP;

                        const RoutePoint& rp1 = graph[point_vt].route_point;
                        const RoutePoint& rp2 = graph[fap_vt].route_point;

                        edge_prop.p0 = rp1.point();
                        edge_prop.p1 = rp2.point();

                        addedToOutput = true;
                        if( graph.addEdge(fap_vt, point_vt, edge_prop, false) )
                            appendToOutputBuffer("AR",
                                                 rp1,
                                                 rp2,
                                                 edge_prop.edge_type,
                                                 fap_vt,
                                                 point_vt,
                                                 false);
                    }
                }
            }
            if ( outFieldInfo.size_RP2FAP(rp.id, fap.id) > 0 ){
                if( outFieldInfo.getTravelCost_RP2FAP(rp.id, fap.id, edge_prop.travelCosts) ){

                    bool edgeExists = false;
                    auto out_edges= boost::out_edges(point_vt, graph);
                    for(;out_edges.first != out_edges.second; out_edges.first++){
                        if( target(*out_edges.first, graph) == fap_vt ){
                            edgeExists = true;
                            break;
                        }
                    }

                    if(!edgeExists){
                        edge_prop.edge_type = EdgeType::RP_TO_FAP;
                        if( graph.addEdge(point_vt, fap_vt, edge_prop, false) )
                            appendToOutputBuffer("RA",
                                                 graph[point_vt].route_point,
                                                 graph[fap_vt].route_point,
                                                 edge_prop.edge_type,
                                                 point_vt,
                                                 fap_vt,
                                                 false);
                    }
                }
            }

        }
    }

}

void GraphBuilder_RoutesBased::insertConnectOutfieldResourcePoints(Graph &graph,
                                                               const std::vector<ResourcePoint> &resource_points,
                                                               const OutFieldInfo::MapUnloadingCosts_t &unloadingCostsMap)
{

    for(auto& resourcePoint : resource_points){//connect each of the resource points to the nearest access point vertex
        bool found = false;
        double min_dist = std::numeric_limits<double>::max();
        vertex_t min_vt;

        for(auto& fap_it : graph.accesspoint_vertex_map()){
            const auto &fap = fap_it.first;
            double dist = arolib::geometry::calc_dist(resourcePoint, fap);
            if(min_dist > dist){
                min_dist = dist;
                min_vt = fap_it.second;
                found = true;
            }
        }

        if(!found){
            m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "No connection to the field was found for resource point with ID " + std::to_string( resourcePoint.id ) + " for all machines (cost map is empty)" );
            continue;
        }


        bool vertexExists = false;

        RoutePoint rp;
        vertex_property vertex_prop;
        vertex_t point_vt;

        rp.point() = resourcePoint.point();
        rp.track_id = -1;
        rp.track_idx = -1;
        rp.time_stamp = -1;
        rp.bunker_mass = -1;

        //check if a vertex for this point already exists in the graph
        auto it = getGraphData(graph).resourcepoint_vertex_map.find(resourcePoint);
        if(it != getGraphData(graph).resourcepoint_vertex_map.end()){
            vertexExists = true;
            point_vt = it->second;
        }
        else{
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Adding new outfield resource point: {" + std::to_string( resourcePoint.id ) + "} "
                              + resourcePoint.toString() );
            rp.type = RoutePoint::RoutePointType::RESOURCE_POINT;
            OutFieldInfo::getUnloadingCosts( unloadingCostsMap,
                                             resourcePoint.id,
                                             vertex_prop.unloadingCosts );
            if( vertex_prop.unloadingCosts.empty() ){
                m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Using default unloading costs in resource point with ID " + std::to_string( resourcePoint.id ) + " for all machines (cost map is empty)" );
                vertex_prop.unloadingCosts[OutFieldInfo::AllMachines] = OutFieldInfo::UnloadingCosts(resourcePoint.defaultUnloadingTime,
                                                                                                     resourcePoint.defaultUnloadingTimePerKg);
            }
        }

        vertex_prop.route_point = rp;


        if(!vertexExists)
            point_vt = graph.addVertex(vertex_prop);

        edge_property edge_prop;
        edge_prop.defWidth = 0;
        edge_prop.distance = min_dist;
        edge_prop.edge_type = EdgeType::DEFAULT;

        RoutePoint rp1 = graph[point_vt].route_point;
        RoutePoint rp2 = graph[min_vt].route_point;

        edge_prop.p0 = rp1.point();
        edge_prop.p1 = rp2.point();

        if( graph.addEdge(point_vt, min_vt, edge_prop, true) )
            appendToOutputBuffer("RP",
                                 rp1,
                                 rp2,
                                 edge_prop.edge_type,
                                 point_vt,
                                 min_vt,
                                 true);
        if(!vertexExists)
            getGraphData(graph).resourcepoint_vertex_map[resourcePoint] = point_vt;

    }


}

void GraphBuilder_RoutesBased::insertHeadlandPoints(Graph &graph,
                                                const Subfield &subfield,
                                                const double &baseTimestampInfield)
{
    bool HLPointsAdded = false;
    if(!graph.HLProcessingVertices().empty()){//use the vertices of the HL routes
        connectHLandIFTracks(graph);
        HLPointsAdded = true;
    }
    else if(!subfield.headlands.complete.tracks.empty()){//create vertices and edges using the tracks
        double res = graph.workingWidth_IF();
        if(graph.workingWidth_HL() <= 1e-5)
            getGraphData(graph).workingWidth_HL = graph.workingWidth_IF();
        if(res < 0)
            res = 10;
        auto tracksHL = subfield.headlands.complete.tracks;
        if( sampleHLTracksPerpendicularly(subfield, tracksHL, graph.workingWidth_IF()) ){
            HLPointsAdded = true;
            connectHLTracks(graph, subfield, tracksHL);
            connectHLandIFTracks(graph);
        }
    }

    if(!HLPointsAdded) //use the (central) headland track
        HLPointsAdded = insertHeadlandMiddleTrack(graph, subfield.headlands.complete.middle_track, baseTimestampInfield);

}

bool GraphBuilder_RoutesBased::insertHeadlandMiddleTrack(Graph & graph,
                                                     Polygon track,
                                                     const double &baseTimestampInfield) {

    arolib::geometry::unsample_polygon(track);
    arolib::geometry::correct_polygon(track);
    if( arolib::geometry::isPolygonValid(track) == arolib::geometry::PolygonValidity::INVALID_POLYGON )
        return false;

    std::unordered_map<Point, std::set<vertex_t>, Point::KeyHash> headlandPoints2TrackPoints;

    //add points to the HL track closest to IF track start/ends
    for(auto& it_track : m_track_endpoints_map){
        for(auto& it_vt : it_track.second){
            auto ind = arolib::geometry::addSampleToGeometryClosestToPoint(track.points, it_vt.second, 1);
            if(ind < 0 || ind >= track.points.size())
                continue;
            headlandPoints2TrackPoints[track.points.at(ind)].insert(it_vt.first);
        }
    }

    //add points to the HL track closest to IF track start/ends
    for(auto& vt_point : m_verticesFirstLastTracks){
        auto ind = arolib::geometry::addSampleToGeometryClosestToPoint(track.points, vt_point.second, 1);
        if(ind < 0 || ind >= track.points.size())
            continue;
        headlandPoints2TrackPoints[track.points.at(ind)].insert(vt_point.first);
    }

    double res = graph.workingWidth_IF();
    if(res < 0)
        res = 10;
    track.points = arolib::geometry::sample_geometry(track.points, res);

    vertex_t predecessor_vertex;
    vertex_t first_vertex;
    bool isClosed = (track.points.front() == track.points.back());

    //add and connect HL vertices
    for (unsigned int i = 0; i < track.points.size(); ++i) {

        Point hp = track.points.at(i);
        /// insert point

        vertex_t current_vertex;
        if(i+1 == track.points.size() && isClosed)
            current_vertex = first_vertex;
        else{
            current_vertex = addHeadlandPoint(graph, hp, baseTimestampInfield);
            getGraphData(graph).HLpoint_vertex_map[hp] = current_vertex;
        }


        /// connect to previous headlandpoint
        if (i > 0)
            insertHeadlandEdges(graph, predecessor_vertex, current_vertex);
        else
            first_vertex = current_vertex;

        if(i+1 == track.points.size() && !isClosed)
            insertHeadlandEdges(graph, current_vertex, first_vertex);

        /// connect to IF tracks
        auto it = headlandPoints2TrackPoints.find(hp);
        if (it != headlandPoints2TrackPoints.end()) {
            for(auto vt : it->second){
                insertHeadlandEdges(graph, vt, current_vertex);
            }
        }
        predecessor_vertex = current_vertex;
    }
    return true;

}

void GraphBuilder_RoutesBased::connectHLandIFTracks(Graph &graph)
{

    //connect all infield track start/ends to the headland
    for(auto& it_eps: m_track_endpoints_map){
        for(auto& it_ep: it_eps.second){
            vertex_t nearest_vertex;
            bool found = false;
            double min_dist = std::numeric_limits<double>::max();

            //search for nearest HLTrack vertex
            for (auto &v_hl : graph.HLProcessingVertices()) {
                DirectedGraph::vertex_property& v_hl_prop = graph[v_hl];
                double current_dist = arolib::geometry::calc_dist(it_ep.second, v_hl_prop.route_point);
                if (current_dist < min_dist) {
                    min_dist = current_dist;
                    nearest_vertex = v_hl;
                    found = true;
                }
            }

            /// connect track point to nearest headland point
            if(found){
                insertHeadlandEdges(graph, nearest_vertex, it_ep.first);
                //mapHLVertexToTrackStartEnd[current_vertex].insert(map_it->first);
            }

        }
    }

    //connect infield first and last track to the headland
    for(auto& it_rp: graph.routepoint_vertex_map()){
        const RoutePoint& rp = it_rp.first;
        if(rp.track_id != graph.minTrackId_IF() && rp.track_id != graph.maxTrackId_IF())
            continue;
        if(rp.type == RoutePoint::TRACK_START || rp.type == RoutePoint::TRACK_END)//already connected
            continue;

        if(m_IFVerticesOutsideIF.find(it_rp.second) != m_IFVerticesOutsideIF.end())//@todo check if not connecting these vertices causes problems in the path search
            continue;

        vertex_t nearest_vertex;
        bool found = false;
        double min_dist = std::numeric_limits<double>::max();

        //search for nearest HLTrack vertex
        for (auto &v_hl : graph.HLProcessingVertices()) {
            DirectedGraph::vertex_property& v_hl_prop = graph[v_hl];
            double current_dist = arolib::geometry::calc_dist(rp, v_hl_prop.route_point);
            if (current_dist < min_dist) {
                min_dist = current_dist;
                nearest_vertex = v_hl;
                found = true;
            }
        }

        /// connect track point to nearest headland point
        if(found){
            insertHeadlandEdges(graph, nearest_vertex, it_rp.second);
            //mapHLVertexToTrackStartEnd[current_vertex].insert(map_it->first);
        }
    }

}

void GraphBuilder_RoutesBased::getHeadlandBoundaryPoints(const Graph &graph,
                                                     const std::vector<HeadlandRoute> &routes,
                                                     const Polygon &field_bounday,
                                                     std::vector<Point> &boundary_points)
{
    boundary_points.clear();
    std::vector<Point> boundary = field_bounday.points;

//    boundary_points.push_back(field_bounday.points.front());

    for(const auto &route : routes){
        for(const auto &p : route.route_points){
            if(p.track_id != graph.minTrackId_HL() || p.type == RoutePoint::TRACK_END)
                continue;
            Point bp;
            if( !getConnectionToHeadland(boundary,
                                         p,
                                         p,
                                         bp,
                                         arolib::geometry::HeadlandConnectionStrategy::MIN_DIST_TO_HEADLAND_SEGMENT,
                                         true) )
                continue;
//            boundary_points.push_back(bp);
        }
    }

//    boundary_points.push_back(field_bounday.points.front());

    boundary_points = boundary;
}

void GraphBuilder_RoutesBased::insertHeadlandBoundaryPoints(Graph &graph,
                                                        const std::vector<Point> &boundary_points)
{
    if(boundary_points.empty())
        return;

    vertex_t predecessor_vertex;
    vertex_t first_vertex;

    std::map<vertex_t, Point> HLVertices;


    bool isClosed = (boundary_points.front() == boundary_points.back());

    for (unsigned int i = 0; i < boundary_points.size(); ++i) {
        /// insert point
        vertex_t current_vertex;

        if(i+1 == boundary_points.size() && isClosed)
            current_vertex = first_vertex;
        else{
            current_vertex = addHeadlandPoint(graph, boundary_points.at(i), -1);
            HLVertices[current_vertex] = boundary_points.at(i);
        }

        /// connect to previous headlandpoint
        if (i > 0)
            insertHeadlandEdges(graph, predecessor_vertex, current_vertex, EdgeType::BOUNDARY_CONN);
        else
            first_vertex = current_vertex;

        if(i+1 == boundary_points.size() && !isClosed)
            insertHeadlandEdges(graph, current_vertex, first_vertex, EdgeType::BOUNDARY_CONN);
        predecessor_vertex = current_vertex;
    }

    /// connect first track to headland boundary
    for(auto &trackVertex : m_verticesFirstLastTracks){
        vertex_t nearest_vertex;
        double min_dist = std::numeric_limits<double>::max();
        bool found = false;
        for(auto &HLVertex : HLVertices){
            double current_dist = arolib::geometry::calc_dist(trackVertex.second, HLVertex.second);
            if (current_dist < min_dist) {
                min_dist = current_dist;
                nearest_vertex = HLVertex.first;
                found = true;
            }
        }
        if(found)
            insertHeadlandEdges(graph, trackVertex.first, nearest_vertex, EdgeType::BOUNDARY_CONN);
    }

}

void GraphBuilder_RoutesBased::insertInterGraphConnection(Graph &graph, const std::vector<HeadlandRoute> &harvHeadlandRoutes, const std::vector<Route> &harvInfieldRoutes)
{
    //TODO: at the moment it is not necessary because the only way the OLV connect the 2 subgraphs is while following the harvester, which is done directly without graph-based planning
}

void GraphBuilder_RoutesBased::insertInitialPositions(Graph &graph,
                                                  const Subfield &subfield,
                                                  const std::vector<Machine> &machines,
                                                  const std::map<MachineId_t, MachineDynamicInfo> &machineInitialStates,
                                                  const OutFieldInfo &outFieldInfo,
                                                  const Polygon &field_bounday,
                                                  bool connectInHeadlandSubGraph,
                                                  bool connectInInfieldSubGraph)
{
    insertOrReplaceInitialPositions(graph,
                                    subfield,
                                    machines,
                                    Machine::getWorkingTypes(true),
                                    machineInitialStates,
                                    outFieldInfo,
                                    field_bounday,
                                    connectInHeadlandSubGraph,
                                    connectInInfieldSubGraph,
                                    true);

    insertOrReplaceInitialPositions(graph,
                                    subfield,
                                    machines,
                                    Machine::getNonWorkingTypes(false),
                                    machineInitialStates,
                                    outFieldInfo,
                                    field_bounday,
                                    connectInHeadlandSubGraph,
                                    connectInInfieldSubGraph,
                                    true);

    for(const auto& it : graph.initialpoint_vertex_map()){
        const auto& machineId = it.first;
        const auto& vt = it.second;
        const auto& v_prop = graph[vt];

        for(auto out_edges = boost::out_edges(vt, graph) ;out_edges.first != out_edges.second; out_edges.first++){
            const auto& edge = *out_edges.first;
            const auto& edge_prop = graph[edge];

            const auto& vt2 = target(edge, graph);
            const auto& v_prop2 = graph[vt2];

            appendToOutputBuffer("INIT (" + std::to_string(machineId) + ")",
                                 v_prop.route_point,
                                 v_prop2.route_point,
                                 edge_prop.edge_type,
                                 vt,
                                 vt2,
                                 false);
        }

    }

}

double GraphBuilder_RoutesBased::getMachineRadius(const std::map<MachineId_t, Machine> &machinesMap, MachineId_t machineId)
{
    double r = 0;
    auto it_m = machinesMap.find(machineId);
    if(it_m == machinesMap.end())
        return -1;
    return it_m->second.workingRadius();
}

void GraphBuilder_RoutesBased::addVisitPeriods(Graph &graph,
                                            const std::map<MachineId_t, Machine> &machinesMap,
                                            const std::vector<HeadlandRoute> &harvHeadlandRoutes,
                                            const std::vector<Route> &harvInfieldRoutes)
{
    for(auto route : harvHeadlandRoutes){
        auto m_it = machinesMap.find(route.machine_id);
        if(m_it == machinesMap.end())
            continue;
        updateVisitingPeriods(graph,
                              m_it->second,
                              route.route_points,
                              0,
                              route.route_points.size()-1);
    }

    for(auto route : harvInfieldRoutes){
        auto m_it = machinesMap.find(route.machine_id);
        if(m_it == machinesMap.end())
            continue;
        updateVisitingPeriods(graph,
                              m_it->second,
                              route.route_points,
                              0,
                              route.route_points.size()-1);
    }
}

bool GraphBuilder_RoutesBased::adjustMap(ArolibGrid_t &map, const Polygon &boundary, const double &default_value)
{

    if(map.isAllocated() && !boundary.points.empty()){
        bool only_perimeter = (default_value == 0);
        return map.expandGridFromPolygon(boundary.points, default_value, only_perimeter, true);
    }
    return true;
}

bool GraphBuilder_RoutesBased::getClosestPointInTrack(const RoutePoint &rpRef, const std::vector<std::pair<vertex_t, vertex_property> > &trackVertices, RoutePoint &rp_out, double &min_dist)
{
    bool ok = false;
    min_dist = std::numeric_limits<double>::max();
    for (const auto &vt_info : trackVertices) {
        const vertex_property &v_prop = vt_info.second;
        const auto& rp = v_prop.route_point;
        double d = arolib::geometry::calc_dist(rp, rpRef);
        if (d > 1e-3 && d <= min_dist) {
            min_dist = d;
            rp_out = rp;
            ok = true;
        }
    }
    return ok;
}

bool GraphBuilder_RoutesBased::getClosestValidHeadlandVertex(const Point &pRef,
                                                         const Point &pInTrack,
                                                         int track_id,
                                                         const std::map<vertex_t, std::pair<RoutePoint, std::set<int> > > &HLVertices,
                                                         const Subfield &subfield,
                                                         double workingWidth,
                                                         double angleLimit,
                                                         vertex_t &vt_out,
                                                         RoutePoint &rp_out,
                                                         double &min_dist)
{
    bool ok = false;
    min_dist = std::numeric_limits<double>::max();

    angleLimit = std::fabs(angleLimit);

    for(const auto &hlv_it : HLVertices){
        const RoutePoint& rpHL = hlv_it.second.first;
        if( hlv_it.second.second.find(track_id) != hlv_it.second.second.end() )
            continue;//headland vertex is already connected to a start/end vertex of that track

        double angle = arolib::geometry::get_angle(pRef, pInTrack, pRef, rpHL, true, true);
        if( std::fabs(angle) < angleLimit ){
            //std::cout << rpHL.point.toStringCSV({ rpHL.point, rpRef.point, pInTrack }, ';', 10) << std::endl;
            continue;
        }

        double d = arolib::geometry::calc_dist(rpHL, pRef);
        if (d <= min_dist && d < 2*workingWidth) {
            min_dist = d;
            vt_out = hlv_it.first;
            rp_out = rpHL;
            ok = true;
        }
    }

    if(ok){
        auto intersections = arolib::geometry::get_intersection(rp_out,
                                              pRef,
                                              subfield.boundary_inner);
        for(auto &pInt : intersections){
            if( arolib::geometry::calc_dist(pInt, pRef) > 2*workingWidth ){
                ok = false;
                break;
            }
        }
    }

    return ok;
}

bool GraphBuilder_RoutesBased::sampleHLTracksPerpendicularly(const Subfield &sf, std::vector<Track> &tracksHL, double resolution)
{
    for(size_t i = 0 ; i < tracksHL.size() ; ++i){
        if(tracksHL.at(i).points.size() < 2){
            m_logger.printError(__FUNCTION__, "The subfield has an invalid valid track: " + std::to_string(i));
            return false;
        }
    }

    if(tracksHL.empty()){
        m_logger.printError(__FUNCTION__, "The subfield has no tracks.");
        return false;
    }

    //sample closest track
    auto& firstTrack = tracksHL.back().points;
    arolib::geometry::unsample_linestring(firstTrack);

    Polygon polyTrack;
    polyTrack.points = firstTrack;
    double multAngle = 1;//multiplier used to obtain the rotation angle (for the perpendicular vectors)
    if(!arolib::geometry::isPolygonClockwise(polyTrack))
        multAngle = -1;

    for(auto& track_it : m_track_endpoints_map){
        for(auto& ep_it : track_it.second)
            arolib::geometry::addSampleToGeometryClosestToPoint(firstTrack, ep_it.second);
    }

    const double minDeltaSample = 0.3*resolution;
    const double minDeltaPerpSample = 0.1*resolution;

    for(int i = tracksHL.size()-2 ; i >= 0 ; --i){
        auto &track = tracksHL.at(i);
        auto &trackRef = tracksHL.at(i+1);

        trackRef.points = arolib::geometry::sample_geometry(trackRef.points, resolution, minDeltaSample);
        arolib::geometry::unsample_linestring(track.points);

        std::vector< std::pair<Point, Point> > perpLines;
        std::vector< Point > refPoints;

        if(trackRef.points.size() > 3){//get the perpendicular points iif
            {
                auto& p0 = trackRef.points.front();
                auto& p1 = trackRef.points.at(1);
                auto& p2 = *( trackRef.points.end()-2 );

                //get the angle between the 2 (consecutive) segments to which p0 belongs (p0 being the connection between the 2 segments)
                double ang1 = ( multAngle>0 ?
                                    arolib::geometry::get_angle(p0, p1, p0, p2, true) :
                                    arolib::geometry::get_angle(p0, p2, p0, p1, true) );
                if( ang1 < 0 || ang1 >= 180){
                    Point pp = arolib::geometry::rotate(p0, p1, M_PI_2 * multAngle);
                    perpLines.push_back( std::make_pair(p0, pp) );
                }
                else if(ang1 > 175)
                    refPoints.emplace_back(p0);
            }
            for(size_t ii = 1 ; ii+1 < trackRef.points.size() ; ++ii){
                auto& p0 = trackRef.points.at(ii);
                auto& p1 = trackRef.points.at(ii+1);
                auto& p2 = trackRef.points.at(ii-1);

                //get the angle between the 2 (consecutive) segments to which p0 belongs (p0 being the connection between the 2 segments)
                double ang1 = ( multAngle>0 ?
                                    arolib::geometry::get_angle(p0, p1, p0, p2, true) :
                                    arolib::geometry::get_angle(p0, p2, p0, p1, true) );
                if( ang1 > 0 && ang1 < 180){
                    if(ang1 > 175)
                        refPoints.emplace_back(p0);
                    continue;
                }
                if( std::fabs( arolib::geometry::get_angle(p0, p1, p0, p2, true) ) < 170){
                    Point pp0 = arolib::geometry::rotate(p0, p2, M_PI_2 * -multAngle);
                    perpLines.push_back( std::make_pair(p0, pp0) );
                }
                Point pp = arolib::geometry::rotate(p0, p1, M_PI_2 * multAngle);
                perpLines.push_back( std::make_pair(p0, pp) );
            }
            {//add last ref perp point
                auto& p0 = trackRef.points.front();
                auto& p1 = trackRef.points.at(1);
                auto& p2 = *( trackRef.points.end()-2 );
                double ang1 = ( multAngle>0 ?
                                    arolib::geometry::get_angle(p0, p1, p0, p2, true) :
                                    arolib::geometry::get_angle(p0, p2, p0, p1, true) );
                if( ang1 < 0 || ang1 >= 180-1e-3){
                    if( std::fabs( arolib::geometry::get_angle(p0, p1, p0, p2, true) ) < 170){
                        Point pp0 = arolib::geometry::rotate(p0, p2, M_PI_2 * -multAngle);
                        perpLines.push_back( std::make_pair(p0, pp0) );
                    }
                }
            }
        }

        size_t refIndex = 0;
        for(int j = -1 ; j+1 < track.points.size() ; ++j){

            Point p0;
            auto& p1 = track.points.at(j+1);

            if(j < 0)
                p0 = *( track.points.end()-2 );
            else
                p0 = track.points.at(j);

            if(refIndex < perpLines.size()){
                auto& p0_ref = perpLines.at(refIndex).first;
                auto& p1_ref = perpLines.at(refIndex).second;
                Point sample;

                if( arolib::geometry::get_intersection( p0_ref, p1_ref, p0, p1, sample, true )
                        && std::fabs( arolib::geometry::get_angle(p0_ref, p1_ref, p0_ref, sample) ) < M_PI_2
                        /*&& arolib::geometry::calc_dist(p0_ref, sample) < 0.5*headlandWidth*/ ){
                    if( arolib::geometry::calc_dist(sample, p0) > minDeltaPerpSample &&
                            arolib::geometry::calc_dist(sample, p1) > minDeltaPerpSample ){

                        Point p3;
                        if(j+2 < track.points.size() )
                            p3 = track.points.at(j+2);
                        else
                            p3 = track.points.at(1);

                        if( arolib::geometry::calc_dist(sample, p1) < resolution
                                && arolib::geometry::is_line(std::vector<Point>{sample, p1, p3}, 0.1) ){
                            if (j == -1){
                                track.points.front() = sample;
                                track.points.back() = sample;
                            }
                            else
                                track.points.at(j+1) = sample;
                            --j;
                        }
                        else{
                            if(j < 0){
                                track.points.insert( track.points.end()-1, sample );
                                --j;
                            }
                            else
                                track.points.insert( track.points.begin()+j+1, sample );
                        }

                    }
                    else
                        --j;
                    ++refIndex;
                }
            }
        }

        for(auto& p :  refPoints)
            arolib::geometry::addSampleToGeometryClosestToPoint(track.points, p, 1, minDeltaPerpSample);
    }

    tracksHL.front().points = arolib::geometry::sample_geometry(tracksHL.front().points, resolution, minDeltaSample);

    return true;
}

void GraphBuilder_RoutesBased::connectHLTracks(Graph &graph, const Subfield &sf, std::vector<Track> &tracksHL)
{
    //create a pseudo headland route from the tracks
    std::vector<HeadlandRoute> route(1);
    for(auto& track : tracksHL){
        for(size_t i = 0 ; i < track.points.size() ; ++i){
            RoutePoint rp;
            if(i == 0)
                rp.type = RoutePoint::TRACK_START;
            else if(i+1 < track.points.size())
                rp.type = RoutePoint::DEFAULT;
            else
                rp.type = RoutePoint::TRACK_END;
            rp.track_id = track.id;
            rp.point() = track.points.at(i);
            rp.time_stamp = -1;
            rp.harvested_mass = 0;
            rp.harvested_volume = 0;
            route.front().route_points.emplace_back(rp);
        }
    }
    insertRoutes(graph, route);
    interconnectTracksHL(graph);
}

void GraphBuilder_RoutesBased::clear()
{
    m_track_endpoints_map.clear();
    m_verticesFirstLastTracks.clear();
}

void GraphBuilder_RoutesBased::appendToOutputBuffer(const std::string &added,
                                                    const RoutePoint &p0,
                                                    const RoutePoint &p1,
                                                    EdgeType edge_type,
                                                    const vertex_t &vt_from,
                                                    const vertex_t &vt_to,
                                                    bool bidirectional)
{
    if(m_outputFile.empty())
        return;
    std::string addedEd = added;
    if(m_currentGraphType == GRAPH_IF)
        addedEd += "__IF";
    else if(m_currentGraphType == GRAPH_HL)
        addedEd += "__HL";
    m_buildingInfoManager.addEdgeInfo( GraphBuildingInfoManager::EdgeInfo(addedEd, edge_type, vt_from, vt_to, p0, p1, p0.type, p1.type, bidirectional) );
}

void GraphBuilder_RoutesBased::flushOutputBuffer()
{

    if(m_outputFile.empty())
        return;
    m_buildingInfoManager.saveEdgesInfo(m_outputFile);

}

}

}
