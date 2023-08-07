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
 
#include "arolib/planning/path_search/graph_builder_tracks_based.hpp"

#include <limits>       // std::numeric_limits

//#define DEBUG_GRAPH


namespace arolib{
namespace DirectedGraph{

GraphBuilder_TracksBased::GraphBuilder_TracksBased(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{

}

GraphBuilder_TracksBased::~GraphBuilder_TracksBased(){

}

Graph GraphBuilder_TracksBased::buildGraph(const Subfield &subfield, const std::vector<Route> &baseWorkingRoutes, const std::vector<ResourcePoint> &resource_points, const std::vector<FieldAccessPoint> &field_access_points, const OutFieldInfo &outFieldInfo, const std::vector<Machine> &machines, const std::map<MachineId_t, MachineDynamicInfo> &machineInitialStates, double workingWidthIF, double workingWidthHL, bool includeVisitPeriods, const std::string &outputFile) const
{

    BuilderWorkspace ws;

    getGraphData(ws.graph).outFieldInfo = outFieldInfo;
    getGraphData(ws.graph).workingWidth_HL = workingWidthHL;
    getGraphData(ws.graph).workingWidth_IF = workingWidthIF;

    ws.machines = Machine::toMachineIdMap(machines);//for easier access to the machines
    ws.outputFile = outputFile;


    double distOffset = ( ws.graph.workingWidth_IF() > 0 ? ws.graph.workingWidth_IF() : ws.graph.workingWidth_HL() );
    distOffset = ( distOffset > 1e-9 ? 0.01 * distOffset : 1e-3 );
    if( !geometry::offsetPolygon(subfield.boundary_inner, ws.innerBoundary_offset, distOffset, true) ){
        ws.innerBoundary_offset = subfield.boundary_inner;
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Error offsetting inner boundary");
    }

    addInnerFieldTracks(ws, subfield);
    interconnectInnerFieldTracks(ws, subfield);
    addHeadlandTracks(ws, subfield);
    interconnectHeadlandTracks(ws, subfield);
    connectHeadlands(ws, subfield);
    connectInnerFieldTracksToHeadland(ws, subfield);
    addBoundaryVertices(ws, subfield);
    connectInnerFieldTracksToBoundary(ws, subfield);
    connectHeadlandTracksToBoundary(ws);
    addFieldAccessPoints(ws, subfield, field_access_points);
    connectFieldAccessPointsToHeadland(ws);
    connectFieldAccessPointsToBoundary(ws);
    interconnectFieldAccessPoints(ws, outFieldInfo);
    addResourcePoints(ws, subfield, resource_points, field_access_points, outFieldInfo);
    connectResourcePoints(ws, subfield, outFieldInfo);
    addInitialPositions(ws, subfield, outFieldInfo, machineInitialStates);
    connectInitialPositions(ws, subfield, outFieldInfo, machineInitialStates);
    addBaseRouteInformation(ws.graph, baseWorkingRoutes, &ws);
    if(includeVisitPeriods)
        addVisitPeriods(ws.graph, ws.machines, baseWorkingRoutes);

    flushOutputBuffer(ws);

    logger().printOut(LogLevel::INFO, __FUNCTION__, "Graph expanded!" );

    return ws.graph;

}

AroResp GraphBuilder_TracksBased::addBaseRouteInformationToGraph(Graph &graph, const std::vector<Machine> &machines, const std::vector<Route> &baseWorkingRoutes, bool includeVisitPeriods) const
{
    if(!graph.routepoint_vertex_map().empty())
        return AroResp::LoggingResp(1, "The given graph has already route information", logger(), LogLevel::ERROR, __FUNCTION__);

    auto machinesMap = Machine::toMachineIdMap(machines);
    for(auto& route : baseWorkingRoutes){
        if( machinesMap.find(route.machine_id) == machinesMap.end() )
            return AroResp::LoggingResp(1, "Not all route machines were given", logger(), LogLevel::ERROR, __FUNCTION__);
    }

    addBaseRouteInformation(graph, baseWorkingRoutes, nullptr);
    if(includeVisitPeriods)
        addVisitPeriods(graph, machinesMap, baseWorkingRoutes);

    return AroResp::ok();
}

void GraphBuilder_TracksBased::addInnerFieldTracks(BuilderWorkspace &ws, const Subfield &sf) const
{
    double workingWidth = std::max(1e-2, ws.graph.workingWidth_HL() > 0 ? ws.graph.workingWidth_HL() : ws.graph.workingWidth_IF() );

    Polygon outterBoundaryEd;
    if(!geometry::offsetPolygon(sf.boundary_outer, outterBoundaryEd, 0.01, true, 0)){
        outterBoundaryEd = sf.boundary_outer;
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Error offsetting outter boundary");
    }
    geometry::correct_polygon(outterBoundaryEd);
    if(geometry::isPolygonValid(outterBoundaryEd) == geometry::PolygonValidity::INVALID_POLYGON)
        outterBoundaryEd.points.clear();

    Polygon innerBoundaryEd;
    if( !geometry::offsetPolygon(sf.boundary_inner, innerBoundaryEd, workingWidth, true) ){
        innerBoundaryEd = sf.boundary_inner;
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Error offsetting inner boundary");
    }

    auto addTrack = [this, &ws, &innerBoundaryEd, &outterBoundaryEd](const std::vector<Point>& points, int track_id){

        auto createVertexProperty = [track_id](const Point& p, bool trackEnd)->vertex_property{
            vertex_property vp;
            vp.route_point.point() = p;
            vp.route_point.time_stamp = -1;
            vp.route_point.track_id = track_id;
            vp.route_point.type = trackEnd ? RoutePoint::TRACK_END : RoutePoint::DEFAULT;
            vp.graph_location = DirectedGraph::vertex_property::DEFAULT;
            return vp;
        };

        int ind_start =  points.size(), ind_end = -1;//indexes containing the first and last tarck point inside the inner boundary (i.e. the points to be considered track-ends)
        for(size_t i = 0 ; i < points.size() ; ++i){
            if( geometry::in_polygon(points.at(i), innerBoundaryEd)
                    || geometry::calc_dist_to_linestring(innerBoundaryEd.points, points.at(i))  ){
                ind_start = i;
                break;
            }
        }
        for(int i = -1 + points.size() ; i > ind_start ; --i){
            if( geometry::in_polygon(points.at(i), innerBoundaryEd)
                    || geometry::calc_dist_to_linestring(innerBoundaryEd.points, points.at(i)) < 1e-9 ){
                ind_end = i;
                break;
            }
        }

        ws.verticesInfieldTracks.push_back({});
        if(points.empty())
            return;

        ws.verticesInfieldTracks.back().reserve( points.size() );

        Point invalidPt = Point::invalidPoint();

        vertex_property prev_vp;
        vertex_t prev_vt;

        bool prevVtExists = false;

        for(size_t i = 0 ; i < points.size() ; ++i){
            if(!outterBoundaryEd.points.empty() && !geometry::in_polygon(points.at(i), outterBoundaryEd)){
                prevVtExists = false;
                continue;
            }

            bool isTrackEnd = ( i == ind_start || i == ind_end );
            vertex_property vp = createVertexProperty(points.at(i), isTrackEnd);
            vertex_t vt = ws.graph.addVertex( vp );
            getGraphData(ws.graph).tracks_vertices_map[track_id].push_back(vt);

            bool pointOutOfIF = i >= ind_start && i <= ind_end && !geometry::in_polygon(vp.route_point, innerBoundaryEd); //for the track points that go outside the inner boundary (i.e. the track was interrupted)
            if (pointOutOfIF){//only add the vertex (@todo: check if this still makes sense)
                ws.IFVerticesOutsideIF.insert(vt);
            }
            else if (prevVtExists){
                addEdge( ws,
                         geometry::calc_dist( points.at(i-1), points.at(i) ),
                         EdgeType::DEFAULT,
                         prev_vt, vt,
                         prev_vp.route_point, vp.route_point,
                         ws.graph.workingWidth_HL(),
                         "IFT",
                         true,
                         true,
                         false );
            }
            prev_vp = vp;
            prev_vt = vt;
            prevVtExists = true;
            ws.verticesInfieldTracks.back().push_back( prev_vt );
            if(i == ind_start){
                if( i+1 <= ind_end )
                    ws.verticesInfieldTracksEnds.push_back( std::make_pair( prev_vt, points.at(i+1) ) );
                else
                    ws.verticesInfieldTracksEnds.push_back( std::make_pair( prev_vt, invalidPt ) );
            }
            else if(i == ind_end){
                if( i-1 >= ind_start )
                    ws.verticesInfieldTracksEnds.push_back( std::make_pair( prev_vt, points.at(i-1) ) );
                else
                    ws.verticesInfieldTracksEnds.push_back( std::make_pair( prev_vt, invalidPt ) );
            }
        }
    };

    for(const auto& track : sf.tracks)
        addTrack(track.points, track.id);

    auto extremaTrackInds = geometry::getInfieldExtremaTracksIndexes(sf, {});
    for(auto& ind : extremaTrackInds)
        getGraphData(ws.graph).extremaTrackIds_IF.insert(sf.tracks.at(ind).id);

    for(auto& track : ws.verticesInfieldTracks){
        ws.verticesInfieldTracks_set.insert(track.begin(), track.end());
    }
}

void GraphBuilder_TracksBased::interconnectInnerFieldTracks(BuilderWorkspace &ws, const Subfield &sf) const
{
    Polygon outterBoundaryEd;
//    if(!geometry::offsetPolygon(sf.boundary_outer, outterBoundaryEd, 0.01, true, 0))
//        outterBoundaryEd = sf.boundary_outer;
//    geometry::correct_polygon(outterBoundaryEd);
//    if(geometry::isPolygonValid(outterBoundaryEd) == geometry::PolygonValidity::INVALID_POLYGON)
//        outterBoundaryEd.points.clear();

    //connect (if possible/suitable) every IF track-point-vertex to the closest vertex in the adjacent tracks
    auto adjacencyList = geometry::getAdjacentTracksList(sf.tracks, true);
    for(size_t i = 0 ; i < sf.tracks.size() ; ++i){
        const auto &track_vts = ws.verticesInfieldTracks.at(i);
        auto it_adj = adjacencyList.find(i);
        if(it_adj == adjacencyList.end()){
            ws.verticesInfieldTracks_additionalConnection.insert( track_vts.begin(), track_vts.end() );
            continue;
        }
        double trackWidthFrom = sf.tracks.at(i).width;

        const std::set<size_t>& adjTrackInds = it_adj->second;


        for (auto &vt_from : track_vts){
            const vertex_property& vp_from = ws.graph[vt_from];

            if(!outterBoundaryEd.points.empty() && !geometry::in_polygon(vp_from.route_point, outterBoundaryEd))
                continue;

            size_t countConnections = 0;

            for(auto adjTrackInd : adjTrackInds){
                const auto &adj_vts = ws.verticesInfieldTracks.at(adjTrackInd);
                double trackWidthTo = sf.tracks.at(adjTrackInd).width;

                double compDist = ws.graph.workingWidth_IF();
                if(trackWidthFrom > 1e-9 && trackWidthTo > 1e-9)
                    compDist = 0.5 * (trackWidthFrom + trackWidthTo);

                double min_dist = std::numeric_limits<double>::max();
                vertex_t vt_to;
                vertex_property vp_to;
                bool found = false;

                //get the vertex in next track closest to the current vertex
                for (auto &vt : adj_vts) {
                    const vertex_property& vp = ws.graph[vt];
                    double d = geometry::calc_dist(vp.route_point, vp_from.route_point);
                    if (d <= min_dist) {
                        min_dist = d;
                        vt_to = vt;
                        vp_to = vp;
                        found = true;
                    }

                }

                if(!found)
                    continue;

//                if( ( rp_next.type == RoutePoint::TRACK_START || rp_next.type == RoutePoint::TRACK_END )
//                        && min_dist > 1.5 * ws.graph.workingWidth_IF() ){//connect to TRACK_START and TRACK_END vertices from the next/previous track iif they are not too far away from the vertex. If they are too far, it might be better to connect it to the headland (later on)
                if(min_dist > 1.5 * compDist) //do not connect if they are far
                    continue;

                addEdge( ws,
                         min_dist,
                         EdgeType::CROSS,
                         vt_from,
                         vt_to,
                         vp_from.route_point,
                         vp_to.route_point,
                         getGraphData(ws.graph).workingWidth_IF,
                         "CIT",
                         true,
                         true,
                         false );

                getGraphData(ws.graph).adjacentTrackIds_IF[vp_from.route_point.track_id].insert(vp_to.route_point.track_id);

                ++countConnections;

            }
            if(countConnections < 2)
                ws.verticesInfieldTracks_additionalConnection.insert(vt_from);
        }

    }

    //add vertices from extrema tracks to list of verticess needing extra connections
    auto extremaTracksInds = geometry::getInfieldExtremaTracksIndexes(sf);
    for(auto& ind : extremaTracksInds)
        ws.verticesInfieldTracks_additionalConnection.insert( ws.verticesInfieldTracks.at(ind).begin(), ws.verticesInfieldTracks.at(ind).end() );
}

void GraphBuilder_TracksBased::addHeadlandTracks(BuilderWorkspace &ws, const Subfield &sf) const
{
    auto boundaryPts = sf.boundary_outer.points;
    geometry::unsample_linestring(boundaryPts);

    auto getIdOfOutermostHLTrack = [&boundaryPts](const std::vector<Track>& tracks) -> int{
        int id = -1;
        double minDist = std::numeric_limits<double>::max();
        for(auto& track : tracks){
            if(track.points.empty())
                continue;
            auto pts = track.points;
            geometry::unsample_linestring(pts);
            int iFrom = 0, iTo = pts.size();
            if(pts.size() > 2){
                iFrom++;
                iTo--;
            }
            for(int i = iFrom ; i < iTo ; ++i){
                auto& p = pts.at(i);
                double dist = geometry::calc_dist_to_linestring(boundaryPts, p);
                if(minDist > dist){
                    minDist = dist;
                    id = track.id;
                }
            }
        }
        return id;
    };

    auto addTrack = [this, &ws](const std::vector<Point>& points, int track_id, size_t headlandIdx){
        auto createVertexProperty = [track_id](const Point& p)->vertex_property{
            vertex_property vp;
            vp.route_point.point() = p;
            vp.route_point.time_stamp = -1;
            vp.route_point.track_id = track_id;
            vp.route_point.type = RoutePoint::HEADLAND;
            vp.graph_location = DirectedGraph::vertex_property::DEFAULT;
            return vp;
        };

        ws.verticesHeadlandTracks.at(headlandIdx).push_back( {} );
        if(points.empty())
            return;
        ws.verticesHeadlandTracks.at(headlandIdx).back().reserve(points.size());

        bool closed = points.size() > 2 && points.front() == points.back();

        vertex_property prev_vp = createVertexProperty(points.front());
        vertex_t prev_vt = ws.graph.addVertex( prev_vp );
        getGraphData(ws.graph).tracks_vertices_map[track_id].push_back(prev_vt);


        ws.verticesHeadlandTracks.at(headlandIdx).back().push_back(prev_vt);

        vertex_t first_vt = prev_vt;
        vertex_property first_vp = prev_vp;

        for(size_t i = 1 ; i+closed < points.size() ; ++i){
            vertex_property vp = createVertexProperty(points.at(i));
            vertex_t vt = ws.graph.addVertex( vp );            
            getGraphData(ws.graph).tracks_vertices_map[track_id].push_back(vt);

            addEdge( ws,
                     geometry::calc_dist( points.at(i-1), points.at(i) ),
                     EdgeType::DEFAULT,
                     prev_vt, vt,
                     prev_vp.route_point, vp.route_point,
                     ws.graph.workingWidth_HL(),
                     "HLT",
                     true,
                     true,
                     false );
            prev_vp = vp;
            prev_vt = vt;
            ws.verticesHeadlandTracks.at(headlandIdx).back().push_back(prev_vt);
        }

        if(closed){
            addEdge( ws,
                     geometry::calc_dist( points.front(), r_at(points, 1) ),
                     EdgeType::DEFAULT,
                     prev_vt, first_vt,
                     prev_vp.route_point, first_vp.route_point,
                     ws.graph.workingWidth_HL(),
                     "HLT",
                     true,
                     true,
                     false );
        }

    };
    const Headlands &headlands = sf.headlands;
    ws.hasSurroundingHeadland = headlands.hasCompleteHeadland();
    if( ws.hasSurroundingHeadland ){
        getGraphData(ws.graph).hasPartialHeadlands = false;
        ws.verticesHeadlandTracks.resize(1);
        if(!headlands.complete.tracks.empty()){
            for( auto& track : headlands.complete.tracks )
                addTrack(track.points, track.id, 0);

            int outermostTrackId = getIdOfOutermostHLTrack(headlands.complete.tracks);
            if(outermostTrackId >= 0)
                getGraphData(ws.graph).outermostTrackIds_HL.insert(outermostTrackId);
        }
        else{//use middle track
            auto points = headlands.complete.middle_track.points;

            //sample track based on the InnerField track points
            geometry::unsample_linestring(points);

            if(!sf.tracks.empty()){//sample to first IF track
                for(auto& p : sf.tracks.front().points)
                    geometry::addSampleToGeometryClosestToPoint(points, p, 1);
            }
            if(sf.tracks.size() > 1){//sample to last IF track
                for(auto& p : sf.tracks.back().points)
                    geometry::addSampleToGeometryClosestToPoint(points, p, 1);
            }

            for(size_t i = 1 ; i+1 < sf.tracks.size() ; ++i){//sample to track-ends
                if(sf.tracks.at(i).points.empty())
                    continue;
                geometry::addSampleToGeometryClosestToPoint(points, sf.tracks.at(i).points.front(), 1);
                geometry::addSampleToGeometryClosestToPoint(points, sf.tracks.at(i).points.back(), 1);
            }
            for(auto vt : ws.verticesInfieldTracks_additionalConnection){//sample to track points needing an additional connection
                geometry::addSampleToGeometryClosestToPoint(points, ws.graph[vt].route_point, 1);
            }

            if(ws.graph.workingWidth_IF() > 1e-6)
                points = geometry::sample_geometry(points, ws.graph.workingWidth_IF());

            addTrack(points, -1, 0);
        }
    }
    else{        
        getGraphData(ws.graph).hasPartialHeadlands = !headlands.partial.empty();
        ws.verticesHeadlandTracks.resize( headlands.partial.size() );
        for( size_t i = 0 ; i < headlands.partial.size() ; ++i ){
            for( auto& track : headlands.partial.at(i).tracks )
                addTrack(track.points, track.id, i);


            int outermostTrackId = getIdOfOutermostHLTrack(headlands.partial.at(i).tracks);
            if(outermostTrackId >= 0)
                getGraphData(ws.graph).outermostTrackIds_HL.insert(outermostTrackId);

            if(headlands.partial.at(i).isConnectingHeadland(false))
                ws.connectingHLsIndexes.insert(i);
        }
    }


    for(size_t i = 0 ; i < ws.verticesHeadlandTracks.size() ; ++i){
        for(auto& track : ws.verticesHeadlandTracks.at(i)){
            for(auto& vt : track){
                ws.verticesHeadlandTracks_set[vt] = i;
            }
        }
    }

}

void GraphBuilder_TracksBased::interconnectHeadlandTracks(BuilderWorkspace &ws, const Subfield &sf) const
{
    struct connectionProp{
        vertex_t vertex;
        RoutePoint p0;
        RoutePoint p1;
    };

    if(ws.verticesHeadlandTracks.empty())
        return;

    Polygon boundary;
    if( !geometry::offsetPolygon(sf.boundary_outer, boundary, ws.graph.workingWidth_HL() * 1e-3, true, 0) )//offset the boundary a bit to avoid intersections with the first and last track points of partial headlands
        boundary = sf.boundary_outer;

    for(auto & hl : ws.verticesHeadlandTracks){
        if(hl.size() < 2)
            continue;

        std::map<vertex_t, connectionProp> connectedVertices_prev;
        int countSwipes = -1;

        while (++countSwipes < 2){//swipe first from lower to highest track, and then reverse for unconnected vertices
            int ind = (countSwipes < 1 ? 0 : hl.size()-1);
            int deltaInd = (countSwipes < 1 ? 1 : -1);
            for( ; (countSwipes < 1 ? ind+1 < hl.size() : ind > 0 ) ; ind += deltaInd ){

                const auto &track_from = hl.at(ind);
                const auto &track_to = hl.at(ind + deltaInd);

                for(auto& vt_from : track_from){
                    const vertex_property& vp_from = ws.graph[vt_from];
                    vertex_property vp_to;

                    double min_dist = std::numeric_limits<double>::max();
                    vertex_t vt_to;
                    bool found = false;

                    for(auto& vt : track_to){
                        const vertex_property& vp = ws.graph[vt];
                        double d = geometry::calc_dist(vp.route_point, vp_from.route_point);
                        if (d <= min_dist) {
                            min_dist = d;
                            vt_to = vt;
                            vp_to = vp;
                            found = true;
                        }
                    }

                    if(!found)
                        continue;

                    if( !boundary.points.empty() && geometry::intersects( vp_from.route_point , vp_to.route_point , boundary.points) )
                        continue;

                    if(countSwipes > 0){//check if that same edge was alredy created
                        auto itConnPrev = connectedVertices_prev.find(vt_from);
                        if(itConnPrev != connectedVertices_prev.end()
                                && itConnPrev->second.vertex == vt_to)
                            continue;//already connected to that vertex in first (fwd) swipe
                    }

                    auto edgeType = EdgeType::CROSS_HL;


                    getGraphData(ws.graph).adjacentTrackIds_HL[vp_from.route_point.track_id].insert(vp_to.route_point.track_id);

                    //in partials headlands, some of the connections might correspond to to vertices which lie in the boundary
                    if( !ws.hasSurroundingHeadland &&
                            !boundary.points.empty() &&
                            geometry::calc_dist_to_linestring(boundary.points, vp_from.route_point) < 1e-2 &&
                            geometry::calc_dist_to_linestring(boundary.points, vp_to.route_point) < 1e-2 ){
                        //edgeType = EdgeType::BOUNDARY_CONN; //  -> treat the edge as BOUNDARY_CONN
                        continue; //  -> do not add edge (the connection will be done via the boundary edges
                    }


                    if( addEdge( ws,
                                 min_dist,
                                 edgeType,
                                 vt_from, vt_to,
                                 vp_from.route_point, vp_to.route_point,
                                 ws.graph.workingWidth_HL(),
                                 (countSwipes == 0 ? "CHTf" : "CHTb"),
                                 true,
                                 true,
                                 false ) ){


                        if(countSwipes == 0){
                            connectionProp connProp;
                            connProp.vertex = vt_from;
                            connProp.p0 = vp_to.route_point;
                            connProp.p1 = vp_from.route_point;
                            connectedVertices_prev[vt_to] = connProp;
                        }
                    }

                }

            }
        }
    }

    if(ws.verticesHeadlandTracks.size() == sf.headlands.partial.size()){//connect track-ends of main-headlands iif at least one of them is not over the boundary
        for(size_t i = 0 ; i < sf.headlands.partial.size() ; ++i){
            if(sf.headlands.partial.at(i).isConnectingHeadland())
                continue;
            const auto hl_vts = ws.verticesHeadlandTracks.at(i);
            for(size_t j = 0 ; j+1 < hl_vts.size() ; ++j){
                const auto& trackVts1 = hl_vts.at(j);
                const auto& trackVts2 = hl_vts.at(j+1);
                if(trackVts1.empty() || trackVts2.empty())
                    continue;


                auto vt1_f = trackVts1.front();
                auto vt1_b = trackVts1.back();
                DirectedGraph::vertex_property vp1_f = ws.graph[ vt1_f ];
                DirectedGraph::vertex_property vp1_b = ws.graph[ vt1_b ];

                auto vt2_f = trackVts2.front();
                auto vt2_b = trackVts2.back();
                DirectedGraph::vertex_property vp2_f = ws.graph[ vt2_f ];
                DirectedGraph::vertex_property vp2_b = ws.graph[ vt2_b ];

                double dist_f1 = geometry::calc_dist(vp1_f.route_point, vp2_f.route_point);
                double dist_f2 = geometry::calc_dist(vp1_f.route_point, vp2_b.route_point);
                double dist_b1 = geometry::calc_dist(vp1_b.route_point, vp2_b.route_point);
                double dist_b2 = geometry::calc_dist(vp1_b.route_point, vp2_f.route_point);
                if( dist_f1+dist_b1 > dist_f2+dist_b2 ){
                    std::swap(vt2_f, vt2_b);
                    std::swap(vp2_f, vp2_b);
                    std::swap(dist_f1, dist_f2);
                    std::swap(dist_b1, dist_b2);
                }

                for(int side = 0 ; side < 2 ; side++){
                    const auto& vt_from = ( side == 0 ? vt1_f : vt1_b );
                    const auto& vt_to = ( side == 0 ? vt2_f : vt2_b );
                    const auto& vp_from = ( side == 0 ? vp1_f : vp1_b );
                    const auto& vp_to = ( side == 0 ? vp2_f : vp2_b );
                    const auto& dist = ( side == 0 ? dist_f1 : dist_b1 );

                    if( geometry::intersects(vp_from.route_point, vp_to.route_point, boundary.points, false, false) )
                        continue;

                    if(boundary.points.empty() ||
                            geometry::calc_dist_to_linestring(boundary.points, vp_from.route_point) > 1e-2 ||
                            geometry::calc_dist_to_linestring(boundary.points, vp_to.route_point) > 1e-2){

                        addEdge( ws,
                                 dist,
                                 EdgeType::CROSS_HL,
                                 vt_from, vt_to,
                                 vp_from.route_point, vp_to.route_point,
                                 ws.graph.workingWidth_HL(),
                                 "CHT",
                                 true,
                                 true,
                                 false );
                    }
                }


            }
        }
    }
}

void GraphBuilder_TracksBased::connectHeadlands(BuilderWorkspace &ws, const Subfield &sf) const
{
    if(ws.hasSurroundingHeadland || ws.verticesHeadlandTracks.size() < 2)
        return;

    std::map<int, size_t> hlIdsMap;
    for(size_t i = 0 ; i < sf.headlands.partial.size() ; ++i)
        hlIdsMap[sf.headlands.partial.at(i).id] = i;

    for(size_t i = 0 ; i < sf.headlands.partial.size() ; ++i){
        if(i >= ws.verticesHeadlandTracks.size())
            continue;
        auto& hl = sf.headlands.partial.at(i);

        for(int connSide = 0 ; connSide < 2 ; ++connSide){
            int connHlId = ( connSide == 0 ? hl.connectingHeadlandIds.first : hl.connectingHeadlandIds.second );
            if(connHlId == PartialHeadland::NoConnectingHeadlandId)
                continue;
            auto it_id = hlIdsMap.find(connHlId);
            if(it_id == hlIdsMap.end() || it_id->second >= ws.verticesHeadlandTracks.size())
                continue;
            DirectedGraph::vertex_t vtSide1;
            auto& vts_hl = ws.verticesHeadlandTracks.at(i);
            auto& vts_hl_next = ws.verticesHeadlandTracks.at(it_id->second);
            if(vts_hl_next.empty())
                continue;

            for(auto& vts_track : vts_hl){
                if(vts_track.empty())
                    continue;
                bool ok = false;
                DirectedGraph::vertex_t vt, vt_next;
                double minDist = std::numeric_limits<double>::max();
                for(int side = 0 ; side < 2 ; ++side){
                    auto vtTmp = ( side == 0 ? vts_track.front() : vts_track.back() );

                    if(hl.connectingHeadlandIds.first == hl.connectingHeadlandIds.second
                            && connSide > 0 && vtSide1 == vtTmp)
                        continue;

                    const RoutePoint& rp = ws.graph[vtTmp].route_point;
                    for(int sideNext = 0 ; sideNext < 2 ; ++sideNext){
                        auto& vts_track_next = ( sideNext == 0 ? vts_hl_next.front() : vts_hl_next.back() );
                        for(auto& vtNextTmp : vts_track_next){
                            const RoutePoint& rpNext = ws.graph[vtNextTmp].route_point;
                            double dist = geometry::calc_dist(rp, rpNext);
                            if(minDist > dist){
                                ok = true;
                                minDist = dist;
                                vt = vtTmp;
                                vt_next = vtNextTmp;
                            }
                        }
                    }
                }
                if(!ok)
                    continue;

                vtSide1 = vt;
                auto& vp_from = ws.graph[vt];
                auto& vp_to = ws.graph[vt_next];

                addEdge( ws,
                         minDist,
                         EdgeType::CROSS_HL,
                         vt, vt_next,
                         vp_from.route_point, vp_to.route_point,
                         ws.graph.workingWidth_HL(),
                         "HLC",
                         true,
                         true,
                         false );

                //@todo should we also add this connection as adjcent?
                //getGraphData(ws.graph).adjacentTrackIds_HL[vp_from.route_point.track_id].insert(vp_to.route_point.track_id);
            }
        }
    }

}

void GraphBuilder_TracksBased::connectInnerFieldTracksToHeadland(BuilderWorkspace &ws, const Subfield &sf) const{


    std::unordered_set<vertex_t> connectedTrackEnds;
    double distComp = std::max(1e-3, ws.graph.workingWidth_IF());

    //go through all headlands
    for( size_t i = 0 ; i < ws.verticesHeadlandTracks.size() ; ++i ){
        Polygon HL_boundary;
        if(ws.hasSurroundingHeadland)
            HL_boundary = sf.headlands.complete.boundaries.second;
        else
            HL_boundary = sf.headlands.partial.at(i).boundary;

        const auto& hl_vts = ws.verticesHeadlandTracks.at(i);

        if(hl_vts.empty() || hl_vts.back().empty())//we will only connect to the last track (i.e. the closest to the inner field)
            continue;

        //connect track ends to headland
        for( auto& vts_ends : ws.verticesInfieldTracksEnds ){
            vertex_t vt = vts_ends.first;

            if(!vts_ends.second.isValid())//no next/previous point in track inside the IF
                continue;

            const vertex_property& v_prop = ws.graph[vt];

            if( geometry::calc_dist_to_linestring(HL_boundary.points, v_prop.route_point) > distComp )//the headland is not adjascent to the point
                continue;

            if( connectToClosestValidVertex(ws,
                                            vt,
                                            v_prop,
                                            v_prop.route_point,
                                            hl_vts.back(),
                                            std::bind( isValid_trackEnds, std::placeholders::_1, v_prop, vts_ends.second, 2*distComp),
                                            true,
                                            EdgeType::DEFAULT,
                                            "IFte-HL") ){
                //ws.verticesInfieldTracks_additionalConnection.erase(vt);//in case it was in the list
                connectedTrackEnds.insert(vt);
            }

        }

    }

    //connect vertices that need additional connection
    if(!ws.verticesInfieldTracks_additionalConnection.empty()){
        std::unordered_set<vertex_t> connectedVts;
        std::vector<DirectedGraph::vertex_t> hl_vts_all;        
        std::vector< std::vector<DirectedGraph::vertex_t> > hl_vts_ed;
        for( size_t i = 0 ; i < ws.verticesHeadlandTracks.size() ; ++i ){
            const auto& hl_vts = ws.verticesHeadlandTracks.at(i);
            if(hl_vts.empty())
                continue;
            //we will only connect to the last track (i.e. the closest to the inner field)
            hl_vts_all.insert(hl_vts_all.end(), hl_vts.back().begin(), hl_vts.back().end());
            hl_vts_ed.push_back( hl_vts.back() );
        }
        for( auto& vt : ws.verticesInfieldTracks_additionalConnection ){

            const vertex_property& v_prop = ws.graph[vt];
//            if( geometry::calc_dist_to_linestring(HL_boundary.points, v_prop.route_point) > distComp )//the headland is not adjascent to the point
//                continue;

            Point pInTrack;
            double dist;
            if(!getClosestPointInTrack(v_prop.route_point, sf.tracks, v_prop.route_point.track_id, pInTrack, dist))
                continue;

            if(v_prop.route_point.isOfType({RoutePoint::TRACK_START, RoutePoint::TRACK_END})){
                for(auto& hl_track_vts : hl_vts_ed){

                    if( connectToClosestValidVertex(ws,
                                                    vt,
                                                    v_prop,
                                                    v_prop.route_point,
                                                    hl_track_vts,
                                                    std::bind( isValid_HL, std::placeholders::_1, v_prop, pInTrack, 2*ws.graph.workingWidth_IF()),
                                                    true,
                                                    EdgeType::DEFAULT,
                                                    "IF*-HL") ){
                        connectedVts.insert(vt);
                    }
                }
            }
            else{
                if( connectToClosestValidVertex(ws,
                                                vt,
                                                v_prop,
                                                v_prop.route_point,
                                                hl_vts_all,
                                                std::bind( isValid_HL, std::placeholders::_1, v_prop, pInTrack, 2*ws.graph.workingWidth_IF()),
                                                true,
                                                EdgeType::DEFAULT,
                                                "IF*-HL") ){
                    connectedVts.insert(vt);
                }
            }
        }
        for(auto& vt : connectedVts)
            ws.verticesInfieldTracks_additionalConnection.erase(vt);
    }

    //update list of IF track-point vertices that need additional connection
    for( auto& vts_ends : ws.verticesInfieldTracksEnds ){
        if( connectedTrackEnds.find(vts_ends.first) == connectedTrackEnds.end() )
            ws.verticesInfieldTracks_additionalConnection.insert(vts_ends.first);
    }
}

void GraphBuilder_TracksBased::addBoundaryVertices(BuilderWorkspace &ws, const Subfield &sf) const{

    if(sf.boundary_outer.points.empty())
        return;

    Polygon boundary = sf.boundary_outer;

    geometry::unsample_polygon(boundary);

    std::map<Point, DirectedGraph::vertex_t> partialHLVerticesOverBoundary;

    //add samples closest to headland track points
    double distCmpr = ws.graph.workingWidth_HL() > 0 ? ws.graph.workingWidth_HL() : ws.graph.workingWidth_IF();
    for( size_t i = 0 ; i < ws.verticesHeadlandTracks.size() ; ++i ){
        if(ws.verticesHeadlandTracks.at(i).empty())
            continue;

        bool isNonConnHL = ws.connectingHLsIndexes.find(i) == ws.connectingHLsIndexes.end();

        //add samples (perpendicularily) closest to headland track points from first track
        if(!ws.verticesHeadlandTracks.at(i).front().empty()){
            vertex_t vt1 = ws.verticesHeadlandTracks.at(i).front().front();
            const vertex_property& vp = ws.graph[vt1];
            double dist = geometry::calc_dist_to_linestring( sf.boundary_outer.points, vp.route_point );
            if(dist > distCmpr)//boundary is not adjascent to headland
                continue;

            if(ws.hasSurroundingHeadland){
                for( auto& vt : ws.verticesHeadlandTracks.at(i).front() ){
                    const vertex_property& vp = ws.graph[vt];
                    int ind = geometry::addSampleToGeometryClosestToPoint( boundary.points, vp.route_point, 1);
                    if(ind >= 0 && ind < boundary.points.size())
                        ws.verticesHeadlandToBoundary[vt] = boundary.points.at(ind);
                }
            }
            else{
                for(size_t j = isNonConnHL ; j+isNonConnHL < ws.verticesHeadlandTracks.at(i).front().size() ; ++j){
                    vertex_t vt, vt_next;

                    if(j+1 < ws.verticesHeadlandTracks.at(i).front().size()){
                        vt = ws.verticesHeadlandTracks.at(i).front().at(j);
                        vt_next = ws.verticesHeadlandTracks.at(i).front().at(j+1);
                    }
                    else if (j>0){
                        vt = ws.verticesHeadlandTracks.at(i).front().at(j);
                        vt_next = ws.verticesHeadlandTracks.at(i).front().at(j-1);
                    }
                    else
                        continue;

                    const vertex_property& vp = ws.graph[vt];
                    const vertex_property& vp_next = ws.graph[vt_next];
                    const Point& p0 = vp.route_point.point();
                    const Point& p1 = vp_next.route_point.point();
                    Point pRef = p0;
                    if(p0 != p1){
                        Point pRot = geometry::rotate(p0, p1, M_PI_2);
                        auto intPoints = geometry::get_intersection(boundary.points, p0, pRot, false, true, true);
                        size_t indInt = geometry::getPointIndInMinDist(intPoints, p0);
                        if(indInt < intPoints.size())
                            pRef = intPoints.at(indInt);
                    }

                    int ind = geometry::addSampleToGeometryClosestToPoint( boundary.points, pRef, 1);
                    if(ind >= 0 && ind < boundary.points.size())
                        ws.verticesHeadlandToBoundary[vt] = boundary.points.at(ind);
                }
            }

        }

        //add samples closest to headland track-end points (iif partial headlands)
        if(!ws.hasSurroundingHeadland && isNonConnHL){
            for(size_t j = 0 ; j < ws.verticesHeadlandTracks.at(i).size() ; ++j){
                const auto& track_vts = ws.verticesHeadlandTracks.at(i).at(j);
                if(track_vts.empty())
                    continue;
                vertex_t vt = ws.verticesHeadlandTracks.at(i).at(j).front();//first track point
                const vertex_property& vp = ws.graph[vt];
                int ind = geometry::addSampleToGeometryClosestToPoint( boundary.points, vp.route_point, 1);
                if(ind >= 0 && ind < boundary.points.size()){
                    if(geometry::calc_dist(vp.route_point, boundary.points.at(ind)) > 1e-2)
                        ws.verticesHeadlandToBoundary[vt] = boundary.points.at(ind);
                    else
                        partialHLVerticesOverBoundary[boundary.points.at(ind)] = vt;
                }
                if(ws.verticesHeadlandTracks.at(i).at(j).size()>1){
                    vertex_t vt = ws.verticesHeadlandTracks.at(i).at(j).back();//last track point
                    const vertex_property& vp = ws.graph[vt];
                    int ind = geometry::addSampleToGeometryClosestToPoint( boundary.points, vp.route_point, 1);
                    if( (ind >= 0 && ind < boundary.points.size())
                            || ws.connectingHLsIndexes.find(i) != ws.connectingHLsIndexes.end() ){
                        if(geometry::calc_dist(vp.route_point, boundary.points.at(ind)) > 1e-2)
                            ws.verticesHeadlandToBoundary[vt] = boundary.points.at(ind);
                        else
                            partialHLVerticesOverBoundary[boundary.points.at(ind)] = vt;
                    }
                }
            }
        }
    }

    //add samples closest to infield track points needing extra connection
    distCmpr = ws.graph.workingWidth_IF() > 0 ? ws.graph.workingWidth_IF() : ws.graph.workingWidth_HL();
    for( auto& vt : ws.verticesInfieldTracks_additionalConnection ){
        const vertex_property& vp = ws.graph[vt];
        double dist = geometry::calc_dist_to_linestring( sf.boundary_outer.points, vp.route_point );
        if(dist > distCmpr)//boundary is not adjascent to point
            continue;
        int ind = geometry::addSampleToGeometryClosestToPoint( boundary.points, vp.route_point, 1);
        if(ind >= 0 && ind < boundary.points.size())
            ws.verticesInfieldToBoundary[vt] = boundary.points.at(ind);
    }

    //add samples closest to access points
    for(auto& fap : sf.access_points){
        geometry::addSampleToGeometryClosestToPoint( boundary.points, fap, 1);
    }

    //add and connect boundary vertices
    auto createVertexProperty = [](const Point& p)->vertex_property{
        vertex_property vp;
        vp.route_point.point() = p;
        vp.route_point.time_stamp = -1;
        vp.route_point.track_id = -1;
        vp.route_point.type = RoutePoint::TRANSIT_OF;
        vp.graph_location = DirectedGraph::vertex_property::DEFAULT;
        return vp;
    };


    const auto& points = boundary.points;
    ws.verticesBoundary.reserve(points.size());

    bool closed = points.size() > 2 && points.front() == points.back();

    vertex_property prev_vp;
    vertex_t prev_vt;

    auto it_hlvt = partialHLVerticesOverBoundary.find(points.front());
    if(it_hlvt == partialHLVerticesOverBoundary.end()){
        prev_vp = createVertexProperty(points.front());
        prev_vt = ws.graph.addVertex( prev_vp );
    }
    else{
        prev_vt = it_hlvt->second;
        prev_vp = ws.graph[prev_vt];
    }


    ws.verticesBoundary.push_back(prev_vt);

    vertex_t first_vt = prev_vt;
    vertex_property first_vp = prev_vp;

    for(size_t i = 1 ; i+closed < points.size() ; ++i){
        vertex_property vp;
        vertex_t vt;

        it_hlvt = partialHLVerticesOverBoundary.find(points.at(i));
        if(it_hlvt == partialHLVerticesOverBoundary.end()){
            vp = createVertexProperty(points.at(i));
            vt = ws.graph.addVertex( vp );
        }
        else{
            vt = it_hlvt->second;
            vp = ws.graph[prev_vt];
        }


        addEdge( ws,
                 geometry::calc_dist( points.at(i-1), points.at(i) ),
                 EdgeType::BOUNDARY_CONN,
                 prev_vt, vt,
                 prev_vp.route_point, vp.route_point,
                 ws.graph.workingWidth_HL(),
                 "CBoun",
                 true,
                 true,
                 false );
        prev_vp = vp;
        prev_vt = vt;
        ws.verticesBoundary.push_back(prev_vt);
    }

    if(closed){
        addEdge( ws,
                 geometry::calc_dist( points.front(), r_at(points, 1) ),
                 EdgeType::BOUNDARY_CONN,
                 prev_vt, first_vt,
                 prev_vp.route_point, first_vp.route_point,
                 ws.graph.workingWidth_HL(),
                 "CBoun",
                 true,
                 true,
                 false );
    }

    getGraphData(ws.graph).boundary_vts.insert(ws.verticesBoundary.begin(), ws.verticesBoundary.end());
}

void GraphBuilder_TracksBased::connectInnerFieldTracksToBoundary(BuilderWorkspace &ws, const Subfield &sf) const{

    std::unordered_set<vertex_t> connectedVts;
    double distComp = std::max(1e-3, ws.graph.workingWidth_IF());

    //connect vertices that need additional connection
    for( auto& vt_it : ws.verticesInfieldToBoundary ){
        const vertex_property& v_prop = ws.graph[vt_it.first];
        if( geometry::calc_dist_to_linestring(sf.boundary_outer.points, v_prop.route_point) > distComp )//the boundary is not adjascent to the point
            continue;

        Point pInTrack;
        double dist;
        if(!getClosestPointInTrack(v_prop.route_point, sf.tracks, v_prop.route_point.track_id, pInTrack, dist))
            continue;

        if( connectToClosestValidVertex(ws,
                                        vt_it.first,
                                        v_prop,
                                        vt_it.second,
                                        ws.verticesBoundary,
                                        std::bind( isValid_perpendicular, std::placeholders::_1, v_prop, pInTrack, ws.graph.workingWidth_IF()),
                                        true,
                                        EdgeType::BOUNDARY_CONN,
                                        "IF*-Boun") ){
            connectedVts.insert(vt_it.first);
        }
    }

    for( auto& vt : connectedVts )
        ws.verticesInfieldTracks_additionalConnection.erase(vt);

}

void GraphBuilder_TracksBased::connectHeadlandTracksToBoundary(BuilderWorkspace &ws) const{

    for( auto& vt_it : ws.verticesHeadlandToBoundary ){
        const vertex_property& v_prop = ws.graph[vt_it.first];
        connectToClosestValidVertex(ws,
                                    vt_it.first,
                                    v_prop,
                                    vt_it.second,
                                    ws.verticesBoundary,
                                    [](const vertex_property&)->bool{return true;},
        true,
        EdgeType::BOUNDARY_CONN,
        "HL*-Boun");
    }

}

void GraphBuilder_TracksBased::addFieldAccessPoints(BuilderWorkspace &ws, const Subfield &sf, const std::vector<FieldAccessPoint> &field_access_points) const{
    std::map<FieldAccessPointId_t, std::pair<FieldAccessPoint, Point>> projectedFAPs;

    //slightly offset the boundary and project the access points to it adding closest samples
    Polygon boundary;
    double offsetDist = ws.graph.workingWidth_IF() > 0 ? ws.graph.workingWidth_IF() : ws.graph.workingWidth_HL();
    if(offsetDist > 0)
        offsetDist *= 0.1;
    else
        offsetDist = 1e-3;
    if( !geometry::offsetPolygon(sf.boundary_outer, boundary, offsetDist, true, 0 ) )
        boundary = sf.boundary_outer;
    geometry::correct_polygon(boundary);
    if(geometry::isPolygonValid(boundary) == geometry::PolygonValidity::INVALID_POLYGON){//do not project
        for(auto& fap : field_access_points)
            projectedFAPs[fap.id] = std::make_pair(fap, fap.point());
    }
    else{
        for(auto& fap : field_access_points){
            int ind = geometry::addSampleToGeometryClosestToPoint(boundary.points, fap, 1);
            if(ind >= 0 && ind < boundary.points.size())
                projectedFAPs[fap.id] = std::make_pair(fap, boundary.points.at(ind));
            else
                projectedFAPs[fap.id] = std::make_pair(fap, fap.point());
        }
    }

    //add vertices
    auto createVertexProperty = [](const Point& p)->vertex_property{
        vertex_property vp;
        vp.route_point.point() = p;
        vp.route_point.time_stamp = -1;
        vp.route_point.track_id = -1;
        vp.route_point.type = RoutePoint::FIELD_ENTRY;
        vp.graph_location = DirectedGraph::vertex_property::DEFAULT;
        return vp;
    };
    for(auto& it_fap : projectedFAPs){
        vertex_property vp = createVertexProperty(it_fap.second.second);//add vertex in the location of the projection
        vertex_t vt = ws.graph.addVertex(vp);
        ws.verticesAccessPoints[it_fap.first] = std::make_pair(vt, it_fap.second.first);
        getGraphData(ws.graph).accesspoint_vertex_map[it_fap.second.first] = vt;
    }
}

void GraphBuilder_TracksBased::connectFieldAccessPointsToHeadland(BuilderWorkspace &ws) const{
    double distCmpr = ws.graph.workingWidth_HL() > 0 ? ws.graph.workingWidth_HL() : ws.graph.workingWidth_IF();
    auto addAccessEdge = [this, &ws](const vertex_t vt_fap, const vertex_property& vp_fap,
            const vertex_t vt_to, const vertex_property& vp_to,
            double dist, FieldAccessPoint::AccessPointType accessType){

        if( accessType == FieldAccessPoint::AP_ENTRY_EXIT
                || accessType == FieldAccessPoint::AP_ENTRY_ONLY ){
            addEdge( ws,
                     dist,
                     EdgeType::DEFAULT,
                     vt_fap, vt_to,
                     vp_fap.route_point, vp_to.route_point,
                     ws.graph.workingWidth_HL(),
                     "AP-HL",
                     false,
                     true,
                     false );
        }

        if( accessType == FieldAccessPoint::AP_ENTRY_EXIT
                || accessType == FieldAccessPoint::AP_EXIT_ONLY ){
            addEdge( ws,
                     dist,
                     EdgeType::DEFAULT,
                     vt_to, vt_fap,
                     vp_to.route_point, vp_fap.route_point,
                     ws.graph.workingWidth_HL(),
                     "HL-AP",
                     false,
                     true,
                     false );
        }
    };

    for( size_t i = 0 ; i < ws.verticesHeadlandTracks.size() ; ++i ){
        if(ws.verticesHeadlandTracks.at(i).empty())
            continue;

        //try connecting access points
        for(auto& fap_vt_it : ws.verticesAccessPoints){
            vertex_t vt_fap = fap_vt_it.second.first;
            const vertex_property& vp_fap = ws.graph[vt_fap];

            //connect to headland vertices of the first track
            for( auto& vt : ws.verticesHeadlandTracks.at(i).front() ){
                const vertex_property& vp = ws.graph[vt];
                double dist = geometry::calc_dist(vp_fap.route_point, vp.route_point);
                if(dist > distCmpr)
                    continue;
                addAccessEdge( vt_fap, vp_fap, vt, vp, dist, fap_vt_it.second.second.accessType );
            }

            //connect to headland vertices corresponding to track-end points (iif partial headlands)
            if(!ws.hasSurroundingHeadland){
                for(size_t j = 1 ; j < ws.verticesHeadlandTracks.at(i).size() ; ++j){
                    const auto& track_vts = ws.verticesHeadlandTracks.at(i).at(j);
                    if(track_vts.empty())
                        continue;
                    vertex_t vt = ws.verticesHeadlandTracks.at(i).at(j).front();//first track point
                    const vertex_property& vp = ws.graph[vt];
                    double dist = geometry::calc_dist( vp_fap.route_point, vp.route_point );
                    if(dist < distCmpr){
                        addAccessEdge( vt_fap, vp_fap, vt, vp, dist, fap_vt_it.second.second.accessType );
                    }
                    if(ws.verticesHeadlandTracks.at(i).at(j).size()>1){
                        vt = ws.verticesHeadlandTracks.at(i).at(j).back();//last track point
                        const vertex_property& vp = ws.graph[vt];
                        double dist = geometry::calc_dist( vp_fap.route_point, vp.route_point );
                        if(dist < distCmpr){
                            addAccessEdge( vt_fap, vp_fap, vt, vp, dist, fap_vt_it.second.second.accessType );
                        }
                    }
                }
            }
        }


    }
}

void GraphBuilder_TracksBased::connectFieldAccessPointsToBoundary(BuilderWorkspace &ws) const{

    auto addAccessEdge = [this, &ws](const vertex_t vt_fap, const vertex_property& vp_fap,
            const vertex_t vt_to, const vertex_property& vp_to,
            double dist, FieldAccessPoint::AccessPointType accessType){

        if( accessType == FieldAccessPoint::AP_ENTRY_EXIT
                || accessType == FieldAccessPoint::AP_ENTRY_ONLY ){
            addEdge( ws,
                     dist,
                     EdgeType::DEFAULT,
                     vt_fap, vt_to,
                     vp_fap.route_point, vp_to.route_point,
                     ws.graph.workingWidth_HL(),
                     "AP-bound",
                     false,
                     true,
                     false );
        }

        if( accessType == FieldAccessPoint::AP_ENTRY_EXIT
                || accessType == FieldAccessPoint::AP_EXIT_ONLY ){
            addEdge( ws,
                     dist,
                     EdgeType::DEFAULT,
                     vt_to, vt_fap,
                     vp_to.route_point, vp_fap.route_point,
                     ws.graph.workingWidth_HL(),
                     "Bound-AP",
                     false,
                     true,
                     false );
        }
    };

    for(auto& fap_vt_it : ws.verticesAccessPoints){
        vertex_t vt_fap = fap_vt_it.second.first;
        const vertex_property& vp_fap = ws.graph[vt_fap];

        double dist;
        vertex_t vt_to;
        vertex_property vp_to;

        if( findClosestValidVertex(ws.graph, vp_fap.route_point, ws.verticesBoundary,
                                   [](const vertex_property&)->bool{return true;},
                                   vt_to, vp_to, &dist) ){
            addAccessEdge( vt_fap, vp_fap, vt_to, vp_to, dist, fap_vt_it.second.second.accessType );
        }

    }
}

void GraphBuilder_TracksBased::interconnectFieldAccessPoints(BuilderWorkspace &ws, const OutFieldInfo &outFieldInfo) const
{
    if( outFieldInfo.mapAccessPoint2AccessPoint().empty() )
        return;

    for(const auto &fap1_it : ws.graph.accesspoint_vertex_map()){
        for(const auto &fap2_it : ws.graph.accesspoint_vertex_map()){
            if(fap1_it == fap2_it)
                continue;
            const FieldAccessPoint& fap1 = fap1_it.first;
            const FieldAccessPoint& fap2 = fap2_it.first;

            const auto& vp1 = ws.graph[fap1_it.second];
            const auto& vp2 = ws.graph[fap2_it.second];

            edge_property edge_prop;
            edge_prop.defWidth = 0;
            edge_prop.distance = -1;//must be calculated during planning depending on the machine and the travel costs
            edge_prop.bidirectional = false;
            edge_prop.edge_type = EdgeType::FAP_TO_FAP;
            edge_prop.p0 = vp1.route_point.point();
            edge_prop.p1 = vp2.route_point.point();

            if ( outFieldInfo.size_FAP2FAP(fap1.id, fap2.id) > 0 ){
                if( outFieldInfo.getTravelCost_FAP2FAP(fap1.id, fap2.id, edge_prop.travelCosts) ){
                    if( ws.graph.addEdge(fap1_it.second, fap2_it.second, edge_prop, false) )
                        addAddedEdgeInfo(ws,
                                         "AA",
                                         vp1.route_point,
                                         vp2.route_point,
                                         edge_prop.edge_type,
                                         fap1_it.second,
                                         fap2_it.second,
                                         false);
                }
            }
        }
    }
}

void GraphBuilder_TracksBased::addResourcePoints(BuilderWorkspace &ws, const Subfield &sf, const std::vector<ResourcePoint> &resource_points, const std::vector<FieldAccessPoint> &field_access_points, const OutFieldInfo &outFieldInfo) const{

    auto connectionToFAPExists = [](ResourcePointId_t rp_id, FieldAccessPointId_t fap_id, const OutFieldInfo& outFieldInfo)->bool{
        OutFieldInfo::MapMachineTravelCosts_t tc;
        if(outFieldInfo.getTravelCost_RP2FAP(rp_id, fap_id, tc) && !tc.empty())
            return true;
        if(outFieldInfo.getTravelCost_FAP2RP(fap_id, rp_id, tc) && !tc.empty())
            return true;
        return false;
    };

    auto createVertexProperty = [&outFieldInfo](const Point& p, ResourcePointId_t id)->vertex_property{
        vertex_property vp;
        vp.route_point.point() = p;
        vp.route_point.time_stamp = -1;
        vp.route_point.track_id = -1;
        vp.route_point.type = RoutePoint::RESOURCE_POINT;
        vp.graph_location = DirectedGraph::vertex_property::DEFAULT;
        outFieldInfo.getUnloadingCosts( id, vp.unloadingCosts );
        return vp;
    };

    Polygon boundary;
    bool isBoundaryOffset = false;
    auto offsetBoundary = [&boundary, &isBoundaryOffset, &ws, &sf](){//slightly offset the boundary for potential project the resource points to it adding closest samples
        if(isBoundaryOffset)
            return;
        double offsetDist = ws.graph.workingWidth_IF() > 0 ? ws.graph.workingWidth_IF() : ws.graph.workingWidth_HL();
        if(offsetDist > 0)
            offsetDist *= 0.1;
        else
            offsetDist = 1e-3;
        if( !geometry::offsetPolygon(sf.boundary_outer, boundary, offsetDist, true, 0 ) )
            boundary = sf.boundary_outer;
        geometry::correct_polygon(boundary);
        isBoundaryOffset = true;
    };

    double distOutsideCmpr = ws.graph.workingWidth_IF() > 0 ? ws.graph.workingWidth_IF() : ws.graph.workingWidth_HL();
    if(distOutsideCmpr < 1e-9)
        distOutsideCmpr = 0.1;

    for(auto& resPt : resource_points){
        bool hasConnectionToFAP = false;
        for(auto& fap_it : ws.verticesAccessPoints){
            if( connectionToFAPExists(resPt.id, fap_it.first, outFieldInfo) ){
                hasConnectionToFAP = true;
                break;
            }
        }

        if(hasConnectionToFAP){//check which resource points have outfield info. If they have, treat the res. point as outside of the field
            Point pProj = resPt.point();
            if(geometry::in_polygon(resPt, sf.boundary_outer)){//project the res. point outside of the field
                offsetBoundary();
                int ind = geometry::addSampleToGeometryClosestToPoint(boundary.points, resPt, 1);
                if(ind >= 0 && ind < boundary.points.size())
                    pProj = boundary.points.at(ind);
            }
            vertex_property vp = createVertexProperty(pProj, resPt.id);
            vertex_t vt = ws.graph.addVertex(vp);
            ws.verticesResourcePoints_FAPConn.emplace_back( std::make_pair(vt, resPt) );
            ws.graph.resourcepoint_vertex_map()[resPt] = vt;
            continue;
        }

        if( !geometry::in_polygon(resPt, sf.boundary_outer) ){
            auto dist = geometry::calc_dist_to_linestring(sf.boundary_outer.points, resPt);
            if(dist > distOutsideCmpr)//too far from the field, do not add
                continue;
            vertex_property vp = createVertexProperty(resPt, resPt.id);
            vertex_t vt = ws.graph.addVertex(vp);
            ws.verticesResourcePoints_boundaryConn.emplace_back( std::make_pair(vt, resPt) );
            ws.graph.resourcepoint_vertex_map()[resPt] = vt;
            continue;
        }

        vertex_property vp = createVertexProperty(resPt, resPt.id);
        vertex_t vt = ws.graph.addVertex(vp);
        ws.verticesResourcePoints_fieldConn.emplace_back( std::make_pair(vt, resPt) );
        ws.graph.resourcepoint_vertex_map()[resPt] = vt;

    }
}

void GraphBuilder_TracksBased::connectResourcePoints(BuilderWorkspace &ws, const Subfield &sf, const OutFieldInfo &outFieldInfo) const{
    connectResourcePointsToAccessPoints(ws, outFieldInfo);
    connectResourcePointsToTracks(ws, sf);
    connectResourcePointsToBoundary(ws);

}

void GraphBuilder_TracksBased::connectResourcePointsToAccessPoints(BuilderWorkspace &ws, const OutFieldInfo &outFieldInfo) const{

    for(auto & vt_it : ws.verticesResourcePoints_FAPConn){
        vertex_t vt_resPt = vt_it.first;
        const vertex_property& vp_resPt = ws.graph[vt_resPt];
        auto id_resPt = vt_it.second.id;
        for(auto& fap_it : ws.verticesAccessPoints){

            vertex_t vt_fap = fap_it.second.first;
            const vertex_property& vp_fap = ws.graph[vt_fap];
            auto id_fap = fap_it.first;

            edge_property edge_prop;
            edge_prop.defWidth = 0;
            edge_prop.distance = -1;//must be calculated during planning depending on the machine

            if ( outFieldInfo.size_FAP2RP(id_fap, id_resPt) > 0 ){
                if( outFieldInfo.getTravelCost_FAP2RP(id_fap, id_resPt, edge_prop.travelCosts) ){
                    edge_prop.edge_type = EdgeType::FAP_TO_RP;
                    edge_prop.p0 = vp_fap.route_point.point();
                    edge_prop.p1 = vp_resPt.route_point.point();
                    if( ws.graph.addEdge(vt_fap, vt_resPt, edge_prop, false) )
                        addAddedEdgeInfo(ws,
                                         "AP-RP",
                                         vp_fap.route_point,
                                         vp_resPt.route_point,
                                         edge_prop.edge_type,
                                         vt_fap,
                                         vt_resPt,
                                         false);
                }
            }
            if ( outFieldInfo.size_RP2FAP(id_resPt, id_fap) > 0 ){
                if( outFieldInfo.getTravelCost_RP2FAP(id_resPt, id_fap, edge_prop.travelCosts) ){
                    edge_prop.edge_type = EdgeType::RP_TO_FAP;
                    edge_prop.p0 = vp_resPt.route_point.point();
                    edge_prop.p1 = vp_fap.route_point.point();
                    if( ws.graph.addEdge(vt_resPt, vt_fap, edge_prop, false) )
                        addAddedEdgeInfo(ws,
                                         "RP-AP",
                                         vp_resPt.route_point,
                                         vp_fap.route_point,
                                         edge_prop.edge_type,
                                         vt_resPt,
                                         vt_fap,
                                         false);
                }
            }
        }
    }
}

void GraphBuilder_TracksBased::connectResourcePointsToTracks(BuilderWorkspace &ws, const Subfield &sf) const{
    for(auto & vt_it : ws.verticesResourcePoints_fieldConn){
        vertex_t vt = vt_it.first;
        const vertex_property& vp = ws.graph[vt];

        if( geometry::in_polygon(vp.route_point, sf.boundary_inner) ){//connect to a track point from the inner field

            connectToClosestValidVertex(ws,
                                        vt,
                                        vp,
                                        vp.route_point,
                                        ws.verticesInfieldTracks,
                                        [](const vertex_property&)->bool{return true;},
            true,
            EdgeType::DEFAULT,
            "ResPt-IFT");
            continue;
        }

        //check in which headland is the res. point located
        int indHL = -1;
        std::set<size_t> indTracks;
        if(ws.hasSurroundingHeadland){
            for(size_t j = 0 ; j+1 < sf.headlands.complete.tracks.size(); ++j){
                Polygon poly1, poly2;
                poly1.points = sf.headlands.complete.tracks.at(j).points;
                poly2.points = sf.headlands.complete.tracks.at(j+1).points;

                if(j == 0 && geometry::calc_dist_to_linestring(poly1.points, vp.route_point, false) < 1e-3)
                    indTracks.insert(j);
                else if(geometry::calc_dist_to_linestring(poly2.points, vp.route_point, false) < 1e-3)
                    indTracks.insert(j+1);

                if(geometry::getGeometryLength(poly1.points) < geometry::getGeometryLength(poly2.points))
                    std::swap(poly1.points, poly2.points);
                if(!geometry::isPolygonClosed(poly2.points))
                    continue;

                if(geometry::in_polygon(vp.route_point, poly1) && !geometry::in_polygon(vp.route_point, poly2)){
                    indTracks.insert(j);
                    indTracks.insert(j+1);
                }
            }
            indHL = std::min(0, (int)ws.verticesHeadlandTracks.size() - 1);
        }
        else{
            for(size_t i = 0 ; i < sf.headlands.partial.size() ; ++i){
                if( geometry::in_polygon(vp.route_point, sf.headlands.partial.at(i).boundary) && i < ws.verticesHeadlandTracks.size() ){
                    for(size_t j = 0 ; j+1 < sf.headlands.partial.at(i).tracks.size(); ++j){
                        const Track& track = sf.headlands.partial.at(i).tracks.at(j);
                        if( (track.width > 1e-9 && geometry::calc_dist_to_linestring(track.points, vp.route_point, false) <= 2*track.width)
                                || geometry::in_polygon(vp.route_point, track.boundary))
                            indTracks.insert(j);
                    }
                    indHL = i;
                    break;
                }
            }
        }

        if(indHL < 0)//no hedland found -> abort
            continue;

        bool added = false;
        for(auto i : indTracks)
            added |= connectToClosestValidVertex(ws,
                                                 vt,
                                                 vp,
                                                 vp.route_point,
                                                 ws.verticesHeadlandTracks.at(indHL).at(i),
                                                 [](const vertex_property&)->bool{return true;},
                                                 true,
                                                 EdgeType::DEFAULT,
                                                 "ResPt-HLT");
        if(!added)
            connectToClosestValidVertex(ws,
                                        vt,
                                        vp,
                                        vp.route_point,
                                        ws.verticesHeadlandTracks.at(indHL),
                                        [](const vertex_property&)->bool{return true;},
                                        true,
                                        EdgeType::DEFAULT,
                                        "ResPt-HLT");

    }

}

void GraphBuilder_TracksBased::connectResourcePointsToBoundary(BuilderWorkspace &ws) const{
    for(auto & vt_it : ws.verticesResourcePoints_boundaryConn){
        vertex_t vt = vt_it.first;
        const vertex_property& vp = ws.graph[vt];
        connectToClosestValidVertex(ws,
                                    vt,
                                    vp,
                                    vp.route_point,
                                    ws.verticesBoundary,
                                    [](const vertex_property&)->bool{return true;},
        true,
        EdgeType::DEFAULT,
        "ResPt-Bound");

    }

}

void GraphBuilder_TracksBased::addInitialPositions(BuilderWorkspace &ws, const Subfield &sf, const OutFieldInfo &outFieldInfo, const std::map<MachineId_t, MachineDynamicInfo> &machineInitialStates) const{
    auto connectionToFAPExists = [](MachineId_t m_id, FieldAccessPointId_t fap_id, const OutFieldInfo& outFieldInfo)->bool{
        OutFieldInfo::MapMachineStateTravelCosts_t tc;
        if(outFieldInfo.getArrivalCost(fap_id, m_id, tc) && !tc.empty())
            return true;
        return false;
    };

    auto createVertexProperty = [](const Point& p)->vertex_property{
        vertex_property vp;
        vp.route_point.point() = p;
        vp.route_point.time_stamp = -1;
        vp.route_point.track_id = -1;
        vp.route_point.type = RoutePoint::INITIAL_POSITION;
        vp.graph_location = DirectedGraph::vertex_property::DEFAULT;
        return vp;
    };

    const bool assumeOutOfFieldIfArrivalDataExists = false;

    Polygon boundary_projOut;
    bool isBoundaryOffset_projOut = false;
    auto offsetBoundary_projOut = [&boundary_projOut, &isBoundaryOffset_projOut, &ws, &sf](){//slightly offset the boundary for potential projection the initial locations to it adding closest samples
        if(isBoundaryOffset_projOut)
            return;
        double offsetDist = ws.graph.workingWidth_IF() > 0 ? ws.graph.workingWidth_IF() : ws.graph.workingWidth_HL();
        if(offsetDist > 0)
            offsetDist *= 0.1;
        else
            offsetDist = 1e-3;
        if( !geometry::offsetPolygon(sf.boundary_outer, boundary_projOut, offsetDist, true, 0 ) )
            boundary_projOut = sf.boundary_outer;
        geometry::correct_polygon(boundary_projOut);
        isBoundaryOffset_projOut = true;
    };

    for(auto& m_it : machineInitialStates){
        const MachineDynamicInfo &mdi = m_it.second;
        bool hasConnectionToFAP = false;

        for(auto& fap_it : ws.verticesAccessPoints){
            if( connectionToFAPExists(m_it.first, fap_it.first, outFieldInfo) ){
                hasConnectionToFAP = true;
                break;
            }
        }

        bool insideField = geometry::in_polygon(mdi.position, sf.boundary_outer);
        bool outsideAndClose = false;

        if(!insideField){

            double distOutsideCmpr = 0.1;
            auto m_it2 = ws.machines.find(m_it.first);
            if(m_it2 != ws.machines.end()){
                distOutsideCmpr = m_it2->second.workingRadius();
                if(distOutsideCmpr < 1e-9)
                    distOutsideCmpr = 0.1;
            }

            auto dist = geometry::calc_dist_to_linestring(sf.boundary_outer.points, mdi.position);
            outsideAndClose = dist < distOutsideCmpr;

        }

        if( (assumeOutOfFieldIfArrivalDataExists || (!insideField && !outsideAndClose) )
                && hasConnectionToFAP){//check if the machine has arrival info. If it has, treat the initial point as outside of the field
            Point pProj = mdi.position;
            if(geometry::in_polygon(mdi.position, sf.boundary_outer)){//project the res. point outside of the field
                offsetBoundary_projOut();
                int ind = geometry::addSampleToGeometryClosestToPoint(boundary_projOut.points, mdi.position, 1);
                if(ind >= 0 && ind < boundary_projOut.points.size())
                    pProj = boundary_projOut.points.at(ind);
            }
            vertex_property vp = createVertexProperty(pProj);
            vertex_t vt = ws.graph.addVertex(vp);
            ws.verticesInitialPos_FAPConn.emplace_back( std::make_pair(vt, m_it.first) );
            ws.graph.initialpoint_vertex_map()[m_it.first] = vt;
            continue;
        }

        if( insideField ){ // inside field -> connect to vertices inside field
            vertex_property vp = createVertexProperty(mdi.position);
            vertex_t vt = ws.graph.addVertex(vp);
            ws.verticesInitialPos_fieldConn.emplace_back( std::make_pair(vt, m_it.first) );
            ws.graph.initialpoint_vertex_map()[m_it.first] = vt;
            continue;
        }

        //outside field and close -> connect directly to boundary vt

        vertex_property vp = createVertexProperty(mdi.position);
        vertex_t vt = ws.graph.addVertex(vp);
        ws.verticesInitialPos_boundaryConn.emplace_back( std::make_pair(vt, m_it.first) );
        ws.graph.initialpoint_vertex_map()[m_it.first] = vt;

    }

}

void GraphBuilder_TracksBased::connectInitialPositions(BuilderWorkspace &ws, const Subfield &sf, const OutFieldInfo &outFieldInfo, const std::map<MachineId_t, MachineDynamicInfo> &machineInitialStates) const{
    connectInitialPositionsToAccessPoints(ws, outFieldInfo, machineInitialStates);
    connectInitialPositionsToTracks(ws, sf);
    connectInitialPositionsToBoundary(ws);

}

void GraphBuilder_TracksBased::connectInitialPositionsToAccessPoints(BuilderWorkspace &ws, const OutFieldInfo &outFieldInfo, const std::map<MachineId_t, MachineDynamicInfo> &machineInitialStates) const{

    for(auto & vt_it : ws.verticesInitialPos_FAPConn){
        vertex_t vt_ip = vt_it.first;
        const vertex_property& vp_ip = ws.graph[vt_ip];
        auto m_id = vt_it.second;
        Machine m;
        auto m_it = ws.machines.find(m_id);
        if(m_it != ws.machines.end())
            m = m_it->second;
        const MachineDynamicInfo& mdi = machineInitialStates.at(m_id);

        OutFieldInfo::MachineBunkerState bunkerState = mdi.bunkerMass > 0.5 * m.bunker_mass ? OutFieldInfo::MACHINE_LOADED : OutFieldInfo::MACHINE_EMPTY;

        for(auto& fap_it : ws.verticesAccessPoints){

            vertex_t vt_fap = fap_it.second.first;
            const vertex_property& vp_fap = ws.graph[vt_fap];
            auto id_fap = fap_it.first;

            edge_property edge_prop;
            edge_prop.defWidth = std::max( {0.0, m.working_width, m.width} );

            if ( outFieldInfo.size_arrivalCosts(id_fap, m_id) > 0 ){
                if( outFieldInfo.getArrivalCost(id_fap, m_id, bunkerState, edge_prop.arrivalCosts) ){
                    edge_prop.edge_type = EdgeType::INIT;
                    edge_prop.p0 = vp_ip.route_point.point();
                    edge_prop.p1 = vp_fap.route_point.point();
                    edge_prop.distance = edge_prop.arrivalCosts.distance;
                    if( ws.graph.addEdge(vt_ip, vt_fap, edge_prop, false) )
                        addAddedEdgeInfo(ws,
                                         "InitOF",
                                         vp_ip.route_point,
                                         vp_fap.route_point,
                                         edge_prop.edge_type,
                                         vt_ip,
                                         vt_fap,
                                         false);
                }
            }
        }
    }
}

void GraphBuilder_TracksBased::connectInitialPositionsToTracks(BuilderWorkspace &ws, const Subfield &sf) const{

    for(auto & vt_it : ws.verticesInitialPos_fieldConn){
        vertex_t vt_ip = vt_it.first;
        const vertex_property& vp_ip = ws.graph[vt_ip];
        auto m_id = vt_it.second;
        Machine m;
        auto m_it = ws.machines.find(m_id);
        if(m_it != ws.machines.end())
            m = m_it->second;

        if( geometry::in_polygon(vp_ip.route_point, sf.boundary_inner) ){//connect to a track point from the inner field

            double radius = std::max(m.workingRadius(), 0.5 * ws.graph.workingWidth_IF());
            auto vts = ws.graph.getVerticesInRadius(vp_ip.route_point, radius,
                                                    [&ws](const vertex_t& vt, const vertex_property& )->bool{
                return ws.verticesInfieldTracks_set.find(vt) != ws.verticesInfieldTracks_set.end();
            });
            if(!vts.empty()){
                double width = std::max( {0.0, m.working_width, m.width} );
                for(auto& vt : vts){
                    const vertex_property& vp = ws.graph[vt];
                    addEdge( ws,
                             geometry::calc_dist(vp_ip.route_point, vp.route_point),
                             EdgeType::INIT,
                             vt_ip, vt,
                             vp_ip.route_point, vp.route_point,
                             width,
                             "InitIF",
                             false,
                             true,
                             false );
                }
                continue;
            }

            connectToClosestValidVertex(ws,
                                        vt_ip,
                                        vp_ip,
                                        vp_ip.route_point,
                                        ws.verticesInfieldTracks,
                                        [](const vertex_property&)->bool{return true;},
            false,
            EdgeType::INIT,
            "InitIF");
            continue;
        }

        //check in which headland is the machine located
        int indHL = -1;
        if(ws.hasSurroundingHeadland)
            indHL = std::min(0, (int)ws.verticesHeadlandTracks.size() - 1);
        else{
            for(size_t i = 0 ; i < sf.headlands.partial.size() ; ++i){
                if( geometry::in_polygon(vp_ip.route_point, sf.headlands.partial.at(i).boundary) && i < ws.verticesHeadlandTracks.size() ){
                    indHL = i;
                    break;
                }
            }
        }
        if(indHL < 0)//no hedland found -> abort
            continue;

        double radius = std::max(m.workingRadius(), 0.5 * ws.graph.workingWidth_HL());
        auto vts = ws.graph.getVerticesInRadius(vp_ip.route_point, radius,
                                                [&ws, indHL](const vertex_t& vt, const vertex_property& )->bool{
            auto it = ws.verticesHeadlandTracks_set.find(vt);
            return it != ws.verticesHeadlandTracks_set.end() && it->second == indHL;
        });
        if(!vts.empty()){
            double width = std::max( {0.0, m.working_width, m.width} );
            for(auto& vt : vts){
                const vertex_property& vp = ws.graph[vt];
                addEdge( ws,
                         geometry::calc_dist(vp_ip.route_point, vp.route_point),
                         EdgeType::INIT,
                         vt_ip, vt,
                         vp_ip.route_point, vp.route_point,
                         width,
                         "InitHL",
                         false,
                         true,
                         false );
            }
            continue;
        }

        connectToClosestValidVertex(ws,
                                    vt_ip,
                                    vp_ip,
                                    vp_ip.route_point,
                                    ws.verticesHeadlandTracks.at(indHL),
                                    [](const vertex_property&)->bool{return true;},
        false,
        EdgeType::INIT,
        "InitHL");

    }

}

void GraphBuilder_TracksBased::connectInitialPositionsToBoundary(BuilderWorkspace &ws) const{
    for(auto & vt_it : ws.verticesInitialPos_boundaryConn){
        vertex_t vt = vt_it.first;
        const vertex_property& vp = ws.graph[vt];
        connectToClosestValidVertex(ws,
                                    vt,
                                    vp,
                                    vp.route_point,
                                    ws.verticesBoundary,
                                    [](const vertex_property&)->bool{return true;},
        false,
        EdgeType::DEFAULT,
        "InitBound");

    }

}

void GraphBuilder_TracksBased::addBaseRouteInformation(Graph & graph, const std::vector<Route> &baseWorkingRoutes, BuilderWorkspace* ws) const{
    std::map<int, RoutePoint> track_starts_hl, track_ends_hl;

    RoutePoint lastAddedRP(Point::invalidPoint());

    for(auto& route : baseWorkingRoutes){
        if(route.route_points.empty())
            continue;

        bool createVertexForFirstPoint = false;

        for(size_t i = 0 ; i < route.route_points.size() ; ++i){

            size_t ind_rp = route.route_points.size()-1-i; //in reverse so that the timestamp for the vertex is the first working timestamp

            const RoutePoint& rp = route.route_points.at(ind_rp);
            //const RoutePoint& rp = processBaseRoutesInReverse ? route.route_points.at( route.route_points.size()-1-i ) : route.route_points.at(i);
            if(!rp.isOfTypeWorking())
                continue;

            auto vts = graph.getVerticesInRadius(rp, 1e-3, [&graph, &rp](const vertex_t& vt, const vertex_property& v_prop)->bool{
                return rp.track_id == v_prop.route_point.track_id;
            });

            if(vts.empty())
                logger().printWarning(__FUNCTION__, "No vertex found for route point " + std::to_string(ind_rp) + " of route from machine " + std::to_string(route.machine_id));

            if(!graph.hasPartialHeadlands() && Track::isHeadlandTrack(rp.track_id)){
                if(rp.type == RoutePoint::TRACK_START)
                    track_starts_hl[rp.track_id] = rp;
                else if(rp.type == RoutePoint::TRACK_END)
                    track_ends_hl[rp.track_id] = rp;
            }

            if(rp.point() == lastAddedRP.point())//check if there were delays corresponding to turning. if so, leave the latest route point
                continue;
            lastAddedRP = rp;

            vertex_t vt;
            vertex_property v_prop;
            if(findClosestValidVertex(graph, rp, vts, [](const vertex_property&)->bool{return true;}, vt, v_prop, nullptr)){

//                auto it_rp = graph.routepoint_vertex_map().find(rp);
//                if(it_rp == graph.routepoint_vertex_map().end())
                {
                    vertex_property& vp = graph[vt];
                    vp.route_point = rp;
                    vp.harvester_id = route.machine_id;

                    if( !graph.hasPartialHeadlands() && Track::isHeadlandTrack(rp.track_id) && rp.type == RoutePoint::TRACK_END )//we assume that the track_end point corresponds to a closed-track and the vertex corresponds to a previous track_start
                        vp.route_point.time_stamp = -1;//we assume the point was already worked

                    if( graph.boundary_vts().find(vt) != graph.boundary_vts().end() )
                        vp.route_point.time_stamp = -1;//if the vertex is over the headland, do not add timestamp

                    graph.routepoint_vertex_map()[rp] = vt;
                }
            }
            else if (ind_rp == 0)//special case where the 1st route point does not correspond to any track points
                createVertexForFirstPoint = true;
        }
        if(createVertexForFirstPoint && route.route_points.size() > 1){
            auto it_rp = graph.routepoint_vertex_map().find(route.route_points.at(1));
            if(it_rp != graph.routepoint_vertex_map().end()){
                DirectedGraph::vertex_t vt1 = it_rp->second;
                addAndConnectFirstRoutePoint(graph, ws, route.route_points.front(), route.machine_id, route.route_points.front(), &vt1);
            }
            else
                addAndConnectFirstRoutePoint(graph, ws, route.route_points.front(), route.machine_id, route.route_points.front(), nullptr);
        }
    }

}

void GraphBuilder_TracksBased::addVisitPeriods(Graph &graph, const std::map<MachineId_t, Machine> &machines, const std::vector<Route> &baseWorkingRoutes) const
{
    for(auto route : baseWorkingRoutes){
        auto m_it = machines.find(route.machine_id);
        if(m_it == machines.end())
            continue;
        updateVisitPeriods(graph,
                           m_it->second,
                           route.route_points,
                           0,
                           route.route_points.size()-1);
    }
}

bool GraphBuilder_TracksBased::isValid_trackEnds(const vertex_property &vp, const vertex_property &vp_trackEnd, const Point pPrev, double maxDist){
    if( vp_trackEnd.route_point.point() == pPrev )
        return true;
    if(maxDist > 1e-6 && geometry::calc_dist(vp_trackEnd.route_point, vp.route_point) > maxDist)
        return false;
    static const double angComp = deg2rad(115);
    double ang = geometry::get_angle(pPrev, vp_trackEnd.route_point, vp.route_point);
    return std::fabs(ang) >= angComp;
}

bool GraphBuilder_TracksBased::isValid_HL(const vertex_property &vp, const vertex_property &vp_ref, const Point pAdj, double distThreshold)
{
    if( vp_ref.route_point.point() == pAdj )
        return false;
    static const double angComp = 75;//degrees
    static const double angLim1 = deg2rad(90 - angComp);
    static const double angLim2 = deg2rad(90 + angComp);
    double ang = std::fabs( geometry::get_angle(pAdj, vp_ref.route_point, vp.route_point) );
    if( ang < angLim1 || ang > angLim2 )
        return false;

    double d = arolib::geometry::calc_dist(vp_ref.route_point, vp.route_point);
    return d < distThreshold;
}

bool GraphBuilder_TracksBased::isValid_perpendicular(const vertex_property &vp, const vertex_property &vp_ref, const Point pAdj, double distThreshold){
    if( vp_ref.route_point.point() == pAdj )
        return false;
    static const double angComp = 30;//degrees
    static const double angLim1 = deg2rad(90 - angComp);
    static const double angLim2 = deg2rad(90 + angComp);
    double ang = std::fabs( geometry::get_angle(pAdj, vp_ref.route_point, vp.route_point) );
    if( ang < angLim1 || ang > angLim2 )
        return false;

    double d = arolib::geometry::calc_dist(vp_ref.route_point, vp.route_point);
    return d < distThreshold;
}

void GraphBuilder_TracksBased::addAndConnectFirstRoutePoint(Graph &graph,
                                                            BuilderWorkspace *ws,
                                                            const RoutePoint &rp,
                                                            MachineId_t machine_id,
                                                            const RoutePoint &rp_next,
                                                            vertex_t *vt_nextRP) const
{
    auto createVertexProperty = [machine_id](const RoutePoint& rp)->vertex_property{
        vertex_property vp;
        vp.route_point = rp;
        vp.harvester_id = machine_id;
        vp.graph_location = DirectedGraph::vertex_property::DEFAULT;
        return vp;
    };


    float radius = 2 * std::max( geometry::calc_dist( rp, rp_next ), Track::isHeadlandTrack(rp.track_id) ? graph.workingWidth_HL() : graph.workingWidth_IF() );

    auto vts_tmp = graph.getVerticesInRadius(rp, radius,
                                                [&rp](const vertex_t& vt, const vertex_property& vertex_prop)->bool{
        return vertex_prop.route_point.track_id >= 0 && std::fabs(vertex_prop.route_point.track_id - rp.track_id) <= 1;
    });

    std::map< int, std::pair<vertex_t, vertex_property> > vts;
    std::map< int, double > minDists;
    minDists[rp.track_id-1] = std::numeric_limits<double>::max();
    minDists[rp.track_id] = std::numeric_limits<double>::max();
    minDists[rp.track_id+1] = std::numeric_limits<double>::max();
    for(auto vt_tmp : vts_tmp){
        vertex_property vt_prop = graph[vt_tmp];
        if( vt_nextRP && vt_tmp == *vt_nextRP )
            continue;

        if( vt_prop.route_point.track_id < rp.track_id -1 || vt_prop.route_point.track_id > rp.track_id+1 )
            continue;

        double dist = geometry::calc_dist( rp, vt_prop.route_point );
        if( minDists[vt_prop.route_point.track_id] > dist ){
            vts[vt_prop.route_point.track_id] = std::make_pair(vt_tmp, vt_prop);
            minDists[vt_prop.route_point.track_id] = dist;
        }
    }

    vertex_property vp = createVertexProperty(rp);
    vertex_t vt = graph.addVertex( vp );
    graph.routepoint_vertex_map()[rp] = vt;

    if(vt_nextRP){
        if(ws)
            addEdge( *ws,
                     geometry::calc_dist( rp, rp_next ),
                     EdgeType::DEFAULT,
                     vt, *vt_nextRP,
                     rp, rp_next,
                     Track::isHeadlandTrack(rp.track_id) ? ws->graph.workingWidth_HL() : ws->graph.workingWidth_IF(),
                     "CBR",
                     false,
                     true,
                     false );
        else
            addEdge( graph,
                     geometry::calc_dist( rp, rp_next ),
                     EdgeType::DEFAULT,
                     vt, *vt_nextRP,
                     rp, rp_next,
                     Track::isHeadlandTrack(rp.track_id) ? graph.workingWidth_HL() : graph.workingWidth_IF(),
                     false,
                     true,
                     false );
    }

    for(auto & it_vt : vts){
        vertex_t vt2 = it_vt.second.first;
        const vertex_property& vp2 = it_vt.second.second;
        if(ws)
            addEdge( *ws,
                     geometry::calc_dist( rp, vp2.route_point ),
                     EdgeType::DEFAULT,
                     vt2, vt,
                     vp2.route_point, rp,
                     Track::isHeadlandTrack(rp.track_id) ? ws->graph.workingWidth_HL() : ws->graph.workingWidth_IF(),
                     "CBR",
                     false,
                     true,
                     false );
        else
            addEdge( graph,
                     geometry::calc_dist( rp, vp2.route_point ),
                     EdgeType::DEFAULT,
                     vt2, vt,
                     vp2.route_point, rp,
                     Track::isHeadlandTrack(rp.track_id) ? graph.workingWidth_HL() : graph.workingWidth_IF(),
                     false,
                     true,
                     false );
    }

}

bool GraphBuilder_TracksBased::findClosestValidVertex(Graph &graph, const Point &pRef, const std::vector<vertex_t> &vts, std::function<bool (const vertex_property &)> isValid, vertex_t &vt_out, vertex_property &vp_out, double *_minDist) const{
    double minDistTmp;
    double *minDist = _minDist ? _minDist : &minDistTmp;
    *minDist = std::numeric_limits<double>::max();
    bool found = false;

    for(auto& vt : vts){
        const vertex_property& vp = graph[vt];

        if(!isValid(vp))
            continue;

        double d = geometry::calc_dist(pRef, vp.route_point);
        if(d < *minDist){
            *minDist = d;
            vt_out = vt;
            vp_out = vp;
            found = true;
        }
    }
    return found;

}

bool GraphBuilder_TracksBased::connectToClosestValidVertex(BuilderWorkspace &ws, vertex_t vt_from, const vertex_property &vp_from, const Point &pRef, const std::vector<vertex_t> &vts, std::function<bool (const vertex_property &)> isValid, bool bidirectional, EdgeType edgeType, std::string description) const{

    vertex_t vt_to;
    vertex_property vp_to;
    double dist;
    bool found = findClosestValidVertex(ws.graph, pRef, vts, isValid, vt_to, vp_to, &dist);
    if(found){
        found = addEdge( ws,
                         geometry::calc_dist(vp_from.route_point, vp_to.route_point),
                         edgeType,
                         vt_from, vt_to,
                         vp_from.route_point, vp_to.route_point,
                         ws.graph.workingWidth_IF(),
                         description,
                         bidirectional,
                         true,
                         false );
    }

    return found;
}

bool GraphBuilder_TracksBased::connectToClosestValidVertex(BuilderWorkspace &ws, vertex_t vt_from, const vertex_property &vp_from, const Point &pRef, const std::vector<std::vector<vertex_t> > &vts, std::function<bool (const vertex_property &)> isValid, bool bidirectional, EdgeType edgeType, std::string description, vertex_t *vt_closest, vertex_property *vp_closest) const{

    vertex_t vt_to, vt;
    vertex_property vp_to, vp;
    double dist;
    double minDist = std::numeric_limits<double>::max();
    bool found = false;

    for(auto& vts2 : vts){
        if( findClosestValidVertex(ws.graph, pRef, vts2, isValid, vt, vp, &dist)
                && dist < minDist){
            minDist = dist;
            vt_to = vt;
            vp_to = vp;
            found = true;
        }
    }
    if(found){
        found = addEdge( ws,
                         geometry::calc_dist(vp_from.route_point, vp_to.route_point),
                         edgeType,
                         vt_from, vt_to,
                         vp_from.route_point, vp_to.route_point,
                         ws.graph.workingWidth_IF(),
                         description,
                         bidirectional,
                         true,
                         false );
    }

    if(vt_closest)
        *vt_closest = vt_to;
    if(vp_closest)
        *vp_closest = vp_to;

    return found;
}

bool GraphBuilder_TracksBased::getClosestPointInTrack(const Point &pRef, const std::vector<Track> &tracks, int track_id, Point &p_out, double &min_dist) const
{
    bool ok = false;
    min_dist = std::numeric_limits<double>::max();
    for (const auto &track : tracks){
        if(track.id != track_id)
            continue;
        for(const auto& p : track.points){
            if(p == pRef)
                continue;
            double d = geometry::calc_dist(p, pRef);
            if (d <= min_dist) {
                min_dist = d;
                p_out = p;
                ok = true;
            }
        }
        break;
    }
    return ok;
}

bool GraphBuilder_TracksBased::addEdge(Graph &graph,
                                       double distance,
                                       EdgeType edgeType,
                                       const vertex_t &vt1,
                                       const vertex_t &vt2,
                                       const RoutePoint &rp1,
                                       const RoutePoint &rp2,
                                       double width,
                                       bool bidirectional,
                                       bool onlyIfNotExisting,
                                       bool overwrite) const
{
    /// create edge
    edge_property edge_prop;
    edge_prop.defWidth = width;
    edge_prop.distance = distance;
    edge_prop.edge_type = edgeType;

    edge_prop.p0 = rp1.point();
    edge_prop.p1 = rp2.point();

    return graph.addEdge(vt1, vt2, edge_prop, bidirectional, onlyIfNotExisting, overwrite);

}

bool GraphBuilder_TracksBased::addEdge(BuilderWorkspace &ws,
                                       double distance,
                                       EdgeType edgeType,
                                       const vertex_t &vt1,
                                       const vertex_t &vt2,
                                       const RoutePoint &rp1,
                                       const RoutePoint &rp2,
                                       double width,
                                       const std::string &addedTag,
                                       bool bidirectional,
                                       bool onlyIfNotExisting,
                                       bool overwrite) const
{
    bool edgeAdded = addEdge(ws.graph,
                             distance, edgeType,
                             vt1, vt2, rp1, rp2,
                             width, bidirectional, onlyIfNotExisting, overwrite);
    if( edgeAdded )
        addAddedEdgeInfo(ws,
                         addedTag,
                         rp1,
                         rp2,
                         edgeType,
                         vt1,
                         vt2,
                         bidirectional);

    return edgeAdded;

}

void GraphBuilder_TracksBased::addAddedEdgeInfo(BuilderWorkspace &ws,
                                                const std::string &added_desc,
                                                const RoutePoint &p0,
                                                const RoutePoint &p1,
                                                EdgeType edge_type,
                                                const vertex_t &vt_from,
                                                const vertex_t &vt_to,
                                                bool bidirectional) const
{
    if(ws.outputFile.empty())
        return;
    ws.buildingInfoManager.addEdgeInfo( GraphBuildingInfoManager::EdgeInfo(added_desc, edge_type, vt_from, vt_to, p0, p1, p0.type, p1.type, bidirectional) );
}

void GraphBuilder_TracksBased::flushOutputBuffer(BuilderWorkspace &ws) const
{
    if(ws.outputFile.empty())
        return;
    ws.buildingInfoManager.saveEdgesInfo(ws.outputFile);
}



}
}
