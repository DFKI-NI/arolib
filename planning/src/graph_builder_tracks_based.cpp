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
 
#include "arolib/planning/graph_builder_tracks_based.hpp"

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
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error offsetting inner boundary");
    }

    addInnerFieldTracks(ws, subfield);
    interconnectInnerFieldTracks(ws, subfield);
    addHeadlandTracks(ws, subfield);
    interconnectHeadlandTracks(ws, subfield.boundary_outer);
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
    addBaseRouteInformation(ws, baseWorkingRoutes);
    if(includeVisitPeriods)
        addVisitPeriods(ws, baseWorkingRoutes);

    flushOutputBuffer(ws);

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Graph expanded!" );

    return ws.graph;

}

void GraphBuilder_TracksBased::addInnerFieldTracks(BuilderWorkspace &ws, const Subfield &sf) const
{
    double workingWidth = std::max(1e-2, ws.graph.workingWidth_HL() > 0 ? ws.graph.workingWidth_HL() : ws.graph.workingWidth_IF() );

    Polygon outterBoundaryEd;
    if(!geometry::offsetPolygon(sf.boundary_outer, outterBoundaryEd, 0.01, true, 0)){
        outterBoundaryEd = sf.boundary_outer;
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error offsetting outter boundary");
    }
    geometry::correct_polygon(outterBoundaryEd);
    if(geometry::isPolygonValid(outterBoundaryEd) == geometry::PolygonValidity::INVALID_POLYGON)
        outterBoundaryEd.points.clear();

    Polygon innerBoundaryEd;
    if( !geometry::offsetPolygon(sf.boundary_inner, innerBoundaryEd, workingWidth, true) ){
        innerBoundaryEd = sf.boundary_inner;
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error offsetting inner boundary");
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

    if(!sf.tracks.empty()){//@todo this assumes that the tracks ids are in order with respect the the vector
        getGraphData(ws.graph).minTrackId_IF = sf.tracks.front().id;
        getGraphData(ws.graph).maxTrackId_IF = sf.tracks.back().id;
        if(getGraphData(ws.graph).minTrackId_IF > getGraphData(ws.graph).maxTrackId_IF)
            std::swap(getGraphData(ws.graph).minTrackId_IF, getGraphData(ws.graph).maxTrackId_IF);
    }

    for(auto& track : ws.verticesInfieldTracks){
        ws.verticesInfieldTracks_set.insert(track.begin(), track.end());
    }
}

void GraphBuilder_TracksBased::interconnectInnerFieldTracks(BuilderWorkspace &ws, const Subfield &sf) const
{
    struct connectionProp{
        vertex_t vertex;
        RoutePoint p0;
        RoutePoint p1;
    };
    std::map<vertex_t, connectionProp> connectedVertices_prev;

    Polygon outterBoundaryEd;
//    if(!geometry::offsetPolygon(sf.boundary_outer, outterBoundaryEd, 0.01, true, 0))
//        outterBoundaryEd = sf.boundary_outer;
//    geometry::correct_polygon(outterBoundaryEd);
//    if(geometry::isPolygonValid(outterBoundaryEd) == geometry::PolygonValidity::INVALID_POLYGON)
//        outterBoundaryEd.points.clear();

    //connect (if possible/suitable) every IF track-point-vertex to the closest vertex in the next and previous track
    int countSwipes = 0;
    while (countSwipes < 2){//swipe first from 'lower' to 'highest' track, and then reverse for unconnected vertices
        int ind = (countSwipes < 1 ? 0 : ws.verticesInfieldTracks.size()-1);
        int deltaInd = (countSwipes < 1 ? 1 : -1);
        for( ; (countSwipes < 1 ? ind+1 < ws.verticesInfieldTracks.size() : ind > 0 ) ; ind += deltaInd ){

            const auto &track_from = ws.verticesInfieldTracks.at(ind);
            const auto &track_to = ws.verticesInfieldTracks.at(ind + deltaInd);

            for (auto &vt_from : track_from) {
                const vertex_property& vp_from = ws.graph[vt_from];

                if(!outterBoundaryEd.points.empty() && !geometry::in_polygon(vp_from.route_point, outterBoundaryEd))
                    continue;

                //                    if(vp_from.route_point.type == RoutePoint::TRACK_END)
                //                        continue;

                double min_dist = std::numeric_limits<double>::max();
                vertex_t vt_to;
                vertex_property vp_to;
                bool found = false;

                //get the vertex in next track closest to the current vertex
                for (auto &vt : track_to) {
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

                if(!outterBoundaryEd.points.empty() && !geometry::in_polygon(vp_to.route_point, outterBoundaryEd))
                    continue;

                if(countSwipes > 0){//check if that same edge was alredy created
                    auto itConnPrev = connectedVertices_prev.find(vt_from);
                    if(itConnPrev != connectedVertices_prev.end()
                            && itConnPrev->second.vertex == vt_to)
                        continue;//already connected to that vertex in first (fwd) swipe
                }

                //                if( ( rp_next.type == RoutePoint::TRACK_START || rp_next.type == RoutePoint::TRACK_END )
                //                        && min_dist > 1.5 * ws.graph.workingWidth_IF() ){//connect to TRACK_START and TRACK_END vertices from the next/previous track iif they are not too far away from the vertex. If they are too far, it might be better to connect it to the headland (later on)
                if(min_dist > 1.5 * ws.graph.workingWidth_IF()){ //do not connect if they are far
                    //if(countSwipes == 0)
                    ws.verticesInfieldTracks_additionalConnection.insert(vt_from);
                    continue;
                }

                addEdge( ws,
                         min_dist,
                         EdgeType::CROSS,
                         vt_from,
                         vt_to,
                         vp_from.route_point,
                         vp_to.route_point,
                         getGraphData(ws.graph).workingWidth_IF,
                         (countSwipes == 0 ? "CITf" : "CITb"),
                         true,
                         true,
                         false );

                connectionProp connProp;
                if(countSwipes == 0){
                    connProp.vertex = vt_from;
                    connProp.p0 = vp_to.route_point;
                    connProp.p1 = vp_from.route_point;
                    connectedVertices_prev[vt_to] = connProp;
                }
                else{
                    connProp.vertex = vt_to;
                    connProp.p0 = vp_from.route_point;
                    connProp.p1 = vp_to.route_point;
                    connectedVertices_prev[vt_from] = connProp;
                }
            }

        }
        ++countSwipes;
    }

    //add vertices from first and last track to list of verticess needing extra connections
    if(!ws.verticesInfieldTracks.empty()){
        ws.verticesInfieldTracks_additionalConnection.insert( ws.verticesInfieldTracks.front().begin(), ws.verticesInfieldTracks.front().end() );
    }
    if(ws.verticesInfieldTracks.size() > 1){
        ws.verticesInfieldTracks_additionalConnection.insert( ws.verticesInfieldTracks.back().begin(), ws.verticesInfieldTracks.back().end() );
    }
}

void GraphBuilder_TracksBased::addHeadlandTracks(BuilderWorkspace &ws, const Subfield &sf) const
{
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
        ws.verticesHeadlandTracks.at(headlandIdx).back().push_back(prev_vt);

        vertex_t first_vt = prev_vt;
        vertex_property first_vp = prev_vp;

        for(size_t i = 1 ; i+closed < points.size() ; ++i){
            vertex_property vp = createVertexProperty(points.at(i));
            vertex_t vt = ws.graph.addVertex( vp );
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
        ws.verticesHeadlandTracks.resize(1);
        if(!headlands.complete.tracks.empty()){
            for( auto& track : headlands.complete.tracks )
                addTrack(track.points, track.id, 0);

            //@todo this assumes that the tracks ids are in order with respect the the vector
            getGraphData(ws.graph).minTrackId_HL = headlands.complete.tracks.front().id;
            getGraphData(ws.graph).maxTrackId_HL = headlands.complete.tracks.back().id;
            if(getGraphData(ws.graph).minTrackId_HL > getGraphData(ws.graph).maxTrackId_HL)
                std::swap(getGraphData(ws.graph).minTrackId_HL, getGraphData(ws.graph).maxTrackId_HL);
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

    for(size_t i = 0 ; i < ws.verticesHeadlandTracks.size() ; ++i){
        for(auto& track : ws.verticesHeadlandTracks.at(i)){
            for(auto& vt : track){
                ws.verticesHeadlandTracks_set[vt] = i;
            }
        }
    }

}

void GraphBuilder_TracksBased::interconnectHeadlandTracks(BuilderWorkspace &ws, const Polygon &boundary_) const
{
    struct connectionProp{
        vertex_t vertex;
        RoutePoint p0;
        RoutePoint p1;
    };

    if(ws.verticesHeadlandTracks.empty())
        return;

    Polygon boundary;
    if( !geometry::offsetPolygon(boundary_, boundary, ws.graph.workingWidth_HL() * 1e-3, true, 0) )//offset the boundary a bit to avoid intersections with the first and last track points of partial headlands
        boundary.points.clear();

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

                    if( addEdge( ws,
                                 min_dist,
                                 EdgeType::CROSS,
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
            break;

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
                                            std::bind( isValid_trackEnds, std::placeholders::_1, v_prop, vts_ends.second),
                                            true,
                                            EdgeType::DEFAULT,
                                            "IFte-HL") ){
                //ws.verticesInfieldTracks_additionalConnection.erase(vt);//in case it was in the list
                connectedTrackEnds.insert(vt);
            }

        }

        //connect vertices that need additional connection
        std::unordered_set<vertex_t> connectedVts;
        for( auto& vt : ws.verticesInfieldTracks_additionalConnection ){
            const vertex_property& v_prop = ws.graph[vt];
            if( geometry::calc_dist_to_linestring(HL_boundary.points, v_prop.route_point) > distComp )//the headland is not adjascent to the point
                continue;

            Point pInTrack;
            double dist;
            if(!getClosestPointInTrack(v_prop.route_point, sf.tracks, v_prop.route_point.track_id, pInTrack, dist))
                continue;

            if( connectToClosestValidVertex(ws,
                                            vt,
                                            v_prop,
                                            v_prop.route_point,
                                            hl_vts.back(),
                                            std::bind( isValid_HL, std::placeholders::_1, v_prop, pInTrack, 2*ws.graph.workingWidth_IF()),
                                            true,
                                            EdgeType::DEFAULT,
                                            "IF*-HL") ){
                connectedVts.insert(vt);
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


    //add samples closest to headland track points
    double distCmpr = ws.graph.workingWidth_HL() > 0 ? ws.graph.workingWidth_HL() : ws.graph.workingWidth_IF();
    for( size_t i = 0 ; i < ws.verticesHeadlandTracks.size() ; ++i ){
        if(ws.verticesHeadlandTracks.at(i).empty())
            continue;

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
                for(size_t j = 1 ; j+1 < ws.verticesHeadlandTracks.at(i).front().size() ; ++j){
                    const vertex_t& vt = ws.verticesHeadlandTracks.at(i).front().at(j);
                    const vertex_t& vt_next = ws.verticesHeadlandTracks.at(i).front().at(j+1);
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
        if(!ws.hasSurroundingHeadland){
            for(size_t j = 0 ; j < ws.verticesHeadlandTracks.at(i).size() ; ++j){
                const auto& track_vts = ws.verticesHeadlandTracks.at(i).at(j);
                if(track_vts.empty())
                    continue;
                vertex_t vt = ws.verticesHeadlandTracks.at(i).at(j).front();//first track point
                const vertex_property& vp = ws.graph[vt];
                double dist = geometry::calc_dist_to_linestring( sf.boundary_outer.points, vp.route_point );
                if(dist < distCmpr * 0.01){//boundary is adjascent to headland
                    int ind = geometry::addSampleToGeometryClosestToPoint( boundary.points, vp.route_point, 1);
                    if(ind >= 0 && ind < boundary.points.size())
                        ws.verticesHeadlandToBoundary[vt] = boundary.points.at(ind);
                }
                if(ws.verticesHeadlandTracks.at(i).at(j).size()>1){
                    vt = ws.verticesHeadlandTracks.at(i).at(j).back();//last track point
                    const vertex_property& vp = ws.graph[vt];
                    dist = geometry::calc_dist_to_linestring( sf.boundary_outer.points, vp.route_point );
                    if(dist < distCmpr * 0.01){//boundary is adjascent to headland
                        int ind = geometry::addSampleToGeometryClosestToPoint( boundary.points, vp.route_point, 1);
                        if(ind >= 0 && ind < boundary.points.size())
                            ws.verticesHeadlandToBoundary[vt] = boundary.points.at(ind);
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
        vp.route_point.type = RoutePoint::HEADLAND;
        vp.graph_location = DirectedGraph::vertex_property::DEFAULT;
        return vp;
    };


    const auto& points = boundary.points;
    ws.verticesBoundary.reserve(points.size());

    bool closed = points.size() > 2 && points.front() == points.back();

    vertex_property prev_vp = createVertexProperty(points.front());
    vertex_t prev_vt = ws.graph.addVertex( prev_vp );
    ws.verticesBoundary.push_back(prev_vt);

    vertex_t first_vt = prev_vt;
    vertex_property first_vp = prev_vp;

    for(size_t i = 1 ; i+closed < points.size() ; ++i){
        vertex_property vp = createVertexProperty(points.at(i));
        vertex_t vt = ws.graph.addVertex( vp );
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

        if( findClosestValidVertex(ws, vp_fap.route_point, ws.verticesBoundary,
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

            edge_property edge_prop;
            edge_prop.defWidth = 0;
            edge_prop.distance = -1;//must be calculated during planning depending on the machine and the travel costs
            edge_prop.bidirectional = false;
            edge_prop.edge_type = EdgeType::FAP_TO_FAP;

            if ( outFieldInfo.size_FAP2FAP(fap1.id, fap2.id) > 0 ){
                if( outFieldInfo.getTravelCost_FAP2FAP(fap1.id, fap2.id, edge_prop.travelCosts) ){
                    if( ws.graph.addEdge(fap1_it.second, fap2_it.second, edge_prop, false) )
                        addAddedEdgeInfo(ws,
                                         "AA",
                                         ws.graph[fap1_it.second].route_point,
                                         ws.graph[fap2_it.second].route_point,
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

        if(hasConnectionToFAP){//check which resource points have outfield info. If the have, treat the res. point as outside of the field
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
        if(ws.hasSurroundingHeadland)
            indHL = std::min(0, (int)ws.verticesHeadlandTracks.size() - 1);
        if(indHL < 0)//no headland found -> abort
            continue;

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

    Polygon boundary;
    bool isBoundaryOffset = false;
    auto offsetBoundary = [&boundary, &isBoundaryOffset, &ws, &sf](){//slightly offset the boundary for potential projection the initial locations to it adding closest samples
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

    for(auto& m_it : machineInitialStates){
        const MachineDynamicInfo &mdi = m_it.second;
        bool hasConnectionToFAP = false;

        for(auto& fap_it : ws.verticesAccessPoints){
            if( connectionToFAPExists(m_it.first, fap_it.first, outFieldInfo) ){
                hasConnectionToFAP = true;
                break;
            }
        }

        if(hasConnectionToFAP){//check if the machine has arrival info. If it has, treat the initial point as outside of the field
            Point pProj = mdi.position;
            if(geometry::in_polygon(mdi.position, sf.boundary_outer)){//project the res. point outside of the field
                offsetBoundary();
                int ind = geometry::addSampleToGeometryClosestToPoint(boundary.points, mdi.position, 1);
                if(ind >= 0 && ind < boundary.points.size())
                    pProj = boundary.points.at(ind);
            }
            vertex_property vp = createVertexProperty(pProj);
            vertex_t vt = ws.graph.addVertex(vp);
            ws.verticesInitialPos_FAPConn.emplace_back( std::make_pair(vt, m_it.first) );
            ws.graph.initialpoint_vertex_map()[m_it.first] = vt;
            continue;
        }

        if( !geometry::in_polygon(mdi.position, sf.boundary_outer) ){
            double distOutsideCmpr = 0.1;
            auto m_it2 = ws.machines.find(m_it.first);
            if(m_it2 != ws.machines.end()){
                distOutsideCmpr = m_it2->second.workingRadius();
                if(distOutsideCmpr < 1e-9)
                    distOutsideCmpr = 0.1;
            }

            auto dist = geometry::calc_dist_to_linestring(sf.boundary_outer.points, mdi.position);
            if(dist > distOutsideCmpr)//too far from the field, do not add
                continue;
            vertex_property vp = createVertexProperty(mdi.position);
            vertex_t vt = ws.graph.addVertex(vp);
            ws.verticesInitialPos_boundaryConn.emplace_back( std::make_pair(vt, m_it.first) );
            ws.graph.initialpoint_vertex_map()[m_it.first] = vt;
            continue;
        }

        vertex_property vp = createVertexProperty(mdi.position);
        vertex_t vt = ws.graph.addVertex(vp);
        ws.verticesInitialPos_fieldConn.emplace_back( std::make_pair(vt, m_it.first) );
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
        if(indHL < 0)//no headland found -> abort
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

void GraphBuilder_TracksBased::addBaseRouteInformation(BuilderWorkspace &ws, const std::vector<Route> &baseWorkingRoutes) const{
    std::map<int, RoutePoint> track_starts_hl, track_ends_hl;
    for(auto& route : baseWorkingRoutes){

        bool createVertexForFirstPoint = false;

        for(size_t i = 0 ; i < route.route_points.size() ; ++i){
            const RoutePoint& rp = route.route_points.at(i);
            //const RoutePoint& rp = processBaseRoutesInReverse ? route.route_points.at( route.route_points.size()-1-i ) : route.route_points.at(i);
            if(!rp.isOfTypeWorking())
                continue;
            auto vts = ws.graph.getVerticesInRadius(rp, 1e-3, [&ws, &rp](const vertex_t& vt, const vertex_property& v_prop)->bool{
                return rp.track_id == v_prop.route_point.track_id &&
                        ( ws.verticesInfieldTracks_set.find(vt) != ws.verticesInfieldTracks_set.end() ||
                        ws.verticesHeadlandTracks_set.find(vt) != ws.verticesHeadlandTracks_set.end() );
            });

            if(vts.empty())
                m_logger.printWarning(__FUNCTION__, "No vertex found for route point " + std::to_string(i) + " of route from machine " + std::to_string(route.machine_id));

            vertex_t vt;
            vertex_property v_prop;
            if(findClosestValidVertex(ws, rp, vts, [](const vertex_property&)->bool{return true;}, vt, v_prop, nullptr)){
                auto it_rp = ws.graph.routepoint_vertex_map().find(rp);
                if(it_rp == ws.graph.routepoint_vertex_map().end()){
                    vertex_property& vp = ws.graph[vt];
                    vp.route_point = rp;
                    vp.harvester_id = route.machine_id;

                    if( Track::isHeadlandTrack(rp.track_id) && rp.type == RoutePoint::TRACK_END )//we assume that the track_end point corresponds to a closed-track and the vertex corresponds to a previous track_start
                        vp.route_point.time_stamp = -1;//we assume the point was already worked

                    ws.graph.routepoint_vertex_map()[rp] = vt;
                }

                if(createVertexForFirstPoint && i == 1)
                    addAndConnectFirstRoutePoint(ws, route.route_points.front(), route.machine_id, rp, &vt);
            }
            else if (i == 0)//special case where the 1st route point does not correspond to any track points
                createVertexForFirstPoint = true;
            else if(createVertexForFirstPoint && i == 1)
                addAndConnectFirstRoutePoint(ws, route.route_points.front(), route.machine_id, rp, nullptr);

            if(Track::isHeadlandTrack(rp.track_id)){
                if(rp.type == RoutePoint::TRACK_START)
                    track_starts_hl[rp.track_id] = rp;
                else if(rp.type == RoutePoint::TRACK_END)
                    track_ends_hl[rp.track_id] = rp;
            }
        }
    }

}

void GraphBuilder_TracksBased::addVisitPeriods(BuilderWorkspace &ws, const std::vector<Route> &baseWorkingRoutes) const
{
    for(auto route : baseWorkingRoutes){
        auto m_it = ws.machines.find(route.machine_id);
        if(m_it == ws.machines.end())
            continue;
        updateVisitingPeriods(ws.graph,
                              m_it->second,
                              route.route_points,
                              0,
                              route.route_points.size()-1);
    }
}

bool GraphBuilder_TracksBased::isValid_trackEnds(const vertex_property &vp, const vertex_property &vp_trackEnd, const Point pPrev){
    if( vp_trackEnd.route_point.point() == pPrev )
        return true;
    static const double angComp = deg2rad(45);
    double ang = geometry::get_angle(vp_trackEnd.route_point, pPrev, vp.route_point);
    return std::fabs(ang) <= angComp;
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

void GraphBuilder_TracksBased::addAndConnectFirstRoutePoint(GraphBuilder_TracksBased::BuilderWorkspace &ws, const RoutePoint &rp, MachineId_t machine_id, const RoutePoint &rp_next, vertex_t *vt_nextRP) const
{
    auto createVertexProperty = [machine_id](const RoutePoint& rp)->vertex_property{
        vertex_property vp;
        vp.route_point = rp;
        vp.harvester_id = machine_id;
        vp.graph_location = DirectedGraph::vertex_property::DEFAULT;
        return vp;
    };


    float radius = 2 * std::max( geometry::calc_dist( rp, rp_next ), Track::isHeadlandTrack(rp.track_id) ? ws.graph.workingWidth_HL() : ws.graph.workingWidth_IF() );

    auto vts_tmp = ws.graph.getVerticesInRadius(rp, radius,
                                                [&rp](const vertex_t& vt, const vertex_property& vertex_prop)->bool{
        return vertex_prop.route_point.track_id >= 0 && std::fabs(vertex_prop.route_point.track_id - rp.track_id) <= 1;
    });

    std::map< int, std::pair<vertex_t, vertex_property> > vts;
    std::map< int, double > minDists;
    minDists[rp.track_id-1] = std::numeric_limits<double>::max();
    minDists[rp.track_id] = std::numeric_limits<double>::max();
    minDists[rp.track_id+1] = std::numeric_limits<double>::max();
    for(auto vt_tmp : vts_tmp){
        vertex_property vt_prop = ws.graph[vt_tmp];
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
    vertex_t vt = ws.graph.addVertex( vp );
    ws.graph.routepoint_vertex_map()[rp] = vt;

    if(vt_nextRP)
        addEdge( ws,
                 geometry::calc_dist( rp, rp_next ),
                 EdgeType::DEFAULT,
                 vt, *vt_nextRP,
                 rp, rp_next,
                 Track::isHeadlandTrack(rp.track_id) ? ws.graph.workingWidth_HL() : ws.graph.workingWidth_IF(),
                 "CBR",
                 false,
                 true,
                 false );

    for(auto & it_vt : vts){
        vertex_t vt2 = it_vt.second.first;
        const vertex_property& vp2 = it_vt.second.second;
        addEdge( ws,
                 geometry::calc_dist( rp, vp2.route_point ),
                 EdgeType::DEFAULT,
                 vt2, vt,
                 vp2.route_point, rp,
                 Track::isHeadlandTrack(rp.track_id) ? ws.graph.workingWidth_HL() : ws.graph.workingWidth_IF(),
                 "CBR",
                 false,
                 true,
                 false );
    }

}

bool GraphBuilder_TracksBased::findClosestValidVertex(BuilderWorkspace &ws, const Point &pRef, const std::vector<vertex_t> &vts, std::function<bool (const vertex_property &)> isValid, vertex_t &vt_out, vertex_property &vp_out, double *_minDist) const{
    double minDistTmp;
    double *minDist = _minDist ? _minDist : &minDistTmp;
    *minDist = std::numeric_limits<double>::max();
    bool found = false;

    for(auto& vt : vts){
        const vertex_property& vp = ws.graph[vt];

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
    bool found = findClosestValidVertex(ws, pRef, vts, isValid, vt_to, vp_to, &dist);
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

bool GraphBuilder_TracksBased::connectToClosestValidVertex(BuilderWorkspace &ws, vertex_t vt_from, const vertex_property &vp_from, const Point &pRef, const std::vector<std::vector<vertex_t> > &vts, std::function<bool (const vertex_property &)> isValid, bool bidirectional, EdgeType edgeType, std::string description) const{

    vertex_t vt_to, vt;
    vertex_property vp_to, vp;
    double dist;
    double minDist = std::numeric_limits<double>::max();
    bool found = false;
    for(auto& vts2 : vts){
        if( findClosestValidVertex(ws, pRef, vts2, isValid, vt, vp, &dist)
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

bool GraphBuilder_TracksBased::addEdge(BuilderWorkspace &ws, double distance, EdgeType edgeType, const vertex_t &vt1, const vertex_t &vt2, const RoutePoint &rp1, const RoutePoint &rp2, double width, const std::string &addedTag, bool bidirectional, bool onlyIfNotExisting, bool overwrite) const
{
    /// create edge
    edge_property edge_prop;
    edge_prop.defWidth = width;
    edge_prop.distance = distance;
    edge_prop.edge_type = edgeType;

    edge_prop.p0 = rp1.point();
    edge_prop.p1 = rp2.point();

    bool edgeAdded = ws.graph.addEdge(vt1, vt2, edge_prop, bidirectional, onlyIfNotExisting, overwrite);
    if( edgeAdded )
        addAddedEdgeInfo(ws,
                         addedTag,
                         rp1,
                         rp2,
                         edge_prop.edge_type,
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
