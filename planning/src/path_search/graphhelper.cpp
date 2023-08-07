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
 
#include "arolib/planning/path_search/graphhelper.hpp"

namespace arolib {


DirectedGraph::vertex_t find_nearest_vertex(const arolib::Point &p, const arolib::DirectedGraph::Graph &graph){
    arolib::DirectedGraph::vertex_iter vp;
    double min_dist = std::numeric_limits<double>::max();
    DirectedGraph::vertex_t nearest;
    for(vp = vertices(graph); vp.first != vp.second; vp.first++){
        double d = arolib::geometry::calc_dist(p, graph[*vp.first].route_point);
        if(d < min_dist) {
            nearest = *vp.first;
            min_dist = d;
        }
    }
    return nearest;
}

bool searchNearestVertices(const DirectedGraph::Graph &graph,
                           const std::vector<DirectedGraph::vertex_t> vertexSet,
                           const DirectedGraph::vertex_t vt,
                           std::vector<DirectedGraph::vertex_t> &nearestVertices,
                           size_t maxVertices,
                           double tolerance,
                           const DirectedGraph::Graph::VertexFilterFct &validityFunction)
{
    nearestVertices.clear();
    const Point vt_point = graph[vt].route_point.point();
    double min_dist = std::numeric_limits<double>::max();
    //arolib::DirectedGraph::vertex_iter vp = vertices(graph);
    bool found = false;

    for(auto& vt2 : vertexSet){
        const auto& v_prop = graph[vt2];
        if(!validityFunction(vt2, v_prop))
            continue;
        RoutePoint rp = v_prop.route_point;
        double d = arolib::geometry::calc_dist(rp, vt_point);
        if( std::fabs(d-min_dist) < tolerance ){
            min_dist = std::min(d, min_dist);
            found = true;
        }
        else if ( d < min_dist) {
            min_dist = d;
            nearestVertices.clear();
            found = true;
        }
        else
            continue;
        if(maxVertices == 0 || nearestVertices.size() < maxVertices)
            nearestVertices.emplace_back(vt2);
    }
    return found;
}

bool searchNearestVerticesOnTrack(const DirectedGraph::Graph &graph,
                                  const DirectedGraph::vertex_t vt,
                                  const int& track_id,
                                  std::vector<DirectedGraph::vertex_t> &nearestVertices,
                                  size_t maxVertices,
                                  double tolerance,
                                  const DirectedGraph::Graph::VertexFilterFct &validityFunction) {
    nearestVertices.clear();
    const Point vt_point = graph[vt].route_point.point();
    double min_dist = std::numeric_limits<double>::max();
    bool found = false;

    auto track_vts_it = graph.tracks_vertices_map().find(track_id);
    if(track_vts_it != graph.tracks_vertices_map().end()){
        for(const auto& vt : track_vts_it->second){
            const auto& vt_prop = graph[vt];
            if(!validityFunction(vt, vt_prop))
                continue;
            double d = arolib::geometry::calc_dist(vt_prop.route_point, vt_point);
            if( std::fabs(d-min_dist) < tolerance ){
                min_dist = std::min(d, min_dist);
                found = true;
            }
            else if ( d < min_dist) {
                min_dist = d;
                nearestVertices.clear();
                found = true;
            }
            else
                continue;
            if(maxVertices == 0 || nearestVertices.size() < maxVertices)
                nearestVertices.emplace_back(vt);
        }
        return found;
    }
    if(track_id >= 0)
        return false;
    for(auto vt_it = vertices(graph); vt_it.first != vt_it.second; vt_it.first++){
        const auto& vt_prop = graph[*vt_it.first];
        if(!validityFunction(*vt_it.first, vt_prop))
            continue;
        if(vt_prop.route_point.track_id == track_id) {
            double d = arolib::geometry::calc_dist(vt_prop.route_point, vt_point);
            if( std::fabs(d-min_dist) < tolerance ){
                min_dist = std::min(d, min_dist);
                found = true;
            }
            else if ( d < min_dist) {
                min_dist = d;
                nearestVertices.clear();
                found = true;
            }
            else
                continue;
            if(maxVertices == 0 || nearestVertices.size() < maxVertices)
                nearestVertices.emplace_back(*vt_it.first);
        }
    }
    return found;
}

bool searchNearestVerticesOnTrack(const DirectedGraph::Graph &graph,
                                  const std::vector<DirectedGraph::vertex_t> vertexSet,
                                  const DirectedGraph::vertex_t vt,
                                  const int& track_id,
                                  std::vector<DirectedGraph::vertex_t> &nearestVertices,
                                  size_t maxVertices,
                                  double tolerance,
                                  const DirectedGraph::Graph::VertexFilterFct &validityFunction) {
    return searchNearestVertices(graph,
                                 vertexSet,
                                 vt,
                                 nearestVertices,
                                 maxVertices,
                                 tolerance,
                                 [track_id, &validityFunction](const DirectedGraph::vertex_t& vertex, const DirectedGraph::vertex_property& vertex_prop)->bool{
                                    return vertex_prop.route_point.track_id == track_id && validityFunction(vertex, vertex_prop);
                                 });
}


std::vector<DirectedGraph::vertex_t> getConnectedVertices(const DirectedGraph::Graph &graph, const DirectedGraph::vertex_t vt, std::set<DirectedGraph::vertex_t> exclude)
{
    auto vts = getConnectedVerticesSet(graph, vt, exclude);
    return std::vector<DirectedGraph::vertex_t>(vts.begin(), vts.end());
}

std::vector<DirectedGraph::vertex_t> getConnectedVertices(const DirectedGraph::Graph &graph,
                                                          const DirectedGraph::vertex_t vt,
                                                          bool towards_vt,
                                                          std::set<DirectedGraph::vertex_t> exclude)
{
    auto vts = getConnectedVerticesSet(graph, vt, towards_vt, exclude);
    return std::vector<DirectedGraph::vertex_t>(vts.begin(), vts.end());
}

std::set<DirectedGraph::vertex_t> getConnectedVerticesSet(const DirectedGraph::Graph &graph, const DirectedGraph::vertex_t vt, std::set<DirectedGraph::vertex_t> exclude)
{
    auto v1 = getConnectedVerticesSet(graph, vt, true, exclude);
    auto v2 = getConnectedVerticesSet(graph, vt, false, exclude);
    v1.insert(v2.begin(), v2.end());
    return v1;
}

std::set<DirectedGraph::vertex_t> getConnectedVerticesSet(const DirectedGraph::Graph &graph, const DirectedGraph::vertex_t vt, bool towards_vt, std::set<DirectedGraph::vertex_t> exclude)
{
    std::set<DirectedGraph::vertex_t> ret;
    if(towards_vt){
        for( auto _edges = boost::in_edges(vt, graph)
             ;_edges.first != _edges.second
             ; _edges.first++){
            DirectedGraph::vertex_t adj = source(*_edges.first, graph);
            if(exclude.find(adj) == exclude.end())
                ret.insert(adj );
        }
    }
    else{
        for( auto _edges = boost::out_edges(vt, graph)
             ;_edges.first != _edges.second
             ; _edges.first++){
            DirectedGraph::vertex_t adj = target(*_edges.first, graph);
            if(exclude.find(adj) == exclude.end())
                ret.insert(adj);
        }
    }

    DirectedGraph::vertex_property vt_prop = graph[vt];
    exclude.insert(vt);
    for(auto adj : ret){//check if the adjacent vertex is in the same location as vt; if so, add the adjacent vertices of this vertex as well
        DirectedGraph::vertex_property adj_prop = graph[adj];
        if( arolib::geometry::calc_dist(vt_prop.route_point, adj_prop.route_point) < 1e-5 ){
            auto ret2 = getConnectedVerticesSet(graph, adj, towards_vt, exclude);
            ret.insert( ret2.begin(), ret2.end() );
        }
    }

    return ret;

}


std::vector<DirectedGraph::vertex_t> getClosestValidVertices(const DirectedGraph::Graph &graph,
                                                             const Point &p,
                                                             const DirectedGraph::Graph::VertexFilterFct &validityFunction,
                                                             size_t maxVts)
{
    std::vector<DirectedGraph::vertex_t> vts;
    if(maxVts == 0)
        return vts;

    std::multimap<double, DirectedGraph::vertex_t> vtsMap;

    for(auto vt_it = boost::vertices(graph); vt_it.first != vt_it.second; vt_it.first++){
        auto vt = *vt_it.first;
        const auto& v_prop = graph[vt];
        if(!validityFunction(vt, v_prop))
            continue;
        double dist = arolib::geometry::calc_dist(v_prop.route_point, p);
        if(vtsMap.size() < maxVts || vtsMap.rbegin()->first > dist )
            vtsMap.insert( std::make_pair(dist, vt) );
        if(vtsMap.size() > maxVts)
            vtsMap.erase(std::prev(vtsMap.end()));
    }

    for(auto &it : vtsMap)
        vts.emplace_back(it.second);

    return vts;

}

DirectedGraph::VisitPeriod getVisitPeriod(MachineId_t machineId,
                                          const std::vector<RoutePoint> &route_points,
                                          double machineRadius,
                                          size_t index,
                                          const DirectedGraph::Graph *graph,
                                          std::shared_ptr<Logger> _logger)
{

    Logger logger(LogLevel::CRITIC, __FUNCTION__);
    logger.setParent(_logger);

    DirectedGraph::VisitPeriod vp(machineId, -1, -1, -1);
    if(index >= route_points.size()
            || route_points.size() < 2
            || route_points.at(index).time_stamp < 1e-3)
        return vp;
    auto& rp = route_points.at(index);
    vp.timestamp = rp.time_stamp;

    if(graph && index+1 < route_points.size()){
        auto& rp2 = route_points.at(index+1);
        if(rp2.type == RoutePoint::FIELD_ENTRY || rp2.type == RoutePoint::FIELD_EXIT){
            FieldAccessPoint fap;
            fap.point() = rp2.point();
            auto v_it = graph->accesspoint_vertex_map().find(fap);
            if(v_it == graph->accesspoint_vertex_map().end())
                logger.printOut(LogLevel::WARNING, __FUNCTION__, "Field-access routepoint not found in graph map");
            else
                vp.next_vt.emplace_back( std::make_pair(v_it->second, rp2) );
        }
        else if (rp2.isOfTypeWorking()){
            auto v_it = graph->routepoint_vertex_map().find(rp2);
            if(v_it == graph->routepoint_vertex_map().end())
                logger.printOut(LogLevel::WARNING, __FUNCTION__, "'Working' routepoint not found in graph map");
            else
                vp.next_vt.emplace_back( std::make_pair(v_it->second, rp2) );
        }
        else if (rp2.type == RoutePoint::RESOURCE_POINT){
            //do nothing
        }
        else{//brute-search the vertex
            DirectedGraph::vertex_t vtTmp = graph->findVertexByLocation(rp2.point());
            if(vtTmp < 0)
                logger.printOut(LogLevel::WARNING, __FUNCTION__, "Vertex corresponding to the routepoint not found in graph");
            else
                vp.next_vt.emplace_back( std::make_pair(vtTmp, rp2) );
        }

    }

    if(machineRadius < 1e-3){
        if(index == 0)
            vp.time_in = rp.time_stamp;
        else
            vp.time_in = rp.time_stamp + 0.5 * ( rp.time_stamp - route_points.at(index-1).time_stamp );

        if(index + 1 == route_points.size())
            vp.time_out = rp.time_stamp;
        else
            vp.time_out = rp.time_stamp + 0.5 * ( route_points.at(index+1).time_stamp - rp.time_stamp );

        return vp;
    }

    double dTmp = machineRadius;
    vp.time_in = rp.time_stamp;
    vp.time_out = rp.time_stamp;

    for(int i = (int)index-1 ; i >= 0 ; --i){
        double d = arolib::geometry::calc_dist( route_points.at(i+1) , route_points.at(i) );
        if(d > dTmp){
            vp.time_in -= ( (route_points.at(i+1).time_stamp - route_points.at(i).time_stamp) * dTmp/d );
            break;
        }
        vp.time_in = route_points.at(i).time_stamp;
        dTmp -= d;
    }

    dTmp = machineRadius;
    for(size_t i = index+1 ; i < route_points.size() ; ++i){
        double d = arolib::geometry::calc_dist( route_points.at(i), route_points.at(i-1) );
        if(d > dTmp){
            vp.time_out += ( (route_points.at(i).time_stamp - route_points.at(i-1).time_stamp) * dTmp/d );
            break;
        }
        vp.time_out = route_points.at(i).time_stamp;
        dTmp -= d;
    }

    return vp;
}

DirectedGraph::vertex_t calcNearestVertexToPoint(const DirectedGraph::Graph &graph,
                                                 const Point &goal_point,
                                                 const std::vector<DirectedGraph::vertex_t>& v) {
    if (v.size() == 0)
        throw std::invalid_argument( "vector is empty" );

    double min_dist = arolib::geometry::calc_dist(goal_point, graph[v.at(0)].route_point);
    int min_id = 0;
    for (int i = 1; i < v.size(); ++i) {
        double d = arolib::geometry::calc_dist(goal_point, graph[v.at(i)].route_point);
        if (d <= min_dist) {
            min_dist = d;
            min_id = i;
        }
    }
    return v.at(min_id);
}

std::vector<DirectedGraph::vertex_t> calcConnectedVertices(const DirectedGraph::Graph &graph,
                                                                  DirectedGraph::vertex_t vt) {
    std::vector<DirectedGraph::vertex_t> connected_vts;
    std::pair<DirectedGraph::out_edge_iterator, DirectedGraph::out_edge_iterator> out_edges =
            boost::out_edges(vt, graph);
    for(;out_edges.first != out_edges.second; out_edges.first++){
        connected_vts.push_back(target(*out_edges.first, graph));
    }
    return connected_vts;
}

bool addOverrollToEdges(DirectedGraph::Graph &graph, DirectedGraph::overroll_property overroll,
                        DirectedGraph::vertex_t vt0, DirectedGraph::vertex_t vt1) {
    DirectedGraph::edge_t e;
    bool edge_found;
    boost::tie(e, edge_found) = boost::edge(vt0, vt1, graph);
    if (edge_found) {
        graph[e].overruns.push_back(overroll);
    } else {
        std::cout << "edge not found in graph" << std::endl;
    }

    // Add overrun to reverse edge
    DirectedGraph::edge_t reverse_edge;
    bool reverse_edge_found;
    boost::tie(reverse_edge, reverse_edge_found) = boost::edge(vt1, vt0, graph);
    if (reverse_edge_found) {
        graph[reverse_edge].overruns.push_back(overroll);
    } else {
        std::cout << "edge not found in graph" << std::endl;
    }
    return edge_found && reverse_edge_found;
}

AroResp insertOrReplaceInitialPositions(DirectedGraph::Graph & graph,
                                        const Subfield &subfield,
                                        const std::vector<Machine> &machines,
                                        const std::set<Machine::MachineType>& machineTypes,
                                        const std::map<MachineId_t, MachineDynamicInfo> &machineInitialStates,
                                        const OutFieldInfo& outFieldInfo,
                                        const Polygon &field_boundary,
                                        bool connectInHeadlandSubGraph,
                                        bool connectInInfieldSubGraph,
                                        bool locateMissingMachinesAtFAP)
{
    AroResp ret = AroResp::ok();

    struct SearchInfo{
        bool ok = false;
        double minDist = std::numeric_limits<double>::max();
        DirectedGraph::vertex_t vt = -1;
        DirectedGraph::vertex_property v_prop;
        Point pt;
    };

    for(const auto& m : machines){
        if(!machineTypes.empty() && machineTypes.find(m.machinetype) == machineTypes.end())
            continue;

        auto it_initP = graph.initialpoint_vertex_map().find(m.id);
        DirectedGraph::vertex_property* v_prop = nullptr;
        DirectedGraph::vertex_property v_prop_tmp;
        if(it_initP != graph.initialpoint_vertex_map().end())
            v_prop = &graph[it_initP->second];

        bool vertexAdded = v_prop;
        if(!v_prop)
            v_prop = &v_prop_tmp;

        RoutePoint& rp = v_prop->route_point;
        rp.bunker_mass = 0;
        rp.bunker_volume = 0;
        rp.time_stamp = -1;
        rp.track_id = -1;
        rp.type = RoutePoint::INITIAL_POSITION;
        v_prop->graph_location = DirectedGraph::vertex_property::DEFAULT;//if changed, the condition to search for closest vertices must not include the init-point vertices

        auto it_mdi = machineInitialStates.find(m.id);
        if(it_mdi == machineInitialStates.end()){
            if(vertexAdded)//do not replace
                continue;

            if(!locateMissingMachinesAtFAP)
                continue;

            if(graph.accesspoint_vertex_map().empty()){
                ret = AroResp(-1, "No current state found for machine with id " + std::to_string(m.id) + " and no available access point to locate the machine");
                continue;
            }

            auto fap_it = graph.accesspoint_vertex_map().begin();
            rp.point() = fap_it->first.point();

            DirectedGraph::edge_property edge_prop;
            edge_prop.edge_type = DirectedGraph::EdgeType::INIT;
            edge_prop.distance = 0;
            edge_prop.defWidth = 0;

            DirectedGraph::vertex_t point_vt = boost::add_vertex(*v_prop, graph);
            graph.initialpoint_vertex_map()[m.id] = point_vt;
            graph.addEdge(point_vt, fap_it->second, edge_prop, false, true, true);
            vertexAdded = true;
            continue;
        }
        else
            rp.point() = it_mdi->second.position;

        if( vertexAdded && arolib::geometry::calc_dist( rp, v_prop->route_point ) < m.workingRadius() * 1e-3 )//at the previous position --> do not replace
            continue;

        if( !arolib::geometry::in_polygon(rp, field_boundary) ){
            //connect to field access points

            DirectedGraph::vertex_t point_vt;

            for(auto &fap_it : graph.accesspoint_vertex_map()){
                const FieldAccessPoint& fap = fap_it.first;

                DirectedGraph::edge_property edge_prop;
                edge_prop.edge_type = DirectedGraph::EdgeType::INIT;
                edge_prop.defWidth = 0;

                if( outFieldInfo.getArrivalCost(fap.id, m.id, OutFieldInfo::MACHINE_EMPTY, edge_prop.arrivalCosts) ){
                    edge_prop.distance = edge_prop.arrivalCosts.distance;
                    if(edge_prop.distance <= 0)
                        edge_prop.distance = arolib::geometry::calc_dist(rp, fap);
                }
                else if(fap.accessType != FieldAccessPoint::AP_EXIT_ONLY){//check if the machine is close to the entry point and connect it to the access point directly
                    double distEps = 5 * m.workingRadius();
                    double dist = arolib::geometry::calc_dist(rp, fap);
                    if( dist < distEps ){//close enough, connect directly to access point vertex

                        edge_prop.arrivalCosts.distance = dist;
                        edge_prop.arrivalCosts.time = 0;
                        if( m.max_speed_empty > 0 )
                            edge_prop.arrivalCosts.time = dist / m.max_speed_empty;

                        edge_prop.distance = dist;
                    }
                    else
                        continue;
                }
                else
                    continue;

                if(!vertexAdded){
                    point_vt = boost::add_vertex(*v_prop, graph);
                    graph.initialpoint_vertex_map()[m.id] = point_vt;
                    vertexAdded = true;
                }
                else
                    point_vt = graph.initialpoint_vertex_map()[m.id];

                graph.addEdge(point_vt, fap_it.second, edge_prop, false, true, true);
            }

        }
        else{
            double timestampLimit = ( m.isOfWorkingType(true) ? 1e-3 : -1e-5 );

            bool inIF = false, outIF = false;
            if(subfield.boundary_inner.points.size() > 2){
                inIF = arolib::geometry::in_polygon(rp, subfield.boundary_inner);
                outIF = !inIF;
            }

            double searchRadius = 1.5 * std::max( {m.workingRadius(),
                                                   graph.workingWidth_HL(),
                                                   graph.workingWidth_IF()} );

            double machineWidth = m.width > 0 ? m.width : m.working_width;
            if(machineWidth <= 0)
                machineWidth = outIF ? graph.workingWidth_HL() : graph.workingWidth_IF();

            double maxAllowedDist = 2 * searchRadius;
            if(maxAllowedDist <= 0)
                maxAllowedDist = 20;

            bool initPointConnected = false;

            auto isVtValid = [&](const DirectedGraph::vertex_t&, const DirectedGraph::vertex_property& v_prop2)->bool
            {
                    if( !v_prop2.route_point.isOfTypeWorking(true) && v_prop2.route_point.type != RoutePoint::HEADLAND )//do not connect to FAP or resource points
                        return false;

                    if(   (!connectInHeadlandSubGraph && v_prop2.graph_location == DirectedGraph::vertex_property::HEADLAND)
                        || (!connectInInfieldSubGraph && v_prop2.graph_location == DirectedGraph::vertex_property::INFIELD) )
                        return false;

                    const RoutePoint& rp2 = v_prop2.route_point;
                    if (rp2.isOfTypeWorking(true) && rp2.time_stamp > timestampLimit)
                        return false;

                    return true;
            };

            if(searchRadius > 0){//try first connecting to all valid vertices in the (working) radius
                auto vts = graph.getVerticesInRadius(rp, searchRadius, isVtValid);

                for( const auto& vt2 : vts ){
                    const DirectedGraph::vertex_property& v_prop2 = graph[vt2];
                    const RoutePoint& rp2 = v_prop2.route_point;

                    DirectedGraph::vertex_t point_vt;
                    if(!vertexAdded){
                        point_vt = boost::add_vertex(*v_prop, graph);
                        graph.initialpoint_vertex_map()[m.id] = point_vt;
                        vertexAdded = true;
                    }
                    else
                        point_vt = graph.initialpoint_vertex_map()[m.id];

                    DirectedGraph::edge_property edge_prop;
                    edge_prop.distance = arolib::geometry::calc_dist(rp, rp2);
                    edge_prop.edge_type = DirectedGraph::EdgeType::DEFAULT;
                    edge_prop.defWidth = machineWidth;
                    graph.addEdge(point_vt, vt2, edge_prop, false, true, true);
                    initPointConnected = true;
                }
            }

            if(!initPointConnected){//try connecting to the closest valid vertices withing a higher radius
                std::vector<SearchInfo> searchInfo(4); //RP_IF, NRP_I, RP_HL, NRP_HL

                auto vts = graph.getVerticesInRadius(rp, maxAllowedDist, isVtValid);
                for( const auto& vt2 : vts ){
                    DirectedGraph::vertex_property v_prop2 = graph[vt2];

//                for(DirectedGraph::vertex_iter vp = boost::vertices(graph); vp.first != vp.second; vp.first++){
//                    DirectedGraph::vertex_t vt2 = *vp.first;
//                    DirectedGraph::vertex_property v_prop2 = graph[vt2];

//                    if( !v_prop2.route_point.isOfTypeWorking(true) && v_prop2.route_point.type != RoutePoint::HEADLAND )//do not connect to FAP or resource points
//                        continue;

//                    if(   (!connectInHeadlandSubGraph && v_prop2.graph_location == vertex_property::HEADLAND)
//                        || (!connectInInfieldSubGraph && v_prop2.graph_location == vertex_property::INFIELD) )
//                        continue;

                    size_t deltaInd = 0;
                    double workingwidth = graph.workingWidth_IF();

                    if(v_prop2.graph_location == DirectedGraph::vertex_property::HEADLAND
                            || Track::isHeadlandTrack(v_prop2.route_point.track_id)){
                        deltaInd = 2;
                        workingwidth = graph.workingWidth_HL();
                    }

                    RoutePoint rp2 = v_prop2.route_point;
                    if (rp2.isOfTypeWorking(true)){
                        if(rp2.time_stamp > timestampLimit )
                            continue;
                        double dist = arolib::geometry::calc_dist(rp, rp2);
                        if( searchInfo.at(deltaInd).minDist > dist && dist < maxAllowedDist ){
                            searchInfo.at(deltaInd).minDist = dist;
                            searchInfo.at(deltaInd).vt = vt2;
                            searchInfo.at(deltaInd).v_prop = v_prop2;
                            searchInfo.at(deltaInd).ok = true;
                            searchInfo.at(deltaInd).pt = rp2.point();
                        }
                    }
                    else{
                        double dist = arolib::geometry::calc_dist(rp, rp2);
                        if(searchInfo.at(deltaInd+1).minDist > dist && dist < 2*workingwidth && dist < maxAllowedDist ){
                            searchInfo.at(deltaInd+1).minDist = dist;
                            searchInfo.at(deltaInd+1).vt = vt2;
                            searchInfo.at(deltaInd+1).v_prop = v_prop2;
                            searchInfo.at(deltaInd+1).ok = true;
                            searchInfo.at(deltaInd+1).pt = rp2.point();
                        }
                    }
                }

                //check if the RP-vertex in headland is better than the non-RP-vertex in infield
                if(connectInInfieldSubGraph){
                    if(!searchInfo.at(1).ok)
                        searchInfo.at(1) = searchInfo.at(2);
                    else if( searchInfo.at(2).ok ){
                        if(searchInfo.at(1).minDist > searchInfo.at(2).minDist)
                            searchInfo.at(1) = searchInfo.at(2);
                    }
                }

                searchInfo.at(0).ok &= connectInInfieldSubGraph;
                searchInfo.at(1).ok &= connectInInfieldSubGraph;
                searchInfo.at(2).ok &= connectInHeadlandSubGraph;
                searchInfo.at(3).ok &= connectInHeadlandSubGraph;

                if(searchInfo.at(2).ok)//only allow connection to the field boundary if it is closer than the connection to the HL route point
                    searchInfo.at(3).ok &= searchInfo.at(3).minDist < searchInfo.at(2).minDist;

                if(inIF){
                    if(searchInfo.at(0).ok)
                        searchInfo.at(1).ok &= searchInfo.at(1).minDist < searchInfo.at(0).minDist;
                }
                if(outIF){
                    if(searchInfo.at(1).ok)
                        searchInfo.at(0).ok &= searchInfo.at(0).minDist < searchInfo.at(1).minDist;
                }

                for(size_t i = 0 ; i < searchInfo.size() ; ++i){
                    auto& si = searchInfo.at(i);
                    if(si.ok){
                        DirectedGraph::vertex_t point_vt;
                        if(!vertexAdded){
                            point_vt = boost::add_vertex(*v_prop, graph);
                            graph.initialpoint_vertex_map()[m.id] = point_vt;
                            vertexAdded = true;
                        }
                        else
                            point_vt = graph.initialpoint_vertex_map()[m.id];

                        DirectedGraph::edge_property edge_prop;
                        edge_prop.distance = si.minDist;
                        edge_prop.edge_type = DirectedGraph::EdgeType::DEFAULT;
                        edge_prop.defWidth = machineWidth;

                        graph.addEdge(point_vt, si.vt, edge_prop, false, true, true) ;
                    }
                }

            }
        }

    }

    return ret;
}

void updateTimestampsFromBaseRoute(DirectedGraph::Graph &graph, const Route &base_route, double delta_time, size_t indStart, int indFinish, double timestamp_th)
{
    indFinish = ( indFinish < 0 || indFinish >= base_route.route_points.size() ? base_route.route_points.size() -1 : indFinish);

    if(indStart >= base_route.route_points.size())
        return;

    if(std::isnan(timestamp_th))
        timestamp_th = base_route.route_points.at(indStart).time_stamp;

    RoutePoint lastUpdatedRP(Point::invalidPoint());

    for (int j = indFinish; j+1 >= indStart+1; --j){//in reverse so that the timestamp for the vertex is the first working timestamp

        auto& rp = base_route.route_points.at(j);

        if(rp.point() == lastUpdatedRP.point())//check if there were delays corresponding to turning. if so, leave the latest route point
            continue;
        lastUpdatedRP = rp;

        double time_stamp = ( rp.time_stamp > -1e-6 ? rp.time_stamp + delta_time : rp.time_stamp );

        if(rp.type == RoutePoint::TRACK_END && Track::isHeadlandTrack(rp.track_id) && !graph.hasPartialHeadlands())//do not update track_ends vertices from headland
            continue;

        auto it_vt = graph.routepoint_vertex_map().find(rp);
        if(it_vt != graph.routepoint_vertex_map().end()){//update the vertex (working) timestamp corresponding to the harvester
            if( graph.boundary_vts().find(it_vt->second) != graph.boundary_vts().end() )//do not update vertices over the boundary
                continue;

            DirectedGraph::vertex_property &v_prop = graph[it_vt->second];

            if(timestamp_th > -1e-9 && v_prop.route_point.time_stamp < timestamp_th - 1e-6)
                continue;

            v_prop.route_point.time_stamp = time_stamp;//important: set the timestamp of the route-point of harvester route (instead of adding the delay to the vertex timestamp), in case the vertex timestamp is outdated
        }

        if(rp.type == RoutePoint::TRACK_START && Track::isHeadlandTrack(rp.track_id) && !graph.hasPartialHeadlands()){//update the corresponding track_end vertex from headland
            auto vts_te = graph.getVerticesInRadius(rp, 1e-1, [&rp](const DirectedGraph::vertex_t&, const DirectedGraph::vertex_property& v_prop)->bool{
                if (rp.track_id != v_prop.route_point.track_id
                        || v_prop.route_point.type != RoutePoint::TRACK_END)
                    return false;
                return true;
            });
            for(auto vt_te : vts_te)
                graph[vt_te].route_point.time_stamp = time_stamp;
        }
    }

}

void resetTimestampsFromBaseRoute(DirectedGraph::Graph &graph, const Route &base_route, size_t indStart, size_t indFinish, double newTimestamp)
{

    if(indStart >= base_route.route_points.size())
        return;
    indFinish = std::min(indFinish, base_route.route_points.size()-1);

    for (int j = indStart; j <= indFinish; ++j){

        auto& rp = base_route.route_points.at(j);

        if(rp.type == RoutePoint::TRACK_END && Track::isHeadlandTrack(rp.track_id))//do not update track_ends vertices from headland
            continue;

        auto it_vt = graph.routepoint_vertex_map().find(rp);
        if(it_vt != graph.routepoint_vertex_map().end()){//update the vertex (working) timestamp corresponding to the harvester

            if( graph.boundary_vts().find(it_vt->second) != graph.boundary_vts().end() )//do not update vertices over the boundary
                continue;

            DirectedGraph::vertex_property &v_prop = graph[it_vt->second];
            v_prop.route_point.time_stamp = newTimestamp;
        }

        if(rp.type == RoutePoint::TRACK_START && Track::isHeadlandTrack(rp.track_id)){//update the corresponding track_end vertex from headland
            auto vts_te = graph.getVerticesInRadius(rp, 1e-1, [&rp](const DirectedGraph::vertex_t&, const DirectedGraph::vertex_property& v_prop)->bool{
                if (rp.track_id != v_prop.route_point.track_id
                        || v_prop.route_point.type != RoutePoint::TRACK_END)
                    return false;
                return true;
            });
            for(auto vt_te : vts_te)
                graph[vt_te].route_point.time_stamp = newTimestamp;
        }
    }
}

std::set<DirectedGraph::vertex_t> updateVisitPeriods(DirectedGraph::Graph &graph,
                                                        const Machine &machine,
                                                        const std::vector<RoutePoint> &route_points,
                                                        size_t ind_0,
                                                        int ind_n)
{
    std::set<DirectedGraph::vertex_t> ret;
    if(ind_n < 0 || ind_n >= route_points.size())
        ind_n = route_points.size();

    if(ind_0 >= route_points.size()|| ind_n <= ind_0)
        return ret;

    double rad = machine.workingRadius();
    double width = std::max(machine.working_width, machine.width);

    std::set<DirectedGraph::vertex_t> nextVts;
    for(size_t i = 0; i <= ind_n - ind_0; i++){//transversing in reverse!
        double dTimePrev, dTimeNext;
        double dDist;
        std::vector<DirectedGraph::vertex_t> vts2;
        const RoutePoint *rpPrev, *rpNext;

        size_t indRef = ind_n - i;
        auto rpRef = route_points.at(indRef);

        if( (indRef == ind_n && ind_n + 1 >= route_points.size())//last segment point, and there are no points after the segment last point
                || (indRef == ind_0 && ind_0 == 0) ) {//first segment point, and there are no points before the first segment point
            if(i == 0){//last point
                rpPrev = &route_points.at(ind_n-1);
                rpNext = &route_points.at(ind_n);
            }
            else{//first point
                rpPrev = &route_points.at(ind_0);
                rpNext = &route_points.at(ind_0+1);
            }
            dDist = rad;
            if (dDist < 1e-6)
                dDist = 0.5 * arolib::geometry::calc_dist(rpNext->point(), rpPrev->point());

            if(rpRef.type == RoutePoint::FIELD_ENTRY)
                rpPrev = &route_points.at(indRef);
            if(rpRef.type == RoutePoint::FIELD_EXIT)
                rpNext = &route_points.at(indRef);

            vts2 = graph.getVerticesInRadius(rpRef, dDist);

        }
        else{
            rpPrev = &route_points.at(indRef-1);
            rpNext = &route_points.at(indRef+1);

            if(rpRef.type == RoutePoint::FIELD_ENTRY)
                rpPrev = &route_points.at(indRef);
            if(rpRef.type == RoutePoint::FIELD_EXIT)
                rpNext = &route_points.at(indRef);

            Point pLine_1 = arolib::geometry::getCentroid( rpRef, rpPrev->point() );
            Point pLine_2 = arolib::geometry::getCentroid( rpRef, rpNext->point() );


            vts2 = graph.getVerticesInRectangle(pLine_1, pLine_2, width);
        }

        if(rpRef.time_stamp > 1e-6){

            dTimePrev = 0.5 * (rpRef.time_stamp - rpPrev->time_stamp);
            dTimeNext = 0.5 * (rpNext->time_stamp - rpRef.time_stamp);

            for(auto vt : vts2){//update the visit period of the proximal vertices
                DirectedGraph::vertex_property& vt_prop = graph[ vt ];
                DirectedGraph::VisitPeriod vp(machine.id, rpRef.time_stamp - dTimePrev, rpRef.time_stamp + dTimeNext, rpRef.time_stamp);
                if( i != 0 ){
                    for(auto nextVt : nextVts){
                        if(vt != nextVt)
                            vp.next_vt.emplace_back( std::make_pair(nextVt, *rpNext) );
                    }
                }
                vt_prop.visitPeriods.insert( std::make_pair(vp.time_in, vp) );
            }
        }

        nextVts.clear();
        nextVts.insert( vts2.begin(), vts2.end() );
        ret.insert( vts2.begin(), vts2.end() );

    }

    return ret;

}


void updateVisitPeriods(std::multimap<double, DirectedGraph::VisitPeriod> &visitPeriods, MachineId_t machineId, double timestamp, double delay, bool onlyTimeOut)
{
    return updateVisitPeriods(visitPeriods,machineId,timestamp,delay,
                              [onlyTimeOut](const DirectedGraph::VisitPeriod &)->bool{ return onlyTimeOut; });
}

void updateVisitPeriods(std::multimap<double, DirectedGraph::VisitPeriod> &visitPeriods, MachineId_t machineId, double timestamp, double delay, std::function<bool (const DirectedGraph::VisitPeriod &)> onlyTimeOutFct)
{
    std::vector<DirectedGraph::VisitPeriod> new_vps;
    for(auto it_vp = visitPeriods.begin() ; it_vp != visitPeriods.end();){
        DirectedGraph::VisitPeriod& vp = it_vp->second;
        if( vp.machineId != machineId
                || ( vp.time_in < timestamp && vp.time_out < timestamp  ) ){
            ++it_vp;
            continue;
        }

        if(onlyTimeOutFct(vp)){//only the time_out has to be adjusted
            vp.time_out += delay;
            ++it_vp;
            continue;
        }

        new_vps.emplace_back(vp);
        new_vps.back().time_in += delay;
        new_vps.back().time_out += delay;
        new_vps.back().timestamp += delay;
        it_vp = visitPeriods.erase(it_vp);
    }
    for(auto& vp : new_vps)
        visitPeriods.insert( std::make_pair(vp.time_in, vp) );

}

std::set<DirectedGraph::vertex_t> getReachableVertices(const DirectedGraph::Graph &graph, const DirectedGraph::vertex_t &vt_from, size_t maxEdges){
    std::set<DirectedGraph::vertex_t> ret;
    if(maxEdges == 0)
        return ret;

    std::function<void(const DirectedGraph::vertex_t&, size_t, std::set<DirectedGraph::vertex_t>&)> update =
            [&graph, maxEdges, &update](const DirectedGraph::vertex_t& vt_from, size_t currentLevel, std::set<DirectedGraph::vertex_t>& visited) -> void{
        static size_t maxLevels = 3;
        if(currentLevel > maxLevels)
            return;
        visited.insert(vt_from);
        auto connVts = getConnectedVertices(graph, vt_from, false);
        for(auto& vt : connVts){
            if(visited.find(vt) != visited.end())
                continue;
            update(vt, currentLevel+1, visited);
        }
    };

    update(vt_from, 0, ret);
    ret.erase(vt_from);
    return ret;
}


}

