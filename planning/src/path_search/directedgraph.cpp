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
 
#include "arolib/planning/path_search/directedgraph.hpp"

#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <limits>       // std::numeric_limits

#define DEBUG_GRAPH


namespace arolib{

namespace DirectedGraph{

EdgeType intToEdgeType(int value)
{
    if(value == EdgeType::DEFAULT)
        return EdgeType::DEFAULT;
    else if(value == EdgeType::CROSS)
        return EdgeType::CROSS;
    else if(value == EdgeType::CROSS_HL)
        return EdgeType::CROSS_HL;
    else if(value == EdgeType::BOUNDARY_CONN)
        return EdgeType::BOUNDARY_CONN;
    else if(value == EdgeType::FAP_TO_RP)
        return EdgeType::FAP_TO_RP;
    else if(value == EdgeType::RP_TO_FAP)
        return EdgeType::RP_TO_FAP;
    else if(value == EdgeType::FAP_TO_FAP)
        return EdgeType::FAP_TO_FAP;
    else if(value == EdgeType::INIT)
        return EdgeType::INIT;

    throw std::invalid_argument( "The given value does not correspond to any DirectedGraph::EdgeType" );
}


VisitPeriod::VisitPeriod(MachineId_t _machineId, double _time_in, double _time_out, double _timestamp, std::vector<std::pair<vertex_t, RoutePoint> > _next_vt):
    machineId(_machineId),
    time_in(_time_in),
    time_out(_time_out),
    timestamp(_timestamp),
    next_vt(_next_vt){

}

bool VisitPeriod::isBusy(const std::multimap<double, VisitPeriod>& map,
                         double time_in,
                         double duration,
                         int maxMachines,
                         double &waiting,
                         vertex_t currentVt,
                         bool& towardsCurrentVt,
                          std::vector<MachineId_t> &machineIds,
                          std::set<MachineId_t> excludedMachines)
{
    waiting = 0;
    towardsCurrentVt = false;
    machineIds.clear();
    if(maxMachines <= 0)
        return false;

    double time_out = time_in + duration;
    std::multimap<double, std::pair<MachineId_t, bool>, std::greater<double>> waitings;

    for(auto it : map){
        if( it.first > time_out )
            break;
        const VisitPeriod& vp = it.second;

        if( time_in > vp.time_out )
            continue;

        if(excludedMachines.find(vp.machineId) != excludedMachines.end())
            continue;

        bool bNextVt = false;
        for(auto &next_vt : vp.next_vt){
            if(next_vt.first == currentVt){
                bNextVt = true;
                break;
            }
        }

        waitings.insert( std::make_pair( vp.time_out - time_in , std::make_pair(vp.machineId, bNextVt) ) );
    }


    if(waitings.size() >= maxMachines){
        //auto it = std::next(waitings.begin(), maxMachines-1);
        auto it = waitings.begin();
        machineIds.push_back( it->second.first );
        for(size_t i = 1 ; i < maxMachines ; ++i ){
            ++it;
            machineIds.push_back( it->second.first );
        }
        waiting = it->first;

        while( it != waitings.end() ){
            if(it->second.second){
                towardsCurrentVt = true;
                break;
            }
            ++it;
        }

    }

    return waitings.size() >= maxMachines;
}

std::set<MachineId_t> VisitPeriod::getVisitingMachines(const std::multimap<double, VisitPeriod> &map, double time_in, double duration, std::set<MachineId_t> excludedMachines)
{
    std::set<MachineId_t> machineIds;
    double time_out = time_in + duration;
    for(auto it : map){
        if( it.first > time_out )
            break;
        const VisitPeriod& vp = it.second;

        if( time_in > vp.time_out )
            continue;

        if(excludedMachines.find(vp.machineId) != excludedMachines.end())
            continue;

        machineIds.insert( vp.machineId );
    }
    return machineIds;
}

bool VisitPeriod::isMachineVisiting(const std::multimap<double, VisitPeriod> &map, MachineId_t machineId, double time_in, double time_out)
{
    if(time_in >= time_out)
        return false;
    for(auto it : map){
        if( it.first > time_out )
            break;
        const VisitPeriod& vp = it.second;

        if( time_in > vp.time_out )
            continue;

        if(machineId == vp.machineId)
            return true;
    }
    return false;

}

vertex_property::GraphLocation vertex_property::intToGraphLocation(int value)
{
    if(value == GraphLocation::DEFAULT)
        return GraphLocation::DEFAULT;
    else if(value == GraphLocation::HEADLAND)
        return GraphLocation::HEADLAND;
    else if(value == GraphLocation::INFIELD)
        return GraphLocation::INFIELD;

    throw std::invalid_argument( "The given value does not correspond to any vertex_property::GraphLocation" );
}

int edge_property::m_countCustomValKey = 0;

bool edge_property::isOfType(const std::set<EdgeType> &types)
{
    return types.find(edge_type) != types.end();
}

int edge_property::getNewCustomValueKey()
{
    return m_countCustomValKey++;
}

std::size_t Graph::RoutePoint_KeyHash::operator()(const RoutePoint &rp) const
{
    return get(rp, 0);
}

std::size_t Graph::RoutePoint_KeyHash::get(const RoutePoint &rp, std::size_t seed)
{
    seed = Point::KeyHash::get( rp.point(), seed );
    boost::hash_combine(seed, (int)rp.type);
    boost::hash_combine(seed, rp.track_id);
    return seed;
}

bool Graph::RoutePoint_KeyEqual::operator()(const RoutePoint &rp1, const RoutePoint &rp2) const
{
    return rp1.point() == rp2.point()
            && rp1.type == rp2.type
            && rp1.track_id == rp2.track_id;
}

/** constructor
*/
Graph::Graph(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{
    m_meta.workingWidth_IF = -1;
    m_meta.workingWidth_HL = -1;

}

//Graph &Graph::operator=(const Graph &other){
//    if (this != &other) {
//        const boost_directed_graph* p_other = &other;
//        boost_directed_graph* p_this = this;
//        boost::copy_graph(*p_other, *p_this);
//        *( (LoggingComponent*) this) = *( (LoggingComponent*) &other );
//        this->m_meta = other.m_meta;
//    }
//    return *this;
//}

void Graph::clear()
{
    m_meta.workingWidth_IF = -1;
    m_meta.workingWidth_HL = -1;    
    m_meta.tracks_vertices_map.clear();
    m_meta.extremaTrackIds_IF.clear();
    m_meta.adjacentTrackIds_IF.clear();
    m_meta.adjacentTrackIds_HL.clear();
    m_meta.outermostTrackIds_HL.clear();
    m_meta.outFieldInfo.clearAll();
    m_meta.routepoint_vertex_map.clear();
    m_meta.accesspoint_vertex_map.clear();
    m_meta.resourcepoint_vertex_map.clear();
    m_meta.initialpoint_vertex_map.clear();
    m_meta.verticesLocationMap.clear();
    boost_directed_graph::clear();
}

vertex_t Graph::addVertex(const vertex_property &vp)
{
    vertex_t vt = boost::add_vertex(vp, *this);
    m_meta.verticesLocationMap[ (int)vp.route_point.x ][ (int)vp.route_point.y ].insert(vt);
    return vt;
}

bool Graph::addEdge(const vertex_t &v1, const vertex_t &v2, const edge_property &edgeProp, bool bidirectional, bool onlyIfNotExisting, bool overwrite)
{
    edge_property edgeProp_rev = edgeProp;
    std::swap(edgeProp_rev.p0, edgeProp_rev.p1);

    DirectedGraph::edge_t edge1, edge2;
    bool addEdge1 = true, addEdge2 = bidirectional;
    bool edge1_found = false, edge2_found = false;
    if(onlyIfNotExisting || overwrite){
        boost::tie(edge1, edge1_found) = boost::edge(v1, v2, *this);
        if(bidirectional)
            boost::tie(edge2, edge2_found) = boost::edge(v2, v1, *this);

        if(onlyIfNotExisting){
            if(edge1_found && !(bidirectional ^ edge2_found) )//edge1 and { edge2 (in case of bidirectional) exist / edge2 (in case of non bidirectional) does not exist } --> do nothing
                return false;
            addEdge1 = !edge1_found;
            addEdge2 = !edge2_found && bidirectional;
        }
        if(overwrite){
            addEdge1 &= !edge1_found;
            addEdge2 &= !edge2_found;
            if(edge1_found)//replace edge property
                (*this)[edge1] = edgeProp;
            if(edge2_found)//replace edge property
                (*this)[edge2] = edgeProp_rev;
        }
    }

    if(addEdge1)
        edge1 = boost::add_edge(v1, v2, edgeProp, *this).first;

    if(addEdge2){
        edge2 = boost::add_edge(v2, v1, edgeProp_rev, *this).first;

        DirectedGraph::edge_property &ep1 = (*this)[edge1];
        DirectedGraph::edge_property &ep2 = (*this)[edge2];
        ep1.revEdge = edge2;
        ep2.revEdge = edge1;
        ep1.bidirectional = true;
        ep2.bidirectional = true;
    }

    return addEdge1;
}

vertex_t Graph::findVertexByLocation(const Point &p,
                                     std::set<RoutePoint::RoutePointType> vertexRPTypes,
                                     std::set<vertex_property::GraphLocation> vertexGraphLocations) const
{
    const double eps = 1e-5;

    for(DirectedGraph::vertex_iter it_vt = boost::vertices(*this); it_vt.first != it_vt.second; it_vt.first++){
        const vertex_property& v_prop = (*this)[*it_vt.first];
        if(!vertexGraphLocations.empty() && vertexGraphLocations.find(v_prop.graph_location) == vertexGraphLocations.end())
            continue;
        if( arolib::geometry::calc_dist(p, v_prop.route_point) < eps
                && (vertexRPTypes.empty() || vertexRPTypes.find(v_prop.route_point.type) != vertexRPTypes.end()) )
            return *it_vt.first;
    }
    return -1;
}

vertex_t Graph::findClosestVertex(const Point &p, double maxDistance,
                                  std::set<RoutePoint::RoutePointType> vertexRPTypes,
                                  std::set<vertex_property::GraphLocation> vertexGraphLocations) const
{
    vertex_t ret = -1;
    double minDist = std::numeric_limits<double>::max();
    for(DirectedGraph::vertex_iter it_vt = boost::vertices(*this); it_vt.first != it_vt.second; it_vt.first++){
        const vertex_property& v_prop = (*this)[*it_vt.first];
        if(!vertexGraphLocations.empty() && vertexGraphLocations.find(v_prop.graph_location) == vertexGraphLocations.end())
            continue;
        double dist = arolib::geometry::calc_dist(p, v_prop.route_point);
        if(dist < maxDistance)
            continue;
        if(minDist > dist
                && (vertexRPTypes.empty() || vertexRPTypes.find(v_prop.route_point.type) != vertexRPTypes.end())){
            minDist = dist;
            ret = *it_vt.first;
        }
    }
    return ret;

}

bool Graph::saveVisitSchedule(const std::string &filename, char sep) const
{
    std::ofstream out(filename);
    if(!out.is_open()){
        logger().printError(__FUNCTION__, "Error opening file " + filename);
        return false;
    }

    out << "Vertex" << sep
        << "Point.x" << sep << "Point.y" << sep
        << "MachineId" << sep
        << "time_in" << sep
        << "time_out" << sep
        << "timestamp" << sep
        << "next_tvs" << std::endl;

    for(DirectedGraph::vertex_iter it_vt = boost::vertices(*this); it_vt.first != it_vt.second; it_vt.first++){
        const vertex_property& v_prop = (*this)[*it_vt.first];
        for(auto it_vp : v_prop.visitPeriods){
            const DirectedGraph::VisitPeriod& vp = it_vp.second;
            std::string nextVts = "";
            for(auto& vt : vp.next_vt)
                nextVts += (std::to_string(vt.first) + "/");
            if(!vp.next_vt.empty())
                nextVts.pop_back();
            out << *it_vt.first << sep
                << v_prop.route_point.point().toStringCSV(sep, 10) << sep
                << vp.machineId << sep
                << vp.time_in << sep
                << vp.time_out << sep
                << vp.timestamp << sep
                << nextVts << std::endl;
        }
    }

    out.close();
    return true;
}

bool Graph::saveVerticesInfo(const std::string &filename, char sep) const
{
    std::ofstream out(filename);
    if(!out.is_open()){
        logger().printError(__FUNCTION__, "Error opening file " + filename);
        return false;
    }

    out << "Vertex" << sep
        << "Point.x" << sep << "Point.y" << sep
        << "Graph_loc" << sep
        << "RP_type" << sep
        << "timestamp" << std::endl;

    for(DirectedGraph::vertex_iter it_vt = boost::vertices(*this); it_vt.first != it_vt.second; it_vt.first++){
        const vertex_property& v_prop = (*this)[*it_vt.first];
        const auto &rp = v_prop.route_point;
        out << *it_vt.first << sep
            << v_prop.route_point.point().toStringCSV(sep, 10) << sep
            << v_prop.graph_location << sep
            << rp.type << sep
            << rp.time_stamp << std::endl;
    }

    out.close();
    return true;

}

bool Graph::addOverrun(const edge_t &e, const overroll_property &overrun)
{
    try{
        DirectedGraph::Graph &graph = *this;
        edge_property& e_prop = graph[e];
        e_prop.overruns.push_back(overrun);

//        if(e_prop.bidirectional){//for some reason graph[e_prop.revEdge] doesn't work
//            edge_property& e_prop_rev = graph[e_prop.revEdge];
//            e_prop_rev.overruns.push_back(overrun);
//        }

        // Add overrun to reverse edge
        DirectedGraph::edge_t reverse_edge;
        bool reverse_edge_found;
        vertex_t v1 = source(e, *this);
        vertex_t v2 = target(e, *this);
        boost::tie(reverse_edge, reverse_edge_found) = boost::edge(v2, v1, graph);
        if (reverse_edge_found)
            graph[reverse_edge].overruns.push_back(overrun);

        return true;
    }
    catch(...){
        return false;
    }
}

bool Graph::addOverrun(const vertex_t &v1, const vertex_t &v2, const overroll_property &overrun)
{
    DirectedGraph::Graph &graph = *this;

    DirectedGraph::edge_t e;
    bool edge_found;
    boost::tie(e, edge_found) = boost::edge(v1, v2, graph);
    if(edge_found)
        graph[e].overruns.push_back(overrun);

    // Add overrun to reverse edge
    DirectedGraph::edge_t reverse_edge;
    bool reverse_edge_found;
    boost::tie(reverse_edge, reverse_edge_found) = boost::edge(v2, v1, graph);
    if (reverse_edge_found)
        graph[reverse_edge].overruns.push_back(overrun);

    return edge_found || reverse_edge_found;
}

const OutFieldInfo &Graph::outFieldInfo() const {
    return m_meta.outFieldInfo;
}

std::map<vertex_t, std::vector<VisitPeriod> > Graph::removeVisitPeriodsAfterTimestamp(std::set<MachineId_t> machineIds, double timestamp)
{
    if(machineIds.empty())
        return {};

    std::map<vertex_t, std::vector<VisitPeriod> > ret;
    for(auto vt_it = vertices(*this); vt_it.first != vt_it.second; vt_it.first++){
        std::multimap<double, DirectedGraph::VisitPeriod>& visitPeriods = (*this)[*vt_it.first].visitPeriods;

        for(auto it_vp = visitPeriods.begin() ; it_vp != visitPeriods.end();){
            DirectedGraph::VisitPeriod& vp = it_vp->second;
            if( machineIds.find(vp.machineId) != machineIds.end()
                    && ( vp.timestamp > timestamp + 1e-6 ) ){
                ret[*vt_it.first].emplace_back( vp );
                it_vp = visitPeriods.erase(it_vp);
            }
            else
                ++it_vp;
        }
    }
    return ret;
}

bool Graph::hasPartialHeadlands() const {return m_meta.hasPartialHeadlands;}

double Graph::workingWidth_IF() const {return m_meta.workingWidth_IF;}

double Graph::workingWidth_HL() const {return m_meta.workingWidth_HL;}

const std::map<int, std::vector<vertex_t> > &Graph::tracks_vertices_map() const {return m_meta.tracks_vertices_map;}

const std::set<int>& Graph::extremaTrackIds_IF() const {return m_meta.extremaTrackIds_IF;}

const std::map<int, std::set<int> > &Graph::adjacentTrackIds_IF() const {return m_meta.adjacentTrackIds_IF;}

const std::map<int, std::set<int> > &Graph::adjacentTrackIds_HL() const {return m_meta.adjacentTrackIds_HL;}

const std::set<int> &Graph::outermostTrackIds_HL() const {return m_meta.outermostTrackIds_HL;}

Graph::RoutePoint2VertexMap_t &Graph::routepoint_vertex_map() {return m_meta.routepoint_vertex_map;}

const Graph::RoutePoint2VertexMap_t &Graph::routepoint_vertex_map() const {return m_meta.routepoint_vertex_map;}

std::map<FieldAccessPoint, vertex_t> &Graph::accesspoint_vertex_map() {return m_meta.accesspoint_vertex_map;}

const std::map<FieldAccessPoint, vertex_t> &Graph::accesspoint_vertex_map() const {return m_meta.accesspoint_vertex_map;}

std::map<ResourcePoint, vertex_t> &Graph::resourcepoint_vertex_map() {return m_meta.resourcepoint_vertex_map;}

const std::map<ResourcePoint, vertex_t> &Graph::resourcepoint_vertex_map() const {return m_meta.resourcepoint_vertex_map;}

std::map<MachineId_t, vertex_t> &Graph::initialpoint_vertex_map() {return m_meta.initialpoint_vertex_map;}

const std::map<MachineId_t, vertex_t> &Graph::initialpoint_vertex_map() const {return m_meta.initialpoint_vertex_map;}

std::set<vertex_t> &Graph::boundary_vts() {return m_meta.boundary_vts;}

const std::set<vertex_t> &Graph::boundary_vts() const {return m_meta.boundary_vts;}


std::vector<vertex_t> Graph::getVerticesInBoundingBox(const Point &lowerLeftCorner, const Point &upperRightCorner, bool precise, const VertexFilterFct &filter) const
{
    std::vector<vertex_t> ret;

    if(lowerLeftCorner.x > upperRightCorner.x || lowerLeftCorner.y > upperRightCorner.y)
        return ret;

    int minX = std::max( (double)std::numeric_limits<int>::lowest()+1, lowerLeftCorner.x );
    int minY = std::max( (double)std::numeric_limits<int>::lowest()+1, lowerLeftCorner.y );
    int maxX = std::min( (double)std::numeric_limits<int>::max()-1, std::ceil( upperRightCorner.x ) );
    int maxY = std::min( (double)std::numeric_limits<int>::max()-1, std::ceil( upperRightCorner.y ) );

    //ret.reserve( (maxX - minX + 1) * (maxY - minY + 1) );

    for(auto it_x = m_meta.verticesLocationMap.lower_bound( minX ) ; it_x != m_meta.verticesLocationMap.end() ; it_x++){
        if(it_x->first > maxX)
            break;
        for(auto it_y = it_x->second.lower_bound( minY ) ; it_y != it_x->second.end() ; it_y++){
            if(it_y->first > maxY)
                break;
            if(precise){
                for(auto& vt : it_y->second){
                    const vertex_property& vertex_prop = (*this)[vt];
                    const Point& p = vertex_prop.route_point.point();

                    if( p.x >= lowerLeftCorner.x && p.x <= upperRightCorner.x
                            && p.y >= lowerLeftCorner.y && p.y <= upperRightCorner.y
                            && filter(vt, vertex_prop) )
                        ret.emplace_back(vt);
                }

            }
            else{
                for(auto& vt : it_y->second){
                    const vertex_property& vertex_prop = (*this)[vt];
                    if( filter(vt, vertex_prop) )
                        ret.emplace_back(vt);
                }
            }

        }
    }

    ret.shrink_to_fit();

    return ret;

}

std::vector<vertex_t> Graph::getVerticesInBoundingBox(const Point &center, float width, float height, bool precise, const VertexFilterFct &filter) const
{
    return getVerticesInBoundingBox( center - Point(0.5*width, 0.5*height), center + Point(0.5*width, 0.5*height), precise, filter );
}

std::vector<vertex_t> Graph::getVerticesInRadius(const Point &center, float radius, const VertexFilterFct &filter) const
{
    std::vector<vertex_t> ret;

    if(radius <= 0)
        return ret;

    int minX = std::max( (double)std::numeric_limits<int>::lowest()+1, center.x - radius );
    int minY = std::max( (double)std::numeric_limits<int>::lowest()+1, center.y - radius );
    int maxX = std::min( (double)std::numeric_limits<int>::max()-1, std::ceil( center.x + radius ) );
    int maxY = std::min( (double)std::numeric_limits<int>::max()-1, std::ceil( center.y + radius ) );

//    size_t reserveSize = std::max(0.0, 0.25 * M_PI * (maxX - minX + 1) * (maxY - minY + 1) );
//    ret.reserve( reserveSize );

    for(auto it_x = m_meta.verticesLocationMap.lower_bound( minX ) ; it_x != m_meta.verticesLocationMap.end() ; it_x++){
        if(it_x->first > maxX)
            break;
        for(auto it_y = it_x->second.lower_bound( minY ) ; it_y != it_x->second.end() ; it_y++){
            if(it_y->first > maxY)
                break;
            for(auto& vt : it_y->second){
                const vertex_property& vertex_prop = (*this)[vt];
                const Point& p = vertex_prop.route_point.point();
                if( arolib::geometry::calc_dist(p, center) <= radius && filter(vt, vertex_prop) )
                    ret.emplace_back(vt);
            }

        }
    }

    ret.shrink_to_fit();

    return ret;
}

std::vector<vertex_t> Graph::getVerticesInRectangle(Point p1, Point p2, float width, const Graph::VertexFilterFct &filter) const
{
    std::vector<vertex_t> ret;
    if(p1 == p2 || width <= 0)
        return ret;

    //extend the geometries to include vertices in the boundary
    float extDist = 1e-3 * std::min( geometry::calc_dist(p1, p2), (double)width );
    width += extDist;
    p1 = geometry::extend_line(p2, p1, extDist);
    p2 = geometry::extend_line(p1, p2, extDist);


    Polygon poly = arolib::geometry::createRectangleFromLine(p1, p2, width);
    double minX, maxX, minY, maxY;
    if(!arolib::geometry::getPolygonLimits(poly, minX, maxX, minY, maxY))
        return ret;

    ret = getVerticesInBoundingBox( Point(minX, minY), Point(maxX, maxY), true, [&poly, &filter](const vertex_t& vt, const vertex_property& vertex_prop)->bool{
        return filter(vt, vertex_prop) && arolib::geometry::in_polygon(vertex_prop.route_point, poly);
    } );
    return ret;
}

bool Graph::getClosestVertexInRadius(const Point &center, float radius, vertex_t &vt_out, const VertexFilterFct &filter) const
{
    auto vts = getVerticesInRadius(center, radius, filter);
    if(vts.empty())
        return false;

    double minDist = std::numeric_limits<double>::max();
    for(auto& vt : vts){
        double dist = geometry::calc_dist(center, (*this)[vt].route_point);
        if(minDist > dist){
            minDist = dist;
            vt_out = vt;
        }
    }

    return true;
}


int Graph::size() const
{
    int total_size(0);
    for(const auto& pair: m_meta.verticesLocationMap)
    {
        total_size += pair.second.size();
    }

    return total_size;
}

void Graph::regenerateVerticesLocationMap()
{
    m_meta.verticesLocationMap.clear();
    for(DirectedGraph::vertex_iter it_vt = boost::vertices(*this); it_vt.first != it_vt.second; it_vt.first++){
        const vertex_property& vp = (*this)[*it_vt.first];
        m_meta.verticesLocationMap[ (int)vp.route_point.x ][ (int)vp.route_point.y ].insert(*it_vt.first);
    }
}

const Graph::GraphMetaData &Graph::get_meta() const
{
    return m_meta;
}


}

}
