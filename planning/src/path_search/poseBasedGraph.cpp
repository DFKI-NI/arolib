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
 
#include "arolib/planning/path_search/poseBasedGraph.hpp"

namespace arolib{

using namespace arolib::geometry;

//const PoseBasedGraph::Vertex_t::VDirection PoseBasedGraph::Vertex_t::DIR_FIELD_ENTRY = PoseBasedGraph::Vertex_t::DIR_PARALLEL;
//const PoseBasedGraph::Vertex_t::VDirection PoseBasedGraph::Vertex_t::DIR_FIELD_EXIT = PoseBasedGraph::Vertex_t::DIR_PERPENDICULAR;

bool PoseBasedGraph::Vertex_t::isOfOutfieldType(bool includAccessPoints) const
{
    switch(type){
    case TYPE_OF_RESOURCE_POINT:
    case TYPE_OF_INITIAL_LOCATION:
        return true;
    case TYPE_ACCESSPOINT:
        return includAccessPoints;
    default:
        return false;
    }
}

bool PoseBasedGraph::Vertex_t::isFieldEntryType() const
{
    return type == Vertex_t::TYPE_ACCESSPOINT && direction == Vertex_t::DIR_FIELD_ENTRY;
}

bool PoseBasedGraph::Vertex_t::isFieldExitType() const
{
    return type == Vertex_t::TYPE_ACCESSPOINT && direction == Vertex_t::DIR_FIELD_EXIT;
}

PoseBasedGraph::PoseBasedGraph(std::shared_ptr<Logger> _parentLogger):
    LoggingComponent(_parentLogger, __FUNCTION__)
{


}

void PoseBasedGraph::getSubStateSpace(PoseBasedGraph &sub_ss, const PoseBasedGraph::VertexFilterFct &filter) const
{
    sub_ss.clearAll();
    sub_ss.m_idCount = m_idCount;
    for(auto& it_x : m_vertices){
        for(auto& it_y : it_x.second){
            for(auto& it_vt : it_y.second){
                if( filter(it_vt.second) ){
                    sub_ss.m_vertices[it_x.first][it_y.first][it_vt.first] = it_vt.second;
                    sub_ss.m_verticesIds[it_vt.first] = it_vt.second.pose.point();
                }
            }
        }
    }

    for(auto& it : m_verticesAccessPoints){
        for(auto vt_id : it.second){
            if(sub_ss.m_verticesIds.find(vt_id) != sub_ss.m_verticesIds.end())
                sub_ss.m_verticesAccessPoints[it.first].insert(vt_id);
        }
    }

    for(auto& it : m_verticesResourcePoints){
        if(sub_ss.m_verticesIds.find(it.second) != sub_ss.m_verticesIds.end())
            sub_ss.m_verticesResourcePoints[it.first] = it.second;
    }

    for(auto& it : m_verticesInitialLoc){
        if(sub_ss.m_verticesIds.find(it.second) != sub_ss.m_verticesIds.end())
            sub_ss.m_verticesInitialLoc[it.first] = it.second;
    }

    for(auto& it_from : m_edges_OF){
        if(sub_ss.m_verticesIds.find(it_from.first) == sub_ss.m_verticesIds.end())
            continue;
        for(auto& it_to : it_from.second){
            if(sub_ss.m_verticesIds.find(it_to.first) == sub_ss.m_verticesIds.end())
                continue;
            sub_ss.m_edges_OF[it_from.first][it_to.first] = it_to.second;
        }
    }

}

void PoseBasedGraph::clearAll()
{
    m_idCount = 0;
    m_vertices.clear();
    m_verticesIds.clear();
    m_edges_OF.clear();
    m_verticesInitialLoc.clear();
    m_verticesResourcePoints.clear();
    m_verticesAccessPoints.clear();

}

bool PoseBasedGraph::getVertex(VertexId_t vt_id, Vertex_t &vt) const
{

    auto it_id = m_verticesIds.find(vt_id);
    if(it_id == m_verticesIds.end()){
        return false;
    }

    const Point& p = it_id->second;

    auto it_x = m_vertices.find( (int)p.x );
    if(it_x == m_vertices.end())
        return false;

    auto it_y = it_x->second.find( (int)p.y );
    if(it_y == it_x->second.end())
        return false;

    auto it_id2 = it_y->second.find( vt_id );
    if(it_id2 == it_y->second.end())
        return false;

    vt = it_id2->second;
    return true;
}

std::vector<PoseBasedGraph::Vertex_t> PoseBasedGraph::getVertices(bool sortById) const
{
    std::vector<Vertex_t> vertices;

    size_t count = 0;
    for(auto &it_x : m_vertices){
        for(auto &it_y : it_x.second){
            count += it_y.second.size();
            for(auto &it_vt : it_y.second){
            }
        }
    }

    vertices.reserve(count);
    for(auto &it_x : m_vertices){
        for(auto &it_y : it_x.second){
            for(auto &it_vt : it_y.second){
                vertices.emplace_back(it_vt.second);
            }
        }
    }

    if(sortById)
        std::sort(vertices.begin(), vertices.end(), [](const Vertex_t& v1, const Vertex_t& v2)->bool {return v1.id < v2.id;});

    return vertices;
}

bool PoseBasedGraph::addTrackVertex(PoseBasedGraph::Vertex_t &vt)
{
    if(vt.type != Vertex_t::TYPE_HL_POINT && vt.type != Vertex_t::TYPE_IF_POINT){
        logger().printError(__FUNCTION__, "The vertex type does not correspond to a track vertex");
        return false;
    }

    vt.id = m_idCount++;
    m_vertices[ (int)vt.pose.x ][ (int)vt.pose.y ][vt.id] = vt;
    m_verticesIds[ vt.id ] = vt.pose;
    return true;
}

bool PoseBasedGraph::addInitialLocationVertex(PoseBasedGraph::Vertex_t &vt, MachineId_t machineId)
{
    if(vt.type != Vertex_t::TYPE_OF_INITIAL_LOCATION){
        logger().printError(__FUNCTION__, "The vertex type does not correspond to an initial-location vertex");
        return false;
    }

    if( m_verticesInitialLoc.find(machineId) != m_verticesInitialLoc.end() ){
        logger().printError(__FUNCTION__, "Vertex for the given id already exists");
        return false;
    }

    vt.id = m_idCount++;
    m_vertices[ (int)vt.pose.x ][ (int)vt.pose.y ][vt.id] = vt;
    m_verticesIds[ vt.id ] = vt.pose;

    m_verticesInitialLoc[ machineId ] = vt.id;

    return true;
}

bool PoseBasedGraph::addResourcePointVertex(PoseBasedGraph::Vertex_t &vt, ResourcePointId_t resPointId)
{
    if(vt.type != Vertex_t::TYPE_IF_RESOURCE_POINT && vt.type != Vertex_t::TYPE_OF_RESOURCE_POINT){
        logger().printError(__FUNCTION__, "The vertex type does not correspond to an access-point vertex");
        return false;
    }

    if( m_verticesResourcePoints.find(resPointId) != m_verticesResourcePoints.end() ){
        logger().printError(__FUNCTION__, "Vertex for the given id already exists");
        return false;
    }

    vt.id = m_idCount++;
    m_vertices[ (int)vt.pose.x ][ (int)vt.pose.y ][vt.id] = vt;
    m_verticesIds[ vt.id ] = vt.pose;

    m_verticesResourcePoints[ resPointId ] = vt.id;

    return true;
}

bool PoseBasedGraph::addAccessPointVertex(PoseBasedGraph::Vertex_t &vt, FieldAccessPointId_t accessPointId)
{
    if(vt.type != Vertex_t::TYPE_ACCESSPOINT){
        logger().printError(__FUNCTION__, "The vertex type does not correspond to a resource-point vertex");
        return false;
    }

    vt.id = m_idCount++;
    m_vertices[ (int)vt.pose.x ][ (int)vt.pose.y ][vt.id] = vt;
    m_verticesIds[ vt.id ] = vt.pose;
    m_verticesAccessPoints[ accessPointId ].insert(vt.id);

    return true;
}

bool PoseBasedGraph::removeVertex(VertexId_t vt_id)
{
    auto it_id = m_verticesIds.find(vt_id);
    if(it_id == m_verticesIds.end()){
        return false;
    }

    const Point& p = it_id->second;

    auto it_x = m_vertices.find( (int)p.x );
    if(it_x == m_vertices.end())
        return false;

    auto it_y = it_x->second.find( (int)p.y );
    if(it_y == it_x->second.end())
        return false;

    auto it_id2 = it_y->second.find( (int)p.y );
    if(it_id2 == it_y->second.end())
        return false;

    Vertex_t vt = it_id2->second;

    it_y->second.erase(it_id2);
    if(it_y->second.empty())
        it_x->second.erase(it_y);

    if(it_x->second.empty())
        m_vertices.erase(it_x);

    bool removeOFConnection = false;
    if(vt.type == Vertex_t::TYPE_ACCESSPOINT){
        for(auto it_fap = m_verticesAccessPoints.begin(); it_fap != m_verticesAccessPoints.end(); it_fap++){
            it_fap->second.erase(vt_id);
            if(it_fap->second.empty()){
                m_verticesAccessPoints.erase(it_fap);
                break;
            }
        }
        removeOFConnection = true;
    }
    else if(vt.type == Vertex_t::TYPE_OF_RESOURCE_POINT){
        for(auto it_rp = m_verticesResourcePoints.begin(); it_rp != m_verticesResourcePoints.end(); it_rp++){
            if(it_rp->second == vt_id){
                m_verticesResourcePoints.erase(it_rp);
                break;
            }
        }
        removeOFConnection = true;
    }
    else if(vt.type == Vertex_t::TYPE_OF_INITIAL_LOCATION){
        for(auto it_il = m_verticesInitialLoc.begin(); it_il != m_verticesInitialLoc.end(); it_il++){
            if(it_il->second == vt_id){
                m_verticesInitialLoc.erase(it_il);
                break;
            }
        }
        removeOFConnection = true;
    }

    if(removeOFConnection){
        m_edges_OF.erase(vt_id);
        for(auto& it_edge : m_edges_OF){
            it_edge.second.erase(vt_id);
        }
    }


    return true;
}

bool PoseBasedGraph::connectOFVertices(VertexId_t vt_from_id, VertexId_t vt_to_id, bool bidirectional, const MachineOutfieldCostsMap &machinesCostsOF)
{
    Vertex_t vt_from, vt_to;
    if(machinesCostsOF.empty())
        return false;
    if(!getVertex(vt_from_id, vt_from) || !vt_from.isOfOutfieldType(true))
        return false;
    if(!getVertex(vt_to_id, vt_to) || !vt_to.isOfOutfieldType(true))
        return false;

    m_edges_OF[vt_from_id][vt_to_id] = machinesCostsOF;

    if(bidirectional)
        m_edges_OF[vt_to_id][vt_from_id] = machinesCostsOF;

    return true;
}

bool PoseBasedGraph::create(const Subfield &sf, const OutFieldInfo &outFieldInfo)
{
    auto boundary = sf.boundary_outer;
    arolib::geometry::correct_polygon(boundary);
    if( arolib::geometry::isPolygonValid(boundary) == arolib::geometry::PolygonValidity::INVALID_POLYGON ){
        logger().printError(__FUNCTION__, "Invalid subfield boundary");
        return false;
    }

    clearAll();

    //add hl-tracks vertices
    if(!sf.headlands.complete.tracks.empty()){
        addTracksVertices(sf.headlands.complete.tracks, Vertex_t::TYPE_HL_POINT);
    }
    else{
        addTrackVertices(sf.headlands.complete.middle_track.points, Vertex_t::TYPE_HL_POINT);
    }

    //add if-tracks vertices
    addTracksVertices(sf.tracks, Vertex_t::TYPE_IF_POINT);

    //add out of field vertices/connections
    addAccessPointsVertices(sf.access_points, outFieldInfo, boundary);
    addResourcePointsVertices(sf, outFieldInfo);
}

bool PoseBasedGraph::updateArrivalVertices(const OutFieldInfo &outFieldInfo, const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates, const Polygon &_boundary)
{
    //remove all initial-location vertices
    while( !m_verticesInitialLoc.empty() )
        removeVertex( m_verticesInitialLoc.begin()->second );

    auto boundary = _boundary;
    arolib::geometry::correct_polygon(boundary);
    if(arolib::geometry::isPolygonValid(boundary) == arolib::geometry::PolygonValidity::INVALID_POLYGON)
        return false;

    addInitialLocationsVertices(outFieldInfo, machineCurrentStates, boundary);

    return true;
}

bool PoseBasedGraph::vertexIdExists(PoseBasedGraph::VertexId_t id) const
{
    return m_verticesIds.find(id) != m_verticesIds.end();
}

const std::unordered_map<PoseBasedGraph::VertexId_t, Point> &PoseBasedGraph::getVertexId_PointMap() const
{
    return m_verticesIds;
}

const std::unordered_map<PoseBasedGraph::VertexId_t, std::unordered_map<PoseBasedGraph::VertexId_t, PoseBasedGraph::MachineOutfieldCostsMap > > &PoseBasedGraph::getOFConnections() const
{
    return m_edges_OF;
}

std::vector<PoseBasedGraph::Vertex_t> PoseBasedGraph::getVerticesInBoundingBox(const Point &lowerLeftCorner, const Point &upperRightCorner, bool precise, const VertexFilterFct &filter) const
{
    std::vector<PoseBasedGraph::Vertex_t> ret;

    if(lowerLeftCorner.x > upperRightCorner.x || lowerLeftCorner.y > upperRightCorner.y)
        return ret;

    int minX = std::max( (double)std::numeric_limits<int>::lowest()+1, lowerLeftCorner.x );
    int minY = std::max( (double)std::numeric_limits<int>::lowest()+1, lowerLeftCorner.y );
    int maxX = std::min( (double)std::numeric_limits<int>::max()-1, std::ceil( upperRightCorner.x ) );
    int maxY = std::min( (double)std::numeric_limits<int>::max()-1, std::ceil( upperRightCorner.y ) );

    ret.reserve( (maxX - minX + 1) * (maxY - minY + 1) );

    for(auto it_x = m_vertices.lower_bound( minX ) ; it_x != m_vertices.end() ; it_x++){
        if(it_x->first > maxX)
            break;
        for(auto it_y = it_x->second.lower_bound( minY ) ; it_y != it_x->second.end() ; it_y++){
            if(it_y->first > maxY)
                break;
            if(precise){
                for(auto& it_vt : it_y->second){
                    const Point& p = it_vt.second.pose;
                    if( p.x >= lowerLeftCorner.x && p.x <= upperRightCorner.x
                            && p.y >= lowerLeftCorner.y && p.y <= upperRightCorner.y
                             && filter(it_vt.second) )
                        ret.emplace_back(it_vt.second);
                }

            }
            else{
                for(auto& it_vt : it_y->second){
                    if( filter(it_vt.second) )
                        ret.emplace_back(it_vt.second);
                }
            }

        }
    }

    ret.shrink_to_fit();

    return ret;

}

std::vector<PoseBasedGraph::Vertex_t> PoseBasedGraph::getVerticesInBoundingBox(const Point &center, float width, float height, bool precise, const VertexFilterFct &filter) const
{
    return getVerticesInBoundingBox( center - Point(0.5*width, 0.5*height), center + Point(0.5*width, 0.5*height), precise, filter );
}

std::vector<PoseBasedGraph::Vertex_t> PoseBasedGraph::getVerticesInRadius(const Point &center, float radius, const VertexFilterFct &filter) const
{
    std::vector<PoseBasedGraph::Vertex_t> ret;

    if(radius <= 0)
        return ret;

    int minX = std::max( (double)std::numeric_limits<int>::lowest()+1, center.x - radius );
    int minY = std::max( (double)std::numeric_limits<int>::lowest()+1, center.y - radius );
    int maxX = std::min( (double)std::numeric_limits<int>::max()-1, std::ceil( center.x + radius ) );
    int maxY = std::min( (double)std::numeric_limits<int>::max()-1, std::ceil( center.y + radius ) );

    size_t reserveSize = std::max(0.0, 0.25 * M_PI * (maxX - minX + 1) * (maxY - minY + 1) );
    ret.reserve( reserveSize );

    for(auto it_x = m_vertices.lower_bound( minX ) ; it_x != m_vertices.end() ; it_x++){
        if(it_x->first > maxX)
            break;
        for(auto it_y = it_x->second.lower_bound( minY ) ; it_y != it_x->second.end() ; it_y++){
            if(it_y->first > maxY)
                break;
            for(auto& it_vt : it_y->second){
                const Point& p = it_vt.second.pose;
                if( arolib::geometry::calc_dist(p, center) <= radius && filter(it_vt.second) )
                    ret.emplace_back(it_vt.second);
            }

        }
    }

    ret.shrink_to_fit();

    return ret;
}

std::vector<PoseBasedGraph::Vertex_t> PoseBasedGraph::getVerticesInRectangle(const Point &p1, const Point &p2, float width, const PoseBasedGraph::VertexFilterFct &filter) const
{
    std::vector<Vertex_t> ret;
    if(p1 == p2 || width <= 0)
        return ret;

    Polygon poly = arolib::geometry::createRectangleFromLine(p1, p2, width);
    double minX, maxX, minY, maxY;
    if(!arolib::geometry::getPolygonLimits(poly, minX, maxX, minY, maxY))
        return ret;

    ret = getVerticesInBoundingBox( Point(minX, minY), Point(maxX, maxY), true, [&poly, &filter](const Vertex_t& vt)->bool{
        return filter(vt) && arolib::geometry::in_polygon(vt.pose, poly);
    } );
    return ret;
}

std::vector<PoseBasedGraph::Vertex_t> PoseBasedGraph::getVerticesClosestToPoint(const Point &p, size_t maxVts, const PoseBasedGraph::VertexFilterFct &filter) const
{
    std::vector<Vertex_t> ret;
    if(maxVts == 0)
        return ret;

    std::multimap<double, Vertex_t> vtsMap;

    int startX = p.x;
    int startY = p.y;

    Point lowerLeftCorner, upperRightCorner;
    if(!getVerticesBoundingBox(lowerLeftCorner, upperRightCorner))
        return ret;


    int stepX = 0;
    int stepY = 0;

    int minX = lowerLeftCorner.x;
    int minY = lowerLeftCorner.y;
    int maxX = upperRightCorner.x;
    int maxY = upperRightCorner.y;

    if( startX < minX )
        stepX = minX - startX;
    else if( startX > maxX )
        stepX = startX - maxX;

    if( startY < minY )
        stepY = minY - startY;
    else if( startY > maxY )
        stepY = startY - maxY;

    int step = std::min(stepX, stepY);

    //@todo finish algorithm
    while(0){
        bool checkX_left;

        step++;
    }

    ret.reserve( vtsMap.size() );
    for(auto &it : vtsMap)
        ret.emplace_back(it.second);

    return ret;
}

std::unordered_map<FieldAccessPointId_t, std::set<PoseBasedGraph::VertexId_t> > PoseBasedGraph::getEntryVertices() const
{
    std::unordered_map<FieldAccessPointId_t, std::set<PoseBasedGraph::VertexId_t> > ret;
    for(const auto &it_fap : m_verticesAccessPoints){
        for(auto vt_id : it_fap.second){
            Vertex_t vtTmp;
            if(getVertex(vt_id, vtTmp) && vtTmp.direction == Vertex_t::DIR_FIELD_ENTRY)
                ret[it_fap.first].insert(vt_id);
        }
    }
    return ret;
}

std::unordered_map<FieldAccessPointId_t, std::set<PoseBasedGraph::VertexId_t> > PoseBasedGraph::getExitVertices() const
{
    std::unordered_map<FieldAccessPointId_t, std::set<PoseBasedGraph::VertexId_t> > ret;
    for(const auto &it_fap : m_verticesAccessPoints){
        for(auto vt_id : it_fap.second){
            Vertex_t vtTmp;
            if(getVertex(vt_id, vtTmp) && vtTmp.direction == Vertex_t::DIR_FIELD_EXIT)
                ret[it_fap.first].insert(vt_id);
        }
    }
    return ret;
}

void PoseBasedGraph::getEntryAndExistVertices(std::unordered_map<FieldAccessPointId_t, std::set<PoseBasedGraph::VertexId_t> > &entryVts,
                                              std::unordered_map<FieldAccessPointId_t, std::set<PoseBasedGraph::VertexId_t> > &exitVts) const
{
    for(const auto &it_fap : m_verticesAccessPoints){
        for(auto vt_id : it_fap.second){
            Vertex_t vtTmp;
            if(getVertex(vt_id, vtTmp) && vtTmp.direction == Vertex_t::DIR_FIELD_ENTRY)
                entryVts[it_fap.first].insert(vt_id);
            else
                exitVts[it_fap.first].insert(vt_id);
        }
    }
}

const std::unordered_map<ResourcePointId_t, PoseBasedGraph::VertexId_t> &PoseBasedGraph::getOFResourcePointVertices() const
{
    return m_verticesResourcePoints;
}

const std::unordered_map<MachineId_t, PoseBasedGraph::VertexId_t> &PoseBasedGraph::getOFInitialLocationVertices() const
{
    return m_verticesInitialLoc;
}

bool PoseBasedGraph::getAccessPointId(PoseBasedGraph::VertexId_t fap_vt_id, FieldAccessPointId_t &id) const
{
    bool found = false;
    for(auto &it : m_verticesAccessPoints){
        if(it.second.find(fap_vt_id) != it.second.end()){
            id = it.first;
            found = true;
            break;
        }
    }
    return found;
}

bool PoseBasedGraph::getResourcePointId(PoseBasedGraph::VertexId_t resP_vt_id, ResourcePointId_t &id) const
{
    bool found = false;
    for(auto &it : m_verticesResourcePoints){
        if(it.second == resP_vt_id){
            id = it.first;
            found = true;
            break;
        }
    }
    return found;

}

bool PoseBasedGraph::getMachineIdFromInitLoc(PoseBasedGraph::VertexId_t initLoc_vt_id, MachineId_t &id) const
{
    bool found = false;
    for(auto &it : m_verticesInitialLoc){
        if(it.second == initLoc_vt_id){
            id = it.first;
            found = true;
            break;
        }
    }
    return found;

}

bool PoseBasedGraph::getVerticesBoundingBox(Point &lowerLeftCorner, Point &upperRightCorner) const
{
    //@todo add an internal parameter holding this information, that is updated when a vertex is added or removed

    lowerLeftCorner.x = lowerLeftCorner.y = std::numeric_limits<double>::max();
    upperRightCorner.x = upperRightCorner.y = std::numeric_limits<double>::lowest();

    for(auto& it_x : m_vertices){
        for(auto& it_y : it_x.second){
            for(auto& it_vt : it_y.second){
                lowerLeftCorner.x = std::min(lowerLeftCorner.x, it_vt.second.pose.x);
                lowerLeftCorner.y = std::min(lowerLeftCorner.y, it_vt.second.pose.y);
                upperRightCorner.x = std::max(upperRightCorner.x, it_vt.second.pose.x);
                upperRightCorner.y = std::max(upperRightCorner.y, it_vt.second.pose.y);
            }
        }

    }

    return lowerLeftCorner.x <= upperRightCorner.x+1e-10;

}

void PoseBasedGraph::addTracksVertices(const std::vector<Track> &tracks, Vertex_t::VType vType)
{
    for( auto& track : tracks ){
        addTrackVertices(track.points, vType);
    }
}

void PoseBasedGraph::addTrackVertices(std::vector<Point> points, Vertex_t::VType vType)
{

    arolib::geometry::remove_repeated_points(points);
    if(points.size() < 2)
        return;

    if( points.front() == points.back() ){
        addTrackVertex(r_at(points, 1), points.front(), points.at(1), vType);
    }
    else{
        addTrackVertex(points.front(), points.front(), points.at(1), vType);
    }

    for( size_t i = 1 ; i+1 < points.size() ; ++i ){
        addTrackVertex(points.at(i-1), points.at(i), points.at(i+1), vType);
    }

    if( points.front() != points.back() ){
        addTrackVertex(r_at(points, 1), points.back(), points.back(), vType);
    }
}

void PoseBasedGraph::addTrackVertex(const Point &pPrev, const Point &p, const Point &pNext, Vertex_t::VType vType)
{
    Vertex_t vt;

    if(p == pPrev && p == pNext)
        return;

    vt.pose.point() = p;
    vt.type = vType;

    vt.direction = Vertex_t::DIR_PARALLEL;
    if(p != pPrev){
        vt.pose.angle = arolib::geometry::get_angle(p, pPrev);//parallel prev
        addTrackVertex(vt);
    }
    else if(vType == Vertex_t::TYPE_IF_POINT){
        vt.pose.angle = arolib::geometry::get_angle(pNext, p);//parallel prev
        addTrackVertex(vt);
    }

    if(p != pNext){
        vt.pose.angle = arolib::geometry::get_angle(p, pNext);//parallel next
        addTrackVertex(vt);
    }
    else if(vType == Vertex_t::TYPE_IF_POINT){
        vt.pose.angle = arolib::geometry::get_angle(pPrev, p);//parallel next
        addTrackVertex(vt);
    }

    vt.direction = Vertex_t::DIR_PERPENDICULAR;
    Point pRot1, pRot2;
    if(p != pPrev && p != pNext){
        auto rotAngle = arolib::geometry::get_angle(pPrev, p, pNext) * 0.5;
        pRot1 = arolib::geometry::rotate( p, pPrev, rotAngle );
        pRot2 = arolib::geometry::rotate( p, pPrev, rotAngle - M_PI );
    }
    else if(p != pPrev){
        pRot1 = arolib::geometry::rotate( p, pPrev, M_PI_2 );
        pRot2 = arolib::geometry::rotate( p, pPrev, -M_PI_2 );
    }
    else{
        pRot1 = arolib::geometry::rotate( p, pNext, M_PI_2 );
        pRot2 = arolib::geometry::rotate( p, pNext, -M_PI_2 );
    }

    vt.pose.angle = arolib::geometry::get_angle(p, pRot1);//perpendicular 1
    addTrackVertex(vt);

    vt.pose.angle = arolib::geometry::get_angle(p, pRot2);//perpendicular 2
    addTrackVertex(vt);

}

void PoseBasedGraph::addAccessPointsVertices(std::vector<FieldAccessPoint> faps, const OutFieldInfo &outFieldInfo, Polygon boundary)
{
    //correct access points location so that they lay in the boundary

    for(auto& fap : faps){
        int idx = arolib::geometry::addSampleToGeometryClosestToPoint(boundary.points, fap, 1);
        if(idx < 0){
            logger().printError(__FUNCTION__, "Error obtaining boundary point closes to access point " + std::to_string(fap.id));
            continue;
        }

        //correct access point location so that it lies in the boundary
        fap.point() = boundary.points.at(idx);

        arolib::geometry::openPolygon(boundary);
        const Point& p = boundary.points.at(idx);
        const Point& pPrev = getPointFromContinuousGeometry(boundary.points, idx-1);
        const Point& pNext = getPointFromContinuousGeometry(boundary.points, idx+1);
        arolib::geometry::closePolygon(boundary);

        auto rotAngle = arolib::geometry::get_angle( pPrev , p, pNext ) * 0.5;
        auto pRot1 = arolib::geometry::rotate( p, pPrev, rotAngle );
        auto pRot2 = arolib::geometry::rotate( p, pPrev, rotAngle - M_PI );
        if(rotAngle < 0)
            std::swap(pRot1, pRot2);


        Vertex_t vt;
        vt.pose.point() = fap.point();
        vt.type = Vertex_t::TYPE_ACCESSPOINT;

        vt.direction = Vertex_t::DIR_FIELD_ENTRY;//towards inside of the field
        vt.pose.angle = arolib::geometry::get_angle(p, pRot1);
        addAccessPointVertex(vt, fap.id);

        vt.direction = Vertex_t::DIR_FIELD_EXIT;//towards outside of the field
        vt.pose.angle = arolib::geometry::get_angle(p, pRot2);
        addAccessPointVertex(vt, fap.id);
    }

    //inter-connect access points

    std::unordered_map< FieldAccessPointId_t, std::set<VertexId_t> > entryVts, exitVts;
    getEntryAndExistVertices(entryVts, exitVts);

    for(size_t i = 0 ; i < faps.size() ; ++i){
        auto fap1 = faps.at(i);
        if(fap1.accessType == FieldAccessPoint::AP_ENTRY_ONLY)
            continue;
        auto it_fap1 = exitVts.find(fap1.id);
        if(it_fap1 == exitVts.end()){
            continue;
        }
        for(size_t j = 1 ; j < faps.size() ; ++j){
            if(i == j)
                continue;

            auto fap2 = faps.at(j);
            if(fap2.accessType == FieldAccessPoint::AP_EXIT_ONLY)
                continue;
            auto it_fap2 = entryVts.find(fap2.id);
            if(it_fap2 == entryVts.end()){
                continue;
            }

            if( outFieldInfo.size_FAP2FAP(fap1.id, fap2.id) > 0 ){
                try{
                    OutFieldInfo::MapMachineTravelCosts_t machinesMap;
                    if(!outFieldInfo.getTravelCost_FAP2FAP(fap1.id, fap2.id, machinesMap))
                        continue;

                    MachineOutfieldCostsMap machinesCosts;
                    for(auto& it_m : machinesMap){
                        OutFieldCosts costs;
                        costs.travelCostsMap = it_m.second;
                        machinesCosts[it_m.first] = costs;
                    }

                    for(auto& fap1_id : it_fap1->second){
                        for(auto& fap2_id : it_fap2->second){
                            connectOFVertices(fap1_id, fap2_id, false, machinesCosts);
                        }
                    }
                }
                catch(std::exception& e){
                    logger().printError(__FUNCTION__, "Unexpected error adding connection between access point " + std::to_string(fap1.id) + " and access point " + std::to_string(fap2.id) + ": " + e.what());
                }
            }
        }
    }

}

void PoseBasedGraph::addResourcePointsVertices(const Subfield &sf, const OutFieldInfo &outFieldInfo)
{
    auto boundary_outer = sf.boundary_outer;
    arolib::geometry::correct_polygon(boundary_outer);

    for(const auto& resP : sf.resource_points){
        if( arolib::geometry::in_polygon(resP, boundary_outer) ){

            //@todo: does it make sense to add these vertices? if they are inside
            //addIFResourcePointVertices(sf, resP);
        }
        else{
            bool vt_added = false;
            Vertex_t vt;

            std::unordered_map< FieldAccessPointId_t, std::set<VertexId_t> > entryVts, exitVts;
            getEntryAndExistVertices(entryVts, exitVts);

            for(auto &fap : sf.access_points){
                auto it_fap = entryVts.find(fap.id);
                if( it_fap != entryVts.end() && fap.accessType != FieldAccessPoint::AP_EXIT_ONLY && outFieldInfo.size_RP2FAP(resP.id, fap.id) > 0 ){
                    if(!vt_added){
                        vt.type = Vertex_t::TYPE_OF_RESOURCE_POINT;
                        vt.pose.point() = resP.point();
                        vt.pose.angle = std::nanf("1");
                        vt_added = addResourcePointVertex(vt, resP.id);
                    }
                    try{
                        OutFieldInfo::MapMachineTravelCosts_t machinesMap;
                        if(!outFieldInfo.getTravelCost_RP2FAP(resP.id, fap.id, machinesMap))
                            continue;

                        MachineOutfieldCostsMap machinesCosts;
                        for(auto& it_m : machinesMap){
                            OutFieldCosts costs;
                            costs.travelCostsMap = it_m.second;
                            machinesCosts[it_m.first] = costs;
                        }

                        for(auto& fap_id : it_fap->second)
                            connectOFVertices(vt.id, fap_id, false, machinesCosts);
                    }
                    catch(std::exception& e){
                        logger().printError(__FUNCTION__, "Unexpected error adding connection between resource point " + std::to_string(resP.id) + " and  access point " + std::to_string(fap.id) + ": " + e.what());
                    }
                }
                it_fap = exitVts.find(fap.id);
                if( it_fap != exitVts.end() && fap.accessType != FieldAccessPoint::AP_ENTRY_ONLY && outFieldInfo.size_FAP2RP(fap.id, resP.id) > 0 ){
                    if(!vt_added){
                        vt.type = Vertex_t::TYPE_OF_RESOURCE_POINT;
                        vt.pose.point() = resP.point();
                        vt.pose.angle = std::nanf("1");
                        vt_added = true;
                    }
                    try{
                        OutFieldInfo::MapMachineTravelCosts_t machinesMap;
                        if(!outFieldInfo.getTravelCost_FAP2RP(fap.id, resP.id, machinesMap))
                            continue;

                        MachineOutfieldCostsMap machinesCosts;
                        for(auto& it_m : machinesMap){
                            OutFieldCosts costs;
                            costs.travelCostsMap = it_m.second;
                            machinesCosts[it_m.first] = costs;
                        }

                        for(auto& fap_id : it_fap->second)
                            connectOFVertices(fap_id, vt.id, false, machinesCosts);
                    }
                    catch(std::exception& e){
                        logger().printError(__FUNCTION__, "Unexpected error adding connection between access point " + std::to_string(fap.id) + " and resource point " + std::to_string(resP.id) + " : " + e.what());
                    }
                }
            }

        }
    }
}

void PoseBasedGraph::addIFResourcePointVertices(const Subfield &sf, const ResourcePoint &resP)
{
    auto boundary_inner = sf.boundary_inner;
    arolib::geometry::correct_polygon(boundary_inner);

    bool inHeadland = true;

    if( arolib::geometry::isPolygonValid(boundary_inner) != arolib::geometry::PolygonValidity::INVALID_POLYGON ){
        inHeadland = arolib::geometry::in_polygon( resP, boundary_inner );
    }

    Vertex_t vt;
    vt.type = Vertex_t::TYPE_IF_RESOURCE_POINT;
    vt.pose.point() = resP.point();

    const std::vector<Point>* refTrack = nullptr;
    double minDist = std::numeric_limits<double>::max();

    auto updateClosestTrack = [&](const std::vector<Track>& tracks){
        for(auto& track : tracks){
            if(track.points.size() < 2)
                continue;
            double dist = arolib::geometry::calc_dist_to_linestring(track.points, resP);
            if(minDist > dist){
                minDist = dist;
                refTrack = &track.points;
            }
        }
    };

    //get the track closest to the resource point to add the angles corresponding to the parallel direction of the track
    if(inHeadland){
        updateClosestTrack(sf.headlands.complete.tracks);
        if(sf.headlands.complete.middle_track.points.size() > 2){
            double dist = arolib::geometry::calc_dist_to_linestring(sf.headlands.complete.middle_track.points, resP);
            if(minDist > dist){
                minDist = dist;
                refTrack = &sf.headlands.complete.middle_track.points;
            }
        }
        for(auto& hl : sf.headlands.partial){
            updateClosestTrack(hl.tracks);
        }
    }
    else{
        updateClosestTrack(sf.tracks);
    }

    //@todo: continue if it makes sense to add IF resource points
}

void PoseBasedGraph::addInitialLocationsVertices(const OutFieldInfo &outFieldInfo, const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates, const Polygon& boundary)
{
    auto entryVts = getEntryVertices();
    for(auto& it_m : machineCurrentStates){
        const MachineDynamicInfo& mdi = it_m.second;
        if( arolib::geometry::in_polygon(mdi.position, boundary) ){
            //@todo check if it makes sense to add this vertex
        }
        else{
            bool vt_added = false;
            Vertex_t vt;
            for(auto &it_fap : entryVts){
                bool addConnection = outFieldInfo.size_arrivalCosts(it_fap.first, it_m.first) > 0;
                if(!addConnection){//check if the machine is close to the entry point and connect it to the access point directly
                    for(auto vt_id : it_fap.second){
                        auto it_vt = m_verticesIds.find( vt_id );
                        if(it_vt == m_verticesIds.end()){
                            logger().printError(__FUNCTION__, "Field access point " + std::to_string(it_fap.first) + " not found in vertex-id map");
                            continue;
                        }
                        double distEps = 1;
                        double dist = arolib::geometry::calc_dist(mdi.position, it_vt->second);
                        if( dist < distEps ){//close enough
                            addConnection = true;
                            break;
                        }
                    }
                }

                if(!vt_added && addConnection){
                    vt.type = Vertex_t::TYPE_OF_INITIAL_LOCATION;
                    vt.pose.point() = mdi.position;
                    vt.pose.angle = std::nanf("1");
                    vt_added = addInitialLocationVertex(vt, it_m.first);
                }
                if(vt_added && addConnection){
                    for(auto fap_vt_id : it_fap.second){
                        try{
                            MachineOutfieldCostsMap machinesCosts;
                            OutFieldCosts costs;
                            if(!outFieldInfo.getArrivalCost(it_fap.first, it_m.first, costs.travelCostsMap))
                                continue;
                            machinesCosts[it_m.first] = costs;

                            connectOFVertices(vt.id, fap_vt_id, false, machinesCosts);
                        }
                        catch(std::exception& e){
                            logger().printError(__FUNCTION__, "Unexpected error adding connection for access point " + std::to_string(it_fap.first) + ": " + e.what());
                        }
                    }
                }
            }

        }
    }
}




}

