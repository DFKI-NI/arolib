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
 
#include "arolib/cartography/gridcellsinfomanager.hpp"

namespace arolib {
namespace gridmap{

GridCellsInfoManager::Edge::Edge(const Point &_p0, const Point &_p1, double _width, bool _precise){

    width = _width;
    precise = _precise;

    //organize p0 and p1 based on the location (for easier sorting and comparizon between edges)

    //order the edge points based on the distance between them and the origin [0,0], p0 being the closest
    double dist0 = arolib::geometry::calc_dist( Point(0,0), _p0 );
    double dist1 = arolib::geometry::calc_dist( Point(0,0), _p1 );
    if( dist0 < dist1 ){
        p0 = _p0;
        p1 = _p1;
        return;
    }
    if( dist0 > dist1 ){
        p0 = _p1;
        p1 = _p0;
        return;
    }

    //if both points are at the same distance from the origin, check the angles w.r.t. the x-axis (i.e. organize based on the quadrant)
    if( arolib::geometry::get_angle( Point(0,0), _p0 ) < arolib::geometry::get_angle( Point(0,0), _p1 ) ){
        p0 = _p0;
        p1 = _p1;
        return;
    }

    p0 = _p1;
    p1 = _p0;

}

bool GridCellsInfoManager::Edge::operator==(const GridCellsInfoManager::Edge &e) const
{
    const double eps = 1e-6;
    if(std::fabs(width - e.width) > eps || precise != e.precise)
        return false;
    return ( p0==e.p0 && p1==e.p1 ) ||
           ( p0==e.p1 && p1==e.p0 );//If p1 == e.p0 and p0 == e.p1, it is considered that the edge points are equal.
}

bool GridCellsInfoManager::Edge::operator<(const GridCellsInfoManager::Edge &e) const
{
    if( *this == e )
        return false;

    if(precise != e.precise)
        return !precise;

    //compare first the distance between the p0 points and the origin [0,0]
    double dist0 = arolib::geometry::calc_dist( Point(0,0), p0 );
    double dist1 = arolib::geometry::calc_dist( Point(0,0), e.p0 );
    if( dist0 < dist1 )
        return true;
    if( dist0 > dist1 )
        return false;

    //next, compare the distance between the p1 points and the origin [0,0]
    dist0 = arolib::geometry::calc_dist( Point(0,0), p1 );
    dist1 = arolib::geometry::calc_dist( Point(0,0), e.p1 );
    if( dist0 < dist1 )
        return true;
    if( dist0 > dist1 )
        return false;

    //next, compare the angles of p0 points w.r.t. the x-axis using the origin as pivot
    dist0 = arolib::geometry::get_angle( Point(0,0), p0 );
    dist1 = arolib::geometry::get_angle( Point(0,0), e.p0 );
    if( dist0 < dist1 )
        return true;
    if( dist0 > dist1 )
        return false;

    //next, compare the angles of p1 points w.r.t. the x-axis using the origin as pivot
    dist0 = arolib::geometry::get_angle( Point(0,0), p1 );
    dist1 = arolib::geometry::get_angle( Point(0,0), e.p1 );
    if( dist0 < dist1 )
        return true;
    if( dist0 > dist1 )
        return false;

    //still all the same? then just compare the widths
    if(width < e.width)
        return true;
    if(width > e.width)
        return false;

    //the edges are equal
    return false;
}

bool GridCellsInfoManager::Edge::operator>(const GridCellsInfoManager::Edge &e) const
{
    if( *this == e )//check first if the edges are equal (because operator< returns false if they are)
        return false;
    return !(*this < e);
}

std::size_t GridCellsInfoManager::Edge::KeyHash::operator()(const GridCellsInfoManager::Edge &e) const
{
    return get(e, 0);
}

std::size_t GridCellsInfoManager::Edge::KeyHash::get(const GridCellsInfoManager::Edge &e, std::size_t seed)
{
    seed = Point::KeyHash::get( e.p0, seed );
    seed = Point::KeyHash::get( e.p1, seed );
    boost::hash_combine(seed, e.width);
    boost::hash_combine(seed, e.precise);
    return seed;
}

GridCellsInfoManager::GridCellsInfoManager(const GridCellsInfoManager& other) 
    : LoggingComponent(other.logger().logLevel())
{
    std::unique_lock<std::mutex> guard(other.m_mutex);

    m_gridProperties = other.m_gridProperties;
    m_edgeProperties = other.m_edgeProperties;
    
    guard.unlock();
}

GridCellsInfoManager::GridCellsInfoManager(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{

}

GridCellsInfoManager::~GridCellsInfoManager()
{

}

bool GridCellsInfoManager::registerGrid(const std::string &name, const Point &lowerLeftCorner, size_t numCellsX, size_t numCellsY, double cellsize, bool replaceIfExists)
{
    std::unique_lock<std::mutex> guard(m_mutex);

    if( name.empty() || numCellsX == 0 || numCellsY == 0 || cellsize <= 0 ){
        logger().printError(__FUNCTION__, "Invalid arguments");
        return false;
    }

    GridSearchProperties gp( Point(cellsize*numCellsX, cellsize*numCellsY), cellsize);

    auto it = m_gridProperties.find( name );
    if(it != m_gridProperties.end()){
        if(!replaceIfExists)
            return true;

        if(it->second.first == gp)//the grid exists and has the same properties --> do nothing
            return true;

        guard.unlock();
        if(!removeGrid(name))
            return false;
        return registerGrid(name, lowerLeftCorner, numCellsX, numCellsY, cellsize, replaceIfExists);
    }

    m_gridProperties[name] = std::make_pair(gp, lowerLeftCorner);

    auto it2 = m_edgeProperties.find(gp);
    if( it2 == m_edgeProperties.end() )
        m_edgeProperties[gp] = std::make_pair(1, EdgeProperties());
    else
        it2->second.first++;

    return true;
}

bool GridCellsInfoManager::registerGrid(const std::string &name, const GridmapLayout &lo, bool replaceIfExists){
    if(!lo.isValid())
        return false;
    return registerGrid( name, lo.getLowerLeftCorner(), lo.getSizeX(), lo.getSizeY(), lo.getCellsize(), replaceIfExists );
}

bool GridCellsInfoManager::removeGrid(const std::string &name)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    auto it = m_gridProperties.find( name );
    if(it == m_gridProperties.end())
        return true;

    auto it2 = m_edgeProperties.find( it->second.first );
    if( it2 == m_edgeProperties.end() || it2->second.first == 0 ){
        logger().printError(__FUNCTION__, "Edge properties not found in internal map");
        return false;
    }

    it2->second.first--;
    if( it2->second.first == 0 )
        m_edgeProperties.erase( it2 );

    m_gridProperties.erase(it);

    return true;
}

bool GridCellsInfoManager::isGridRegistered(const std::string &gridName)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    auto it = m_gridProperties.find( gridName );
    if(it == m_gridProperties.end())
        return false;

    auto it2 = m_edgeProperties.find( it->second.first );
    if(it2 == m_edgeProperties.end()){
        logger().printCritic(__FUNCTION__, "Grid properties not found in internal map");
        return false;
    }
    return true;
}

bool GridCellsInfoManager::checkGridGeometry(const std::string &gridName, const GridmapLayout &lo){
    if(!lo.isValid())
        return false;

    std::lock_guard<std::mutex> guard(m_mutex);

    auto it = m_gridProperties.find( gridName );
    if(it == m_gridProperties.end()){
        logger().printError(__FUNCTION__, "Grid not registered");
        return false;
    }

    GridSearchProperties gp( lo );
    return gp == it->second.first;
}

bool GridCellsInfoManager::updateCellsInfo(const std::string &gridName,
                                           const Edge &edge,
                                           const std::vector<gridmap::GridmapLayout::GridCellOverlap> &cellsInfo)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    auto it = m_gridProperties.find( gridName );
    if(it == m_gridProperties.end()){
        logger().printError(__FUNCTION__, "Grid not registered");
        return false;
    }

    auto it2 = m_edgeProperties.find( it->second.first );
    if(it2 == m_edgeProperties.end()){
        logger().printCritic(__FUNCTION__, "Grid properties not found in internal map");
        return false;
    }

    //move the edge to the reference frame of the grid
    Point pTranslate = it->second.second;
    Edge edge_ed( edge.p0-pTranslate, edge.p1-pTranslate, edge.width, edge.precise );

    EdgeProperties& edgeProp = it2->second.second;
    edgeProp.gridCellsInfo[edge_ed] = cellsInfo;

    return true;
}

bool GridCellsInfoManager::computeAndUpdateCellsInfo(const std::string &gridName, const GridCellsInfoManager::Edge &edge, const GridmapLayout &lo, bool computeOnlyIfNotExisting, std::vector<GridmapLayout::GridCellOverlap> *cellsInfo){
    if(!lo.isValid()){
        logger().printError(__FUNCTION__, "Invalid gridmap layout");
        return false;
    }

    std::lock_guard<std::mutex> guard(m_mutex);

    auto it = m_gridProperties.find( gridName );
    if(it == m_gridProperties.end()){
        logger().printError(__FUNCTION__, "Grid not registered");
        return false;
    }

    auto it2 = m_edgeProperties.find( it->second.first );
    if(it2 == m_edgeProperties.end()){
        logger().printCritic(__FUNCTION__, "Grid properties not found in internal map");
        return false;
    }

    GridSearchProperties gp( lo );
    if( gp != it->second.first ){
        logger().printError(__FUNCTION__, "Grid geometric parameters do not concurr with saved ones");
        return false;
    }

    //move the edge to the reference frame of the grid
    Point pTranslate = it->second.second;
    Edge edge_ed( edge.p0-pTranslate, edge.p1-pTranslate, edge.width, edge.precise );

    EdgeProperties& edgeProp = it2->second.second;

    auto it3 = edgeProp.gridCellsInfo.find(edge_ed);
    if(it3 != edgeProp.gridCellsInfo.end()){
        if(!computeOnlyIfNotExisting)
            computeListCellsUnderLine(lo, edge, it3->second);

        if (cellsInfo)
            *cellsInfo = it3->second;
        return true;
    }

    it3 = edgeProp.gridCellsInfo.insert( std::make_pair(edge_ed, std::vector<gridmap::GridmapLayout::GridCellOverlap>()) ).first;
    computeListCellsUnderLine(lo, edge, it3->second);

    if (cellsInfo)
        *cellsInfo = it3->second;

    return true;
}

bool GridCellsInfoManager::computeListCellsUnderLine(const GridmapLayout &lo, const GridCellsInfoManager::Edge &edge, std::vector<GridmapLayout::GridCellOverlap> &cellsInfo){
    cellsInfo.clear();
    if(!lo.isValid())
        return true;
    if(edge.precise)
        cellsInfo = lo.getCellsOverlapUnderLine(edge.p0, edge.p1, edge.width);
    else{
        CellsRangeList indexMap = lo.getCellsUnderLine(edge.p0, edge.p1, edge.width);
        if (!indexMap.empty()){
            int min_y, max_y;
            for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
                auto yRanges = indexMap.getColumn(x);
                int lastY = -1;
                for(const auto& yRange : yRanges){
                    min_y = yRange.first;
                    max_y = yRange.second;
                    if(lastY == min_y)
                        ++min_y;
                    lastY = max_y;
                    for (int y = min_y ; y <= max_y ; ++y){
                        cellsInfo.push_back( gridmap::GridmapLayout::GridCellOverlap(x,y,1) );
                    }
                }
            }
        }
    }
    return true;
}

bool GridCellsInfoManager::hasCellsInfo(const std::string &gridName, const GridCellsInfoManager::Edge &edge) const
{
    std::lock_guard<std::mutex> guard(m_mutex);

    auto it = m_gridProperties.find( gridName );
    if(it == m_gridProperties.end()){
        logger().printError(__FUNCTION__, "Grid not registered");
        return false;
    }

    auto it2 = m_edgeProperties.find( it->second.first );
    if(it2 == m_edgeProperties.end()){
        logger().printCritic(__FUNCTION__, "Grid properties not found in internal map");
        return false;
    }

    //move the edge to the reference frame of the grid
    Point pTranslate = it->second.second;
    Edge edge_ed( edge.p0-pTranslate, edge.p1-pTranslate, edge.width, edge.precise );

    const EdgeProperties& edgeProp = it2->second.second;

    auto it_edge = edgeProp.gridCellsInfo.find(edge_ed);

    return it_edge != edgeProp.gridCellsInfo.end();

}

bool GridCellsInfoManager::getCellsInfo(const std::string &gridName, const GridCellsInfoManager::Edge &edge, std::vector<gridmap::GridmapLayout::GridCellOverlap> &cellsInfo) const
{
    cellsInfo.clear();

    std::lock_guard<std::mutex> guard(m_mutex);

    auto it = m_gridProperties.find( gridName );
    if(it == m_gridProperties.end()){
        logger().printError(__FUNCTION__, "Grid not registered");
        return false;
    }

    auto it2 = m_edgeProperties.find( it->second.first );
    if(it2 == m_edgeProperties.end()){
        logger().printCritic(__FUNCTION__, "Grid properties not found in internal map");
        return false;
    }

    //move the edge to the reference frame of the grid
    Point pTranslate = it->second.second;
    Edge edge_ed( edge.p0-pTranslate, edge.p1-pTranslate, edge.width, edge.precise );

    const EdgeProperties& edgeProp = it2->second.second;

    auto it3 = edgeProp.gridCellsInfo.find(edge_ed);
    if(it3 == edgeProp.gridCellsInfo.end())
        return false;

    cellsInfo = it3->second;

    return true;
}

void GridCellsInfoManager::clearAll()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    m_gridProperties.clear();
    m_edgeProperties.clear();
}


GridCellsInfoManager::GridSearchProperties::GridSearchProperties(const Point &_corner, double _cellsize):
    corner(_corner)
  , cellsize(_cellsize)
{

}

GridCellsInfoManager::GridSearchProperties::GridSearchProperties(const GridmapLayout &lo){
    if(!lo.isValid())
        return;
    corner.x = lo.getSizeX() * lo.getCellsize();
    corner.y = lo.getSizeY() * lo.getCellsize();
    cellsize = lo.getCellsize();
}

bool GridCellsInfoManager::GridSearchProperties::operator==(const GridCellsInfoManager::GridSearchProperties &other) const
{
    const double eps = 1e-3;
    double refDist = std::min( {corner.x, corner.y, other.corner.x, other.corner.y} );

    if(std::fabs(cellsize - other.cellsize) > refDist*eps)
        return false;

    return arolib::geometry::calc_dist( corner, other.corner ) < refDist*eps;

}

bool GridCellsInfoManager::GridSearchProperties::operator!=(const GridCellsInfoManager::GridSearchProperties &other) const
{
    return !( *this == other );
}

bool GridCellsInfoManager::GridSearchProperties::operator<(const GridCellsInfoManager::GridSearchProperties &other) const
{

    if( *this == other )
        return false;

    //compare first the distance between the corners and the origin [0,0]
    double dist0 = arolib::geometry::calc_dist( Point(0,0), corner );
    double dist1 = arolib::geometry::calc_dist( Point(0,0), other.corner );
    if( dist0 < dist1 )
        return true;
    if( dist0 > dist1 )
        return false;

    return cellsize < other.cellsize;
}

bool GridCellsInfoManager::GridSearchProperties::operator>(const GridCellsInfoManager::GridSearchProperties &other) const
{
    if( *this == other )//check first if equal (because operator< returns false if they are)
        return false;
    return !(*this < other);
}

GridCellsInfoManager::EdgeProperties::EdgeProperties(){

}

}

}

