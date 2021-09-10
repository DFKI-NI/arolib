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
 
#include "arolib/planning/planningworkspace.h"

namespace arolib {


PlanningWorkspace::Edge::Edge(const Point &_p0, const Point &_p1, double _width){
    if(_width <= 0)
        throw std::invalid_argument( "PlanningWorkspace::Edge::Edge: Invalid width" );

    width = _width;

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

bool PlanningWorkspace::Edge::operator==(const PlanningWorkspace::Edge &e) const
{
    const double eps = 1e-6;
    if(std::fabs(width - e.width) > eps)
        return false;
    return ( p0==e.p0 && p1==e.p1 ) ||
           ( p0==e.p1 && p1==e.p0 );//If p1 == e.p0 and p0 == e.p1, it is considered that the edge points are equal.
}

bool PlanningWorkspace::Edge::operator<(const PlanningWorkspace::Edge &e) const
{
    if( *this == e )
        return false;

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

bool PlanningWorkspace::Edge::operator>(const PlanningWorkspace::Edge &e) const
{
    if( *this == e )//check first if the edges are equal (because operator< returns false if they are)
        return false;
    return !(*this < e);
}



void PlanningWorkspace::EdgeProperties::updateGridCells(PlanningWorkspace::GridType type, const ArolibGrid_t &grid, const Edge &edge, bool addIfNotSet)
{
    //ckeck for invalid parameters
    if(type == PlanningWorkspace::GridType::GridType_TOTAL)//NOTE! GridType_TOTAL is not supported!
        throw std::invalid_argument( "PlanWorkspace::Edge::updateGridCells: Invalid grid type" );
    if(edge.width <= 0)
        throw std::invalid_argument( "PlanWorkspace::Edge::updateGridCells: Invalid width" );

    gridCellsInfo.at(type).clear();
    if(!gridCellsInfoIsSet.at(type) && !addIfNotSet)
        return;

    gridCellsInfoIsSet.at(type) = true;
    if(!grid.isAllocated())
        return;

    gridCellsInfo.at(type) = grid.getCellsOverlapUnderLine(edge.p0, edge.p1, edge.width);

}

void PlanningWorkspace::EdgeProperties::clearGridCells(PlanningWorkspace::GridType type)
{
    if(type != PlanningWorkspace::GridType::GridType_TOTAL){
        gridCellsInfo.at(type).clear();
        gridCellsInfoIsSet.at(type) = false;
        return;
    }

    //clear all and re-initialize (with empty data) the gridCellsInfo and gridCellsInfoIsSet vectors (for each grid type)
    gridCellsInfo.clear();
    gridCellsInfo.resize(PlanningWorkspace::GridType::GridType_TOTAL);
    gridCellsInfoIsSet.clear();
    gridCellsInfoIsSet.resize(PlanningWorkspace::GridType::GridType_TOTAL, false);
}

PlanningWorkspace::PlanningWorkspace(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_maps( PlanningWorkspace::GridType::GridType_TOTAL )//initialize the maps/grids container with the right amount of empty (unallocated) grids
{

}

void PlanningWorkspace::clearAll()
{
    m_field.clear();
    clearRouteMaps(RouteType_TOTAL);
    m_machines.clear();
    m_machineCurrentStates.clear();
    m_outFieldInfo.clearAll();
}

bool PlanningWorkspace::clearSubfield(int subfieldIdx, bool onlyInfield)
{
    if(subfieldIdx >= m_field.subfields.size()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfield index");
        return false;
    }

    if(subfieldIdx < 0){//clear all subfields
        for( auto &sf : m_field.subfields ){
            if(!onlyInfield){
                sf.boundary_inner.points.clear();
                sf.headlands.complete.middle_track.points.clear();
                sf.headlands.complete.tracks.clear();
            }
            sf.tracks.clear();
        }
        return true;
    }

    if(!onlyInfield){
        m_field.subfields.at(subfieldIdx).boundary_inner.points.clear();
        m_field.subfields.at(subfieldIdx).headlands.complete.middle_track.points.clear();
    }
    m_field.subfields.at(subfieldIdx).tracks.clear();

    return true;
}

void PlanningWorkspace::clearRoutes(RouteType routeType)
{
    if(routeType == RouteType::BASE_HEADLAND || routeType == RouteType::RouteType_TOTAL){
        for(auto &it_r : m_baseRoutes_headland)
            it_r.second.clear();
        m_edgesHeadland.clear();
    }
    if(routeType == RouteType::BASE_INFIELD || routeType == RouteType::RouteType_TOTAL){
        for(auto &it_r : m_baseRoutes_infield)
            it_r.second.clear();
        m_edgesInfield.clear();
    }
    if(routeType == RouteType::BASE_PROCESSED || routeType == RouteType::RouteType_TOTAL){
        for(auto &it_r : m_baseRoutesProcessed_headland)
            it_r.second.clear();
        for(auto &it_r : m_baseRoutesProcessed_infield)
            it_r.second.clear();
        for(auto &it_r : m_connectedBaseRoutes)
            it_r.second.clear();
    }
    if(routeType == RouteType::PLANNED || routeType == RouteType::RouteType_TOTAL)
        for(auto &it_r : m_plannedRoutes)
            it_r.second.clear();
}

void PlanningWorkspace::setField(const Field &_field)
{
    m_field = _field;

    if(m_field.subfields.empty()){//if the field has no subfiels, create one with the same outer boundary as the field
       m_field.subfields.push_back( Subfield() );
       m_field.subfields.back().boundary_outer = m_field.outer_boundary;
    }

    //initialize the routes' map-elements for all the subfields with empty containers (no routes)
    clearRouteMaps(PlanningWorkspace::RouteType_TOTAL);
    for(size_t i = 0 ; i < m_field.subfields.size() ; ++i){
        m_baseRoutes_headland[i] = std::vector<HeadlandRoute>(0);
        m_baseRoutes_infield[i] = std::vector<Route>(0);
        m_plannedRoutes[i] = std::vector<Route>(0);
    }
}

void PlanningWorkspace::setMachines(const std::vector<Machine> &_machines)
{
    m_machines = _machines;
}

void PlanningWorkspace::setOutFieldInfo(const OutFieldInfo &_outFieldInfo)
{
    m_outFieldInfo = _outFieldInfo;
}

void PlanningWorkspace::setMachineCurrentStates(const std::map<MachineId_t, MachineDynamicInfo> &_machineCurrentStates)
{
    m_machineCurrentStates = _machineCurrentStates;
}

void PlanningWorkspace::setGrid(PlanningWorkspace::GridType gridType, const ArolibGrid_t &grid, bool updateCellsIfDifferent)
{
    if(gridType == PlanningWorkspace::GridType::GridType_TOTAL)
        throw std::invalid_argument( "PlanWorkspace::setGrid: Invalid grid type" );

    if(!m_maps.at(gridType).equalGeometry(grid)){//check if they have the same geometry (if they do, the edges cells' information will be the same as before)
        if(updateCellsIfDifferent){//update the cells' info from existing edges (iif they were set already)
            updateGridCells(m_edgesHeadland, gridType, grid, false);
            updateGridCells(m_edgesInfield, gridType, grid, false);
        }
        else{//clear all cells' info related to this grid type
            clearGridCells(m_edgesHeadland, gridType);
            clearGridCells(m_edgesInfield, gridType);
        }
    }
    m_maps.at(gridType) = grid;
}

const ArolibGrid_t &PlanningWorkspace::getGrid(PlanningWorkspace::GridType gridType)
{
    if(gridType == PlanningWorkspace::GridType::GridType_TOTAL)
        throw std::invalid_argument( "PlanWorkspace::setGrid: Invalid grid type" );
    return m_maps.at(gridType);
}

const std::map<int, std::vector<HeadlandRoute> > &PlanningWorkspace::getBaseRoutes_headland(bool processed) const {
    if(processed)
        return m_baseRoutesProcessed_headland;
    else
        return m_baseRoutes_headland;
}

std::map<int, std::vector<HeadlandRoute> > &PlanningWorkspace::getBaseRoutes_headland(bool processed) {
    if(processed)
        return m_baseRoutesProcessed_headland;
    else
        return m_baseRoutes_headland;}

const std::map<int, std::vector<Route> > &PlanningWorkspace::getBaseRoutes_infield(bool processed) const {
    if(processed)
        return m_baseRoutesProcessed_infield;
    else
        return m_baseRoutes_infield;}

std::map<int, std::vector<Route> > &PlanningWorkspace::getBaseRoutes_infield(bool processed) {
    if(processed)
        return m_baseRoutesProcessed_infield;
    else
        return m_baseRoutes_infield;}

const std::vector<gridmap::GridmapLayout::GridCellOverlap> &PlanningWorkspace::getEdgeCellsInfo(PlanningWorkspace::GridType gridType,
                                                                     ProcessType processType,
                                                                     const Point p0,
                                                                     const Point p1,
                                                                     double width)
{
    if(gridType == PlanningWorkspace::GridType::GridType_TOTAL)
        throw std::invalid_argument( "PlanWorkspace::setGrid: Invalid grid type" );
    const ArolibGrid_t &map = m_maps.at(gridType);

    std::map<Edge, EdgeProperties> * edges;
    if(processType == PROCESS_INFIELD)
        edges = &m_edgesInfield;
    else
        edges = &m_edgesHeadland;

    PlanningWorkspace::Edge edge( p0,
                                  p1,
                                  width);

    auto it_edge = edges->find(edge);
    PlanningWorkspace::EdgeProperties &ep = (*edges)[edge];

    if(it_edge == edges->end() || !ep.gridCellsInfoIsSet.at(gridType))
        ep.updateGridCells( gridType, map, edge, true );
    else
        m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Edge already present : " + edge.p0.toString() + " <--> " + edge.p1.toString() + " ; w = " + std::to_string(edge.width) );

    return ep.gridCellsInfo.at(gridType);

}

void PlanningWorkspace::clearRouteMaps(PlanningWorkspace::RouteType routeType)
{
    if(routeType == RouteType::BASE_HEADLAND || routeType == RouteType::RouteType_TOTAL){
        m_baseRoutes_headland.clear();
        m_edgesHeadland.clear();
    }
    if(routeType == RouteType::BASE_INFIELD || routeType == RouteType::RouteType_TOTAL){
        m_baseRoutes_infield.clear();
        m_edgesInfield.clear();
    }
    if(routeType == RouteType::BASE_PROCESSED || routeType == RouteType::RouteType_TOTAL){
        m_baseRoutesProcessed_headland.clear();
        m_baseRoutesProcessed_infield.clear();
        m_connectedBaseRoutes.clear();
    }
    if(routeType == RouteType::PLANNED || routeType == RouteType::RouteType_TOTAL)
        m_plannedRoutes.clear();
}

void PlanningWorkspace::updateGridCells(std::map<Edge, EdgeProperties> &edges, GridType gridType, const ArolibGrid_t &grid, bool addIfNotSet)
{
    for(auto &edge : edges)
        edge.second.updateGridCells(gridType, grid, edge.first, addIfNotSet);
}

void PlanningWorkspace::clearGridCells(std::map<PlanningWorkspace::Edge, PlanningWorkspace::EdgeProperties> &edges, PlanningWorkspace::GridType gridType)
{
    for(auto &edge : edges)
        edge.second.clearGridCells(gridType);
}


}
