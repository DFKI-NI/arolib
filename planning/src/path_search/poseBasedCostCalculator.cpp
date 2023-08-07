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
 

#include "arolib/planning/path_search/poseBasedCostCalculator.hpp"

namespace arolib{


const std::string IPoseBasedCostCalculator::GridMap_Field = "GM_field";
const std::string IPoseBasedCostCalculator::GridMap_RestrictedAreas = "GM_restricted";
const std::string IPoseBasedCostCalculator::GridMap_Visits = "GM_visits";

const int IPoseBasedCostCalculator::CellValue_obstacle = -1;
const int IPoseBasedCostCalculator::CellValue_outOfBoundary = -5;

IPoseBasedCostCalculator::RestrictedAreaOption IPoseBasedCostCalculator::intToRestrictedAreaOption(int value)
{

    if(value == RestrictedAreaOption::RESTRICT_WORKED_AREAS)
        return RestrictedAreaOption::RESTRICT_WORKED_AREAS;
    else if(value == RestrictedAreaOption::RESTRICT_UNWORKED_AREAS)
        return RestrictedAreaOption::RESTRICT_UNWORKED_AREAS;

    throw std::invalid_argument( "The given value does not correspond to any RestrictedAreaOption" );
}

bool IPoseBasedCostCalculator::GeneralParameters::parseFromStringMap(const std::map<std::string, std::string> &strMap, bool strict)
{
    GeneralParameters tmp = *this;

    int iRestrictedAreaOption;
    std::map<std::string, double*> dMap = { {"clearanceTime" , &tmp.clearanceTime} ,
                                            {"crossCostCoef" , &tmp.crossCostCoef} ,
                                            {"crossCostCoef_HL" , &tmp.crossCostCoef_HL} ,
                                            {"outOfFieldCostCoef" , &tmp.outOfBoundaryCostCoef} };
    std::map<std::string, bool*> bMap = { {"includeWaitInCost" , &tmp.includeWaitInCost},
                                            {"withCollisionAvoidance" , &tmp.withCollisionAvoidance} };
    std::map<std::string, int*> enumMap = { {"restrictedAreaOption" , &iRestrictedAreaOption} };

    tmp.restrictedAreaOption = intToRestrictedAreaOption(iRestrictedAreaOption);

    if( !setValuesFromStringMap( strMap, dMap, strict) ||
            !setValuesFromStringMap( strMap, bMap, strict) ||
            !setValuesFromStringMap( strMap, enumMap, strict) )
        return false;

    *this = tmp;
    return true;
}

std::map<std::string, std::string> IPoseBasedCostCalculator::GeneralParameters::parseToStringMap() const
{
    std::map<std::string, std::string> ret;
    ret["clearanceTime"] = double2string( clearanceTime );
    ret["crossCostCoef"] = double2string( crossCostCoef );
    ret["crossCostCoef_HL"] = double2string( crossCostCoef_HL);
    ret["outOfBoundaryCostCoef"] = double2string( outOfBoundaryCostCoef );
    ret["includeWaitInCost"] = std::to_string( includeWaitInCost );
    ret["withCollisionAvoidance"] = std::to_string( withCollisionAvoidance );
    ret["restrictedAreaOption"] = std::to_string( restrictedAreaOption );
    return ret;
}


IPoseBasedCostCalculator::CostExtended_t::PathPoint::PathPoint(const Point &_p, double _time_stamp)
    : p(_p), time_stamp(_time_stamp){

}

IPoseBasedCostCalculator::IPoseBasedCostCalculator(std::shared_ptr<Logger> parentLogger)
    : IPoseBasedCostCalculator(__FUNCTION__, parentLogger)
{
    m_gridmap_field.logger().setParent(loggerPtr());
    m_gridmap_restrictedAreas.logger().setParent(loggerPtr());
    m_gridmap_visits.logger().setParent(loggerPtr());
}

void IPoseBasedCostCalculator::setCalcSpeedFunct(const CalcSpeedFunct &f){
    m_calcSpeedFunct = f;
}

IPoseBasedCostCalculator::CostExtended_t::Validity IPoseBasedCostCalculator::isVertexValid(const IPoseBasedCostCalculator::Vertex_t &vt,
                                                                                           const Machine * const m,
                                                                                           double time_start,
                                                                                           const std::set<MachineId_t> &restrictedMachines) const
{
    double rad = m ? 0.5 * std::max(m->working_width, m->width) : -1;
    if(m_gridmap_field.isAllocated()){//check for obstacles
        bool errorTmp;
        auto val = m_gridmap_field.getValue(vt.pose, &errorTmp);
        if(!errorTmp && val == CellValue_obstacle)
            return CostExtended_t::INVALID__OBSTACLE;
    }
    if(m_gridmap_restrictedAreas.isAllocated()){//check for restricted areas
        bool errorTmp;
        if(m_gridmap_restrictedAreas.hasValue(vt.pose)){
            auto val = m_gridmap_restrictedAreas.getValue(vt.pose, &errorTmp);
            if(!errorTmp && restrictedMachines.find(val.machineId) != restrictedMachines.end()){
                if(m_general.restrictedAreaOption == RESTRICT_UNWORKED_AREAS && val.timestamp > time_start)
                    return CostExtended_t::INVALID__RESTRICTED_AREA;
                else if(m_general.restrictedAreaOption == RESTRICT_WORKED_AREAS && val.timestamp < time_start)
                    return CostExtended_t::INVALID__RESTRICTED_AREA;
            }
        }
    }
    return CostExtended_t::VALID;
}

bool IPoseBasedCostCalculator::getBestOutfieldConnection(const PoseBasedGraph &graph,
                                                         const std::unordered_set<VertexId_t> &start_vts,
                                                         const std::unordered_set<VertexId_t> &goal_vts,
                                                         const MachineId_t &machine_id,
                                                         VertexId_t &best_start_vt,
                                                         VertexId_t &best_goal_vt,
                                                         Cost_t &best_cost) const
{

    bool found = false;
    best_cost = std::numeric_limits<IPoseBasedCostCalculator::Cost_t>::max();
    auto OFconnections = graph.getOFConnections();

    for(auto start_vt : start_vts){
        auto it_con1 = OFconnections.find(start_vt);
        if(it_con1 == OFconnections.end())
            continue;

        for(auto goal_vt : goal_vts){
            auto it_con2 = it_con1->second.find(goal_vt);
            if(it_con2 == it_con1->second.end())
                continue;

            const PoseBasedGraph::MachineOutfieldCostsMap& ofCostMap = it_con2->second;
            auto it_m = ofCostMap.find(machine_id);
            if(it_m == ofCostMap.end())
                it_m = ofCostMap.find(OutFieldInfo::AllMachines);
            if(it_m == ofCostMap.end())//no OF cost for given machine
                continue;

            Cost_t costOF;
            const OutFieldInfo::MapMachineStateTravelCosts_t& tcm = it_m->second.travelCostsMap;
            OutFieldInfo::TravelCosts tc1, tc2;
            bool ok1 = OutFieldInfo::getTravelCost(tcm, OutFieldInfo::MACHINE_EMPTY, tc1);
            bool ok2 = OutFieldInfo::getTravelCost(tcm, OutFieldInfo::MACHINE_LOADED, tc2);
            if(!ok1 && !ok2)
                continue;
            if(ok1 && ok2)//select the lowers travel costs
                costOF = tc1.time <= tc2.time ? tc1.time : tc2.time;
            else if(ok1)
                costOF = tc1.time;
            else
                costOF = tc2.time;

            if(best_cost > costOF){
                best_cost = costOF;
                best_start_vt = start_vt;
                best_goal_vt = goal_vt;
            }
            found = true;
        }
    }

    return found;
}

double IPoseBasedCostCalculator::CalcSpeedDefault(const Machine &m, const std::vector<Point> &, double bunkerMass){
    return m.calcSpeed(bunkerMass);
}

bool IPoseBasedCostCalculator::setGridmap_field(const IPoseBasedCostCalculator::GridmapField_t &gridmap)
{
    if(!gridmap.isAllocated()){
        logger().printError(__FUNCTION__, "Invalid gridmap");
        return false;
    }
    m_gridmap_field = gridmap;
}

bool IPoseBasedCostCalculator::setGridmap_restrictedAreas(const IPoseBasedCostCalculator::GridmapRestrictedAreas_t &gridmap)
{
    if(!gridmap.isAllocated()){
        logger().printError(__FUNCTION__, "Invalid gridmap");
        return false;
    }
    m_gridmap_restrictedAreas = gridmap;
}

bool IPoseBasedCostCalculator::setGridmap_visits(const IPoseBasedCostCalculator::GridmapVisits_t &gridmap)
{
    if(!gridmap.isAllocated()){
        logger().printError(__FUNCTION__, "Invalid gridmap");
        return false;
    }
    m_gridmap_visits = gridmap;
}

bool IPoseBasedCostCalculator::removeGridmap(const std::string &name)
{
    if(name == GridMap_Field)
        m_gridmap_field.destroy();
    else if(name == GridMap_RestrictedAreas)
        m_gridmap_restrictedAreas.destroy();
    else if(name == GridMap_Visits)
        m_gridmap_visits.destroy();
    else{
        logger().printError(__FUNCTION__, "Invalid gridmap name");
        return false;
    }
    return true;
}

const IPoseBasedCostCalculator::GridmapField_t &IPoseBasedCostCalculator::getGridmap_field() const
{
    return m_gridmap_field;
}

const IPoseBasedCostCalculator::GridmapRestrictedAreas_t &IPoseBasedCostCalculator::getGridmap_restrictedAreas() const
{
    return m_gridmap_restrictedAreas;
}

const IPoseBasedCostCalculator::GridmapVisits_t &IPoseBasedCostCalculator::getGridmap_visits() const
{
    return m_gridmap_visits;
}

size_t IPoseBasedCostCalculator::setObstacles(const std::vector<Obstacle> &obs)
{
    m_obstacles.clear();
    for(auto& ob : obs){
        Obstacle o = ob;
        arolib::geometry::correct_polygon(o.boundary);
        if( arolib::geometry::isPolygonValid(o.boundary) == arolib::geometry::PolygonValidity::VALID_CLOSED_CW )
            m_obstacles.push_back(o);
    }
    return m_obstacles.size();
}

bool IPoseBasedCostCalculator::addObstacle(Obstacle obs)
{
    arolib::geometry::correct_polygon(obs.boundary);
    if( arolib::geometry::isPolygonValid(obs.boundary) != arolib::geometry::PolygonValidity::VALID_CLOSED_CW )
        return false;
    m_obstacles.push_back(obs);
    return true;
}

const std::vector<Obstacle> &IPoseBasedCostCalculator::getObstacles() const
{
    return m_obstacles;
}

void IPoseBasedCostCalculator::registerDelay(MachineId_t machineId, double time_from, double delay)
{
    VisitPeriod::DelayInfo d;
    d.time_from = time_from;
    d.delay = delay;
    m_delays[machineId].insert(d);
}

bool IPoseBasedCostCalculator::initGridmap_field(const Subfield &sf)
{
    if(!m_gridmap_field.isAllocated()){
        logger().printError(__FUNCTION__, "Field gridmap is not initialized/allocated");
        return false;
    }

    for(size_t x = 0 ; x < m_gridmap_field.getSizeX() ; x++){
        for(size_t y = 0 ; y < m_gridmap_field.getSizeY() ; y++){
            m_gridmap_field.setValue(x, y, CellValue_outOfBoundary);
        }
    }

    m_gridmap_field.setPolygon( sf.boundary_outer, nullptr );

    for(auto &track : sf.headlands.complete.tracks){
        for(size_t i = 0 ; i+1 < track.points.size() ; ++i)
            m_gridmap_field.setLine( track.points.at(i), track.points.at(i+1), 0, track.id );
    }
    for(auto &hl : sf.headlands.partial){
        for(auto track : hl.tracks){
            for(size_t i = 0 ; i+1 < track.points.size() ; ++i)
                m_gridmap_field.setLine( track.points.at(i), track.points.at(i+1), 0, track.id );
        }
    }
    for(auto &track : sf.tracks){
        for(size_t i = 0 ; i+1 < track.points.size() ; ++i)
            m_gridmap_field.setLine( track.points.at(i), track.points.at(i+1), 0, track.id );
    }
    for(auto &obs : sf.obstacles){
        m_gridmap_field.setPolygon(obs.boundary, CellValue_obstacle);
    }
    return true;
}

void IPoseBasedCostCalculator::addVisitPeriods(MachineId_t machine_id, const std::vector<RoutePoint> &route_points, double width, int idx_from)
{
    if(!m_gridmap_visits.isAllocated() || width < -1e-9
            || route_points.size() < 2 || idx_from >= route_points.size())
        return;
    width = std::max(0.0, width);
    idx_from = std::max(0, idx_from);
    for(size_t i = idx_from ; i+1 < route_points.size() ; ++i){
        const auto& rp1 = route_points.at(i);
        const auto& rp2 = route_points.at(i+1);

        //disregard out-of-field transit
        if(rp1.isFieldAccess() && ( rp2.isOfType({RoutePoint::RESOURCE_POINT, RoutePoint::INITIAL_POSITION}) || rp2.isFieldAccess() ))
            continue;
        if(rp2.isFieldAccess() && ( rp1.isOfType({RoutePoint::RESOURCE_POINT, RoutePoint::INITIAL_POSITION}) || rp1.isFieldAccess() ))
            continue;

        VisitPeriod vp;
        vp.timestamp = rp1.time_stamp;
        vp.time_in = rp1.time_stamp;
        vp.time_out = rp2.time_stamp;
        double widthTmp = std::max(width, 1e-6);
        vp.next_area.push_back( arolib::geometry::createRectangleFromLine(rp1, rp2, widthTmp) );
        vp.machineId = machine_id;

        auto indexMap = m_gridmap_visits.getCellsUnderLine( rp1, rp2, width );
        bool errorTmp;
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
                    std::vector<VisitPeriod> vps;
                    if(m_gridmap_visits.hasValue(x, y, &errorTmp))
                        vps = m_gridmap_visits.getValue(x, y, &errorTmp);

                    if(errorTmp)
                        continue;

                    vps.push_back(vp);
                    m_gridmap_visits.setValue(x, y, vps);
                }
            }
        }

    }
}

void IPoseBasedCostCalculator::addRestrictedAreaInfo(MachineId_t machine_id, const std::vector<RoutePoint> &route_points, double width, int idx_from, bool startFromLast)
{
    if(!m_gridmap_restrictedAreas.isAllocated() || width < -1e-9
            || route_points.size() < 2 || idx_from >= route_points.size())
        return;
    width = std::max(0.0, width);
    idx_from = std::max(0, idx_from);
    int idx_to = route_points.size() - 1;
    int deltaIdx = startFromLast ? -1 : 1;
    if(startFromLast){
        std::swap(idx_from, idx_to);
        deltaIdx = -1;
    }

    std::unordered_map<int, Point> trackStarts;
    for(auto& rp : route_points){
        if(startFromLast && rp.type == RoutePoint::TRACK_START)
            trackStarts[rp.track_id] = rp.point();
        else if(!startFromLast && rp.type == RoutePoint::TRACK_END)
            trackStarts[rp.track_id] = rp.point();
    }

    for(int i = startFromLast ? route_points.size() - 1 : idx_from ;
        startFromLast ? i > 0 : i+1 < route_points.size() ;
        i+=deltaIdx){

        if(i == 488)
            std::cout << "";


        auto rp1 = route_points.at( startFromLast ? i - 1 : i );
        auto rp2 = route_points.at( startFromLast ? i : i + 1 );

        //take into account only working segments
        if(!rp1.isOfTypeWorking() || !rp2.isOfTypeWorking())
            continue;

        //do not add inter-track segments
        if(rp1.isOfType({RoutePoint::TRACK_START, RoutePoint::TRACK_END}) && rp2.isOfType({RoutePoint::TRACK_START, RoutePoint::TRACK_END}))
            continue;

        //check if the track start/end point corresponds to a track end/start from the same track. (specifically for closed tracks)
        if( (startFromLast && rp2.type == RoutePoint::TRACK_END) || (!startFromLast && rp1.type == RoutePoint::TRACK_START) ){
            auto& rpRef = startFromLast ? rp2 : rp1;
            const auto& rpOther = startFromLast ? rp1 : rp2;
            auto it = trackStarts.find(rpRef.track_id);
            if(it != trackStarts.end()){
                if(arolib::geometry::calc_dist(it->second, rpRef) < 1e-9){//it is a closed track -> do not add the complete segment
                    if( arolib::geometry::calc_dist(rp1, rp2) < width )
                        continue;
                    rpRef.point() = arolib::geometry::getPointInLineAtDist(rpRef, rpOther, width);//remove the last 'width' meters from the segment
                }
            }
        }

        RestrictedAreaCell val;
        val.timestamp = rp2.time_stamp;
        val.machineId = machine_id;

        auto indexMap = m_gridmap_restrictedAreas.getCellsUnderLine( rp1, rp2, width );
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
                    m_gridmap_restrictedAreas.setValue(x, y, val);
                }
            }
        }

        val.timestamp = rp1.time_stamp;
        m_gridmap_restrictedAreas.setValue(rp1, val);

    }
}

void IPoseBasedCostCalculator::setGeneralParameters(const GeneralParameters &params)
{
    m_general = params;
}

bool IPoseBasedCostCalculator::parseFromStringMap(const std::map<std::string, std::string> &strMap, bool strict)
{
    if( !m_general.parseFromStringMap(strMap, strict) )
        return false;

    if( !parseOtherParametersFromStringMap(strMap, strict) )
        return false;

    return true;

}

std::map<std::string, std::string> IPoseBasedCostCalculator::parseToStringMap() const
{
    std::map<std::string, std::string> ret = m_general.parseToStringMap();
    parseAndAppendOtherParametersToStringMap(ret);
    return ret;
}

IPoseBasedCostCalculator::IPoseBasedCostCalculator(const std::string childName, std::shared_ptr<Logger>parentLogger)
    : LoggingComponent(parentLogger, childName)
{
    m_gridmap_field.logger().setParent(loggerPtr());
    m_gridmap_restrictedAreas.logger().setParent(loggerPtr());
    m_gridmap_visits.logger().setParent(loggerPtr());
}

std::vector<gridmap::GridmapLayout::GridCellOverlap> *IPoseBasedCostCalculator::getOverlapCells(std::map<ComputedGridCellsKey, std::vector<gridmap::GridmapLayout::GridCellOverlap> > &computedCells,
                                                                                                const gridmap::GridmapLayout &lo,
                                                                                                bool precise,
                                                                                                const std::vector<Point> &path,
                                                                                                double width)
{
    auto addToCellsMap = [](std::unordered_map<int, std::unordered_map<int, float>>& cellsMap,
                            const std::vector< gridmap::GridmapLayout::GridCellOverlap >& cells){
        for(auto cell : cells){
            auto it_x = cellsMap.find(cell.x);
            if(it_x == cellsMap.end()){
                cellsMap[cell.x][cell.y] = cell.overlap;
                continue;
            }
            auto it_y = it_x->second.find(cell.y);
            if(it_y == it_x->second.end()){
                it_x->second[cell.y] = cell.overlap;
                continue;
            }

            //leave the highest overlap
            it_y->second = std::max(it_y->second, cell.overlap);
        }

    };

    if(!lo.isValid())
        return nullptr;

    ComputedGridCellsKey key;
    key.lo = lo;
    key.precise = precise;
    auto it = computedCells.find(key);
    if(it != computedCells.end())
        return &(it->second);

    std::vector< gridmap::GridmapLayout::GridCellOverlap > cells;
    if(path.empty())
        return nullptr;
    if(path.size() == 1){
        gridmap::GridmapLayout::GridCellOverlap ol;
        unsigned int x, y;
        if(!lo.point2index(path.front(), x, y))
            return nullptr;
        ol.x = x;
        ol.y = y;
        ol.overlap = 1;
        cells.push_back(ol);
    }
    else{
        Polygon poly;
        if(width > 0){
            bool usePoly = !arolib::geometry::intersects(path);
            if(usePoly){
                if( arolib::geometry::offsetLinestring_boost(path,
                                           poly,
                                           0.5*width,
                                           0.5*width,
                                           true) ){
                    if(precise)
                        cells = lo.getCellsOverlapUnderPolygon(poly);
                    else
                        cells = gridmap::GridmapLayout::toGridCellOverlap( lo.getCellsUnderPolygon(poly), 1.0 );
                }
                else
                    usePoly = false;
            }
            if(!usePoly){
                if(precise){
                    std::unordered_map<int, std::unordered_map<int, float>> cellsMap;//used to avoid repeated cells
                    //add cells for each segment
                    for(size_t i = 0 ; i+1 < path.size() ; ++i){
                        addToCellsMap(cellsMap, lo.getCellsOverlapUnderLine( path.at(i), path.at(i+1), width ));
                    }
                    //add cells for each corner
                    for(size_t i = 1 ; i+1 < path.size() ; ++i){
                        auto circle = arolib::geometry::create_circle(path.at(i), 0.5*width, 13);
                        addToCellsMap(cellsMap, lo.getCellsOverlapUnderPolygon( circle ));
                    }
                    //retrieve (non repeated) cells from map
                    for(auto& it_x : cellsMap){
                        for(auto& it_y : it_x.second){
                            cells.emplace_back( gridmap::GridmapLayout::GridCellOverlap(it_x.first, it_y.first, 1.0) );
                        }
                    }
                }
                else{
                    std::unordered_map<int, std::unordered_set<int>> cellsMap;//used to avoid repeated cells
                    //add cells for each segment
                    for(size_t i = 0 ; i+1 < path.size() ; ++i){
                        auto cellsTmp = gridmap::GridmapLayout::toGridCellOverlap( lo.getCellsUnderLine( path.at(i), path.at(i+1), width ), 1.0 );
                        for(auto cell : cellsTmp)
                            cellsMap[cell.x].insert(cell.y);
                    }
                    //retrieve (non repeated) cells from map
                    for(auto& it_x : cellsMap){
                        for(auto& y : it_x.second){
                            cells.emplace_back( gridmap::GridmapLayout::GridCellOverlap(it_x.first, y, 1.0) );
                        }
                    }
                }
            }
        }
        else{
            std::unordered_map<int, std::unordered_set<int>> cellsMap;//used to avoid repeated cells
            //add cells for each segment
            for(size_t i = 0 ; i+1 < path.size() ; ++i){
                auto cellsTmp = gridmap::GridmapLayout::toGridCellOverlap( lo.getCellsUnderLine( path.at(i), path.at(i+1), false ), 1.0 );
                for(auto cell : cellsTmp)
                    cellsMap[cell.x].insert(cell.y);
            }
            //retrieve (non repeated) cells from map
            for(auto& it_x : cellsMap){
                for(auto& y : it_x.second){
                    cells.emplace_back( gridmap::GridmapLayout::GridCellOverlap(it_x.first, y, 1.0) );
                }
            }
        }
    }

    auto it_pair = computedCells.insert( std::make_pair(key, cells) );
    if(it_pair.second)
        return &(it_pair.first->second);
    return nullptr;
}

bool IPoseBasedCostCalculator::checkObstaclesInPath(std::map<IPoseBasedCostCalculator::ComputedGridCellsKey, std::vector<gridmap::GridmapLayout::GridCellOverlap> > &computedCells,
                                                    bool precise,
                                                    const std::vector<Point> &path,
                                                    double width) const
{
    if(m_obstacles.empty())
        return true;

    width = std::max(0.0, width);
    if(width > 1e-9){
        if(!arolib::geometry::intersects(path)){
            Polygon poly;
            if( arolib::geometry::offsetLinestring_boost(path,
                                       poly,
                                       0.5*width,
                                       0.5*width,
                                       true) ){
                for(auto& obs : m_obstacles){
                    if( arolib::geometry::intersects(poly, obs.boundary) )
                        return false;
                }

                return true;
            }
        }

        for(size_t i = 0 ; i+1 < path.size() ; ++i){
            auto poly = arolib::geometry::createRectangleFromLine( path.at(i), path.at(i+1), width );
            for(auto& obs : m_obstacles){
                if( arolib::geometry::intersects(poly, obs.boundary) )
                    return false;
            }
        }
        return true;
    }

    for(auto& obs : m_obstacles){
        if( arolib::geometry::intersects(path, obs.boundary, false, false) )
            return false;
    }
    return true;
}

size_t IPoseBasedCostCalculator::getInnerFieldTrackCrossings(std::vector<Point> path) const
{
    size_t count = 0;
    if(!m_gridmap_field.isAllocated() || path.size() < 2)
        return count;

    int prevIFTrack = std::numeric_limits<int>::lowest();

    path = arolib::geometry::sample_geometry(path, 0.25*m_gridmap_field.getCellsize());
    for(auto& p : path){
        bool errorTmp;
        if(!m_gridmap_field.hasValue(p, &errorTmp) || errorTmp)
            continue;
        auto val = m_gridmap_field.getValue(p, &errorTmp);
        if(errorTmp || val == prevIFTrack || val < 0)
            continue;

        if(val >= Track::DeltaHLTrackId){//drove over headland -> reset
            prevIFTrack = std::numeric_limits<int>::lowest();
            continue;
        }
        ++count;
        prevIFTrack = val;
    }

    return count;
}

void IPoseBasedCostCalculator::computeRouteSegment(const std::vector<Point> &path,
                                                   const Machine* m,
                                                   double width,
                                                   double time_start,
                                                   double speed,
                                                   const std::set<MachineId_t>& restrictedMachines,
                                                   bool precise,
                                                   IPoseBasedCostCalculator::CostExtended_t &costInfo) const
{
    costInfo.valid = CostExtended_t::VALID;
    width = std::max(0.0, width);
    double clearanceTime = std::max(0.0, m_general.clearanceTime);
    if(path.empty()){
        costInfo.waitTime = costInfo.time = 0;
        costInfo.path.clear();
        return;
    }

    if(speed == 0){
        costInfo.valid = CostExtended_t::INVALID__SPEED;
        return;
    }

    std::set<MachineId_t> excludeMachinesVisits;
    if(m)
        excludeMachinesVisits.insert(m->id);

    costInfo.waitTime = 0;
    costInfo.path.reserve( path.size() );
    costInfo.path.emplace_back( CostExtended_t::PathPoint(path.front(), time_start) );

    if(!m_gridmap_restrictedAreas.isAllocated() && !m_gridmap_visits.isAllocated()){
        for(size_t i = 1 ; i < path.size() ; ++i){
            costInfo.path.emplace_back( CostExtended_t::PathPoint(path[i],
                                                                  costInfo.path.back().time_stamp + arolib::geometry::calc_dist(path[i-1], path[i])/speed) );
        }
        costInfo.time = costInfo.path.back().time_stamp - time_start;
        return;
    }

    gridmap::GridCellsInfoManager cim;
    cim.logger().setParent(loggerPtr());

    bool useGridmap_restrictedAreas = m_gridmap_restrictedAreas.isAllocated();
    bool useGridmap_visits = m_gridmap_visits.isAllocated() && m_general.withCollisionAvoidance;

    if( useGridmap_restrictedAreas )
        cim.registerGrid(GridMap_RestrictedAreas, m_gridmap_restrictedAreas);
    if( useGridmap_visits )
        cim.registerGrid(GridMap_Visits, m_gridmap_visits);

    for(size_t i = 1 ; i < path.size() ; ++i){
        double timestamp_restrictedArea = std::numeric_limits<double>::lowest();
        double waitTime_visits = 0;
        double timestamp_speed = costInfo.path.back().time_stamp + arolib::geometry::calc_dist(path[i-1], path[i])/speed;
        std::vector<gridmap::GridmapLayout::GridCellOverlap> cellsInfo;
        gridmap::GridCellsInfoManager::Edge edge(path.at(i-1), path.at(i), width, precise );

        if( useGridmap_restrictedAreas && cim.computeAndUpdateCellsInfo(GridMap_RestrictedAreas, edge, m_gridmap_restrictedAreas, true, &cellsInfo) ){
            for(auto& cell : cellsInfo){
                bool errorTmp;
                if(!m_gridmap_restrictedAreas.hasValue(cell.x, cell.y, &errorTmp) || errorTmp)
                    continue;
                auto info = m_gridmap_restrictedAreas.getValue(cell.x, cell.y, &errorTmp);
                if(errorTmp)
                    continue;

                //check restricted areas
                if(restrictedMachines.find(info.machineId) != restrictedMachines.end()){
                    if(m_general.restrictedAreaOption == RESTRICT_UNWORKED_AREAS && info.timestamp > costInfo.path.back().time_stamp){
                        costInfo.valid = CostExtended_t::INVALID__RESTRICTED_AREA;
                        return;
                    }
                    else if(m_general.restrictedAreaOption == RESTRICT_WORKED_AREAS && info.timestamp < timestamp_speed){
                        costInfo.valid = CostExtended_t::INVALID__RESTRICTED_AREA;
                        return;
                    }
                }

                //check for waiting times in restricted areas
                if(m_general.restrictedAreaOption == RESTRICT_UNWORKED_AREAS && info.timestamp + clearanceTime > costInfo.path.back().time_stamp){
                    timestamp_restrictedArea = std::max( timestamp_restrictedArea, info.timestamp + clearanceTime - costInfo.path.back().time_stamp );
                }
                else if(m_general.restrictedAreaOption == RESTRICT_WORKED_AREAS && info.timestamp < timestamp_speed + clearanceTime){
                    timestamp_restrictedArea = std::max( timestamp_restrictedArea, timestamp_speed + clearanceTime - info.timestamp );
                }
            }
        }

        if( useGridmap_visits && cim.computeAndUpdateCellsInfo(GridMap_Visits, edge, m_gridmap_visits, true, &cellsInfo) ){
            for(auto& cell : cellsInfo){
                bool errorTmp;
                if(!m_gridmap_visits.hasValue(cell.x, cell.y, &errorTmp) || errorTmp)
                    continue;
                auto vps = m_gridmap_visits.getValue(cell.x, cell.y, &errorTmp);
                if(errorTmp)
                    continue;

                double waitTimeTmp;
                bool towardsCurrentLocation;
                std::vector<MachineId_t> visitingMachines;
                if( VisitPeriod::isBusy(vps,
                                        costInfo.path.back().time_stamp - clearanceTime,
                                        timestamp_speed - costInfo.path.back().time_stamp + 2 * clearanceTime,
                                        1,
                                        waitTimeTmp,
                                        path[i],
                                        towardsCurrentLocation,
                                        visitingMachines,
                                        excludeMachinesVisits,
                                        m_delays) ){

                    if(towardsCurrentLocation){
                        costInfo.valid = CostExtended_t::INVALID__BUSY_AREA;
                        return;
                    }

                    waitTime_visits = std::max(waitTime_visits, waitTimeTmp);
                }

            }
        }

        double waitingTime = 0;
        if(timestamp_speed < timestamp_restrictedArea){
            waitingTime = timestamp_restrictedArea - timestamp_speed;
            waitTime_visits = waitTime_visits - waitingTime;
        }
        if(waitTime_visits > 0)
            waitingTime += waitTime_visits;

        if(waitingTime > 0.1){
            costInfo.path.emplace_back( costInfo.path.back() );
            costInfo.path.back().time_stamp += waitingTime;
        }

        costInfo.path.emplace_back( CostExtended_t::PathPoint(path[i], timestamp_speed + waitingTime ) );
        costInfo.waitTime += waitingTime;
    }

    costInfo.time = costInfo.path.back().time_stamp - time_start;
    return;


}

//------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------PBCC_timeOpt---------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------

PBCC_timeOpt::PBCC_timeOpt(std::shared_ptr<Logger> parentLogger)
    : IPoseBasedCostCalculator(__FUNCTION__, parentLogger)
{

}


IPoseBasedCostCalculator::CostExtended_t PBCC_timeOpt::calcCost(const std::vector<Point> &path, const Machine &m, double time_start, double bunkerMass, const std::set<MachineId_t> &restrictedMachines) const{

    double speed = std::fabs( m_calcSpeedFunct(m, path, bunkerMass) );
    double width = std::max(m.width, m.working_width);

    CostExtended_t ret;
    ret.waitTime = 0;
    ret.numCrossings = ret.numCrossings_HL = -1;

    if(speed == 0){
        ret.valid = CostExtended_t::INVALID__SPEED;
        ret.cost = std::numeric_limits<Cost_t>::max();
        ret.time = std::numeric_limits<double>::max();
        return ret;
    }

    std::map<ComputedGridCellsKey, std::vector<gridmap::GridmapLayout::GridCellOverlap> > computedCells;
    if(!checkObstaclesInPath(computedCells, false, path, width)){
        ret.valid = CostExtended_t::INVALID__OBSTACLE;
        ret.cost = std::numeric_limits<Cost_t>::max();
        ret.time = std::numeric_limits<double>::max();
        return ret;
    }

    auto sampled_path = arolib::geometry::sample_geometry(path, m.workingRadius(), 0.2 * m.workingRadius());
    computeRouteSegment(sampled_path,
                        &m,
                        width,
                        time_start,
                        speed,
                        restrictedMachines,
                        false,
                        ret);
    if(ret.valid != CostExtended_t::VALID)
        return ret;

    ret.numCrossings = ret.numCrossings_HL = -1;
    if(std::fabs(m_general.crossCostCoef) > 1e-5)//if ==0, do not compute
        ret.numCrossings = getInnerFieldTrackCrossings(path);

    const double multDist = 1e-4;
    double pathLength = arolib::geometry::getGeometryLength(path);
    double penalty = 0;
    if(ret.numCrossings > 0)
        penalty = (ret.time - ret.waitTime) * m_general.crossCostCoef;

    ret.cost = ret.time + penalty + multDist*pathLength;//a distance factor is added so that if two plans have similar time-costs, the shortest one will give lower costs

    return ret;
}

IPoseBasedCostCalculator::CostExtended_t PBCC_timeOpt::calcCost(const PoseBasedGraph::OutFieldCosts &OFCosts, const Machine &m, double time_start, double bunkerMass) const{
    CostExtended_t ret;

    OutFieldInfo::MachineBunkerState bunkerState = bunkerMass < 0.2 * m.bunker_mass ? OutFieldInfo::MACHINE_EMPTY : OutFieldInfo::MACHINE_LOADED;
    auto it_state = OFCosts.travelCostsMap.find(bunkerState);
    if( it_state == OFCosts.travelCostsMap.end() )
        it_state = OFCosts.travelCostsMap.find(OutFieldInfo::ALL_MACHINE_STATES);
    if( it_state != OFCosts.travelCostsMap.end() ){
        ret.cost = it_state->second.time;
        ret.time = it_state->second.time;
    }
    else{
        ret.valid = CostExtended_t::INVALID__MISSING_OUTFIELD_INFO;
        ret.cost = std::numeric_limits<Cost_t>::max();
        ret.time = std::numeric_limits<double>::max();
    }
    ret.waitTime = 0;
    ret.numCrossings = ret.numCrossings_HL = 0;
    return ret;
}

IPoseBasedCostCalculator::Cost_t PBCC_timeOpt::calcHeuristic(const Vertex_t &start_vt, const Vertex_t &goal_vt, const Machine &m) const{

    double dist = 0;
    if( is_nan(start_vt.pose.angle) || is_nan(goal_vt.pose.angle)
            || arolib::geometry::calc_dist( start_vt.pose, goal_vt.pose ) < 1e-6 ){
        dist = arolib::geometry::calc_dist( start_vt.pose, goal_vt.pose );
    }
    else{//use dubins
        arolib::geometry::DubinsParams dp;
        dp.type = arolib::geometry::DubinsParams::SHORTEST;
        dp.p1 = start_vt.pose.point();
        dp.p2 = goal_vt.pose.point();
        dp.rho1 = start_vt.pose.angle;
        dp.rho2 = goal_vt.pose.angle;
        dist = calcDubinsPathLength(dp, m.getTurningRadius());
    }

    double speed = std::fabs( m.calcSpeed(0.0) );

    if(speed == 0 || dist < -1e-9)
        return std::numeric_limits<Cost_t>::max();

    return dist / speed;

}

IPoseBasedCostCalculator::Cost_t PBCC_timeOpt::calcHeuristic(const PoseBasedGraph &graph, const Vertex_t &start_vt, const Vertex_t &goal_vt, const Machine &m) const{
    bool isStartOF = start_vt.isOfOutfieldType(false);
    bool isGoalOF = goal_vt.isOfOutfieldType(false);
    if(isStartOF == isGoalOF)
        return calcHeuristic(start_vt, goal_vt, m);

    bool found = false;
    Cost_t minCost = std::numeric_limits<Cost_t>::max();
    Cost_t costIF, costOF;
    auto OFconnections = graph.getOFConnections();
    Vertex_t vtTmp;
    if(isStartOF){
        auto it_con1 = OFconnections.find(start_vt.id);
        if(it_con1 == OFconnections.end())
            return calcHeuristic(start_vt, goal_vt, m);
        auto entry_vts = graph.getEntryVertices();
        for(auto &vt_it : entry_vts){//check all entry vts <fap_id, vts_set>
            for(auto &it_con2 : it_con1->second){//check all vertices connected OF to the start_vt
                if( vt_it.second.find(it_con2.first) == vt_it.second.end() )//start_vt is NOT connected to entry_vt
                    continue;
                if(!graph.getVertex(it_con2.first, vtTmp))
                    continue;
                const PoseBasedGraph::MachineOutfieldCostsMap& ofCostMap = it_con2.second;
                auto it_m = ofCostMap.find(m.id);
                if(it_m == ofCostMap.end())
                    it_m = ofCostMap.find(OutFieldInfo::AllMachines);
                if(it_m == ofCostMap.end())//no OF cost for given machine
                    continue;
                const OutFieldInfo::MapMachineStateTravelCosts_t& tcm = it_m->second.travelCostsMap;
                OutFieldInfo::TravelCosts tc1, tc2;
                bool ok1 = OutFieldInfo::getTravelCost(tcm, OutFieldInfo::MACHINE_EMPTY, tc1);
                bool ok2 = OutFieldInfo::getTravelCost(tcm, OutFieldInfo::MACHINE_LOADED, tc2);
                if(!ok1 && !ok2)
                    continue;
                if(ok1 && ok2)//select the lowers travel costs
                    costOF = tc1.time <= tc2.time ? tc1.time : tc2.time;
                else if(ok1)
                    costOF = tc1.time;
                else
                    costOF = tc2.time;
                costIF = vtTmp.id == goal_vt.id ? 0 : calcHeuristic(vtTmp, goal_vt, m);
                minCost = std::min( minCost, costOF + costIF);
                found = true;
            }
        }
    }
    else{
        auto exit_vts = graph.getExitVertices();
        for(auto& it_con1 : OFconnections){//check all vertices connected OF to the goal_vt
            auto it_con2 = it_con1.second.find(goal_vt.id);
            if(it_con2 == it_con1.second.end())
                continue;

            for(auto &vt_it : exit_vts){//check all exit vts <fap_id, vts_set>
                if( vt_it.second.find(it_con1.first) == vt_it.second.end() )//goal_vt is NOT connected to exit_vt
                    continue;
                if(!graph.getVertex(it_con1.first, vtTmp))
                    continue;
                const PoseBasedGraph::MachineOutfieldCostsMap& ofCostMap = it_con2->second;
                auto it_m = ofCostMap.find(m.id);
                if(it_m == ofCostMap.end())
                    it_m = ofCostMap.find(OutFieldInfo::AllMachines);
                if(it_m == ofCostMap.end())//no OF cost for given machine
                    continue;
                const OutFieldInfo::MapMachineStateTravelCosts_t& tcm = it_m->second.travelCostsMap;
                OutFieldInfo::TravelCosts tc1, tc2;
                bool ok1 = OutFieldInfo::getTravelCost(tcm, OutFieldInfo::MACHINE_EMPTY, tc1);
                bool ok2 = OutFieldInfo::getTravelCost(tcm, OutFieldInfo::MACHINE_LOADED, tc2);
                if(!ok1 && !ok2)
                    continue;
                if(ok1 && ok2)//select the lowers travel costs
                    costOF = tc1.time <= tc2.time ? tc1.time : tc2.time;
                else if(ok1)
                    costOF = tc1.time;
                else
                    costOF = tc2.time;
                costIF = vtTmp.id == start_vt.id ? 0 : calcHeuristic(start_vt, vtTmp, m);
                minCost = std::min( minCost, costOF + costIF);
                found = true;
            }

        }

    }

    if(!found)
        return calcHeuristic(start_vt, goal_vt, m);

    return minCost;
}

bool PBCC_timeOpt::parseOtherParametersFromStringMap(const std::map<std::string, std::string> &strMap, bool strict)
{
    return true;
}

void PBCC_timeOpt::parseAndAppendOtherParametersToStringMap(std::map<std::string, std::string> &strMap) const
{

}

}
