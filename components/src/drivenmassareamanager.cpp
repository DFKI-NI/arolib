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
 
#include "arolib/components/drivenmassareamanager.h"

namespace arolib {

const std::string DrivenMassAreaManager::DrivenMassAreaMapName = "DRIV_MASS_AREA";

DrivenMassAreaManager::DrivenMassAreaManager(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_gridsManager(logLevel)
{
    m_gridsManager.logger().setParent(loggerPtr());
}

AroResp DrivenMassAreaManager::init(const Field &field, double cellsize)
{
    clear();

    if(cellsize < 1e-6){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid cell size.");
        return AroResp(1, "Invalid cell size." );;
    }

    double minX, maxX, minY, maxY;
    if(!arolib::geometry::getPolygonLimits(field.outer_boundary, minX, maxX, minY, maxY)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error obtaining the outer boundary limits.");
        return AroResp(1, "Error obtaining the outer boundary limits." );;
    }

    if( (maxX-minX) < 10*cellsize || (maxY-minY) < 10*cellsize ){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Resolution (cell size) too low for the outer boundary.");
        return AroResp(1, "Resolution (cell size) too low for the outer boundary." );;
    }

    auto ram = initDrivenMassAreaMap();

    if(!ram->createGrid(minX-cellsize, maxX+cellsize, minY-cellsize, maxY+cellsize, cellsize)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating grid.");
        return AroResp(1, "Error creating grid" );;
    }

    if(!ram->setPolygon(field.outer_boundary, 0.0)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error initializing grid with outer boundary.");
        return AroResp(1, "Error initializing grid with outer boundary." );;
    }

    m_drivenMassAreaMap = ram;
    m_gridsManager.addGrid(DrivenMassAreaMapName, m_drivenMassAreaMap, true);

    m_field = field;
    m_ready = true;
    return AroResp(0, "OK" );
}

AroResp DrivenMassAreaManager::init(const Field &field, const std::vector<Machine> &machines)
{
    clear();

    if(machines.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "No machines were given to calculate the cell size.");
        return AroResp(1, "No machines were given to calculate the cell size." );;
    }

    double cellsize = std::numeric_limits<double>::max();
    for(auto&m : machines){
        if(m.working_width > 1e-3)
            cellsize = std::min(cellsize, m.working_width);
    }

    if(cellsize > 1e3){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid working width in machines.");
        return AroResp(1, "Invalid working width in machines." );;
    }

    return init(field, cellsize*0.5);
}

AroResp DrivenMassAreaManager::init(const Field &field, const ArolibGrid_t &basemap, bool initMap)
{
    clear();

    if(!basemap.isAllocated()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid map (not allocated).");
        return AroResp(1, "Invalid map (not allocated)." );;
    }

    auto ram = initDrivenMassAreaMap();
    *ram = basemap;

    if(initMap){
        Polygon boundary_exp;
        if(!arolib::geometry::offsetPolygon(field.outer_boundary, boundary_exp, ram->getCellsize(), true)){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting the outer boundary.");
            return AroResp(1, "Error offsetting the outer boundary." );;
        }

        if(!ram->expandGridFromPolygon(boundary_exp.points, 0.0)){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error expanding the grid.");
            return AroResp(1, "Error expanding the grid." );;
        }
    }

    m_drivenMassAreaMap = ram;
    m_gridsManager.addGrid(DrivenMassAreaMapName, m_drivenMassAreaMap, true);

    m_field = field;
    m_ready = true;
    return AroResp(0, "OK" );

}

void DrivenMassAreaManager::setGridCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim)
{
    m_gridsManager.setCellsInfoManager(cim);
}

bool DrivenMassAreaManager::isReady() const
{
    return m_ready;
}

void DrivenMassAreaManager::clear()
{
    m_ready = false;
    m_field.clear();
    m_drivenMassAreaMap = nullptr;
    m_gridsManager.clearAll();
    m_prevData.clear();
    m_GPSFrontDisplacements.clear();
}

void DrivenMassAreaManager::forgetMachineLocation(const MachineId_t &machineId)
{
    m_prevData.erase(machineId);
}

void DrivenMassAreaManager::forgetAllMachineLocations()
{
    m_prevData.clear();
}

bool DrivenMassAreaManager::setDefaultGPSFrontDisplacement(float val)
{
    if(val > 1.00001)
        return false;
    if(val < -1e-6)
        m_defaultGPSFrontDisplacement = -1;
    else
        m_defaultGPSFrontDisplacement = std::max(0.0f, std::min(1.0f, val));
    return true;
}

bool DrivenMassAreaManager::setMachineGPSFrontDisplacement(MachineId_t machineId, float val)
{
    if(val > 1.00001)
        return false;
    if(val < -1e-6)
        m_GPSFrontDisplacements[machineId] = -1;
    else
        m_GPSFrontDisplacements[machineId] = std::max(0.0f, std::min(1.0f, val));
    return true;
}

void DrivenMassAreaManager::setUsePolygonIntersection(bool usePolygonIntersection)
{
    m_usePolygonIntersection = usePolygonIntersection;
}

AroResp DrivenMassAreaManager::addData(const Machine &machine, const Point &pt, double bunker_mass, bool be_precise)
{
    if(!m_ready){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Not ready/initialized.");
        return AroResp(1, "Not ready/initialized." );;
    }

    if(machine.width < 1e-5){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid machine width.");
        return AroResp(1, "Invalid machine working width." );;
    }

    if( !m_gridsManager.hasGrid(DrivenMassAreaMapName) )
        return AroResp::LoggingResp(1, "The gridmap has not been initialized", m_logger, LogLevel::ERROR, __FUNCTION__);

    if(m_usePolygonIntersection)
        return addData_polygonIntersection(machine, pt, bunker_mass, be_precise);
    return addData_cellsAnalysis(machine, pt, bunker_mass, be_precise);
}

gridmap::SharedGridsManager::ConstGridPtr DrivenMassAreaManager::getDrivenMassAreaMap() const
{
    return m_gridsManager.getGrid(DrivenMassAreaMapName);
}

gridmap::SharedGridsManager::GridPtr DrivenMassAreaManager::initDrivenMassAreaMap()
{
    auto ram = std::make_shared<ArolibGrid_t>();
    ram->logger().setParent(loggerPtr());
    return ram;
}

DrivenMassAreaManager::GridCellsInfoMap_t DrivenMassAreaManager::cellsInfoVecToMap(const std::vector<gridmap::GridmapLayout::GridCellOverlap> &vec)
{
    GridCellsInfoMap_t ret;
    for(auto& cellInfo : vec){
        float overlap;
        if(!cellsInfoMapGetValue(ret, cellInfo.x, cellInfo.y, overlap)
                || cellInfo.overlap > overlap){
            ret[cellInfo.x][cellInfo.y] = cellInfo.overlap;
        }
    }
    return ret;
}

bool DrivenMassAreaManager::cellsInfoMapGetValue(const GridCellsInfoMap_t &cellsInfoMap, int x, int y, float &val)
{
    auto it_x = cellsInfoMap.find(x);
    if(it_x == cellsInfoMap.end())
        return false;
    auto it_y = it_x->second.find(y);
    if(it_y == it_x->second.end())
        return false;
    val = it_y->second;
    return true;
}

void DrivenMassAreaManager::removeCellsInfoFromMap(GridCellsInfoMap_t &cellsInfoMap, int x, int y)
{
    auto it_x = cellsInfoMap.find(x);
    if(it_x == cellsInfoMap.end())
        return;
    it_x->second.erase(y);
    if(it_x->second.empty())
        cellsInfoMap.erase(x);
}

AroResp DrivenMassAreaManager::addData_polygonIntersection(const Machine &machine, const Point &pt, double bunker_mass, bool be_precise)
{
    float massTotal = std::max(0.0, machine.weight) + std::max(0.0, bunker_mass);

    auto it = m_prevData.find(machine.id);
    if(it == m_prevData.end() || it->second.points.empty()){//first point
        Data& prevData = m_prevData[machine.id];
        prevData.points.emplace_back(pt);
        prevData.massTotal = massTotal;
        return AroResp(0, "OK" );
    }

    bool sameEdge = false;

    Data& prevData = it->second;
    std::vector<Point>& points = prevData.points;

    if( geometry::calc_dist(pt, points.back()) < 1e-3 ){//same point as the last one
        if(points.size() == 1 || massTotal <= prevData.massTotal)//no previous edge or same edge
            return AroResp(0, "OK" );
        sameEdge = true;// same edge with higher mass
    }
    else if(points.size() == 2){//a previous edge exists
        points.front() = points.back();
        points.back() = pt;
    }
    else//first edge
        points.emplace_back(pt);

    prevData.massTotal = massTotal;

    auto gpsFrontDisplacement = m_defaultGPSFrontDisplacement;
    auto it_mfd = m_GPSFrontDisplacements.find(machine.id);
    if(it_mfd != m_GPSFrontDisplacements.end())
        gpsFrontDisplacement = it_mfd->second;

    Point p1 = points.front(), p2 = points.back();
    if(machine.length > 1e-9 && gpsFrontDisplacement > -1e-9){//extend the edge based on machine length and relative location of GPS
        float deltaFront = machine.length * gpsFrontDisplacement;
        float deltaBack = machine.length - deltaFront;
        p2 = geometry::getPointInLineAtDist(p2, p1, -deltaFront);
        p1 = geometry::getPointInLineAtDist(p1, p2, -deltaBack);
    }

    std::vector<Polygon> intersectionPolys;
    if(sameEdge){
        intersectionPolys.push_back(prevData.machineProj);
    }
    else{
        auto machineProj = geometry::createRectangleFromLine(p1, p2, machine.width);

        if( !arolib::geometry::in_polygon(machineProj.points, m_field.outer_boundary, false)
                && !arolib::geometry::intersects(machineProj.points, m_field.outer_boundary) ){// edge projection outside of the field
            prevData.cellsInfoMass.clear();
            prevData.cellsInfoOverlap.clear();
            return AroResp(0, "OK" );
        }
        intersectionPolys = geometry::get_intersection(machineProj, prevData.machineProj);
        prevData.machineProj = machineProj;
    }

    std::vector<gridmap::GridmapLayout::GridCellOverlap> cellsInfo;
    GridCellsInfoMap_t cellInfoMapIntersections;

    if(!m_gridsManager.getCellsInfoUnderLine( DrivenMassAreaMapName,
                                              p1, p2, machine.width,
                                              be_precise ? gridmap::SharedGridsManager::PRECISE : gridmap::SharedGridsManager::PRECISE_ONLY_IF_AVAILABLE,
                                              cellsInfo) )
        return AroResp::LoggingResp(1, "Error obtaining the overlaping cells", m_logger, LogLevel::ERROR, __FUNCTION__);

    for(auto& poly : intersectionPolys){
        if(poly.points.size() < 4)
            continue;
        auto cellsInfoInt = m_drivenMassAreaMap->getCellsOverlapUnderPolygon(poly);
        for(auto& cellInfo : cellsInfoInt){
            float overlap;
            cellInfo.overlap = std::max(0.0f, std::min(1.0f, cellInfo.overlap));
            if( cellsInfoMapGetValue(cellInfoMapIntersections, cellInfo.x, cellInfo.y, overlap) )
                cellInfo.overlap += overlap;
            cellInfo.overlap = std::max(0.0f, std::min(1.0f, cellInfo.overlap));
            cellInfoMapIntersections[cellInfo.x][cellInfo.y] = cellInfo.overlap;
        }
    }

    auto& prevCellsInfoMap = prevData.cellsInfoMass;
    auto newCellsInfoMap = cellsInfoVecToMap(cellsInfo);

    //remove the previous mass values of cells that do not overlap the current edge
    std::vector< std::pair<int, int> > cellsToRemove;
    for(auto& it_x : prevCellsInfoMap){
        auto x = it_x.first;
        for(auto& it_y : it_x.second){
            auto y = it_y.first;
            float overlap;
            if( !cellsInfoMapGetValue(newCellsInfoMap, x, y, overlap)
                    || !cellsInfoMapGetValue(cellInfoMapIntersections, x, y, overlap) ){
                cellsToRemove.emplace_back(std::make_pair(x, y));
            }
        }
    }
    for(auto& cell : cellsToRemove)
        removeCellsInfoFromMap(prevCellsInfoMap, cell.first, cell.second);

    //update the previous mass values based on the cells that are overlapped by the current edge
    for(auto& it_x : cellInfoMapIntersections){
        auto x = it_x.first;
        for(auto& it_y : it_x.second){
            auto y = it_y.first;
            auto& overlapInt = it_y.second;

            float overlap;
            if( !cellsInfoMapGetValue(newCellsInfoMap, x, y, overlap) )
                continue;

            overlap = std::max(0.0f, std::min(1.0f, overlap));
            auto overlapDiff = std::max(0.0f, std::min(1.0f, overlap - overlapInt));

            float& prevMassProp = prevCellsInfoMap[x][y];
            float newMassProp = massTotal * overlapDiff;
            float prevMassPropInt = prevData.massTotal * overlapInt;
            float newMassPropInt = massTotal * overlapInt;

            auto prevMassPropCompare = std::max(prevMassProp, prevMassPropInt);

            //add and update the mass for the intersection overlap
            if(newMassPropInt > prevMassPropCompare){//replace the value in current cell and add only the mass difference
                prevMassProp += (newMassPropInt - prevMassPropCompare);
                newMassProp += (newMassPropInt - prevMassPropCompare);
            }

            m_drivenMassAreaMap->addValue(x, y, newMassProp, false);
            removeCellsInfoFromMap(newCellsInfoMap, x, y);//remove not to add again in next step
        }
    }

    //update the mass values of the remaining (new) cells that are overlapped by the current edge
    for(auto& it_x : newCellsInfoMap){
        auto x = it_x.first;
        for(auto& it_y : it_x.second){
            auto y = it_y.first;
            auto overlap = it_y.second;
            overlap = std::max(0.0f, std::min(1.0f, overlap));
            float newMassProp = massTotal * overlap;
            prevCellsInfoMap[x][y] = newMassProp;
            m_drivenMassAreaMap->addValue(x, y, newMassProp, false);
        }
    }
    return AroResp(0, "OK" );

}

AroResp DrivenMassAreaManager::addData_cellsAnalysis(const Machine &machine, const Point &pt, double bunker_mass, bool be_precise)
{
    float massTotal = std::max(0.0, machine.weight) + std::max(0.0, bunker_mass);

    auto it = m_prevData.find(machine.id);
    if(it == m_prevData.end() || it->second.points.empty()){//first point
        Data& prevData = m_prevData[machine.id];
        prevData.points.emplace_back(pt);
        prevData.massTotal = massTotal;
        return AroResp(0, "OK" );
    }

    bool isReverse = false;

    Data& prevData = it->second;
    std::vector<Point>& points = prevData.points;

    if( geometry::calc_dist(pt, points.back()) < 1e-3 ){//same point as the last one
        if(points.size() == 1 || massTotal <= prevData.massTotal)//no previous edge or same edge
            return AroResp(0, "OK" );
        isReverse = true;// same edge with higher mass
    }
    else if(points.size() == 2){//a previous edge exists
        auto angle = geometry::get_angle( points.front(), points.back(), pt );
        isReverse = ( std::fabs(angle) < M_PI_2 );
        points.front() = points.back();
        points.back() = pt;
    }
    else//first edge
        points.emplace_back(pt);

    prevData.massTotal = massTotal;

    auto gpsFrontDisplacement = m_defaultGPSFrontDisplacement;
    auto it_mfd = m_GPSFrontDisplacements.find(machine.id);
    if(it_mfd != m_GPSFrontDisplacements.end())
        gpsFrontDisplacement = it_mfd->second;

    Point p1 = points.front(), p2 = points.back();
    if(machine.length > 1e-9 && gpsFrontDisplacement > -1e-9){//extend the edge based on machine length and relative location of GPS
        float deltaFront = machine.length * gpsFrontDisplacement;
        float deltaBack = machine.length - deltaFront;
        p2 = geometry::getPointInLineAtDist(p2, p1, -deltaFront);
        p1 = geometry::getPointInLineAtDist(p1, p2, -deltaBack);
    }
    prevData.machineProj = geometry::createRectangleFromLine(p1, p2, machine.width);

    if( !arolib::geometry::in_polygon(prevData.machineProj.points, m_field.outer_boundary, false)
            && !arolib::geometry::intersects(prevData.machineProj.points, m_field.outer_boundary) ){// edge projection outside of the field
        prevData.cellsInfoMass.clear();
        prevData.cellsInfoOverlap.clear();
        return AroResp(0, "OK" );
    }

    std::vector<gridmap::GridmapLayout::GridCellOverlap> cellsInfo;

    if(!m_gridsManager.getCellsInfoUnderLine( DrivenMassAreaMapName,
                                              p1, p2, machine.width,
                                              be_precise ? gridmap::SharedGridsManager::PRECISE : gridmap::SharedGridsManager::PRECISE_ONLY_IF_AVAILABLE,
                                              cellsInfo) )
        return AroResp::LoggingResp(1, "Error obtaining the overlaping cells", m_logger, LogLevel::ERROR, __FUNCTION__);

    auto& prevCellsInfoMap = prevData.cellsInfoMass;
    auto newCellsInfoMap = cellsInfoVecToMap(cellsInfo);

    //update the previous mass values based on the cells that are overlapped by the current edge
    std::vector< std::pair<int, int> > cellsToRemove;
    for(auto& it_x : prevCellsInfoMap){
        auto x = it_x.first;
        for(auto& it_y : it_x.second){
            auto y = it_y.first;
            auto& prevMassProp = it_y.second;
            float overlap;
            if( !cellsInfoMapGetValue(newCellsInfoMap, x, y, overlap) ){//a previous cell is not present in the current edge cells
                cellsToRemove.emplace_back(std::make_pair(x, y));
                continue;
            }
            overlap = std::max(0.0f, std::min(1.0f, overlap));
            float newMassProp = massTotal * overlap;
            if(isReverse){
                if(newMassProp > prevMassProp){//replace the value in current cell and add only the mass difference
                    prevCellsInfoMap[x][y] = newMassProp;
                    newMassProp -= prevMassProp;
                }
                else // a higher cell mass was already added -> do not add more
                    newMassProp = 0;
            }
            else{//update the cell mass and add the new cell mass
                prevCellsInfoMap[x][y] = newMassProp + prevMassProp;
            }

            m_drivenMassAreaMap->addValue(x, y, newMassProp, false);
            removeCellsInfoFromMap(newCellsInfoMap, x, y);//remove not to add again in next step
        }
    }
    for(auto& cell : cellsToRemove)
        removeCellsInfoFromMap(prevCellsInfoMap, cell.first, cell.second);

    //update the mass values of the remaining (new) cells that are overlapped by the current edge
    for(auto& it_x : newCellsInfoMap){
        auto x = it_x.first;
        for(auto& it_y : it_x.second){
            auto y = it_y.first;
            auto overlap = it_y.second;
            overlap = std::max(0.0f, std::min(1.0f, overlap));
            float newMassProp = massTotal * overlap;
            prevCellsInfoMap[x][y] = newMassProp;
            m_drivenMassAreaMap->addValue(x, y, newMassProp, false);
        }
    }

    return AroResp(0, "OK" );

}

}
