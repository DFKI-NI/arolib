/*
 * Copyright 2021-2025 DFKI GmbH
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

#include "arolib/geometry/geometry_helper.hpp"

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
        if(m.width > 1e-3)
            cellsize = std::min(cellsize, m.width);
    }

    if(cellsize > 1e3){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid width in machines.");
        return AroResp(1, "Invalid width in machines." );;
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

void DrivenMassAreaManager::setCellDistanceThreshold(float dist)
{
    m_cellDistanceThreshold = dist;
}

void DrivenMassAreaManager::setUsePolygonIntersection(bool usePolygonIntersection)
{
    m_usePolygonIntersection = usePolygonIntersection;
}

void DrivenMassAreaManager::setProjMachineLength(float k)
{
    m_projMachineLength = std::max(0.0f, std::min(1.0f, k));
}

AroResp DrivenMassAreaManager::addData(const Machine &machine, Point pt, double bunker_mass)
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

    if(m_filterAng)
        pt = getFilteredPoint(machine, pt);

    if(m_precision == PrecisionOption::LOW_PRECISION)
        return addData_simple(machine, pt, bunker_mass);

    if(m_usePolygonIntersection)
        return addData_polygonIntersection(machine, pt, bunker_mass, m_precision == PrecisionOption::HIGH_PRECISION);
    return addData_cellsAnalysis(machine, pt, bunker_mass, m_precision == PrecisionOption::HIGH_PRECISION);
}

DrivenMassAreaManager::ConstGridPtr DrivenMassAreaManager::getDrivenMassAreaMap() const
{
    return m_gridsManager.getGrid(DrivenMassAreaMapName);
}

Polygon DrivenMassAreaManager::getEdgeProjection(const Machine &machine, Point pBack, Point pFront, double width, Point *pBackEd, Point *pFrontEd)
{
    if(width < 1e-9)
        width = machine.width;
    width = std::max(0.0, width);

    auto gpsFrontDisplacement = m_defaultGPSFrontDisplacement;
    auto it_mfd = m_GPSFrontDisplacements.find(machine.id);
    if(it_mfd != m_GPSFrontDisplacements.end())
        gpsFrontDisplacement = it_mfd->second;

    gpsFrontDisplacement = std::max(0.0f, gpsFrontDisplacement);

    if(machine.length > 1e-9){//extend the edge based on machine length and relative location of GPS
        float deltaFront = machine.length * gpsFrontDisplacement;
        pFront = geometry::getPointInLineAtDist(pFront, pBack, -deltaFront);
        pBack = geometry::getPointInLineAtDist(pBack, pFront, deltaFront - machine.length * m_projMachineLength);
    }

    if(pBackEd)
        *pBackEd = pBack;
    if(pBackEd)
        *pFrontEd = pFront;

    return geometry::createRectangleFromLine(pBack, pFront, width);
}

Polygon DrivenMassAreaManager::getMachineProjection(const Machine &machine, Point pBack, Point pFront, double width, Point *pBackEd, Point *pFrontEd)
{
    if(width < 1e-9)
        width = machine.width;
    width = std::max(0.0, width);

    auto gpsFrontDisplacement = m_defaultGPSFrontDisplacement;
    auto it_mfd = m_GPSFrontDisplacements.find(machine.id);
    if(it_mfd != m_GPSFrontDisplacements.end())
        gpsFrontDisplacement = it_mfd->second;

    gpsFrontDisplacement = std::max(0.0f, gpsFrontDisplacement);

    auto pFrontTmp = pFront;
    auto pBackTmp = pBack;
    if(machine.length > 1e-9){//extend the edge based on machine length and relative location of GPS
        float deltaFront = machine.length * gpsFrontDisplacement;
        pFrontTmp = geometry::getPointInLineAtDist(pFront, pBack, -deltaFront);
        pBackTmp = geometry::getPointInLineAtDist(pFront, pBack, machine.length * m_projMachineLength);
    }

    if(pBackEd)
        *pBackEd = pBackTmp;
    if(pFrontEd)
        *pFrontEd = pFrontTmp;
    if(pBackTmp == pFrontTmp){
        Polygon poly;
        if(width < 1e-9)
            poly.points.push_back(pFrontTmp);
        else{
            Point ptTmp = geometry::rotate(pBack, pFrontTmp, M_PI_2);
            poly.points.push_back( geometry::getPointInLineAtDist(pFrontTmp, ptTmp, 0.5 * width) );
            poly.points.push_back( geometry::getPointInLineAtDist(poly.points.back(), pFrontTmp, width) );
        }
        return poly;
    }
    if(width < 1e-9){
        Polygon poly;
        poly.points.push_back( pBackTmp );
        poly.points.push_back( pFrontTmp );
        return poly;
    }
    return geometry::createRectangleFromLine(pBackTmp, pFrontTmp, width);

}


bool DrivenMassAreaManager::filterWithRemainingAreaMap(const ArolibGrid_t &ram, float threshold, bool be_precise)
{
    if(!m_drivenMassAreaMap || !m_drivenMassAreaMap->isAllocated())
        return false;


    if(m_drivenMassAreaMap->equalGeometry(ram)){
        for(size_t x = 0 ; x < m_drivenMassAreaMap->getSizeX() ; x++){
            for(size_t y = 0 ; y < m_drivenMassAreaMap->getSizeY() ; y++){
                if(!ram.hasValue(x, y) || ram.getValue(x, y) > threshold)
                    m_drivenMassAreaMap->setNoValue(x, y);
            }
        }

        return true;
    }

    return filterWithRemainingAreaMap(std::make_shared<ArolibGrid_t>(ram), threshold, be_precise);

}

bool DrivenMassAreaManager::filterWithRemainingAreaMap(std::shared_ptr<const ArolibGrid_t> ram, float threshold, bool be_precise)
{
    if(!ram || !ram->isAllocated() || !m_drivenMassAreaMap || !m_drivenMassAreaMap->isAllocated())
        return false;


    threshold = std::max(0.0001f, std::min(0.9999f, threshold));

    if(m_drivenMassAreaMap->equalGeometry(*ram)){
        for(size_t x = 0 ; x < m_drivenMassAreaMap->getSizeX() ; x++){
            for(size_t y = 0 ; y < m_drivenMassAreaMap->getSizeY() ; y++){
                if(!ram->hasValue(x, y) || ram->getValue(x, y) > threshold)
                    m_drivenMassAreaMap->setNoValue(x, y);
            }
        }

        return true;
    }

    gridmap::SharedGridsManager gridsManagerTmp;
    gridsManagerTmp.addGrid("ram", ram, true);
    gridsManagerTmp.setCellsInfoManager(m_gridsManager.getCellsInfoManager());

    Point p0, p1;
    double width, area = m_drivenMassAreaMap->getCellArea();
    bool errorTmp;
    Polygon cellPoly;
    std::vector<gridmap::GridmapLayout::GridCellOverlap> cellsInfo;

    for(size_t x = 0 ; x < m_drivenMassAreaMap->getSizeX() ; x++){
        for(size_t y = 0 ; y < m_drivenMassAreaMap->getSizeY() ; y++){
//            m_drivenMassAreaMap->getCellPolygon(x, y, cellPoly);
//            m_drivenMassAreaMap->getCellCenter(x, y, cellPoly.points.back());
//            size_t countInvalidPts = 0;
//            for(const auto& p : cellPoly.points){
//                ram->getValue(p, &errorTmp);
//                countInvalidPts += errorTmp;
//            }
//            if(countInvalidPts > 3){  // workarround because noValues in the ram are added as 0 (worked) in AVERAGE_TOTAL computation
//                m_drivenMassAreaMap->setNoValue(x, y);
//                continue;
//            }

            m_drivenMassAreaMap->getCellPolygonAsLine(x, y, p0, p1, width);

            if(!gridsManagerTmp.getCellsInfoUnderLine( "ram",
                                                       p0, p1, width,
                                                       be_precise ? gridmap::SharedGridsManager::PRECISE : gridmap::SharedGridsManager::PRECISE_ONLY_IF_AVAILABLE,
                                                       cellsInfo) )
                continue;

            auto value = ram->getCellsComputedValue( cellsInfo,
                                                     1.0,
                                                     ArolibGrid_t::AVERAGE_TOTAL,
                                                     area,
                                                     false,
                                                     &errorTmp );
            if(errorTmp || value <= threshold)
                continue;

            m_drivenMassAreaMap->setNoValue(x, y);

        }
    }

    return true;

}

std::string DrivenMassAreaManager::toCellKey(size_t x, size_t y)
{
    return(std::to_string(x) + "," + std::to_string(y));
}

void DrivenMassAreaManager::toCellCoords(const std::string &key, size_t &x, size_t &y)
{
    auto ind = key.find(',');
    x = std::stoi(key.substr(0, ind));
    y = std::stoi(key.substr(ind+1));
}

gridmap::GridmapLayout DrivenMassAreaManager::getCellMiniLayout(size_t &x, size_t &y, bool be_precise)
{
    size_t factor = be_precise ? 15 : 5;
    gridmap::GridmapLayout lo;
    Point corner;
    m_drivenMassAreaMap->getCellMinCorner(x, y, corner);
    lo.init(corner.x, corner.y, factor, factor, m_drivenMassAreaMap->getCellsize() / factor);
    return lo;
}

DrivenMassAreaManager::GridPtr DrivenMassAreaManager::initDrivenMassAreaMap()
{
    auto ram = std::make_shared<ArolibGrid_t>();
    ram->logger().setParent(loggerPtr());
    return ram;
}

std::map<std::string, float> DrivenMassAreaManager::cellsInfoVecToMap(const std::vector<gridmap::GridmapLayout::GridCellOverlap> &vec)
{
    std::map<std::string, float> ret;
    for(auto& cellInfo : vec){
        auto cellKey = toCellKey(cellInfo.x, cellInfo.y);
        auto it = ret.find(cellKey);
        if( it == ret.end() )
            ret[cellKey] = cellInfo.overlap;
        else if( cellInfo.overlap > it->second ){
            it->second = cellInfo.overlap;
        }
    }
    return ret;
}

bool DrivenMassAreaManager::isCellCloseToMachine(const std::string &cell, const CellData& cellData, const Polygon& machineProj)
{
    if(m_cellDistanceThreshold < 1e-9)
        return false;

    for(auto& proj : cellData.machineProj){
        for(size_t i = 0 ; i+1 < proj->points.size() ; ++i){
            if( geometry::calc_dist_to_linestring(machineProj.points, proj->points.at(i), false) <  m_cellDistanceThreshold)
                return true;
        }
    }

    size_t x, y;
    toCellCoords(cell, x, y);
    Polygon cellPoly;
    m_drivenMassAreaMap->getCellPolygon(x, y, cellPoly);
    for(size_t i = 0 ; i+1 < cellPoly.points.size() ; ++i){
        if( geometry::calc_dist_to_linestring(machineProj.points, cellPoly.points.at(i), false) <  m_cellDistanceThreshold)
            return true;
    }
    return false;
}

AroResp DrivenMassAreaManager::addData_simple(const Machine &machine, const Point &pt, double bunker_mass)
{
    float massTotal = std::max(0.0, machine.weight) + std::max(0.0, bunker_mass);

    auto it = m_prevData.find(machine.id);
    if(it == m_prevData.end() || it->second.points.empty()){//first point
        Data& prevData = m_prevData[machine.id];
        prevData.points.emplace_back(pt);
        return AroResp(0, "OK" );
    }

    double edgeWidth = machine.width > m_drivenMassAreaMap->getCellsize() ? machine.width - 0.1 * m_drivenMassAreaMap->getCellsize() : machine.width;

    Data& prevData = it->second;
    std::vector<Point>& points = prevData.points;
    std::vector<Point> pointsStill;

    if( geometry::calc_dist(pt, points.back()) < 1e-3 ){//same point as the last one
        if(points.size() > 1){
            Point ptDisp;
            getEdgeProjection(machine, points.front(), pt, edgeWidth, nullptr, &ptDisp);
            Point ptTmp = geometry::rotate(ptDisp, points.front(), M_PI_2);
            pointsStill.push_back( geometry::getPointInLineAtDist(ptDisp, ptTmp, 0.5 * edgeWidth) );
            pointsStill.push_back( geometry::getPointInLineAtDist(pointsStill.back(), ptDisp, edgeWidth) );
        }
        points = {pt};
        return AroResp(0, "OK" );
    }

    if(points.size() > 1){
        points.front() = points.back();
        points.back() = pt;
    }
    else
        points.push_back(pt);

    std::set<std::string> overlappedCells;
    Point p1, p2;
    double width;
    if(!pointsStill.empty()){
        p1 = pointsStill.front();
        p2 = pointsStill.back();
        width = 0;
    }
    else{
        p1 = points.front();
        p2 = points.back();
        width = edgeWidth;
        auto edgeProj = getEdgeProjection(machine, p1, p2, width, &p1, &p2);
        if( !arolib::geometry::in_polygon(edgeProj.points, m_field.outer_boundary, false)
                && !arolib::geometry::intersects(edgeProj.points, m_field.outer_boundary) ){// edge projection outside of the field
            // prevData.cellsData.clear();
            return AroResp(0, "OK" );
        }
    }

    auto indexMap = m_drivenMassAreaMap->getCellsUnderLine(p1, p2, width);
    if (indexMap.empty()){
        prevData.cellsData.clear();
        return AroResp(0, "OK" );
    }

    for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
        auto yRanges = indexMap.getColumn(x);
        for(const auto& yRange : yRanges){
            for (int y = yRange.first ; y <= yRange.second ; ++y)
                overlappedCells.insert( toCellKey(x, y) );
        }
    }

    auto pMachineProj = std::make_shared<Polygon>( getMachineProjection(machine, points.front(), points.back(), machine.width) );

    if(pointsStill.empty()){//remove the cells that are not overlapped anymore
        for (auto it = prevData.cellsData.cbegin(); it != prevData.cellsData.cend();){
            if(overlappedCells.find(it->first) == overlappedCells.end() && !isCellCloseToMachine(it->first, it->second, *pMachineProj)){
                it = prevData.cellsData.erase(it);
                continue;
            }
            ++it;
        }
    }
    for(auto overlappedCell : overlappedCells){
        auto& cellData = prevData.cellsData[overlappedCell];

        cellData.machineProj.emplace_back(pMachineProj);

        if(cellData.mass >= massTotal)
            continue;

        size_t cellX, cellY;
        toCellCoords(overlappedCell, cellX, cellY);

        if(!m_drivenMassAreaMap->hasValue(cellX, cellY))
            m_drivenMassAreaMap->setValue(cellX, cellY, massTotal);
        else{
            auto val = m_drivenMassAreaMap->getValue(cellX, cellY);
            m_drivenMassAreaMap->setValue(cellX, cellY, val - cellData.mass + massTotal);
        }

        cellData.mass = massTotal;
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
        return AroResp(0, "OK" );
    }

    Data& prevData = it->second;
    std::vector<Point>& points = prevData.points;
    std::vector<Point> pointsStill;



    if( geometry::calc_dist(pt, points.back()) < 1e-3 ){//same point as the last one
        if(points.size() > 1){
            Point ptDisp;
            getEdgeProjection(machine, points.front(), pt, machine.width, nullptr, &ptDisp);
            Point ptTmp = geometry::rotate(ptDisp, points.front(), M_PI_2);
            pointsStill.push_back( geometry::getPointInLineAtDist(ptDisp, ptTmp, 0.5 * machine.width) );
            pointsStill.push_back( geometry::getPointInLineAtDist(pointsStill.back(), ptDisp, machine.width) );
        }
        points = {pt};
        return AroResp(0, "OK" );
    }

    if(points.size() > 1){
        points.front() = points.back();
        points.back() = pt;
    }
    else
        points.push_back(pt);

    std::set<std::string> overlappedCells;
    Point p1, p2;
    double width;
    if(!pointsStill.empty()){
        p1 = pointsStill.front();
        p2 = pointsStill.back();
        width = 0;
    }
    else{
        p1 = points.front();
        p2 = points.back();
        width = machine.width * 0.999;
        auto edgeProj = getEdgeProjection(machine, p1, p2, width, &p1, &p2);
        if( !arolib::geometry::in_polygon(edgeProj.points, m_field.outer_boundary, false)
                && !arolib::geometry::intersects(edgeProj.points, m_field.outer_boundary) ){// edge projection outside of the field
            // prevData.cellsData.clear();
            return AroResp(0, "OK" );
        }
    }

    auto indexMap = m_drivenMassAreaMap->getCellsUnderLine(p1, p2, width);
    if (indexMap.empty()){
//        prevData.cellsData.clear();
        return AroResp(0, "OK" );
    }

    for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
        auto yRanges = indexMap.getColumn(x);
        for(const auto& yRange : yRanges){
            for (int y = yRange.first ; y <= yRange.second ; ++y)
                overlappedCells.insert( toCellKey(x, y) );
        }
    }

    auto pMachineProj = std::make_shared<Polygon>( getMachineProjection(machine, points.front(), points.back(), machine.width) );

    if(pointsStill.empty()){//remove the cells that are not overlapped anymore
        for (auto it = prevData.cellsData.cbegin(); it != prevData.cellsData.cend();){
            if(overlappedCells.find(it->first) == overlappedCells.end() && !isCellCloseToMachine(it->first, it->second, *pMachineProj)){
                it = prevData.cellsData.erase(it);
                continue;
            }
            ++it;
        }
    }

    for(auto overlappedCell : overlappedCells){
        size_t cellX, cellY;
        toCellCoords(overlappedCell, cellX, cellY);

        auto lo = getCellMiniLayout(cellX, cellY, be_precise);
        indexMap = lo.getCellsUnderLine(p1, p2, width);
        if(indexMap.empty())
            continue;

        auto& cellData = prevData.cellsData[overlappedCell];

        cellData.machineProj.emplace_back(pMachineProj);

        size_t countMiniCells = 0;
        float adjustedMass = 0;
        for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
            auto yRanges = indexMap.getColumn(x);
            for(const auto& yRange : yRanges){
                for (int y = yRange.first ; y <= yRange.second ; ++y){
                    countMiniCells++;
                    auto miniCellKey = toCellKey(x, y);
                    auto it = cellData.overlappedMiniCells.find(miniCellKey);
                    if(it == cellData.overlappedMiniCells.end()){
                        adjustedMass += massTotal;
                        cellData.overlappedMiniCells[miniCellKey] = massTotal;
                    }
                    else if(it->second > massTotal){
                        adjustedMass += (massTotal - it->second);
                        it->second = massTotal;
                    }
                }
            }
        }
        double factor = 1.0 / (lo.getSizeX() * lo.getSizeY());
        if(!m_drivenMassAreaMap->hasValue(cellX, cellY)){
            m_drivenMassAreaMap->setValue(cellX, cellY, massTotal * countMiniCells * factor);
            continue;
        }
        adjustedMass *= factor;
        auto val = m_drivenMassAreaMap->getValue(cellX, cellY);
        m_drivenMassAreaMap->setValue(cellX, cellY, val + adjustedMass);

    }

    return AroResp(0, "OK" );

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
        if(points.size() == 1 || massTotal <= prevData.massTotal){//no previous edge or same edge with lower/equal mass
            prevData.massTotal = std::max(massTotal, prevData.massTotal);
            return AroResp(0, "OK" );
        }
        sameEdge = true;// same edge with higher mass
    }
    else if(points.size() == 2){//a previous edge exists
        points.front() = points.back();
        points.back() = pt;
    }
    else//first edge
        points.emplace_back(pt);

    prevData.massTotal = massTotal;

    Point p1 = points.front(), p2 = points.back();
    auto edgeProj = getEdgeProjection(machine, p1, p2, machine.width, &p1, &p2);

    std::vector<Polygon> intersectionPolys;
    if(sameEdge){
        intersectionPolys.push_back(prevData.machineProj);
    }
    else{
        if( !arolib::geometry::in_polygon(edgeProj.points, m_field.outer_boundary, false)
                && !arolib::geometry::intersects(edgeProj.points, m_field.outer_boundary) ){// edge projection outside of the field
            // prevData.cellsData.clear();
            return AroResp(0, "OK" );
        }
        intersectionPolys = geometry::get_intersection(edgeProj, prevData.machineProj);
        prevData.machineProj = edgeProj;
    }

    std::vector<gridmap::GridmapLayout::GridCellOverlap> cellsInfo;
    std::map<std::string, float> cellInfoMapIntersections;

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
            cellInfo.overlap = std::max(0.0f, std::min(1.0f, cellInfo.overlap));
            auto cellKey = toCellKey(cellInfo.x, cellInfo.y);
            auto it = cellInfoMapIntersections.find(cellKey);
            if(it == cellInfoMapIntersections.end())
                cellInfoMapIntersections[cellKey] = std::max(0.0f, std::min(1.0f, cellInfo.overlap));
            else
                it->second = std::max(0.0f, std::min(1.0f, cellInfo.overlap + it->second));
        }
    }

    auto& prevCellsDataMap = prevData.cellsData;
    auto newCellsOverlapMap = cellsInfoVecToMap(cellsInfo);

    auto pMachineProj = std::make_shared<Polygon>( getMachineProjection(machine, points.front(), points.back(), machine.width) );

    //remove the previous mass values of cells that do not overlap the current edge
    for (auto it = prevCellsDataMap.begin(); it != prevCellsDataMap.end();){
        if( ( newCellsOverlapMap.find(it->first) == newCellsOverlapMap.end() || cellInfoMapIntersections.find(it->first) == cellInfoMapIntersections.end() )
                && !isCellCloseToMachine(it->first, it->second, *pMachineProj) ){
            it = prevCellsDataMap.erase(it);
            continue;
        }
        ++it;
    }

    //update the previous mass values based on the cells that are overlapped by the current edge
    for(auto& it_int : cellInfoMapIntersections){
        auto it_new = newCellsOverlapMap.find(it_int.first);
        if(it_new == newCellsOverlapMap.end())
            continue;
        auto it_prev = prevCellsDataMap.find(it_int.first);
        if(it_prev == prevCellsDataMap.end())
            continue;

        float overlap = std::max(0.0f, std::min(1.0f, it_new->second));
        float overlapInt = it_int.second;
        auto overlapDiff = std::max(0.0f, std::min(1.0f, overlap - overlapInt));

        float& prevMassProp = it_prev->second.mass;
        float newMassProp = massTotal * overlapDiff;
        float prevMassPropInt = prevData.massTotal * overlapInt;
        float newMassPropInt = massTotal * overlapInt;

        auto prevMassPropCompare = std::max(prevMassProp, prevMassPropInt);

        //add and update the mass for the intersection overlap
        if(newMassPropInt > prevMassPropCompare){//replace the value in current cell and add only the mass difference
            prevMassProp += (newMassPropInt - prevMassPropCompare);
            newMassProp += (newMassPropInt - prevMassPropCompare);
        }
        size_t x, y;
        toCellCoords(it_int.first, x, y);
        m_drivenMassAreaMap->addValue(x, y, newMassProp, false);
        newCellsOverlapMap.erase(it_new);//remove not to add again in next step
        it_prev->second.machineProj.emplace_back(pMachineProj);
    }

    //update the mass values of the remaining (new) cells that are overlapped by the current edge
    for(auto& it : newCellsOverlapMap){
        float overlap = std::max(0.0f, std::min(1.0f, it.second));
        float newMassProp = massTotal * overlap;
        auto& cellData = prevCellsDataMap[it.first];
        cellData.machineProj.emplace_back(pMachineProj);
        cellData.mass = newMassProp;
        size_t x, y;
        toCellCoords(it.first, x, y);
        m_drivenMassAreaMap->addValue(x, y, newMassProp, false);
    }
    return AroResp(0, "OK" );

}

Point DrivenMassAreaManager::getFilteredPoint(const Machine &machine, const Point &pt)
{
    // @todo!!!
    return pt;
}

}
