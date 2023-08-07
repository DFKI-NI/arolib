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
 
#include "arolib/components/workedareamanager.h"

namespace arolib {

const std::string WorkedAreaManager::RemainingAreaMapName = "REM_AREA";

WorkedAreaManager::WorkedAreaManager(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_gridsManager(logLevel)
{
    m_gridsManager.logger().setParent(loggerPtr());
}

AroResp WorkedAreaManager::init(const Field &field, double cellsize, bool removeHeadland)
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

    auto ram = initRemainingAreaMap();

    if(!ram->createGrid(minX-cellsize, maxX+cellsize, minY-cellsize, maxY+cellsize, cellsize)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating grid.");
        return AroResp(1, "Error creating grid" );;
    }

    if(!ram->setPolygon(field.outer_boundary, removeHeadland ? 0.0 : 1.0)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error initializing grid with outer boundary.");
        return AroResp(1, "Error initializing grid with outer boundary." );;
    }

    if(removeHeadland){
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Removing headland...");
        for(size_t i = 0 ; i < field.subfields.size() ; ++i){
            const auto& sf = field.subfields.at(i);
            if(sf.boundary_outer.points.size() > 2 && sf.boundary_inner.points.size() > 2){
                if(!ram->updatePolygonProportionally(sf.boundary_inner, 1.0)){
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error initializing grid with inner boundary of subfield " + std::to_string(i));
                    return AroResp(1, "Error initializing grid with inner boundary of subfield " + std::to_string(i) );;
                }
            }
            else
                logger().printOut(LogLevel::DEBUG, __FUNCTION__, 10, "Unable to remove headland from subfield [", i, "]: "
                                                                     "OB(", sf.boundary_outer.points.size(), "); IB(", sf.boundary_inner.points.size(), ")");
        }
    }

    m_remainingAreaMap = ram;
    m_gridsManager.addGrid(RemainingAreaMapName, m_remainingAreaMap, true);

    m_field = field;
    m_ready = true;
    return AroResp(0, "OK" );
}

AroResp WorkedAreaManager::init(const Field &field, const std::vector<Machine> &machines, bool removeHeadland)
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

    return init(field, cellsize*0.5, removeHeadland);
}

AroResp WorkedAreaManager::init(const Field &field, const ArolibGrid_t &remainingAreaMap, bool initMap)
{
    clear();

    if(!remainingAreaMap.isAllocated()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid map (not allocated).");
        return AroResp(1, "Invalid map (not allocated)." );;
    }

    auto ram = initRemainingAreaMap();
    *ram = remainingAreaMap;

    if(initMap){
        Polygon boundary_exp;
        if(!arolib::geometry::offsetPolygon(field.outer_boundary, boundary_exp, ram->getCellsize(), true)){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting the outer boundary.");
            return AroResp(1, "Error offsetting the outer boundary." );;
        }

        if(!ram->expandGridFromPolygon(boundary_exp.points, 1.0)){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error expanding the grid.");
            return AroResp(1, "Error expanding the grid." );;
        }
    }

    m_remainingAreaMap = ram;
    m_gridsManager.addGrid(RemainingAreaMapName, m_remainingAreaMap, true);

    m_field = field;
    m_ready = true;
    return AroResp(0, "OK" );

}

void WorkedAreaManager::setGridCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim)
{
    m_gridsManager.setCellsInfoManager(cim);
}

bool WorkedAreaManager::isReady() const
{
    return m_ready;
}

void WorkedAreaManager::clear()
{
    m_ready = false;
    m_field.clear();
    m_remainingAreaMap = nullptr;
    m_gridsManager.clearAll();
    m_drivenEdges.clear();
}

void WorkedAreaManager::forgetMachineLocation(const MachineId_t &machineId)
{
    m_drivenEdges.erase(machineId);
}

void WorkedAreaManager::forgetAllMachineLocations()
{
    m_drivenEdges.clear();
}

AroResp WorkedAreaManager::addPoint(const Machine &machine, const Point &pt, bool be_precise)
{
    if(!m_ready){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Not ready/initialized.");
        return AroResp(1, "Not ready/initialized." );;
    }
    if(machine.working_width < 1e-5){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid machine working width.");
        return AroResp(1, "Invalid machine working width." );;
    }    
    if( !m_gridsManager.hasGrid(RemainingAreaMapName) )
        return AroResp::LoggingResp(1, "The gridmap has not been initialized", m_logger, LogLevel::ERROR, __FUNCTION__);

    auto it = m_drivenEdges.find(machine.id);
    if(it == m_drivenEdges.end() || it->second.empty()){
        m_drivenEdges[machine.id].emplace_back(pt);
        return AroResp(0, "OK" );
    }

    std::vector<Point>& points = it->second;
    if(points.size() == 2){
        points.front() = points.back();
        points.back() = pt;
    }
    else
        points.emplace_back(pt);

    if( geometry::calc_dist(points.front(), points.back()) < 1e-3 )
        return AroResp(0, "OK" );

    Polygon machineProj = geometry::createRectangleFromLine(points.front(), points.back(), machine.working_width);
    if( !arolib::geometry::in_polygon(machineProj.points, m_field.outer_boundary, false) && !arolib::geometry::intersects(machineProj.points, m_field.outer_boundary) )
        return AroResp(0, "OK" );

    std::vector<gridmap::GridmapLayout::GridCellOverlap> cellsInfo;

    if(!m_gridsManager.getCellsInfoUnderLine( RemainingAreaMapName,
                                              points.front(), points.back(), machine.working_width,
                                              be_precise ? gridmap::SharedGridsManager::PRECISE : gridmap::SharedGridsManager::PRECISE_ONLY_IF_AVAILABLE,
                                              cellsInfo) )
        return AroResp::LoggingResp(1, "Error obtaining the overlaping cells", m_logger, LogLevel::ERROR, __FUNCTION__);

    static const ArolibGrid_t::FuncGetNewValueGCOverlap funcGetNewValue =
            [](const std::shared_ptr<const ArolibGridVal_t>& currentVal, const gridmap::GridmapLayout::GridCellOverlap& cell) -> std::shared_ptr<ArolibGridVal_t>
            {
                float overlap = std::min(1.0f, std::max(0.0f, cell.overlap) );
                if(!currentVal || std::isnan(*currentVal))
                    return std::make_shared<ArolibGridVal_t>(1.0-overlap);

                ArolibGridVal_t newVal = std::min( (ArolibGridVal_t)1.0, std::max( (ArolibGridVal_t)0.0, *currentVal - overlap ) );

                return std::make_shared<ArolibGridVal_t>(newVal);
            };

    if( !m_remainingAreaMap->setCellsValue(cellsInfo, funcGetNewValue) ){
        return AroResp::LoggingResp(1, "Error setting values in the overlaping cells", m_logger, LogLevel::ERROR, __FUNCTION__);
    }

    return AroResp(0, "OK" );
}

gridmap::SharedGridsManager::ConstGridPtr WorkedAreaManager::getRemainingAreaMap() const
{
    return m_gridsManager.getGrid(RemainingAreaMapName);
}

gridmap::SharedGridsManager::GridPtr WorkedAreaManager::initRemainingAreaMap()
{
    auto ram = std::make_shared<ArolibGrid_t>();
    ram->logger().setParent(loggerPtr());
    return ram;
}

}
