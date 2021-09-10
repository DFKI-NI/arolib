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
 
#include "arolib/components/workedareamanager.h"

namespace arolib {

WorkedAreaManager::WorkedAreaManager(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{

}

AroResp WorkedAreaManager::init(const Field &field, double cellsize, bool removeHeadland)
{
    clear();

    if(cellsize < 1e-6){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid cell size.");
        return AroResp(1, "Invalid cell size." );;
    }

    double minX, maxX, minY, maxY;
    if(!arolib::geometry::getPolygonLimits(field.outer_boundary, minX, maxX, minY, maxY)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error obtaining the outer boundary limits.");
        return AroResp(1, "Error obtaining the outer boundary limits." );;
    }

    if( (maxX-minX) < 10*cellsize || (maxY-minY) < 10*cellsize ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Resolution (cell size) too low for the outer boundary.");
        return AroResp(1, "Resolution (cell size) too low for the outer boundary." );;
    }

    if(!m_remainingAreaMap.createGrid(minX-cellsize, maxX+cellsize, minY-cellsize, maxY+cellsize, cellsize)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error creating grid.");
        return AroResp(1, "Error creating grid" );;
    }

    if(!m_remainingAreaMap.setPolygon(field.outer_boundary, removeHeadland ? 0.0 : 1.0)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error initializing grid with outer boundary.");
        return AroResp(1, "Error initializing grid with outer boundary." );;
    }

    if(removeHeadland){
        m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Removing headland...");
        for(size_t i = 0 ; i < field.subfields.size() ; ++i){
            const auto& sf = field.subfields.at(i);
            if(sf.boundary_outer.points.size() > 2 && sf.boundary_inner.points.size() > 2){
                if(!m_remainingAreaMap.updatePolygonProportionally(sf.boundary_inner, 1.0)){
                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error initializing grid with inner boundary of subfield " + std::to_string(i));
                    return AroResp(1, "Error initializing grid with inner boundary of subfield " + std::to_string(i) );;
                }
            }
            else
                m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, 10, "Unable to remove headland from subfield [", i, "]: "
                                                                     "OB(", sf.boundary_outer.points.size(), "); IB(", sf.boundary_inner.points.size(), ")");
        }
    }

    m_field = field;
    m_ready = true;
    return AroResp(0, "OK" );
}

AroResp WorkedAreaManager::init(const Field &field, const std::vector<Machine> &machines, bool removeHeadland)
{
    clear();

    if(machines.empty()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No machines were given to calculate the cell size.");
        return AroResp(1, "No machines were given to calculate the cell size." );;
    }

    double cellsize = std::numeric_limits<double>::max();
    for(auto&m : machines){
        if(m.working_width > 1e-3)
            cellsize = std::min(cellsize, m.working_width);
    }

    if(cellsize > 1e3){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid working width in machines.");
        return AroResp(1, "Invalid working width in machines." );;
    }

    return init(field, cellsize*0.5, removeHeadland);
}

AroResp WorkedAreaManager::init(const Field &field, const ArolibGrid_t &remainingAreaMap)
{
    clear();

    m_remainingAreaMap = remainingAreaMap;

    if(!m_remainingAreaMap.isAllocated()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid map (not allocated).");
        return AroResp(1, "Invalid map (not allocated)." );;
    }

    Polygon boundary_exp;
    if(!arolib::geometry::offsetPolygon(field.outer_boundary, boundary_exp, m_remainingAreaMap.getCellsize(), true)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting the outer boundary.");
        return AroResp(1, "Error offsetting the outer boundary." );;
    }

    if(!m_remainingAreaMap.expandGridFromPolygon(boundary_exp.points, 1.0)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error expanding the grid.");
        return AroResp(1, "Error expanding the grid." );;
    }
    m_field = field;
    m_ready = true;
    return AroResp(0, "OK" );

}

bool WorkedAreaManager::isReady() const
{
    return m_ready;
}

void WorkedAreaManager::clear()
{
    m_ready = false;
    m_field.clear();
    m_remainingAreaMap.destroy();
    m_drivenEdges.clear();
}

AroResp WorkedAreaManager::addPoint(const Machine &machine, const Point &pt, bool be_precise)
{
    if(!m_ready){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Not ready/initialized.");
        return AroResp(1, "Not ready/initialized." );;
    }
    if(machine.working_width < 1e-5){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid machine working width.");
        return AroResp(1, "Invalid machine working width." );;
    }

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

    if( !arolib::geometry::in_polygon(points, m_field.outer_boundary, false) && !arolib::geometry::intersects(points, m_field.outer_boundary) )
        return AroResp(0, "OK" );

    if(!m_remainingAreaMap.addToLine(points.front(),
                                     points.back(),
                                     machine.working_width,
                                     -1,
                                     be_precise,
                                     0,
                                     1)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error updating the map with the driven edge.");
        return AroResp(1, "Error updating the map with the driven edge." );;
    }

    return AroResp(0, "OK" );

}

const arolib::ArolibGrid_t &WorkedAreaManager::getRemainingAreaMap() const
{
    return m_remainingAreaMap;
}

}
