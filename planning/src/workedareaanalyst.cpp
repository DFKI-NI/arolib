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
 
#include "arolib/planning/workedareaanalyst.hpp"


namespace{
    std::string WorkedAreaMapName = "WA";
}

namespace arolib{

void WorkedAreaAnalyst::setWorkedAreaMap(std::shared_ptr<const ArolibGrid_t> &gridmap, bool readValueAsWorked)
{
    if(!gridmap || !gridmap->isAllocated()){
        if(m_gridsManager.hasGrid(WorkedAreaMapName))
            m_gridsManager.removeGrid(WorkedAreaMapName);
    }
    else
        m_gridsManager.addGrid(WorkedAreaMapName, gridmap, true);
    m_readAsWorked = readValueAsWorked;
}

std::shared_ptr<const ArolibGrid_t> WorkedAreaAnalyst::getWorkedAreaMap(bool *readValueAsWorked)
{
    if(readValueAsWorked)
        *readValueAsWorked = m_readAsWorked;
    return m_gridsManager.getGrid(WorkedAreaMapName);
}

void WorkedAreaAnalyst::setGridCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim)
{
    m_gridsManager.setCellsInfoManager(cim);
}

bool WorkedAreaAnalyst::setThresholds(float lower, float upper)
{
    if(lower <= 0 || upper >= 1 || lower > upper)
        return false;
    m_thresholdIsWorkedLB = lower;
    m_thresholdIsWorkedUB = upper;
    return true;
}

std::pair<WorkedAreaAnalyst::WorkedState, float> WorkedAreaAnalyst::isSegmentWorked(const Point &p0, const Point &p1, gridmap::SharedGridsManager::PreciseCalculationOption precision)
{
    if(!m_gridsManager.hasGrid(WorkedAreaMapName))
        return std::make_pair(WorkedState::UNKNOWN, std::nanf("1"));


    std::vector<gridmap::GridmapLayout::GridCellOverlap> cellsInfo;
    m_gridsManager.getCellsInfoUnderLine(WorkedAreaMapName, p0, p1, 0,
                                         precision,
                                         cellsInfo);

    size_t count = 0;
    float valueSum;
    auto gridmap = m_gridsManager.getGrid(WorkedAreaMapName);
    bool someValue = false;
    for(auto& ci : cellsInfo){
        if( gridmap->hasValue(ci.x, ci.y) ){
            valueSum += std::max( 0.0f, std::min( 1.0f, gridmap->getValue(ci.x, ci.y) ) );
            someValue = true;
        }
        else
            valueSum += 0.5;
        ++count;
    }
    if(count == 0 || !someValue)
        return std::make_pair(WorkedState::UNKNOWN, std::nanf("1"));

    return correctValueAndGetState(valueSum / count);
}

std::pair<WorkedAreaAnalyst::WorkedState, float> WorkedAreaAnalyst::isSegmentWorked(const Polygon &boundary, const Point &p0, const Point &p1, gridmap::SharedGridsManager::PreciseCalculationOption precision)
{
    if(boundary.points.empty())
        return isSegmentWorked(p0, p1, precision);

    if(geometry::get_intersection({p0, p1}, boundary).empty()){
        if( geometry::in_polygon( geometry::getCentroid(p0, p1), boundary ) )
            return isSegmentWorked(p0, p1, precision);
        return std::make_pair(WorkedState::UNKNOWN, std::nanf("1"));
    }


    auto gridmap = m_gridsManager.getGrid(WorkedAreaMapName);
    size_t count = 0;
    float valueSum;
    bool someValue = false;
    auto cellsInfo = gridmap->getCellsOverlapUnderLine(p0, p1, 0, boundary);
    for(auto& ci : cellsInfo){
        if( gridmap->hasValue(ci.x, ci.y) ){
            valueSum += std::max( 0.0f, std::min( 1.0f, gridmap->getValue(ci.x, ci.y) ) );
            someValue = true;
        }
        else
            valueSum += 0.5;
        ++count;
    }
    if(count == 0 || !someValue)
        return std::make_pair(WorkedState::UNKNOWN, std::nanf("1"));

    return correctValueAndGetState(valueSum / count);

}

std::pair<WorkedAreaAnalyst::WorkedState, float> WorkedAreaAnalyst::isSegmentWorked(const Point &p0, const Point &p1, double width, gridmap::SharedGridsManager::PreciseCalculationOption precision)
{
    if(width < 1e-6)
        return isSegmentWorked(p0, p1, precision);

    if(!m_gridsManager.hasGrid(WorkedAreaMapName))
        return std::make_pair(WorkedState::UNKNOWN, std::nanf("1"));

    std::vector<gridmap::GridmapLayout::GridCellOverlap> cellsInfo;
    m_gridsManager.getCellsInfoUnderLine(WorkedAreaMapName, p0, p1, width,
                                         precision,
                                         cellsInfo);

    bool errorTmp = true;
    float value;

    float area = geometry::calc_dist(p0, p1) * width;

    auto gridmap = m_gridsManager.getGrid(WorkedAreaMapName);
    if(area > 1e-6)
        value = gridmap->getCellsComputedValue(cellsInfo,
                                               ArolibGrid_t::AVERAGE_TOTAL,
                                               area,
                                               false,
                                               &errorTmp);
    else if(gridmap->hasValue(p0))
        value = gridmap->getValue(p0, &errorTmp);

    if(errorTmp)
        return std::make_pair(WorkedState::UNKNOWN, std::nanf("1"));
    return correctValueAndGetState(value);
}

std::pair<WorkedAreaAnalyst::WorkedState, float> WorkedAreaAnalyst::isSegmentWorked(const Polygon &boundary, const Point &p0, const Point &p1, double width, gridmap::SharedGridsManager::PreciseCalculationOption precision)
{
    if(width <= 0 || !m_gridsManager.hasGrid(WorkedAreaMapName))
        return std::make_pair(WorkedState::UNKNOWN, std::nanf("1"));

    if(boundary.points.empty())
        return isSegmentWorked(p0, p1, width, precision);


    Polygon segmentPoly = geometry::createRectangleFromLine( p0, p1, width );

    size_t countPointsInside = 0;
    for(size_t i = 0 ; i+1 < segmentPoly.points.size(); ++i)
        countPointsInside += geometry::in_polygon(segmentPoly.points.at(i), boundary);

    if(countPointsInside > 3 || (countPointsInside == 3 && precision == gridmap::SharedGridsManager::NOT_PRECISE))
        return isSegmentWorked(p0, p1, width, precision);


    std::vector<Polygon> intersectionPolys = geometry::get_intersection(boundary, segmentPoly);

    double areaComplete = geometry::calc_area(p0, p1, width);
    double area = 0;
    std::vector<double> areas(intersectionPolys.size());
    for(size_t i = 0 ; i < intersectionPolys.size() ; ++i){
        areas.at(i) = geometry::calc_area( intersectionPolys.at(i) );
        area += areas.at(i);
    }

    if(areaComplete <= 0 || area / areaComplete < 0.5)
        return std::make_pair(WorkedState::UNKNOWN, std::nanf("1"));


    auto gridmap = m_gridsManager.getGrid(WorkedAreaMapName);
    float value = 0;
    bool valueOK = false;
    for(size_t i = 0 ; i < intersectionPolys.size() ; ++i){
        if(areas.at(i) < 1e-3)
            continue;
        bool errorTmp;

        float valueTmp = gridmap->getPolygonComputedValue(intersectionPolys.at(i),
                                                          ArolibGrid_t::AVERAGE_TOTAL,
                                                          precision == gridmap::SharedGridsManager::PRECISE,
                                                          &errorTmp);
        if(errorTmp)
            continue;
        value += std::max(0.0f, std::min(1.0f, valueTmp) ) * areas.at(i) / area;
        valueOK = true;
    }

    if(!valueOK)
        return std::make_pair(WorkedState::UNKNOWN, std::nanf("1"));
    return correctValueAndGetState(value);
}

std::pair<WorkedAreaAnalyst::WorkedState, float> WorkedAreaAnalyst::isPointWorked(const Point &p)
{
    if(!m_gridsManager.hasGrid(WorkedAreaMapName))
        return std::make_pair(WorkedState::UNKNOWN, std::nanf("1"));

    float value;
    bool errorTmp = true;
    auto gridmap = m_gridsManager.getGrid(WorkedAreaMapName);
    if(gridmap->hasValue(p)){
        value = gridmap->getValue(p, &errorTmp);
        value = std::max( 0.0f, std::min(1.0f, value) );
    }
    if(errorTmp)
        return std::make_pair(WorkedState::UNKNOWN, std::nanf("1"));
    return correctValueAndGetState(value);

}

std::pair<WorkedAreaAnalyst::WorkedState, float> WorkedAreaAnalyst::correctValueAndGetState(float value)
{
    value = std::max( 0.0f, std::min(1.0f, value) );
    if( !m_readAsWorked )
        value = 1 - value;
    if(value > m_thresholdIsWorkedLB && value < m_thresholdIsWorkedUB)
        return std::make_pair(WorkedState::UNKNOWN, value);
    return std::make_pair( (value >= m_thresholdIsWorkedUB ? WorkedState::WORKED : WorkedState::NOT_WORKED) , value);

}


}
