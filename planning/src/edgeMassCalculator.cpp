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
 

#include "arolib/planning/edgeMassCalculator.hpp"

namespace arolib{

const std::string IEdgeMassCalculator::FactorMapName = "MASS_FACTOR";

IEdgeMassCalculator::IEdgeMassCalculator(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_gridsManager(logLevel)
{    
    m_gridsManager.logger().setParent(&m_logger);
}

bool IEdgeMassCalculator::setFactorMap(std::shared_ptr<const ArolibGrid_t> map)
{
    bool ok = true;
    if(!map){
        if( m_gridsManager.hasGrid(FactorMapName) && !m_gridsManager.removeGrid(FactorMapName) ){
            logger().printError(__FUNCTION__, "Error removing map");
            ok = false;
        }
    }
    else{
        if( !m_gridsManager.addGrid(FactorMapName, map, true) ){
            logger().printError(__FUNCTION__, "Error setting map");
            ok = false;
        }
    }
    return ok;
}

void IEdgeMassCalculator::setGridCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim)
{
    m_gridsManager.setCellsInfoManager(cim, true);
}

void IEdgeMassCalculator::setMapsComputationPrecision(gridmap::SharedGridsManager::PreciseCalculationOption mapsCompPrecision)
{
    m_mapsPrecision = mapsCompPrecision;
}

IEdgeMassCalculator::IEdgeMassCalculator(const std::string childName, const LogLevel &logLevel):
    LoggingComponent(logLevel, childName),
    m_gridsManager(logLevel)
{

    m_gridsManager.logger().setParent(&m_logger);
}


//----------------------------------------------CustomEdgeMassCalculator-------------------------------------------------


CustomEdgeMassCalculator::CustomEdgeMassCalculator(const CustomEdgeMassCalculator::CalcMassFunc &_calcMass, const LogLevel &logLevel):
    IEdgeMassCalculator(__FUNCTION__, logLevel)
  , m_calcMass(_calcMass)
{

}

double CustomEdgeMassCalculator::calcMass(const Point &p0, const Point &p1, double width)
{
    return m_calcMass(p0, p1, width);
}


//----------------------------------------------EdgeMassCalculatorDef----------------------------------------------------

EdgeMassCalculatorDef::EdgeMassCalculatorDef(const LogLevel &logLevel):
    IEdgeMassCalculator(__FUNCTION__, logLevel)
{

}

double EdgeMassCalculatorDef::calcMass(const Point &p0, const Point &p1, double width)
{
    double area = arolib::geometry::calc_area(p0, p1, width);

    if(area <= 0)
        return 0;

    double mass = area * t_ha2Kg_sqrm( m_massPerArea );

    auto factorAreaMap = m_gridsManager.getGrid(FactorMapName);
    if(factorAreaMap && factorAreaMap->isAllocated()){
        std::vector<gridmap::GridmapLayout::GridCellOverlap> cellsInfo;
        if( m_gridsManager.getCellsInfoUnderLine(FactorMapName, p0, p1, width, m_mapsPrecision, cellsInfo) ){

            if(cellsInfo.empty())
                return 0;

            bool errorTmp;
            double factor = factorAreaMap->getCellsComputedValue( cellsInfo,
                                                                  ArolibGrid_t::AVERAGE_TOTAL,
                                                                  area,
                                                                  false,
                                                                  &errorTmp );

            if(errorTmp)
                return 0;

            //mass *= std::min(1.0, std::max(0.0, factor));
            mass *= factor;
        }
    }

    return mass;
}

bool EdgeMassCalculatorDef::setParameters(double massPerArea_def)
{
    m_massPerArea = massPerArea_def;
    return true;
}

double EdgeMassCalculatorDef::getMassPerArea() const
{
    return m_massPerArea;
}


//----------------------------------------------EMC_MassGrid----------------------------------------------------

const std::string EMC_MassGrid::MassMapName = "MASS";

EMC_MassGrid::EMC_MassGrid(const LogLevel &logLevel):
    IEdgeMassCalculator(__FUNCTION__, logLevel),
    m_defCalculator(logLevel)
{
    m_defCalculator.logger().setParent(&m_logger);
}

double EMC_MassGrid::calcMass(const Point &p0, const Point &p1, double width)
{

    double mass;

    if( getFromMemory(p0, p1, width, mass) )
        return mass;

    double massPerArea = getMassPerArea(p0, p1, width);

    if( std::fabs(massPerArea) < 1e-9)
        return 0;

    double massPerArea_original = m_defCalculator.getMassPerArea();
    m_defCalculator.setParameters(massPerArea);
    mass = m_defCalculator.calcMass(p0, p1, width);
    m_defCalculator.setParameters(massPerArea_original);

    if(m_memorySize > 0){
        auto it = m_prevValuesMap.insert( std::make_pair(gridmap::GridCellsInfoManager::Edge(p0, p1, width, true), mass) );
        m_prevValues.emplace_back( it.first );
    }

    removeOldValues();
    return mass;

}

bool EMC_MassGrid::setParameters(double massPerArea_def)
{
    return m_defCalculator.setParameters(massPerArea_def);
}

bool EMC_MassGrid::setMassMap(std::shared_ptr<const ArolibGrid_t> map)
{
    bool ok = true;
    if(!map){
        if( m_gridsManager.hasGrid(MassMapName) && !m_gridsManager.removeGrid(MassMapName) ){
            logger().printError(__FUNCTION__, "Error removing map");
            ok = false;
        }
    }
    else{
        if( !m_gridsManager.addGrid(MassMapName, map, true) ){
            logger().printError(__FUNCTION__, "Error setting map");
            ok = false;
        }
    }
    return ok;
}

void EMC_MassGrid::setMemorySize(size_t memSize)
{
    m_memorySize = memSize;
    removeOldValues();
}

bool EMC_MassGrid::setFactorMap(std::shared_ptr<const ArolibGrid_t> map)
{
    return IEdgeMassCalculator::setFactorMap(map)
            && m_defCalculator.setFactorMap(map);
}

void EMC_MassGrid::setGridCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim)
{
    IEdgeMassCalculator::setGridCellsInfoManager(cim);
    m_defCalculator.setGridCellsInfoManager(cim);
}

void EMC_MassGrid::setMapsComputationPrecision(gridmap::SharedGridsManager::PreciseCalculationOption mapsCompPrecision)
{
    IEdgeMassCalculator::setMapsComputationPrecision(mapsCompPrecision);
    m_defCalculator.setMapsComputationPrecision(mapsCompPrecision);
}

Unit EMC_MassGrid::getUnits()
{
    Unit units = Unit::UNIT_CUSTOM;

    auto map = m_gridsManager.getGrid(MassMapName);

    if(map && map->isAllocated())
        units = map->getUnits();

    if(units == Unit::UNIT_CUSTOM){
        units = Unit::UNIT_TONS_PER_HECTARE;
        //m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Mass map units are set to UNIT_CUSTOM. Seting working units to UNIT_TONS_PER_HECTARE");
    }
    else if( !isMassUnit(units) && !isMassPerAreaUnit(units) ){
        units = Unit::UNIT_TONS_PER_HECTARE;
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid mass map units. Seting working units to UNIT_TONS_PER_HECTARE");
    }
    return units;
}

double EMC_MassGrid::getMassPerArea(const Point &p0, const Point &p1, double width)
{
    double area = arolib::geometry::calc_dist(p0, p1) * width;
    if(area <= 0)
        return 0;

    auto map = m_gridsManager.getGrid(MassMapName);

    if(map && map->isAllocated()){
        bool errorTmp;
        double massPerArea;
        auto units = getUnits();

        if( m_mapsPrecision != gridmap::SharedGridsManager::PRECISE
                && std::min( width, arolib::geometry::calc_dist(p0, p1) ) > 10*map->getCellsize() ){//compute value directly from map
            massPerArea = map->getLineComputedValue(p0,
                                                    p1,
                                                    width,
                                                    false,
                                                    isMassUnit(units) ? ArolibGrid_t::SUM : ArolibGrid_t::AVERAGE_TOTAL,
                                                    &errorTmp);

        }
        else{
            std::vector<gridmap::GridmapLayout::GridCellOverlap> gridCellsInfo;
            m_gridsManager.getCellsInfoUnderLine(MassMapName,
                                                 p0,
                                                 p1,
                                                 width,
                                                 m_mapsPrecision,
                                                 gridCellsInfo);

            if(gridCellsInfo.empty())
                return 0;


            massPerArea = map->getCellsComputedValue( gridCellsInfo,
                                                      isMassUnit(units) ? ArolibGrid_t::SUM : ArolibGrid_t::AVERAGE_TOTAL,
                                                      area,
                                                      false,
                                                      &errorTmp );
        }

        if (!errorTmp){
            if (isMassUnit(units))
                massPerArea /= area;

            try{
                massPerArea = convertUnits(massPerArea, map->getUnits(), Unit::UNIT_TONS_PER_HECTARE);
            }
            catch(std::exception &e){
                if(map->getUnits() != Unit::UNIT_CUSTOM)
                    m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error converting mass/area map units to t/ha : " + std::string( e.what() ) + ". Values read as t/ha." );
            }
            return massPerArea;
        }

        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error obtaining the massPerArea from maps. Using default (average) mass/area." );
    }

    return m_defCalculator.getMassPerArea();
}

bool EMC_MassGrid::getFromMemory(const Point &p0, const Point &p1, double width, double &mass)
{
    auto it = m_prevValuesMap.find( gridmap::GridCellsInfoManager::Edge(p0, p1, width, true) );
    if(it == m_prevValuesMap.end())
        return false;

    for(size_t i = 0 ; i+1 < m_prevValues.size(); ++i){
        if(m_prevValues.at(i) == it){
            m_prevValues.erase( m_prevValues.begin() + i);
            m_prevValues.emplace_back(it);
            break;
        }
    }

    mass = it->second;

    return true;
}

void EMC_MassGrid::removeOldValues()
{
    while(m_prevValues.size() > m_memorySize){
        m_prevValuesMap.erase(m_prevValues.front());
        pop_front(m_prevValues);
    }
}


}

