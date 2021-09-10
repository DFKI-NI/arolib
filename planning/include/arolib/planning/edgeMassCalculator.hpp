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
 
#ifndef ARO_EDGEMASSCALCULATOR_HPP
#define ARO_EDGEMASSCALCULATOR_HPP

#include <functional>

#include "aro_functions.hpp"
#include "arolib/planning/planningworkspace.h"
#include "arolib/types/point.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/cartography/sharedgridsmanager.hpp"

namespace arolib{

/**
 * @brief Interface class used to compute the matter mass in an edge/segment
 */
class IEdgeMassCalculator : public virtual LoggingComponent
{
public:

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit IEdgeMassCalculator(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Compute the edge mass.
     * @param p0 First point of the edge/segment.
     * @param p1 Second point of the edge/segment.
     * @param width Width of the edge/segment.
     * @return Mass [Kg].
     */
    virtual double calcMass (const Point& p0, const Point& p1, double width) = 0;

    /**
     * @brief Set the map used to factor the mass values. If map values are 0 and 1, it would correspond to a valid-area map.
     * @param map Map.
     * @return True on success
     */
    virtual bool setFactorMap(std::shared_ptr<const ArolibGrid_t> map);

    /**
     * @brief Set shared CellsInfoManager to record shared edge cells data
     * @param cim CellsInfoManager. If null, no recording will be done
     */
    virtual void setGridCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim);

    /**
     * @brief Set the precision in which maps' computations will be done
     * @param cim CellsInfoManager. If null, no recording will be done
     */
    virtual void setMapsComputationPrecision(gridmap::SharedGridsManager::PreciseCalculationOption mapsCompPrecision = gridmap::SharedGridsManager::PRECISE);

    /**
     * @brief Check if a calculator if of a given type T.
     * @return True if the same type.
     */
    template<typename T,
             typename = typename std::enable_if< std::is_base_of<IEdgeMassCalculator, T>::value >::type>
    bool isOfType() const {
        return typeid(*this) == typeid(T);
    }

protected:
    /**
     * @brief Constructor.
     * @param logLevel Log level
     * @param childName Child class
     */
    explicit IEdgeMassCalculator(const std::string childName, const LogLevel &logLevel = LogLevel::INFO);

protected:
    gridmap::SharedGridsManager m_gridsManager; /**< Maps/grids manager */
    gridmap::SharedGridsManager::PreciseCalculationOption m_mapsPrecision = gridmap::SharedGridsManager::PRECISE; /**< Maps presicion calculation option >*/
    static const std::string FactorMapName;/**< Mass-factor gridmap name >*/
};


//----------------------------------------------CustomEdgeMassCalculator----------------------------------------------------

/**
 * @brief Default class used to compute the matter mass in an edge/segment
 */
class CustomEdgeMassCalculator : public IEdgeMassCalculator
{
public:

    using CalcMassFunc = std::function< double ( const Point&, const Point&, double ) >;

    /**
     * @brief Constructor.
     * @param _calcMass Function to be called when IEdgeMassCalculator::calcMass is called
     * @param logLevel Log level
     */
    explicit CustomEdgeMassCalculator(const CalcMassFunc& _calcMass, const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Compute the edge mass.
     * @sa ICalcEdgeMassFunction::calcMass
     */
    virtual double calcMass (const Point& p0, const Point& p1, double width) override;

protected:
    CalcMassFunc m_calcMass;/**< Function to be called when IEdgeMassCalculator::calcMass is called >*/

};



//----------------------------------------------EdgeMassCalculatorDef----------------------------------------------------

/**
 * @brief Default class used to compute the matter mass in an edge/segment
 */
class EdgeMassCalculatorDef : public IEdgeMassCalculator
{
public:

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit EdgeMassCalculatorDef(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Compute the edge mass.
     * @sa ICalcEdgeMassFunction::calcMass
     */
    virtual double calcMass (const Point& p0, const Point& p1, double width) override;

    /**
     * @brief Set the calculator parameters.
     * @param massPerArea_def Default (average) mass per area [t/ha]
     */
    virtual bool setParameters(double massPerArea_def);

    /**
     * @brief get the default (average) mass per area [t/ha]
     */
    virtual double getMassPerArea() const;

protected:
    double m_massPerArea = defYieldmassPerArea_siloMaize__t_ha;/**< Default (average) mass per area [t/ha] */
};


//----------------------------------------------EMC_MassGrid----------------------------------------------------

/**
 * @brief Class used to compute the mass from a mass grid
 */
class EMC_MassGrid : public IEdgeMassCalculator
{
public:

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit EMC_MassGrid(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Compute the edge mass.
     * @sa ICalcEdgeMassFunction::calcMass
     */
    virtual double calcMass (const Point& p0, const Point& p1, double width) override;

    /**
     * @brief Set the calculator parameters (map based).
     * @param massPerArea_def Default (average) mass per area [t/ha]
     * @return True on success
     */
    virtual bool setParameters(double massPerArea_def);

    /**
     * @brief Set the field mass map
     * @param map Mass map (either mass map or mass_per_area map)
     * @return True on success
     */
    virtual bool setMassMap(std::shared_ptr<const ArolibGrid_t> map);

    /**
     * @brief Set the amount of previous values that will be saved.
     * @param memSize pMemory size
     */
    virtual void setMemorySize(size_t memSize);



    /**
     * @brief Set the map used to factor the mass values. If map values are 0 and 1, it would correspond to a valid-area map.
     * @param map Map.
     * @return True on success
     */
    virtual bool setFactorMap(std::shared_ptr<const ArolibGrid_t> map) override;

    /**
     * @brief Set shared CellsInfoManager to record shared edge cells data
     * @param cim CellsInfoManager. If null, no recording will be done
     */
    virtual void setGridCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim) override;

    /**
     * @brief Set the precision in which maps' computations will be done
     * @param cim CellsInfoManager. If null, no recording will be done
     */
    virtual void setMapsComputationPrecision(gridmap::SharedGridsManager::PreciseCalculationOption mapsCompPrecision = gridmap::SharedGridsManager::PRECISE) override;

protected:
    /**
     * @brief Get the units to be used.
     * @return Units to be used.
     */
    virtual Unit getUnits();

    /**
     * @brief Get the mass per area under the edge/segment [t/ha].
     * @param p0 First point of the edge/segment.
     * @param p1 Second point of the edge/segment.
     * @param width Width of the edge/segment.
     * @return Mass per area [t/ha].
     */
    virtual double getMassPerArea(const Point& p0, const Point& p1, double width);

    /**
     * @brief Get the mass from the saved values.
     *
     * Internally manages the memory containers if value is found in memory (not necessry to save the value again)
     * @param p0 First point of the edge/segment.
     * @param p1 Second point of the edge/segment.
     * @param width Width of the edge/segment.
     * @param mass [out] Saved mass.
     * @return true if the value for that edge was found in memory.
     */
    virtual bool getFromMemory (const Point& p0, const Point& p1, double width, double& mass);

    /**
     * @brief Remove old values.
     */
    virtual void removeOldValues();

protected:

    double m_massPerArea = defYieldmassPerArea_siloMaize__t_ha; /**< Default (average) mass per area [t/ha] */
    EdgeMassCalculatorDef m_defCalculator; /**< Default calculator */

    size_t m_memorySize = 5; /**< Memory size */

    std::map<gridmap::GridCellsInfoManager::Edge, double> m_prevValuesMap; /**< Map containing previous edge values */
    std::vector< std::map<gridmap::GridCellsInfoManager::Edge, double>::iterator > m_prevValues; /**< Previous edge values in memory */

    static const std::string MassMapName;/**< Mass-gridmap name >*/
};

}

#endif // ARO_EDGEMASSCALCULATOR_HPP
