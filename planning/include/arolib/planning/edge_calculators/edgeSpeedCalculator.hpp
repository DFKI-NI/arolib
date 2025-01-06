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
 
#ifndef ARO_EDGESPEEDCALCULATOR_HPP
#define ARO_EDGESPEEDCALCULATOR_HPP

#include <functional>

#include "arolib/misc/loggingcomponent.h"
#include "arolib/planning/edge_calculators/edgeMassCalculator.hpp"


namespace arolib{

/**
 * @brief Interface class used to compute the machine speed in an edge/segment
 */
class IEdgeSpeedCalculator : public virtual LoggingComponent
{
public:

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit IEdgeSpeedCalculator(const LogLevel &logLevel = LogLevel::INFO);

    /**
     * @brief Compute the edge speed.
     * @param p0 First point of the edge/segment.
     * @param p1 Second point of the edge/segment.
     * @param bunker_mass Mass in the bunker [Kg].
     * @param machine Machine driving over the edge/segment.
     * @return Speed [m/s].
     */
    virtual double calcSpeed (const Point& p0, const Point& p1, double bunker_mass, const Machine& machine) = 0;

    /**
     * @brief Compute the edge speed (route point based).
     * @param p0 First route point of the edge/segment.
     * @param p1 Second route point of the edge/segment.
     * @param machine Machine driving over the edge/segment.
     * @return Speed [m/s].
     */
    virtual double calcSpeed (const RoutePoint& p0, const RoutePoint& p1, const Machine& machine);

    /**
     * @brief Compute the edge speed for a given segment based on the previous/next points.
     * @param points Points.
     * @param ind_p0 Index corresponding to the first point of the segment to analyze (the second point corresponds to index ind_p0+1).
     * @param bunker_mass Mass in the bunker [Kg] for the given segment.
     * @param machine Machine driving over the edge/segment.
     * @return Speed [m/s].
     */
    virtual double calcSpeed (const std::vector<Point>& points, size_t ind_p0, double bunker_mass, const Machine& machine);

    /**
     * @brief Compute the edge speed for a given segment based on the previous/next route points.
     * @param points Route points.
     * @param ind_p0 Index corresponding to the first point of the segment to analyze (the second point corresponds to index ind_p0+1).
     * @param bunker_mass Mass in the bunker [Kg] for the given segment.
     * @param machine Machine driving over the edge/segment.
     * @return Speed [m/s].
     */
    virtual double calcSpeed (const std::vector<RoutePoint>& points, size_t ind_p0, const Machine& machine);

    /**
     * @brief Compute the turning time.
     * @param angle Angle [Rad]
     * @param bunker_mass Mass in the bunker [Kg].
     * @param machine Machine driving over the edge/segment.
     * @return Turning time [s].
     */
    virtual double calcTurningTime (double angle, double bunker_mass, const Machine& machine) = 0;

    /**
     * @brief Check if the calculations depend on the machine.
     * @return True if the calculations depend on the machine.
     */
    virtual bool dependsOnMachine() const = 0;

    /**
     * @brief Check if the calculations depend on the load.
     * @return True if the calculations depend on the load.
     */
    virtual bool dependsOnLoad() const = 0;

    /**
     * @brief Check if a calculator if of a given type T.
     * @return True if the same type.
     */
    template<typename T,
             typename = typename std::enable_if< std::is_base_of<IEdgeSpeedCalculator, T>::value >::type>
    bool isOfType() const {
        return typeid(*this) == typeid(T);
    }

protected:

    /**
     * @brief Constructor.
     * @param logLevel Log level
     * @param childName Child class
     */
    explicit IEdgeSpeedCalculator(const std::string &childName, const LogLevel &logLevel = LogLevel::INFO);
};


//----------------------------------------------CustomEdgeSpeedCalculator----------------------------------------------------

/**
 * @brief Default class used to compute the machine speed in an edge/segment
 */
class CustomEdgeSpeedCalculator : public IEdgeSpeedCalculator
{
public:
    using CalcSpeedFunc = std::function< double  ( const Point&, const Point&, double, const Machine& ) >;
    using CalcTurningTimeFunc = std::function< double  ( double, double, const Machine& ) >;

    /**
     * @brief Constructor.
     * @param _calcSpeed Function to be called when IEdgeMassCalculator::calcSpeed is called
     * @param _calcTurningTime Function to be called when IEdgeMassCalculator::calcTurningTime is called
     * @param dependsOnMachine True if the calculations depend on the machine
     * @param dependsOnLoad True if the calculations depend on the load
     * @param logLevel Log level
     */
    explicit CustomEdgeSpeedCalculator(const CalcSpeedFunc& _calcSpeed,
                                       const CalcTurningTimeFunc& _calcTurningTime,
                                       bool dependsOnMachine,
                                       bool dependsOnLoad,
                                       const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Compute the edge speed.
     * @sa ICalcEdgeSpeedFunction::calcSpeed
     */
    virtual double calcSpeed (const Point& p0, const Point& p1, double bunker_mass, const Machine& machine) override;

    /**
     * @brief Compute the turning time.
     * @sa ICalcEdgeSpeedFunction::calcTurningTime
     */
    virtual double calcTurningTime (double angle, double bunker_mass, const Machine& machine) override;

    /**
     * @brief Check if the calculations depend on the machine.
     * @return True if the calculations depend on the machine.
     */
    virtual bool dependsOnMachine() const override { return m_dependsOnMachine; }

    /**
     * @brief Check if the calculations depend on the load.
     * @return True if the calculations depend on the load.
     */
    virtual bool dependsOnLoad() const override { return m_dependsOnLoad; }

protected:
    CalcSpeedFunc m_calcSpeed; /**< Function to be called when IEdgeMassCalculator::calcSpeed is called >*/
    CalcTurningTimeFunc m_calcTurningTime; /**< Function to be called when IEdgeMassCalculator::calcTurningTime is called >*/
    bool m_dependsOnMachine; /**< True if the calculations depend on the machine >*/
    bool m_dependsOnLoad; /**< True if the calculations depend on the load >*/

};

//----------------------------------------------EdgeWorkingSpeedCalculatorDef----------------------------------------------------

/**
 * @brief Default class used to compute the machine working speed in an edge/segment
 */
class EdgeWorkingSpeedCalculatorDef : public IEdgeSpeedCalculator
{
public:

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit EdgeWorkingSpeedCalculatorDef(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Compute the edge speed.
     * @sa ICalcEdgeSpeedFunction::calcSpeed
     */
    virtual double calcSpeed (const Point&, const Point&, double bunker_mass, const Machine& machine) override;

    /**
     * @brief Compute the turning time.
     * @sa ICalcEdgeSpeedFunction::calcTurningTime
     */
    virtual double calcTurningTime (double angle, double bunker_mass, const Machine& machine) override;

    /**
     * @brief Check if the calculations depend on the machine.
     * @return True if the calculations depend on the machine.
     */
    virtual bool dependsOnMachine() const override { return true; }

    /**
     * @brief Check if the calculations depend on the load.
     * @return True if the calculations depend on the load.
     */
    virtual bool dependsOnLoad() const override { return true; }
};

//----------------------------------------------EdgeTransitSpeedCalculatorDef----------------------------------------------------

/**
 * @brief Default class used to compute the machine transit speed in an edge/segment
 */
class EdgeTransitSpeedCalculatorDef : public IEdgeSpeedCalculator
{
public:

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit EdgeTransitSpeedCalculatorDef(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Compute the edge speed.
     * @sa ICalcEdgeSpeedFunction::calcSpeed
     */
    virtual double calcSpeed (const Point&, const Point&, double bunker_mass, const Machine& machine) override;

    /**
     * @brief Compute the turning time.
     * @sa ICalcEdgeSpeedFunction::calcTurningTime
     */
    virtual double calcTurningTime (double angle, double bunker_mass, const Machine& machine) override;

    /**
     * @brief Check if the calculations depend on the machine.
     * @return True if the calculations depend on the machine.
     */
    virtual bool dependsOnMachine() const override { return true; }

    /**
     * @brief Check if the calculations depend on the load.
     * @return True if the calculations depend on the load.
     */
    virtual bool dependsOnLoad() const override { return true; }
};



//----------------------------------------------ESCMassBased_custom----------------------------------------------------

/**
 * @brief Class used to compute the harvester speed using a simplified equation similar to the one used in the prospective.HARVEST project
 */
class ESCMassBased_custom : public IEdgeSpeedCalculator
{
public:
    using CalcSpeedFunc = std::function< double  ( const Machine&, double ) >; /**< Input parameters: Machine, yield mass / area [t/ha] ; return: speed [m/s] >*/

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit ESCMassBased_custom(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Compute the edge speed using the internal calculator parameters
     * @sa ICalcEdgeSpeedFunction::calcSpeed
     */
    virtual double calcSpeed (const Point& p0, const Point& p1, double, const Machine& machine) override;

    /**
     * @brief Compute the turning time.
     * @sa ICalcEdgeSpeedFunction::calcTurningTime
     */
    virtual double calcTurningTime (double angle, double bunker_mass, const Machine& machine) override;

    /**
     * @brief Set edge mass calculator used to obtain the amount of mass under the edge/rectangle
     * @param edgeMassCalculator Edge mass calculator
     */
    virtual bool setEdgeMassCalculator(std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator);

    /**
     * @brief Set the function to be called to obtain the speed based on the machine and the yield mass / area [t/ha]
     * @param calcSpeed Speed function
     */
    virtual void setMassBasedSpeedFunction(const CalcSpeedFunc& calcSpeed);

    /**
     * @brief Check if the calculations depend on the machine.
     * @return True if the calculations depend on the machine.
     */
    virtual bool dependsOnMachine() const override { return true; }

    /**
     * @brief Check if the calculations depend on the load.
     * @return True if the calculations depend on the load.
     */
    virtual bool dependsOnLoad() const override { return true; }

protected:
    CalcSpeedFunc DefCalcSpeedFunc = [](const Machine& m, double)->double{
        return m.def_working_speed;
    }; /**< Default calc-speed function >*/


    std::shared_ptr<IEdgeMassCalculator> m_edgeMassCalculator; /**< Mass calculator used to obtain the amount of mass under the edge/rectangle >*/
    CalcSpeedFunc m_calcSpeed = DefCalcSpeedFunc;/**< Function called to obtain the speed based on the machine and the yield mass / area [t/ha] >*/
};

}

#endif // ARO_EDGESPEEDCALCULATOR_HPP
