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
 

#include "arolib/planning/edge_calculators/edgeSpeedCalculator.hpp"

namespace arolib{

IEdgeSpeedCalculator::IEdgeSpeedCalculator(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{

}

double IEdgeSpeedCalculator::calcSpeed(const RoutePoint &p0, const RoutePoint &p1, const Machine &machine)
{
    return calcSpeed(p0.point(), p1.point(), 0.5 * (p0.bunker_mass + p1.bunker_mass) , machine);
}

double IEdgeSpeedCalculator::calcSpeed(const std::vector<Point> &points, size_t ind_p0, double bunker_mass, const Machine &machine)
{
    if( ind_p0+1 >= points.size() )
        return 0;
    return calcSpeed( points[ind_p0], points[ind_p0+1], bunker_mass, machine );
}

double IEdgeSpeedCalculator::calcSpeed(const std::vector<RoutePoint> &points, size_t ind_p0, const Machine &machine)
{
    if( ind_p0+1 >= points.size() )
        return 0;
    return calcSpeed( points[ind_p0], points[ind_p0+1], machine );
}

IEdgeSpeedCalculator::IEdgeSpeedCalculator(const std::string &childName, const LogLevel &logLevel):
    LoggingComponent(logLevel, childName)
{

}

//----------------------------------------------CustomEdgeSpeedCalculator----------------------------------------------------


CustomEdgeSpeedCalculator::CustomEdgeSpeedCalculator(const CustomEdgeSpeedCalculator::CalcSpeedFunc &_calcSpeed, const CalcTurningTimeFunc &_calcTurningTime, const LogLevel &logLevel):
    IEdgeSpeedCalculator(__FUNCTION__, logLevel)
  , m_calcSpeed(_calcSpeed)
  , m_calcTurningTime(_calcTurningTime)
{

}

double CustomEdgeSpeedCalculator::calcSpeed(const Point& p0, const Point& p1, double bunker_mass, const Machine& machine)
{
    return m_calcSpeed(p0, p1, bunker_mass, machine);
}

double CustomEdgeSpeedCalculator::calcTurningTime(double angle, double bunker_mass, const Machine &machine)
{
    return m_calcTurningTime(angle, bunker_mass, machine);
}

//----------------------------------------------EdgeWorkingSpeedCalculatorDef----------------------------------------------------

EdgeWorkingSpeedCalculatorDef::EdgeWorkingSpeedCalculatorDef(const LogLevel &logLevel):
    IEdgeSpeedCalculator(__FUNCTION__, logLevel)
{
}

double EdgeWorkingSpeedCalculatorDef::calcSpeed(const Point &, const Point &, double bunker_mass, const Machine &machine)
{
    if(machine.def_working_speed > 0)
        return machine.def_working_speed;
    return machine.calcSpeed(bunker_mass);
}

double EdgeWorkingSpeedCalculatorDef::calcTurningTime(double angle, double bunker_mass, const Machine &machine)
{
    // @todo: improve turning time computations

    const double speedFactor = 0.3;

    angle = std::fabs(angle);
    angle = std::fmod(angle, 2*M_PI);

    double turningRad = machine.getTurningRadius();
    double speed = speedFactor * machine.calcSpeed(bunker_mass);

    if(speed <= 0)
        return 0;

    if(turningRad < 1e-9){//special case: turn in place
        angle = std::fabs(M_PI-angle);
        return (angle / M_PI) * 20; //@todo the time to rotate 180° should be obtained from a machine parameter (not existing yet); for now, 20 s
    }
    if(angle > M_PI)
        angle = 2*M_PI - angle;
    double dRef = 0.5 * turningRad + std::max(0.0, 1 - 0.2*turningRad);
    double angTH = 2 * std::atan2(turningRad, dRef);

    if(angle >= angTH)
        return 0;

    double deltaAng = angTH - angle;
    double perimeter = 2 * M_PI * turningRad;
    double dist = deltaAng * perimeter / angTH;

    return dist / speed;

}

//----------------------------------------------EdgeTransitSpeedCalculatorDef----------------------------------------------------

EdgeTransitSpeedCalculatorDef::EdgeTransitSpeedCalculatorDef(const LogLevel &logLevel):
    IEdgeSpeedCalculator(__FUNCTION__, logLevel)
{
}

double EdgeTransitSpeedCalculatorDef::calcSpeed(const Point &, const Point &, double bunker_mass, const Machine &machine)
{
    return machine.calcSpeed(bunker_mass);
}

double EdgeTransitSpeedCalculatorDef::calcTurningTime(double angle, double bunker_mass, const Machine &machine)
{
    return EdgeWorkingSpeedCalculatorDef(logger().logLevel()).calcTurningTime(angle, bunker_mass, machine);
}


//----------------------------------------------ESCMassBased_custom----------------------------------------------------

ESCMassBased_custom::ESCMassBased_custom(const LogLevel &logLevel):
    IEdgeSpeedCalculator(__FUNCTION__, logLevel),
    m_edgeMassCalculator( nullptr )
{
    m_edgeMassCalculator = std::make_shared<EdgeMassCalculatorDef>(logLevel);
}

double ESCMassBased_custom::calcSpeed(const Point &p0, const Point &p1, double, const Machine &machine)
{
    double width = machine.working_width > 0 ? machine.working_width : machine.width;
    double area = arolib::geometry::calc_dist(p0, p1) * width;
    double mass = m_edgeMassCalculator->calcMass(p0, p1, width); //in Kg
    double massPerArea = mass/area; //in Kg/m²
    massPerArea = Kg_sqrm2t_ha(massPerArea);
    return m_calcSpeed(machine, massPerArea);
}

double ESCMassBased_custom::calcTurningTime(double angle, double bunker_mass, const Machine &machine)
{
    return EdgeWorkingSpeedCalculatorDef(logger().logLevel()).calcTurningTime(angle, bunker_mass, machine);
}

bool ESCMassBased_custom::setEdgeMassCalculator(std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator)
{
    if(!edgeMassCalculator)
        return false;
    m_edgeMassCalculator = edgeMassCalculator;
    return true;
}

void ESCMassBased_custom::setMassBasedSpeedFunction(const ESCMassBased_custom::CalcSpeedFunc &calcSpeed)
{
    m_calcSpeed = calcSpeed;
}



}

