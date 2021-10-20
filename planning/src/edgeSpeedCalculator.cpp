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
 

#include "arolib/planning/edgeSpeedCalculator.hpp"

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


CustomEdgeSpeedCalculator::CustomEdgeSpeedCalculator(const CustomEdgeSpeedCalculator::CalcSpeedFunc &_calcSpeed, const LogLevel &logLevel):
    IEdgeSpeedCalculator(__FUNCTION__, logLevel)
  , m_calcSpeed(_calcSpeed)
{

}

double CustomEdgeSpeedCalculator::calcSpeed(const Point& p0, const Point& p1, double bunker_mass, const Machine& machine)
{
    return m_calcSpeed(p0, p1, bunker_mass, machine);
}

//----------------------------------------------EdgeSpeedCalculatorDef----------------------------------------------------

EdgeSpeedCalculatorDef::EdgeSpeedCalculatorDef(const LogLevel &logLevel):
    IEdgeSpeedCalculator(__FUNCTION__, logLevel)
{
}

double EdgeSpeedCalculatorDef::calcSpeed(const Point &, const Point &, double bunker_mass, const Machine &machine)
{
    if(machine.def_working_speed > 0)
        return machine.def_working_speed;
    return machine.calcSpeed(bunker_mass);
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

