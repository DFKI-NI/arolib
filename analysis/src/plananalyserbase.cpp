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
 
#include "arolib/analysis/plananalyserbase.hpp"
namespace arolib {

using namespace arolib::geometry;

PlanAnalyserBase::IndexRange::IndexRange(int _min, int _max) : min(_min), max(_max)
{

}

PlanAnalyserBase::PlanAnalyserBase(const LogLevel &logLevel, const std::string &child):
    LoggingComponent(logLevel,
                     child.empty() ? __FUNCTION__ : child)
{

}

bool PlanAnalyserBase::setData(const Field &field)
{
    m_fieldReady = false;
    m_field = field;

    if(!arolib::geometry::closePolygon(m_field.outer_boundary) || m_field.outer_boundary.points.size() < 4){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid field (outer boundary)");
        return false;
    }
    if(m_field.subfields.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid field (no subfields)");
        return false;
    }
    for (size_t i = 0 ; i < m_field.subfields.size() ; ++i){
        auto sf = field.subfields.at(i);
        if(!arolib::geometry::closePolygon(sf.boundary_outer) || sf.boundary_outer.points.size() < 4){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfield [" + std::to_string(i) + "] (outer boundary)");
            return false;
        }
        if(!arolib::geometry::closePolygon(sf.boundary_inner) || sf.boundary_inner.points.size() < 4){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfield [" + std::to_string(i) + "] (inner boundary)");
            return false;
        }
    }

    m_fieldReady = true;
    return true;
}

bool PlanAnalyserBase::addRoute(size_t subfieldId, const MachineId_t &machineId, const Route &route1, const Route &route2)
{
    if(!m_fieldReady){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "No valid field has been set");
        return false;
    }
    if(subfieldId >= m_field.subfields.size()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfield id");
        return false;
    }
    if(route1.route_points.empty() && route2.route_points.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid routes: none of them have route points.");
        return false;
    }

    m_routes[subfieldId][machineId] = std::make_pair(route1, route2);

    return true;
}

void PlanAnalyserBase::clearRoutes()
{
    m_routes.clear();
}

void PlanAnalyserBase::clearMachineRoutes(MachineId_t &machineId)
{
    for (auto it_sf = m_routes.begin(); it_sf != m_routes.end() ; ){
        auto it_m = it_sf->second.find(machineId);
        if(it_m != it_sf->second.end())
            it_sf->second.erase(it_m);
        if (it_sf->second.empty())
            it_sf = m_routes.erase(it_sf);
        else
          ++it_m;
    }
}

void PlanAnalyserBase::clearMachineRoutes(size_t subfieldId, MachineId_t &machineId)
{
    auto it_sf = m_routes.find(subfieldId);
    if(it_sf == m_routes.end())
        return;

    auto it_m = it_sf->second.find(machineId);
    if(it_m != it_sf->second.end())
        it_sf->second.erase(it_m);
    if (it_sf->second.empty())
        it_sf = m_routes.erase(it_sf);
}

void PlanAnalyserBase::clearSubfieldRoutes(size_t subfieldId)
{
    auto it_sf = m_routes.find(subfieldId);
    if(it_sf != m_routes.end())
        it_sf = m_routes.erase(it_sf);
}


}
