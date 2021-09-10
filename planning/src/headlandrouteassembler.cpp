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
 
#include "arolib/planning/headlandrouteassembler.hpp"


namespace arolib{

HeadlandRouteAssembler::HeadlandRouteAssembler(RoutePoint::RoutePointType workingRPType, const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_workingRPType(workingRPType)
{
    clear();
}

HeadlandRouteAssembler::HeadlandRouteAssembler(const HeadlandRoute &route, RoutePoint::RoutePointType workingRPType, const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_route(route),
    m_workingRPType(workingRPType)
{

}

HeadlandRouteAssembler::HeadlandRouteAssembler(const int &machine_id, const int &route_id, RoutePoint::RoutePointType workingRPType, const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_workingRPType(workingRPType)
{
    m_route.machine_id = machine_id;
    m_route.machine_id = route_id;
    m_route.route_points.clear();
}

void HeadlandRouteAssembler::addTrack(const std::vector<Point> &_points,
                                      int track_id,
                                      bool reverse,
                                      const Machine &machine,
                                      IEdgeMassCalculator &edgeMassCalculator,
                                      IEdgeSpeedCalculator &edgeSpeedCalculator)
{
    double minSpeed = 0.01;//@TODO hardcoded. should we get this as a parameter?

    double maxSpeed = machine.max_speed_empty;
    if(machine.bunker_mass > 1e-5 || machine.bunker_volume > 1e-5)//@TODO tempotaty workaround. This should be computed based on the amount of yield in the bunker, but this is planned afterwards
        maxSpeed = 0.5 * (machine.max_speed_empty + machine.max_speed_full);

    double defSpeed = (machine.def_working_speed > 0 ? machine.def_working_speed : maxSpeed);

    std::vector<Point> points = _points;

    if(reverse)
        std::reverse(points.begin(),points.end());
    for (size_t i = 0; i < points.size(); ++i) {
        RoutePoint rp;
        rp.point() = points.at(i);
        rp.bunker_mass = 0.0;//@TODO should we update the bunker_mass of the harvester routepoints when the harvester has a bunker?
        if (m_route.route_points.size() > 0) {

            double mass = edgeMassCalculator.calcMass(m_route.route_points.back(),
                                                      rp,
                                                      m_workingWidth);

            double calcSpeed = edgeSpeedCalculator.calcSpeed(m_route.route_points.back(),
                                                             rp,
                                                             0,
                                                             machine);

            if (calcSpeed > maxSpeed)
                calcSpeed = maxSpeed;
            if (calcSpeed < minSpeed){
                m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Calculated harvester speed in track " + std::to_string(track_id)
                                   + " (index = " + std::to_string(i) + ") is lower than " + std::to_string(minSpeed) + " m/s");
                calcSpeed = minSpeed;
            }

            if(mass < 1e-5 && m_route.route_points.back().time_stamp < 0)
                rp.time_stamp = -1; //the route points will have a timestamp of -1 until some yield to be harvested is found
            else{
                if(m_route.route_points.back().time_stamp < 0)
                    m_route.route_points.back().time_stamp = 0;

                double segment_length = arolib::geometry::calc_dist(m_route.route_points.back(), rp);
                rp.time_stamp = m_route.route_points.back().time_stamp + segment_length / calcSpeed;
            }

            rp.harvested_mass = m_route.route_points.back().harvested_mass;;
            rp.harvested_volume = m_route.route_points.back().harvested_volume;
            if(i != 0){
                rp.harvested_mass += mass;
                rp.harvested_volume += 0.0;//@TODO
            }
        } else {
            rp.time_stamp = 0.0;
            rp.harvested_mass = 0.0;
            rp.harvested_volume = 0.0;
        }
        rp.type = m_workingRPType;
        if(reverse == false) {
            rp.track_idx = i;
        } else {
            rp.track_idx = points.size() - i;
        }
        if(i == 0/* && track_id > 0*/) {
            rp.type = RoutePoint::TRACK_START;
        } else if(i == points.size() - 1/* && track_id > 0*/) {
            rp.type = RoutePoint::TRACK_END;
        }
        rp.track_id = track_id;
        m_route.route_points.push_back(rp);
    }

}

void HeadlandRouteAssembler::addTrack(double speed,
                                      double default_yield,
                                      const std::vector<Point> &_points,
                                      int track_id,
                                      bool reverse)
{
    std::vector<Point> points = _points;
    if (speed < 0.01) {
        throw std::invalid_argument("Speed must not be < 0.01");
    }
    if(reverse) {
        std::reverse(points.begin(),points.end());
    }

    for (size_t i = 0; i < points.size(); ++i) {
        RoutePoint rp;
        rp.point() = points.at(i);
        rp.bunker_mass = 0.0;//@TODO should we update the bunker_mass of the harvester routepoints when the harvester has a bunker?
        if (m_route.route_points.size() > 0) {
            double segment_length = arolib::geometry::calc_dist(m_route.route_points.back(), rp);
            rp.time_stamp = m_route.route_points.back().time_stamp + segment_length / speed;//@TODO: or should we use the default or (input)

            rp.harvested_mass = m_route.route_points.back().harvested_mass;;
            rp.harvested_volume = m_route.route_points.back().harvested_volume;
            if(i != 0){
                rp.harvested_mass += t_ha2Kg_sqrm(default_yield) * segment_length * m_workingWidth; //now in Kg;
                rp.harvested_volume += 0.0;//@TODO
            }
        } else {
            rp.time_stamp = 0.0;
            rp.harvested_mass = 0.0;
            rp.harvested_volume = 0.0;
        }
        rp.type = m_workingRPType;
        if(reverse == false) {
            rp.track_idx = i;
        } else {
            rp.track_idx = points.size() - i;
        }
        if(i == 0/* && track_id > 0*/) {
            rp.type = RoutePoint::TRACK_START;
        } else if(i == points.size() - 1/* && track_id > 0*/) {
            rp.type = RoutePoint::TRACK_END;
        }
        rp.track_id = track_id;
        m_route.route_points.push_back(rp);
    }

}

void HeadlandRouteAssembler::addTrack(double speed, double default_yield,
                                      const Linestring& l,
                                      int track_id,
                                      bool reverse) {
    return addTrack(speed, default_yield, l.points, track_id, reverse);
}

void HeadlandRouteAssembler::clear()
{
    m_route.machine_id = -1;
    m_route.route_id = -1;
    m_route.route_points.clear();

}

void HeadlandRouteAssembler::setIDs(int machine_id, int route_id)
{
    m_route.machine_id = machine_id;
    m_route.route_id = route_id;
}

bool HeadlandRouteAssembler::setWorkingWidth(double workingWidth)
{
    if(workingWidth < 0)
        return false;
    m_workingWidth = workingWidth;
    return true;
}

}
