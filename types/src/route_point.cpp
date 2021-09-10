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
 
#include "arolib/types/route_point.hpp"

namespace arolib {

const std::set<RoutePoint::RoutePointType> RoutePoint::WorkingRoutePointTypes = {RoutePoint::HARVESTING,
                                                                                 RoutePoint::SEEDING,
                                                                                 RoutePoint::SPRAYING,
                                                                                 RoutePoint::CULTIVATING,
                                                                                 RoutePoint::PLOUGHING,
                                                                                 RoutePoint::TRACK_START,
                                                                                 RoutePoint::TRACK_END};



RoutePoint::RoutePointType RoutePoint::intToRoutePointType(int value){

    if(value == RoutePointType::DEFAULT)
        return RoutePointType::DEFAULT;
    else if(value == RoutePointType::TRACK_START)
        return RoutePointType::TRACK_START;
    else if(value == RoutePointType::TRACK_END)
        return RoutePointType::TRACK_END;
    else if(value == RoutePointType::RESOURCE_POINT)
        return RoutePointType::RESOURCE_POINT;
    else if(value == RoutePointType::FIELD_ENTRY)
        return RoutePointType::FIELD_ENTRY;
    else if(value == RoutePointType::FIELD_EXIT)
        return RoutePointType::FIELD_EXIT;
    else if(value == RoutePointType::OVERLOADING_START)
        return RoutePointType::OVERLOADING_START;
    else if(value == RoutePointType::OVERLOADING_FINISH)
        return RoutePointType::OVERLOADING_FINISH;
    else if(value == RoutePointType::OVERLOADING)
        return RoutePointType::OVERLOADING;
    else if(value == RoutePointType::HEADLAND)
        return RoutePointType::HEADLAND;
    else if(value == RoutePointType::INITIAL_POSITION)
        return RoutePointType::INITIAL_POSITION;
    else if(value == RoutePointType::HARVESTING)
        return RoutePointType::HARVESTING;
    else if(value == RoutePointType::TRANSIT)
        return RoutePointType::TRANSIT;
    else if(value == RoutePointType::SEEDING)
        return RoutePointType::SEEDING;
    else if(value == RoutePointType::SPRAYING)
        return RoutePointType::SPRAYING;
    else if(value == RoutePointType::CULTIVATING)
        return RoutePointType::CULTIVATING;
    else if(value == RoutePointType::PLOUGHING)
        return RoutePointType::PLOUGHING;
    else if(value == RoutePointType::SCANNING)
        return RoutePointType::SCANNING;
    else if(value == RoutePointType::TRANSIT_OF)
        return RoutePointType::TRANSIT_OF;

    throw std::invalid_argument( "The given value does not correspond to any RoutePoint::RoutePointType" );

}

bool RoutePoint::MachineRelationInfo::operator !=(const RoutePoint::MachineRelationInfo &other) const
{
    return !( operator ==(other) );
}

bool RoutePoint::MachineRelationInfo::operator ==(const RoutePoint::MachineRelationInfo &other) const
{
    return machine_id == other.machine_id
            && route_id == other.route_id
            && routePointType == other.routePointType
            && routePointIndex == other.routePointIndex ;
}

std::size_t RoutePoint::KeyHash::operator()(const RoutePoint &rp) const
{
    return get(rp, 0);
}

std::size_t RoutePoint::KeyHash::get(const RoutePoint &rp, std::size_t seed)
{
    seed = Point::KeyHash::get( rp.point(), seed );
    boost::hash_combine(seed, (int)rp.type);
    boost::hash_combine(seed, rp.track_id);
    boost::hash_combine(seed, rp.track_idx);
    boost::hash_combine(seed, rp.time_stamp);
    return seed;
}

bool RoutePoint::operator <(const RoutePoint &rp) const
{
    if (track_id != rp.track_id)
        return track_id < rp.track_id;
    if (track_idx != rp.track_idx)
        return track_idx < rp.track_idx;
    if (type != rp.type)
        return type < rp.type;
    return this->point() < rp.point();
}

bool RoutePoint::operator ==(const RoutePoint &other) const
{
    return(   point() == other.point()
           && type == other.type
           && track_id == other.track_id
           && std::fabs(time_stamp - other.time_stamp) < 1e-3
           && std::fabs(bunker_mass - other.bunker_mass) < 1e-3
           && std::fabs(bunker_volume - other.bunker_volume) < 1e-3
           && std::fabs(harvested_mass - other.harvested_mass) < 1e-3
           && std::fabs(harvested_volume - other.harvested_volume) < 1e-3
              && machineRelations == other.machineRelations );
}

bool RoutePoint::operator !=(const RoutePoint &other) const
{
    return !(operator ==(other));
}

RoutePoint::RoutePointType RoutePoint::getDefaultRPType(const Machine &machine)
{
    if(machine.machinetype == Machine::HARVESTER)
        return RoutePoint::HARVESTING;
    if(machine.machinetype == Machine::SOWER)
        return RoutePoint::SEEDING;
    if(machine.machinetype == Machine::SPRAYER)
        return RoutePoint::SPRAYING;
    if(machine.machinetype == Machine::CULTIVATOR)
        return RoutePoint::CULTIVATING;
    if(machine.machinetype == Machine::PLOUGH)
        return RoutePoint::PLOUGHING;
    if(machine.machinetype == Machine::SCANNER)
        return RoutePoint::SCANNING;
    if(machine.machinetype == Machine::OLV)
        return RoutePoint::OVERLOADING;
    return RoutePoint::DEFAULT;
}

std::string RoutePoint::RPTypeToString(RoutePoint::RoutePointType type)
{
    if(type == RoutePoint::DEFAULT)
        return "DEFAULT";
    if(type == RoutePoint::TRACK_START)
        return "TRACK_START";
    if(type == RoutePoint::TRACK_END)
        return "TRACK_END";
    if(type == RoutePoint::RESOURCE_POINT)
        return "RESOURCE_POINT";
    if(type == RoutePoint::FIELD_ENTRY)
        return "FIELD_ENTRY";
    if(type == RoutePoint::FIELD_EXIT)
        return "FIELD_EXIT";
    if(type == RoutePoint::OVERLOADING_START)
        return "OVERLOADING_START";
    if(type == RoutePoint::OVERLOADING_FINISH)
        return "OVERLOADING_FINISH";
    if(type == RoutePoint::OVERLOADING)
        return "OVERLOADING";
    if(type == RoutePoint::HEADLAND)
        return "HEADLAND";
    if(type == RoutePoint::INITIAL_POSITION)
        return "INITIAL_POSITION";
    if(type == RoutePoint::HARVESTING)
        return "HARVESTING";
    if(type == RoutePoint::TRANSIT)
        return "TRANSIT";
    if(type == RoutePoint::SEEDING)
        return "SEEDING";
    if(type == RoutePoint::SPRAYING)
        return "SPRAYING";
    if(type == RoutePoint::CULTIVATING)
        return "CULTIVATING";
    if(type == RoutePoint::PLOUGHING)
        return "PLOUGHING";
    if(type == RoutePoint::SCANNING)
        return "SCANNING";
    if(type == RoutePoint::TRANSIT_OF)
        return "TRANSIT_OF";
    return "---";
}

std::string RoutePoint::RPTypeToShortString2c(RoutePoint::RoutePointType type)
{
    if(type == RoutePoint::DEFAULT)
        return "DF";
    if(type == RoutePoint::TRACK_START)
        return "TS";
    if(type == RoutePoint::TRACK_END)
        return "TE";
    if(type == RoutePoint::RESOURCE_POINT)
        return "RP";
    if(type == RoutePoint::FIELD_ENTRY)
        return "FI";
    if(type == RoutePoint::FIELD_EXIT)
        return "FO";
    if(type == RoutePoint::OVERLOADING_START)
        return "OS";
    if(type == RoutePoint::OVERLOADING_FINISH)
        return "OF";
    if(type == RoutePoint::OVERLOADING)
        return "OL";
    if(type == RoutePoint::HEADLAND)
        return "HL";
    if(type == RoutePoint::INITIAL_POSITION)
        return "IP";
    if(type == RoutePoint::HARVESTING)
        return "HV";
    if(type == RoutePoint::TRANSIT)
        return "TR";
    if(type == RoutePoint::SEEDING)
        return "SD";
    if(type == RoutePoint::SPRAYING)
        return "SP";
    if(type == RoutePoint::CULTIVATING)
        return "CL";
    if(type == RoutePoint::PLOUGHING)
        return "PL";
    if(type == RoutePoint::SCANNING)
        return "SC";
    if(type == RoutePoint::TRANSIT_OF)
        return "TO";
    return "--";
}

size_t RoutePoint::getNextIndByType(const std::vector<RoutePoint> &route_points, const std::set<RoutePointType> &types, size_t ind0, int indn)
{
    if(indn < 0)
        indn = route_points.size()-1;

    for(size_t i = ind0; i < route_points.size() && i <= indn ; ++i){
        if( types.find( route_points.at(i).type ) != types.end() )
            return i;
    }
    return route_points.size();
}

void RoutePoint::copyBasicWorkingValuesFrom(const RoutePoint &from)
{
    harvested_mass = from.harvested_mass;
    harvested_volume = from.harvested_volume;
    bunker_mass = from.bunker_mass;
    bunker_volume = from.bunker_volume;
}


}
