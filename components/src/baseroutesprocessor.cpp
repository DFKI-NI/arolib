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
 
#include "arolib/components/baseroutesprocessor.h"

namespace arolib {

bool BaseRoutesProcessor::Settings::parseFromStringMap(Settings &params, const std::map<std::string, std::string> &map, bool strict)
{
    BaseRoutesProcessor::Settings tmp;

    std::map<std::string, double*> dMap = { {"sampleResolution" , &tmp.sampleResolution} };
    std::map<std::string, bool*> bMap = { {"bePreciseWithMaps" , &tmp.bePreciseWithMaps} };

    if( !setValuesFromStringMap( map, dMap, strict)
            || !setValuesFromStringMap( map, bMap, strict) )
        return false;

    params = tmp;
    return true;
}

std::map<std::string, std::string> BaseRoutesProcessor::Settings::parseToStringMap(const Settings &params)
{
    std::map<std::string, std::string> ret;

    ret["sampleResolution"] = double2string( params.sampleResolution );
    ret["bePreciseWithMaps"] = std::to_string( params.bePreciseWithMaps );

    return ret;
}

BaseRoutesProcessor::BaseRoutesProcessor(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{

}

Route BaseRoutesProcessor::reverseRoute(const Route &route)
{
    auto routeRev = route;
    auto& route_points = routeRev.route_points;
    for(RoutePoint& rp : route_points){
        rp.time_stamp = route_points.back().time_stamp - rp.time_stamp;
        rp.worked_mass = route_points.back().worked_mass - rp.worked_mass;
        rp.worked_volume = route_points.back().worked_volume - rp.worked_volume;
        if(rp.type == RoutePoint::FIELD_ENTRY)
            rp.type = RoutePoint::FIELD_EXIT;
        else if(rp.type == RoutePoint::FIELD_EXIT)
            rp.type = RoutePoint::FIELD_ENTRY;
        else if(rp.type == RoutePoint::TRACK_START)
            rp.type = RoutePoint::TRACK_END;
        else if(rp.type == RoutePoint::TRACK_END)
            rp.type = RoutePoint::TRACK_START;
        else if(rp.type == RoutePoint::TRACK_START)
            rp.type = RoutePoint::TRACK_END;
    }
    std::reverse(route_points.begin(), route_points.end());
    return routeRev;
}

}
