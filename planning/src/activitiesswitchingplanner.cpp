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
 
#include "arolib/planning/activitiesswitchingplanner.hpp"

namespace arolib{

bool ASP_GeneralSettings::parseFromStringMap(ASP_GeneralSettings &params, const std::map<std::string, std::string> &map, bool strict)
{
    ASP_GeneralSettings tmp;
    std::map<std::string, bool*> bMap = { {"switchOnlyAtTrackEnd" , &tmp.switchOnlyAtTrackEnd},
                                          {"switchOnlyAtTrackEndHL" , &tmp.switchOnlyAtTrackEndHL} };

    if( !setValuesFromStringMap( map, bMap, strict) )
        return false;

    params = tmp;
    return true;
}

std::map<std::string, std::string> ASP_GeneralSettings::parseToStringMap(const ASP_GeneralSettings &params)
{
    std::map<std::string, std::string> ret;
    ret["switchOnlyAtTrackEnd"] = std::to_string( params.switchOnlyAtTrackEnd );
    ret["switchOnlyAtTrackEndHL"] = std::to_string( params.switchOnlyAtTrackEndHL );
    return ret;
}

}
