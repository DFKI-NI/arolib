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
 

#include "arolib/planning/generalplanningparameters.hpp"

namespace arolib{

bool FieldGeneralParameters::parseFromStringMap(FieldGeneralParameters &params, const std::map<std::string, std::string> &map, bool strict)
{
    FieldGeneralParameters tmp;
    std::map<std::string, double*> dMap = { {"avgMassPerArea", &tmp.avgMassPerArea},
                                            {"avgDrymatter", &tmp.avgDrymatter} };
    if( !setValuesFromStringMap(map, dMap, strict) )
        return false;
    params = tmp;
    return true;
}

std::map<std::string, std::string> FieldGeneralParameters::parseToStringMap(const FieldGeneralParameters &params)
{
    std::map<std::string, std::string> ret;
    ret["avgMassPerArea"] = double2string( params.avgMassPerArea );
    ret["avgDrymatter"] = double2string( params.avgDrymatter );
    return ret;
}

bool GridComputationSettings::parseFromStringMap(GridComputationSettings &params, const std::map<std::string, std::string> &map, bool strict)
{
    GridComputationSettings tmp;
    std::map<std::string, bool*> bMap = { {"bePreciseWithRemainingAreaMap", &tmp.bePreciseWithRemainingAreaMap},
                                          {"bePreciseWithMatterMap", &tmp.bePreciseWithMatterMap},
                                          {"bePreciseWithSoilMap", &tmp.bePreciseWithSoilMap} };
    if( !setValuesFromStringMap(map, bMap, strict) )
        return false;
    params = tmp;
    return true;

}

std::map<std::string, std::string> arolib::GridComputationSettings::parseToStringMap(const GridComputationSettings &params)
{
    std::map<std::string, std::string> ret;
    ret["bePreciseWithRemainingAreaMap"] = double2string( params.bePreciseWithRemainingAreaMap );
    ret["bePreciseWithMatterMap"] = double2string( params.bePreciseWithMatterMap );
    ret["bePreciseWithSoilMap"] = double2string( params.bePreciseWithSoilMap );
    return ret;
}



}

