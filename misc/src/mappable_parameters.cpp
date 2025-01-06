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
 
#include "arolib/misc/mappable_parameters.h"

namespace arolib
{

MappableParameters::ParameterBase::ParameterBase(std::string parameter_name)
    : name(parameter_name){

}

bool MappableParameters::parseFromStringMap(const std::map<std::string, std::string> &strMap, bool strict)
{
    auto params = get_parameters();
    for (auto &param : params)
    {
        if (strMap.count(param->name)) // key is in both maps
            param->deserealize(strMap.at(param->name));

        else if (strict) // key missing from input stringmap
            return false;
    }
    return true;
}

std::map<std::string, std::string> MappableParameters::parseToStringMap()
{
    auto params = get_parameters();
    std::map<std::string, std::string> ret;
    for (auto &param : params)
        ret[param->name] = param->serealize();
    return ret;
}


}
