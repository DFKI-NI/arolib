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
 
#ifndef ARO_GENERALPLANNINGPARAMETERS_HPP
#define ARO_GENERALPLANNINGPARAMETERS_HPP

#include <map>

#include "arolib/misc/basicconversions.hpp"
#include "arolib/planning/aro_functions.hpp"

namespace arolib{

/**
 * @brief Class containing general field parameters
 */
struct FieldGeneralParameters{
    double avgMassPerArea = defYieldmassPerArea_siloMaize__t_ha; /**< Deafault mass proportion (t/ha) */
    double avgDrymatter = defDrymatter_siloMaize; /**< Deafault drymatter */

    /**
     * @brief Parse the parameters from a string map, starting from a default FieldGeneralParameters
     * @param [out] param Parameters
     * @param map String map containing the parameter values
     * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the default values will remain)
     * @return True on success
     */
    static bool parseFromStringMap( FieldGeneralParameters& params, const std::map<std::string, std::string>& map, bool strict = false);

    /**
     * @brief Parse the parameters to a string map
     * @param param Parameters
     * @return String map containing the parameter values
     */
    static std::map<std::string, std::string> parseToStringMap( const FieldGeneralParameters& params);
};

/**
 * @brief Class containing general settings related to grid computations
 */
struct GridComputationSettings{
    bool bePreciseWithRemainingAreaMap = true; /**< Be precise when computing on remaining-area gridmaps */
    bool bePreciseWithMatterMap = false; /**< Be precise when computing on matter gridmaps */
    bool bePreciseWithSoilMap = true; /**< Be precise when computing on soil-cost gridmaps */

    /**
     * @brief Parse the parameters from a string map, starting from a default GridComputationSettings
     * @param [out] param Parameters
     * @param map String map containing the parameter values
     * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the default values will remain)
     * @return True on success
     */
    static bool parseFromStringMap( GridComputationSettings& params, const std::map<std::string, std::string>& map, bool strict = false);

    /**
     * @brief Parse the parameters to a string map
     * @param param Parameters
     * @return String map containing the parameter values
     */
    static std::map<std::string, std::string>  parseToStringMap( const GridComputationSettings& params);
};

}

#endif // ARO_GENERALPLANNINGPARAMETERS_HPP
