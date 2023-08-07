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
 
#ifndef AROLIB_BASEROUTESPROCESSOR_H
#define AROLIB_BASEROUTESPROCESSOR_H

#include <unistd.h>
#include <iostream>
#include <math.h>
#include <string>

#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/container_helper.h"
#include "arolib/geometry/field_geometry_processing.hpp"

#include "arolib/types/machine.hpp"

#include "arolib/planning/planningworkspace.h"
#include "arolib/planning/edge_calculators/edgeMassCalculator.hpp"
#include "arolib/planning/track_connectors/infieldtracksconnectordef.hpp"
#include "arolib/misc/basic_responses.h"

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>


namespace arolib {

/**
 * @brief Class used to process the initial base routes (prior to planning); It takes the base-routes for IF and HL processing and combines (connects) them  
 */
class BaseRoutesProcessor : public LoggingComponent, protected PlanningWorkspaceAccessor
{
public:

    /**
     * @brief Settings
     */
    struct Settings{
        bool bePreciseWithMaps = true; /**< Perform map/grid operations precisely (except for remaining-area map) */
        double sampleResolution = -1; /**< Sample resulution for generated paths */

        /**
         * @brief Parse the parameters from a string map, starting from a default Settings
         * @param [out] param Settings
         * @param map String map containing the parameter values
         * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the default values will remain)
         * @return True on success
         */
        static bool parseFromStringMap( Settings& params, const std::map<std::string, std::string>& map, bool strict = false);

        /**
         * @brief Parse the parameters to a string map
         * @param param Parameters
         * @return String map containing the parameter values
         */
        static std::map<std::string, std::string> parseToStringMap(const Settings &params);
    };

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit BaseRoutesProcessor(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Takes a base route and reverses it.
     * @param route Base route
     * @return Reversed base route
     */
    static Route reverseRoute(const Route &route);
};

}

#endif // AROLIB_BASEROUTESPROCESSOR_H
