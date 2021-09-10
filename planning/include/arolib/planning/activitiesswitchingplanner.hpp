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
 
#ifndef AROLIB_ACTIVITIESSWITCHINGPLANNER_H
#define AROLIB_ACTIVITIESSWITCHINGPLANNER_H

#include <ctime>

#include "arolib/types/route.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/planning/olvPlan.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/logger.h"

namespace arolib{

/**
 * @brief Planner parameters/settings
 */
struct ASP_GeneralSettings{
    bool switchOnlyAtTrackEnd = false; /**< If true, switching between working activities is done only at the end of the tracks */

    /**
     * @brief Parse the parameters from a string map, starting from a default ASP_GeneralSettings
     * @param [out] param Parameters
     * @param map String map containing the parameter values
     * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the default values will remain)
     * @return True on success
     */
    static bool parseFromStringMap( ASP_GeneralSettings& params, const std::map<std::string, std::string>& map, bool strict = false);

    /**
     * @brief Parse the parameters to a string map
     * @param param Parameters
     * @return String map containing the parameter values
     */
    static std::map<std::string, std::string> parseToStringMap( const ASP_GeneralSettings& params);
};

class IActivitiesSwitchingPlanner : public LoggingComponent
{
public:
    struct ActivitiesInfo
    {
        std::pair<int, int> indexRangeMin;// indexes of the base route corresponding to <start, end> of the activities when the base machine does no work before starting the activity (e.g. if the base machine is a harvester with bunker, this indexes correspond to the window when the harvester does not load anything in its bunker before the olv starts overloading)
        std::pair<int, int> indexRangeMax;// indexes of the base route corresponding to <start, end> of the activities when the base machine works to its limit before starting the next activity (e.g. if the base machine is a harvester with bunker, this indexes correspond to the window when the harvester has loaded its bunker capacity and needs and cannot continue on its own)
        Machine machine; /**< Machine assigned to the activity */
        bool toResourcePointFirst = false; /**< If true, a trip to a resource point is needed before the activity is started */

    };

};

}

#endif // AROLIB_ACTIVITIESSWITCHINGPLANNER_H
