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
 
#ifndef AROLIB_COMPONENTPARAMETERSHELPER_H
#define AROLIB_COMPONENTPARAMETERSHELPER_H

#include "headlandplanner.h"
#include "tracksgenerator.h"
#include "baseroutesinfieldplanner.h"
#include "fieldprocessplanner.h"

#include "arolib/io/io_common.hpp"

namespace arolib{

/**
 * @brief Parse the components parameters into a string map
 * @param headlandPlannerParameters Parameters of the headland planner
 * @param tracksGeneratorParameters Parameters of the tracks generator
 * @param baseRoutesPlannerParameters Parameters of the base-routes planner
 * @param fieldProcessPlannerParameters  Parameters of the field-process  planner
 * @return String map containing the component parameters
 */
std::map<std::string, std::map<std::string, std::string> >
    getParametersAsMap(const HeadlandPlanner::PlannerParameters & headlandPlannerParameters,
                        const TracksGenerator::TracksGeneratorParameters & tracksGeneratorParameters,
                        const BaseRoutesInfieldPlanner::PlannerParameters & baseRoutesPlannerParameters,
                        const FieldProcessPlanner::PlannerParameters & fieldProcessPlannerParameters );

/**
 * @brief Updates the components parameters parsing from a string map containing their values
 * @param String map containing the component parameters' values
 * @param [in/out] headlandPlannerParameters Parameters of the headland planner
 * @param [in/out] tracksGeneratorParameters Parameters of the tracks generator
 * @param [in/out] baseRoutesPlannerParameters Parameters of the base-routes planner
 * @param [in/out] fieldProcessPlannerParameters  Parameters of the field-process  planner
 * @return True on success
 */
bool setParametersFromMap(const std::map<std::string, std::map<std::string, std::string> >& map,
                          HeadlandPlanner::PlannerParameters &headlandPlannerParameters,
                          TracksGenerator::TracksGeneratorParameters &tracksGeneratorParameters,
                          BaseRoutesInfieldPlanner::PlannerParameters &baseRoutesPlannerParameters,
                          FieldProcessPlanner::PlannerParameters &fieldProcessPlannerParameters);


}

#endif // AROLIB_COMPONENTPARAMETERSHELPER_H
