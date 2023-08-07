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
 
#ifndef AROLIB_COMPONENTPARAMETERSHELPER_H
#define AROLIB_COMPONENTPARAMETERSHELPER_H

#include "fieldgeometryprocessor.h"
#include "baseroutesplanner.h"
#include "fieldprocessplanner.h"

#include "arolib/io/io_common.hpp"

namespace arolib{

/**
 * @brief Parse the components parameters into a string map
 * @param fieldGeometryProcessorParameters_headland Headland parameters of the field geometry processor
 * @param fieldGeometryProcessorParameters_infield Inner-field parameters of the field geometry processor
 * @param baseRoutesPlannerParameters Parameters of the base-routes planner
 * @param fieldProcessPlannerParameters  Parameters of the field-process  planner
 * @return String map containing the component parameters
 */
std::map<std::string, std::map<std::string, std::string> >
    getParametersAsMap(const FieldGeometryProcessor::HeadlandParameters & fieldGeometryProcessorParameters_headland,
                       const FieldGeometryProcessor::InfieldParameters & fieldGeometryProcessorParameters_infield,
                       const BaseRoutesPlanner::PlannerParameters & baseRoutesPlannerParameters,
                       const FieldProcessPlanner::PlannerParameters & fieldProcessPlannerParameters );

/**
 * @brief Updates the components parameters parsing from a string map containing their values
 * @param String map containing the component parameters' values
 * @param [in/out] fieldGeometryProcessorParameters_headland Headland parameters of the field geometry processor
 * @param [in/out] fieldGeometryProcessorParameters_infield Inner-field parameters of the field geometry processor
 * @param [in/out] baseRoutesPlannerParameters Parameters of the base-routes planner
 * @param [in/out] fieldProcessPlannerParameters  Parameters of the field-process  planner
 * @return True on success
 */
bool setParametersFromMap(const std::map<std::string, std::map<std::string, std::string> >& map,
                          FieldGeometryProcessor::HeadlandParameters & fieldGeometryProcessorParameters_headland,
                          FieldGeometryProcessor::InfieldParameters & fieldGeometryProcessorParameters_infield,
                          BaseRoutesPlanner::PlannerParameters & baseRoutesPlannerParameters,
                          FieldProcessPlanner::PlannerParameters & fieldProcessPlannerParameters);


}

#endif // AROLIB_COMPONENTPARAMETERSHELPER_H
