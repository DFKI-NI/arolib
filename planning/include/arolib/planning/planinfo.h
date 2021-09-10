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
 
#ifndef AROLIB_PLANINFO_HPP
#define AROLIB_PLANINFO_HPP

#include <vector>

namespace arolib{



/**
 * @brief Plan information
 */
struct PlanGeneralInfo{
    double planOverallCost = 0; /**< Overall plan cost (for all harvester routes) */
    std::vector<double> overallDelays; /**< Holds the delays of all harvesters (w.r.t. their original planned routes) */
    double planningDuration = -1; /**< Duration of planning [s] */
    double planDuration = -1; /**< Duration of the plan (routes) [s] */

    /**
     * @brief Get the sum of the delays of all harvesters (w.r.t. their original planned routes)
     * @return Sum of the delays of all harvesters (w.r.t. their original planned routes)
     */
    double getOverallDelay() const;
};

}

#endif // AROLIB_PLANINFO_HPP
