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
 
#ifndef AROLIB_OVERRIDEMAP_PROCESSOR_HPP
#define AROLIB_OVERRIDEMAP_PROCESSOR_HPP

#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>

#include "arolib/types/machine.hpp"
#include "arolib/types/route.hpp"
#include "arolib/geometry/curves_helper.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/container_helper.h"
#include "arolib/misc/basic_responses.h"
#include "arolib/cartography/common.hpp"

namespace arolib{

/**
 * @brief 
 */
class OverrideMapProcessor : public LoggingComponent{

public:

    /**
     * @brief Constructor
     * @param logLevel Log level
     */
    explicit OverrideMapProcessor (LogLevel logLevel = LogLevel::INFO);


    /**
     * @brief Create the machine override maps for a subfield from the routes (simple generation)
     * @param logLevel Log level
     * @param boundary Field/subfield boundary
     * @param routes Routes
     * @param machines Machines
     * @param resolution Cell size of the resulting maps. Id <= 0, is selected automatically from the machine widths
     * @param [out] overrideCountMap Map with the machine override counts (if null, no map is generated)
     * @param [out] overrideMassMap Map with the machine override mass [Kg] (if null, no map is generated)
     * @param [out] overrideMassPerTimeMap Map with the machine override mass times the duration [kg * s] (if null, no map is generated)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp createMaps_simple(const Polygon &boundary,
                              const std::vector<Route>& routes,
                              const std::vector<Machine>& machines,
                              double resolution,
                              std::shared_ptr<ArolibGrid_t> overrideCountMap = nullptr ,
                              std::shared_ptr<ArolibGrid_t> overrideMassMap = nullptr ,
                              std::shared_ptr<ArolibGrid_t> overrideMassPerTimeMap = nullptr );

    /**
     * @brief Create the machine override maps for a subfield from the routes
     * @param logLevel Log level
     * @param boundary Field/subfield boundary
     * @param routes Routes
     * @param machines Machines
     * @param resolution Cell size of the resulting maps. Id <= 0, is selected automatically from the machine widths
     * @param bePresice Be precise with the grid operations
     * @param [out] overrideCountMap Map with the machine override counts (if null, no map is generated)
     * @param [out] overrideMassMap Map with the machine override mass [Kg] (if null, no map is generated)
     * @param [out] overrideMassPerTimeMap Map with the machine override mass per time [Kg/s] (if null, no map is generated)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp createMaps(const Polygon &boundary,
                       const std::vector<Route>& routes,
                       const std::vector<Machine>& machines,
                       double resolution,
                       bool bePrecise,
                       std::shared_ptr<ArolibGrid_t> overrideCountMap = nullptr ,
                       std::shared_ptr<ArolibGrid_t> overrideMassMap = nullptr ,
                       std::shared_ptr<ArolibGrid_t> overrideMassPerTimeMap = nullptr );
protected:
    /**
     * @brief Initialize the maps from a boundary
     * @param boundary Boundary
     * @param resolution Cell size
     * @param [out] overrideCountMap Map with the machine override counts (if null, no map is generated)
     * @param [out] overrideMassMap Map with the machine override mass [Kg] (if null, no map is generated)
     * @param [out] overrideMassPerTimeMap Map with the machine override mass per time [Kg/s] (if null, no map is generated)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp initMaps(const Polygon &boundary,
                     double resolution,
                     std::shared_ptr<ArolibGrid_t> overrideCountMap,
                     std::shared_ptr<ArolibGrid_t> overrideMassMap,
                     std::shared_ptr<ArolibGrid_t> overrideMassPerTimeMap);


};

}

#endif // AROLIB_OVERRIDECOUNTMAP_PROCESSOR_HPP
