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
 
#ifndef AROLIB_WORKEDAREAMANAGER_H
#define AROLIB_WORKEDAREAMANAGER_H

#include <map>

#include "arolib/misc/basic_responses.h"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/types/field.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/cartography/common.hpp"
#include "arolib/geometry/geometry_helper.hpp"

namespace arolib {

/**
 * @brief Class used to generate and update remaining area maps
 */
class WorkedAreaManager : public LoggingComponent
{
public:
    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    WorkedAreaManager(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Initialized the class with a virgin field. The remaining-area map is created with the field outer boundary.
     * @param field Working field
     * @param cellsize Grid's cell size
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp init(const Field &field, double cellsize, bool removeHeadland = false);

    /**
     * @brief Initialized the class with a virgin field, computing an appropiate cell size from the machineS' working group. The remaining-area map is created with the field outer boundary.
     * @param field Working field
     * @param machines Working group. The cell size is computed from their working widths
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp init(const Field &field, const std::vector<Machine> &machines, bool removeHeadland = false);

    /**
     * @brief Initialized the class with a current remaining-area map.
     * @param field Working field
     * @param remainingAreaMap Current remaining-area map
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp init(const Field &field, const ArolibGrid_t& remainingAreaMap);

    /**
     * @brief Check if it is ready (i.e. correctly initialized).
     * @return True if ready
     */
    bool isReady() const;

    /**
     * @brief Clear all data
     */
    void clear();

    /**
     * @brief Updates the grid/map with the given location (using the edge from the last location, if existent).
     * @param machine Driving machine (harvester)
     * @param pt New location
     * @param be_precise Perform map/grid operations precisely
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp addPoint(const Machine &machine, const Point& pt, bool be_precise = true);


    /**
     * @brief Get the current remaining-area map (if not ready, it is not allocated).
     * @return Current remaining-area map
     */
    const ArolibGrid_t& getRemainingAreaMap() const;

protected:
    bool m_ready = false; /**< Is the manager ready? */
    Field m_field; /**< Field */
    ArolibGrid_t m_remainingAreaMap; /**< Remaining area map */
    std::map<MachineId_t, std::vector<Point>> m_drivenEdges; /**< Driven edges */

};

}

#endif // AROLIB_WORKEDAREAMANAGER_H
