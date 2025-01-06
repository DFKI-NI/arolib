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
 
#ifndef AROLIB_WORKEDAREAANALYST_H
#define AROLIB_WORKEDAREAANALYST_H

#include <memory>

#include "arolib/cartography/common.hpp"
#include "arolib/cartography/sharedgridsmanager.hpp"

namespace arolib{

/**
 * @brief Class used to anaylize worked states
 */
class WorkedAreaAnalyst
{
public:

    /**
     * @brief Worked state
     */
    enum WorkedState{
        WORKED, /**< Considered worked */
        NOT_WORKED, /**< Considered not-worked */
        UNKNOWN /**< Worked state unknown */
    };

    /**
     * @brief Set the worked- / remaining- area gridmap
     * @param gridmap Worked- / remaining- area gridmap.
     * @param readValueAsWorked Flag stating how to read the cell values [0, 1]. If true (worked-area map), 0: not-worked, 1: worked; otherwise (remaining-area map), 1: not-worked, 0: worked
     */
    void setWorkedAreaMap(std::shared_ptr<const ArolibGrid_t>& gridmap, bool readValueAsWorked);

    /**
     * @brief Get the worked- / remaining- area gridmap
     * @param [out] readValueAsWorked Flag stating how to read the cell values [0, 1]. If true (worked-area map), 0: not-worked, 1: worked; otherwise (remaining-area map), 1: not-worked, 0: worked
     * @return gridmap Worked- / remaining- area gridmap.
     */
    std::shared_ptr<const ArolibGrid_t> getWorkedAreaMap(bool* readValueAsWorked = nullptr);

    /**
     * @brief Set shared CellsInfoManager to record cells data
     * @param cim CellsInfoManager.
     */
    void setGridCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim);

    /**
     * @brief Set lower and upper bound threshold [0,1] used to consider an area worked or not, where 0:= not_worked and 1:= worked
     *
     * The values of a cell are converted to 'worked' values between [0, 1], where 0:=not_worked and 1:=worked. If this value is < lower, the cell is considered not worked; if it is > upper, it is considered worked; otherwise it is considered as unknown.
     *
     * @param lower Lower bound.
     * @param upper Upper bound.
     */
    bool setThresholds(float lower = 0.3, float upper = 0.7);

    /**
     * @brief Checks if a segment is worked based on the values of the overlapped cells of the worked-/remaining- area map/grid
     * @param p0 First point of the segment
     * @param p1 Second point of the segment
     * @param precision Precision used for computations with the worked/remaining-area map
     * @return first: worked state; second: computed value (to be read as worked value: i.e., 0:= not-worked and 1:= worked ; NAN if no valid value was computed)
     */
    std::pair<WorkedState, float> isSegmentWorked(const Point &p0, const Point &p1, gridmap::SharedGridsManager::PreciseCalculationOption precision);

    /**
     * @brief Checks if a segment is worked based on the values of the overlapped cells of the worked-/remaining- area map/grid (checking the boundary)
     *
     * The line segments inside given boundary will be the one used to obtain the overlapped cells.
     *
     * @param boundary Boundary
     * @param p0 First point of the segment
     * @param p1 Second point of the segment
     * @param precision Precision used for computations with the worked/remaining-area map
     * @return first: worked state; second: computed value (to be read as worked value: i.e., 0:= not-worked and 1:= worked ; NAN if no valid value was computed)
     */
    std::pair<WorkedState, float> isSegmentWorked(const Polygon &boundary, const Point &p0, const Point &p1, gridmap::SharedGridsManager::PreciseCalculationOption precision);

    /**
     * @brief Checks if a segment is worked based on the values of the overlapped cells of the worked-/remaining- area map/grid
     * @param p0 First point of the segment
     * @param p1 Second point of the segment
     * @param width Width of the segment
     * @param precision Precision used for computations with the worked/remaining-area map
     * @return first: worked state; second: computed value (to be read as worked value: i.e., 0:= not-worked and 1:= worked ; NAN if no valid value was computed)
     */
    std::pair<WorkedState, float> isSegmentWorked(const Point &p0, const Point &p1, double width, gridmap::SharedGridsManager::PreciseCalculationOption precision);

    /**
     * @brief Checks if a segment is worked based on the values of the overlapped cells of the worked-/remaining- area map/grid (checking the boundary)
     *
     * The intersection of the segment polygon with the given boundary will be the one used to obtain the overlapped cells.
     *
     * @param boundary Boundary
     * @param p0 First point of the segment
     * @param p1 Second point of the segment
     * @param width Width of the segment
     * @param precision Precision used for computations with the worked/remaining-area map
     * @return first: worked state; second: computed value (to be read as worked value: i.e., 0:= not-worked and 1:= worked ; NAN if no valid value was computed)
     */
    std::pair<WorkedState, float> isSegmentWorked(const Polygon &boundary, const Point &p0, const Point &p1, double width, gridmap::SharedGridsManager::PreciseCalculationOption precision);


    /**
     * @brief Checks if a point is worked based on the value of the corresponding cell of the worked-/remaining- area map/grid
     * @param p Point
     * @return first: worked state; second: computed value (to be read as worked value: i.e., 0:= not-worked and 1:= worked ; NAN if no valid value was computed)
     */
    std::pair<WorkedState, float> isPointWorked(const Point &p);

protected:
    std::pair<WorkedState, float> correctValueAndGetState(float value);

protected:
    float m_thresholdIsWorkedLB = 0.3; //0.4; /**< Lower bound threshold [0,1] used to consider an area worked or not, where 0:= not_worked and 1:= worked */
    float m_thresholdIsWorkedUB = 0.7; //0.6; /**< Upper bound threshold [0,1] used to consider an area worked or not, where 0:= not_worked and 1:= worked */
    std::shared_ptr<gridmap::GridCellsInfoManager> m_cim = nullptr; /**< Grid-cells-info manager */
    gridmap::SharedGridsManager m_gridsManager; /**< Gridmaps manager (holding the needed gridmaps) */
    bool m_readAsWorked = false; /**< Flag stating how to read the cell values [0, 1] */
};

}

#endif // AROLIB_WORKEDAREAANALYST_H
