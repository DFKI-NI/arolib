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
 
#ifndef AROLIB_SHAREDGRIDSMANAGER_HPP
#define AROLIB_SHAREDGRIDSMANAGER_HPP

#include <memory>
#include <mutex>

#include "arolib/misc/randomgeneration.hpp"
#include "arolib/cartography/gridcellsinfomanager.hpp"

namespace arolib {
namespace gridmap{

/**
 * @brief Class used to manage gridmaps of type ArolibGrid_t that are shared by multiple objects
 */
class SharedGridsManager : public LoggingComponent
{
public:
    using GridType = ArolibGrid_t;
    using GridPtr = std::shared_ptr<GridType>;
    using ConstGridPtr = std::shared_ptr<const GridType>;

    /**
     * @brief Type for calculations
     */
    enum PreciseCalculationOption{
        NOT_PRECISE, /**< Simple calculations. */
        PRECISE, /**< Precise calculations. */
        PRECISE_ONLY_IF_AVAILABLE /**< Precise calculations only if cells range data is available. Otherwise simple calculations */
    };

    /**
     * @brief Constructor
     * @param logLevel Log level
     */
    explicit SharedGridsManager(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Destructor
     */
    virtual ~SharedGridsManager();

    /**
     * @brief Set shared CellsInfoManager to record edge cells data
     * @param cim CellsInfoManager. If null, no recording will be done
     * @param removeGrids If true, the grids from the manager will be removed from the current GridCellsInfoManager (if set). If false, it won't be removed and it will only be removed if done extarnally (the manager loses track)
     * @return True on success
     */
    virtual bool setCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim, bool removeGrids = true);

    /**
     * @brief Check if a gridgridmap exists
     * @param name Name of the gridmap
     * @return True if the gridmap exists
     */
    virtual bool hasGrid(const std::string &name);

    /**
     * @brief Add a shared grid
     * @param name Grid name
     * @param grid Grid
     * @param replaceIfExists Replace the grid if a grid with the same name already exists. If set to true && the grid exists && CellsInfoManager is set && the new grid has the same geometric properties, the cells information will not be removed
     * @return True if the gridmap exists
     */
    virtual bool addGrid(const std::string &name, ConstGridPtr grid, bool replaceIfExists = true);

    /**
     * @brief Remove a shared grid
     * @param name Grid name
     * @return True if the gridmap exists
     */
    virtual bool removeGrid(const std::string &name);

    /**
     * @brief Get a shared grid
     * @param name Grid name
     * @return Shared grid (null if grid does not exist)
     */
    virtual ConstGridPtr getGrid(const std::string &name) const;

    /**
     * @brief Get a grid copy
     * @param name Grid name
     * @return Copy grid (null if grid does not exist)
     */
    virtual GridPtr getGridCopy(const std::string &name) const;

    /**
     * @brief Clear data
     */
    virtual bool clearAll();

    /**
     * @brief Get the list of GridCellInfo under a line/rectangle for a given grid
     * @param p0 First point of the line
     * @param p1 Second point of the line
     * @param width Width of the line
     * @param precise Precision calculation option
     * @param cellsInfo [out] List of GridCellInfo under the line/rectangle
     * @return True on success
     */
    virtual bool getCellsInfoUnderLine(const std::string& gridName,
                                       const Point& p0,
                                       const Point& p1,
                                       double width,
                                       PreciseCalculationOption precise,
                                       std::vector<gridmap::GridmapLayout::GridCellOverlap>& cellsInfo);


protected:
    /**
     * @brief Removes all grids from current CellsInfoManager
     */
    virtual void removeGridsFromCIM();

protected:
    int m_id; /**< Internal id of the SharedGridsManager object. */
    std::string m_prefix; /**< Internal prefix for the SharedGridsManager. */
    std::map< std::string, ConstGridPtr > m_grids; /**< Shared grids. */
    std::shared_ptr<gridmap::GridCellsInfoManager> m_cellsInfoManager = nullptr; /**< GridCellsInfoManager. */

};

}
}

#endif // AROLIB_SHAREDGRIDSMANAGER_HPP
