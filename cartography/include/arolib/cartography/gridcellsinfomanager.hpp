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
 
#ifndef AROLIB_GRIDCELLSINFOMANAGER_H
#define AROLIB_GRIDCELLSINFOMANAGER_H

#include <memory>
#include <mutex>
#include <unordered_map>

#include "arolib/cartography/common.hpp"
#include "arolib/misc/loggingcomponent.h"

namespace arolib {
namespace gridmap{

/**
 * @brief Class used to manage the cells info from several grids that might have the same geometries
 */
class GridCellsInfoManager : public LoggingComponent
{
public:
    /**
     * @brief Holds the geometric information of an edge/segment
     */
    struct Edge{
        Point p0; /**< First point */
        Point p1; /**< Second point */
        double width; /**< Edge width */
        bool precise = true; /**< Is precise */

        /**
         * @brief Constructor.
         * @param _p0 First point.
         * @param _p1 Second point.
         * @param _width Edge width.
         * @param _precise Is precise.
         */
        explicit Edge(const Point &_p0, const Point &_p1, double _width, bool _precise);

        /**
         * @brief operator==.
         *
         * If p1 == e.p0 and p0 == e.p1, it is considered that the edge points are equal.
         * @param e Other edge.
         * @return True if the edge is equal to the given (other) edge
         */
        bool operator==(const Edge &e) const;

        /**
         * @brief operator>.
         *
         * Mostly based on the distance between the edges and the origin [0,0]. Used for sorting the edges.
         * @param e Other edge.
         * @return True if the edge is > than the given (other) edge
         */
        bool operator>(const Edge &e) const;

        /**
         * @brief operator<.
         *
         * Mostly based on the distance between the edges and the origin [0,0]. Used for sorting the edges.
         * @param e Other edge.
         * @return True if the edge is < than the given (other) edge
         */
        bool operator<(const Edge &e) const;

        /**
          * @brief Struct to get hash value
          */
        struct KeyHash{
            /**
             * @brief operator().
             *
             * Get hash value for an edge
             * @param e Edge.
             * @return Hash value
             */
            std::size_t operator()(const Edge &e) const;

            /**
             * @brief Get hash value for an edge.
             * @param e Edge.
             * @param seed Seed (for hash values of compound objects).
             * @return Hash value
             */
            static std::size_t get(const Edge &e, std::size_t seed = 0);
        };
    };

    // copy constructor - not created automatically because of the mutex and required for pyarolib
    explicit GridCellsInfoManager(const GridCellsInfoManager& other);

    /**
     * @brief Constructor
     *
     * @param logLevel Log level
     */
    explicit GridCellsInfoManager(const LogLevel& logLevel=arolib::LogLevel::INFO);

    /**
     * @brief Destructor
     */
    virtual ~GridCellsInfoManager();

    /**
     * @brief Register a gridmap
     *
     * @param name Name of the gridmap
     * @param lowerLeftCorner Point corresponding to the lower-left corner of the gridmap
     * @param numCellsX number of x-axis cells (columns) of the gridmap
     * @param numCellsY number of y-axis cells (rows) of the gridmap
     * @param cellsize Size of the cell [m]
     * @param replaceIfExists If true and a gridmap with thesame name is already registered, it will be replaced.
     * @return true on success
     */
    bool registerGrid(const std::string &name, const Point& lowerLeftCorner, size_t numCellsX, size_t numCellsY, double cellsize, bool replaceIfExists = true);

    /**
     * @brief Register a gridmap
     *
     * @param name Name of the gridmap
     * @param lo Gridmap layout
     * @param replaceIfExists If true and a gridmap with thesame name is already registered, it will be replaced.
     * @return true on success
     */
    bool registerGrid(const std::string &name, const gridmap::GridmapLayout &lo, bool replaceIfExists = true);

    /**
     * @brief Register a gridmap
     *
     * @param name Name of the gridmap
     * @param grid Gridmap
     * @param replaceIfExists If true and a gridmap with thesame name is already registered, it will be replaced.
     * @return true on success
     */
    template<typename T1>
    bool registerGrid(const std::string &name, const gridmap::Gridmap<T1> &grid, bool replaceIfExists = true){
        return registerGrid( name, grid.getLayout(), replaceIfExists );
    }

    /**
     * @brief Removes a gridmap and, if necessary, the data related to it
     * @param name Name of the gridmap to be removed
     * @return true on success
     */
    bool removeGrid(const std::string &name);

    /**
     * @brief Check if a gridmap is registered
     * @param name Name of the gridmap
     * @return true if registered
     */
    bool isGridRegistered(const std::string &gridName);

    /**
     * @brief Check if the geometry of a registered grid concurrs with the given grid
     * @param gridName Name of the gridmap
     * @param lo Gridmap layout
     * @return true if concurrs
     */
    bool checkGridGeometry(const std::string &gridName, const gridmap::GridmapLayout &lo);

    /**
     * @brief Check if the geometry of a registered grid concurrs with the given grid
     * @param gridName Name of the gridmap
     * @param grid Gridmap
     * @return true if concurrs
     */
    template<typename T1>
    bool checkGridGeometry(const std::string &gridName, const gridmap::Gridmap<T1> &grid){
        return checkGridGeometry(gridName, grid.getLayout());
    }

    /**
     * @brief Updates the cells information of a given edge/segment for a registered grid
     * @param gridName Name of the gridmap
     * @param edge Edge
     * @param cellsInfo Edge cells information
     * @return true on success
     */
    bool updateCellsInfo( const std::string& gridName, const Edge& edge, const std::vector<gridmap::GridmapLayout::GridCellOverlap>& cellsInfo );

    /**
     * @brief Computes and updates the cells information of a given edge/segment for a registered grid
     * @param gridName Name of the gridmap
     * @param edge Edge
     * @param lo Gridmap layout
     * @param computeOnlyIfNotExisting If true, if the cells information for the given edge and gridmap exists, no computation is done.
     * @param [out] cellsInfo Computed/retreived edge cells information
     * @return true on success
     */
    bool computeAndUpdateCellsInfo( const std::string& gridName, const Edge& edge, const gridmap::GridmapLayout &lo, bool computeOnlyIfNotExisting = true, std::vector<gridmap::GridmapLayout::GridCellOverlap>* cellsInfo = nullptr );

    /**
     * @brief Computes and updates the cells information of a given edge/segment for a registered grid
     * @param gridName Name of the gridmap
     * @param edge Edge
     * @param grid Gridmap
     * @param computeOnlyIfNotExisting If true, if the cells information for the given edge and gridmap exists, no computation is done.
     * @param [out] cellsInfo Computed/retreived edge cells information
     * @return true on success
     */
    template<typename T1>
    bool computeAndUpdateCellsInfo( const std::string& gridName, const Edge& edge, const gridmap::Gridmap<T1> &grid, bool computeOnlyIfNotExisting = true, std::vector<gridmap::GridmapLayout::GridCellOverlap>* cellsInfo = nullptr ){
        return computeAndUpdateCellsInfo(gridName, edge, grid.getLayout(), computeOnlyIfNotExisting, cellsInfo);
    }

    /**
     * @brief Computes the cells information of a given edge/segment for a given grid (does not store data)
     * @param lo Gridmap layout
     * @param edge Edge
     * @param [out] cellsInfo Computed edge cells information
     * @return true on success
     */
    static bool computeListCellsUnderLine( const gridmap::GridmapLayout &lo, const Edge& edge, std::vector<gridmap::GridmapLayout::GridCellOverlap>& cellsInfo );

    /**
     * @brief Computes the cells information of a given edge/segment for a given grid (does not store data)
     * @param grid Gridmap
     * @param edge Edge
     * @param [out] cellsInfo Computed edge cells information
     * @return true on success
     */
    template<typename T1>
    static bool computeListCellsUnderLine( const gridmap::Gridmap<T1> &grid, const Edge& edge, std::vector<gridmap::GridmapLayout::GridCellOverlap>& cellsInfo ){
        return computeListCellsUnderLine(grid.getLayout(), edge, cellsInfo);
    }

    /**
     * @brief Checks if the cells information of a given edge/segment for a registered grid exist
     * @param gridName Name of the gridmap
     * @param edge Edge
     * @return true if exists
     */
    bool hasCellsInfo(const std::string& gridName, const Edge& edge) const;

    /**
     * @brief Gets the (saved) cells information of a given edge/segment for a registered grid
     * @param gridName Name of the gridmap
     * @param edge Edge
     * @param [out] cellsInfo Edge cells information
     * @return true on success
     */
    bool getCellsInfo( const std::string& gridName, const Edge& edge, std::vector<gridmap::GridmapLayout::GridCellOverlap>& cellsInfo ) const;

    /**
     * @brief Clears all data
     */
    void clearAll();


protected:
    /**
     * @brief Holds the basic geometric properties of the grid relative to the origin (0,0)
     */
    struct GridSearchProperties{
        Point corner; /**< Point corresponding to the upper-right corner of the grid after translation to [0,0] */
        double cellsize = -1; /**< Grid cellsize/resolution */

        /**
         * @brief Constructor.
         */
        explicit GridSearchProperties() = default;

        /**
         * @brief Constructor.
         * @param _corner Corner.
         * @param _cellsize Cellsize.
         */
        explicit GridSearchProperties(const Point &_corner, double _cellsize);

        /**
         * @brief Constructor (from a gridmap layout).
         * @param lo Gridmap layout.
         */
        explicit GridSearchProperties(const gridmap::GridmapLayout &lo);

        /**
         * @brief operator==.
         * @param other Other GridSearchProperties.
         * @return true if equal
         */
        bool operator==(const GridSearchProperties &other) const;

        /**
         * @brief operator!=.
         * @param other Other GridSearchProperties.
         * @return true if not equal
         */
        bool operator!=(const GridSearchProperties &other) const;

        /**
         * @brief operator<.
         * @param other Other GridSearchProperties.
         * @return true if this < other
         */
        bool operator<(const GridSearchProperties &other) const;

        /**
         * @brief operator>.
         * @param other Other GridSearchProperties.
         * @return true if this > other
         */
        bool operator>(const GridSearchProperties &other) const;
    };

    /**
     * @brief Holds the edge complex information
     */
    struct EdgeProperties{
        std::unordered_map< Edge, std::vector<gridmap::GridmapLayout::GridCellOverlap>, Edge::KeyHash > gridCellsInfo; /**< Holds the information of the cells related to an edge (i.e. the cells that an edge overlaps) for each of the grid */

        /**
         * @brief Constructor.
         */
        explicit EdgeProperties();
    };

protected:
    mutable std::mutex m_mutex; /**< Mutex */
    std::map<std::string, std::pair<GridSearchProperties, Point>> m_gridProperties; /**< Holds the properties (plus lower-left corner) of a grid */
    std::map<GridSearchProperties, std::pair<size_t, EdgeProperties> > m_edgeProperties; /**< Holds the EdgeProperties for each grid and the number of grids registered to it*/
};


}
}
#endif // AROLIB_GRIDCELLSINFOMANAGER_H
