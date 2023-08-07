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
 

#ifndef _AROLIB_GRIDMAP_LAYOUT_HPP_
#define _AROLIB_GRIDMAP_LAYOUT_HPP_

#include <type_traits>
#include <string>
#include <sys/stat.h>
#include <math.h>
#include <limits>
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <future>
#include <unordered_map>
#include <unordered_set>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/algorithm/string.hpp>

#include "cellsrange.hpp"
#include "cellsrangeset.hpp"
#include "arolib/types/units.hpp"
#include "arolib/types/field.hpp"
#include "arolib/types/coordtransformer.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/color_helper.hpp"

#include <gdal/gdal_priv.h>
#include <gdal/ogr_spatialref.h>
#include <gdal/gdalwarper.h>

#include <png++/image.hpp>
#include <png++/rgb_pixel.hpp>

namespace arolib {

/**
 * Gridmap namespace.
 */
namespace gridmap{

/**
 * @brief Class holding the geometric layout of a gridmap
 */
class GridmapLayout : public LoggingComponent{

public:

    /**
     * @brief Holds the basic information of a cell in the layout
     */
    struct GridCell{
        int x = -1; /**< X-index */
        int y = -1; /**< Y-index */

        /**
         * @brief Default constructor
         */
        GridCell() = default;

        /**
         * @brief Constructor
         * @param _x X-index
         * @param _y Y-index
         */
        GridCell(int _x, int _y):x(_x),y(_y){}

        /**
         * @brief operator== (check if the cell information is equal)
         * @param other Other cell
         * @return True if the cell information is equal
         */
        bool operator==(const GridCell &other) const{
            return x == other.x && y == other.y;
        }
    };

    /**
     * @brief Holds information about the overlap of a cell plus its basic information (inherits from GridCell)
     */
    struct GridCellOverlap : public GridCell{
        float overlap = 0; /**< Overlap [0,1] */

        /**
         * @brief Default constructor
         */
        GridCellOverlap() = default;

        /**
         * @brief Constructor
         * @param _x X-index
         * @param _y Y-index
         * @param _overlap Overlap [0,1]
         */
        GridCellOverlap(int _x, int _y, float _overlap):GridCell(_x, _y), overlap(_overlap){}


        /**
         * @brief operator== (check if the cell information + overlap are equal)
         * @param other Other cell
         * @return True if the cell information + overlap are equal
         */
        bool operator==(const GridCellOverlap &other) const{
            return GridCell::operator ==(other) && std::fabs(overlap - other.overlap) < 1e-10;
        }
    };

    /**
     * Constructor.
     * @param minX Minimum x-axis position (in real world coordinates).
     * @param maxX Maximum x-axis position (in real world coordinates).
     * @param sizeX number of cells in x-axis (i.e., number of columns).
     * @param sizeY number of cells in y-axis (i.e., number of rows).
     * @param cellsize cellsize (m).
     * @param logLevel Log level.
     */
    explicit GridmapLayout(double minX, double minY,
                           size_t sizeX, size_t sizeY,
                           double cellsize,
                           LogLevel logLevel = LogLevel::INFO);

    /**
     * Constructor.
     * @param logLevel Log level.
     */
    explicit GridmapLayout(LogLevel logLevel = LogLevel::INFO);

    /**
     * Copy constructor.
     * @param other Other layout to copy from.
     */
    GridmapLayout(const GridmapLayout& other);

    /**
     * Destructor.
     */
    virtual ~GridmapLayout();

    //------------------------------------
    //--------------OPERATORS-------------
    //------------------------------------

    /**
     * Copy assignment.
     * @param other Other layout to copy from.
     * @return this.
     */
    virtual GridmapLayout& operator=(const GridmapLayout& other);

    /**
     * @brief operator== (check if the layouts are equal)
     * @param other Other layout
     * @return True if the layouts are equal
     */
    virtual bool operator==(const GridmapLayout& other) const;

    /**
     * @brief operator!= (check if the layouts are different)
     * @param other Other layout
     * @return True if the layouts are different
     */
    virtual inline bool operator!=(const GridmapLayout& other) const { return !( *this == other ); }


    /**
     * @brief operator<
     * @param other Other layout
     * @return True if this is considered to be "lower" than other
     */
    virtual bool operator<(const GridmapLayout& other) const;


    //------------------------------------
    //--------------SET-PARAM-------------
    //------------------------------------

    /**
     * Reset parameters.
     */
    virtual void reset();

    /**
     * Initializes the layout
     * @param minX Minimum x-axis position (in real world coordinates).
     * @param minY Minimum y-axis position (in real world coordinates).
     * @param sizeX number of cells in x-axis (i.e., number of columns).
     * @param sizeY number of cells in y-axis (i.e., number of rows).
     * @param cellsize Resolution.
     * @return True if the final layout is valid
     */
    virtual bool init(double minX, double minY,
                      size_t sizeX, size_t sizeY,
                      double cellsize);

    /**
     * Sets the sizes of x and y axis and recomputes the limits based on the internal cellsize.
     * @param sizeX number of cells in x-axis (i.e., number of columns).
     * @param sizeY number of cells in y-axis (i.e., number of rows).
     * @return True if the final layout is valid
     */
    virtual bool setSize(size_t sizeX, size_t sizeY);

    /**
     * Sets the tolerance (in real world coordinates) to accept a value out of the X- and Y- axis range.
     * @param tolerance Tolerance.
     * @return True true in success
     */
    virtual bool setTolerance(double tolerance);

    /**
     * Increase/decrease the resolution
     * @param scale If > 0, the new_resolution = old_resolution * scale ; if < 0, the new_resolution = old_resolution * (-1/scale).
     * @return True in success
     */
    virtual bool scaleResolution(int scale);

    /**
     * Sets the option to make computations in multiple threads.
     * @param val value
     */
    virtual void setComputeInMultiThread(bool val);


    //------------------------------------
    //--------------GET-PARAM-------------
    //------------------------------------

    /**
     * Check if the grid layout is valid.
     * @return True if valid
     */
    virtual bool isValid() const;

    /**
     * Get the number of columns of the grid.
     * @return Number of columns of the grid
     */
    virtual unsigned int getSizeX() const;

    /**
     * Get the number of rows of the grid.
     * @return Number of rows of the grid
     */
    virtual unsigned int getSizeY() const;

    /**
     * Get the size of the grid cell (i.e. the grid resolution).
     * @return Size of the grid cell (i.e. the grid resolution)
     */
    virtual double getCellsize() const;

    /**
     * Get the area of a grid cell (in m²).
     * @return Area of a grid cell
     */
    virtual double getCellArea() const;

    /**
     * Get the minimum x-axis position (in real world coordinates) of the grid.
     * @return Minimum x-axis position
     */
    virtual double getMinPointX() const;

    /**
     * Get the maximum x-axis position (in real world coordinates) of the grid.
     * @return Maximum x-axis position
     */
    virtual double getMinPointY() const;

    /**
     * Get the minimum y-axis position (in real world coordinates) of the grid.
     * @return Minimum y-axis position
     */
    virtual double getMaxPointX() const;

    /**
     * Get the maximum y-axis position (in real world coordinates) of the grid.
     * @return Maximum y-axis position
     */
    virtual double getMaxPointY() const;

    /**
     * Get the point corresponding to the lower-left corner of the grid boundary (in real world coordinates).
     * @return lower-left corner
    */
    virtual Point getLowerLeftCorner() const;

    /**
    * Get the point corresponding to the upper-right corner of the grid boundary (in real world coordinates).
    * @return upper-left corner
    */
    virtual Point getUpperRightCorner() const;

    /**
     * Returns the tolerance (in real world coordinates) to accept a value out of the X- and Y- axis range.
     * @return Tolerance
     */
    virtual double getTolerance() const;


    /**
     * Returns the ComputeInMultiThread setting.
     * @return ComputeInMultiThread setting
     */
    virtual bool getComputeInMultiThread() const;


    //------------------------------------
    //--------------GET-VALUE-------------
    //------------------------------------


    /**
    * Get the indexes ranges of the cells that are in touch with the line.
    * @param start Start-point of the line (in *real world coordinates*).
    * @param end End-point of the line (in *real world coordinates*).
    * @param simple Simple computation?.
    * @return A CellsRangeList used to obtain the range of columns (X-indexes) touched by the line, and the range of rows (Y-idexes) corresponding to each column
    */
    virtual CellsRangeList getCellsUnderLine(const Point& start, const Point& end, bool simple) const;

    /**
     * Get the indexes ranges of the cells that are in touch with the line.
     * @param start Start-point of the line (in *real world coordinates*).
     * @param end End-point of the line (in *real world coordinates*).
     * @return A CellsRangeList used to obtain the range of columns (X-indexes) touched by the line, and the range of rows (Y-idexes) corresponding to each column
     */
    virtual CellsRangeList getCellsUnderLine2(const Point& start, const Point& end) const;

   /**
    * Get the indexes ranges of the cells that are in touch with the line.
    * @param start Start-point of the line (in *real world coordinates*).
    * @param end End-point of the line (in *real world coordinates*).
    * @return A CellsRangeList used to obtain the range of columns (X-indexes) touched by the line, and the range of rows (Y-idexes) corresponding to each column
    */
   virtual CellsRangeList getCellsUnderLine3(const Point& start, const Point& end) const;

    /**
     * Get the indexes ranges of the cells that are in touch with the expanded line.
     * @param start Start-point of the line (in *real world coordinates*).
     * @param end End-point of the line (in *real world coordinates*).
     * @param width Width of the line .The line will be extended sideways a distance if width/2 (i.e. width/2 to each side).
     * @return A CellsRangeList used to obtain the range of columns (X-indexes) touched by the extended line, and the range of rows (Y-idexes) corresponding to each column
     */
    virtual CellsRangeList getCellsUnderLine(const Point& start,
                                             const Point& end,
                                             double width) const;

   /**
    * Get the indexes ranges of the cells that are in touch with the expanded line.
    * @param start Start-point of the line (in *real world coordinates*).
    * @param end End-point of the line (in *real world coordinates*).
    * @param width Width of the line .The line will be extended sideways a distance if width/2 (i.e. width/2 to each side).
    * @return A CellsRangeList used to obtain the range of columns (X-indexes) touched by the extended line, and the range of rows (Y-idexes) corresponding to each column
    */
   virtual CellsRangeList getCellsUnderLine2(const Point& start,
                                             const Point& end,
                                             double width) const;

    /**
     * Get the indexes of the cells that are in touch with the expanded line, as well as how much of the cell is under the extended line.
     * @param start Start-point of the line (in *real world coordinates*).
     * @param end End-point of the line (in *real world coordinates*).
     * @param width Width of the line .The line will be extended sideways a distance if width/2 (i.e. width/2 to each side).
     * @param boundary Check for proportion inside the boundary. No taken into account if empty.
     * @return A vetor of GridCellOverlap's, where the value (between 0 and 1) of the GridCellOverlap corresponds to the amount of the cell that is under the extended line (e.g. if the extended line only covers half of the cell's area, the value will be 0.5)
     */
    virtual std::vector< GridCellOverlap > getCellsOverlapUnderLine(const Point& start,
                                                            const Point& end,
                                                            double width,
                                                            const Polygon& boundary = Polygon()) const;

    /**
     * Get the indexes of the cells that are in touch with the expanded line, as well as how much of the cell is under the extended line.
     * @param start Start-point of the line (in *real world coordinates*).
     * @param end End-point of the line (in *real world coordinates*).
     * @param width Width of the line .The line will be extended sideways a distance if width/2 (i.e. width/2 to each side).
     * @param boundary Check for proportion inside the boundary. No taken into account if empty.
     * @return A vetor of GridCellOverlap's, where the value (between 0 and 1) of the GridCellOverlap corresponds to the amount of the cell that is under the extended line (e.g. if the extended line only covers half of the cell's area, the value will be 0.5)
     */
    virtual std::vector< GridCellOverlap > getCellsOverlapUnderLine_v1(const Point& start,
                                                                       const Point& end,
                                                                       double width,
                                                                       const Polygon& boundary = Polygon()) const;

    /**
     * Get the indexes of the cells that are in touch with the expanded line, as well as how much of the cell is under the extended line.
     * @param start Start-point of the line (in *real world coordinates*).
     * @param end End-point of the line (in *real world coordinates*).
     * @param width Width of the line .The line will be extended sideways a distance if width/2 (i.e. width/2 to each side).
     * @param boundary Check for proportion inside the boundary. No taken into account if empty.
     * @return A vetor of GridCellOverlap's, where the value (between 0 and 1) of the GridCellOverlap corresponds to the amount of the cell that is under the extended line (e.g. if the extended line only covers half of the cell's area, the value will be 0.5)
     */
    virtual std::vector< GridCellOverlap > getCellsOverlapUnderLine_v2(const Point& start,
                                                                       const Point& end,
                                                                       double width,
                                                                       const Polygon& boundary = Polygon()) const;

    /**
     * Get the indexes ranges of the cells that are in touch with the polygon.
     * @param _poly polygon (in *real world coordinates*)
     * @param [out] boundaryCells Cells overlaping the polygon perimeter
     * @return A CellsRangeList used to obtain the range of columns (X-indexes) touched by the polygon, and the range of rows (Y-idexes) corresponding to each column
     */
    virtual CellsRangeList getCellsUnderPolygon(const Polygon& _poly, std::unordered_set< std::pair<int, int>, boost::hash< std::pair<int, int> > > *boundaryCells = nullptr) const; //@TODO it was tested only with a couble of simple polygons

    /**
     * Get the indexes ranges of the cells that are in touch with the polygon.
     * @param _poly polygon (in *real world coordinates*)
     * @param simple Simple computation?
     * @return A CellsRangeList used to obtain the range of columns (X-indexes) touched by the polygon, and the range of rows (Y-idexes) corresponding to each column
     */
    virtual CellsRangeList getCellsUnderPolygon2(const Polygon& _poly, bool simple = false) const;

    /**
     * Get the indexes of the cells that are in touch with the polygon, as well as how much of the cell is under the polygon.
     * @param _poly polygone (in *real world coordinates*)
     * @param boundary Check for proportion inside the boundary. No taken into account if empty.
     * @return A vector of GridCellOverlap's, where the value (between 0 and 1) of the GridCellOverlap corresponds to the amount of the cell that is under the polygon (e.g. if the polygon only covers half of the cell's area, the value will be 0.5)
     */
    virtual std::vector< GridCellOverlap > getCellsOverlapUnderPolygon(const Polygon& _poly,
                                                                    const Polygon& boundary = Polygon()) const;

    /**
     * Get the indexes of the cells that are in touch with the polygon, as well as how much of the cell is under the polygon.
     * @param _poly polygone (in *real world coordinates*)
     * @param boundary Check for proportion inside the boundary. No taken into account if empty.
     * @return A vector of GridCellOverlap's, where the value (between 0 and 1) of the GridCellOverlap corresponds to the amount of the cell that is under the polygon (e.g. if the polygon only covers half of the cell's area, the value will be 0.5)
     */

    virtual std::vector< GridCellOverlap > getCellsOverlapUnderPolygon_v1(const Polygon& _poly,
                                                                    const Polygon& boundary = Polygon()) const;

    /**
     * Get the indexes of the cells that are in touch with the polygon, as well as how much of the cell is under the polygon.
     *
     * Only computes intersection/overlap area of cells under the polygon boundary
     * @param _poly polygone (in *real world coordinates*)
     * @param boundary Check for proportion inside the boundary. No taken into account if empty.
     * @return A vector of GridCellOverlap's, where the value (between 0 and 1) of the GridCellOverlap corresponds to the amount of the cell that is under the polygon (e.g. if the polygon only covers half of the cell's area, the value will be 0.5)
     */

    virtual std::vector< GridCellOverlap > getCellsOverlapUnderPolygon_v2(const Polygon& _poly,
                                                                    const Polygon& boundary = Polygon()) const;

    /**
     * Get the indexes of the cells that are in touch with the polygon, as well as how much of the cell is under the polygon.
     *
     * Increases resolution and makes a simple cellsUnderPolygon
     * @param _poly polygone (in *real world coordinates*)
     * @param boundary Check for proportion inside the boundary. No taken into account if empty.
     * @return A vector of GridCellOverlap's, where the value (between 0 and 1) of the GridCellOverlap corresponds to the amount of the cell that is under the polygon (e.g. if the polygon only covers half of the cell's area, the value will be 0.5)
     */
    virtual std::vector< GridCellOverlap > getCellsOverlapUnderPolygon_v3(const Polygon& _poly,
                                                                    const Polygon& boundary = Polygon()) const;


    /**
     * Convert a CellsRangeList to a vector of GridCellOverlap with a given overlap
     * @param cellsList Input CellsRangeList
     * @param overlap Overlap [0,1]
     * @return Resulting vector of GridCellOverlap
     */
    static std::vector< GridCellOverlap > toGridCellOverlap(const CellsRangeList& cellsList, float overlap = 1.0);

    /**
     * Obtain the coordinates coresponding to the center given input cell.
     * @param cellX X-index of the desired cell.
     * @param cellY Y-index of the desired cell.
     * @param center Point corresponding to the cell's center.
     * @return True if successful (cell in range)
     */
    bool getCellCenter (unsigned int cellX, unsigned int cellY, Point &center) const;

    /**
     * Obtain the coordinates coresponding to the minimum corner of the given input cell.
     * @param cellX X-index of the desired cell.
     * @param cellY Y-index of the desired cell.
     * @param corner Point corresponding to the cell's minimum corner.
     * @return True if successful (cell in range)
     */
    virtual bool getCellMinCorner (unsigned int cellX, unsigned int cellY, Point &corner) const;

    /**
     * Obtain the line parameters coresponding to the given cell.
     * @param cellX X-index of the desired cell.
     * @param cellY Y-index of the desired cell.
     * @param p0 [out] First point of the line.
     * @param p1 [out] Second point of the line.
     * @param width [out] Line width.
     * @return True if successful (cell in range)
     */
    virtual bool getCellPolygonAsLine (unsigned int cellX, unsigned int cellY, Point &p0, Point &p1, double &width) const;

    /**
     * Obtain the arolib::polygon coresponding to the given cell.
     * @param cellX X-index of the desired cell.
     * @param cellY Y-index of the desired cell.
     * @param poly Polygon of the cell.
     * @return True if successful (cell in range)
     */
    virtual bool getCellPolygon (unsigned int cellX, unsigned int cellY, Polygon &poly) const;

    /**
     * Obtain the arolib::polygon coresponding to the grid (in real world coordinates).
     * @param _error_ [out] Indicates if there was an error in the calculation
     * @return The arolib::polygon coresponding to the grid (in real world coordinates).
     */
    virtual Polygon getGridPolygon(bool * _error_ = nullptr) const;


    /**
     * Obtain the area of the grid.
     * @param _error_ [out] Indicates if there was an error in the calculation
     * @return Area.
     */
    virtual double getGridArea(bool * _error_ = nullptr) const;



    /**
     * Computes the intersection area between a cell and a polygon.
     * @param cellPoly Polygon of the cell
     * @param cellsize Size of the cell (resolution)
     * @param poly Polygon
     * @param minCell Min index of the polygon coresponding to the given cell (e.g., for the given cell in x-index 'X', we know that the range of y-indexes under the polygon in column 'X' is [minCellY, maxCellY]
     * @param maxCell Max index of the polygon coresponding to the given cell (e.g., for the given cell in x-index 'X', we know that the range of y-indexes under the polygon in column 'X' is [minCellY, maxCellY]
     * @return Intersection area between a cell and the polygon.
     */
    static double getCellAreaIntersection(const Polygon& cellPoly,
                                          double cellsize,
                                          const Polygon& poly,
                                          int minCell,
                                          int maxCell);


    /**
     * Computes the intersection area between a cell and a polygon corresponding to a line.
     * @param cellPoly Polygon of the cell
     * @param cellsize Size of the cell (resolution)
     * @param linePoly Polygon corresponding to the line
     * @param lineWidth With of the line
     * @return Intersection area between a cell and the polygon.
     */
    static double getCellAreaIntersection(const Polygon& cellPoly,
                                          double cellsize,
                                          const Polygon& linePoly,
                                          double lineWidth);


    /**
     * Computes the intersection area between a cell and a polygon.
     * @param x X-index of the cell
     * @param y Y-index of the cell
     * @param poly Polygon
     * @param minCell Min index of the polygon coresponding to the given cell (e.g., for the given cell in x-index 'X', we know that the range of y-indexes under the polygon in column 'X' is [minCellY, maxCellY]
     * @param maxCell Max index of the polygon coresponding to the given cell (e.g., for the given cell in x-index 'X', we know that the range of y-indexes under the polygon in column 'X' is [minCellY, maxCellY]
     * @return Intersection area between a cell and the polygon.
     */
    virtual double getCellAreaIntersection(unsigned int x,
                                           unsigned int y,
                                           const Polygon& poly,
                                           int minCell,
                                           int maxCell) const;


    /**
     * Computes the intersection area between a cell and a polygon corresponding to a line.
     * @param x X-index of the cell
     * @param y Y-index of the cell
     * @param linePoly Polygon corresponding to the line
     * @param lineWidth With of the line
     * @return Intersection area between a cell and the polygon.
     */
    virtual double getCellAreaIntersection(unsigned int x,
                                           unsigned int y,
                                           const Polygon& linePoly,
                                           double lineWidth) const;


    /**
     * Computes the intersection area between a cell and a polygon.
     * @param x X-index of the cell
     * @param y Y-index of the cell
     * @param poly Polygon
     * @return Intersection area between a cell and the polygon.
     */
    virtual double getCellAreaIntersection(unsigned int x,
                                           unsigned int y,
                                           const Polygon& poly) const;

    //------------------------------------
    //----------------CHECK---------------
    //------------------------------------

    /**
     * Checks if the given point (in real world coordinates) is in the grid.
     * @param point Point.
     * @return True if the given point is in the grid
     */
    virtual bool checkPointInRange(const Point& point) const;

    /**
     * Checks if the x- and y- indexes lie within the grid's valid range.
     * @param x x-index (Grid column)
     * @param y y-index (Grid row)
     * @return True if indexes are valid
     */
    virtual bool checkIndexesInRange(unsigned int x, unsigned int y) const;

    /**
     * Checks if the x- and y- indexes lie within the grid's valid range.
     * @param x x-index (Grid column)
     * @param y y-index (Grid row)
     * @return True if indexes are valid
     */
    virtual bool checkIndexesInRange(int x, int y) const;

    //------------------------------------
    //-------------CONVERSIONS------------
    //------------------------------------

    /**
     * Get the grid indexes corresponding to a point (in real-world coordinates).
     * @param _p Point.
     * @param x Index in the x-axis (column).
     * @param y Index in the y-axis (row).
     * @param withTolerance If set to true, points outside the limits but within a tolerance will return the indexes of the closest limit point.
     * @return False if the conversion was unsuccesfull (e.g. the point lies outside the limit perimeter)
     */
    virtual bool point2index(const Point& _p, unsigned int &x, unsigned int &y, bool withTolerance=true) const;



protected:

    double m_minPointX = -1;  /**< Minimum X value (coordinates) for the grid */
    double m_minPointY = -1;  /**< Minimum Y value (coordinates) for the grid */

    // not strictly needed, but make debugging easier
    double m_maxPointX = -1;  /**< Maximum X value (coordinates) for the grid */
    double m_maxPointY = -1;  /**< Maximum Y value (coordinates) for the grid */

    double m_cellsize = -1;  /**< Size of the cell: the cell is a squere, hence the size of the cell referes the length/width of the square */

    unsigned int m_sizeX = 0;  /**< Number of columns of the grid's matrix */
    unsigned int m_sizeY = 0;  /**< Number of rows of the grid's matrix */

    double m_limitTolerance = 0.5;  /**< Tolerance (in real world coordinates) to accept a value out of the X- and Y- axis range */

    bool m_computeInMultiThread = true; /**< Make computations in multiple threads? */
};


}
}


#endif //_AROLIB_GRIDMAP_LAYOUT_HPP_

