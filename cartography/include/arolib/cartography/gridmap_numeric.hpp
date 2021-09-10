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
 
#ifndef _AROLIB_GRIDMAP_NUMERIC_H_
#define _AROLIB_GRIDMAP_NUMERIC_H_

#include <type_traits>
#include <string>
#include <sys/stat.h>
#include <math.h>
#include <limits>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <future>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/algorithm/string.hpp>

#include "cellsrange.hpp"
#include "cellsrangeset.hpp"
#include "gridmap.hpp"
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
namespace gridmap {


/**
 * @brief Gridmap with numeric cell values
 */
template <typename T = float>
class NumericGridmap : public Gridmap<T>{

    static_assert(std::is_arithmetic<T>::value, "The cell value type of an AroGrid must be numeric (template argument)");

public:

    /**
     * @brief Types of values that can be computed
     */
    enum ComputedValueType{
        SUM, /**< sums of all values in the intersection area (assumes the value in the cell is the total ammount in the cell area) */
        AVERAGE_VALID, /**< calculates the average with the area of the geometry (polygone) that lies within the grid (unset values = 0) */
        AVERAGE_VALID_AND_SET, /**< calculates the average with the area of the geometry (polygone) that lies within the grid for the area that is set */
        AVERAGE_TOTAL /**< calculates the average with the total area of the geometry (polygone) (assumes that the value outside the grid is 0, unset values = 0) */
    };

    /**
     * @brief Get the ComputedValueType from its int value
     * @param value Int value
     * @return Coresponding ComputedValueType
     */
    static ComputedValueType intToComputedValueType(int value);

    /**
     * @brief Holds statistic information
     */
    struct GridStatistics{
        double avg = 0; /**< Average value. */
        double sd = 0; /**< Standard deviation. */
        T min = std::numeric_limits<T>::max(); /**< Minimum value. */
        T max = std::numeric_limits<T>::lowest(); /**< Maximum value. */
        size_t countCells = 0; /**<  Number of cells. */
        size_t validCells = 0; /**<  Number of valid cells. */
    };


    /**
    * Constructor.
    *
    * Already allocates the grid with the given size if the layout is valid.
    * @param lo Gridmap geometric layout.
    * @param logLevel Log level
    */
    explicit NumericGridmap(const GridmapLayout &lo, LogLevel logLevel = LogLevel::INFO);

    /**
     * Constructor.
     * The grid is not allocated yet (it doesn't exist).
    * @param logLevel Log level
     */
    explicit NumericGridmap(LogLevel logLevel = LogLevel::INFO);

    /**
     * Copy constructor.
     * @param other Other gridmap.
     */
    NumericGridmap(const NumericGridmap<T>& other);

    /**
     * Destructor.
     */
    virtual ~NumericGridmap();

    //------------------------------------
    //--------------OPERATORS-------------
    //------------------------------------

    /**
    * Copy assignment
    * @param other Other grid.
    */
    virtual NumericGridmap<T>& operator=(const NumericGridmap<T>& other);


    //------------------------------------
    //--------------SET-VALUE-------------
    //------------------------------------


    /**
     * Adds the given the data(value) to the current value of all the cells within the grid.
     * @param _value Value
     * @param onlyCellsWithValue If true, if the cell has no value, it will remain without value;
     * @return True on success
     */
    virtual bool addValue(const T &_value,
                          bool onlyCellsWithValue = false);


    /**
     * Adds the given the data(value) to the current value of a cell within the grid, specified by the cell's x- and y- indexes.
     * @param x X-index of the cell.
     * @param y Y-index of the cell.
     * @param _value Value
     * @param onlyCellsWithValue If true, if the cell has no value, it will remain without value;
     * @return True on success
     */
    virtual bool addValue(unsigned int x,
                          unsigned int y,
                          const T &_value,
                          bool onlyCellsWithValue = false);

    /**
     * Adds the given the data(value) to the current value of a grid point, given in *real world coordinates*.
     * @param p Point.
     * @param value Value
     * @param onlyCellsWithValue If true, if the cell has no value, it will remain without value;
     * @return True on success
     */
    virtual bool addValue(const Point& p,
                          const T &value,
                          bool onlyCellsWithValue = false);

    /**
     * Multiplies the current value of all cells within the grid by the given the value.
     * @param _value Value.
     * @return True on success
     */
    virtual bool multValue(const T &_value);


    /**
     * Multiplies the current value of a cell within the grid (specified by the cell's x- and y- indexes) by the given the value.
     * @param x X-index of the cell.
     * @param y Y-index of the cell.
     * @param _value Value.
     * @return True on success
     */
    virtual bool multValue(unsigned int x,
                           unsigned int y,
                           const T &_value);

    /**
     * Multiplies the current value of a grid point by the given the value, given in *real world coordinates*.
     * @param p Point.
     * @param value Value.
     * @return True on success
     */
    virtual bool multValue(const Point& p,
                           const T &value);


    /**
     * Set the data(value) of the cells corresponding to a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
     * If the expanded-line does not completely include a cell, the value of the cell will be updated proportionally to the overlaying area taking into account the cell's previous value
     * (e.g. if half of a cell (with initial value 1) must be updated with a value 0, the new value of the cell will be 0.5)
     * @param start Start-point of the line (in *real world coordinates*).
     * @param end End-point of the line (in *real world coordinates*).
     * @param width Width of the line .The line will be extended sideways a distance if width/2 (i.e. width/2 to each side).
     * @param value Value
     * @return True if successful
     */
    virtual bool updateLineProportionally(const Point& start,
                                          const Point& end,
                                          double width,
                                          const T &value,
                                          bool onlyCellsWithValue = false);

    /**
     * Adds to the the data(value) of the cells corresponding to a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
     * If the expanded-line does not completely include a cell, the value to be added to the cell will be updated proportionally to the overlaying area
     * (e.g. if half of a cell (with initial value 1) must be updated by adding a value 10, the new value of the cell will be 6)
     * @param start Start-point of the line (in *real world coordinates*).
     * @param end End-point of the line (in *real world coordinates*).
     * @param width Width of the line .The line will be extended sideways a distance if width/2 (i.e. width/2 to each side).
     * @param value Value to be added
     * @param be_precise If set to true, the values will be calculated precisely using area/intersection analysis
     * @return True if successful
     */
    virtual bool addToLine(const Point& start,
                           const Point& end,
                           double width,
                           const T &value,
                           bool onlyCellsWithValue = false,
                           bool be_precise = true,
                           const T &min_value = std::numeric_limits<T>::lowest(),
                           const T &max_value = std::numeric_limits<T>::max());

    /**
     * Set the data(value) of the cells corresponding to a polygon within the grid, given in *real world coordinates*.
     * If the polygon does not completely include a cell, the value of the cell will be updated proportionally to the overlaying area taking into account the cell's previous value
     * (e.g. if half of a cell (with initial value 0) must be updated with a value 9, the new value of the cell will be 0.5)
     * @param _poly polygone (in *real world coordinates*)
     * @param value New value
     * @return True if successful
     */
    virtual bool updatePolygonProportionally(const Polygon& _poly,
                                             const T & value,
                                             bool onlyCellsWithValue = false);

    /**
     * Updates the cells in the list with a new value.
     * The new value will be equal to the old_value*(mult-1)+(new_)value*mult, where the mult is obtained from the input vector cells.
     * @param cells List of cells (with multimpliers as cell.value) to be used
     * @param value New value of the cells.
     * @return True if successful
     */
    virtual bool updateCellsValue(const std::vector<GridmapLayout::GridCellOverlap>& cells,
                                  const T& value,
                                  bool onlyCellsWithValue = false);

    /**
     * Adds the corresponding value to the cells in the list
     * @param _value New value of the cells.
     * @param cells List of cells to be used
     * @param onlyCellsWithValue If true, if the cell has no value, it will remain without value;
     * @return True if successful
     */
    template<typename K = GridmapLayout::GridCell, typename = typename std::enable_if< std::is_base_of<GridmapLayout::GridCell, K>::value, void >::type>
    bool addCellsValue(const T& _value,
                               const std::vector<K>& cells,
                               bool onlyCellsWithValue = false );
    /**
     * Adds the corresponding value to the cells in the list
     * the cells.overlap will be used as multiplier of the given value
     * @param _value New value of the cells.
     * @param cells List of cells to be used
     * @param onlyCellsWithValue If true, if the cell has no value, it will remain without value;
     * @return True if successful
     */
    virtual bool addCellsValue(const T& _value,
                               const std::vector<GridmapLayout::GridCellOverlap>& cells,
                               bool onlyCellsWithValue = false );


    /**
     * Adds the corresponding value to the cells in the list spreading evenly based on the (overlaped) cell area and the (given) total area
     * @param _value New value of the cells.
     * @param cells List of cells to be used
     * @param onlyCellsWithValue If true, if the cell has no value, it will remain without value;
     * @return True if successful
     */
    template<typename K = GridmapLayout::GridCell, typename = typename std::enable_if< std::is_base_of<GridmapLayout::GridCell, K>::value, void >::type>
    bool addCellsValue(const T& _value,
                       double area,
                       const std::vector<K>& cells,
                       bool onlyCellsWithValue = false);

    virtual bool addCellsValue(const T& _value,
                               double area,
                               const std::vector<GridmapLayout::GridCellOverlap>& cells,
                               bool onlyCellsWithValue = false);

    /**
     * Adds the corresponding value to the cells in the list
     * @param cells List of cells and values to be used
     * @return True if successful
     */
    template<typename K = typename Gridmap<T>::GridCellInfo, typename = typename std::enable_if< std::is_base_of<typename Gridmap<T>::GridCellInfo, K>::value, void >::type>
    bool addCellsValue(const std::vector<K>& cells,
                               bool onlyCellsWithValue = false);



    /**
     * Multiplies the corresponding value to the cells in the list
     * @param _value New value of the cells.
     * @param cells List of cells to be used
     * @return True if successful
     */
    template<typename K = GridmapLayout::GridCell, typename = typename std::enable_if< std::is_base_of<GridmapLayout::GridCell, K>::value, void >::type>
    bool multCellsValue(const T& _value,
                               const std::vector<K>& cells);
    /**
     * Multiplies the corresponding value to the cells in the list
     * the cells.overlap will be used as multiplier of the given value
     * @param _value New value of the cells.
     * @param cells List of cells to be used
     * @return True if successful
     */
    virtual bool multCellsValue(const T& _value,
                                const std::vector<GridmapLayout::GridCellOverlap>& cells);


    /**
     * Multiplies the corresponding value to the cells in the list spreading evenly based on the (overlaped) cell area and the (given) total area
     * @param _value New value of the cells.
     * @param cells List of cells to be used
     * @return True if successful
     */
    template<typename K = GridmapLayout::GridCell, typename = typename std::enable_if< std::is_base_of<GridmapLayout::GridCell, K>::value, void >::type>
    bool multCellsValue(const T& _value,
                       double area,
                       const std::vector<K>& cells);

    virtual bool multCellsValue(const T& _value,
                               double area,
                               const std::vector<GridmapLayout::GridCellOverlap>& cells);

    /**
     * Multiplies the corresponding value to the cells in the list
     * @param cells List of cells and values to be used
     * @return True if successful
     */
    template<typename K = typename Gridmap<T>::GridCellInfo, typename = typename std::enable_if< std::is_base_of<typename Gridmap<T>::GridCellInfo, K>::value, void >::type>
    bool multCellsValue(const std::vector<K>& cell);


    /**
     * Intersects the grid values with a polygon (the values inside the polygon stay the same in the grid, whereas the cells outside the polygon are set to a given value)
     * @param _poly polygon (in *real world coordinates*)
     * @param valueOut Value to set in  the no-intersection area
     * @return True if successful
     */
    virtual bool intersectProportionaly( const T & valueOut,
                                         const Polygon& _poly);

    /**
     * Intersects the grid values with a polygon (the values inside the polygon stay the same in the grid, whereas the cells outside the polygon are set to no-value if it exixts)
     * @param _poly polygon (in *real world coordinates*)
     * @return True if successful
     */
    virtual bool intersectProportionaly( const Polygon& _poly);


//    //------------------------------------
//    //--------------GET-VALUE-------------
//    //------------------------------------


    /**
     * (Old implementation) Obtain the data(value) of a line, specified by the indexes of two *grid cells*, within the grid . The line is extended sideways a distance if width/2.
     * @param startX X-index of the first (start) cell of the line.
     * @param startY Y-index of first (start) cell of the line.
     * @param stopX X-index of the second (stop) cell of the line.
     * @param stopY Y-index of the second (stop) cell of the line.
     * @param width Width of the line .The line will be extended sideways a distance if width/2 (i.e. width/2 to each side).
     * @param _error_ (output) Indicates if there was an error in the calculation (e.g. index/point out of range).
     * @param value_type Type of desired return value
     * @return Computed data value
     */
    virtual long double getLineComputedValue(unsigned int startX,
                                             unsigned int startY,
                                             unsigned int stopX,
                                             unsigned int stopY,
                                             double width,
                                             ComputedValueType value_type = ComputedValueType::AVERAGE_TOTAL,
                                             bool *_error_ = nullptr) const;

    /**
     * (Old implementation) Obtain the data(value) of a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
     * @param start Start-point of the line (in *real world coordinates*).
     * @param stop End-point of the line (in *real world coordinates*).
     * @param width Width of the line .The line will be extended sideways a distance if width/2 (i.e. width/2 to each side).
     * @param _error_ (output) Indicates if there was an error in the calculation (e.g. index/point out of range).
     * @param value_type Type of desired return value
     * @return Computed data value
     */
    virtual long double getLineComputedValue2(const Point& start,
                                             const Point& stop,
                                             double width,
                                             ComputedValueType value_type = ComputedValueType::AVERAGE_TOTAL,
                                             bool *_error_ = nullptr) const;

    /**
     * Obtain the data(value) of a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
     * @param start Start-point of the line (in *real world coordinates*).
     * @param end End-point of the line (in *real world coordinates*).
     * @param width Width of the line .The line will be extended sideways a distance if width/2 (i.e. width/2 to each side).
     * @param _error_ (output) Indicates if there was an error in the calculation (e.g. index/point out of range).
     * @param be_precise If set to true, the values will be calculated precisely using area/intersection analysis
     * @param value_type Type of desired return value
     * @return Computed data value
     */
    virtual long double getLineComputedValue(const Point& start,
                                             const Point& end,
                                             double width,
                                             bool be_precise = false,
                                             ComputedValueType value_type = ComputedValueType::AVERAGE_TOTAL,
                                             bool *_error_ = nullptr) const;

    virtual long double getLineComputedValue_v1(const Point& start,
                                             const Point& end,
                                             double width,
                                             bool be_precise = false,
                                             ComputedValueType value_type = ComputedValueType::AVERAGE_TOTAL,
                                             bool *_error_ = nullptr) const;

    virtual long double getLineComputedValue_v2(const Point& start,
                                             const Point& end,
                                             double width,
                                             bool be_precise = false,
                                             ComputedValueType value_type = ComputedValueType::AVERAGE_TOTAL,
                                             bool *_error_ = nullptr) const;
    /**
     * Obtain the data(value) of a line within the grid, given in *real world coordinates*.
     * @param start Start-point of the line (in *real world coordinates*).
     * @param end End-point of the line (in *real world coordinates*).
     * @param _error_ (output) Indicates if there was an error in the calculation (e.g. index/point out of range).
     * @param value_type Type of desired return value
     * @return Computed data value
     */
    virtual long double getLineComputedValue(const Point& start,
                                             const Point& end,
                                             ComputedValueType value_type = ComputedValueType::AVERAGE_TOTAL,
                                             bool *_error_ = nullptr) const;

    /**
     * Obtain the data(value) of a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
     * @param start Start-point of the line (in *real world coordinates*).
     * @param end End-point of the line (in *real world coordinates*).
     * @param width Width of the line .The line will be extended sideways a distance if width/2 (i.e. width/2 to each side).
     * @param cells List of cells and (multiplier)values under the polygon
     * @param _error_ (output) Indicates if there was an error in the calculation (e.g. index/point out of range).
     * @param value_type Type of desired return value
     * @param checkForRepeatedCells If set to true, if will check if there are repeated cells in the list, so that the value is calculated only once per cell. If a cell is repeated, the (multiplier)value of the first cell (with valid multiplier-value) in the vector will be used
     * @return Computed data value
     */
    template<typename K = GridmapLayout::GridCellOverlap, typename = typename std::enable_if< std::is_base_of<GridmapLayout::GridCellOverlap, K>::value, void >::type>
    long double getLineComputedValue(const Point& start,
                                     const Point& end,
                                     double width,
                                     const std::vector<K>& cells,
                                     ComputedValueType value_type = ComputedValueType::AVERAGE_TOTAL,
                                     bool checkForRepeatedCells = false,
                                     bool *_error_ = nullptr) const;


    /**
     * Obtain the data(value) of a polygon within the grid, given in *real world coordinates*.
     * @param _poly polygon (in *real world coordinates*)
     * @param _error_ (output) Indicates if there was an error in the calculation (e.g. index/point out of range).
     * @param be_precise If set to true, the values will be calculated precisely using area/intersection analysis
     * @param value_type Type of desired return value
     * @return Computed data value
     */
    virtual long double getPolygonComputedValue(const Polygon& _poly,
                                                ComputedValueType value_type = ComputedValueType::AVERAGE_TOTAL,
                                                bool be_precise = false,
                                                bool *_error_ = nullptr) const;
    virtual long double getPolygonComputedValue_v1(const Polygon& _poly,
                                                   ComputedValueType value_type = ComputedValueType::AVERAGE_TOTAL,
                                                   bool be_precise = false,
                                                   bool *_error_ = nullptr) const;
    virtual long double getPolygonComputedValue_v2(const Polygon& _poly,
                                                   ComputedValueType value_type = ComputedValueType::AVERAGE_TOTAL,
                                                   bool be_precise = false,
                                                   bool *_error_ = nullptr) const;

    /**
     * Obtain the data(value) of a polygon within the grid, given in *real world coordinates*, given the list of cells under it
     * The value [0...1] in the GridCellInfo is taken as a multiplier to the value in the corresponding grid cell (e.g. if the cell value is 0.4 and the GridCellInfo (multiplier)value is 0.25, the calculated value for that cell will be 0.1)
     * If the (multiplier) value in the GridCellInfo is 0, that cell won't be taken into account in the AVERAGE_VALID_AND_SET calculation
     * If the (multiplier) value in the GridCellInfo does not lie within (0...1], that cell won't be taken into account in the calculation
     * @param _poly polygon (in *real world coordinates*)
     * @param cells List of cells and (multiplier)values under the polygon
     * @param _error_ (output) Indicates if there was an error in the calculation (e.g. index/point out of range).
     * @param value_type Type of desired return value
     * @param checkForRepeatedCells If set to true, if will check if there are repeated cells in the list, so that the value is calculated only once per cell. If a cell is repeated, the (multiplier)value of the first cell (with valid multiplier-value) in the vector will be used
     * @return Computed data value
     */
    template<typename K = GridmapLayout::GridCellOverlap, typename = typename std::enable_if< std::is_base_of<GridmapLayout::GridCellOverlap, K>::value, void >::type>
    long double getPolygonComputedValue(const Polygon& _poly,
                                        const std::vector<K>& cells,
                                        ComputedValueType value_type = ComputedValueType::AVERAGE_TOTAL,
                                        bool checkForRepeatedCells = false,
                                        bool *_error_ = nullptr) const;

    /**
     * Get the data(values) of the cells in the list
     * The value [0...1] in the GridCellInfo is taken as a multiplier to the value in the corresponding grid cell (e.g. if the cell value is 0.4 and the GridCellInfo (multiplier)value is 0.25, the calculated value for that cell will be 0.1)
     * If the (multiplier) value in the GridCellInfo does not lie within [0...1], that cell will be considered invalid
     * @param cells List of cells and (multiplier)values to be used
     * @param _error_ (output) Indicates if there was an error in the calculation (e.g. index/point out of range).
     * @param value_type Type of desired return value
     * @param area Area used for value_type AVERAGE_TOTAL and AVERAGE_VALID (AVERAGE_VALID_AND_SET depends on the valid and set cells). if area <= 0, AVERAGE_TOTAL and AVERAGE_VALID will depend on the given cells)
     * @param checkForRepeatedCells If set to true, if will check if there are repeated cells in the list, so that the value is calculated only once per cell. If a cell is repeated, the (multiplier)value of the first cell (with valid multiplier-value) in the vector will be used
     * @return Value of the cells in the list
     */
    template<typename K = GridmapLayout::GridCellOverlap, typename = typename std::enable_if< std::is_base_of<GridmapLayout::GridCellOverlap, K>::value, void >::type>
    long double getCellsComputedValue(const std::vector<K>& cells,
                                      ComputedValueType value_type = ComputedValueType::AVERAGE_TOTAL,
                                      double area = 0,
                                      bool checkForRepeatedCells = false,
                                      bool *_error_ = nullptr) const;

    /**
     * Get the sum of the values of the cells in the list
     * The value [0...1] in the GridCellInfo is taken as a multiplier to the value in the corresponding grid cell (e.g. if the cell value is 0.4 and the GridCellInfo (multiplier)value is 0.25, the calculated value for that cell will be 0.1)
     * @param cells List of cells and (multiplier)values to be used
     * @param _error_ (output) Indicates if there was an error in the calculation (e.g. index/point out of range).
     * @param checkForRepeatedCells If set to true, if will check if there are repeated cells in the list, so that the value is calculated only once per cell. If a cell is repeated, the (multiplier)value of the first cell (with valid multiplier-value) in the vector will be used
     * @return Value of the cells in the list
     */
    template<typename K = GridmapLayout::GridCellOverlap, typename = typename std::enable_if< std::is_base_of<GridmapLayout::GridCellOverlap, K>::value, void >::type>
    long double getCellsValueSum(const std::vector<K>& cells,
                                 bool checkForRepeatedCells = false,
                                 bool* _error_ = nullptr) const;


    /**
     * Get the data(values) of the cells in the list
     * @param cells List with the range of cells
     * @param _error_ (output) Indicates if there was an error in the calculation (e.g. index/point out of range).
     * @param value_type Type of desired return value
     * @return Value of the cells in the list
     */
    virtual long double getCellsComputedValue(const CellsRangeList& cells,
                                              bool average = true,
                                              bool *_error_ = nullptr) const;


    virtual bool getStatistics(GridStatistics& stats) const;

    virtual bool getStatistics(GridStatistics& stats, T filterMinValue, T filterMaxValue) const;


    //------------------------------------
    //-----------------IO-----------------
    //------------------------------------

    /**
     * Loads the grid from a file.
     * If the file extension is not given in the input file name, it will try to load from a .tif, a .tiff and a .png (in that order)
     * @param filename Name of the file to be read.
     * @param forceValueRange If set to true, values outside the internal value range will be approximated to the closest valid limit. If set to false, if any value lies outside the internal value range, the internal value range will be updated accordingly
     * @return True on success
     */
    virtual bool readGridFromFile(const std::string& filename);

    /**
     * Saves the grid as a PNG file (plus meta-file).
     * @param filename Name of the output file.
     * @return True on success
     */
    virtual bool saveGridAsPNG(const std::string& _filename, Range2RGBColorType colorType = Range2RGBColorType::COLOR_HEAT) const;

    /**
    * Gets the grid as a PNG ostringstream.
    * @param image_out output stringstream containing the png data.
    * @return True on success
    */
    virtual bool saveGridAsPNG(std::ostringstream &image_out, Range2RGBColorType colorType = Range2RGBColorType::COLOR_HEAT) const;

    /**
    * Gets the grid as a PNG ostream.
    * @param image_out output stream containing the png data.
    * @return True on success
    */
    virtual bool saveGridAsPNG(std::ostream &image_out, Range2RGBColorType colorType = Range2RGBColorType::COLOR_HEAT) const;

    /**
    * Loads the grid from a RGBA PNG file.
    * @param filename Name of the file to be read.
    * @param forceValueRange If set to true, values outside the internal value range will be approximated to the closest valid limit. If set to false, if any value lies outside the internal value range, the internal value range will be updated accordingly
    * @return True on success
    */
    virtual bool readGridFromPNG(const std::string& filename);

    /**
    * Gets the grid as a PNG ostringstream.
    * @param image_in c_string containing the png data.
    * @param size size of the image_in c_string containing the png data.
    * @param forceValueRange If set to true, values outside the internal value range will be approximated to the closest valid limit. If set to false, if any value lies outside the internal value range, the internal value range will be updated accordingly
    * @return True on success
    */
    virtual bool readGridFromPNG(std::istream &image_in,
                                double cellsize,
                                double dataMinX,
                                double dataMinY,
                                double minValue_meta,
                                double maxValue_meta,
                                Range2RGBColorType colorType,
                                Unit units);

    /**
    * Gets the grid as a PNG ostringstream.
    * @param image_in c_string containing the png data.
    * @param size size of the image_in c_string containing the png data.
    * @param forceValueRange If set to true, values outside the internal value range will be approximated to the closest valid limit. If set to false, if any value lies outside the internal value range, the internal value range will be updated accordingly
    * @return True on success
    */
    virtual bool readGridFromPNG(std::istream &image_in,
                                double dataMinX,
                                double dataMinY,
                                double dataMaxX,
                                double dataMaxY,
                                double minValue_meta,
                                double maxValue_meta,
                                Range2RGBColorType colorType,
                                Unit units);


    /**
    * Sets the grid as from a PNG input string.
    * @param image_in stream containing the png data.
    * @param size size of the image_in c_string containing the png data.
    * @param forceValueRange If set to true, values outside the internal value range will be approximated to the closest valid limit. If set to false, if any value lies outside the internal value range, the internal value range will be updated accordingly
    * @return True on success
    */
    virtual bool readGridFromPNG(const std::string& image_in,
                                 double cellsize,
                                 double dataMinX,
                                 double dataMinY,
                                 double minValue_meta,
                                 double maxValue_meta,
                                 Range2RGBColorType colorType,
                                 Unit units);

    /**
    * Sets the grid as from a PNG input string.
    * @param image_in stream containing the png data.
    * @param size size of the image_in c_string containing the png data.
    * @param forceValueRange If set to true, values outside the internal value range will be approximated to the closest valid limit. If set to false, if any value lies outside the internal value range, the internal value range will be updated accordingly
    * @return True on success
    */
    virtual bool readGridFromPNG(const std::string& image_in,
                                 double dataMinX,
                                 double dataMinY,
                                 double dataMaxX,
                                 double dataMaxY,
                                 double minValue_meta,
                                 double maxValue_meta,
                                 Range2RGBColorType colorType,
                                 Unit units);


    /**
    * Gets the grid as a PNG ostringstream.
    * @param image_in c_string containing the png data.
    * @param size size of the image_in c_string containing the png data.
    * @param forceValueRange If set to true, values outside the internal value range will be approximated to the closest valid limit. If set to false, if any value lies outside the internal value range, the internal value range will be updated accordingly
    * @return True on success
    */
    virtual bool readGridFromPNG(const char* image_in,
                                 unsigned int size,
                                 double cellsize,
                                 double dataMinX,
                                 double dataMinY,
                                 double minValue_meta,
                                 double maxValue_meta,
                                 Range2RGBColorType colorType,
                                 Unit units);

    /**
    * Gets the grid as a PNG ostringstream.
    * @param image_in c_string containing the png data.
    * @param size size of the image_in c_string containing the png data.
    * @param forceValueRange If set to true, values outside the internal value range will be approximated to the closest valid limit. If set to false, if any value lies outside the internal value range, the internal value range will be updated accordingly
    * @return True on success
    */
    virtual bool readGridFromPNG(const char* image_in,
                                unsigned int size,
                                double dataMinX,
                                double dataMinY,
                                double dataMaxX,
                                double dataMaxY,
                                double minValue_meta,
                                double maxValue_meta,
                                Range2RGBColorType colorType,
                                Unit units);

    /**
     * Saves the grid as a TIFF file.
     * @param filename Name of the output file.
     * @param utmZone UTM-zone.
     * @return True on success
     */
    virtual bool saveGridAsGeoTiff(const std::string& _filename, int utmZone=32) const;

    /**
     * Saves the grid as a GEOTIFF string.
     * @param data String containing the tiff data.
     * @param forceValueRange If set to true, values outside the internal value range will be approximated to the closest valid limit. If set to false, if any value lies outside the internal value range, the internal value range will be updated accordingly
     * @return True on success
     */
    virtual bool saveGridAsGeoTiffString(std::string& data, int utmZone=32) const;


    /**
     * Loads the grid from a TIFF file.
     * @param filename Name of the file to be read.
     * @param forceValueRange If set to true, values outside the internal value range will be approximated to the closest valid limit. If set to false, if any value lies outside the internal value range, the internal value range will be updated accordingly
     * @return True on success
     */
    virtual bool readGridFromGeoTiff(const std::string& filename) ;


    /**
     * Loads the grid from a GEOTIFF string.
     * @param data String containing the tiff data.
     * @param forceValueRange If set to true, values outside the internal value range will be approximated to the closest valid limit. If set to false, if any value lies outside the internal value range, the internal value range will be updated accordingly
     * @return True on success
     */
    virtual bool readGridFromGeoTiffString(const std::string& data, bool isInWGS = false) ;

    /**
     * Saves the grid as a PPM file (plus meta-file).
     * @param filename Name of the output file.
     * @return True on success
     */
    virtual bool saveGridAsPPM(const std::string& filename) const;

    /**
     * Loads the grid from a PPM file.
     * @param filename Name of the file to be read.
     * @param forceValueRange If set to true, values outside the internal value range will be approximated to the closest valid limit. If set to false, if any value lies outside the internal value range, the internal value range will be updated accordingly
     * @return True on success
     */
    virtual bool readGridFromPPM(const std::string& filename);

    /**
     * Saves the grid values in CSV matrix.
     * @param filename Name of the output file.
     * @return True on success
     */
    virtual bool saveValuesInCSV(const std::string& _filename, const std::string &sep = ";") const;

    /**
     * Checks if a given file exists.
     * @return True if it exists
     */
    static bool fileExists(const std::string& filename);


//    //------------------------------------
//    //----------------OTHERS--------------
//    //------------------------------------


protected:

    /**
     * Set value to a cell.
     * @param x X-index of the cell
     * @param y Y-index of the cell
     * @param _value Value
     * @return True on success
     */
    virtual bool set_internal(unsigned int x,
                              unsigned int y,
                              long double _value);

    /**
     * Set value (with limits) to a cell.
     * @param x X-index of the cell
     * @param y Y-index of the cell
     * @param _value Value
     * @param min Minimum value constrain
     * @param max Maximum value constrain
     * @return True on success
     */
    virtual bool set_internal(unsigned int x,
                              unsigned int y,
                              long double _value,
                              long double min,
                              long double max);

};

}
}

#include "arolib/cartography/impl/gridmap_numeric.tcc"

#endif

