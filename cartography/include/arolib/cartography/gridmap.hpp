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
 

#ifndef _AROLIB_GRIDMAP_HPP_
#define _AROLIB_GRIDMAP_HPP_

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
#include <functional>
#include <exception>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/algorithm/string.hpp>

#include "cellsrange.hpp"
#include "cellsrangeset.hpp"
#include "gridmap_layout.hpp"
#include "arolib/types/units.hpp"
#include "arolib/types/field.hpp"
#include "arolib/types/coordtransformer.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/has_operator_helper.hpp"

#include <gdal/gdal_priv.h>
#include <gdal/ogr_spatialref.h>
#include <gdal/gdalwarper.h>

#include <png++/image.hpp>
#include <png++/rgb_pixel.hpp>

namespace arolib {

namespace gridmap{


/**
 * @brief Base Gridmap class
 */
template <typename T>
class Gridmap : public LoggingComponent{

    using ValType = T;
    using TStore = std::unique_ptr<T>;
    using GridCell = GridmapLayout::GridCell;

public:

    using FuncGetNewValue = std::function< std::shared_ptr<T> //return: new value (nullptr -> clear cell value)
                                                (const std::shared_ptr<const T>&, //current value (nullptr if not set)
                                                  unsigned int x, unsigned int y) >;
    using FuncGetNewValueGCOverlap = std::function< std::shared_ptr<T> //return: new value (nullptr -> clear cell value)
                                                        (const std::shared_ptr<const T>&, //current value (nullptr if not set)
                                                         const GridmapLayout::GridCellOverlap&) >;

    /**
     * @brief Holds cell's data
     */
    struct GridCellInfo : public GridmapLayout::GridCell{
        std::unique_ptr<T> value = nullptr; /**< Value/data (if nullptr -> no value/data) */

        /**
         * @brief Constructor
         */
        GridCellInfo() = default;

        /**
         * @brief Constructor
         * @param _x Cell x-index
         * @param _y Cell y-index
         * @param _value Cell value (if nullptr -> no value/data)
         */
        GridCellInfo(int _x, int _y, const T* _value):GridmapLayout::GridCell(_x, _y){
            if(_value) value = std::unique_ptr<T>(new T ( *_value ));
        }

        /**
         * @brief Constructor
         * @param _x Cell x-index
         * @param _y Cell y-index
         * @param _value Cell value
         */
        GridCellInfo(int _x, int _y, const T& _value):GridCellInfo(_x, _y, &_value){}

        /**
         * @brief operator==. Check if two cells vahe the same value
         * @param other Other
         * @return True if they have the same value
         */
        bool operator==(const GridCellInfo &other) const{
            if(!value && !other.value){}
            else if(!value || !other.value || !areEqual<T>(*value, *other.value))
                return false;

            return GridmapLayout::GridCell::operator ==(other);
        }

        /**
         * @brief Set the value/data of a cell
         * @param _value Cell value
         */
        void setValue(const T& _value) { value = std::unique_ptr<T>(new T ( _value )); }

        /**
         * @brief Clear the value/data of a cell (--> no data)
         */
        void clearValue() { value = nullptr; }

    private:

        /**
         * @brief Used to check if two real (floating) values are considered equal
         */
        template< typename T2 = T >
        static typename std::enable_if< std::is_floating_point<T2>::value, bool >::type
        areEqual( const T2& v1, const T2& v2 ){
            return std::fabs(v1 - v2) < 1e-10;
        }

        /**
         * @brief Used to check if two non-real (non-floating) values are considered equal
         */
        template< typename T2 = T >
        static typename std::enable_if< !std::is_floating_point<T2>::value, bool >::type
        areEqual( const T2& v1, const T2& v2 ){
            return v1 == v2;
        }
    };

     /**
     * Constructor.
     *
     * Already allocates the grid with the given size if the layout is valid.
     * @param lo Gridmap geometric layout.
     * @param logLevel Log level
     */
    explicit Gridmap(const GridmapLayout &lo, LogLevel logLevel = LogLevel::INFO);

    /**
     * Constructor.
     *
     * The grid is not allocated yet (it doesn't exist).
     * @param logLevel Log level
     */
    explicit Gridmap(LogLevel logLevel = LogLevel::INFO);

    /**
     * Copy constructor.
     * @param other Other gridmap.
     */
    Gridmap(const Gridmap<T>& other);

    /**
     * Destructor.
     */
    virtual ~Gridmap();

    //------------------------------------
    //--------------OPERATORS-------------
    //------------------------------------

    /**
    * Copy assignment
    * @param other Other grid.
    */
    virtual Gridmap<T>& operator=(const Gridmap<T>& other);

    /**
    * Operator ==.
    * @param other Other grid.
    */
    template<typename K, typename = typename std::enable_if< has_is_equal_method<K>::value && std::is_same<T, K>::value, void >::type>
    bool operator==(const Gridmap<K>& other) const;

    /**
    * Check if the grid is equal to another grid.
    * @param other Other grid.
    * @param isDataEqual function to compare cell data.
    * @return True if equal.
    */
    bool equals(const Gridmap<T>& other, const std::function<bool(const T&, const T&)>& isDataEqual ) const;

    /**
    * Check if the grid has the same geometry (bounding box and location) as another grid.
    * @param other Other grid.
    * @return True if same geometry.
    */
    virtual bool equalGeometry(const Gridmap<T>& other) const;


    //------------------------------------
    //--------------SET-PARAM-------------
    //------------------------------------
    /**
     * De-allocates the grid.
     * @param resetAllParameters If set to true, all the internal parameters (cellsize, min/max data-values, etc) will be reset to the default values; if set to false, only the parameters related to the grid size and allocation are reset.
     */
    virtual void destroy();

    /**
     * (Re) Creates a new grid by setting the limits and resolution allocating the needed memory. If the grid was allocated, it is destroyed.
     * @param minX Minimum x-axis position (in real world coordinates).
     * @param maxX Maximum x-axis position (in real world coordinates).
     * @param minY Minimum y-axis position (in real world coordinates).
     * @param maxY Maximum y-axis position (in real world coordinates).
     * @param cellsize Resolution.
     * @param cellValue [optional] Value to set the cells (if nullptr -> noData or 0).
     * @return True on success
     */
    virtual bool createGrid(double minX,
                            double maxX,
                            double minY,
                            double maxY,
                            double cellsize,
                            const T* cellValue = nullptr);

    /**
     * (Re) Creates a new grid by setting the limits and resolution allocating the needed memory. If the grid was allocated, it is destroyed.
     * @param minX Minimum x-axis position (in real world coordinates).
     * @param maxX Maximum x-axis position (in real world coordinates).
     * @param minY Minimum y-axis position (in real world coordinates).
     * @param maxY Maximum y-axis position (in real world coordinates).
     * @param cellsize Resolution.
     * @param cellValue Value to set the cells
     * @return True on success
     */
    virtual inline bool createGrid(double minX,
                                   double maxX,
                                   double minY,
                                   double maxY,
                                   double cellsize,
                                   const T& cellValue){ return createGrid(minX, maxX, minY, maxY, cellsize, &cellValue); }

    /**
     * (Re) Creates a new grid based on a given gridmap geometric layout and initialized all cell values/data to a given value.
     * @param lo Gridmap geometric layout.
     * @param cellValue [optional] Value to set the cells (if nullptr -> noData or 0).
     * @return True on success
     */
    virtual bool createGrid(const GridmapLayout& lo,
                            const T* cellValue = nullptr);

    /**
     * (Re) Creates a new grid based on a given gridmap geometric layout and initialized all cell values/data to a given value.
     * @param lo Gridmap geometric layout.
     * @param cellValue Value to set the cells.
     * @return True on success
     */
    virtual inline bool createGrid(const GridmapLayout& lo,
                                   const T& cellValue){ return createGrid(lo, &cellValue); }

    /**
     * (Re) Creates a new grid by copying the data from another gridmap.
     * @param other Other gridmap
     * @param clearValues If true, the cell values will not be copied and all cell will have no-value
     * @return True on success
     */
    virtual bool copyFrom(const Gridmap<T>& other, bool clearValues);

    /**
     * (Re) Set the lower limits (in real world coordinates) of the x- and y- axis of the grid, and recalculates the upper axis limits. No Allocation is done.
     * @param minX Minimum x-axis position (in real world coordinates).
     * @param minY Minimum y-axis position (in real world coordinates).
     * @param cellsize Cellsize (resolution). If <= 0, it uses the internal cellsize.
     * @return True if resulting layout is valid
     */
    virtual bool setPointLimits_min(double minX, double minY, double cellsize = 0);

    /**
     * Creates the grid from a given polygon.
     * Cells that lie outside the polygone are set equal to 0
     * @param boundary Points of the polygon.
     * @param resolution Resolution (cell size) given in meters.
     * @param _val [optional] Value to set the cells that lie inside the polygon. (if nullptr -> noData or 0)
     * @param only_perimeter If set to true, only the cells in touch with the perimeter of the boundary will be set with the given value.
     * @return True on success
     */
    virtual bool convertPolygonToGrid(const std::vector<Point>& boundary,
                                      double resolution,
                                      const T *_val = nullptr,
                                      bool only_perimeter = false);

    /**
     * Creates the grid from a given polygon.
     * Cells that lie outside the polygone are set equal to 0
     * @param boundary Points of the polygon.
     * @param resolution Resolution (cell size) given in meters.
     * @param _val Value to be set to the cells that lie inside the polygon.
     * @param only_perimeter If set to true, only the cells in touch with the perimeter of the boundary will be set with the given value.
     * @return True on success
     */
    virtual inline bool convertPolygonToGrid(const std::vector<Point>& boundary,
                                             double resolution,
                                             const T &_val,
                                             bool only_perimeter = false){
        return convertPolygonToGrid(boundary, resolution, &_val, only_perimeter);
    }


    /**
     * Takes the current grid and expands it (if necesary) to include the given polygon.
     * The values of the new cells that lie inside the polygon will be set to the desired input value.
     * @param boundary Points of the polygon.
     * @param _val [optional] Value to be set to the new cells that lie inside the polygon. (if nullptr -> noData or 0)
     * @param only_perimeter If set to true, only the added-cells in touch with the perimeter of the boundary will be set with the given value.
     * @param reduceToPolygon If set to true, the parts of the grid that do not overlap with the polygon will be deleted.
     * @return True on success
     */
    virtual bool expandGridFromPolygon(const std::vector<Point>& boundary,
                                       const T *_val = nullptr,
                                       bool only_perimeter = false,
                                       bool reduceToPolygon = true);

    /**
     * Takes the current grid and expands it (if necesary) to include the given polygon.
     * The values of the new cells that lie inside the polygon will be set to the desired input value.
     * @param boundary Points of the polygon.
     * @param _val Value to be set to the new cells that lie inside the polygon.
     * @param only_perimeter If set to true, only the added-cells in touch with the perimeter of the boundary will be set with the given value.
     * @param reduceToPolygon If set to true, the parts of the grid that do not overlap with the polygon will be deleted.
     * @return True on success
     */
    virtual inline bool expandGridFromPolygon(const std::vector<Point>& boundary,
                                              const T &_val,
                                              bool only_perimeter = false,
                                              bool reduceToPolygon = true){
        return expandGridFromPolygon(boundary, &_val, only_perimeter, reduceToPolygon);
    }

    /**
     * Takes the current grid and reduces it (if necesary) so that only cells that lie under the polygon are kept.
     * The values of the new cells that lie inside the polygon will be set to the desired input value.
     * @param boundary Points of the polygon.
     * @param expandToPolygon If set to true, the current grid and expands it (if necesary) to include all given polygon.
     * @param _val [optional] Value to be set to the new cells that lie inside the polygon (in case expandToPolygon is true). (if nullptr -> noData or 0)
     * @param only_perimeter If set to true (and expandToPolygon == true), only the added-cells in touch with the perimeter of the boundary will be set with the given value.
     * @return True on success
     */
    virtual bool reduceGridToPolygon(const std::vector<Point>& boundary,
                                     bool  expandToPolygon = true,
                                     const T *_val = nullptr,
                                     bool only_perimeter = false);

    /**
     * Takes the current grid and reduces it (if necesary) so that only cells that lie under the polygon are kept.
     * The values of the new cells that lie inside the polygon will be set to the desired input value.
     * @param boundary Points of the polygon.
     * @param expandToPolygon If set to true, the current grid and expands it (if necesary) to include all given polygon.
     * @param _val Value to be set to the new cells that lie inside the polygon (in case expandToPolygon is true).
     * @param only_perimeter If set to true (and expandToPolygon == true), only the added-cells in touch with the perimeter of the boundary will be set with the given value.
     * @return True on success
     */
    virtual inline bool reduceGridToPolygon(const std::vector<Point>& boundary,
                                            bool  expandToPolygon = true,
                                            const T &_val = T(),
                                            bool only_perimeter = false){
        return reduceGridToPolygon(boundary, expandToPolygon, &_val, only_perimeter);
    }


    /**
     * Increase/decrease the resolution
     * @param scale If > 0, the new_resolution = old_resolution * scale ; if < 0, the new_resolution = old_resolution * (-1/scale).
     * @return True on success
     */
    virtual bool scaleResolution(int scale);

    /**
     * Set the units of the gridmap
     * @param units Units
     */
    virtual void setUnits(Unit units);

    /**
     * Sets the layout tolerance (in real world coordinates) to accept a value out of the X- and Y- axis range.
     * @param tolerance Tolerance
     * @return True True on success
     */
    virtual bool setLayoutTolerance(double tolerance);

    /**
     * Sets the option to make computations in multiple threads.
     * @param val value
     */
    virtual void setComputeInMultiThread(bool val);

    //------------------------------------
    //--------------SET-VALUE-------------
    //------------------------------------


    /**
     * Set the data(value) of a cell within the grid, specified by the cell's x- and y- indexes.
     * @param x X-index of the cell.
     * @param y Y-index of the cell.
     * @param _value Value/data (if nullptr -> noData or 0).
     * @return True on success
     */
    virtual bool setValue(unsigned int x,
                          unsigned int y,
                          const T* _value);

    /**
     * Set the data(value) of a cell within the grid, specified by the cell's x- and y- indexes.
     * @param x X-index of the cell.
     * @param y Y-index of the cell.
     * @param _value Value/data.
     * @return True on success
     */
    virtual inline bool setValue(unsigned int x,
                                 unsigned int y,
                                 const T& _value){ return setValue(x, y, &_value); }

    /**
     * Set the data(value) of a grid point, given in *real world coordinates*.
     * @param p Point.
     * @param value Value/data (if nullptr -> noData or 0)
     * @return True on success
     */
    virtual bool setValue(const Point& p,
                          const T* value);

    /**
     * Set the data(value) of a grid point, given in *real world coordinates*.
     * @param p Point.
     * @param value Value/data.
     * @return True on success
     */
    virtual inline bool setValue(const Point& p, const T& value){ return setValue(p, &value); }


    /**
     * Set the data(value) of a cell within the grid to 'noDataValue', specified by the cell's x- and y- indexes.
     * @param x X-index of the cell.
     * @param y Y-index of the cell.
     * @return True on success
     */
    virtual bool setNoValue(unsigned int x,
                            unsigned int y);

    /**
     * Set the data(value) of a grid point to 'noDataValue', given in *real world coordinates*.
     * @param p Point.
     * @return True on success
     */
    virtual bool setNoValue(const Point& p);


    /**
     * Set the data(value) of a all cells.
     * @param _value Value/data (if nullptr -> noData or 0).
     * @return True on success
     */
    virtual bool setAllValues(const T* _value);

    /**
     * Set the data(value) of a all cells.
     * @param_value Value/data.
     * @return True on success
     */
    virtual inline bool setAllValues(const T& value){ return setAllValues(&value); }

    /**
     * Unset the data(value) of a all cells.
     * @return True on success
     */
    virtual inline bool unsetAllValues(){
        return setAllValues(nullptr);
    }

    /**
     * Sets a new cell value based on the given callback function.
     * @param x X-index of the cell.
     * @param y Y-index of the cell.
     * @param funcGetNewValue Callback function to get the new value based on the current one
     * @return True on success
     */
    virtual bool setValue(unsigned int x,
                          unsigned int y,
                          const FuncGetNewValue& funcGetNewValue);

    /**
     * Sets a new cell value based on the given callback function.
     * @param p Point.
     * @param funcGetNewValue Callback function to get the new value based on the current one
     * @return True on success
     */
    virtual bool setValue(const Point& p,
                          const FuncGetNewValue& funcGetNewValue);

    /**
     * Set the data(value) of the cells corresponding to a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
     * @param startX X-index of the start-point of the line.
     * @param startY Y-index of the start-point of the line.
     * @param stopX X-index of the end-point of the line.
     * @param stopY Y-index of the end-point of the line.
     * @param width Width of the line .The line will be extended sideways a distance if width/2 (i.e. width/2 to each side).
     * @param _val New value/data (if nullptr -> noData or 0)
     * @return True if successful
     */
    virtual bool setLine(unsigned int startX,
                         unsigned int startY,
                         unsigned int stopX,
                         unsigned int stopY,
                         double width,
                         const T *_val);

    /**
     * Set the data(value) of the cells corresponding to a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
     * @param startX X-index of the start-point of the line.
     * @param startY Y-index of the start-point of the line.
     * @param stopX X-index of the end-point of the line.
     * @param stopY Y-index of the end-point of the line.
     * @param width Width of the line .The line will be extended sideways a distance if width/2 (i.e. width/2 to each side).
     * @param _val New value/data
     * @return True if successful
     */
    virtual inline bool setLine(unsigned int startX,
                                unsigned int startY,
                                unsigned int stopX,
                                unsigned int stopY,
                                double width,
                                const T &_val){
        return setLine(startX, startY, stopX, stopY, width, &_val);
    }

    /**
     * Set the data(value) of the cells corresponding to a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
     * @param start Start-point of the line (in *real world coordinates*).
     * @param end End-point of the line (in *real world coordinates*).
     * @param width Width of the line .The line will be extended sideways a distance if width/2 (i.e. width/2 to each side).
     * @param _val New value/data (if nullptr -> noData or 0)
     * @return True if successful
     */
    virtual bool setLine2(const Point& start,
                             const Point& end,
                             double width,
                             const T *_val);

    /**
     * Set the data(value) of the cells corresponding to a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
     * @param start Start-point of the line (in *real world coordinates*).
     * @param end End-point of the line (in *real world coordinates*).
     * @param width Width of the line .The line will be extended sideways a distance if width/2 (i.e. width/2 to each side).
     * @param _val New value/data
     * @return True if successful
     */
    virtual inline bool setLine2(const Point& start,
                             const Point& end,
                             double width,
                             const T &_val){
        return setLine2(start, end, width, &_val);
    }

    /**
     * Set the data(value) of the cells corresponding to a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
     * If the expanded-line does not completely include a cell, the value of the cell will be updated if the relative overlaying area is > than overlapThreshold
     * @param start Start-point of the line (in *real world coordinates*).
     * @param end End-point of the line (in *real world coordinates*).
     * @param width Width of the line .The line will be extended sideways a distance if width/2 (i.e. width/2 to each side).
     * @param _val Value (if nullptr -> noData or 0)
     * @param overlapThreshold If > 0 (0,1), the overlapping cells will be calculated precisely using area/intersection analysis (cell will be set if overlap > overlapThreshold)
     * @return True if successful
     */
    virtual bool setLine(const Point& start,
                         const Point& end,
                         double width,
                         const T *_val,
                         float overlapThreshold = -1);

    /**
     * Set the data(value) of the cells corresponding to a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
     * If the expanded-line does not completely include a cell, the value of the cell will be updated if the relative overlaying area is > than overlapThreshold
     * @param start Start-point of the line (in *real world coordinates*).
     * @param end End-point of the line (in *real world coordinates*).
     * @param width Width of the line .The line will be extended sideways a distance if width/2 (i.e. width/2 to each side).
     * @param _val Value
     * @param overlapThreshold If > 0 (0,1), the overlapping cells will be calculated precisely using area/intersection analysis (cell will be set if overlap > overlapThreshold)
     * @return True if successful
     */
    virtual inline bool setLine(const Point& start,
                                 const Point& end,
                                 double width,
                                 const T &_val,
                                 float overlapThreshold = -1){
        return setLine(start, end, width, &_val, overlapThreshold);
    }

    /**
     * Sets the cells in the list with the corresponding value
     * @param _value New value of the cells (if nullptr -> noData or 0).
     * @param cells List of cells to be used (the values on the list elements will be disregarded)
     * @return True if successful
     */
    template<typename K = GridCell, typename = typename std::enable_if< std::is_base_of<GridCell, K>::value, void >::type>
    bool setCellsValue(const T* _value,
                       const std::vector<K>& cells);


    /**
     * Sets the cells in the list with the corresponding value
     * @param _value New value of the cells.
     * @param cells List of cells to be used (the values on the list elements will be disregarded)
     * @return True if successful
     */
    template<typename K = GridCell, typename = typename std::enable_if< std::is_base_of<GridCell, K>::value, void >::type>
    inline bool setCellsValue(const T& _value,
                              const std::vector<K>& cells){ return setCellsValue(&_value, cells); }

    /**
     * Sets the cells in the list with the value obtained from the callback function
     * @param cells List of cells to be used (the values on the list elements will be disregarded)
     * @param funcGetNewValue Callback function to get the new value based on the current one
     * @return True if successful
     */
    template<typename K = GridCell, typename = typename std::enable_if< std::is_base_of<GridCell, K>::value, void >::type>
    bool setCellsValue(const std::vector<K>& cells,
                       const FuncGetNewValue& funcGetNewValue);

    /**
     * Sets the cells in the list with the value obtained from the callback function
     * @param cells List of cells (incl. overlap) to be used
     * @param funcGetNewValue Callback function to get the new value based on the current one
     * @return True if successful
     */
    virtual bool setCellsValue(const std::vector<GridmapLayout::GridCellOverlap>& cells,
                               FuncGetNewValueGCOverlap funcGetNewValue);

    /**
     * Sets the cells in the list with the corresponding value
     * @param cells List of cells to be used (the values on the list elements will be set)
     * @return True if successful
     */
    template<typename K = GridCellInfo, typename = typename std::enable_if< std::is_base_of<GridCellInfo, K>::value, void >::type>
    bool setCellsValue(const std::vector<K>& cells);

    /**
     * Set the data(value) of the cells corresponding to a polygon within the grid, given in *real world coordinates*.
     * If the polygon does not completely include a cell, the value of the cell will be updated proportionally to the overlaying area taking into account the cell's previous value
     * (e.g. if half of a cell (with initial value 0) must be updated with a value 9, the new value of the cell will be 0.5)
     * @param _poly polygone (in *real world coordinates*)
     * @param value New value/data (if nullptr -> noData or 0)
     * @param overlapThreshold If > 0 (0,1), the overlapping cells will be calculated precisely using area/intersection analysis (cell will be set if overlap > overlapThreshold)
     * @return True if successful
     */
    virtual bool setPolygon(const Polygon& _poly,
                            const T * value,
                            float overlapThreshold = -1);

    /**
     * Set the data(value) of the cells corresponding to a polygon within the grid, given in *real world coordinates*.
     * If the polygon does not completely include a cell, the value of the cell will be updated proportionally to the overlaying area taking into account the cell's previous value
     * (e.g. if half of a cell (with initial value 0) must be updated with a value 9, the new value of the cell will be 0.5)
     * @param _poly polygone (in *real world coordinates*)
     * @param value New value/data
     * @param overlapThreshold If > 0 (0,1), the overlapping cells will be calculated precisely using area/intersection analysis (cell will be set if overlap > overlapThreshold)
     * @return True if successful
     */
    virtual inline bool setPolygon(const Polygon& _poly,
                                   const T & value,
                                   float overlapThreshold = -1){ return setPolygon(_poly, &value, overlapThreshold); }

    /**
     * Intersects the grid values with a polygon (the values inside the polgon stay the same in the grid, whereas the cells outside the polygon are set to a given value)
     * @param _poly polygon (in *real world coordinates*)
     * @param valueOut Value to set in  the no-intersection area (if nullptr -> noData or 0)
     * @param overlapThreshold If > 0 (0,1), the overlapping cells will be calculated precisely using area/intersection analysis (cell will be set if overlap > overlapThreshold)
     * @return True if successful
     */
    virtual bool intersect( const Polygon& _poly,
                            const T * valueOut = nullptr,
                            float overlapThreshold = -1 );

    /**
     * Intersects the grid values with a polygon (the values inside the polgon stay the same in the grid, whereas the cells outside the polygon are set to a given value)
     * @param _poly polygon (in *real world coordinates*)
     * @param valueOut Value to set in  the no-intersection area
     * @param overlapThreshold If > 0 (0,1), the overlapping cells will be calculated precisely using area/intersection analysis (cell will be set if overlap > overlapThreshold)
     * @return True if successful
     */
    virtual inline bool intersect( const Polygon& _poly,
                                   const T & valueOut,
                                   float overlapThreshold = -1 ){ return intersect(_poly, &valueOut, overlapThreshold); }

    //------------------------------------
    //--------------GET-PARAM-------------
    //------------------------------------

    /**
     * Check if the grid is allocated (the grid exists).
     * @return True if the grid is allocated
     */
    virtual bool isAllocated() const;

    /**
     * Get geometric layout.
     * @return layout
     */
    virtual const GridmapLayout& getLayout() const;

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
    virtual double getCellsize () const;

    /**
     * Get the area of a grid cell (in m²).
     * @return Area of a grid cell
     */
    virtual double getCellArea () const;

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

    virtual Unit getUnits() const;

    /**
     * Returns the layout tolerance (in real world coordinates) to accept a value out of the X- and Y- axis range.
     * @return Tolerance
     */
    virtual double getLayoutTolerance() const;


    /**
     * Returns the ComputeInMultiThread setting.
     * @return ComputeInMultiThread setting
     */
    bool getComputeInMultiThread() const;


    //------------------------------------
    //--------------GET-VALUE-------------
    //------------------------------------

    /**
    * Check if a cell has value, specified by the cell's x- and y- indexes.
    * @param x X-index of the cell.
    * @param y Y-index of the cell.
    * @param [out] _error_ Indicates if there was an error in the calculation (e.g. index/point out of range).
    * @return True if cell has value
    */
    virtual bool hasValue(unsigned int x,
                          unsigned int y,
                          bool* _error_) const;

    virtual bool hasValue(unsigned int x,
                          unsigned int y) const;

    /**
    * Check if a cell has value, specified by the cell's x- and y- indexes.
    * @param p Point
    * @param [out] _error_ Indicates if there was an error in the calculation (e.g. index/point out of range).
    * @return True if cell has value
    */
    virtual bool hasValue(const Point& p,
                          bool* _error_ = nullptr) const;


    /**
     * Obtain the data(value) of a cell within the grid, specified by the cell's x- and y- indexes.
     * @param x X-index of the cell.
     * @param y Y-index of the cell.
     * @param [out] _error_ Indicates if there was an error in the calculation (e.g. index/point out of range).
     * @return Cell's value/data
     */
    virtual T getValue(unsigned int x,
                       unsigned int y,
                       bool* _error_ = nullptr) const;

    /* Returns value (x,y) if it exists, otherwise an exception is thrown (required for pyarolib) */
    virtual T getValueThrows(unsigned int x,
                            unsigned int y) const;

    /**
     * Obtain the data(value) of a grid point, given in *real world coordinates*.
     * @param p Point.
     * @param [out] _error_ Indicates if there was an error in the calculation (e.g. index/point out of range).
     * @return Cell's value/data
     */
    virtual T getValue(const Point& p,
                       bool* _error_ = nullptr) const;

    /**
    * Get the indexes ranges of the cells that are in touch with the line.
    * @param start Start-point of the line (in *real world coordinates*).
    * @param end End-point of the line (in *real world coordinates*).
    * @param simple Simple computation?
    * @return A CellsRangeList used to obtain the range of columns (X-indexes) touched by the line, and the range of rows (Y-idexes) corresponding to each column
    */
    virtual CellsRangeList getCellsUnderLine(const Point& start, const Point& end, bool simple) const;

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
     * @return A vetor of GridCellInfo's, where the value (between 0 and 1) of the GridCellInfo corresponds to the amount of the cell that is under the extended line (e.g. if the extended line only covers half of the cell's area, the value will be 0.5)
     */
    virtual std::vector<GridmapLayout::GridCellOverlap> getCellsOverlapUnderLine(const Point& start,
                                                                              const Point& end,
                                                                              double width,
                                                                              const Polygon& boundary = Polygon()) const;

    /**
     * Get the indexes ranges of the cells that are in touch with the polygon.
     * @param _poly polygon (in *real world coordinates*)
     * @return A CellsRangeList used to obtain the range of columns (X-indexes) touched by the polygon, and the range of rows (Y-idexes) corresponding to each column
     */
    virtual CellsRangeList getCellsUnderPolygon(const Polygon& _poly) const; //@TODO it was tested only with rectangles

    /**
     * Get the indexes of the cells that are in touch with the polygon, as well as how much of the cell is under the polygon.
     * @param _poly polygone (in *real world coordinates*)
     * @param boundary Check for proportion inside the boundary. No taken into account if empty.
     * @return A vetor of GridCellOverlap's, where the value (between 0 and 1) of the GridCellOverlap corresponds to the amount of the cell that is under the polygon (e.g. if the polygon only covers half of the cell's area, the value will be 0.5)
     */
    virtual std::vector< GridmapLayout::GridCellOverlap > getCellsOverlapUnderPolygon(const Polygon& _poly,
                                                                                   const Polygon& boundary = Polygon()) const;
    /**
     * Obtain the coordinates coresponding to the center given input cell.
     * @param cellX X-index of the desired cell.
     * @param cellY Y-index of the desired cell.
     * @param corner Point corresponding to the cell's center.
     * @return True if successful (cell in range)
     */
    virtual bool getCellCenter (unsigned int cellX, unsigned int cellY, Point &center) const;

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
     * @param [out] _error_ Indicates if there was an error in the calculation (e.g. index/point out of range).
     * @return The arolib::polygon coresponding to the grid (in real world coordinates).
     */
    virtual Polygon getGridPolygon(bool * _error_ = nullptr) const;

    /**
     * Obtain the area of the grid.
     * @param [out] _error_ Indicates if there was an error in the calculation (e.g. index/point out of range).
     * @return Area.
     */
    virtual double getGridArea(bool * _error_ = nullptr) const;

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

    //------------------------------------
    //----------PROTECTED METHODS---------
    //------------------------------------

    /**
     * Creates a new grid by allocating the needed memory.
     * @param x Number of columns of the grid.
     * @param y Number of rows of the grid.
     * @return True on success
     */
    virtual bool allocate();

     /**
      * Set directly the data(value) of a cell within the grid, specified by the cell's x- and y- indexes.
      * @param x X-index of the cell.
      * @param y Y-index of the cell.
      * @param _value Value/data (if nullptr -> noData or 0).
      * @return True on success
      */
    virtual bool set(unsigned int x,
                     unsigned int y,
                     const T *_value);

    /**
     * Set directly the data(value) of a cell within the grid, specified by the cell's x- and y- indexes.
     * @param x X-index of the cell.
     * @param y Y-index of the cell.
     * @param _value Value/data.
     * @return True on success
     */
    virtual inline bool set(unsigned int x,
                            unsigned int y,
                            const T &_value) { return set(x, y, &_value); }

    /**
     * Unset (value = no data value) directly the data(value) of a cell within the grid, specified by the cell's x- and y- indexes.
     * @param x X-index of the cell.
     * @param y Y-index of the cell.
     * @return True on success
     */
    virtual bool unSet(unsigned int x, unsigned int y);


    /**
     * Check if a cell has value/data.
     * @param x X-index of the cell.
     * @param y Y-index of the cell.
     * @return True if set
     */
    virtual bool isSet(unsigned int x, unsigned int y) const;


     /**
      * Obtain directly the data(value) of a cell within the grid, specified by the cell's x- and y- indexes.
      * @param x X-index of the cell.
      * @param y Y-index of the cell.
      * @param [out] _error_ Indicates if there was an error in the calculation (e.g. index/point out of range).
      * @return Cell's value/data
      */
     virtual T get(unsigned int x,
                   unsigned int y) const;


    /**
     * Check if two cells (i.e. the store values) are considered equal.
     * @param v1 Store value 1
     * @param v2 Store value 2
     * @return True if equal
     */
    static bool isEqual(const std::unique_ptr<T>& v1, const std::unique_ptr<T>& v2, const std::function<bool(const T &, const T &)> &isDataEqual);


    /**
     * Checks that the internal parameters (is_allocated, grid size, cellsize, min/max coordinates,...) are correct to perform operations in the grid.
     * @param printError If set to true, and in case of an error, an output message will be printed with the respective error.
     * @param function (optional) Name of the function that called the check.
     * @return True if all parameters are correct.
     */
    virtual bool checkInternalParameters(bool printError = true, const std::string & function = "") const;


protected:
    GridmapLayout m_layout; /**< Contains the geometric layout parameters of the gridmap */
    TStore** m_grid; /**< Values' matrix */
    bool  m_allocated;  /**< Has the space for the matrix been allocated? (i.e. does a grid already exist?) */
    Unit m_units = Unit::UNIT_CUSTOM; /**< Gridmap units */
    bool m_computeInMultiThread = true; /**< Make computations in multiple threads? */
};

}
}

#include "arolib/cartography/impl/gridmap.tcc"

#endif //_AROLIB_GRIDMAP_HPP_

