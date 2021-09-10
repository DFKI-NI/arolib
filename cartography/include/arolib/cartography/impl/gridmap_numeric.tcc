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
 

#ifndef _AROLIB_GRIDMAP_NUMERIC_TCC_
#define _AROLIB_GRIDMAP_NUMERIC_TCC_


#include "arolib/cartography/gridmap_numeric.hpp"

namespace arolib {
namespace gridmap {

using namespace arolib::geometry;

template<typename T>
typename NumericGridmap<T>::ComputedValueType NumericGridmap<T>::intToComputedValueType(int value){
    if(value == ComputedValueType::SUM)
        return ComputedValueType::SUM;
    else if(value == ComputedValueType::AVERAGE_VALID)
        return ComputedValueType::AVERAGE_VALID;
    else if(value == ComputedValueType::AVERAGE_VALID_AND_SET)
        return ComputedValueType::AVERAGE_VALID_AND_SET;
    else if(value == ComputedValueType::AVERAGE_TOTAL)
        return ComputedValueType::AVERAGE_TOTAL;

    throw std::invalid_argument( "The given value does not correspond to any NumericGridmap::ComputedValueType" );
}

 /**
 * Constructor.
 * Already allocates the grid with the given size.
 */
template<typename T>
NumericGridmap<T>::NumericGridmap(const GridmapLayout &lo, LogLevel logLevel)
    : Gridmap<T>(lo, logLevel)
{
    this->logger().setBaseName(__FUNCTION__);
}

/**
 * Constructor.
 * The grid is not allocated yet (it doesn't exist).
 */
template<typename T>
NumericGridmap<T>::NumericGridmap(LogLevel logLevel)
    : NumericGridmap<T>(GridmapLayout(), logLevel)
{
    this->logger().setBaseName(__FUNCTION__);
}

/**
 * Copy constructor.
 */
template<typename T>
NumericGridmap<T>::NumericGridmap(const NumericGridmap<T> &other)
    : Gridmap<T>(other)
{
    this->logger().setBaseName(__FUNCTION__);
}

/**
 * Destructor.
 */
template<typename T>
NumericGridmap<T>::~NumericGridmap()
{
}

//------------------------------------
//--------------OPERATORS-------------
//------------------------------------

// copy assignment
template<typename T>
NumericGridmap<T>& NumericGridmap<T>::operator=(const NumericGridmap<T>& other){
    Gridmap<T>::operator =(other);
    return *this;
}


//------------------------------------
//--------------SET-VALUE-------------
//------------------------------------


/**
 * Adds the given the data(value) to the current value of all the cells within the grid.
 */
template<typename T>
bool NumericGridmap<T>::addValue(const T &_value,
                      bool onlyCellsWithValue)
{
    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    for(size_t x = 0 ; x < this->getSizeX() ; x++){
        for(size_t y = 0 ; y < this->getSizeY() ; y++){
            long double valTmp = 0;

            if(!this->isSet(x, y)){
                if(onlyCellsWithValue)
                    continue;
            }
            else
                valTmp = this->get(x, y);

            valTmp += _value;

            if( !std::is_floating_point<T>::value )
                valTmp = std::round(valTmp);

            this->set(x, y, (T)valTmp);
        }
    }
    return true;

}


/**
 * Adds the given the data(value) to the current value of a cell within the grid, specified by the cell's x- and y- indexes.
 */
template<typename T>
bool NumericGridmap<T>::addValue(unsigned int x,
                                     unsigned int y,
                                     const T &_value,
                                     bool onlyCellsWithValue)
{
    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    if ( !this->checkIndexesInRange(x,y) ){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Indexes out of range");
        return false;
    }

    long double valTmp = 0;

    if(!this->isSet(x, y)){
        if(onlyCellsWithValue)
            return true;
    }
    else
        valTmp = this->get(x, y);

    valTmp += _value;

    return set_internal(x, y, valTmp);
}

/**
 * Adds the given the data(value) to the current value of a grid point, given in *real world coordinates*.
 */
template<typename T>
bool NumericGridmap<T>::addValue(const Point& p,
                      const T &value,
                      bool onlyCellsWithValue) {

    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    if ( !this->checkPointInRange(p) ) {
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Input point " + p.toString()
                                                         + "out of range : range(x) = [ " + std::to_string( this->m_layout.getMinPointX() )
                                                         + "," + std::to_string(this->m_layout.getMaxPointX())
                                                         + " ] ; range(y) = [ " + std::to_string(this->m_layout.getMinPointY())
                                                         + "," + std::to_string(this->m_layout.getMaxPointY()) + " ]");
        return false;
    }

    unsigned int gridX, gridY;
    this->point2index(p, gridX, gridY);

    return addValue(gridX, gridY, value, onlyCellsWithValue);
}

/**
 * Multiplies the current value of all cells within the grid by the given the value.
 */
template<typename T>
bool NumericGridmap<T>::multValue(const T &_value)
{
    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    for(size_t x = 0 ; x < this->getSizeX() ; x++){
        for(size_t y = 0 ; y < this->getSizeY() ; y++){
            if(!this->isSet(x, y))
                continue;

            long double valTmp = this->get(x, y);

            valTmp += _value;

            set_internal(x, y, valTmp);
        }
    }
    return true;
}


/**
 * Multiplies the current value of a cell within the grid (specified by the cell's x- and y- indexes) by the given the value.
 */
template<typename T>
bool NumericGridmap<T>::multValue(unsigned int x,
                       unsigned int y,
                       const T &_value)
{
    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    if ( !this->checkIndexesInRange(x,y) ){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Indexes out of range");
        return false;
    }

    if(!this->isSet(x, y))
        return true;

    long double valTmp = this->get(x, y);

    valTmp += _value;
    return set_internal(x, y, valTmp);
}

/**
 * Multiplies the current value of a grid point by the given the value, given in *real world coordinates*.
 */
template<typename T>
bool NumericGridmap<T>::multValue(const Point& p,
                       const T &value) {

    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    if ( !this->checkPointInRange(p) ) {
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Input point " + p.toString()
                                                         + "out of range : range(x) = [ " + std::to_string(this->m_layout.getMinPointX())
                                                         + "," + std::to_string(this->m_layout.getMaxPointX())
                                                         + " ] ; range(y) = [ " + std::to_string(this->m_layout.getMinPointY())
                                                         + "," + std::to_string(this->m_layout.getMaxPointY()) + " ]");
        return false;
    }

    unsigned int gridX, gridY;
    this->point2index(p, gridX, gridY);

    return multValue(gridX, gridY, value);
}

/**
 * Set the data(value) of the cells corresponding to a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
 * If the expanded-line does not completely include a cell, the value of the cell will be updated proportionally to the overlaying area taking into account the cell's previous value
 * (e.g. if half of a cell (with initial value 1) must be updated with a value 0, the new value of the cell will be 0.5)
 */
template<typename T>
bool NumericGridmap<T>::updateLineProportionally(const Point& start,
                                                 const Point& end,
                                                 double width,
                                                 const T& value,
                                                 bool onlyCellsWithValue)
{
    const double areaCell = this->getCellArea();
    CellsRangeList indexMap;
    Polygon linePoly;

    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;


    //get the cells under the extended line
    indexMap = this->getCellsUnderLine(start, end, width);
    if (indexMap.empty())
        return false;

    linePoly = arolib::geometry::createRectangleFromLine(start,end,width);

    //calculate the data(value) for the overall polygon area based on the grid cells with whom it overlaps
    if(this->m_computeInMultiThread){
        std::vector< std::future< void > > futures_x (indexMap.maxX() - indexMap.minX() + 1);
        for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
            auto& fu_x = futures_x.at(x - indexMap.minX());
            fu_x = std::async(std::launch::async,
                            [&, linePoly, x](){
                int min_y, max_y;
                auto yRanges = indexMap.getColumn(x);
                int lastY = -1;
                for(const auto& yRange : yRanges){
                    min_y = yRange.first;
                    max_y = yRange.second;
                    if(lastY == min_y)
                        ++min_y;
                    lastY = max_y;
                    for (int y = min_y ; y <= max_y ; ++y){
                        bool errorTmp;
                        long double prevValue = 0;
                        if(this->hasValue(x, y, &errorTmp) && !errorTmp)
                            prevValue = this->get(x, y);
                        else if(onlyCellsWithValue)
                            continue;

                        if(errorTmp)
                            continue;

                        double areaIntersection = this->m_layout.getCellAreaIntersection(x, y, linePoly, width);
                        long double newValue = ( (areaCell - areaIntersection) * prevValue + areaIntersection * value ) / areaCell;

                        set_internal(x, y, newValue);
                    }
                }

            });
        }

        for(auto& fu_x : futures_x)
            fu_x.wait();
    }
    else{
        bool errorTmp;
        int min_y, max_y;
        for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
            auto yRanges = indexMap.getColumn(x);
            int lastY = -1;
            for(const auto& yRange : yRanges){
                min_y = yRange.first;
                max_y = yRange.second;
                if(lastY == min_y)
                    ++min_y;
                lastY = max_y;
                for (int y = min_y ; y <= max_y ; ++y){
                    long double prevValue = 0;
                    if(this->hasValue(x, y, &errorTmp) && !errorTmp)
                        prevValue = this->get(x, y);
                    else if(onlyCellsWithValue)
                        continue;

                    if(errorTmp)
                        continue;
                    double areaIntersection = this->m_layout.getCellAreaIntersection(x, y, linePoly, width);
                    long double newValue = ( (areaCell - areaIntersection) * prevValue + areaIntersection * value ) / areaCell;

                    set_internal(x, y, newValue);
                }
            }
        }
    }

    return true;

}

/**
 * Adds to the the data(value) of the cells corresponding to a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
 * If the expanded-line does not completely include a cell, the value to be added to the cell will be updated proportionally to the overlaying area
 * (e.g. if half of a cell (with initial value 1) must be updated by adding a value 10, the new value of the cell will be 6)
 */
template<typename T>
bool NumericGridmap<T>::addToLine(const Point& start,
                                  const Point& end,
                                  double width,
                                  const T &value,
                                  bool onlyCellsWithValue,
                                  bool be_precise,
                                  const T &min_value,
                                  const T &max_value)
{
    double areaCell = this->getCellArea();
    Polygon linePoly;
    CellsRangeList indexMap;

    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    if (be_precise){

        //get the cells under the extended line
        indexMap = this->getCellsUnderLine(start, end, width);
        if (indexMap.empty())
            return false;

        linePoly = arolib::geometry::createRectangleFromLine(start,end,width);

        //calculate the data(value) for the overall polygon area based on the grid cells with whom it overlaps
        if(this->m_computeInMultiThread){

            std::vector< std::future< void > > futures_x (indexMap.maxX() - indexMap.minX() + 1);
            for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
                auto& fu_x = futures_x.at(x - indexMap.minX());
                fu_x = std::async(std::launch::async,
                                [&, linePoly, x](){
                    int min_y, max_y;
                    auto yRanges = indexMap.getColumn(x);
                    int lastY = -1;
                    for(const auto& yRange : yRanges){
                        min_y = yRange.first;
                        max_y = yRange.second;
                        if(lastY == min_y)
                            ++min_y;
                        lastY = max_y;
                        for (int y = min_y ; y <= max_y ; ++y){
                            bool errorTmp;
                            long double prevValue = 0;
                            if(this->hasValue(x, y, &errorTmp) && !errorTmp)
                                prevValue = this->get(x, y);
                            else if(onlyCellsWithValue)
                                continue;

                            if(errorTmp)
                                continue;

                            double areaIntersection = this->m_layout.getCellAreaIntersection(x, y, linePoly, width);
                            long double newValue = prevValue + ( areaIntersection * value ) / areaCell;

                            set_internal(x, y, newValue, min_value, max_value);
                        }
                    }

                });
            }

            for(auto& fu_x : futures_x)
                fu_x.wait();
        }
        else{
            bool errorTmp;
            int min_y, max_y;
            for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
                auto yRanges = indexMap.getColumn(x);
                int lastY = -1;
                for(const auto& yRange : yRanges){
                    min_y = yRange.first;
                    max_y = yRange.second;
                    if(lastY == min_y)
                        ++min_y;
                    lastY = max_y;
                    for (int y = min_y ; y <= max_y ; ++y){
                        long double prevValue = 0;
                        if(this->hasValue(x, y, &errorTmp) && !errorTmp)
                            prevValue = this->get(x, y);
                        else if(onlyCellsWithValue)
                            continue;

                        if(errorTmp)
                            continue;

                        double areaIntersection = this->m_layout.getCellAreaIntersection(x, y, linePoly, width);
                        long double newValue = prevValue + ( areaIntersection * value ) / areaCell;

                        set_internal(x, y, newValue, min_value, max_value);
                    }
                }
            }
        }
    }
    else{
        //get the cells under the extended line
        indexMap = this->getCellsUnderLine(start, end, width);
        if (indexMap.empty())
            return false;

        bool errorTmp;
        int min_y, max_y;
        for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
            auto yRanges = indexMap.getColumn(x);
            int lastY = -1;
            for(const auto& yRange : yRanges){
                min_y = yRange.first;
                max_y = yRange.second;
                if(lastY == min_y)
                    ++min_y;
                lastY = max_y;
                for (int y = min_y ; y <= max_y ; ++y){
                    long double prevValue = 0;
                    if(this->hasValue(x, y, &errorTmp) && !errorTmp)
                        prevValue = this->get(x, y);
                    else if(onlyCellsWithValue)
                        continue;
                    if(errorTmp)
                        continue;
                    long double newValue = value+prevValue;
                    set_internal(x, y, newValue, min_value, max_value);
                }
            }
        }
        //return setGridLine(start, end, width, value);
    }

    return true;

}


/**
 * Set the data(value) of the cells corresponding to a polygon within the grid, given in *real world coordinates*.
 * If the polygon does not completely include a cell, the value of the cell will be updated proportionally to the overlaying area taking into account the cell's previous value
 * (e.g. if half of a cell (with initial value 0) must be updated with a value 9, the new value of the cell will be 0.5)
 */
template<typename T>
bool NumericGridmap<T>::updatePolygonProportionally(const Polygon& _poly,
                                                    const T & value,
                                                    bool onlyCellsWithValue)
{
    double areaCell = this->getCellArea();
    Polygon poly = _poly;
    arolib::geometry::correct_polygon(poly);
    CellsRangeList indexMap;

    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;
    if (_poly.points.size() < 3){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "The polygon must have at least 3 points");
        return false;
    }

    //get the cells under the polygon
    //indexMap = this->getCellsUnderPolygon(poly, true, true);
    indexMap = this->getCellsUnderPolygon(poly);

    if (indexMap.empty()){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Polygon outside of the grid (resulting polygon indexMap is empty)");
        return false;
    }

    if(this->m_computeInMultiThread){

        std::vector< std::future< void > > futures_x (indexMap.maxX() - indexMap.minX() + 1);
        for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
            auto& fu_x = futures_x.at(x - indexMap.minX());
            fu_x = std::async(std::launch::async,
                              [&/*, poly*/, x](){
                int min_y, max_y;
                auto yRanges = indexMap.getColumn(x);
                int lastY = -1;
                for(const auto& yRange : yRanges){
                    min_y = yRange.first;
                    max_y = yRange.second;
                    if(lastY == min_y)
                        ++min_y;
                    lastY = max_y;
                    for (int y = min_y ; y <= max_y ; ++y){
                        bool errorTmp;
                        long double prevValue = 0;
                        if(this->hasValue(x, y, &errorTmp) && !errorTmp)
                            prevValue = this->get(x, y);
                        else if(onlyCellsWithValue)
                            continue;

                        if(errorTmp)
                            continue;

                        double areaIntersection = this->m_layout.getCellAreaIntersection(x, y, poly, min_y, max_y);
                        long double newValue = ( (areaCell - areaIntersection) * prevValue + areaIntersection * value ) / areaCell;

                        set_internal(x, y, newValue);
                    }
                }

            });
        }

        for(auto& fu_x : futures_x)
            fu_x.wait();
    }
    else{
        int min_y, max_y;
        bool errorTmp;
        for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
            auto yRanges = indexMap.getColumn(x);
            int lastY = -1;
            for(const auto& yRange : yRanges){
                min_y = yRange.first;
                max_y = yRange.second;
                if(lastY == min_y)
                    ++min_y;
                lastY = max_y;
                for (int y = min_y ; y <= max_y ; ++y){
                    long double prevValue = 0;
                    if(this->hasValue(x, y, &errorTmp) && !errorTmp)
                        prevValue = this->get(x, y);
                    else if(onlyCellsWithValue)
                        continue;

                    if(errorTmp)
                        continue;

                    double areaIntersection = this->m_layout.getCellAreaIntersection(x, y, poly, min_y, max_y);
                    long double newValue = ( (areaCell - areaIntersection) * prevValue + areaIntersection * value ) / areaCell;

                    set_internal(x, y, newValue);
                }
            }

        }
    }
    return true;
}


/**
 * Updates the cells in the list with a new value.
 * The new value will be equal to the old_value*(mult-1)+(new_)value*mult, where the mult is obtained from the input vector cells.
 * @param cells List of cells (with multimpliers as cell.value) to be used
 * @param value New value of the cells.
 * @param is_norm If set to true, the given value will be read as a normalized value [0...1].
 * @return True if successful
 */
template<typename T>
bool NumericGridmap<T>::updateCellsValue(const std::vector<GridmapLayout::GridCellOverlap> &cells, const T& value, bool onlyCellsWithValue)
{
    int x, y;
    long double prevValue, newValue;
    double mult;
    bool errorTmp;

    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    for (unsigned int i = 0 ; i < cells.size() ; ++i){
        x = cells.at(i).x;
        y = cells.at(i).y;
        if (this->checkIndexesInRange(x,y)){
            mult = std::min( std::max(cells.at(i).overlap, 0.0f) , 1.0f );
            prevValue = 0;
            if(this->hasValue(x, y, &errorTmp) && !errorTmp)
                prevValue = this->get(x, y);
            else if(onlyCellsWithValue)
                continue;

            if(errorTmp)
                continue;

            newValue = ( (1-mult) * prevValue + mult * value );
            this->set_internal(x, y, newValue);
        }
    }

    return true;
}

/**
 * Adds the corresponding value to the cells in the list
 */
template<typename T>
template<typename K, typename>
bool NumericGridmap<T>::addCellsValue(const T& _value,
                                      const std::vector<K> &cells,
                                      bool onlyCellsWithValue)
{
    int x, y;

    if ( !this->checkInternalParameters(true, __FUNCTION__) )
        return false;

    for (unsigned int i = 0 ; i < cells.size() ; ++i){
        x = cells.at(i).x;
        y = cells.at(i).y;
        this->addValue(x, y, _value, onlyCellsWithValue);
    }
    return true;
}

template<typename T>
bool NumericGridmap<T>::addCellsValue(const T& _value,
                                      const std::vector<GridmapLayout::GridCellOverlap> &cells,
                                      bool onlyCellsWithValue)
{
    int x, y;

    if ( !this->checkInternalParameters(true, __FUNCTION__) )
        return false;

    for (unsigned int i = 0 ; i < cells.size() ; ++i){
        x = cells.at(i).x;
        y = cells.at(i).y;
        auto mult = std::min( std::max(cells.at(i).overlap, 0.0f) , 1.0f );
        this->addValue(x, y, _value*mult, onlyCellsWithValue);
    }
    return true;
}

/**
 * Adds the corresponding value to the cells in the list spreading evenly based on the (overlaped) cell area and the (given) total area
 */
template<typename T>
template<typename K, typename>
bool NumericGridmap<T>::addCellsValue(const T& _value,
                           double area,
                           const std::vector<K>& cells,
                           bool onlyCellsWithValue)
{
    int x, y;

    if(area == 0)
        return false;

    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    double cellArea = this->getCellArea();
    double invArea = 1.0/area;

    for (unsigned int i = 0 ; i < cells.size() ; ++i){
        x = cells.at(i).x;
        y = cells.at(i).y;
        double mult = cellArea * invArea;
        this->addValue(x, y, _value*mult, onlyCellsWithValue);
    }
    return true;
}


template<typename T>
bool NumericGridmap<T>::addCellsValue(const T& _value,
                                      double area,
                                      const std::vector<GridmapLayout::GridCellOverlap>& cells,
                                      bool onlyCellsWithValue)
{
    int x, y;

    if(area == 0)
        return false;

    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    double cellArea = this->getCellArea();
    double invArea = 1.0/area;

    for (unsigned int i = 0 ; i < cells.size() ; ++i){
        x = cells.at(i).x;
        y = cells.at(i).y;
        auto mult = std::min( std::max(cells.at(i).overlap, 0.0f) , 1.0f );
        mult *= cellArea * invArea;
        this->addValue(x, y, _value*mult, onlyCellsWithValue);
    }
    return true;
}


/**
 * Adds the corresponding value to the cells in the list
 */
template<typename T>
template<typename K, typename>
bool NumericGridmap<T>::addCellsValue(const std::vector<K>& cells,
                           bool onlyCellsWithValue)
{
    int x, y;

    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    auto gridTmp = *this;
    for (unsigned int i = 0 ; i < cells.size() ; ++i){
        x = cells.at(i).x;
        y = cells.at(i).y;
        gridTmp.addValue(x, y, cells.at(i).value, onlyCellsWithValue );
    }
    *this = gridTmp;
    return true;
}



/**
 * Multiplies the corresponding value to the cells in the list
 */
template<typename T>
template<typename K, typename>
bool NumericGridmap<T>::multCellsValue(const T& _value,
                                      const std::vector<K> &cells)
{
    int x, y;

    if ( !this->checkInternalParameters(true, __FUNCTION__) )
        return false;

    for (unsigned int i = 0 ; i < cells.size() ; ++i){
        x = cells.at(i).x;
        y = cells.at(i).y;
        this->multValue(x, y, _value);
    }
    return true;
}

template<typename T>
bool NumericGridmap<T>::multCellsValue(const T& _value,
                                      const std::vector<GridmapLayout::GridCellOverlap> &cells)
{
    int x, y;

    if ( !this->checkInternalParameters(true, __FUNCTION__) )
        return false;

    for (unsigned int i = 0 ; i < cells.size() ; ++i){
        x = cells.at(i).x;
        y = cells.at(i).y;
        auto mult = std::min( std::max(cells.at(i).overlap, 0.0f) , 1.0f );
        this->multValue(x, y, _value*mult);
    }
    return true;
}

/**
 * Multiplies the corresponding value to the cells in the list spreading evenly based on the (overlaped) cell area and the (given) total area
 */
template<typename T>
template<typename K, typename>
bool NumericGridmap<T>::multCellsValue(const T& _value,
                           double area,
                           const std::vector<K>& cells)
{
    int x, y;

    if(area == 0)
        return false;

    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    double cellArea = this->getCellArea();
    double invArea = 1.0/area;

    for (unsigned int i = 0 ; i < cells.size() ; ++i){
        x = cells.at(i).x;
        y = cells.at(i).y;
        double mult = cellArea * invArea;
        this->multValue(x, y, _value*mult);
    }
    return true;
}


template<typename T>
bool NumericGridmap<T>::multCellsValue(const T& _value,
                                      double area,
                                      const std::vector<GridmapLayout::GridCellOverlap>& cells)
{
    int x, y;

    if(area == 0)
        return false;

    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    double cellArea = this->getCellArea();
    double invArea = 1.0/area;

    for (unsigned int i = 0 ; i < cells.size() ; ++i){
        x = cells.at(i).x;
        y = cells.at(i).y;
        auto mult = std::min( std::max(cells.at(i).overlap, 0.0f) , 1.0f );
        mult *= cellArea * invArea;
        this->multValue(x, y, _value*mult);
    }
    return true;
}


/**
 * Multiplies the corresponding value to the cells in the list
 */
template<typename T>
template<typename K, typename>
bool NumericGridmap<T>::multCellsValue(const std::vector<K>& cells)
{
    int x, y;

    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    auto gridTmp = *this;
    for (unsigned int i = 0 ; i < cells.size() ; ++i){
        x = cells.at(i).x;
        y = cells.at(i).y;
        gridTmp.multValue(x, y, cells.at(i).value );
    }
    *this = gridTmp;
    return true;
}


/**
 * Intersects the grid values with a polygon (the values inside the polgon stay the same in the grid, whereas the cells outside the polygon are set to a given value)
 */
template<typename T>
bool NumericGridmap<T>::intersectProportionaly(const T & valueOut,
                        const Polygon& _poly){

    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;
    if (_poly.points.size() < 3){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "The polygon must have at least 3 points");
        return false;
    }


    NumericGridmap<float> gridTmp(this->m_layout, this->logger().logLevel());

    if(!gridTmp.isAllocated()){
        this->m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error creating temporal gridmap");
        return false;
    }

    for(size_t x = 0 ; x < this->getSizeX() ; x++){
        for(size_t y = 0 ; y < this->getSizeY() ; y++)
            this->setValue(x, y, 0.0);
    }

    if( !gridTmp.updatePolygonProportionally(_poly, 1) ){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error setting initial polygon cell values");
        return false;
    }

    for(size_t x = 0 ; x < this->getSizeX() ; x++){
        for(size_t y = 0 ; y < this->getSizeY() ; y++){
            auto valueTmp = gridTmp.getValue(x, y);
            if(valueTmp > 0.00001){
                if(valueTmp > 0.99999)
                    valueTmp = 1;
                long double valueOld = 0;
                if(!this->isSet(x, y)){
                    if(valueTmp > 0.99999)
                        continue;
                }
                else
                    valueOld = this->get(x, y);
                this->setValue(x, y, valueOld * valueTmp + valueOut * (1-valueTmp));
            }
            else
                this->set(x, y, valueOut);
        }
    }

    return true;

}

/**
 * Intersects the grid values with a polygon (the values inside the polgon stay the same in the grid, whereas the cells outside the polygon are set to no-value if it exixts)
 */
template<typename T>
bool NumericGridmap<T>::intersectProportionaly(const Polygon& _poly){

    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;
    if (_poly.points.size() < 3){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "The polygon must have at least 3 points");
        return false;
    }


    NumericGridmap<float> gridTmp(this->m_layout, this->logger().logLevel());

    if(!gridTmp.isAllocated()){
        this->m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error creating temporal gridmap");
        return false;
    }

    for(size_t x = 0 ; x < this->getSizeX() ; x++){
        for(size_t y = 0 ; y < this->getSizeY() ; y++)
            this->setValue(x, y, 0.0);
    }

    if( !gridTmp.updatePolygonProportionally(_poly, 1) ){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error setting initial polygon cell values");
        return false;
    }

    for(size_t x = 0 ; x < this->getSizeX() ; x++){
        for(size_t y = 0 ; y < this->getSizeY() ; y++){
            auto valueTmp = gridTmp.getValue(x, y);
            if(valueTmp > 0.00001){
                if(valueTmp > 0.99999)
                    valueTmp = 1;
                long double valueOld = 0;
                if(!this->isSet(x, y)){
                    if(valueTmp > 0.99999)
                        continue;
                }
                else
                    valueOld = this->get(x, y);
                this->setValue(x, y, valueOld * valueTmp);
            }
            else
                this->unSet(x, y);
        }
    }

    return true;

}


////------------------------------------
////--------------GET-VALUE-------------
////------------------------------------


/**
 * (Old implementation) Obtain the data(value) of a line, specified by the indexes of two *grid cells*, within the grid . The line is extended sideways a distance if width/2.
 */
template<typename T>
long double NumericGridmap<T>::getLineComputedValue(unsigned int startX,
                                                    unsigned int startY,
                                                    unsigned int stopX,
                                                    unsigned int stopY,
                                                    double width,
                                                    ComputedValueType value_type,
                                                    bool* _error_) const
{

    if(_error_) *_error_ = false;

    if ( !this->checkInternalParameters(true,__FUNCTION__) ){
        if(_error_) *_error_ = true;
        return 0;
    }

    if( !this->checkIndexesInRange(startX,startY) ){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Start indexes out of  range");
        if(_error_) *_error_ = true;
        return 0;
    }
    if( !this->checkIndexesInRange(stopX,stopY) ){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Stop indexes out of  range");
        if(_error_) *_error_ = true;
        return 0;
    }

    if( width<-1e-9 ){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "The width must have a positive/zero value");
        if(_error_) *_error_ = true;
        return 0;
    }


    Point p1, p2;
    this->getCellCenter(startX, startY, p1);
    this->getCellCenter(stopX, stopY, p2);

    if(width<1e-9)
        return getLineComputedValue(p1, p2, value_type, _error_);

    const long double nothing = 0;
    long double value = nothing;
    double areaLine = arolib::geometry::calc_dist(p1, p2) * width;
    double areaCell = this->getCellArea();
    int countValid = 0, countSet = 0;

    Polygon linePoly = arolib::geometry::createRectangleFromLine(p1, p2, width);

    bool lineInside = true;
    for(auto& p : linePoly.points){
        if(!this->checkPointInRange(p)){
            lineInside = false;
            break;
        }
    }

    if(!lineInside){
        Polygon gridPoly = this->getGridPolygon();
        auto intersection = arolib::geometry::get_likely_intersection(gridPoly, linePoly, this->getCellsize()*1e-3);
        if(intersection.empty())
            return 0;
        linePoly = intersection.front();
    }

    auto& points = linePoly.points;

    std::map<int, std::set<int>> sortedIndexMap;// < x_index, y_indexes >
    for(size_t i = 0 ; i+1 < points.size() ; ++i){
        auto indexMap = this->getCellsUnderLine(points.at(i), points.at(i+1), true);
        for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
            auto yRanges = indexMap.getColumn(x);
            auto& sortedIndexes = sortedIndexMap[x];
            for(const auto& yRange : yRanges){
                sortedIndexes.insert(yRange.first);
                sortedIndexes.insert(yRange.second);
            }
        }
    }

    for(auto &col_it : sortedIndexMap){
        if(col_it.second.empty())
            continue;
        int x = col_it.first;
        int min_y = *col_it.second.begin();
        int max_y = *col_it.second.rbegin();
        for (int y = min_y ; y <= max_y ; ++y){
            if(!this->checkIndexesInRange(x,y))
                continue;
            ++countValid;
            if(!this->isSet(x, y))
                continue;
            value += this->get(x, y);
            ++countSet;
        }
    }


    if (value_type == ComputedValueType::AVERAGE_TOTAL ){
        if (areaLine == 0)
            return 0;
        return ( value * areaCell/areaLine );
    }

    if (value_type == ComputedValueType::AVERAGE_VALID){
        if (countValid == 0)
            return 0;
        return ( value / countValid );
    }

    if (value_type == ComputedValueType::AVERAGE_VALID_AND_SET){
        if (countSet == 0)
            return 0;
        return ( value / countSet );
    }

    return value;

}

/**
 * (Old implementation) Obtain the data(value) of a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
 */
template<typename T>
long double NumericGridmap<T>::getLineComputedValue2(const Point& start,
                                                     const Point& stop,
                                                     double width,
                                                     ComputedValueType value_type,
                                                     bool* _error_) const
{
    if(_error_) *_error_ = false;

    if ( !this->checkInternalParameters(true,__FUNCTION__) ){
        if(_error_) *_error_ = true;
        return 0;
    }

//        if (width<=0){
//            this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "The line width must have a positive value");
//            _error_ = true;
//            return 0;
//        }

    if ( !this->checkPointInRange(start) ) {
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "START point " + start.toString()
                                                         + "out of range : range(x) = [ " + std::to_string(this->m_layout.getMinPointX())
                                                         + "," + std::to_string(this->m_layout.getMaxPointX())
                                                         + " ] ; range(y) = [ " + std::to_string(this->m_layout.getMinPointY())
                                                         + "," + std::to_string(this->m_layout.getMaxPointY()) + " ]");
        if(_error_) *_error_ = true;
        return 0;
    }

    if ( !this->checkPointInRange(stop) ) {
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "STOP point " + stop.toString()
                                                         + "out of range : range(x) = [ " + std::to_string(this->m_layout.getMinPointX())
                                                         + "," + std::to_string(this->m_layout.getMaxPointX())
                                                         + " ] ; range(y) = [ " + std::to_string(this->m_layout.getMinPointY())
                                                         + "," + std::to_string(this->m_layout.getMaxPointY()) + " ]");
        if(_error_) *_error_ = true;
        return 0;
    }

    unsigned int startX, startY, stopX, stopY;
    this->point2index(start, startX, startY);
    this->point2index(stop, stopX, stopY);

    return getLineComputedValue(startX, startY, stopX, stopY, width, value_type, _error_);
}

/**
 * Obtain the data(value) of a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
 */
template<typename T>
long double NumericGridmap<T>::getLineComputedValue(const Point& start,
                                                    const Point& end,
                                                    double width,
                                                    bool be_precise,
                                                    ComputedValueType value_type,
                                                    bool* _error_) const
{
    return getLineComputedValue_v1(start, end, width, be_precise, value_type, _error_);
}

template<typename T>
long double NumericGridmap<T>::getLineComputedValue_v1(const Point& start,
                                                       const Point& end,
                                                       double width,
                                                       bool be_precise,
                                                       ComputedValueType value_type,
                                                       bool* _error_) const
{
    if(_error_) *_error_ = false;

    const long double nothing = 0;
    long double value = nothing;
    double areaCell = this->getCellArea();
    double areaSet = 0, areaLine;
    CellsRangeList indexMap;
    Polygon linePoly;

    if ( !this->checkInternalParameters(true,__FUNCTION__) ){
        if(_error_) *_error_ = true;
        return 0;
    }

    linePoly = arolib::geometry::createRectangleFromLine(start,end,width);
    areaLine = arolib::geometry::calc_area(start,end,width);
    if (areaLine <= 0){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Line parameters are not correct");
        if(_error_) *_error_ = true;
        return 0;
    }

    //get the cells under the polygon
    indexMap = this->getCellsUnderLine(start, end, width);
    if(indexMap.empty()){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Line out of the grid (indexMap is empty)");
        if(_error_) *_error_ = true;
        return 0;
    }

    if (be_precise){
        //calculate the data(value) for the overall polygon area based on the grid cells with whom it overlaps
        if(this->m_computeInMultiThread){

            std::vector< std::future< std::pair<long double, double> > > futures_x (indexMap.maxX() - indexMap.minX() + 1);
            for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
                auto& fu_x = futures_x.at(x - indexMap.minX());
                fu_x = std::async(std::launch::async,
                                [&, linePoly, x]()->std::pair<long double, double>{

                    std::pair<long double, double> valuesOut;
                    valuesOut.first = nothing;//i.e. value
                    valuesOut.second = 0;//i.e. areaLineValid

                    int min_y, max_y;
                    auto yRanges = indexMap.getColumn(x);
                    int lastY = -1;
                    for(const auto& yRange : yRanges){
                        min_y = yRange.first;
                        max_y = yRange.second;
                        if(lastY == min_y)
                            ++min_y;
                        lastY = max_y;
                        for (int y = min_y ; y <= max_y ; ++y){

                            bool errorTmp = false;
                            if(!this->hasValue(x, y, &errorTmp) || errorTmp)
                                continue;

                            double areaIntersection = this->m_layout.getCellAreaIntersection(x, y, linePoly, width);
                            long double valueTemp = this->get(x, y);
                            valuesOut.first += (valueTemp * areaIntersection / areaCell);
                            valuesOut.second += areaIntersection;

                        }
                    }

                    return valuesOut;
                });
            }

            for(auto& fu_x : futures_x){
                auto values = fu_x.get();
                value += values.first;
                areaSet += values.second;
            }
        }
        else{
            bool errorTmp;
            int min_y, max_y;
            for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
                auto yRanges = indexMap.getColumn(x);
                int lastY = -1;
                for(const auto& yRange : yRanges){
                    min_y = yRange.first;
                    max_y = yRange.second;
                    if(lastY == min_y)
                        ++min_y;
                    lastY = max_y;
                    for (int y = min_y ; y <= max_y ; ++y){

                        if(!this->hasValue(x, y, &errorTmp) || errorTmp)
                            continue;

                        double areaIntersection = this->m_layout.getCellAreaIntersection(x, y, linePoly, width);
                        long double valueTemp = this->get(x, y);
                        value += (valueTemp * areaIntersection / areaCell);
                        areaSet += areaIntersection;
                    }
                }
            }
        }
    }
    else{
        int min_y, max_y;
        bool errorTmp;
        //calculate the data(value) for the overall polygon area based on the grid cells with whom it overlaps
        for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
            auto yRanges = indexMap.getColumn(x);
            int lastY = -1;
            for(const auto& yRange : yRanges){
                min_y = yRange.first;
                max_y = yRange.second;
                if(lastY == min_y)
                    ++min_y;
                lastY = max_y;
                for (int y = min_y ; y <= max_y ; ++y){

                    if(!this->hasValue(x, y, &errorTmp) || errorTmp)
                        continue;
                    value += this->get(x, y);
                    areaSet += areaCell;
                }
            }
        }

    }

    if (value_type == ComputedValueType::AVERAGE_TOTAL ){
        if (areaLine == 0)
            return 0;
        return ( value * areaCell/areaLine );
    }

    if (value_type == ComputedValueType::AVERAGE_VALID){
        Polygon gridPoly = this->getGridPolygon();
        double areaValid = 0;
        auto intersection = arolib::geometry::get_likely_intersection(gridPoly, linePoly, this->getCellsize()*1e-3);
        for (unsigned int k = 0 ; k < intersection.size() ; k++)
            areaValid += arolib::geometry::calc_area(intersection.at(k));

        if (areaValid == 0)
            return 0;
        return ( value * areaCell/areaValid );
    }

    if (value_type == ComputedValueType::AVERAGE_VALID_AND_SET){
        if (areaSet == 0)
            return 0;
        return ( value * areaCell/areaSet );
    }

    return value;
}

template<typename T>
long double NumericGridmap<T>::getLineComputedValue_v2(const Point& start,
                                                       const Point& end,
                                                       double width,
                                                       bool be_precise,
                                                       ComputedValueType value_type,
                                                       bool* _error_) const
{
    if(_error_) *_error_ = false;

    const long double nothing = 0;
    long double value = nothing;
    double areaCell = this->getCellArea();
    double areaSet = 0, areaLine;
    CellsRangeList indexMap;
    Polygon linePoly;

    if ( !this->checkInternalParameters(true,__FUNCTION__) ){
        if(_error_) *_error_ = true;
        return 0;
    }

    linePoly = arolib::geometry::createRectangleFromLine(start,end,width);
    areaLine = arolib::geometry::calc_area(start,end,width);
    if (areaLine <= 0){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Line parameters are not correct");
        if(_error_) *_error_ = true;
        return 0;
    }

    //get the cells under the polygon
    indexMap = this->getCellsUnderLine(start, end, width);
    if(indexMap.empty()){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Line out of the grid (indexMap is empty)");
        if(_error_) *_error_ = true;
        return 0;
    }

    if (be_precise){

        //get the cells that lie under the boundary of the poly
        std::unordered_set< std::pair<int, int>, boost::hash< std::pair<int, int> > > boundaryCells;
        for(size_t i = 0 ; i+1 < linePoly.points.size() ; ++i){
            auto indexMapLine = this->m_layout.getCellsUnderLine( linePoly.points.at(i), linePoly.points.at(i+1), false );
            for (unsigned int x = indexMapLine.minX() ; x <= indexMapLine.maxX() ; ++x){
                auto yRanges = indexMapLine.getColumn(x);

                if(yRanges.empty())
                    continue;

                auto& yRange = *yRanges.begin();//since it is a line, there must be unly one range

                int min_y = yRange.first;
                int max_y = yRange.second;
                for (int y = min_y ; y <= max_y ; ++y)
                    boundaryCells.insert( std::make_pair(x, y) );

            }

        }

        //calculate the data(value) for the overall polygon area based on the grid cells with whom it overlaps
        if(this->m_computeInMultiThread){

            std::vector< std::future< std::pair<long double, double> > > futures_x (indexMap.maxX() - indexMap.minX() + 1);
            for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
                auto& fu_x = futures_x.at(x - indexMap.minX());
                fu_x = std::async(std::launch::async,
                                [&, linePoly, x]()->std::pair<long double, double>{

                    std::pair<long double, double> valuesOut;
                    valuesOut.first = nothing;//i.e. value
                    valuesOut.second = 0;//i.e. areaLineValid

                    int min_y, max_y;
                    auto yRanges = indexMap.getColumn(x);
                    int lastY = -1;
                    for(const auto& yRange : yRanges){
                        min_y = yRange.first;
                        max_y = yRange.second;
                        if(lastY == min_y)
                            ++min_y;
                        lastY = max_y;
                        for (int y = min_y ; y <= max_y ; ++y){

                            bool errorTmp = false;
                            if(!this->hasValue(x, y, &errorTmp) || errorTmp)
                                continue;

                            if(boundaryCells.find( std::make_pair(x, y) ) != boundaryCells.end()){
                                double areaIntersection = this->m_layout.getCellAreaIntersection(x, y, linePoly, width);
                                long double valueTemp = this->get(x, y);
                                valuesOut.first += (valueTemp * areaIntersection / areaCell);
                                valuesOut.second += areaIntersection;
                            }
                            else
                                valuesOut.second += areaCell;

                        }
                    }

                    return valuesOut;
                });
            }

            for(auto& fu_x : futures_x){
                auto values = fu_x.get();
                value += values.first;
                areaSet += values.second;
            }
        }
        else{
            bool errorTmp;
            int min_y, max_y;
            for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
                auto yRanges = indexMap.getColumn(x);
                int lastY = -1;
                for(const auto& yRange : yRanges){
                    min_y = yRange.first;
                    max_y = yRange.second;
                    if(lastY == min_y)
                        ++min_y;
                    lastY = max_y;
                    for (int y = min_y ; y <= max_y ; ++y){

                        if(!this->hasValue(x, y, &errorTmp) || errorTmp)
                            continue;

                        if(boundaryCells.find( std::make_pair(x, y) ) != boundaryCells.end()){
                            double areaIntersection = this->m_layout.getCellAreaIntersection(x, y, linePoly, width);
                            long double valueTemp = this->get(x, y);
                            value += (valueTemp * areaIntersection / areaCell);
                            areaSet += areaIntersection;
                        }
                        else
                            areaSet += areaCell;
                    }
                }
            }
        }
    }
    else{
        int min_y, max_y;
        bool errorTmp;
        //calculate the data(value) for the overall polygon area based on the grid cells with whom it overlaps
        for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
            auto yRanges = indexMap.getColumn(x);
            int lastY = -1;
            for(const auto& yRange : yRanges){
                min_y = yRange.first;
                max_y = yRange.second;
                if(lastY == min_y)
                    ++min_y;
                lastY = max_y;
                for (int y = min_y ; y <= max_y ; ++y){

                    if(!this->hasValue(x, y, &errorTmp) || errorTmp)
                        continue;
                    value += this->get(x, y);
                    areaSet += areaCell;
                }
            }
        }

    }

    if (value_type == ComputedValueType::AVERAGE_TOTAL ){
        if (areaLine == 0)
            return 0;
        return ( value * areaCell/areaLine );
    }

    if (value_type == ComputedValueType::AVERAGE_VALID){
        Polygon gridPoly = this->getGridPolygon();
        double areaValid = 0;
        auto intersection = arolib::geometry::get_likely_intersection(gridPoly, linePoly, this->getCellsize()*1e-3);
        for (unsigned int k = 0 ; k < intersection.size() ; k++)
            areaValid += arolib::geometry::calc_area(intersection.at(k));

        if (areaValid == 0)
            return 0;
        return ( value * areaCell/areaValid );
    }

    if (value_type == ComputedValueType::AVERAGE_VALID_AND_SET){
        if (areaSet == 0)
            return 0;
        return ( value * areaCell/areaSet );
    }

    return value;
}



/**
 * Obtain the data(value) of a line within the grid, given in *real world coordinates*.
 */
template<typename T>
long double NumericGridmap<T>::getLineComputedValue(const Point& start,
                                                    const Point& end,
                                                    ComputedValueType value_type,
                                                    bool *_error_) const{

    if(_error_) *_error_ = false;

    if ( !this->checkInternalParameters(true,__FUNCTION__) ){
        if(_error_) *_error_ = true;
        return 0;
    }

    const long double nothing = 0;
    long double value = nothing;
    int countValid = 0;
    int countSet = 0;
    int countTotal = 0;

    auto indexMap = this->getCellsUnderLine(start, end, true);
    for (int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
        auto yRanges = indexMap.getColumn(x);
        int lastY = -1;
        for(const auto& yRange : yRanges){
            int min_y = yRange.first;
            int max_y = yRange.second;
            if(lastY == min_y)
                ++min_y;
            lastY = max_y;
            for (int y = min_y ; y <= max_y ; ++y){
                ++countTotal;
                if(!this->checkIndexesInRange(x, y))
                    continue;

                ++countValid;

                if(!this->isSet(x, y))
                    continue;

                value += this->get(x, y);
                ++countSet;
            }
        }
    }

    if (value_type == ComputedValueType::AVERAGE_TOTAL ){
        if(countTotal == 0)
            return 0;
        if(value == 0)
            return 0;

        double lengthValid = 0;
        bool startIn = this->checkPointInRange(start);
        bool endIn = this->checkPointInRange(end);
        if( startIn && endIn ){
            return value / countTotal;
        }

        auto intersections = arolib::geometry::get_intersection(start, end, this->getGridPolygon().points, false, false, false);
        if(intersections.empty())
            return 0;
        if(startIn)
            lengthValid = arolib::geometry::calc_dist(start, intersections.front());
        else if(endIn)
            lengthValid = arolib::geometry::calc_dist(end, intersections.front());
        else{
            if(intersections.size() < 2)
                return 0;
            lengthValid = arolib::geometry::calc_dist(intersections.front(), intersections.at(1));
        }

        if(lengthValid == 0)
            return 0;
        return value / ( countTotal * arolib::geometry::calc_dist(start, end) / lengthValid );

    }

    if (value_type == ComputedValueType::AVERAGE_VALID){
        if (countValid == 0)
            return 0;
        return ( value / countValid );
    }

    if (value_type == ComputedValueType::AVERAGE_VALID_AND_SET){
        if (countSet == 0)
            return 0;
        return ( value / countSet );
    }

    return value;
}

/**
 * Obtain the data(value) of a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
 */
template<typename T>
template<typename K, typename>
long double NumericGridmap<T>::getLineComputedValue(const Point& start,
                                                    const Point& end,
                                                    double width,
                                                    const std::vector<K> &cells,
                                                    ComputedValueType value_type,
                                                    bool checkForRepeatedCells,
                                                    bool* _error_) const
{
    if(_error_) *_error_ = false;
    double area = 0;

    if ( !this->checkInternalParameters(true,__FUNCTION__) ){
        if(_error_) *_error_ = true;
        return 0;
    }

    if (value_type == ComputedValueType::AVERAGE_TOTAL)
        area = arolib::geometry::calc_area(start,end,width);

    return getCellsComputedValue(cells, value_type, area, checkForRepeatedCells, _error_);
}

/**
 * Obtain the data(value) of a polygon within the grid, given in *real world coordinates*.
 */
template<typename T>
long double NumericGridmap<T>::getPolygonComputedValue(const Polygon& _poly,
                                                       ComputedValueType value_type,
                                                       bool be_precise,
                                                       bool* _error_) const
{
    return getPolygonComputedValue_v1(_poly, value_type, be_precise, _error_);
}

template<typename T>
long double NumericGridmap<T>::getPolygonComputedValue_v1(const Polygon& _poly,
                                                       ComputedValueType value_type,
                                                       bool be_precise,
                                                       bool* _error_) const
{
    if(_error_) *_error_ = false;

    const long double nothing = 0;
    long double value = nothing;
    double areaCell = this->getCellArea();
    double areaSet = 0, areaPoly;
    CellsRangeList indexMap;
    Polygon poly = _poly;

    if ( !this->checkInternalParameters(true,__FUNCTION__) ){
        if(_error_) *_error_ = true;
        return 0;
    }
    if (_poly.points.size() < 3){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "The polygon must have at least 3 points");
        if(_error_) *_error_ = true;
        return 0;
    }

    arolib::geometry::correct_polygon(poly);
    areaPoly = arolib::geometry::calc_area(poly);

    //get the cells under the polygon
    //indexMap = this->getCellsUnderPolygon(poly, true, true);
    indexMap = this->getCellsUnderPolygon(poly);
    if(indexMap.empty()){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Polygon out of the grid (indexMap is empty)");
        if(_error_) *_error_ = true;
        return 0;
    }

    bool errorTmp;
    if (be_precise){
        //calculate the data(value) for the overall polygon area based on the grid cells with whom it overlaps

        if(this->m_computeInMultiThread){

            std::vector< std::future< std::pair<long double, double> > > futures_x (indexMap.maxX() - indexMap.minX() + 1);
            for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
                auto& fu_x = futures_x.at(x - indexMap.minX());
                fu_x = std::async(std::launch::async,
                                [&/*, poly*/, x]()->std::pair<long double, double>{

                    std::pair<long double, double> valuesOut;
                    valuesOut.first = nothing;//i.e. value
                    valuesOut.second = 0;//i.e. areaLineValid

                    int min_y, max_y;
                    auto yRanges = indexMap.getColumn(x);
                    int lastY = -1;
                    for(const auto& yRange : yRanges){
                        min_y = yRange.first;
                        max_y = yRange.second;
                        if(lastY == min_y)
                            ++min_y;
                        lastY = max_y;
                        for (int y = min_y ; y <= max_y ; ++y){

                            bool errorTmp = false;
                            if(!this->hasValue(x, y, &errorTmp) || errorTmp)
                                continue;

                            double areaIntersection = this->m_layout.getCellAreaIntersection(x, y, poly, min_y, max_y);
                            long double valueTemp = this->get(x, y);
                            valuesOut.first += (valueTemp * areaIntersection / areaCell);
                            valuesOut.second += areaIntersection;
                        }
                    }

                    return valuesOut;
                });
            }

            for(auto& fu_x : futures_x){
                auto values = fu_x.get();
                value += values.first;
                areaSet += values.second;
            }
        }
        else{
            int min_y, max_y;
            for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
                auto yRanges = indexMap.getColumn(x);
                int lastY = -1;
                for(const auto& yRange : yRanges){
                    min_y = yRange.first;
                    max_y = yRange.second;
                    if(lastY == min_y)
                        ++min_y;
                    lastY = max_y;
                    for (int y = min_y ; y <= max_y ; ++y){


                        if(!this->hasValue(x, y, &errorTmp) || errorTmp)
                            continue;

                        double areaIntersection = this->m_layout.getCellAreaIntersection(x, y, poly, min_y, max_y);
                        long double valueTemp = this->get(x, y);
                        value += (valueTemp * areaIntersection / areaCell);
                        areaSet += areaIntersection;
                    }
                }
            }
        }
    }
    else{

        int min_y, max_y;

        //calculate the data(value) for the overall polygon area based on the grid cells with whom it overlaps
        for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
            auto yRanges = indexMap.getColumn(x);
            int lastY = -1;
            for(const auto& yRange : yRanges){
                min_y = yRange.first;
                max_y = yRange.second;
                if(lastY == min_y)
                    ++min_y;
                lastY = max_y;
                for (int y = min_y ; y <= max_y ; ++y){

                    if(!this->hasValue(x, y, &errorTmp) || errorTmp)
                        continue;
                    long double valueTemp = this->get(x, y);
                    value += valueTemp;
                    areaSet += areaCell;
                }
            }
        }


    }

    if (value_type == ComputedValueType::AVERAGE_TOTAL ){
        if (areaPoly == 0)
            return 0;
        return ( value * areaCell/areaPoly );
    }

    if (value_type == ComputedValueType::AVERAGE_VALID){
        Polygon gridPoly = this->getGridPolygon();
        double areaValid = 0;
        auto intersection = arolib::geometry::get_likely_intersection(gridPoly, poly, this->getCellsize()*1e-3);
        for (unsigned int k = 0 ; k < intersection.size() ; k++)
            areaValid += arolib::geometry::calc_area(intersection.at(k));

        if (areaValid == 0)
            return 0;
        return ( value * areaCell/areaValid );
    }

    if (value_type == ComputedValueType::AVERAGE_VALID_AND_SET){
        if (areaSet == 0)
            return 0;
        return ( value * areaCell/areaSet );
    }

    return value;
}

template<typename T>
long double NumericGridmap<T>::getPolygonComputedValue_v2(const Polygon& _poly,
                                                       ComputedValueType value_type,
                                                       bool be_precise,
                                                       bool* _error_) const
{
    if(_error_) *_error_ = false;

    const long double nothing = 0;
    long double value = nothing;
    double areaCell = this->getCellArea();
    double areaSet = 0, areaPoly;
    CellsRangeList indexMap;
    Polygon poly = _poly;

    if ( !this->checkInternalParameters(true,__FUNCTION__) ){
        if(_error_) *_error_ = true;
        return 0;
    }
    if (_poly.points.size() < 3){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "The polygon must have at least 3 points");
        if(_error_) *_error_ = true;
        return 0;
    }

    arolib::geometry::correct_polygon(poly);
    arolib::geometry::unsample_polygon(poly);
    areaPoly = arolib::geometry::calc_area(poly);

    //get the cells under the polygon
    //indexMap = this->getCellsUnderPolygon(poly, true, true);
    indexMap = this->getCellsUnderPolygon(poly);
    if(indexMap.empty()){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Polygon out of the grid (indexMap is empty)");
        if(_error_) *_error_ = true;
        return 0;
    }

    bool errorTmp;
    if (be_precise){

        //get the cells that lie under the boundary of the poly
        std::unordered_set< std::pair<int, int>, boost::hash< std::pair<int, int> > > boundaryCells;
        for(size_t i = 0 ; i+1 < poly.points.size() ; ++i){
            auto indexMapLine = this->m_layout.getCellsUnderLine( poly.points.at(i), poly.points.at(i+1), false );
            for (unsigned int x = indexMapLine.minX() ; x <= indexMapLine.maxX() ; ++x){
                auto yRanges = indexMapLine.getColumn(x);

                if(yRanges.empty())
                    continue;

                auto& yRange = *yRanges.begin();//since it is a line, there must be unly one range

                int min_y = yRange.first;
                int max_y = yRange.second;
                for (int y = min_y ; y <= max_y ; ++y)
                    boundaryCells.insert( std::make_pair(x, y) );

            }

        }

        //calculate the data(value) for the overall polygon area based on the grid cells with whom it overlaps

        if(this->m_computeInMultiThread){

            std::vector< std::future< std::pair<long double, double> > > futures_x (indexMap.maxX() - indexMap.minX() + 1);
            for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
                auto& fu_x = futures_x.at(x - indexMap.minX());
                fu_x = std::async(std::launch::async,
                                [&/*, poly*/, x]()->std::pair<long double, double>{

                    std::pair<long double, double> valuesOut;
                    valuesOut.first = nothing;//i.e. value
                    valuesOut.second = 0;//i.e. areaLineValid

                    int min_y, max_y;
                    auto yRanges = indexMap.getColumn(x);
                    int lastY = -1;
                    for(const auto& yRange : yRanges){
                        min_y = yRange.first;
                        max_y = yRange.second;
                        if(lastY == min_y)
                            ++min_y;
                        lastY = max_y;
                        for (int y = min_y ; y <= max_y ; ++y){

                            bool errorTmp = false;
                            if(!this->hasValue(x, y, &errorTmp) || errorTmp)
                                continue;

                            if(boundaryCells.find( std::make_pair(x, y) ) != boundaryCells.end()){
                                double areaIntersection = this->m_layout.getCellAreaIntersection(x, y, poly, min_y, max_y);
                                long double valueTemp = this->get(x, y);
                                valuesOut.first += (valueTemp * areaIntersection / areaCell);
                                valuesOut.second += areaIntersection;
                            }
                            else
                                valuesOut.second += areaCell;
                        }
                    }

                    return valuesOut;
                });
            }

            for(auto& fu_x : futures_x){
                auto values = fu_x.get();
                value += values.first;
                areaSet += values.second;
            }
        }
        else{
            int min_y, max_y;
            for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
                auto yRanges = indexMap.getColumn(x);
                int lastY = -1;
                for(const auto& yRange : yRanges){
                    min_y = yRange.first;
                    max_y = yRange.second;
                    if(lastY == min_y)
                        ++min_y;
                    lastY = max_y;
                    for (int y = min_y ; y <= max_y ; ++y){

                        if(!this->hasValue(x, y, &errorTmp) || errorTmp)
                            continue;

                        if(boundaryCells.find( std::make_pair(x, y) ) != boundaryCells.end()){
                            double areaIntersection = this->m_layout.getCellAreaIntersection(x, y, poly, min_y, max_y);
                            long double valueTemp = this->get(x, y);
                            value += (valueTemp * areaIntersection / areaCell);
                            areaSet += areaIntersection;
                        }
                        else
                            areaSet += areaCell;
                    }
                }
            }
        }
    }
    else{

        int min_y, max_y;

        //calculate the data(value) for the overall polygon area based on the grid cells with whom it overlaps
        for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
            auto yRanges = indexMap.getColumn(x);
            int lastY = -1;
            for(const auto& yRange : yRanges){
                min_y = yRange.first;
                max_y = yRange.second;
                if(lastY == min_y)
                    ++min_y;
                lastY = max_y;
                for (int y = min_y ; y <= max_y ; ++y){

                    if(!this->hasValue(x, y, &errorTmp) || errorTmp)
                        continue;
                    long double valueTemp = this->get(x, y);
                    value += valueTemp;
                    areaSet += areaCell;
                }
            }
        }


    }

    if (value_type == ComputedValueType::AVERAGE_TOTAL ){
        if (areaPoly == 0)
            return 0;
        return ( value * areaCell/areaPoly );
    }

    if (value_type == ComputedValueType::AVERAGE_VALID){
        Polygon gridPoly = this->getGridPolygon();
        double areaValid = 0;
        auto intersection = arolib::geometry::get_likely_intersection(gridPoly, poly, this->getCellsize()*1e-3);
        for (unsigned int k = 0 ; k < intersection.size() ; k++)
            areaValid += arolib::geometry::calc_area(intersection.at(k));

        if (areaValid == 0)
            return 0;
        return ( value * areaCell/areaValid );
    }

    if (value_type == ComputedValueType::AVERAGE_VALID_AND_SET){
        if (areaSet == 0)
            return 0;
        return ( value * areaCell/areaSet );
    }

    return value;
}

/**
 * Obtain the data(value) of a polygon within the grid, given in *real world coordinates*, given the list of cells under it
 */
template<typename T>
template<typename K, typename>
long double NumericGridmap<T>::getPolygonComputedValue(const Polygon& _poly,
                                                       const std::vector<K>& cells,
                                                       ComputedValueType value_type,
                                                       bool checkForRepeatedCells,
                                                       bool* _error_) const
{
    if(_error_) *_error_ = false;

    Polygon poly = _poly;
    double area = 0;

    if ( !this->checkInternalParameters(true,__FUNCTION__) ){
        if(_error_) *_error_ = true;
        return 0;
    }
    if (_poly.points.size() < 3){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "The polygon must have at least 3 points");
        if(_error_) *_error_ = true;
        return 0;
    }

    if (value_type == ComputedValueType::AVERAGE_TOTAL){
        arolib::geometry::correct_polygon(poly);
        area = arolib::geometry::calc_area(poly);
        if (area == 0)
            return 0;
    }
    else if(value_type == ComputedValueType::AVERAGE_TOTAL){
        arolib::geometry::correct_polygon(poly);
        Polygon gridPoly = this->getGridPolygon();
        area = 0;
        auto intersection = arolib::geometry::get_likely_intersection(gridPoly, poly, this->getCellsize()*1e-3);
        for (unsigned int k = 0 ; k < intersection.size() ; k++)
            area += arolib::geometry::calc_area(intersection.at(k));

        if (area == 0)
            return 0;
    }

    return getCellsComputedValue(cells, value_type, area, checkForRepeatedCells, _error_);

}


/**
 * Get the data(values) of the cells in the list
 */
template<typename T>
template<typename K, typename>
long double NumericGridmap<T>::getCellsComputedValue(const std::vector<K>& cells,
                                                     ComputedValueType value_type,
                                                     double area,
                                                     bool checkForRepeatedCells,
                                                     bool* _error_) const
{
    if(_error_) *_error_ = false;

    if ( !this->checkInternalParameters(true,__FUNCTION__) ){
        if(_error_) *_error_ = true;
        return 0;
    }

    const long double nothing = 0;
    double value = nothing, mult, cellValue;
    double multSum = 0, multSumValid = 0, multSumSet = 0;
    int x, y;
    std::set< std::pair<int, int > > cellsSet;
    double areaCell = this->getCellArea();
    double areaValid = 0, areaSet = 0, areaTotal = 0;

    for (unsigned int i = 0 ; i < cells.size() ; ++i){
        mult = cells.at(i).overlap;
        x = cells.at(i).x;
        y = cells.at(i).y;
        if (checkForRepeatedCells){
            if (cellsSet.find( std::make_pair(x,y) ) != cellsSet.end())
                continue;
            cellsSet.insert( std::make_pair(x,y) );
        }

        areaTotal += areaCell;

        if (mult < -1e-9 || mult > 1+1e-9)
            continue;

        mult = std::max( 0.0, std::min( mult, 1.0 ) );

        multSum += mult;

        if (this->checkIndexesInRange(x,y)){
            areaValid += areaCell*mult;            
            multSumValid += mult;
            if( this->isSet(x, y) ){
                areaSet += areaCell*mult;
                cellValue = this->get(x, y);
                value += ( cellValue * mult );
                multSumSet += mult;
            }
        }
    }

    if (value_type == ComputedValueType::AVERAGE_TOTAL ){
        if (area > 1e-5){
            area = std::max(area, areaCell * multSum);//in case the computation of the cells list is not precise
            return value * areaCell/area;
        }
        if (areaTotal == 0)
            return 0;
        return ( value * areaCell/areaTotal );
    }

    if (value_type == ComputedValueType::AVERAGE_VALID){
        if (area > 1e-5){
            area = std::max(area, areaCell * multSumValid);//in case the computation of the cells list is not precise
            return value * areaCell/area;
        }
        if (areaValid == 0)
            return 0;
        return ( value * areaCell/areaValid );
    }

    if (value_type == ComputedValueType::AVERAGE_VALID_AND_SET){
        if (areaSet == 0)
            return 0;
        return ( value * areaCell/areaSet );
    }

    return value ;

}

/**
 * Get the sum of the values of the cells in the list
 * The value [0...1] in the GridCellInfo is taken as a multiplier to the value in the corresponding grid cell (e.g. if the cell value is 0.4 and the GridCellInfo (multiplier)value is 0.25, the calculated value for that cell will be 0.1)
 */
template<typename T>
template<typename K, typename>
long double NumericGridmap<T>::getCellsValueSum(const std::vector<K>& cells,
                                                bool checkForRepeatedCells,
                                                bool* _error_) const
{
    return getCellsComputedValue(cells, ComputedValueType::SUM, 0, checkForRepeatedCells, _error_);
}



/**
 * Get the data(values) of the cells in the list
 * @param area Area of the polygon (used only for value_type = AVERAGE_TOTAL)
 * @param _error_ (output) Indicates if there was an error in the calculation (e.g. index/point out of range).
 * @param in_norm If set to true, the returned value will be normalized. (iif the values are normalized).
 * @param value_type Type of desired return value
 * @return Value of the cells in the list
 */
template<typename T>
long double NumericGridmap<T>::getCellsComputedValue(const CellsRangeList& cells,
                                                     bool average,
                                                     bool* _error_) const {
    if(_error_) *_error_ = false;

    if ( !this->checkInternalParameters(true,__FUNCTION__) ){
        if(_error_) *_error_ = true;
        return 0;
    }

    if(cells.empty()){
        if(_error_) *_error_ = true;
        return 0;
    }

    int min_y, max_y, count = 0;
    long double value = 0;
    for (unsigned int x = cells.minX() ; x <= cells.maxX() ; ++x){
        auto yRanges = cells.getColumn(x);
        int lastY = -1;
        for(const auto& yRange : yRanges){
            min_y = yRange.first;
            max_y = yRange.second;
            if(lastY == min_y)
                ++min_y;
            lastY = max_y;
            for (int y = min_y ; y <= max_y ; ++y){
                bool errorTmp = true;

                if( !this->hasValue(x, y, &errorTmp) || errorTmp )
                    continue;

                value += this->get(x, y);
                ++count;
            }
        }
    }

    if(count == 0){
        if(_error_) *_error_ = true;
        return 0;
    }

    if(average)
        value = value / count;
    return value;
}

template<typename T>
bool NumericGridmap<T>::getStatistics(GridStatistics& stats) const{
    stats = GridStatistics();
    stats.avg = stats.sd = 0;
    stats.countCells = this->getSizeX() * this->getSizeY();
    bool errorTmp;
    T val;
    long double total = 0;
    for(size_t x = 0 ; x < this->getSizeX() ; ++x){
        for(size_t y = 0 ; y < this->getSizeY() ; ++y){
            if(!this->hasValue(x, y, &errorTmp) || errorTmp)
                continue;
            val = this->get(x, y);
            total += val;
            stats.min = std::min( stats.min , val);
            stats.max = std::max( stats.max , val);
            ++stats.validCells;
        }
    }

    if(stats.validCells > 0){
        stats.avg = total / stats.validCells;

        total = 0;
        for(size_t x = 0 ; x < this->getSizeX() ; ++x){
            for(size_t y = 0 ; y < this->getSizeY() ; ++y){
                if(!this->hasValue(x, y, &errorTmp) || errorTmp)
                    continue;
                val = this->get(x, y);
                total += ( (stats.avg - val) * (stats.avg - val) );
            }
        }
        stats.sd = std::sqrt( total / stats.validCells );
    }

    return true;
}

template<typename T>
bool NumericGridmap<T>::getStatistics(GridStatistics& stats, T filterMinValue, T filterMaxValue) const{

    stats = GridStatistics();
    stats.avg = stats.sd = 0;
    stats.countCells = this->getSizeX() * this->getSizeY();
    bool errorTmp;
    T val;
    long double total = 0;
    for(size_t x = 0 ; x < this->getSizeX() ; ++x){
        for(size_t y = 0 ; y < this->getSizeY() ; ++y){
            if(!this->hasValue(x, y, &errorTmp) || errorTmp)
                continue;
            val = this->get(x, y);

            if(val < filterMinValue || val > filterMaxValue)
                continue;

            total += val;
            stats.min = std::min( stats.min , val);
            stats.max = std::max( stats.max , val);
            ++stats.validCells;
        }
    }
    if(stats.validCells > 0){
        stats.avg = total / stats.validCells;

        total = 0;
        for(size_t x = 0 ; x < this->getSizeX() ; ++x){
            for(size_t y = 0 ; y < this->getSizeY() ; ++y){
                if(!this->hasValue(x, y, &errorTmp) || errorTmp)
                    continue;
                val = this->get(x, y);

                if(val < filterMinValue || val > filterMaxValue)
                    continue;

                total += ( (stats.avg - val) * (stats.avg - val) );
            }
        }
        stats.sd = std::sqrt( total / stats.validCells );
    }

    return true;
}


////------------------------------------
////----------------FILES---------------
////------------------------------------

/**
 * Loads the grid from a file.
 */
template<typename T>
bool NumericGridmap<T>::readGridFromFile(const std::string& filename)
{
    enum mapFileType {UNDEFINED,TIFFMAP,PNGMAP};
    std::string file_name = filename;

    mapFileType file_type = mapFileType::UNDEFINED;

    size_t index_extension;

    //check the type of file (.tif, .tiff, .png, or undefined)
    index_extension = file_name.find(".tif");
    if ( file_name.size() > 4 && index_extension == file_name.size()-4 )
        file_type = mapFileType::TIFFMAP;
    else {
        index_extension = file_name.find(".tiff");
        if ( file_name.size() > 5 && index_extension == file_name.size()-5 )
            file_type = mapFileType::TIFFMAP;
        else {
            index_extension = file_name.find(".png");
            if ( file_name.size() > 4 && index_extension == file_name.size()-4 )
                file_type = mapFileType::PNGMAP;
        }
    }

    if (file_type == mapFileType::UNDEFINED){//try to search the file by adding .tif and .png extension
        if ( fileExists(file_name + ".tif") )
            return readGridFromGeoTiff(file_name + ".tif");
        if ( fileExists(file_name + ".tiff") )
            return readGridFromGeoTiff(file_name + ".tiff");
        if ( fileExists(file_name + ".png") )
            return readGridFromPNG(file_name + ".png");
    }
    else if (file_type == mapFileType::TIFFMAP)
        return readGridFromGeoTiff(file_name);
    else if (file_type == mapFileType::PNGMAP)
        return readGridFromPNG(file_name);

    return false;
}

/**
 * Saves the grid as a PNG file (plus meta-file).
 */
template<typename T>
bool NumericGridmap<T>::saveGridAsPNG(const std::string& _filename, Range2RGBColorType colorType) const  {
    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    GridStatistics stats;
    if(!getStatistics(stats)){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error computing grid statistics.");
        return false;
    }

    std::string filename = _filename;
    std::string metadata = _filename;
    size_t index = filename.find(".png");
    if (index != std::string::npos) {
        metadata.replace(index, 4, ".meta");
    }
    else{
        filename += ".png";
        metadata += ".meta";
    }
    std::ofstream outMeta(metadata.c_str());
    if(!outMeta.is_open()){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating/opening file for metadata.");
        return false;
    }
    outMeta << std::setprecision(12) << this->getCellsize()
            << " " << this->m_layout.getMinPointX() << " " << this->m_layout.getMinPointY()
            << " " << stats.min << " " << stats.max
            << " " << unitToString(this->m_units) << " " << colorType << std::endl;
    outMeta.close();

    std::ofstream image_out(filename);
    if(!image_out.is_open()){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating/opening image file.");
        return false;
    }

    bool ok = saveGridAsPNG(image_out, colorType);
    image_out.close();

    if(ok)
        this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Grid saved successfully.");

    return ok;
}


/**
* Gets the grid as a PNG ostringstream.
*/
template<typename T>
bool NumericGridmap<T>::saveGridAsPNG(std::ostringstream &image_out, Range2RGBColorType colorType) const  {
    try{
         image_out.str(std::string());
         std::ostream& image_out_ref = image_out;
         return saveGridAsPNG(image_out_ref, colorType) && image_out.good();
    }
    catch (std::exception &e){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error converting grid to png: " + std::string( e.what() ) );
        return false;
    }
}

/**
 * Gets the grid as a PNG ostream.
*/
template<typename T>
bool NumericGridmap<T>::saveGridAsPNG(std::ostream &image_out, Range2RGBColorType colorType) const{
    try{
         if ( !this->checkInternalParameters(true,__FUNCTION__) )
             return false;

         GridStatistics stats;
         if(!getStatistics(stats)){
             this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error computing grid statistics.");
             return false;
         }

         //    if(grayscale){

         //        png::image< png::gray_pixel > image(this->getSizeX(), this->getSizeY());
         //        png::gray_pixel pixelVal;
         //        double value;
         //        bool errorTmp;

         //        for (unsigned int x=0; x < this->getSizeX(); x++) {
         //            for (unsigned int y=0; y < this->getSizeY(); y++) {
         //                pixelVal = 0;
         //                if(stats.min == stats.max)
         //                    pixelVal = std::numeric_limits<png::gray_pixel>::max();
         //                else if( this->hasValue(x, y, errorTmp) ){
         //                    value = getValue(x, y, errorTmp, m_isNormalized);
         //                    if(!errorTmp)
         //                        pixelVal = getGrayValue<png::gray_pixel>(value, stats.min, stats.max);
         //                }
         //                image[this->getSizeY()-y-1][x] = pixelVal;
         //            }
         //        }

         //        image.write(filename.c_str());

         //        this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Grid saved successfully.");

         //        return true;
         //    }

         png::image< png::rgba_pixel > image(this->getSizeX(), this->getSizeY());

         long double value;
         bool errorTmp;

         for (unsigned int x=0; x < this->getSizeX(); x++) {
             for (unsigned int y=0; y < this->getSizeY(); y++) {
                 png::rgba_pixel pixel;
                 pixel.alpha = 0;
                 if( this->hasValue(x, y, &errorTmp) && !errorTmp ){
                     value = this->get(x, y);
                     auto rgb = getRGBValue<png::byte>(value, stats.min, stats.max, colorType);
                     pixel.red = rgb.r;
                     pixel.green = rgb.g;
                     pixel.blue = rgb.b;
                     pixel.alpha = std::numeric_limits<png::byte>::max();
                 }
                 image[this->getSizeY()-y-1][x] = pixel;
             }
         }

         image.write_stream(image_out);

         return true;
    }
    catch (std::exception &e){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error converting grid to png: " + std::string( e.what() ) );
        return false;
    }
}

/**
 * Loads the grid from a PNG file.
 */
template<typename T>
bool NumericGridmap<T>::readGridFromPNG(const std::string& filename) {

    this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Reading grid from " + filename);
    std::string metadata = filename;
    size_t index = metadata.find(".png");
    if (index != std::string::npos) {
        metadata.replace(index, 4, ".meta");
    }
    std::ifstream inMeta(metadata.c_str());
    if (!inMeta.is_open()) {
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Cannot read from file " + metadata);
        return false;
    }

    this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Meta information read from " + metadata);
    double cellsize,dataMinX,dataMinY;
    std::string sUnit, sMinValue, sMaxValue;
    int iColorType;
    Unit unit = Unit::UNIT_CUSTOM;
    Range2RGBColorType colorType;
    inMeta >> cellsize >> dataMinX >> dataMinY >> sMinValue >> sMaxValue >> sUnit >> iColorType;
    inMeta.close();

    try{
        unit = stringToUnit(sUnit);
    }
    catch(...){
        if(sUnit.empty())
            this->logger().printOut(LogLevel::WARNING, __FUNCTION__, "No units given. Setting UNIT_CUSTOM as default.");
        else
            this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid unit. Setting UNIT_CUSTOM as default.");
    }

    try{
        colorType = intToRange2RGBColorType(iColorType);
    }
    catch(...){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid Range2RGBColorType. Setting HEAT as default.");
        colorType = Range2RGBColorType::COLOR_HEAT;
    }

    double minValue_meta = string2double(sMinValue);
    double maxValue_meta = string2double(sMaxValue);

    if (cellsize <= 0 ) {
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error in cellsize input parameter");
        return false;
    }

    std::ifstream image_in(filename.c_str());
    if (!image_in.is_open()) {
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Cannot read from file " + filename);
        return false;
    }

    return readGridFromPNG(image_in,
                           cellsize,
                           dataMinX,
                           dataMinY,
                           minValue_meta,
                           maxValue_meta,
                           colorType,
                           unit);
}


/**
* Sets the grid as from a PNG input stream.
*/
template<typename T>
bool NumericGridmap<T>::readGridFromPNG(std::istream& image_in,
                                        double cellsize,
                                        double dataMinX,
                                        double dataMinY,
                                        double minValue_meta,
                                        double maxValue_meta,
                                        Range2RGBColorType colorType,
                                        Unit units) {
    try{
        this->destroy();

        png::image< png::rgba_pixel > image;
        image.read_stream( image_in );

        image_in.seekg(0, image_in.beg);

        this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Generating grid with size: " + std::to_string(image.get_width())
                                                         + " x " + std::to_string(image.get_height()));

        int width = image.get_width();
        int height = image.get_height();
        if(width <= 0 || height <= 0){
            this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid image size");
            return false;
        };


        if( !this->m_layout.init(dataMinX, dataMinY, width, height, cellsize) ){
            this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error setting geometric layout");
            return false;
        }

        if ( !this->allocate() ){
            this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error allocating new grid");
            return false;
        };

        this->m_units = units;

        long double value;

        for (unsigned int x=0; x < this->getSizeX(); x++) {
            for (unsigned int y=0; y < this->getSizeY(); y++) {
                png::rgba_pixel pixelVal = image[this->getSizeY()-y-1][x];
                if( pixelVal.alpha < 0.5 * std::numeric_limits<png::byte>::max() ){
                    this->unSet(x, y);
                    continue;
                }

                RGBPixel<png::byte> pixel(pixelVal.red, pixelVal.green, pixelVal.blue);
                value = getValueFromRGB(pixel, minValue_meta, maxValue_meta, colorType);
                this->set_internal(x, y, value);
            }
        }

        this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Grid generated successfully.");

        return true;
    }
    catch (std::exception &e){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating grid from png: " + std::string( e.what() ) );
        return false;
    }
}

/**
* Sets the grid as from a PNG input stream.
*/
template<typename T>
bool NumericGridmap<T>::readGridFromPNG(std::istream& image_in,
                                        double dataMinX,
                                        double dataMinY,
                                        double dataMaxX,
                                        double dataMaxY,
                                        double minValue_meta,
                                        double maxValue_meta,
                                        Range2RGBColorType colorType,
                                        Unit units) {
    try{
        png::image< png::rgba_pixel > image;
        image.read_stream( image_in );
        image_in.seekg(0, image_in.beg);

        int width = image.get_width();
        int height = image.get_height();
        if(width <= 0 || height <= 0){
            this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid image size");
            return false;
        };

        double cellsize = 0.5 * ( (dataMaxX - dataMinX) / width + (dataMaxY - dataMinY) / height );

        return readGridFromPNG(image_in, cellsize, dataMinX, dataMinY, minValue_meta, maxValue_meta, colorType, units);
    }
    catch (std::exception &e){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating grid from png: " + std::string( e.what() ) );
        return false;
    }
}

/**
* Sets the grid from a PNG string.
*/
template<typename T>
bool NumericGridmap<T>::readGridFromPNG(const std::string& image_in,
                                                        double cellsize,
                                                        double dataMinX,
                                                        double dataMinY,
                                                        double minValue_meta,
                                                        double maxValue_meta,
                                                        Range2RGBColorType colorType,
                                                        Unit units) {
    try{
        if ( image_in.empty() )
           return false;

        std::istringstream stream;
        stream.str( image_in );
        return readGridFromPNG(stream,
                              cellsize,
                              dataMinX,
                              dataMinY,
                              minValue_meta,
                              maxValue_meta,
                              colorType,
                              units);
    }
    catch (std::exception &e){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating grid from png: " + std::string( e.what() ) );
        return false;
    }
}

/**
* Sets the grid from a PNG string.
*/
template<typename T>
bool NumericGridmap<T>::readGridFromPNG(const std::string& image_in,
                                        double dataMinX,
                                        double dataMinY,
                                        double dataMaxX,
                                        double dataMaxY,
                                        double minValue_meta,
                                        double maxValue_meta,
                                        Range2RGBColorType colorType,
                                        Unit units) {
    try{
        if ( image_in.empty() )
           return false;

        std::istringstream stream;
        stream.str( image_in );
        return readGridFromPNG(stream,
                               dataMinX,
                               dataMinY,
                               dataMaxX,
                               dataMaxY,
                               minValue_meta,
                               maxValue_meta,
                               colorType,
                               units);
    }
    catch (std::exception &e){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating grid from png: " + std::string( e.what() ) );
        return false;
    }
}

/**
* Gets the grid as a PNG ostringstream.
*/
template<typename T>
bool NumericGridmap<T>::readGridFromPNG(const char* image_in,
                                                        unsigned int size,
                                                        double cellsize,
                                                        double dataMinX,
                                                        double dataMinY,
                                                        double minValue_meta,
                                                        double maxValue_meta,
                                                        Range2RGBColorType colorType,
                                                        Unit units) {

    try{
        if ( !image_in || size == 0 )
           return false;

        //we could do the same as the setGridfromPNG (string,...) to avoid creating a copy with all the data, but for some reason the istringstream.read(image_in,size) is not working

        return readGridFromPNG( std::string(image_in, size),
                               cellsize,
                               dataMinX,
                               dataMinY,
                               minValue_meta,
                               maxValue_meta,
                               colorType,
                               units );
    }
    catch (std::exception &e){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating grid from png: " + std::string( e.what() ) );
        return false;
    }
}

/**
* Gets the grid as a PNG ostringstream.
*/
template<typename T>
bool NumericGridmap<T>::readGridFromPNG(const char* image_in,
                                                         unsigned int size,
                                                         double dataMinX,
                                                         double dataMinY,
                                                         double dataMaxX,
                                                         double dataMaxY,
                                                         double minValue_meta,
                                                         double maxValue_meta,
                                                         Range2RGBColorType colorType,
                                                         Unit units) {

    try{
        if ( !image_in || size == 0 )
           return false;

        //we could do the same as the setGridfromPNG (string,...) to avoid creating a copy with all the data, but for some reason the istringstream.read(image_in,size) is not working

        return readGridFromPNG( std::string(image_in, size),
                                dataMinX,
                                dataMinY,
                                dataMaxX,
                                dataMaxY,
                                minValue_meta,
                                maxValue_meta,
                                colorType,
                                units );
    }
    catch (std::exception &e){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating grid from png: " + std::string( e.what() ) );
        return false;
    }
}

/**
 * Saves the grid as a TIFF file.
 */
template<typename T>
bool NumericGridmap<T>::saveGridAsGeoTiff(const std::string& _filename, int utmZone) const  {

    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    std::string filename = _filename;
    if (filename.rfind(".tif") != filename.size()-4
            && filename.rfind(".tiff") != filename.size()-5)
        filename += ".tif";

    const char *pszFormat = "GTiff";
    GDALDriver *poDriver;
    GDALAllRegister();
    poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
    if( poDriver == NULL ) {
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Cannot load driver for writing GeoTiff with gdal.");
        return false;
    }

    GDALDataset *poDstDS;
    char **papszOptions = NULL;
    poDstDS = poDriver->Create( filename.c_str(),
                                this->getSizeX(),
                                this->getSizeY(),
                                1,
                                GDT_Float64,
                                papszOptions );
    if(!poDstDS){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating GDAL dataset.");
        return false;
    }

    double adfGeoTransform[6] = { this->m_layout.getMinPointX(),
                                  this->getCellsize(),
                                  0,
                                  this->m_layout.getMaxPointY(),
                                  0,
                                  -this->getCellsize()};
    poDstDS->SetGeoTransform( adfGeoTransform );

    char *pszSRS_WKT = NULL;
    OGRSpatialReference oSRS;

    oSRS.SetWellKnownGeogCS( "WGS84" );

    if(utmZone < 0){
        oSRS.SetProjection("longlat");
        oSRS.SetWellKnownGeogCS("EPSG:32632");
    }
    else{
        oSRS.SetUTM( utmZone, TRUE );
        oSRS.exportToWkt( &pszSRS_WKT );
        poDstDS->SetProjection( pszSRS_WKT );
        CPLFree( pszSRS_WKT );
    }

    GDALRasterBand *poBand = poDstDS->GetRasterBand(1);

    typedef double TTif; //@TODO can the values in the tiff be different than doubles?
    GDALDataType vDataType = GDT_Float64;
    TTif line[this->getSizeX()];

    const TTif noDataValue = std::nan("1");
    TTif value;
    long double minValue = std::numeric_limits<TTif>::lowest();
    long double maxValue = std::numeric_limits<TTif>::max();
    bool errorTmp;

    for (unsigned int y=0; y < this->getSizeY(); ++y) {
        for (unsigned int x=0; x < this->getSizeX(); ++x) {
            if( !this->hasValue(x, y, &errorTmp) || errorTmp )
                value = noDataValue;
            else
                value = std::min(  std::max( static_cast<long double>( this->get(x, y) ) , minValue ) , maxValue );

            line[x] = value;
        }
        CPLErr cplerror = poBand->RasterIO( GF_Write, 0, this->getSizeY()-1-y, this->getSizeX(), 1,
                                            line, this->getSizeX(), 1, vDataType, 0, 0 );
        if(cplerror == CE_Failure || cplerror == CE_Fatal)
            this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error writing band " + std::to_string(y) + ": CPLErr = " + std::to_string(cplerror));
    }

    poBand->SetMetadataItem("Units", unitToString(this->m_units).c_str());

    poBand->SetNoDataValue(noDataValue);

    GDALClose( (GDALDatasetH) poDstDS );

    this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Grid saved successfully.");
    return true;
}


/**
 * Saves the grid as a GEOTIFF string.
 */
template<typename T>
bool NumericGridmap<T>::saveGridAsGeoTiffString(std::string& data, int utmZone) const {

    this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Saving grid as GeoTiff string...");
    std::string vFilename = "/vsimem/__tiffdata__" + std::to_string( (long)this ) + ".tif";

    VSILFILE * vsilfileHandler = VSIFOpenL(vFilename.c_str(),"w");
    if( !vsilfileHandler ){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating virtual file");
        return false;
    }

    if( !saveGridAsGeoTiff(vFilename, utmZone) ){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error saving virtual file");
        VSIFCloseL(vsilfileHandler);
        return false;
    }

    VSIFCloseL(vsilfileHandler);

    vsi_l_offset dataLength, maxLength = data.max_size();
    GByte* cData = VSIGetMemFileBuffer(vFilename.c_str(), &dataLength, true);
    data.clear();

    if( dataLength > maxLength ){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "File is too big to be saved in a string");
        return false;
    }
    data.append( (char*)cData, dataLength);

    delete[] cData;
    return true;
}


/**
 * Loads the grid from a TIFF file.
 */
template<typename T>
bool NumericGridmap<T>::readGridFromGeoTiff(const std::string& filename) {

    this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Reading grid from " + filename);
//        std::ifstream in(filename.c_str());
//        if (!in) {
//            std::cerr << "ERROR: Cannot read from file " << filename << std::endl;
//            return false;
//        }

    GDALDataset  *poDataset;
    GDALAllRegister();
    poDataset = (GDALDataset *) GDALOpen( filename.c_str(), GA_ReadOnly );
    if( poDataset == NULL ) {
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Cannot read from file " + filename);
        return false;
    }

    double adfGeoTransform[6];
//        printf( "Driver: %s/%s\n",
//                poDataset->GetDriver()->GetDescription(),
//                poDataset->GetDriver()->GetMetadataItem( GDAL_DMD_LONGNAME ) );
//        if( poDataset->GetProjectionRef()  != NULL )
//            printf( "Projection is `%s'\n", poDataset->GetProjectionRef() );
    if( poDataset->GetGeoTransform( adfGeoTransform ) != CE_None ) {
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Could not get GeoTransform");
        return false;
    }

    if (poDataset->GetRasterCount() != 1)
        this->logger().printOut(LogLevel::WARNING, __FUNCTION__, "Warning: Dataset has " + std::to_string(poDataset->GetRasterCount()) + " raster bands");


    GDALRasterBand *poBand;
    poBand = poDataset->GetRasterBand(1);

    const char *unit_s = poBand->GetMetadataItem("Units");

    int xSize = poBand->GetXSize();
    int ySize = poBand->GetYSize();
    if (xSize != poDataset->GetRasterXSize() || ySize != poDataset->GetRasterYSize()) {
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Band size different from raster size");
        return false;
    }

    double cellsize = adfGeoTransform[1]; // x size
    double dataMinX = adfGeoTransform[0];
    double dataMaxX = dataMinX + xSize * cellsize;
    double dataMaxY = adfGeoTransform[3];
    double dataMinY = dataMaxY - ySize * cellsize;

    if (xSize <= 0 || ySize <= 0 || cellsize <= 0 ) {
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error in input parameters (sizeX,sizeY,cellsize)");
        return false;
    }
    if (dataMinX >= dataMaxX || dataMinY >= dataMaxY){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error in min/max input parameters");
        return false;
    }

    this->destroy();

    if( !this->m_layout.init(dataMinX, dataMinY, xSize, ySize, cellsize) ){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error setting geometric layout");
        return false;
    }

    if ( !this->allocate() ){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error allocating new grid");
        return false;
    };

    this->m_units = Unit::UNIT_CUSTOM;
    if(unit_s){
        try{
            this->m_units = stringToUnit(unit_s);
        }
        catch(...){
            this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid unit. Setting UNIT_CUSTOM as default.");
        }
    }
    else
        this->logger().printOut(LogLevel::WARNING, __FUNCTION__, "No units given. Setting UNIT_CUSTOM as default.");

    typedef double TTif; //@TODO can the values in the tiff be different than doubles?
    GDALDataType vDataType = GDT_Float64;

    double *pafScanline;
    pafScanline = (TTif *) CPLMalloc(sizeof(TTif)*xSize);

    long double minValue = std::numeric_limits<T>::lowest();
    long double maxValue = std::numeric_limits<T>::max();

    int hasNoDataValue;
    auto noDataValue = poBand->GetNoDataValue(&hasNoDataValue);

    for (int y=0; y < ySize; ++y) {
        CPLErr cplerror = poBand->RasterIO( GF_Read, 0, ySize-1-y, xSize, 1,
                                            pafScanline, xSize, 1, vDataType,
                                            0, 0 );
        if(cplerror == CE_Failure || cplerror == CE_Fatal){
            this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error reading band " + std::to_string(y) + ": CPLErr = " + std::to_string(cplerror));
            continue;
        }

        for (int x=0; x < xSize; ++x) {
            auto valIn = pafScanline[x];
            //if (pafScanline[x] < 0)  value = 0; else
            if( hasNoDataValue
                 && ( valIn == noDataValue || std::isnan(valIn) ) )
                this->unSet(x,y);
            else{
                set_internal(x, y, valIn, minValue, maxValue);
            }

//                if(std::isnan(pafScanline[x]))
//                     std::cout << "[" << x << "," << y << "] = "
//                               << pafScanline[x] << " :: " << value << " :: " << (float)m_grid[x][y] << std::endl;

        }

    }
    CPLFree(pafScanline);
    GDALClose( (GDALDatasetH) poDataset);

    this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Grid generated successfully.");
    return true;
}



/**
 * Loads the grid from a GEOTIFF string.
 */
template<typename T>
bool NumericGridmap<T>::readGridFromGeoTiffString(const std::string& data, bool isInWGS) {

    this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Reading grid from GeoTiff string");

    if(isInWGS){
        int ret;
        std::string fileNameIn = "/tmp/__tiffdata__" + std::to_string( (long)this ) + "_in.tif";
        std::ofstream fileIn(fileNameIn, std::ofstream::binary);
        if(!fileIn.is_open())
            return false;
        fileIn.write(data.c_str(), data.size());
        fileIn.close();

        std::string fileNameOut = "/tmp/__tiffdata__" + std::to_string( (long)this ) + "_out.tif";
        std::string command = "gdalwarp -t_srs '+proj=utm +zone=32U +datum=WGS84' -overwrite " + fileNameIn + " " + fileNameOut;
        ret = system(command.c_str());

        bool success = readGridFromGeoTiff(fileNameOut);

        command = "rm " + fileNameIn;
        ret = system(command.c_str());
        command = "rm " + fileNameOut;
        ret = system(command.c_str());

        return success;
    }
    else{
        std::string vFilename = "/vsimem/__tiffdata__" + std::to_string( (long)this ) + ".tif";

        unsigned char* cData = new unsigned char[data.size()];
        memcpy(cData, data.c_str(), data.size());

        VSILFILE * vsilfileHandler = VSIFileFromMemBuffer(vFilename.c_str(), cData, data.size(), false);
        if( !vsilfileHandler ){
            this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating virtual file");
            return false;
        }

        if( !readGridFromGeoTiff(vFilename) ){
            this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error reading virtual file");
            return false;
        }

        VSIFCloseL(vsilfileHandler);

        if( VSIUnlink(vFilename.c_str()) != 0 )
            this->logger().printOut(LogLevel::WARNING, __FUNCTION__, "Error deleting the virtual file.");

        delete[] cData;

        return true;

    }

}

/**
 * Saves the grid as a PPM file (plus meta-file).
 */
template<typename T>
bool NumericGridmap<T>::saveGridAsPPM(const std::string& filename) const  {
    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    GridStatistics stats;
    if(!getStatistics(stats)){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error computing grid statistics.");
        return false;
    }

    this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Saving grid as " + filename);

    std::ofstream out(filename.c_str());
    if (!out) {
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Cannot write to file " + filename);
        return false;
    }

    std::string metadata = filename;
    size_t index = metadata.find(".ppm");
    if (index != std::string::npos) {
        metadata.replace(index, 4, ".meta");
    }
    std::ofstream outMeta(metadata.c_str());
    outMeta << std::setprecision(12) << this->getCellsize()
            << " " << this->m_layout.getMinPointX() << " " << this->m_layout.getMinPointY()
            << " " << stats.min << " " << stats.max
            << " " << unitToString(this->m_units) << std::endl;
    outMeta.close();

    typedef unsigned char TPPM; // unsigned char for gray scale

    TPPM valueOut;
    long double minValue = std::numeric_limits<TPPM>::lowest();
    long double maxValue = std::numeric_limits<TPPM>::max();
    long double deltaVal = stats.max - stats.min;

    out << "P3" << std::endl
        << "# creator: AroLib" << std::endl
        << this->getSizeX() << " " << this->getSizeY() << std::endl
        << std::numeric_limits<TPPM>::max() << std::endl;

    for (unsigned int y=this->getSizeY()-1; y+1 > 0 ; y--) {
        for (unsigned int x=0; x < this->getSizeX(); x++) {
            valueOut = 0;
            if( this->isSet(x, y) ){
                long double val = this->get(x, y);
                if(deltaVal > 0){
                    val = (val - stats.min) / deltaVal;
                    valueOut = std::min( maxValue, std::max(minValue, deltaVal * maxValue) );
                }
                else
                    valueOut = std::numeric_limits<TPPM>::max();
            }
            out << valueOut << " " << valueOut << " " << valueOut << " ";
        }
        out << std::endl;
    }
    out.close();
    out.clear();

    this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Grid saved successfully.");
    return true;
}

/**
 * Loads the grid from a PPM file.
 */
template<typename T>
bool NumericGridmap<T>::readGridFromPPM(const std::string& filename) {

    this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Reading grid from " + filename);

    std::ifstream in(filename.c_str());
    if (!in) {
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Cannot read from file " + filename);
        return false;
    }


    std::string metadata = filename;
    size_t index = metadata.find(".ppm");
    if (index != std::string::npos) {
        metadata.replace(index, 4, ".meta");
    }
    std::ifstream inMeta(metadata.c_str());
    if (!inMeta) {
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Cannot read from file " + metadata);
        return false;
    }
    double cellsize,dataMinX,dataMinY,dataMaxX,dataMaxY;
    std::string sUnit, sMinValue, sMaxValue;
    Unit unit = Unit::UNIT_CUSTOM;
    inMeta >> cellsize >> dataMinX >> dataMinY >> sMinValue >> sMaxValue >> sUnit;
    inMeta.close();

    double minValue_meta = string2double(sMinValue);
    double maxValue_meta = string2double(sMaxValue);
    long double deltaVal = maxValue_meta - minValue_meta;


    try{
        unit = stringToUnit(sUnit);
    }
    catch(...){
        if(sUnit.empty())
            this->logger().printOut(LogLevel::WARNING, __FUNCTION__, "No units given. Setting UNIT_CUSTOM as default.");
        else
            this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid unit. Setting UNIT_CUSTOM as default.");
    }

    this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Meta information: \n\tCellsize = " + std::to_string(this->getCellsize())
                      + ", min = (" + std::to_string(this->m_layout.getMinPointX())
                      + ", " + std::to_string(this->m_layout.getMinPointY())
                      + "), max = (" + std::to_string(this->m_layout.getMaxPointX())
                      + ", " + std::to_string(this->m_layout.getMaxPointY()) + ")", 12);

    if (cellsize <= 0 ) {
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error in cellsize input parameter");
        return false;
    }
    if (dataMinX >= dataMaxX || dataMinY >= dataMaxY){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error in min/max input parameters");
        return false;
    }


    std::string line;
    while (std::getline(in, line) && (line.empty() || line.front() == '#') ){
        line.clear();
    }
    if(line != "P3"){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Expected format P3. Got '" + line + "' instead.");
        return false;
    }
    line.clear();
    while (std::getline(in, line) && (line.empty() || line.front() == '#') ){
        line.clear();
    }
    if(line.empty()){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid file.");
        return false;
    }

    auto indSep = line.find(" ");
    if(indSep == std::string::npos){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid file (size)");
        return false;
    }

    int sizeX = std::stoi( line.substr(0, indSep) );
    int sizeY = std::stoi( line.substr(indSep+1) );

    if (sizeX <= 0 || sizeY <= 0){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error in input parameters (sizeX,sizeY)");
        return false;
    }

    this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Generating grid with size: " + std::to_string(this->getSizeX())
                                                     + " x " + std::to_string(this->getSizeY()));

    this->destroy();

    if( !this->m_layout.init(dataMinX, dataMinY, sizeX, sizeY, cellsize) ){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error setting geometric layout");
        return false;
    }

    if ( !this->allocate() ){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error allocating new grid");
        return false;
    };

    this->m_units = unit;

    long double value;
    long double minValue = std::numeric_limits<T>::lowest();
    long double maxValue = std::numeric_limits<T>::max();

    int g;
    for (unsigned int y=this->getSizeY()-1; y+1 > 0; y--) {
        for (unsigned int x=0; x < this->getSizeX(); x++) {
            if(deltaVal > 0){
                in >> g >> g >> g; // redundant, due to grayscale values
                value = g;
                value *= ( deltaVal / 255.0 );
            }
            else
                value = minValue_meta;
            set_internal(x, y, value, minValue, maxValue);
        }
    }

    in.close();

    this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Grid generated successfully.");
    return true;
}



/**
 * Saves the grid values in CSV matrix.
 */
template<typename T>
bool NumericGridmap<T>::saveValuesInCSV(const std::string& _filename, const std::string& sep) const{

    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    std::string filename = _filename;
    if (filename.rfind(".csv") != filename.size()-4)
        filename += ".csv";

    std::ofstream fileout(filename);
    if(!fileout.is_open()){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Unable to open/create output file.");
        return false;
    }
    fileout << std::setprecision(10);


    for (unsigned int x=0; x < this->getSizeX(); ++x){
        double xFrom = this->m_layout.getMinPointX() + x * this->getCellsize();
        double xTo = xFrom + this->getCellsize();
        fileout << sep << "x=" << x << " (" << xFrom << "~" << xTo << ")";;
    }

    bool errorTmp;
    for (unsigned int y=0; y < this->getSizeY(); ++y) {
        size_t yRev = this->getSizeY() - y - 1;
        double yFrom = this->m_layout.getMinPointY() + yRev * this->getCellsize();
        double yTo = yFrom + this->getCellsize();
        fileout << std::endl << "y=" << yRev << " (" << yFrom << "~" << yTo << ")";
        for (unsigned int x=0; x < this->getSizeX(); ++x) {
            if( !this->hasValue(x, yRev, &errorTmp) || errorTmp )
                fileout << sep;
            else{
                double value = this->get(x, yRev);
                fileout << sep << value;
            }
        }
    }

    fileout.close();

    this->logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Grid values saved successfully.");
    return true;
}


/**
 * Checks if a given file exists.
 */
template<typename T>
bool NumericGridmap<T>::fileExists(const std::string& filename)
{
    static struct stat64 fileinfo;
    return(stat64(filename.c_str(), &fileinfo) == 0);
}


////------------------------------------
////----------PROTECTED METHODS---------
////------------------------------------


template<typename T>
bool NumericGridmap<T>::set_internal(unsigned int x,
                                     unsigned int y,
                                     long double _value){

    if( !std::is_floating_point<T>::value ){
        _value = std::round(_value) + 1e-1;
        return this->set(x, y, (T)_value);
    }
    return this->set(x, y, _value);
}

template<typename T>
bool NumericGridmap<T>::set_internal(unsigned int x,
                                     unsigned int y,
                                     long double _value,
                                     long double min,
                                     long double max){

    _value = std::max( min, std::min( max, _value ) );
    return this->set_internal(x, y, _value);
}

}
}


#endif //_AROLIB_GRIDMAP_NUMERIC_TCC_

