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
 

#ifndef _AROLIB_GRIDMAP_TCC_
#define _AROLIB_GRIDMAP_TCC_

#include "arolib/cartography/gridmap.hpp"

namespace arolib {
namespace gridmap {

 /**
 * Constructor.
 * Already allocates the grid with the given size.
 * @param x Number of columns of the grid.
 * @param y Number of rows of the grid.
 */
template<typename T>
Gridmap<T>::Gridmap(const GridmapLayout &lo, LogLevel logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_layout(lo),
    m_allocated(false)
{
    m_layout.logger().setParent(loggerPtr());

    if(m_layout.getSizeX() == 0 || m_layout.getSizeY() == 0)
        m_layout.setSize(0,0);

    if (m_layout.isValid()){
        allocate();
    }
    m_layout.setComputeInMultiThread(m_computeInMultiThread);
}

/**
 * Constructor.
 * The grid is not allocated yet (it doesn't exist).
 */
template<typename T>
Gridmap<T>::Gridmap(LogLevel logLevel):
  LoggingComponent(logLevel, __FUNCTION__),
  m_allocated(false) {
    m_layout.logger().setParent(loggerPtr());
}

/**
 * Copy constructor.
 */
template<typename T>
Gridmap<T>::Gridmap(const Gridmap<T>& other):
      LoggingComponent(other),
      m_allocated(false)
{
    m_layout = other.m_layout;
    m_layout.logger().setParent(loggerPtr());

    m_units = other.m_units;
    m_computeInMultiThread = other.m_computeInMultiThread;
    if(!other.isAllocated())
        return;
    try{
        if ( allocate() ){
            for (unsigned int x = 0 ; x < m_layout.getSizeX(); x++){
                for (unsigned int y = 0 ; y < m_layout.getSizeY(); y++){
                    if(other.isSet(x, y))
                        set( x, y, other.get(x, y) );
//                    else
//                        unSet(x, y); //should be initialized unset during allocation
                }
            }
        }
    }
    catch (...){
        destroy();
    }
}

/**
 * Destructor.
 */
template<typename T>
Gridmap<T>::~Gridmap()
{
    destroy();
}

//------------------------------------
//--------------OPERATORS-------------
//------------------------------------

// copy assignment
template<typename T>
Gridmap<T>& Gridmap<T>::operator=(const Gridmap<T>& other){
    copyFrom(other, false);
    return *this;
}

template<typename T>
template<typename K, typename>
bool Gridmap<T>::operator ==(const Gridmap<K>& other) const{
    return equals(other, [](const T &v1, const T &v2)->bool {return v1 == v2;});
}

template<typename T>
bool Gridmap<T>::equals(const Gridmap<T>& other, const std::function<bool(const T&, const T&)>& isDataEqual) const{
    if ( m_layout.getSizeX() != other.m_layout.getSizeX() ||
         m_layout.getSizeY() != other.m_layout.getSizeY() ||
         m_layout.getCellsize() != other.m_layout.getCellsize() ||
         m_layout.getMaxPointX() != other.m_layout.getMaxPointX() ||
         m_layout.getMaxPointY() != other.m_layout.getMaxPointY() ||
         m_layout.getMinPointX() != other.m_layout.getMinPointX() ||
         m_layout.getMinPointY() != other.m_layout.getMinPointY() ||
         m_units != other.m_units )
        return false;

    for (unsigned int x=0; x<m_layout.getSizeX(); x++){
        for (unsigned int y=0; y<m_layout.getSizeY(); y++){
            if( !isEqual( m_grid[x][y], other.m_grid[x][y], isDataEqual ) )
                return false;
        }
    }
    return true;
}

/**
* Check if the grid has the same geometry (bounding box and location) as another grid.
*/
template<typename T>
bool Gridmap<T>::equalGeometry(const Gridmap<T>& other) const{
    return m_layout == other.m_layout;
}


//------------------------------------
//--------------SET-PARAM-------------
//------------------------------------

/**
 * De-allocates the grid.
 */
template<typename T>
void Gridmap<T>::destroy()
{
    if (!m_allocated)
        return;

    for(unsigned int x = 0; x < m_layout.getSizeX(); ++x)
        delete[] m_grid[x];
    delete[] m_grid;

    m_allocated = false;
}

/**
 * Creates a new grid by setting the limits and resolution allocating the needed memory. If the grid was allocated, it is destroyed.
 */
template<typename T>
bool Gridmap<T>::createGrid(double minX,
                            double maxX,
                            double minY,
                            double maxY,
                            double cellsize,
                            const T *cellValue){
    GridmapLayout lo;
    if(minX >= maxX || minY >= maxY)
        return false;
    if(cellsize <= 0)
        return false;

    unsigned int sizeX = std::ceil( (maxX - minX)/cellsize );
    unsigned int sizeY = std::ceil( (maxY - minY)/cellsize );

    if(!lo.init(minX, minY, sizeX, sizeY, cellsize))
        return false;

    return createGrid(lo, cellValue);
}

template<typename T>
bool Gridmap<T>::createGrid(const GridmapLayout& lo,
                            const T *cellValue){
    if(!lo.isValid())
        return false;

    m_layout = lo;

    if(isAllocated())
        destroy();

    if(!allocate())
        return false;

    if(cellValue){
        for(size_t x = 0 ; x < m_layout.getSizeX() ; ++x){
            for(size_t y = 0 ; y < m_layout.getSizeY() ; ++y){
                set(x, y, cellValue);
            }
        }
    }

    return true;
}

/**
 * Creates a new grid by setting the limits and resolution allocating the needed memory. If the grid was allocated, it is destroyed.
 */
template<typename T>
bool Gridmap<T>::copyFrom(const Gridmap<T>& other, bool clearValues){
    if (this != &other) {
        destroy();

        m_logger = other.m_logger;
        m_layout = other.m_layout;
        m_units = other.m_units;

        if(!other.isAllocated())
            return true;

        if( !allocate() )
            return false;

        if(!clearValues){
            for (unsigned int x = 0 ; x < m_layout.getSizeX(); x++){
                for (unsigned int y = 0 ; y < m_layout.getSizeY(); y++){
                    if(other.isSet(x, y))
                        set(x, y, other.get(x, y));
//                    else
//                        unSet(x, y); //should be initialized unset during allocation
                }
            }
        }
    }
    return true;
}

/**
 * Set the lower limits (in real world coordinates) of the x- and y- axis of the grid, and recalculates the upper axis limits.
 */
template<typename T>
bool Gridmap<T>::setPointLimits_min(double minX, double minY, double cellsize)
{
    if (cellsize <= 0)
        cellsize = m_layout.getCellsize();
    return m_layout.init(minX, minY, m_layout.getSizeX(), m_layout.getSizeY(), cellsize);
}


/**
 * Creates the grid from a given polygon.
 */
template<typename T>
bool Gridmap<T>::convertPolygonToGrid(const std::vector<Point>& boundary,
                                      double resolution,
                                      const T *_val,
                                      bool only_perimeter){

    if (resolution <= 0){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The resolution must be positive");
        return false;
    }
    if (boundary.size() < 3){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The boundary must have at least 3 different points");
        return false;
    }

    // check the dimensions / bounding box:
    double xMin = std::numeric_limits<double>::max();
    double yMin = std::numeric_limits<double>::max();
    double xMax = std::numeric_limits<double>::lowest();
    double yMax = std::numeric_limits<double>::lowest();

    for (size_t i=0; i < boundary.size(); i++) {
        if (boundary[i].x < xMin) xMin = boundary[i].x;
        if (boundary[i].x > xMax) xMax = boundary[i].x;
        if (boundary[i].y < yMin) yMin = boundary[i].y;
        if (boundary[i].y > yMax) yMax = boundary[i].y;
    }

    if (xMin == xMax || yMin == yMax){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The boundary must have at least 3 different points which don't belong to the same line");
        return false;
    }

    if(!createGrid(xMin, xMax, yMin, yMax, resolution))
        return false;

    if(_val){
        if ( !only_perimeter ){
            Polygon poly;
            poly.points = boundary;
            arolib::geometry::correct_polygon(poly);
            setPolygon(poly, _val);
        }
        else{
            double width0 = 0;
            for (size_t i=0; i+1 < boundary.size(); i++){
                setLine(boundary.at(i), boundary.at(i+1), width0, _val);
            }
        }
    }

    return true;
}

/**
 * Takes the current grid and expands it (if necesary) to include the given polygon.
 */
template<typename T>
bool Gridmap<T>::expandGridFromPolygon(const std::vector<Point>& boundary,
                                       const T *_val,
                                       bool only_perimeter,
                                       bool reduceToPolygon){
    if(!checkInternalParameters(true, __FUNCTION__))
        return false;

    if (boundary.size() < 3){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The boundary must have at least 3 different points");
        return false;
    }

    // check the dimensions / bounding box:
    double xMin = std::numeric_limits<double>::max();
    double yMin = std::numeric_limits<double>::max();
    double xMax = std::numeric_limits<double>::lowest();
    double yMax = std::numeric_limits<double>::lowest();

    for (size_t i=0; i < boundary.size(); i++) {
        if (boundary[i].x < xMin) xMin = boundary[i].x;
        if (boundary[i].x > xMax) xMax = boundary[i].x;
        if (boundary[i].y < yMin) yMin = boundary[i].y;
        if (boundary[i].y > yMax) yMax = boundary[i].y;
    }

    if (xMin == xMax || yMin == yMax){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The boundary must have at least 3 different points which don't belong to the same line");
        return false;
    }

    if (xMin >= m_layout.getMaxPointX() || xMax <= m_layout.getMinPointX() || yMin >= m_layout.getMaxPointY() || yMax <= m_layout.getMinPointY()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The boundary must lie at least partially inside the grid's boundary");
        return false;
    }

    if ( std::fabs(xMin - m_layout.getMinPointX()) < 0.00001 ) xMin = m_layout.getMinPointX();
    if ( std::fabs(xMax - m_layout.getMaxPointX()) < 0.00001 ) xMax = m_layout.getMaxPointX();
    if ( std::fabs(yMin - m_layout.getMinPointY()) < 0.00001 ) yMin = m_layout.getMinPointY();
    if ( std::fabs(yMax - m_layout.getMaxPointY()) < 0.00001 ) yMax = m_layout.getMaxPointY();

    int cellsLeft = 0, cellsRight = 0, cellsDown = 0, cellsUp = 0;

    cellsLeft  = std::ceil( (m_layout.getMinPointX() - xMin)/m_layout.getCellsize() );
    cellsRight = std::ceil( (xMax - m_layout.getMaxPointX())/m_layout.getCellsize() );
    cellsDown  = std::ceil( (m_layout.getMinPointY() - yMin)/m_layout.getCellsize() );
    cellsUp    = std::ceil( (yMax - m_layout.getMaxPointY())/m_layout.getCellsize() );

    if (!reduceToPolygon){
        cellsLeft  = std::max(cellsLeft,0);
        cellsRight = std::max(cellsRight,0);
        cellsDown  = std::max(cellsDown,0);
        cellsUp    = std::max(cellsUp,0);
    }

    if (cellsLeft == 0 && cellsRight == 0 && cellsDown == 0 && cellsUp == 0)
        return true;

    auto gridOld = *(this);
    unsigned int sizeXOld = m_layout.getSizeX();
    unsigned int sizeYOld = m_layout.getSizeY();
    double xMinOld = m_layout.getMinPointX();
    double yMinOld = m_layout.getMinPointY();

    unsigned int sizeX = m_layout.getSizeX() + cellsLeft + cellsRight;
    unsigned int sizeY = m_layout.getSizeY() + cellsDown + cellsUp;

    destroy();

    if(reduceToPolygon){
        if( !m_layout.init( xMinOld - m_layout.getCellsize()*cellsLeft,
                            yMinOld - m_layout.getCellsize()*cellsDown,
                            sizeX, sizeY, m_layout.getCellsize() ) ){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error setting new layout");
            *(this) = gridOld;
            return false;
        }
    }
    else{
        if( !m_layout.init( std::min( xMinOld - m_layout.getCellsize()*cellsLeft  , gridOld.m_layout.getMinPointX() ),
                            std::min( yMinOld - m_layout.getCellsize()*cellsDown  , gridOld.m_layout.getMinPointY() ),
                            sizeX, sizeY, m_layout.getCellsize() ) ){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error setting new layout");
            *(this) = gridOld;
            return false;
        }
    }

    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Allocate new grid of size " + std::to_string(sizeX) + " x " + std::to_string(sizeY));
    if (!allocate()) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Unable to allocate grid. The new grid might be too large");
        *(this) = gridOld;
        return false;
    }

    if ( !only_perimeter ){
        Polygon poly;
        poly.points = boundary;
        arolib::geometry::correct_polygon(poly);
        setPolygon(poly, _val);
    }
    else{
        double width0 = 0;
        for (size_t i=0; i+1 < boundary.size(); i++){
            setLine(boundary.at(i), boundary.at(i+1), width0, _val);
        }
    }

    //copy previous values into new grid
    unsigned int x1 = 0, y1 = 0, x2 = sizeXOld, y2 = sizeYOld;
    if (reduceToPolygon){
        x1 = std::max( -cellsLeft, 0 );
        y1 = std::max( -cellsDown, 0 );
        x2 = sizeXOld - std::max( -cellsRight, 0 );
        y2 = sizeYOld - std::max( -cellsUp, 0 );

    }

    for (unsigned int  x = x1 ; x < x2 ; ++x){
        for (unsigned int  y = y1 ; y < y2 ; ++y){
            if( gridOld.isSet(x, y) )
                set( x+cellsLeft , y+cellsDown , gridOld.get(x, y) );
            else
                unSet( x+cellsLeft , y+cellsDown );
        }
    }

    return true;

}

/**
 * Takes the current grid and reduces it (if necesary) so that only cells that lie under the polygon are kept.
 * The values of the new cells that lie inside the polygon will be set to the desired input value.
 */
template<typename T>
bool Gridmap<T>::reduceGridToPolygon(const std::vector<Point>& boundary,
                                 bool  expandToPolygon,
                                 const T *_val,
                                 bool only_perimeter){
    if (expandToPolygon)
        return expandGridFromPolygon(boundary, _val, only_perimeter, true);

    if(!checkInternalParameters(true, __FUNCTION__))
        return false;

    if (boundary.size() < 3){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The boundary must have at least 3 different points");
        return false;
    }

    // check the dimensions / bounding box:
    double xMin = std::numeric_limits<double>::max();
    double yMin = std::numeric_limits<double>::max();
    double xMax = std::numeric_limits<double>::lowest();
    double yMax = std::numeric_limits<double>::lowest();

    for (size_t i=0; i < boundary.size(); i++) {
        if (boundary[i].x < xMin) xMin = boundary[i].x;
        if (boundary[i].x > xMax) xMax = boundary[i].x;
        if (boundary[i].y < yMin) yMin = boundary[i].y;
        if (boundary[i].y > yMax) yMax = boundary[i].y;
    }

    if (xMin == xMax || yMin == yMax){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The boundary must have at least 3 different points which don't belong to the same line");
        return false;
    }

    if (xMin >= m_layout.getMaxPointX() || xMax <= m_layout.getMinPointX() || yMin >= m_layout.getMaxPointY() || yMax <= m_layout.getMinPointY()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The boundary must lie at least partially inside the grid's boundary");
        return false;
    }

    if ( std::fabs(xMin - m_layout.getMinPointX()) < 0.00001 ) xMin = m_layout.getMinPointX();
    if ( std::fabs(xMax - m_layout.getMaxPointX()) < 0.00001 ) xMax = m_layout.getMaxPointX();
    if ( std::fabs(yMin - m_layout.getMinPointY()) < 0.00001 ) yMin = m_layout.getMinPointY();
    if ( std::fabs(yMax - m_layout.getMaxPointY()) < 0.00001 ) yMax = m_layout.getMaxPointY();

    int cellsLeft  = std::min( std::ceil( (m_layout.getMinPointX() - xMin)/m_layout.getCellsize() ) , 0.0 );
    int cellsRight = std::min( std::ceil( (xMax - m_layout.getMaxPointX())/m_layout.getCellsize() ) , 0.0 );
    int cellsDown  = std::min( std::ceil( (m_layout.getMinPointY() - yMin)/m_layout.getCellsize() ) , 0.0 );
    int cellsUp    = std::min( std::ceil( (yMax - m_layout.getMaxPointY())/m_layout.getCellsize() ) , 0.0 );

    if (cellsLeft == 0 && cellsRight == 0 && cellsDown == 0 && cellsUp == 0)
        return true;

    Gridmap gridOld = *(this);
    unsigned int sizeXOld = m_layout.getSizeX();
    unsigned int sizeYOld = m_layout.getSizeY();
    double xMinOld = m_layout.getMinPointX();
    double yMinOld = m_layout.getMinPointY();
    unsigned int sizeX = m_layout.getSizeX() + cellsLeft + cellsRight;
    unsigned int sizeY = m_layout.getSizeY() + cellsDown + cellsUp;

    if( !m_layout.init( xMinOld - m_layout.getCellsize()*cellsLeft,
                        yMinOld - m_layout.getCellsize()*cellsDown,
                        sizeX, sizeY, m_layout.getCellsize() ) ){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error setting new layout");
        *(this) = gridOld;
        return false;
    }

    destroy();

    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Allocating new grid of size " + std::to_string(sizeX) + " x " + std::to_string(sizeY));
    if (!allocate()) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Unable to allocate grid. The new grid might be too large.");
        *(this) = gridOld;
        return false;
    }

    if ( !only_perimeter ){
        Polygon poly;
        poly.points = boundary;
        arolib::geometry::correct_polygon(poly);
        setPolygon(poly, _val);
    }
    else{
        double width0 = 0;
        for (size_t i=0; i+1 < boundary.size(); i++){
            setLine(boundary.at(i), boundary.at(i+1), width0, _val);
        }
    }

    //copy previous values into new grid
    unsigned int x1 = -cellsLeft;
    unsigned int y1 = -cellsDown;
    unsigned int x2 = sizeXOld + cellsRight;
    unsigned int y2 = sizeYOld + cellsUp;
    for (unsigned int  x = x1 ; x < x2 ; ++x){
        for (unsigned int  y = y1 ; y < y2 ; ++y){
            if( gridOld.isSet(x, y) )
                set( x+cellsLeft , y+cellsDown , gridOld.get(x, y) );
            else
                unSet( x+cellsLeft , y+cellsDown );
        }
    }

    return true;

}


/**
 * Increase/decrease the resolution
 */
template<typename T>
bool Gridmap<T>::scaleResolution(int scale){
    if(scale == 0){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The scale must be different from zero");
        return false;
    }
    if(scale < 0){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Negative values (0<scale<1) are not supported at the moment");
        return false;
    }
    if(!checkInternalParameters(true, __FUNCTION__))
        return false;
    auto originalGrid = *this;

    destroy();

    if(!m_layout.scaleResolution(scale)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error scaling layout");
        *(this) = originalGrid;
        return false;
    }

    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Allocating new grid of size " + std::to_string(m_layout.getSizeX()) + " x " + std::to_string(m_layout.getSizeY()));
    if (!allocate()) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Unable to allocate grid. The new grid might be too large.");
        *(this) = originalGrid;
        return false;
    }

    for(size_t x = 0 ; x < m_layout.getSizeX() ; x++){
        for(size_t y = 0 ; y < m_layout.getSizeY() ; y++){
            size_t x2 = x/scale;
            size_t y2 = y/scale;
            if( originalGrid.isSet(x2, y2) )
                set( x, y , originalGrid.get(x2, y2) );
//                    else
//                        unSet(x, y); //should be initialized unset during allocation
        }
    }
    return true;
}


template<typename T>
void Gridmap<T>::setUnits(Unit units){
    m_units = units;
}

template<typename T>
bool Gridmap<T>::setLayoutTolerance(double tolerance){
    return m_layout.setTolerance(tolerance);
}

template<typename T>
void Gridmap<T>::setComputeInMultiThread(bool val){
    m_computeInMultiThread = val;
    m_layout.setComputeInMultiThread(val);
}

//------------------------------------
//--------------SET-VALUE-------------
//------------------------------------


/**
 * Set the data(value) of a cell within the grid, specified by the cell's x- and y- indexes.
 */
template<typename T>
bool Gridmap<T>::setValue(unsigned int x,
                      unsigned int y,
                      const T *_value)
{
    if ( !checkInternalParameters(true,__FUNCTION__) )
        return false;

    if ( !checkIndexesInRange(x,y) ){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Indexes out of range");
        return false;
    }

    return set(x, y, _value);
}


/**
 * Set the data(value) of a grid point, given in *real world coordinates*.
 */
template<typename T>
bool Gridmap<T>::setValue(const Point& p,
                      const T *value) {

    if ( !checkPointInRange(p) ) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Input point " + p.toString()
                                                         + "out of range : range(x) = [ " + std::to_string(m_layout.getMinPointX())
                                                         + "," + std::to_string(m_layout.getMaxPointX())
                                                         + " ] ; range(y) = [ " + std::to_string(m_layout.getMinPointY())
                                                         + "" + std::to_string(m_layout.getMaxPointY()) + " ]");
        return false;
    }

    unsigned int gridX, gridY;
    point2index(p, gridX, gridY);

    return setValue(gridX, gridY, value);
}

/**
 * Set the data(value) of a all cells.
 */
template<typename T>
bool Gridmap<T>::setAllValues(const T* _value){

    if ( !checkInternalParameters(true,__FUNCTION__) )
        return false;

    for (unsigned int x = 0 ; x < m_layout.getSizeX(); x++){
        for (unsigned int y = 0 ; y < m_layout.getSizeY(); y++){
            set(x, y, _value);
        }
    }
    return true;
}

/**
 * Sets a new cell value based on the given callback function.
 */
template<typename T>
bool Gridmap<T>::setValue(unsigned int x,
                      unsigned int y,
                      const FuncGetNewValue& funcGetNewValue)
{
    if ( !this->checkInternalParameters(true,__FUNCTION__) )
        return false;

    if ( !this->checkIndexesInRange(x,y) ){
        this->logger().printOut(LogLevel::ERROR, __FUNCTION__, "Indexes out of range");
        return false;
    }

    std::shared_ptr<T> valNew = nullptr;
    bool isCellSet = this->isSet(x, y);
    if(isCellSet)
        valNew = funcGetNewValue( std::make_shared<T>( this->get(x, y) ), x, y );
    else
        valNew = funcGetNewValue( nullptr, x, y );

    if(!valNew)
        return this->unSet(x, y);
    return set(x, y, *valNew);
}

/**
 * Sets a new cell value based on the given callback function.
 * @return True on success
 */
template<typename T>
bool Gridmap<T>::setValue(const Point& p,
                      const FuncGetNewValue& funcGetNewValue)
{
    if ( !checkPointInRange(p) ) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Input point " + p.toString()
                                                         + "out of range : range(x) = [ " + std::to_string(m_layout.getMinPointX())
                                                         + "," + std::to_string(m_layout.getMaxPointX())
                                                         + " ] ; range(y) = [ " + std::to_string(m_layout.getMinPointY())
                                                         + "" + std::to_string(m_layout.getMaxPointY()) + " ]");
        return false;
    }

    unsigned int gridX, gridY;
    point2index(p, gridX, gridY);
    return setValue(gridX, gridY, funcGetNewValue);
}

/**
 * Set the data(value) of a cell within the grid to 'noDataValue', specified by the cell's x- and y- indexes.
 */
template<typename T>
bool Gridmap<T>::setNoValue(unsigned int x, unsigned int y)
{
    if ( !checkInternalParameters(true,__FUNCTION__) )
        return false;

    if ( !checkIndexesInRange(x,y) ){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Indexes out of range");
        return false;
    }

    return unSet(x, y);
}

/**
 * Set the data(value) of a grid point to 'noDataValue', given in *real world coordinates*.
 */
template<typename T>
bool Gridmap<T>::setNoValue(const Point& p) {

    if ( !checkPointInRange(p) ) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Input point " + p.toString()
                                                         + "out of range : range(x) = [ " + std::to_string(m_layout.getMinPointX())
                                                         + "," + std::to_string(m_layout.getMaxPointX())
                                                         + " ] ; range(y) = [ " + std::to_string(m_layout.getMinPointY())
                                                         + "" + std::to_string(m_layout.getMaxPointY()) + " ]");
        return false;
    }

    unsigned int gridX, gridY;
    point2index(p, gridX, gridY);

    return setNoValue(gridX, gridY);
}


/**
 * Set the data(value) of the cells corresponding to a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
 */
template<typename T>
bool Gridmap<T>::setLine(unsigned int startX,
                         unsigned int startY,
                         unsigned int stopX,
                         unsigned int stopY,
                         double width,
                         const T *_val)
{

    if ( !checkInternalParameters(true,__FUNCTION__) )
        return false;

    if( !checkIndexesInRange(startX,startY) ){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Start indexes out of range");
        return false;
    }

    if( !checkIndexesInRange(stopX,stopY) ){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Stop indexes out of range");
        return false;
    }

//        if( width<=0 ){
//            logger().printOut(LogLevel::ERROR, __FUNCTION__, "The width must have a positive value");
//            return false;
//        }


    if (startX == stopX && startY == stopY){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Start and Stop indexes corespond to the same cell");
        return false;
    }

    double vecX = (double)stopX - (double)startX;
    double vecY = (double)stopY - (double)startY;
    double length = sqrt( vecX*vecX + vecY*vecY );

    double precision = 1;
    int precWidth = (int)(ceil(width/2.0 / (double)m_layout.getCellsize()));  // divited by 2, since we're expanding the edge in both directions

    //int steps = (int)(length / precision + 0.5);
    int steps = (int)std::ceil(length / precision);
    if (steps != 0) {
        // normed vector orthogonal to the edge vector defined above
        double nX = -vecY / length;
        double nY = +vecX / length;

        vecX /= (double)steps;
        vecY /= (double)steps;

        double pointX, pointY;
        //for (int i=0; i < steps; i++) {
        for (double i=0; i < steps; i+=0.5) {//@TODO: i+=0.5 workaround to assure that all cells within the expanded line are checked
            pointX = startX + vecX * i;
            pointY = startY + vecY * i;
            setValue(pointX, pointY, _val);
            for (int j=1; j < precWidth; j++) {
                setValue(pointX + nX*j, pointY + nY*j, _val);
                setValue(pointX - nX*j, pointY - nY*j, _val);
            }
        }
        pointX = stopX;
        pointY = stopY;
        setValue(pointX, pointY, _val);
        for (int j=1; j < precWidth; j++) {
            setValue(pointX + nX*j, pointY + nY*j, _val);
            setValue(pointX - nX*j, pointY - nY*j, _val);
        }
    } else {
        return setValue(startX, startY, _val);
    }
    return true;

}

/**
 * Set the data(value) of the cells corresponding to a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
 */
template<typename T>
bool Gridmap<T>::setLine2(const Point& start,
                             const Point& end,
                             double width,
                             const T *_val)
{
    if ( !checkInternalParameters(true,__FUNCTION__) )
        return false;

//        if (width<=0){
//            logger().printOut(LogLevel::ERROR, __FUNCTION__, "The line width must have a positive value");
//            return false;
//        }

    Point p0 = start, p1 = end;
    if (start==end)
        return setValue(start,_val);

    if ( !checkPointInRange(start) || !checkPointInRange(end) ){
        Polygon gridPoly = getGridPolygon();
        geometry::getLineSegmentInPolygon(gridPoly,start,end,p0,p1);
    }

    unsigned int startX, startY, stopX, stopY;
    point2index(p0, startX, startY);
    point2index(p1, stopX, stopY);

    if(startX == stopX && startY == stopY){
        if(width <= 0)
            return setValue(start,_val);

        Point tmp0, tmp1;
        geometry::offset_line(start, end, tmp0, tmp1, 0.5*width);
        Point start2 ( 0.5 * ( tmp0.x + tmp1.x ) , 0.5 * ( tmp0.y + tmp1.y ) );
        geometry::offset_line(p0, p1, tmp0, tmp1, 0.5*width);
        Point end2 ( 0.5 * ( tmp0.x + tmp1.x ) , 0.5 * ( tmp0.y + tmp1.y ) );
        return setLine(start2, end2, 0, _val);
    }

    return setLine(startX, startY, stopX, stopY, width, _val);

}

/**
 * Set the data(value) of the cells corresponding to a line within the grid, given in *real world coordinates*. The line is extended sideways a distance if width/2.
 */
template<typename T>
bool Gridmap<T>::setLine(const Point& start,
                         const Point& end,
                         const double width,
                         const T *_val,
                         float overlapThreshold)
{
    overlapThreshold = std::min((float)(1.0-1e-4), overlapThreshold);
    if(overlapThreshold > 1e-4 && width > 1e-9){
        const Polygon linePoly = arolib::geometry::createRectangleFromLine(start,end,width);
        return this->setPolygon(linePoly, _val, overlapThreshold);
    }
    else{
        if ( !checkInternalParameters(true,__FUNCTION__) )
            return false;
        int min_y(m_layout.getSizeY()), max_y(-1);
        //get the cells under the extended line
        CellsRangeList indexMap = getCellsUnderLine(start, end, width);
        if (indexMap.empty())
            return false;

        for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
            auto yRanges = indexMap.getColumn(x);
            int lastY = -1;
            for(const auto& yRange : yRanges){
                min_y = yRange.first;
                max_y = yRange.second;
                if(lastY == min_y)
                    ++min_y;
                lastY = max_y;
                for (int y = min_y ; y <= max_y ; ++y)
                    setValue(x,y,_val);
            }
        }
    }

    return true;
}

/**
 * Sets the cells in the list with the corresponding value
 */
template<typename T>
template<typename K, typename>
bool Gridmap<T>::setCellsValue(const T *_value,
                               const std::vector<K> &cells)
{
    if ( !checkInternalParameters(true,__FUNCTION__) )
        return false;

    bool ok = true;
    for (auto& cell : cells){
        if(cell.x < 0 || cell.y < 0){
            ok = false;
            continue;
        }
        ok &= setValue(cell.x, cell.y, _value);
    }
    return ok;
}

/**
 * Sets the cells in the list with the value obtained from the callback function
 */
template<typename T>
template<typename K, typename>
bool Gridmap<T>::setCellsValue(const std::vector<K>& cells, const FuncGetNewValue& funcGetNewValue)
{
    if ( !checkInternalParameters(true,__FUNCTION__) )
        return false;

    bool ok = true;
    for (auto& cell : cells){
        if(cell.x < 0 || cell.y < 0){
            ok = false;
            continue;
        }
        ok &= setValue(cell.x, cell.y, funcGetNewValue);
    }
    return ok;
}

/**
 * Sets the cells in the list with the value obtained from the callback function
 */
template<typename T>
bool Gridmap<T>::setCellsValue(const std::vector<GridmapLayout::GridCellOverlap>& cells,
                               FuncGetNewValueGCOverlap funcGetNewValue)
{
    if ( !checkInternalParameters(true,__FUNCTION__) )
        return false;

    bool ok = true;
    for (auto& cell : cells){
        if(cell.x < 0 || cell.y < 0){
            ok = false;
            continue;
        }
        if ( !this->checkIndexesInRange(cell.x, cell.y) ){
            ok = false;
            continue;
        }

        std::shared_ptr<T> valNew = nullptr;
        bool isCellSet = this->isSet(cell.x, cell.y);
        if(isCellSet)
            valNew = funcGetNewValue( std::make_shared<T>( this->get(cell.x, cell.y) ), cell );
        else
            valNew = funcGetNewValue( nullptr, cell );

        if(!valNew)
            ok &= this->unSet(cell.x, cell.y);
        else
            ok &= set(cell.x, cell.y, *valNew);
    }
    return ok;
}


/**
 * Sets the cells in the list with the corresponding value
 */
template<typename T>
template<typename K, typename>
bool Gridmap<T>::setCellsValue(const std::vector<K> &cells)
{
    if ( !checkInternalParameters(true,__FUNCTION__) )
        return false;

    bool ok = true;
    for (auto& cell : cells){
        if(cell.x < 0 || cell.y < 0){
            ok = false;
            continue;
        }
        if(cell.value)
            ok &= setValue(cell.x, cell.y, *cell.value);
        else
            ok &= setNoValue(cell.x, cell.y);
    }
    return ok;
}


/**
 * Set the data(value) of the cells corresponding to a polygon within the grid, given in *real world coordinates*.
 */
template<typename T>
bool Gridmap<T>::setPolygon(const Polygon& _poly, const T *value, const float overlapThreshold)
{
    int min_y = m_layout.getSizeY(), max_y = -1;
    double areaCell = getCellArea();
    Polygon poly = _poly;
    arolib::geometry::correct_polygon(poly);
    CellsRangeList indexMap;

    if ( !checkInternalParameters(true,__FUNCTION__) )
        return false;
    if (_poly.points.size() < 3){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The polygon must have at least 3 points");
        return false;
    }

    indexMap = getCellsUnderPolygon(poly);

    if (indexMap.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Polygon outside of the grid (resulting polygon indexMap is empty)");
        return false;
    }

    const float tolerance = 1e-4;
    float my_threshold = std::min((float)(1.0-tolerance), overlapThreshold);
    my_threshold = std::max(tolerance, my_threshold);
    const float threshold_area = my_threshold * getCellArea();

    if(m_computeInMultiThread){
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
                        if(overlapThreshold <= tolerance)
                        {
                            Point cellCenter;
                            getCellCenter(x, y, cellCenter);
                            if( arolib::geometry::in_polygon(cellCenter, poly) ){
                                setValue(x, y, value);
                                continue;
                            }

                            Polygon cellPoly;
                            getCellPolygon(x,y,cellPoly);
                            if( arolib::geometry::in_polygon(cellPoly.points, poly, false) ){
                                setValue(x, y, value);
                                continue;
                            }                                
                        }

                        double areaIntersection = this->m_layout.getCellAreaIntersection(x, y, poly, min_y, max_y);
                        if(areaIntersection >= threshold_area)
                            setValue(x, y, value);
                    }
                }
            });
        }

        for(auto& fu_x : futures_x)
            fu_x.wait();
    }
    else{
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
                    if(overlapThreshold <= tolerance)
                    {
                        Point cellCenter;
                        getCellCenter(x, y, cellCenter);
                        if( arolib::geometry::in_polygon(cellCenter, poly) ){
                            setValue(x, y, value);
                            continue;
                        }

                        Polygon cellPoly;
                        getCellPolygon(x,y,cellPoly);
                        if( arolib::geometry::in_polygon(cellPoly.points, poly, false) ){
                            setValue(x, y, value);
                            continue;
                        }                                
                    }
                    double areaIntersection = this->m_layout.getCellAreaIntersection(x, y, poly, min_y, max_y);
                    if(areaIntersection >= threshold_area)
                        setValue(x, y, value);
                }
            }
        }
    }
    return true;
}




/**
 * Intersects the grid values with a polygon (the values inside the polgon stay the same in the grid, whereas the cells outside the polygon are set to a given value)
 */
template<typename T>
bool Gridmap<T>::intersect(const Polygon& _poly, const T *valueOut, float overlapThreshold){

    if ( !checkInternalParameters(true,__FUNCTION__) )
        return false;
    if (_poly.points.size() < 3){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The polygon must have at least 3 points");
        return false;
    }

    Gridmap<bool> gridTmp(m_layout, logger().logLevel());

    if(!gridTmp.isAllocated()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating temporal gridmap");
        return false;
    }

    if( !gridTmp.setPolygon(_poly, true, overlapThreshold) ){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error setting initial polygon cell values");
        return false;
    }
    for(size_t x = 0 ; x < gridTmp.getSizeX() ; x++){
        for(size_t y = 0 ; y < gridTmp.getSizeY() ; y++){
            if( !gridTmp.hasValue(x, y) )
                set(x, y, valueOut);
        }
    }

    return true;

}

//------------------------------------
//--------------GET-PARAM-------------
//------------------------------------

/**
 * Check if the grid is allocated (the grid exists).
 * @return True if the grid is allocated
 */
template<typename T>
bool Gridmap<T>::isAllocated() const { return m_allocated; }

/**
 * Get geometric layout.
 */
template<typename T>
const GridmapLayout& Gridmap<T>::getLayout() const { return m_layout; }

/**
 * Get the number of columns of the grid.
 * @return Number of columns of the grid
 */
template<typename T>
unsigned int Gridmap<T>::getSizeX() const { return m_layout.getSizeX(); }

/**
 * Get the number of rows of the grid.
 * @return Number of rows of the grid
 */
template<typename T>
unsigned int Gridmap<T>::getSizeY() const { return m_layout.getSizeY(); }

/**
 * Get the size of the grid cell (i.e. the grid resolution).
 * @return Size of the grid cell (i.e. the grid resolution)
 */
template<typename T>
double Gridmap<T>::getCellsize () const { return m_layout.getCellsize(); }

/**
 * Get the area of a grid cell (in m²).
 * @return Area of a grid cell
 */
template<typename T>
double Gridmap<T>::getCellArea () const { return m_layout.getCellsize() * m_layout.getCellsize(); }

/**
 * Get the minimum x-axis position (in real world coordinates) of the grid.
 * @return Minimum x-axis position
 */
template<typename T>
double Gridmap<T>::getMinPointX() const { return m_layout.getMinPointX(); }

/**
 * Get the maximum x-axis position (in real world coordinates) of the grid.
 * @return Maximum x-axis position
 */
template<typename T>
double Gridmap<T>::getMinPointY() const { return m_layout.getMinPointY(); }

/**
 * Get the minimum y-axis position (in real world coordinates) of the grid.
 * @return Minimum y-axis position
 */
template<typename T>
double Gridmap<T>::getMaxPointX() const { return m_layout.getMaxPointX(); }

/**
 * Get the maximum y-axis position (in real world coordinates) of the grid.
 * @return Maximum y-axis position
 */
template<typename T>
double Gridmap<T>::getMaxPointY() const { return m_layout.getMaxPointY(); }


/**
 * Get the point corresponding to the lower-left corner of the grid boundary (in real world coordinates).
 * @return lower-left corner
*/
template<typename T>
Point Gridmap<T>::getLowerLeftCorner() const { return Point( getMinPointX(), getMinPointY() ); }

/**
* Get the point corresponding to the upper-right corner of the grid boundary (in real world coordinates).
* @return upper-left corner
*/
template<typename T>
Point Gridmap<T>::getUpperRightCorner() const { return Point( getMaxPointX(), getMaxPointY() ); }


template<typename T>
Unit Gridmap<T>::getUnits() const { return m_units; }

template<typename T>
double Gridmap<T>::getLayoutTolerance() const { return m_layout.getTolerance(); }


template<typename T>
bool Gridmap<T>::getComputeInMultiThread() const{ return m_computeInMultiThread; }


//------------------------------------
//--------------GET-VALUE-------------
//------------------------------------

template<typename T>
bool Gridmap<T>::hasValue(unsigned int x,
                      unsigned int y,
                      bool *_error_) const{
    if(_error_) *_error_ = false;

    if ( !checkInternalParameters(true,__FUNCTION__) ){
        if(_error_) *_error_ = true;
        return false;
    }

    if ( !checkIndexesInRange(x,y) ){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Indexes out of range");
        if(_error_) *_error_ = true;
        return false;
    }

    return isSet(x,y);
}

template<typename T>
bool Gridmap<T>::hasValue(unsigned int x,
                      unsigned int y) const{
    return hasValue(x, y, nullptr); 
}

template<typename T>
bool Gridmap<T>::hasValue(const Point& p, bool *_error_) const{

    if ( !checkPointInRange(p) ) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Input point " + p.toString()
                                                         + "out of range : range(x) = [ " + std::to_string(m_layout.getMinPointX())
                                                         + "," + std::to_string(m_layout.getMaxPointX())
                                                         + " ] ; range(y) = [ " + std::to_string(m_layout.getMinPointY())
                                                         + "" + std::to_string(m_layout.getMaxPointY()) + " ]");
        if(_error_) *_error_ = true;
        return false;
    }

    unsigned int gridX, gridY;
    point2index(p, gridX, gridY);

    return hasValue(gridX, gridY, _error_);

}

/**
 * Obtain the data(value) of a cell within the grid, specified by the cell's x- and y- indexes.
 */
template<typename T>
T Gridmap<T>::getValue(unsigned int x,
                       unsigned int y,
                       bool *_error_) const
{
    if(_error_) *_error_ = false;
    T val;
    if ( !checkInternalParameters(true,__FUNCTION__) ){
        if(_error_) *_error_ = true;
        return val;
    }

    if ( !checkIndexesInRange(x,y) ){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Indexes out of range");
        if(_error_) *_error_ = true;
        return val;
    }

    val = get(x,y);
    return val;
}

template<typename T>
T Gridmap<T>::getValueThrows(unsigned int x,
                       unsigned int y) const
{
    bool error = false;
    T val = getValue(x,y,&error);
    if (error)
        throw std::runtime_error("Gridmap getValue failed! Maybe there is no value at the coords you provided?");
    return val;
}

/**
 * Obtain the data(value) of a grid point, given in *real world coordinates*.
 */
template<typename T>
T Gridmap<T>::getValue(const Point& p,
                            bool *_error_) const
{
    T val;

    if ( !checkInternalParameters(true,__FUNCTION__) ){
        if(_error_) *_error_ = true;
        return val;
    }

    if ( !checkPointInRange(p) ) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Input point " + p.toString()
                                                         + "out of range : range(x) = [ " + std::to_string(m_layout.getMinPointX())
                                                         + "," + std::to_string(m_layout.getMaxPointX())
                                                         + " ] ; range(y) = [ " + std::to_string(m_layout.getMinPointY())
                                                         + "" + std::to_string(m_layout.getMaxPointY()) + " ]");
        if(_error_) *_error_ = true;
        return val;
    }

    unsigned int gridX, gridY;
    point2index(p, gridX, gridY);

    /*if (gridX < 0 || gridX >= m_layout.getSizeX() || gridY < 0 || gridY >= m_layout.getSizeY() ) {
        std::cerr << "[ERROR] grid position out of range: " << gridX << ", " << gridY  << " (of " << m_layout.getSizeX() << ", " << m_layout.getSizeY() << ")" << std::endl;
        _error_ = true;
        return 0;
    }*/

    val = getValue(gridX, gridY, _error_);
    return val;
}


/**
 * Get the indexes ranges of the cells that are in touch with the line.
 */
template<typename T>
CellsRangeList Gridmap<T>::getCellsUnderLine(const Point& start, const Point& end, bool simple) const
{
    return m_layout.getCellsUnderLine(start, end, simple);
}


/**
 * Get the indexes ranges of the cells that are in touch with the expanded line.
 */
template<typename T>
CellsRangeList Gridmap<T>::getCellsUnderLine(const Point& start,
                                         const Point& end,
                                         double width) const
{
    return m_layout.getCellsUnderLine(start, end, width);
}

/**
* Get the indexes ranges of the cells that are in touch with the expanded line.
*/
template<typename T>
CellsRangeList Gridmap<T>::getCellsUnderLine2(const Point& start,
                                         const Point& end,
                                         double width) const
{
    return m_layout.getCellsUnderLine2(start, end, width);
}

/**
 * Get the indexes of the cells that are in touch with the expanded line, as well as how much of the cell is under the extended line.
 */
template<typename T>
std::vector< GridmapLayout::GridCellOverlap > Gridmap<T>::getCellsOverlapUnderLine(const Point& start,
                                                                                const Point& end,
                                                                                double width,
                                                                                const Polygon& boundary) const
{
    return m_layout.getCellsOverlapUnderLine(start, end, width, boundary);
}

/**
 * Get the indexes ranges of the cells that are in touch with the polygone.
 */
template<typename T>
CellsRangeList Gridmap<T>::getCellsUnderPolygon(const Polygon& _poly) const //@TODO it was tested only with rectangles
{
    return m_layout.getCellsUnderPolygon(_poly);
}

/**
 * Get the indexes of the cells that are in touch with the polygon, as well as how much of the cell is under the polygon.
 */
template<typename T>
std::vector< GridmapLayout::GridCellOverlap > Gridmap<T>::getCellsOverlapUnderPolygon(const Polygon& _poly,
                                                                                   const Polygon& boundary) const
{
    return m_layout.getCellsOverlapUnderPolygon(_poly, boundary);
}

/**
 * Obtain the coordinates coresponding to the center of the given input cell.
 */
template<typename T>
bool Gridmap<T>::getCellCenter (unsigned int cellX, unsigned int cellY, Point &center) const
{
    return m_layout.getCellCenter(cellX, cellY, center);
}

/**
 * Obtain the coordinates coresponding to the minimum corner of the given input cell.
 */
template<typename T>
bool Gridmap<T>::getCellMinCorner(unsigned int cellX, unsigned int cellY, Point &corner) const
{
    return m_layout.getCellMinCorner(cellX, cellY, corner);
}

/**
 * Obtain the line parameters coresponding to the given cell.
 */
template<typename T>
bool Gridmap<T>::getCellPolygonAsLine(unsigned int cellX, unsigned int cellY, Point &p0, Point &p1, double &width) const
{
    return m_layout.getCellPolygonAsLine(cellX, cellY, p0, p1, width);
}

/**
 * Obtain the arolib::polygon coresponding to the given cell.
 */
template<typename T>
bool Gridmap<T>::getCellPolygon(unsigned int cellX, unsigned int cellY, Polygon &poly) const
{
    return m_layout.getCellPolygon(cellX, cellY, poly);
}

/**
 * Obtain the arolib::polygon coresponding to the grid (in real world coordinates).
 * @return The arolib::polygon coresponding to the grid (in real world coordinates).
 */
template<typename T>
Polygon Gridmap<T>::getGridPolygon(bool *_error_) const
{
    return m_layout.getGridPolygon(_error_);
}

/**
 * Obtain the grid area.
 */
template<typename T>
double Gridmap<T>::getGridArea(bool * _error_) const
{
    return m_layout.getGridArea(_error_);
}

//------------------------------------
//----------------CHECK---------------
//------------------------------------

/**
 * Checks if the given point (in real world coordinates) is in the grid.
 */
template<typename T>
bool Gridmap<T>::checkPointInRange(const Point& point) const
{
    return m_layout.checkPointInRange(point);
}

/**
 * Checks if the x- and y- indexes lie within the grid's valid range.
 */
template<typename T>
bool Gridmap<T>::checkIndexesInRange(unsigned int x, unsigned int y) const
{
    return m_layout.checkIndexesInRange(x, y);
}

/**
 * Checks if the x- and y- indexes lie within the grid's valid range.
 */
template<typename T>
bool Gridmap<T>::checkIndexesInRange(int x, int y) const
{
    return m_layout.checkIndexesInRange(x, y);
}

//------------------------------------
//-------------CONVERSIONS------------
//------------------------------------

/**
 * Get the grid indexes corresponding to a point (in real-world coordinates).
 */
template<typename T>
bool Gridmap<T>::point2index(const Point& _p, unsigned int &x, unsigned int &y, bool withTolerance) const
{
    return m_layout.point2index(_p, x, y, withTolerance);
}



//------------------------------------
//----------PROTECTED METHODS---------
//------------------------------------


/**
 * Creates a new grid by allocating the needed memory.
 */
template<typename T>
bool Gridmap<T>::allocate()
{
    if (isAllocated())
        return false;

    if(!m_layout.isValid())
        return false;

    m_grid = new TStore*[ m_layout.getSizeX() ];
    if (!m_grid) {
        m_layout.setSize(0,0);
        return false;
    }

    m_allocated = true;//set it here in case we need to destroy

    for (unsigned int x=0; x < m_layout.getSizeX(); x++) {
        m_grid[x] = new TStore[ m_layout.getSizeY() ];
        if (!m_grid[x]) {
            m_layout.setSize(x, m_layout.getSizeY());
            destroy();
            return false;
        }

        // initialize the values
        for (unsigned int y=0; y<m_layout.getSizeY(); y++){
            m_grid[x][y] = nullptr;
        }
    }
    return true;
}

 /**
  * Set directly the data(value) of a cell within the grid, specified by the cell's x- and y- indexes.
  */
template<typename T>
bool Gridmap<T>::set(unsigned int x,
                     unsigned int y,
                     const T *_value)
 {
    if(!_value)
        return unSet(x, y);

     m_grid[x][y] = std::unique_ptr<T>( new T( *( _value ) ) );// @todo: change to make_unique when migrating to c++14 or higher

     return true;
 }

/**
 * Unset (value = no data value) directly the data(value) of a cell within the grid, specified by the cell's x- and y- indexes.
 */
template<typename T>
bool Gridmap<T>::unSet(unsigned int x, unsigned int y)
{
    m_grid[x][y] = nullptr;
    return true;
}

/**
 * Check if a cell has value/data.
 */
template<typename T>
bool Gridmap<T>::isSet(unsigned int x, unsigned int y) const
{
    if( m_grid[x][y] )
        return true;
    return false;
}

 /**
  * Obtain directly the data(value) of a cell within the grid, specified by the cell's x- and y- indexes.
  */
template<typename T>
T Gridmap<T>::get(unsigned int x,
                   unsigned int y) const
{
    T val;
    if(isSet(x,y))
        val = *m_grid[x][y];
    return val;
}

/**
 * Check if two cells (i.e. the store values) are considered equal.
 */
template<typename T>
bool Gridmap<T>::isEqual(const std::unique_ptr<T>& v1, const std::unique_ptr<T>& v2, const std::function<bool(const T &, const T &)> &isDataEqual){
    if( !v1 && !v2 )
        return true;
    if( ( v1 && !v2 ) || ( !v1 && v2 ) )
        return false;
    return isDataEqual(*v1, *v2);
}

/**
 * Checks that the internal parameters (is_allocated, grid size, cellsize, min/max coordinates,...) are correct to perform operations in the grid.
 */
template<typename T>
bool Gridmap<T>::checkInternalParameters(bool printError, const std::string & function) const
{
    if (!isAllocated()){
        if (printError)
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Grid is not allocated yet");
        return false;
    }
    if (!m_layout.isValid()){
        if (printError)
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Grid geometric layout is not valid");
        return false;
    }
    return true;
}



}
}


#endif //_AROLIB_GRIDMAP_TCC_

