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
 
#include "arolib/cartography/gridmap_layout.hpp"

namespace arolib {
namespace gridmap {

using namespace arolib::geometry;

/**
 * Constructor.
 * The grid is not allocated yet (it doesn't exist).
 */

GridmapLayout::GridmapLayout(double minX, double minY, size_t sizeX, size_t sizeY, double cellsize, LogLevel logLevel)
    : LoggingComponent(logLevel, __FUNCTION__)
    , m_minPointX(minX), m_minPointY(minY), m_maxPointX(minX+sizeX*cellsize), m_maxPointY(minY+sizeY*cellsize)
    , m_cellsize(cellsize), m_sizeX(sizeX), m_sizeY(sizeY)
    , m_computeInMultiThread(true)
{
}

GridmapLayout::GridmapLayout(LogLevel logLevel)
    : LoggingComponent(logLevel, __FUNCTION__)
    , m_computeInMultiThread(true){
    reset();
}


/**
 * Copy constructor.
 */

GridmapLayout::GridmapLayout(const GridmapLayout &other):
      LoggingComponent(other)
{
    m_minPointX = other.m_minPointX;
    m_minPointY = other.m_minPointY;
    m_maxPointX = other.m_maxPointX;
    m_maxPointY = other.m_maxPointY;
    m_cellsize = other.m_cellsize;
    m_sizeX = other.m_sizeX;
    m_sizeY = other.m_sizeY;
    m_computeInMultiThread = other.m_computeInMultiThread;
}

/**
 * Destructor.
 */

GridmapLayout::~GridmapLayout()
{
}

//------------------------------------
//--------------OPERATORS-------------
//------------------------------------

// copy assignment

GridmapLayout& GridmapLayout::operator=(const GridmapLayout& other){
    m_minPointX = other.m_minPointX;
    m_minPointY = other.m_minPointY;
    m_maxPointX = other.m_maxPointX;
    m_maxPointY = other.m_maxPointY;
    m_cellsize = other.m_cellsize;
    m_sizeX = other.m_sizeX;
    m_sizeY = other.m_sizeY;
    return *this;
}


bool GridmapLayout::operator==(const GridmapLayout& other) const {
    const double EPS = 1e-9;
    return m_sizeX == other.m_sizeX ||
            m_sizeY == other.m_sizeY ||
            std::fabs(m_cellsize - other.m_cellsize) < EPS ||
            std::fabs(m_minPointX - other.m_minPointX) < EPS ||
            std::fabs(m_minPointY - other.m_minPointY) < EPS ||
            std::fabs(m_maxPointX - other.m_maxPointX) < EPS ||
            std::fabs(m_maxPointY - other.m_maxPointY) < EPS;
}

bool GridmapLayout::operator<(const GridmapLayout& other) const {
    const double EPS = 1e-9;
    double delta_x = m_maxPointX - m_minPointX;
    double delta_y = m_maxPointY - m_minPointY;
    double area = delta_x * delta_y;
    double delta_x_other = other.m_maxPointX - other.m_minPointX;
    double delta_y_other = other.m_maxPointY - other.m_minPointY;
    double area_other = delta_x_other * delta_y_other;

    if( std::fabs(area - area_other) > EPS )
        return area < area_other;
    if( std::fabs(delta_x - delta_x_other) > EPS )
        return delta_x < delta_x_other;
    if( std::fabs(delta_y - delta_y_other) > EPS )
        return delta_y < delta_y_other;
    if(m_cellsize != other.m_cellsize)
        return m_cellsize < other.m_cellsize;
    if( std::fabs(m_minPointX - other.m_minPointX) > EPS )
        return m_minPointX < other.m_minPointX;
    if( std::fabs(m_minPointY - other.m_minPointY) > EPS )
        return m_minPointY < other.m_minPointY;

    return false;
}


//------------------------------------
//--------------SET-PARAM-------------
//------------------------------------

/**
 * De-allocates the grid.
 * @param resetAllParameters If set to true, all the internal parameters (cellsize, min/max data-values, etc) will be reset to the default values; if set to false, only the parameters related to the grid size and allocation are reset.
 */

void GridmapLayout::reset()
{
    m_minPointX = m_minPointY = m_maxPointX = m_minPointY = m_cellsize = -1;
    m_sizeX = m_sizeY = 0;
}

bool GridmapLayout::init(double minX, double minY, size_t sizeX, size_t sizeY, double cellsize)
{
    m_minPointX = minX;
    m_minPointY = minY;
    m_maxPointX = minX + sizeX * cellsize;
    m_maxPointY = minY + sizeY * cellsize;
    m_cellsize = cellsize;
    m_sizeX = sizeX;
    m_sizeY = sizeY;

    return isValid();
}

bool GridmapLayout::setSize(size_t sizeX, size_t sizeY)
{
    return init(m_minPointX, m_minPointY, sizeX, sizeY, m_cellsize);
}

bool GridmapLayout::setTolerance(double tolerance)
{
    if(tolerance < 0)
        return false;

    this->m_limitTolerance = tolerance;
    return true;
}


/**
 * Increase/decrease the resolution
 */

bool GridmapLayout::scaleResolution(int scale){
    if(scale == 0){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The scale must be different from zero");
        return false;
    }
    if(scale < 0){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Negative values (0<scale<1) are not supported at the moment");
        return false;
    }
    if(!isValid()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The grid layout is not valid");
        return false;
    }

    m_sizeX *= scale;
    m_sizeY *= scale;
    m_cellsize /= scale;
    return true;
}



void GridmapLayout::setComputeInMultiThread(bool val){
    m_computeInMultiThread = val;
}


//------------------------------------
//--------------GET-PARAM-------------
//------------------------------------


bool GridmapLayout::isValid() const
{
    return m_minPointX < m_maxPointX
            && m_minPointY < m_maxPointY
            && m_sizeX > 0 && m_sizeY > 0
            && m_cellsize > 0;
}

/**
 * Get the number of columns of the grid.
 * @return Number of columns of the grid
 */

unsigned int GridmapLayout::getSizeX() const { return m_sizeX; }

/**
 * Get the number of rows of the grid.
 * @return Number of rows of the grid
 */

unsigned int GridmapLayout::getSizeY() const { return m_sizeY; }

/**
 * Get the size of the grid cell (i.e. the grid resolution).
 * @return Size of the grid cell (i.e. the grid resolution)
 */

double GridmapLayout::getCellsize() const { return m_cellsize; }

/**
 * Get the area of a grid cell (in m²).
 * @return Area of a grid cell
 */

double GridmapLayout::getCellArea() const { return m_cellsize * m_cellsize; }

/**
 * Get the minimum x-axis position (in real world coordinates) of the grid.
 * @return Minimum x-axis position
 */

double GridmapLayout::getMinPointX() const { return m_minPointX; }

/**
 * Get the maximum x-axis position (in real world coordinates) of the grid.
 * @return Maximum x-axis position
 */

double GridmapLayout::getMinPointY() const { return m_minPointY; }

/**
 * Get the minimum y-axis position (in real world coordinates) of the grid.
 * @return Minimum y-axis position
 */

double GridmapLayout::getMaxPointX() const { return m_maxPointX; }

/**
 * Get the maximum y-axis position (in real world coordinates) of the grid.
 * @return Maximum y-axis position
 */

double GridmapLayout::getMaxPointY() const { return m_maxPointY; }


/**
 * Get the point corresponding to the lower-left corner of the grid boundary (in real world coordinates).
 * @return lower-left corner
*/

Point GridmapLayout::getLowerLeftCorner() const { return Point( getMinPointX(), getMinPointY() ); }

/**
* Get the point corresponding to the upper-right corner of the grid boundary (in real world coordinates).
* @return upper-left corner
*/

Point GridmapLayout::getUpperRightCorner() const { return Point( getMaxPointX(), getMaxPointY() ); }

double GridmapLayout::getTolerance() const
{
    return this->m_limitTolerance;
}


bool GridmapLayout::getComputeInMultiThread() const{ return m_computeInMultiThread; }


//------------------------------------
//--------------GET-VALUE-------------
//------------------------------------


/**
 * Get the indexes ranges of the cells that are in touch with the line.
 */

CellsRangeList GridmapLayout::getCellsUnderLine(const Point& start, const Point& end, bool simple) const
{
    CellsRangeList indexMap;
    CellsRangeSet indexMapSet;
    Point p0 = start, p1 = end;
    unsigned int indX_start, indY_start, indX_finish, indY_finish;

    if ( !isValid() ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The grid layout is not valid");
        return indexMap;
    }

    if (start==end){
        if( point2index(start, indX_start, indY_start) ){
            indexMap.init(indX_start,1);
            indexMap.appendToColumn(indX_start, indY_start, indY_start);
        }
        return indexMap;
    }

    if ( !checkPointInRange(start) || !checkPointInRange(end) ){
        Polygon gridPoly = getGridPolygon();
        if(!getLineSegmentInPolygon(gridPoly,start,end,p0,p1))
            return indexMap;
    }

    if(p0.x > p1.x)
        std::swap(p0,p1);
    point2index(p0, indX_start, indY_start);
    point2index(p1, indX_finish, indY_finish);

    indexMap.initR(indX_start,indX_finish);
    indexMapSet.initR(indX_start,indX_finish);

    if(indX_start == indX_finish){
        if(indY_start > indY_finish)
            std::swap(indY_start, indY_finish);
        indexMap.appendToColumn(indX_start, indY_start, indY_finish);
        return indexMap;
    }
    if(indY_start == indY_finish){
        while(indX_start <= indX_finish)
            indexMap.appendToColumn(indX_start++, indY_start, indY_start);
        return indexMap;
    }
    if(simple){//use Bresenham's line algorithm
        const bool steep = (std::fabs(p1.y - p0.y) > std::fabs(p1.x - p0.x));

        Point p0Tmp = p0, p1Tmp = p1;
        if(steep){
          std::swap(p0Tmp.x, p0Tmp.y);
          std::swap(p1Tmp.x, p1Tmp.y);
        }

        if(p0Tmp.x > p1Tmp.x)
          std::swap(p0Tmp, p1Tmp);

        const double dx = p1Tmp.x - p0Tmp.x;
        const double dy = std::fabs(p1Tmp.y - p0Tmp.y);

        double error = 0.5 * dx;
        const int ystep = (p0Tmp.y < p1Tmp.y) ? 1 : -1;

        unsigned int minX, maxX, minY, maxY;
        point2index(p0Tmp, minX, minY);
        point2index(p1Tmp, maxX, maxY);

        int y = minY;

        for(int x = minX; x <= maxX; ++x){
            if(steep)
                indexMapSet.insertInColumn(y, x);
            else
                indexMapSet.insertInColumn(x, y);

            error -= dy;
            if(error < 0){
                y += ystep;
                error += dx;
            }
        }
    }
    else{
        indexMapSet.insertInColumn(indX_start, indY_start);
        indexMapSet.insertInColumn(indX_finish, indY_finish);

        Point pRef(m_minPointX + getCellsize() * indX_start, 0);
        const double m = (p1.y - p0.y) / (p1.x - p0.x);
        const double b = p0.y - m * p0.x;

        for(size_t x = indX_start; x < indX_finish; ++x){
            unsigned int x2;
            pRef.x += getCellsize();
            pRef.y = m * pRef.x + b;
            unsigned int y;
            if(point2index(pRef, x2, y)){
                indexMapSet.insertInColumn(x, y);
                indexMapSet.insertInColumn(x+1, y);
            }
        }
    }


    if( indexMapSet.minX() == indexMapSet.maxX() && indexMapSet.getColumn(indexMapSet.minX()).empty()){
        indexMap.clear();
        return indexMap;
    }

    //Check if some cells were wrongly left out
    int minY, maxY, minY2, maxY2;
    Point cellCorner;
    for (int x = indexMapSet.minX(); x <= indexMapSet.maxX(); ++x){

        //check for empty columns (happens when a point lies exactly in the cell edge)
        if(indexMapSet.getColumn(x).empty()){
            if( x == indexMapSet.minX() ){
                minY = indexMapSet.getColumnMin(x+1);
                maxY = indexMapSet.getColumnMax(x+1);
            }
            else if( x == indexMapSet.maxX() ){
                minY = indexMapSet.getColumnMin(x-1);
                maxY = indexMapSet.getColumnMax(x-1);
            }
            else{
                auto minY_prev = indexMapSet.getColumnMin(x-1);
                auto minY_next = indexMapSet.getColumnMin(x+1);
                auto maxY_prev = indexMapSet.getColumnMax(x-1);
                auto maxY_next = indexMapSet.getColumnMax(x+1);

                minY = (minY_prev + minY_next)/1.999999;
                maxY = (maxY_prev + maxY_next)/1.999999;            }

        }
        else{
            minY = indexMapSet.getColumnMin(x);
            maxY = indexMapSet.getColumnMax(x);
        }

        //check for the cells over and under the max and min y indexes, respectively

        minY2 = std::max( minY-1, 0 );
        maxY2 = maxY < 0 ? m_sizeY-1 : std::min( maxY+1, (int)m_sizeY-1 );

        if(minY != minY2){
            getCellMinCorner(x, minY2, cellCorner);
            if ( checkLineSegmentInRectangle(cellCorner, m_cellsize, m_cellsize, p0, p1 ) )
                minY = minY2;
        }
        if(maxY != maxY2){
            getCellMinCorner(x, maxY2, cellCorner);
            if ( checkLineSegmentInRectangle(cellCorner, m_cellsize, m_cellsize, p0, p1 ) )
                maxY = maxY2;
        }

        indexMap.appendToColumn(x,minY, maxY);
    }

    //for (auto const& i : indexMapSet)
    //indexMap[ i.first ] = std::make_pair( *( i.second.begin() ), *( i.second.rbegin() ) );


    return indexMap;

}

/**
 * Get the indexes ranges of the cells that are in touch with the line.
 */

CellsRangeList GridmapLayout::getCellsUnderLine2(const Point& start, const Point& end) const
{
    CellsRangeList indexMap;
    CellsRangeSet indexMapSet;
    Point p0 = start, p1 = end;
    unsigned int indX, indY;

    if ( !isValid() ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The grid layout is not valid");
        return indexMap;
    }

    if (start==end){
        if( point2index(start, indX, indY) ){
            indexMap.init(indX,1);
            indexMap.appendToColumn(indX, indY, indY);
        }
        return indexMap;
    }

    if ( !checkPointInRange(start) || !checkPointInRange(end) ){
        Polygon gridPoly = getGridPolygon();
        getLineSegmentInPolygon(gridPoly,start,end,p0,p1);
    }

    unsigned int minX, maxX;
    unsigned int startX, endX;
    point2index(p0, startX, indY);
    point2index(p1, endX, indY);
    minX = std::max( std::min(startX,endX) , (unsigned int)0 );
    maxX = std::min( std::max(startX,endX) , m_sizeX-1 );
    indexMap.initR(minX,maxX);
    indexMapSet.initR(minX,maxX);

    double vecX = p1.x - p0.x;
    double vecY = p1.y - p0.y;
    double length = sqrt( vecX*vecX + vecY*vecY );

    double precision_mult = 1;
    double step = (double)m_cellsize * precision_mult;//the precision is needed in cases the line is in touch with the cell in the corner. a high precision_mult (e.g. 1) might miss some of the cells

    Point p, p_parallel;
    p_parallel.x = vecX / length;
    p_parallel.y = vecY / length;
    for (double i = 0; i < length; i+=step){
        p = p0 + p_parallel*i;
        if ( point2index(p,indX,indY) )
            indexMapSet.insertInColumn(indX,indY);
    }
    if ( point2index(p1,indX,indY) )
        indexMapSet.insertInColumn(indX,indY);


    if( indexMapSet.minX() == indexMapSet.maxX() && indexMapSet.getColumn(indexMapSet.minX()).empty()){
        indexMap.clear();
        return indexMap;
    }

    //Check if some cells were wrongly left out
    int minY, maxY, minY2, maxY2;
    Point cellCorner;
    for (int x = indexMapSet.minX(); x <= indexMapSet.maxX(); ++x){

        //check for empty columns (happens when a point lies exactly in the cell edge)
        if(indexMapSet.getColumn(x).empty()){
            if( x == indexMapSet.minX() ){
                minY = indexMapSet.getColumnMin(x+1);
                maxY = indexMapSet.getColumnMax(x+1);
            }
            else if( x == indexMapSet.maxX() ){
                minY = indexMapSet.getColumnMin(x-1);
                maxY = indexMapSet.getColumnMax(x-1);
            }
            else{
                auto minY_prev = indexMapSet.getColumnMin(x-1);
                auto minY_next = indexMapSet.getColumnMin(x+1);
                auto maxY_prev = indexMapSet.getColumnMax(x-1);
                auto maxY_next = indexMapSet.getColumnMax(x+1);

                minY = (minY_prev + minY_next)/1.999999;
                maxY = (maxY_prev + maxY_next)/1.999999;            }

        }
        else{
            minY = indexMapSet.getColumnMin(x);
            maxY = indexMapSet.getColumnMax(x);
        }


        //check for the cells over and under the max and min y indexes, respectively

        minY2 = std::max( minY-1, 0 );
        maxY2 = maxY < 0 ? m_sizeY-1 : std::min( maxY+1, (int)m_sizeY-1 );

        if(minY != minY2){
            getCellMinCorner(x, minY2, cellCorner);
            if ( checkLineSegmentInRectangle(cellCorner, m_cellsize, m_cellsize, p0, p1 ) )
                minY = minY2;
        }
        if(maxY != maxY2){
            getCellMinCorner(x, maxY2, cellCorner);
            if ( checkLineSegmentInRectangle(cellCorner, m_cellsize, m_cellsize, p0, p1 ) )
                maxY = maxY2;
        }

        indexMap.appendToColumn(x,minY, maxY);
    }

    //for (auto const& i : indexMapSet)
    //indexMap[ i.first ] = std::make_pair( *( i.second.begin() ), *( i.second.rbegin() ) );


    return indexMap;

}

/**
* Get the indexes ranges of the cells that are in touch with the line.
*/

CellsRangeList GridmapLayout::getCellsUnderLine3(const Point& start, const Point& end) const
{
   CellsRangeList indexMap;
   CellsRangeSet indexMapSet;
   Point p0 = start, p1 = end;
   unsigned int indX, indY;

   if ( !isValid() ){
       m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The grid layout is not valid");
       return indexMap;
   }

   if (start==end){
       if( point2index(start, indX, indY) ){
           indexMap.init(indX,1);
           indexMap.appendToColumn(indX, indY, indY);
       }
       return indexMap;
   }

   if ( !checkPointInRange(start) || !checkPointInRange(end) ){
       Polygon gridPoly = getGridPolygon();
       getLineSegmentInPolygon(gridPoly,start,end,p0,p1);
   }

   unsigned int minX, maxX;
   unsigned int startX, endX;
   point2index(p0, startX, indY);
   point2index(p1, endX, indY);
   minX = std::max( std::min(startX,endX) , (unsigned int)0 );
   maxX = std::min( std::max(startX,endX) , m_sizeX-1 );
   indexMap.initR(minX,maxX);
   indexMapSet.initR(minX,maxX);

   std::vector<Point> line = {p0, p1};
   line = arolib::geometry::sample_geometry( line , m_cellsize*0.5);

   for (double i = 0; i < line.size(); ++i){
       if ( point2index(line.at(i),indX,indY) )
           indexMapSet.insertInColumn(indX,indY);
   }

   if( indexMapSet.minX() == indexMapSet.maxX() && indexMapSet.getColumn(indexMapSet.minX()).empty()){
       indexMap.clear();
       return indexMap;
   }

   //Check if some cells were wrongly left out
   int minY, maxY, minY2, maxY2;
   Point cellCorner;
   for (int x = indexMapSet.minX(); x <= indexMapSet.maxX(); ++x){

       //check for empty columns (happens when a point lies exactly in the cell edge)
       if(indexMapSet.getColumn(x).empty()){
           if( x == indexMapSet.minX() ){
               minY = indexMapSet.getColumnMin(x+1);
               maxY = indexMapSet.getColumnMax(x+1);
           }
           else if( x == indexMapSet.maxX() ){
               minY = indexMapSet.getColumnMin(x-1);
               maxY = indexMapSet.getColumnMax(x-1);
           }
           else{
               auto minY_prev = indexMapSet.getColumnMin(x-1);
               auto minY_next = indexMapSet.getColumnMin(x+1);
               auto maxY_prev = indexMapSet.getColumnMax(x-1);
               auto maxY_next = indexMapSet.getColumnMax(x+1);

               minY = (minY_prev + minY_next)/1.999999;
               maxY = (maxY_prev + maxY_next)/1.999999;            }

       }
       else{
           minY = indexMapSet.getColumnMin(x);
           maxY = indexMapSet.getColumnMax(x);
       }

       //check for the cells over and under the max and min y indexes, respectively

       minY2 = std::max( minY-1, 0 );
       maxY2 = maxY < 0 ? m_sizeY-1 : std::min( maxY+1, (int)m_sizeY-1 );

       if(minY != minY2){
           getCellMinCorner(x, minY2, cellCorner);
           if ( checkLineSegmentInRectangle(cellCorner, m_cellsize, m_cellsize, p0, p1 ) )
               minY = minY2;
       }
       if(maxY != maxY2){
           getCellMinCorner(x, maxY2, cellCorner);
           if ( checkLineSegmentInRectangle(cellCorner, m_cellsize, m_cellsize, p0, p1 ) )
               maxY = maxY2;
       }

       if( checkIndexesInRange(x,minY) && checkIndexesInRange(x,maxY) )
           indexMap.appendToColumn(x,minY, maxY);
   }

   //for (auto const& i : indexMapSet)
   //indexMap[ i.first ] = std::make_pair( *( i.second.begin() ), *( i.second.rbegin() ) );


   return indexMap;

}

/**
 * Get the indexes ranges of the cells that are in touch with the expanded line.
 */

CellsRangeList GridmapLayout::getCellsUnderLine(const Point& start,
                                                const Point& end,
                                                double width) const{

    CellsRangeList indexMap;
    CellsRangeSet indexMapSet;
    Polygon gridPoly, linePoly;
    std::vector<Polygon> intersectionPoly;
    Point p0 = start, p1 = end;
    unsigned int indX, indY;
    Point p, p_parallel, p0_shift, p1_shift;

    if ( !isValid() ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The grid layout is not valid");
        return indexMap;
    }

    if (start==end){
        if( point2index(start, indX, indY) ){
            indexMap.init(indX,1);
            indexMap.appendToColumn(indX, indY, indY);
        }
        return indexMap;
    }

    if( width<=0 )
        return getCellsUnderLine(p0,p1,false);

    linePoly = arolib::geometry::createRectangleFromLine(start,end,width);
    gridPoly = getGridPolygon();
    intersectionPoly = arolib::geometry::get_likely_intersection(gridPoly, linePoly, m_cellsize*1e-3);

    if (intersectionPoly.empty())
        return indexMap;

    //@TODO: a segment of the line might not be in the polygone, but the extension could. check if the next code must be used or not.
    //    //check only the segment of the line that is inside the polygon
    //    if ( !checkPointInRange(start) || !checkPointInRange(end) ){
    //        Polygon gridPoly = getGridPolygon();
    //        getLineSegmentInPolygon(gridPoly,start,end,p0,p1);
    //    }

    unsigned int minX = std::numeric_limits<unsigned int>::max(), maxX = std::numeric_limits<unsigned int>::lowest();
    std::multimap<unsigned int, unsigned int> xyIndexes;

    for (unsigned int i = 0; i < intersectionPoly.size() ; ++i){
        std::vector<Point> points = intersectionPoly.at(i).points;
        for (unsigned int j = 0; j < points.size()-1 ; ++j){
            point2index( points.at(j), indX, indY);
            minX = std::min( std::min(minX,indX) , m_sizeX-1 );
            maxX = std::min( std::max(maxX,indX) , m_sizeX-1 );

            unsigned int xTmp = std::min( indX , m_sizeX-1 );
            xyIndexes.insert( std::make_pair( xTmp, std::min( indY , m_sizeY-1 ) ) );
        }
    }
    indexMap.initR(minX,maxX);
    indexMapSet.initR(minX,maxX);

    //add the y indexes of the intersection points to ensure that there will be at least one valid y-index in the set of each x-index
    for(auto it_xy : xyIndexes)
        indexMapSet.insertInColumn(it_xy.first, it_xy.second);

    double vecX = p1.x - p0.x;
    double vecY = p1.y - p0.y;
    double length = sqrt( vecX*vecX + vecY*vecY );

    double precision_mult = 1;
    double step = (double)m_cellsize * precision_mult;//the precision is needed in cases the line is in touch with the cell in the corner. a high precision_mult (e.g. 1) might miss some of the cells

    p_parallel.x = vecX / length;
    p_parallel.y = vecY / length;

    for (double d = -width*0.5; d < width*0.5; d+=step){
        offset_line( p0, p1, p0_shift, p1_shift, d );
        //if ( checkLineSegmentInPolygon(gridPoly, p0_shift, p1_shift) )
        {
            for (double i = 0; i < length; i+=step){
                p =  p0_shift + p_parallel*i;
                if ( point2index(p,indX,indY) )
                    indexMapSet.insertInColumn(indX,indY);

            }
            if ( point2index(p1_shift,indX,indY) )
                indexMapSet.insertInColumn(indX,indY);
        }
    }
    offset_line( p0, p1, p0_shift, p1_shift, width*0.5 );

    //if ( checkLineSegmentInPolygon(gridPoly, p0_shift, p1_shift) )
    {
        for (double i = 0; i < length; i+=step){
            p =  p0_shift + p_parallel*i;
            if ( point2index(p,indX,indY) )
                indexMapSet.insertInColumn(indX,indY);

        }
        if ( point2index(p1_shift,indX,indY) )
            indexMapSet.insertInColumn(indX,indY);
    }

    if( indexMapSet.minX() == indexMapSet.maxX() && indexMapSet.getColumn(indexMapSet.minX()).empty()){
        indexMap.clear();
        return indexMap;
    }

    //Check if some cells were wrongly left out
    int minY, maxY;
    int minY2, maxY2;
    Point cellCorner;
    for (int x = indexMapSet.minX(); x <= indexMapSet.maxX(); ++x){

        //check for empty columns (happens when a point lies exactly in the cell edge)
        if(indexMapSet.getColumn(x).empty()){
            if( x == indexMapSet.minX() ){
                minY = indexMapSet.getColumnMin(x+1);
                maxY = indexMapSet.getColumnMax(x+1);
            }
            else if( x == indexMapSet.maxX() ){
                minY = indexMapSet.getColumnMin(x-1);
                maxY = indexMapSet.getColumnMax(x-1);
            }
            else{
                auto minY_prev = indexMapSet.getColumnMin(x-1);
                auto minY_next = indexMapSet.getColumnMin(x+1);
                auto maxY_prev = indexMapSet.getColumnMax(x-1);
                auto maxY_next = indexMapSet.getColumnMax(x+1);

                minY = (minY_prev + minY_next)/1.999999;
                maxY = (maxY_prev + maxY_next)/1.999999;            }

        }
        else{
            minY = indexMapSet.getColumnMin(x);
            maxY = indexMapSet.getColumnMax(x);
        }

        //check for the cells over and under the max and min y indexes, respectively

        minY2 = std::max( minY-1, 0 );
        maxY2 = maxY < 0 ? m_sizeY-1 : std::min( maxY+1, (int)m_sizeY-1 );

        if(minY != minY2){
            getCellMinCorner(x, minY2, cellCorner);
            for(size_t i = 0 ; i+1 < linePoly.points.size() ; ++i){
                if ( checkLineSegmentInRectangle(cellCorner, m_cellsize, m_cellsize, linePoly.points.at(i), linePoly.points.at(i+1) ) ){
                    minY = minY2;
                    break;
                }
            }
        }
        if(maxY != maxY2){
            getCellMinCorner(x, maxY2, cellCorner);
            for(size_t i = 0 ; i+1 < linePoly.points.size() ; ++i){
                if ( checkLineSegmentInRectangle(cellCorner, m_cellsize, m_cellsize, linePoly.points.at(i), linePoly.points.at(i+1) ) ){
                    maxY = maxY2;
                    break;
                }
            }
        }

        if( checkIndexesInRange(x,minY) && checkIndexesInRange(x,maxY) )
            indexMap.appendToColumn(x, minY, maxY);
    }

    return indexMap;

}

/**
* Get the indexes ranges of the cells that are in touch with the expanded line.
*/

CellsRangeList GridmapLayout::getCellsUnderLine2(const Point& start,
                                         const Point& end,
                                         double width) const
{
   Polygon linePoly = arolib::geometry::createRectangleFromLine(start,end,width);
   return getCellsUnderPolygon(linePoly);
}

/**
 * Get the indexes of the cells that are in touch with the expanded line, as well as how much of the cell is under the extended line.
 */

std::vector< GridmapLayout::GridCellOverlap > GridmapLayout::getCellsOverlapUnderLine(const Point& start,
                                                        const Point& end,
                                                        double width,
                                                        const Polygon& boundary) const
{
    return getCellsOverlapUnderLine_v1(start, end, width, boundary);
}

std::vector< GridmapLayout::GridCellOverlap > GridmapLayout::getCellsOverlapUnderLine_v1(const Point& start,
                                                                                         const Point& end,
                                                                                         double width,
                                                                                         const Polygon& boundary) const
{
    CellsRangeList indexMap;
    std::vector< GridCellOverlap > cellsList;
    Polygon linePoly;
    double areaCell = getCellArea();

    if ( !isValid() ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The grid layout is not valid");
        return cellsList;
    }

    indexMap = getCellsUnderLine(start, end, width);
    if (indexMap.empty())
        return cellsList;

    linePoly = arolib::geometry::createRectangleFromLine(start, end, width);

    if(boundary.points.size() > 2){
        auto sampledPolyPoints = arolib::geometry::sample_geometry(linePoly.points, m_cellsize*0.2);

        bool intersectPolys = false;

        if( !arolib::geometry::in_polygon(sampledPolyPoints, boundary, false) ){//the polygon is outside the boundary (it might be surrounding it)
            if( !arolib::geometry::in_polygon(boundary.points.front(), linePoly) )
                return cellsList;

            //all the boundary is inside the polygon
            intersectPolys = true;
        }
        else if( !arolib::geometry::in_polygon(sampledPolyPoints, boundary, true) )//not all the polygon is in the boundary
            intersectPolys = true;

        if(intersectPolys){
            auto intersections_boundary = arolib::geometry::get_likely_intersection(linePoly, boundary, m_cellsize*1e-3);
            if(intersections_boundary.empty())//shouldn't be the case, but just in case
                return cellsList;
            linePoly = intersections_boundary.front();
            arolib::geometry::unsample_polygon(linePoly);
        }
    }

    if(m_computeInMultiThread){
        std::vector< std::future< std::vector< GridCellOverlap >> > futures_x (indexMap.maxX() - indexMap.minX() + 1);
        for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
            auto& fu_x = futures_x.at(x - indexMap.minX());
            fu_x = std::async(std::launch::async,
                            [&, linePoly, x]()->std::vector< GridCellOverlap >{
                int min_y, max_y;
                std::vector< GridCellOverlap > ret;
                auto yRanges = indexMap.getColumn(x);
                int lastY = -1;
                for(const auto& yRange : yRanges){
                    min_y = yRange.first;
                    max_y = yRange.second;
                    if(lastY == min_y)
                        ++min_y;
                    lastY = max_y;

                    for (int y = min_y ; y <= max_y ; ++y){
                        double areaIntersection = getCellAreaIntersection(x, y, linePoly, width);
                        double overlap = std::min( areaIntersection/areaCell, 1.0 );
                        ret.emplace_back( GridCellOverlap(x,y,overlap) );
                    }


    //                    std::vector< std::future< GridCellOverlap > > futures_y (max_y - min_y + 1);
    //                    for (int y = min_y ; y <= max_y ; ++y){
    //                        auto& fu_y = futures_y.at(y - min_y);
    //                        fu_y = std::async(std::launch::async,
    //                                          [&, linePoly, x, y]()->GridCellOverlap{
    //                            double areaIntersection = getCellAreaIntersection(x, y, linePoly, width);
    //                            double overlap = std::min( areaIntersection/areaCell, 1.0 );
    //                            return  GridCellOverlap(x,y,overlap);
    //                        });
    //                    }

    //                    for(size_t i = 0 ; i < futures_y.size() ; ++i)
    //                        ret.emplace_back( futures_y.at(i).get() );
                }

                return ret;
            });
        }

        for(auto& fu_x : futures_x){
            auto listTmp = fu_x.get();
            cellsList.insert( cellsList.end(), listTmp.begin(), listTmp.end() );
        }
    }
    else{
        Polygon cellPoly;
        double areaIntersection;
        std::vector<Polygon> intersections;
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
                    getCellPolygon(x,y,cellPoly);

                    if ( width > m_cellsize*4.0 && arolib::geometry::in_polygon(cellPoly, linePoly) ) //is more time-efficient to check this before instead of calculating direcly the area, assuming that most of the cells in the map are inside the polygon
                        areaIntersection = areaCell;
                    else{
                        areaIntersection = 0;
                        intersections = arolib::geometry::get_likely_intersection(linePoly, cellPoly, m_cellsize*1e-3);
                        for (unsigned int k = 0 ; k < intersections.size() ; ++k)
                            areaIntersection += std::min( arolib::geometry::calc_area( intersections.at(k) ), areaCell );
                    }
                    double overlap = std::min( areaIntersection/areaCell, 1.0 );
                    cellsList.emplace_back(  GridCellOverlap(x,y,overlap) );
                }
            }
        }
    }


    return cellsList;
}

std::vector< GridmapLayout::GridCellOverlap > GridmapLayout::getCellsOverlapUnderLine_v2(const Point& start,
                                                                                         const Point& end,
                                                                                         double width,
                                                                                         const Polygon& boundary) const
{
    auto linePoly = arolib::geometry::createRectangleFromLine(start, end, width);
    return getCellsOverlapUnderPolygon(linePoly, boundary);
}

/**
 * Get the indexes ranges of the cells that are in touch with the polygone.
 */

CellsRangeList GridmapLayout::getCellsUnderPolygon(const Polygon& _poly, std::unordered_set<std::pair<int, int>, boost::hash<std::pair<int, int> > > *boundaryCells) const //@TODO it was tested only with rectangles
{
//    CellsRangeSet indexMapSet;
    CellsRangeList indexMap, indexMapLine;
    Polygon gridPoly, poly = _poly;
    std::vector<Polygon> intersectionPoly;

    if(boundaryCells)
        boundaryCells->clear();

    if ( !isValid() ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The grid layout is not valid");
        return indexMap;
    }

    if (_poly.points.size() < 3){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The polygon must have at least 3 points");
        return indexMap;
    }

    arolib::geometry::correct_polygon(poly);
    gridPoly = getGridPolygon();
    intersectionPoly = arolib::geometry::get_likely_intersection(gridPoly, poly, m_cellsize*1e-3);

    if (intersectionPoly.empty())
        return indexMap;


    unsigned int indX, indY;
    unsigned int minX = std::numeric_limits<unsigned int>::max(), maxX = std::numeric_limits<unsigned int>::lowest();

    for (unsigned int i = 0; i < intersectionPoly.size() ; ++i){
        std::vector<Point> points = intersectionPoly.at(i).points;
        for (unsigned int j = 0; j+1 < points.size() ; ++j){
            if( point2index( points.at(j), indX, indY) ){
                minX = std::min( std::min(minX,indX) , m_sizeX-1 );
                maxX = std::min( std::max(maxX,indX) , m_sizeX-1 );
            }
        }
    }
    indexMap.initR(minX,maxX);
//    indexMapSet.initR(minX,maxX);

    std::map<int, std::vector< std::pair<int,int> >> indexMapTmp;

    int min_y, max_y;
    for (unsigned int i = 0; i < intersectionPoly.size() ; ++i){
        std::vector<Point> points = intersectionPoly.at(i).points;
        if (points.size()>2){
            for (unsigned int j = 0 ; j+1 < points.size() ; ++j ){
                unsigned int start_x, start_y, end_x, end_y;
                point2index( points.at(j), start_x, start_y );
                point2index( points.at(j+1), end_x, end_y );
                indexMapLine = getCellsUnderLine( points.at(j), points.at(j+1), false );

                Point pPrev = j > 0 ? points.at(j-1) : r_at(points, 1);
                Point pNext = j+2 < points.size() ? points.at(j+2) : points.at(1);
                bool changeInXdir = ( ( pPrev.x - points.at(j).x ) * ( points.at(j+1).x - points.at(j).x ) >= 0 );
                bool changeInXdir2 = ( ( pNext.x - points.at(j).x ) * ( points.at(j).x - points.at(j+1).x ) >= 0 );

                for (unsigned int x = indexMapLine.minX() ; x <= indexMapLine.maxX() ; ++x){

                    auto& rangesTmp = indexMapTmp[x];

                    auto yRanges = indexMapLine.getColumn(x);

                    if(yRanges.empty())
                        continue;

                    auto& yRange = *yRanges.begin();//since it is a line, there must be unly one range

                    min_y = yRange.first;
                    max_y = yRange.second;
                    if(min_y <= max_y){
//                        indexMapSet.insertInColumn(k,min_y);
//                        indexMapSet.insertInColumn(k,max_y);

                        if(boundaryCells){
                            for (int y = min_y ; y <= max_y ; ++y)
                                boundaryCells->insert( std::make_pair(x, y) );
                        }

                        if(!changeInXdir
                                && x == start_x && j != 0
                                && !rangesTmp.empty() ){//update the ranges from the x (column) corresponding to the last point of previous segment
                            auto & updatedRange = rangesTmp.back();
                            updatedRange.first = std::min(updatedRange.first, min_y);
                            updatedRange.second = std::max(updatedRange.second, max_y);
                        }
                        else if(!changeInXdir2
                                && x == end_x && j+2 == points.size()
                                && !rangesTmp.empty() ){//update the ranges from the x (column) corresponding to the first point of first segment
                            auto & updatedRange = rangesTmp.front();
                            updatedRange.first = std::min(updatedRange.first, min_y);
                            updatedRange.second = std::max(updatedRange.second, max_y);
                        }
                        else
                            rangesTmp.emplace_back( std::make_pair(min_y, max_y) );
                    }

                }
            }
        }
    }

    for(auto& ranges : indexMapTmp){
        std::sort(ranges.second.begin(), ranges.second.end(), [](const std::pair<int,int>& a, const std::pair<int,int>& b)->bool{
            if(a.first < b.first)
                return true;
            if(a.first > b.first)
                return false;
            return a.second < b.second;
        });
        for(size_t i = 0 ; i+1 < ranges.second.size() ; i+=2){
            auto& range1 = ranges.second.at(i);
            auto& range2 = ranges.second.at(i+1);
            indexMap.appendToColumn( ranges.first, range1.first, range2.second );
        }
        if(ranges.second.size() % 2 == 1){//@todo check if this does not affect the output
            indexMap.appendToColumn( ranges.first, ranges.second.back().first, ranges.second.back().second );
        }
    }

//    for (unsigned int i = indexMapSet.minX() ; i <= indexMapSet.maxX() ; ++i)
//        indexMap.appendToColumn( i, indexMapSet.getColumnMin(i), indexMapSet.getColumnMax(i) );

    return indexMap;
}

CellsRangeList GridmapLayout::getCellsUnderPolygon2(const Polygon& _poly, bool simple) const
{
    CellsRangeSet indexMapSet;
    CellsRangeList indexMap, indexMapLine;
    Polygon gridPoly, poly = _poly;
    std::vector<Polygon> intersectionPoly;

    if ( !isValid() ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The grid layout is not valid");
        return indexMap;
    }

    if (_poly.points.size() < 3){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The polygon must have at least 3 points");
        return indexMap;
    }

    arolib::geometry::correct_polygon(poly);
    gridPoly = getGridPolygon();
    intersectionPoly = arolib::geometry::get_likely_intersection(gridPoly, poly, m_cellsize*1e-3);

    if (intersectionPoly.empty())
        return indexMap;


    unsigned int indX, indY;
    unsigned int minX = std::numeric_limits<unsigned int>::max(), maxX = std::numeric_limits<unsigned int>::lowest();

    for (unsigned int i = 0; i < intersectionPoly.size() ; ++i){
        std::vector<Point> points = intersectionPoly.at(i).points;
        for (unsigned int j = 0; j+1 < points.size() ; ++j){
            if( point2index( points.at(j), indX, indY) ){
                minX = std::min( std::min(minX,indX) , m_sizeX-1 );
                maxX = std::min( std::max(maxX,indX) , m_sizeX-1 );
            }
        }
    }
    indexMap.initR(minX,maxX);
    indexMapSet.initR(minX,maxX);

    int min_y, max_y;
    for (unsigned int i = 0; i < intersectionPoly.size() ; ++i){
        std::vector<Point> points = intersectionPoly.at(i).points;
        if (points.size()>1){

            for (unsigned int j = 0 ; j+1 < points.size() ; ++j ){
                unsigned int start_x, start_y, end_x, end_y;
                point2index( points.at(j), start_x, start_y );
                point2index( points.at(j+1), end_x, end_y );

                if(start_x == end_x && start_y == end_y)
                    std::cout << "";

                indexMapLine = getCellsUnderLine( points.at(j), points.at(j+1), false );

                for (unsigned int k = indexMapLine.minX() ; k <= indexMapLine.maxX() ; ++k){

                    auto yRanges = indexMapLine.getColumn(k);

                    if(yRanges.empty())
                        continue;

                    auto& yRange = *yRanges.begin();//since it is a line, there must be unly one range

                    min_y = yRange.first;
                    max_y = yRange.second;
                    if(min_y <= max_y){
                        indexMapSet.insertInColumn(k,min_y);
                        indexMapSet.insertInColumn(k,max_y);

                    }

                }
            }
        }
    }


    for (unsigned int x = indexMapSet.minX() ; x <= indexMapSet.maxX() ; ++x){
        int minY = -1;
        for(int y = indexMapSet.getColumnMin(x) ; y <= indexMapSet.getColumnMax(x) ; ++y){
            Point pLL;
            getCellMinCorner(x, y, pLL);
            Point pC = pLL + Point(0.5*m_cellsize, 0.5*m_cellsize);
            Point pUL = pLL + Point(0, m_cellsize);
            Point pLR = pLL + Point(m_cellsize, 0);
            Point pUR = pLL + Point(m_cellsize, m_cellsize);
            if( !arolib::geometry::in_polygon(pC, poly) && ( simple || ( !arolib::geometry::in_polygon(pLL, poly) && !arolib::geometry::in_polygon(pUL, poly) && !arolib::geometry::in_polygon(pLR, poly) && !arolib::geometry::in_polygon(pUR, poly) ) ) ){
                if(minY >= 0){
                    indexMap.appendToColumn( x, minY, y );
                    minY = -1;
                }
            }
            else if(minY < 0)
                minY = y;
        }
        if(minY >= 0)
            indexMap.appendToColumn( x, minY, indexMapSet.getColumnMax(x) );
    }

    return indexMap;
}

/**
 * Get the indexes of the cells that are in touch with the polygon, as well as how much of the cell is under the polygon.
 */

std::vector< GridmapLayout::GridCellOverlap > GridmapLayout::getCellsOverlapUnderPolygon(const Polygon& _poly,
                                                           const Polygon& boundary) const
{
    return getCellsOverlapUnderPolygon_v2(_poly, boundary);
}

std::vector< GridmapLayout::GridCellOverlap > GridmapLayout::getCellsOverlapUnderPolygon_v1(const Polygon& _poly,
                                                           const Polygon& boundary) const
{
    CellsRangeList indexMap;
    std::vector< GridCellOverlap > cellsList;
    Polygon poly = _poly;
    double areaCell = getCellArea();

    if ( !isValid() ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The grid layout is not valid");
        return cellsList;
    }

    arolib::geometry::correct_polygon(poly);
    arolib::geometry::unsample_polygon(poly);
    indexMap = getCellsUnderPolygon(poly);
    if (indexMap.empty())
        return cellsList;

    if(boundary.points.size() > 2){
        auto sampledPolyPoints = arolib::geometry::sample_geometry(poly.points, m_cellsize*0.2);

        bool intersectPolys = false;

        if( !arolib::geometry::in_polygon(sampledPolyPoints, boundary, false) ){//the polygon is outside the boundary (it might be surrounding it)
            if( !arolib::geometry::in_polygon(boundary.points.front(), poly) )
                return cellsList;

            //all the boundary is inside the polygon
            intersectPolys = true;
        }
        else if( !arolib::geometry::in_polygon(sampledPolyPoints, boundary, true) )//not all the polygon is in the boundary
            intersectPolys = true;

        if(intersectPolys){
            auto intersections_boundary = arolib::geometry::get_likely_intersection(poly, boundary, m_cellsize*1e-3);
            if(intersections_boundary.empty())//shouldn't be the case, but just in case
                return cellsList;
            poly = intersections_boundary.front();
            arolib::geometry::unsample_polygon(poly);
        }
    }

    if(m_computeInMultiThread){
        std::vector< std::future< std::vector< GridCellOverlap >> > futures_x (indexMap.maxX() - indexMap.minX() + 1);
        for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
            auto& fu_x = futures_x.at(x - indexMap.minX());
            fu_x = std::async(std::launch::async,
                            [&/*, poly*/, x]()->std::vector< GridCellOverlap >{
                int min_y, max_y;
                std::vector< GridCellOverlap > ret;
                auto yRanges = indexMap.getColumn(x);
                int lastY = -1;
                for(const auto& yRange : yRanges){
                    min_y = yRange.first;
                    max_y = yRange.second;
                    if(lastY == min_y)
                        ++min_y;
                    lastY = max_y;
                    for (int y = min_y ; y <= max_y ; ++y){
                        double areaIntersection = getCellAreaIntersection(x, y, poly, min_y, max_y);
                        double overlap = std::min( areaIntersection/areaCell, 1.0 );
                        ret.emplace_back( GridCellOverlap(x,y,overlap) );
                    }
                }


                return ret;
            });
        }

        for(auto& fu_x : futures_x){
            auto listTmp = fu_x.get();
            cellsList.insert( cellsList.end(), listTmp.begin(), listTmp.end() );
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
                    double areaIntersection = getCellAreaIntersection(x, y, poly, min_y, max_y);
                    double overlap = std::min( areaIntersection/areaCell, 1.0 );
                    cellsList.push_back(  GridCellOverlap(x,y,overlap) );
                }
            }
        }
    }

    return cellsList;
}

std::vector< GridmapLayout::GridCellOverlap > GridmapLayout::getCellsOverlapUnderPolygon_v2(const Polygon& _poly,
                                                           const Polygon& boundary) const
{
    CellsRangeList indexMap;
    std::vector< GridCellOverlap > cellsList;
    Polygon poly = _poly;
    double areaCell = getCellArea();

    if ( !isValid() ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The grid layout is not valid");
        return cellsList;
    }

    arolib::geometry::correct_polygon(poly);
    arolib::geometry::unsample_polygon(poly);

    if(boundary.points.size() > 2){
        auto sampledPolyPoints = arolib::geometry::sample_geometry(poly.points, m_cellsize*0.2);

        bool intersectPolys = false;

        if( !arolib::geometry::in_polygon(sampledPolyPoints, boundary, false) ){//the polygon is outside the boundary (it might be surrounding it)
            if( !arolib::geometry::in_polygon(boundary.points.front(), poly) )
                return cellsList;

            //all the boundary is inside the polygon
            intersectPolys = true;
        }
        else if( !arolib::geometry::in_polygon(sampledPolyPoints, boundary, true) )//not all the polygon is in the boundary
            intersectPolys = true;

        if(intersectPolys){
            auto intersections_boundary = arolib::geometry::get_likely_intersection(poly, boundary, m_cellsize*1e-3);
            if(intersections_boundary.empty())//shouldn't be the case, but just in case
                return cellsList;
            poly = intersections_boundary.front();
            arolib::geometry::unsample_polygon(poly);
        }
    }

    //get the cells tha lie under the boundary of the poly
    std::unordered_set< std::pair<int, int>, boost::hash< std::pair<int, int> > > boundaryCells;


    indexMap = getCellsUnderPolygon(poly, &boundaryCells);
    if (indexMap.empty())
        return cellsList;

    if(m_computeInMultiThread){
        std::vector< std::future< std::vector< GridCellOverlap >> > futures_x (indexMap.maxX() - indexMap.minX() + 1);
        for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
            auto& fu_x = futures_x.at(x - indexMap.minX());
            fu_x = std::async(std::launch::async,
                            [&/*, poly*/, x]()->std::vector< GridCellOverlap >{
                int min_y, max_y;
                std::vector< GridCellOverlap > ret;
                auto yRanges = indexMap.getColumn(x);
                int lastY = -1;
                for(const auto& yRange : yRanges){
                    min_y = yRange.first;
                    max_y = yRange.second;
                    if(lastY == min_y)
                        ++min_y;
                    lastY = max_y;
                    for (int y = min_y ; y <= max_y ; ++y){
                        if(boundaryCells.find( std::make_pair(x, y) ) != boundaryCells.end()){
                            double areaIntersection = getCellAreaIntersection(x, y, poly, min_y, max_y);
                            double overlap = std::min( areaIntersection/areaCell, 1.0 );
                            ret.emplace_back( GridCellOverlap(x,y,overlap) );
                        }
                        else
                            ret.emplace_back( GridCellOverlap(x,y,1.0) );
                    }
                }


                return ret;
            });
        }

        for(auto& fu_x : futures_x){
            auto listTmp = fu_x.get();
            cellsList.insert( cellsList.end(), listTmp.begin(), listTmp.end() );
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
                    if(boundaryCells.find( std::make_pair(x, y) ) != boundaryCells.end()){
                        double areaIntersection = getCellAreaIntersection(x, y, poly, min_y, max_y);
                        double overlap = std::min( areaIntersection/areaCell, 1.0 );
                        cellsList.push_back(  GridCellOverlap(x,y,overlap) );
                    }
                    else
                        cellsList.emplace_back( GridCellOverlap(x,y,1.0) );
                }
            }
        }
    }

    return cellsList;
}

std::vector< GridmapLayout::GridCellOverlap > GridmapLayout::getCellsOverlapUnderPolygon_v3(const Polygon& _poly,
                                                           const Polygon& boundary) const
{
    CellsRangeList indexMap;
    std::vector< GridCellOverlap > cellsList;
    Polygon poly = _poly;

    if ( !isValid() ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The grid layout is not valid");
        return cellsList;
    }

    arolib::geometry::correct_polygon(poly);
    arolib::geometry::unsample_polygon(poly);

    int scale = 10;
    auto scaledLayout = *this;
    scaledLayout.scaleResolution(scale);

    indexMap = scaledLayout.getCellsUnderPolygon(poly);
    if (indexMap.empty())
        return cellsList;


    if(boundary.points.size() > 2){
        auto sampledPolyPoints = arolib::geometry::sample_geometry(poly.points, m_cellsize*0.2);

        bool intersectPolys = false;

        if( !arolib::geometry::in_polygon(sampledPolyPoints, boundary, false) ){//the polygon is outside the boundary (it might be surrounding it)
            if( !arolib::geometry::in_polygon(boundary.points.front(), poly) )
                return cellsList;

            //all the boundary is inside the polygon
            intersectPolys = true;
        }
        else if( !arolib::geometry::in_polygon(sampledPolyPoints, boundary, true) )//not all the polygon is in the boundary
            intersectPolys = true;

        if(intersectPolys){
            auto intersections_boundary = arolib::geometry::get_likely_intersection(poly, boundary, m_cellsize*1e-3);
            if(intersections_boundary.empty())//shouldn't be the case, but just in case
                return cellsList;
            poly = intersections_boundary.front();
            arolib::geometry::unsample_polygon(poly);
        }
    }

    double scaledCellsPerRealCell = scale * scale;
    int min_y, max_y;

    struct SumCells{
        int val = 0;
    };

    for (unsigned int x_scaled = indexMap.minX() ; x_scaled <= indexMap.maxX() ; ){
        std::unordered_map<int, SumCells> sumScaledCells;

        for (unsigned int x_step = 0 ; x_step < scale - x_scaled%scale && x_scaled+x_step <= indexMap.maxX() ; ++x_step){

            auto yRanges = indexMap.getColumn(x_scaled+x_step);
            int lastY = -1;
            for(const auto& yRange : yRanges){
                min_y = yRange.first;
                max_y = yRange.second;
                if(lastY == min_y)
                    ++min_y;
                lastY = max_y;
                for (int y_scaled = min_y ; y_scaled <= max_y ; ){
                    int sumScaledCells_y = std::min(scale - y_scaled % scale, max_y - y_scaled + 1);


                    if(sumScaledCells_y <= 0)
                        std::cout << "";

                    sumScaledCells[ y_scaled / scale ].val += sumScaledCells_y;
                    y_scaled += sumScaledCells_y;
                }
            }
        }
        for(auto& s : sumScaledCells)
            cellsList.push_back(  GridCellOverlap(x_scaled / scale, s.first, (double)s.second.val / scaledCellsPerRealCell ) );
        x_scaled += (scale - x_scaled % scale);
    }

    return cellsList;
}

std::vector<GridmapLayout::GridCellOverlap> GridmapLayout::toGridCellOverlap(const CellsRangeList &cellsList, float overlap)
{
    overlap = std::min(1.0f, std::max(0.0f, overlap));
    std::vector<GridCellOverlap> ret;
    int min_y, max_y;
    for (unsigned int x = cellsList.minX() ; x <= cellsList.maxX() ; ++x){
        auto yRanges = cellsList.getColumn(x);
        int lastY = -1;
        for(const auto& yRange : yRanges){
            min_y = yRange.first;
            max_y = yRange.second;
            if(lastY == min_y)
                ++min_y;
            lastY = max_y;
            for (int y = min_y ; y <= max_y ; ++y){
                ret.emplace_back( GridCellOverlap(x,y,overlap) );
            }
        }
    }
    return ret;
}


bool GridmapLayout::getCellCenter(unsigned int cellX, unsigned int cellY, Point &center) const
{
    bool ok = getCellMinCorner(cellX, cellY, center);
    center.x += 0.5*m_cellsize;
    center.y += 0.5*m_cellsize;
    return ok;

}

/**
 * Obtain the coordinates coresponding to the minimum corner of the given input cell.
 */

bool GridmapLayout::getCellMinCorner (unsigned int cellX, unsigned int cellY, Point &corner) const
{
    if ( !isValid() ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The grid layout is not valid");
        return false;
    }

    corner.x = m_minPointX + (double)cellX * m_cellsize;
    corner.y = m_minPointY + (double)cellY * m_cellsize;
    if ( !checkIndexesInRange(cellX,cellY) ) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Input cell indexes (" + std::to_string(cellX)
                                                         + "," + std::to_string(cellY)
                                                         + ") out of range : range(x) = [0," + std::to_string(m_sizeX-1)
                                                         + "] ; range(y) = [0," + std::to_string(m_sizeY-1) + " ]");
        return false;
    }
    return true;
}

/**
 * Obtain the line parameters coresponding to the given cell.
 */

bool GridmapLayout::getCellPolygonAsLine (unsigned int cellX, unsigned int cellY, Point &p0, Point &p1, double &width) const
{
    if ( !isValid() ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The grid layout is not valid");
        return false;
    }

    Point corner;

    if ( !getCellMinCorner(cellX,cellY,corner) )
        return false;

    p0 = p1 = ( corner + Point(0, m_cellsize * 0.5) );
    p0.x = corner.x;
    p1.x = corner.x + m_cellsize;
    width = m_cellsize;

    return true;
}

/**
 * Obtain the polygon coresponding to the given cell.
 */

bool GridmapLayout::getCellPolygon (unsigned int cellX, unsigned int cellY, Polygon &poly) const
{
    if ( !isValid() ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The grid layout is not valid");
        return false;
    }

    Point corner;
    poly.points.resize(5);

    if ( !getCellMinCorner(cellX,cellY,corner) )
        return false;

    poly.points.at(0) = ( corner );
    poly.points.at(1) = ( corner + Point(0, m_cellsize) );
    poly.points.at(2) = ( corner + Point(m_cellsize, m_cellsize) );
    poly.points.at(3) = ( corner + Point(m_cellsize, 0) );
    poly.points.at(4) = ( corner );

    return true;
}

/**
 * Obtain the polygon coresponding to the grid (in real world coordinates).
 * @return The polygon coresponding to the grid (in real world coordinates).
 */

Polygon GridmapLayout::getGridPolygon(bool *_error_) const
{
    Polygon poly;
    if(_error_) *_error_ = false;

    if(!isValid()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The grid layout is not valid");
        if(_error_) *_error_ = true;
        return poly;
    }

    poly.points.push_back( Point(m_minPointX, m_minPointY) );
    poly.points.push_back( Point(m_minPointX, m_maxPointY) );
    poly.points.push_back( Point(m_maxPointX, m_maxPointY) );
    poly.points.push_back( Point(m_maxPointX, m_minPointY) );
    poly.points.push_back( Point(m_minPointX, m_minPointY) );

    return poly;
}

double GridmapLayout::getGridArea(bool *_error_) const
{
    if(_error_) *_error_ = false;

    if(!isValid()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The grid layout is not valid");
        if(_error_) *_error_ = true;
        return 0;
    }

    return m_cellsize * m_cellsize * (m_sizeX * m_sizeY);
    //return (m_maxPointX - m_minPointX) * (m_maxPointY - m_minPointY);

}

double GridmapLayout::getCellAreaIntersection(const Polygon &cellPoly, double cellsize, const Polygon &poly, int minCell, int maxCell)
{
    double cellArea = cellsize*cellsize;
    double areaIntersection = 0;
    if ( (maxCell - minCell) > 4 && arolib::geometry::in_polygon(cellPoly, poly) ) //is more time-efficient to check this before instead of calculating direcly the area, assuming that most of the cells in the map are inside the polygon
        areaIntersection = cellArea;
    else{
        auto intersections = arolib::geometry::get_likely_intersection(poly, cellPoly, cellsize*1e-3);
        for (unsigned int k = 0 ; k < intersections.size() ; ++k)
            areaIntersection += std::min( arolib::geometry::calc_area( intersections.at(k) ), cellArea );
    }

    return areaIntersection;
}

double GridmapLayout::getCellAreaIntersection(const Polygon &cellPoly, double cellsize, const Polygon &linePoly, double lineWidth)
{
    double cellArea = cellsize*cellsize;
    double areaIntersection = 0;

    if ( lineWidth > cellsize*4.0 && arolib::geometry::in_polygon(cellPoly, linePoly) ) //is more time-efficient to check this before instead of calculating direcly the area, assuming that most of the cells in the map are inside the polygon
        areaIntersection = cellArea;
    else{
        auto intersections = arolib::geometry::get_likely_intersection(linePoly, cellPoly, cellsize*1e-3);
        for (unsigned int k = 0 ; k < intersections.size() ; ++k)
            areaIntersection += std::min( arolib::geometry::calc_area( intersections.at(k) ) , cellArea);
    }

    return areaIntersection;
}

double GridmapLayout::getCellAreaIntersection(unsigned int x, unsigned int y, const Polygon &poly, int minCell, int maxCell) const
{
    Polygon cellPoly;
    if( !getCellPolygon(x, y, cellPoly) )
        return 0;
    return getCellAreaIntersection(cellPoly, m_cellsize, poly, minCell, maxCell);
}

double GridmapLayout::getCellAreaIntersection(unsigned int x, unsigned int y, const Polygon &linePoly, double lineWidth) const
{
    Polygon cellPoly;
    if( !getCellPolygon(x, y, cellPoly) )
        return 0;
    return getCellAreaIntersection(cellPoly, m_cellsize, linePoly, lineWidth);
}

double GridmapLayout::getCellAreaIntersection(unsigned int x, unsigned int y, const Polygon &poly) const
{
    double cellsize_2 = 0.5 * m_cellsize;
    Point c;
    getCellCenter(x, y, c);
    std::vector<Point> controls = { Point(c.x-cellsize_2, c.y+cellsize_2), Point(c.x, c.y+cellsize_2), Point(c.x+cellsize_2, c.y+cellsize_2),
                                    Point(c.x-cellsize_2, c.y),            Point(c.x, c.y),            Point(c.x+cellsize_2, c.y),
                                    Point(c.x-cellsize_2, c.y-cellsize_2), Point(c.x, c.y-cellsize_2), Point(c.x+cellsize_2, c.y-cellsize_2) };

    float count = 0;

    for(auto& p : controls)
        count += arolib::geometry::in_polygon(p, poly);
    return getCellArea() * count / controls.size();

}


//------------------------------------
//----------------CHECK---------------
//------------------------------------

/**
 * Checks if the given point (in real world coordinates) is in the grid.
 */

bool GridmapLayout::checkPointInRange(const Point& point) const
{
    return !( point.x < (m_minPointX - m_limitTolerance) ||
              point.x > (m_maxPointX + m_limitTolerance) ||
              point.y < (m_minPointY - m_limitTolerance) ||
              point.y > (m_maxPointY + m_limitTolerance) );
}

/**
 * Checks if the x- and y- indexes lie within the grid's valid range.
 */

bool GridmapLayout::checkIndexesInRange(unsigned int x, unsigned int y) const
{
    return !( /*x < 0 ||*/ x >= m_sizeX || /*y < 0 ||*/ y >= m_sizeY );
}

/**
 * Checks if the x- and y- indexes lie within the grid's valid range.
 */

bool GridmapLayout::checkIndexesInRange(int x, int y) const
{
    if ( x < 0 /*|| x >= m_sizeX*/ || y < 0 /*|| y >= m_sizeY*/ )
        return false;
    return checkIndexesInRange( (unsigned int)x, (unsigned int)y );
}

//------------------------------------
//-------------CONVERSIONS------------
//------------------------------------

/**
 * Get the grid indexes corresponding to a point (in real-world coordinates).
 */

bool GridmapLayout::point2index(const Point& _p, unsigned int &x, unsigned int &y, bool withTolerance) const
{

    if ( !isValid() ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The grid layout is not valid");
        return false;
    }

    Point p = _p;

    if (withTolerance){
        if ( p.x >= m_maxPointX && p.x <= (m_maxPointX + m_limitTolerance) )
            p.x = m_maxPointX;
        if ( p.y >= m_maxPointY && p.y <= (m_maxPointY + m_limitTolerance) )
            p.y = m_maxPointY;

        if ( p.x <= m_minPointX && p.x >= (m_minPointX - m_limitTolerance) )
            p.x = m_minPointX;
        if ( p.y <= m_minPointY && p.y >= (m_minPointY - m_limitTolerance) )
            p.y = m_minPointY;
    }

    x = (unsigned int)( (p.x - m_minPointX)/m_cellsize /*+ 0.5*/ );
    y = (unsigned int)( (p.y - m_minPointY)/m_cellsize /*+ 0.5*/ );

    if ( p.x == m_maxPointX && x >= m_sizeX )
        x = m_sizeX-1;
    if ( p.y == m_maxPointY && y >= m_sizeY )
        y = m_sizeY-1;

    return checkIndexesInRange(x,y) && checkPointInRange(_p);
}


}
}


