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
 
#ifndef AROLIB_GEOMETRY_HELPER_TCC
#define AROLIB_GEOMETRY_HELPER_TCC
#include "arolib/geometry/geometry_helper.hpp"

namespace arolib{
namespace geometry{

template< typename T, typename >
double getGeometryLength(const std::vector<T>&geom,
                         int start_index,
                         int end_index){
    double length = 0.0;
    if(end_index < 0)
        end_index = geom.size()-1;
    if (start_index < 0 || start_index >= geom.size() || end_index >= geom.size())
        return 0;
    for (int i = start_index; i+1 <= end_index; ++i)
        length += calc_dist(geom.at(i), geom.at(i+1));
    return length;
}


template< typename T, typename >
bool getNormVector(const std::vector<T> &points, Point& vec, double length)
{
    vec = Point(0,0);
    Point vecTmp;
    if(points.size() < 2)
        return false;
    double geomLength = getGeometryLength(points);
    if(geomLength == 0)
        return false;
    for(size_t i = 0 ; i+1 < points.size() ; ++i){
        double d = calc_dist(points.at(i), points.at(i+1))/geomLength;
        if(getNormVector(points.at(i), points.at(i+1), vecTmp, d))
            vec += vecTmp;
    }
    if( !setVectorLength(vec, length) )
        return false;
    return true;
}


template< typename T, typename >
double calc_dist_to_linestring(const std::vector<T>& points, const Point& p, bool infinite){
    if(points.empty())
        return std::nan("");

    if(points.size() == 1)
        return calc_dist(points.at(0), p);

    if(points.size() == 2)
        return calc_dist_to_line(points.at(0), points.at(1), p, infinite, true);

    double minDist = calc_dist_to_line2(points.at(0), points.at(1), p, infinite, false, true);
    for(size_t i = 1 ; i+1 < points.size() ; ++i){
        double dist;
        if(i+1 != points.size()-1)
            dist = calc_dist_to_line(points.at(i), points.at(i+1), p, false, true);
        else
            dist = calc_dist_to_line2(points.at(i), points.at(i+1), p, false, infinite, true);
        if(minDist > dist)
            minDist = dist;
    }
    return minDist;
}


//template< typename T1, typename T2, typename >
//double calc_discrete_frechet_distance (const std::vector<T1>& points1, const std::vector<T2>& points2){
//    return boost::geometry::discrete_frechet_distance(toBoostLinesting(points1), toBoostLinesting(points2));
//}


template< typename T,  typename >
double getMinSampleDistance(const std::vector<T> &points)
{
    if (points.size() < 2)
        return -1;
    double dist = std::numeric_limits<double>::max();
    for(size_t i = 0 ; i+1 < points.size() ; ++i)
        dist = std::min( dist, calc_dist(points.at(i), points.at(i+1)) );
    return dist;
}


template< typename T, typename >
double getMaxSampleDistance(const std::vector<T> &points)
{
    double dist = -1;
    for(size_t i = 0 ; i+1 < points.size() ; ++i)
        dist = std::max( dist, calc_dist(points.at(i), points.at(i+1)) );
    return dist;
}


template< typename T, typename >
double getGeometryResolution(const std::vector<T>& geom){
    return getMaxSampleDistance(geom);
}


//----------------------------------------------------------------------


template< typename T, typename >
T rotate(const Point &pivot, const T &p, double angle, bool inDeg)
{
    T pRot = p;
    if(inDeg)
        angle = deg2rad(angle);
    double dist = calc_dist(pivot, p);
    double angle0 = get_angle(pivot, pivot + Point(10,0),
                              pivot, p);
    pRot.x = pivot.x + std::cos(angle0 + angle) * dist;
    pRot.y = pivot.y + std::sin(angle0 + angle) * dist;

    return pRot;

//    Point new_point;
//    if(inDeg)
//        angle *= (M_PI / 180);

//    double cosTheta = cos(angle);
//    double sinTheta = sin(angle);
//    new_point.x = (cosTheta * (pointToRotate.x - centerPoint.x) -
//                   sinTheta * (pointToRotate.y - centerPoint.y) + centerPoint.x);
//    new_point.y = (sinTheta * (pointToRotate.x - centerPoint.x)+
//                   cosTheta * (pointToRotate.y - centerPoint.y) + centerPoint.y);

//    return new_point;


}

template< typename T, typename >
std::vector<T> rotate(const Point &pivot, const std::vector<T> &points, double angle, bool inDeg)
{
    std::vector<T> points_rot = points;
    for(size_t i = 0 ; i < points.size() ; ++i){
        points_rot.at(i) = rotate(pivot, points.at(i), angle, inDeg);
    }
    return points_rot;
}


template< typename T, typename >
std::vector<T> translate(const std::vector<T>& points, const Point &direction)
{
    std::vector<T> ret = points;
    for(auto& p : ret)
        p += direction;

//    for(size_t i = 0 ; i < ret.size() ; ++i){
//        Point *p = static_cast<Point *>(&ret.at(i));
//        *p = points.at(i) + direction;
//    }
    return ret;
}


template< typename T, typename >
std::vector<T> translate(const std::vector<T>& points, double distance, const Point &direction){
    if (distance == 0)
        return points;
    Point dir = direction;
    setVectorLength(dir, distance);
    return translate(points, dir);
}


//----------------------------------------------------------------------

template< typename T, typename >
bool is_line(const std::vector<T>& points, double tolerance){
    if (points.size() < 2)
        return false;
    if (points.size() < 3)
        return true;

    tolerance = std::fabs(tolerance);
    size_t i2 = 0;

    //get the first point (from the back) different from the first point
    for (size_t i = points.size()-1 ; i > 0 ; --i){
        if ( calc_dist( points.at(0), points.at(i) ) > tolerance ){
            i2 = i;
            break;
        }
    }
    if(i2 == 0)
        return true;

    const Point& p1 = points.at(0);
    const Point& p2 = points.at(i2);
    double dist;
    for (size_t i = 1 ; i < i2 ; ++i){
        dist = calc_dist_to_line(p1, p2, points.at(i), false);
        if ( dist > tolerance )
            return false;
    }
    return true;
}

template< typename T, typename >
bool is_line2(const std::vector<T>& points, double tolerance){
    if (points.size() < 2)
        return false;
    if (points.size() < 3)
        return true;

    tolerance = std::fabs(tolerance);
    size_t i2 = points.size()-1;

    //get the first point different from the first point
    for (unsigned int i = 1 ; i < points.size() ; ++i){
        if ( calc_dist( points.at(0), points.at(i) ) > tolerance ){
            i2 = i;
            break;
        }
    }

    Point p1 = points.at(0);
    Point p2 = points.at(i2);
    double dist;
    for (size_t i = i2+1 ; i < points.size() ; ++i){
        dist = calc_dist_to_line(p1, p2, points.at(i));
        if ( dist > tolerance )
            return false;
    }
    return true;
}

template< typename T, typename >
bool is_line3(const std::vector<T>& points, double tolerance){
    if (points.size() < 2)
        return false;
    if (points.size() < 3)
        return true;

    tolerance = std::fabs(tolerance);
    size_t i2 = points.size()-1;

    //get the first point different from the first point
    for (unsigned int i = 1 ; i < points.size() ; ++i){
        if ( calc_dist( points.at(0), points.at(i) ) > tolerance ){
            i2 = i;
            break;
        }
    }

    Point p1 = points.at(0);
    Point p2 = points.at(i2);
    double y;
    for (size_t i = i2+1 ; i < points.size() ; ++i){
        if ( fabs( p2.x - p1.x ) < 0.000001 ){
            if ( fabs( points.at(i).x - p1.x ) > tolerance )
                return false;
        }
        else{
            y = interp_line(p1, p2, points.at(i).x);
            if ( fabs( points.at(i).y - y ) > tolerance )
                return false;
        }
    }
    return true;
}

template< typename T, typename >
bool is_line4(const std::vector<T>& points, double tolerance, double angTolerance){
    if (points.size() < 2)
        return false;
    if (points.size() < 3)
        return true;

    tolerance = std::fabs(tolerance);
    angTolerance = std::fabs(angTolerance);
    size_t i2 = 0;

    //get the first point (from the back) different from the first point
    for (size_t i = points.size()-1 ; i > 0 ; --i){
        if ( calc_dist( points.at(0), points.at(i) ) > tolerance ){
            i2 = i;
            break;
        }
    }
    if(i2 == 0)
        return true;

    const Point& p1 = points.at(0);
    const Point& p2 = points.at(i2);
    double angle;
    for (size_t i = 1 ; i < i2 ; ++i){
        if( calc_dist(p1, points.at(i)) < tolerance )
            continue;
        angle = get_angle(points.at(i), p1, p2);
        if ( angle > angTolerance )
            return false;
    }
    return true;
}



//----------------------------------------------------------------------

template< typename T, typename >
size_t count_points_in_polygon(const std::vector<T>& points, const Polygon& polygon){
    std::deque<Point> ring(polygon.points.begin(), polygon.points.end());
    size_t counter = 0;
    for(size_t i = 0; i < points.size(); i++){
        bool in = boost::geometry::within(points[i], ring);
        if(in) ++counter;
    }
    return counter;
}


template< typename T, typename >
std::vector<Point> sample_geometry(const std::vector<T>& geometry_in, double resolution, double minDist, bool reverse){
    std::vector<Point> geometry_out;
    resolution = fabs(resolution);
    if (resolution == 0)
        return geometry_out;

    if(minDist < -1e-6)
        minDist = resolution * 0.05;

    for(size_t i = 0; i + 1 < geometry_in.size(); i++){
        Point new_p;
        auto p0 = reverse ? r_at(geometry_in, i) : geometry_in[i];
        auto &p1 = reverse ? r_at(geometry_in, i+1) : geometry_in[i+1];
        geometry_out.push_back(p0);
        double dist = calc_dist(p0, p1);
        while(dist > resolution + minDist){
            new_p = getPointInLineAtDist(p0, p1,resolution);
            geometry_out.push_back(new_p);
            p0 = new_p;
            dist = calc_dist(p0, p1);
        }
    }
    if(geometry_in.size() > 0) {
        if (reverse)
            geometry_out.push_back(geometry_in.front());
        else
            geometry_out.push_back(geometry_in.back());
    }

    if(reverse)
        std::reverse(geometry_out.begin(), geometry_out.end());

    return geometry_out;
}

template< typename T, typename >
std::vector<Point> sample_geometry_segment(const std::vector<T>& geometry_in, double resolution, size_t indFrom, size_t indTo, double minDist){
    std::vector<Point> geometry_out;

    if(geometry_in.size() < 2){
        geometry_out = geometry_in;
        return geometry_out;
    }

    resolution = fabs(resolution);
    if (resolution == 0)
        return geometry_out;

    if(minDist < -1e-6)
        minDist = resolution * 0.05;

    bool reverse = false;

    if(indFrom > indTo){
        reverse = true;
        std::swap(indFrom, indTo);
    }

    indFrom = std::min(indFrom, geometry_in.size());
    indTo = std::min(indTo, geometry_in.size() - 1);

    if(indFrom >= indTo){
        geometry_out = geometry_in;
        return geometry_out;
    }

    int delta_i = reverse? -1 : 1;
    for(int i = reverse ? indTo : indFrom; i+delta_i <= indTo && i-delta_i >= indFrom; i += delta_i){
        auto p0 = geometry_in[i];
        auto &p1 = geometry_in[i+delta_i];
        geometry_out.push_back(p0);
        double dist = calc_dist(p0, p1);
        while(dist > resolution + minDist){
            p0 = getPointInLineAtDist(p0, p1, resolution);
            geometry_out.push_back(p0);
            dist = calc_dist(p0, p1);
        }
    }

    if(reverse)
        std::reverse(geometry_out.begin(), geometry_out.end());

    geometry_out.insert( geometry_out.begin(), geometry_in.begin(), geometry_in.begin()+indFrom );
    geometry_out.insert( geometry_out.end(), geometry_in.begin()+indTo, geometry_in.end() );

    return geometry_out;
}



template< typename T, typename >
std::vector<Point> sample_geometry_ends(const std::vector<T>& geometry_in, double resolution, double minDist){
    if (geometry_in.size() < 4) {
        return sample_geometry(geometry_in, resolution, minDist);
    }

    // sample from second point to first
    std::vector<Point> p1_to_p0;
    p1_to_p0.push_back(geometry_in[1]);
    p1_to_p0.push_back(geometry_in[0]);
    std::vector<Point> sampled_start = sample_geometry(p1_to_p0, resolution, minDist);
    std::reverse(sampled_start.begin(), sampled_start.end());

    std::vector<Point> endpoints;
    endpoints.push_back(geometry_in[geometry_in.size()-2]);
    endpoints.push_back(geometry_in[geometry_in.size()-1]);
    std::vector<Point> sampled_end = sample_geometry(endpoints, resolution, minDist);

    std::vector<Point> geometry_out;
    geometry_out.reserve(sampled_start.size() + sampled_end.size() + geometry_in.size() - 4);
    geometry_out.insert(geometry_out.end(), sampled_start.begin(), sampled_start.end());
    geometry_out.insert(geometry_out.end(), geometry_in.begin()+2, geometry_in.end()-2);
    geometry_out.insert(geometry_out.end(), sampled_end.begin()++, sampled_end.end());

    return geometry_out;
}



//----------------------------------------------------------------------

template< typename T, typename >
bool remove_repeated_points(std::vector<T>& points, bool leaveFirst, double tolerance){
    if (points.empty())
        return false;

    tolerance = std::fabs(tolerance);

    for ( size_t i = 0 ; i+1 < points.size() ; ++i ){
        if( calc_dist( points.at(i) , points.at(i+1) ) <= tolerance ){
            size_t j = i+2;
            while(j < points.size()){
                if( calc_dist( points.at(j-1) , points.at(j) ) > tolerance )
                    break;
                ++j;
            }
            points.erase(points.begin() + i + leaveFirst, points.begin() + j - !leaveFirst);
        }
    }
    return !points.empty();
}


template< typename T,  typename >
bool unsample_linestring(std::vector<T>& points, double tolerance, double angTolerance){

    tolerance = std::fabs(tolerance);
    angTolerance = std::fabs(angTolerance);

    for ( size_t i = 1 ; i+1 < points.size() ; ++i ){
        const Point& p1 = points.at(i-1);
        const Point& p2 = points.at(i);
        const Point& p3 = points.at(i+1);
        if( calc_dist(p1, p2) < tolerance || calc_dist(p2, p3) < tolerance
                || ( std::fabs( get_angle(p2, p1, p3) ) < angTolerance && calc_dist(p1, p2) < calc_dist(p1, p3) ) ){
            points.erase( points.begin() + i );
            i--;
        }
    }

    if( points.size() == 2 ){
        if(points.front() == points.back())
            points.pop_back();
        return true;
    }

    return !points.empty();
}

template< typename T, typename >
bool unsample_linestring2(std::vector<T>& points, double tolerance){
    if (points.empty())
        return false;
    if( points.size() == 1 )
        return true;
    if( points.size() == 2 ){
        if(points.front() == points.back())
            points.pop_back();
        return true;
    }

    tolerance = std::fabs(tolerance);

    for ( size_t i = 1 ; i+1 < points.size() ; ++i ){
        std::vector<Point> segment = { points.at(i-1) , points.at(i) , points.at(i+1) };
        if (is_line( segment, tolerance )){
            points.erase( points.begin() + i );
            i--;
        }
    }
    return !points.empty();
}

template< typename T, typename >
bool unsample_linestring__old(std::vector<T>& points, double tolerance){

    if (points.empty())
        return false;
    if (points.size() < 3)
        return true;

    tolerance = std::fabs(tolerance);

    for ( size_t i = 1 ; i+1 < points.size() ; ++i ){
        size_t ind_next = i+1;
        std::vector<Point> vec = { points.at(i-1) , points.at(i), points.at(ind_next) };
        bool isLine = false;
        while(is_line( vec, tolerance )){
            isLine = true;
            if(ind_next+1 >= points.size())
                break;
            vec.emplace_back( points.at(++ind_next) );
        }

        if(!isLine)
            continue;

        while( !is_line( std::vector<Point>(vec.rbegin(), vec.rbegin()+3), tolerance ) )
            vec.pop_back();

        double minDist = std::numeric_limits<double>::max();
        size_t min_j = 1;
        for(size_t j = 1 ; j+1 < vec.size() ; ++j){
            double dist = calc_dist( vec.at(j-1), vec.at(j) ) + calc_dist( vec.at(j), vec.at(j+1) );
            if(dist < minDist){
                minDist = dist;
                min_j = j;
            }
        }
        points.erase( points.begin() + (i + min_j - 1) );
        --i;
    }
    return !points.empty();
}



//----------------------------------------------------------------------

template< typename T, typename >
bool createPolygon(Polygon &poly, std::vector<T> ls, double width, bool clockwise){
    poly.points.clear();
    if(width <= 0)
        return false;
    if(!unsample_linestring(ls))
        return false;
    if(ls.size() < 2)
        return false;

    std::vector<Point> sideB;

    for(size_t i = 0 ; i+1 < ls.size() ; ++i){
        Point p0_A, p1_A, p0_B, p1_B;
        offset_line( ls.at(i), ls.at(i+1), p0_A, p1_A, 0.5*width );
        offset_line( ls.at(i), ls.at(i+1), p0_B, p1_B, -0.5*width );
        poly.points.emplace_back(p0_A);
        poly.points.emplace_back(p1_A);
        sideB.emplace_back(p0_B);
        sideB.emplace_back(p1_B);
    }
    poly.points.insert( poly.points.end(), sideB.rbegin(), sideB.rend() );
    closePolygon(poly);
    correct_polygon(poly, clockwise);

    return isPolygonValid(poly) && isPolygonClosed(poly) && isPolygonClockwise(poly) == clockwise;
}

template< typename T, typename >
Linestring toLinestring(const std::vector<T>& points){
    Linestring ls;
    ls.points.assign(points.begin(), points.end());
    return ls;
}


//----------------------------------------------------------------------

template< typename T, typename >
T getPointInMinDist(const std::vector<T> points, const Point &p){
    if(points.empty())
        throw std::invalid_argument( std::string(__FUNCTION__) + " geom.size must be > 0" );
    return points.at( getPointIndInMinDist(points, p ) );
}


template< typename T, typename >
size_t getPointIndInMinDist(const std::vector<T> points, const Point &p){
    if(points.empty())
        return 0;
    size_t ind = 0;
    double dist = calc_dist(points.at(0), p);
    for(size_t i = 1; i < points.size(); i++){
        double distTmp = calc_dist(points.at(i), p);
        if(distTmp < dist){
            dist = distTmp;
            ind = i;
        }
    }
    return ind;
}


template< typename T,
         typename >
std::pair<Point, int> getPointAtRelativeDist(const std::vector<T> &geom, double d){

    std::pair<Point, int> ret;
    double length_ls, dist_ls;
    double dist = 0;
    if (geom.empty()){
        ret.second = -1;
        return ret;
    }

    d = std::min( 1.0, std::max(0.0, d) );

    if( d + 1e-5 >= 1 ){
        ret.first = geom.back();
        ret.second = geom.size()-1;
        return ret;
    }

    length_ls = getGeometryLength(geom);
    dist_ls = d * length_ls;
    for(size_t i = 0; i+1 < geom.size(); i++){
        const auto &p1 = geom[i];
        const auto &p2 = geom[i+1];
        double point_dist = calc_dist(p1, p2);
        dist += point_dist;

        if( dist_ls > dist )
            continue;

        double diff = dist_ls - (dist - point_dist);
        ret.first = getPointInLineAtDist(p1, p2, diff);
        ret.second = i;
        break;
    }

    return ret;
}


template< typename T,
         typename >
std::pair<Point, int> getPointAtHalfLength(const std::vector<T> &geom){
    return getPointAtRelativeDist(geom, 0.5);
}


template< typename T, typename >
int getPointIndex(const std::vector<T> &geom, const Point &p){
   int index = -1;
   for(size_t i = 0; i < geom.size(); i++){
       bool equal = boost::geometry::equals(geom[i], p);
       if(equal){
           index = i;
           break;
       }
   }
   return index;
}



template< typename T, typename >
void getGeomIndices(const std::vector<T>& geom, const Point &p,
                   int &predecessor, int& sucessor){
   double dist, min_dist = std::numeric_limits<double>::max();
   for (unsigned int i = 0; i+1 < geom.size(); ++i) {
       dist = calc_dist_to_line(geom.at(i), geom.at(i+1), p, false, true);
       if(dist < min_dist) {
           min_dist = dist;
           predecessor = i;
           sucessor = i+1;
       }
   }
}


template< typename T, typename >
size_t getGeomIndex(const std::vector<T>& geom, const Point &p, bool distToLine){
   if (geom.size() == 0)
       throw std::invalid_argument( std::string(__FUNCTION__) + " geom.size must be > 0" );
   size_t min_id = 0;
   double d, min_dist = std::numeric_limits<double>::max();
   if(distToLine){
       for (unsigned int i = 0; i+1 < geom.size(); ++i) {
           d = calc_dist_to_line(geom.at(i), geom.at(i+1), p, false, true);
           if(d < min_dist) {
               min_dist = d;
               if( calc_dist(geom.at(i), p) < calc_dist(geom.at(i+1), p) )
                   min_id = i;
               else
                   min_id = i+1;
           }
       }
   }
   else{
       for (unsigned int i = 0; i < geom.size(); ++i) {
         d = calc_dist(p, geom.at(i));
         if(d < min_dist) {
           min_dist = d;
           min_id = i;
         }
       }
   }
   return min_id;
}


template< typename T, typename >
size_t getIndexFromContinuousGeometry(const std::vector<T>& geom, int ind, Point *p_out){
   if(geom.empty())
       throw std::invalid_argument( std::string(__FUNCTION__) + " geom.size must be > 0" );

   if(ind >= 0)
       ind = ind % geom.size();
   else{
       ind = -1 * (ind + 1);
       ind = ind % geom.size();
       ind = geom.size() - 1 - ind;
   }

   if(p_out)
     *p_out = geom.at(ind);
   return ind;
}


template< typename T, typename >
T getPointFromContinuousGeometry(const std::vector<T>& geom, int ind, size_t* indReal){
   if(geom.empty())
       throw std::invalid_argument( std::string(__FUNCTION__) + " geom.size must be > 0" );

   if(ind >= 0)
       ind = ind % geom.size();
   else{
       ind = -1 * (ind + 1);
       ind = ind % geom.size();
       ind = geom.size() - 1 - ind;
   }
   if(indReal)
       *indReal = ind;
   return geom.at(ind);
}


//-------------------------------------------------------------------------------------

template< typename T, typename >
std::vector<Point> getGeometryPart(const std::vector<T>& geom,
                                   int start_index, int end_index){
    std::vector<Point> geom_out;
    if (start_index < 0 || start_index >= geom.size()  || end_index < 0 || end_index > geom.size())
        return geom_out;
    if(start_index > end_index) {
      geom_out.insert(geom_out.begin(),geom.begin() + start_index,geom.end());
      geom_out.insert(geom_out.end(),geom.begin(), geom.begin() + end_index + 1);
    } else {
        if (end_index + 1 < geom.size()) {
            geom_out.insert(geom_out.begin(), geom.begin() + start_index, geom.begin() + end_index + 1);
        } else {
            geom_out.insert(geom_out.end(), geom.begin() + start_index, geom.end());
        }
    }
    return geom_out;
  }

template< typename T, typename >
std::vector<T> getShortestGeometryPart(const std::vector<T>& geom,
                                           const Point &p1, const Point &p2,
                                           size_t &start, size_t &end,
                                           bool distToLine){
    // get the indices for the given points
    size_t _start = getGeomIndex(geom,p1,distToLine);
    size_t _end = getGeomIndex(geom,p2,distToLine);

    // get the sub parts of the given geometry in both directions
    std::vector<T> part = getGeometryPart(geom, _start, _end);
    std::vector<T> part_reverse = getGeometryPart(geom, _end, _start);
    // return the shorter sub part
    if(getGeometryLength(part) > getGeometryLength(part_reverse)) {
        std::reverse(part_reverse.begin(), part_reverse.end());
        end = _start;
        start = _end;
        return part_reverse;
    }
    start = _start;
    end = _end;
    return part;
}

template< typename T, typename >
std::vector<T> getShortestGeometryPart(const std::vector<T>& geom,
                                       const Point &p1, const Point &p2,
                                       bool distToLine){
    size_t i,j;
    return getShortestGeometryPart(geom, p1, p2, i, j, distToLine);
}

template< typename T, typename >
std::vector<T> getLongestGeometryPart(const std::vector<T>& geom,
                                           const Point &p1, const Point &p2,
                                           size_t &start, size_t &end,
                                           bool distToLine){
    // get the indices for the given points
    size_t _start = getGeomIndex(geom,p1,distToLine);
    size_t _end = getGeomIndex(geom,p2,distToLine);

    // get the sub parts of the given geometry in both directions
    std::vector<T> part = getGeometryPart(geom, _start, _end);
    std::vector<T> part_reverse = getGeometryPart(geom, _end, _start);
    // return the shorter sub part
    if(getGeometryLength(part) < getGeometryLength(part_reverse)) {
        std::reverse(part_reverse.begin(), part_reverse.end());
        end = _start;
        start = _end;
        return part_reverse;
    }
    start = _start;
    end = _end;
    return part;
}

template< typename T, typename >
std::vector<T> getLongestGeometryPart(const std::vector<T>& geom,
                                           const Point &p1, const Point &p2,
                                           bool distToLine){
    size_t i,j;
    return getLongestGeometryPart(geom, p1, p2, i, j, distToLine);
}

//-------------------------------------------------------------------------------------


template< typename T, typename >
bool intersects(const Point& p0, const Point& p1, const std::vector<T>& points, bool p0ToInfinity, bool p1ToInfinity){
    typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> bg_point_t;
    typedef boost::geometry::model::linestring<bg_point_t> bg_linestring_t;

    if(!p0ToInfinity && !p1ToInfinity){
        bg_linestring_t line;
        boost::geometry::append(line, bg_point_t(p0.x, p0.y));
        boost::geometry::append(line, bg_point_t(p1.x, p1.y));
        bg_linestring_t seg = line;
        for(size_t i = 0 ; i+1 < points.size() ; ++i){
            seg.at(0) = bg_point_t(points.at(i).x, points.at(i).y);
            seg.at(1) = bg_point_t(points.at(i+1).x, points.at(i+1).y);
//            boost::geometry::append(seg, bg_point_t(points.at(i).x, points.at(i).y));
//            boost::geometry::append(seg, bg_point_t(points.at(i+1).x, points.at(i+1).y));
            if( boost::geometry::intersects( line, seg ) )
                return true;
        }
        return false;
    }

    for(size_t i = 0 ; i+1 < points.size() ; ++i){
        Point intersection;
        double length = calc_dist(p0,p1);
        if( get_intersection(p0,
                             p1,
                             points.at(i),
                             points.at(i+1),
                             intersection,
                             true,
                             false,
                             0) ){
            if(p0ToInfinity && p1ToInfinity)
                return true;
            if(p0ToInfinity){
                double dist = calc_dist(p0,intersection);
                if( dist <= length || dist < calc_dist(p1,intersection) )
                    return true;
            }
            else{
                double dist = calc_dist(p1,intersection);
                if( dist <= length || dist < calc_dist(p0,intersection) )
                    return true;
            }
        }
    }
    return false;
}


template< typename T1, typename T2, typename >
bool intersects(const std::vector<T1>& points1, const std::vector<T2>& points2, bool p0ToInfinity, bool pnToInfinity){
    for(size_t i = 0 ; i+1 < points1.size() ; ++i){
        if( intersects( points1.at(i),
                        points1.at(i+1),
                        points2,
                        i == 0 && p0ToInfinity,
                        i+2 == points1.size() && pnToInfinity) )
            return true;

    }
    return false;
}


template< typename T, typename >
bool intersects(const std::vector<T>& points, const Polygon& poly, bool p0ToInfinity, bool pnToInfinity){
    for(size_t i = 0 ; i+1 < points.size() ; ++i){
        if( intersects( points.at(i),
                        points.at(i+1),
                        poly,
                        i == 0 && p0ToInfinity,
                        i+2 == points.size() && pnToInfinity) )
            return true;

    }
    return false;
}


template< typename T,typename >
bool intersects(const std::vector<T>& geom){

    typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> bg_point_t;
    typedef boost::geometry::model::linestring<bg_point_t> bg_linestring_t;

    bg_linestring_t ls;

    for(auto& p : geom)
        boost::geometry::append(ls, bg_point_t(p.x, p.y));


    return boost::geometry::intersects( ls );
}


template< typename T, typename >
size_t count_intersections(const std::vector<T>& points, const Polygon& poly, bool p0ToInfinity, bool pnToInfinity, double tolerance){
    std::vector<Point> intersections;
    for(size_t i = 0 ; i+1 < points.size() ; ++i){
        std::vector<Point> intersectionsTmp = get_intersection( points.at(i),
                                                                points.at(i+1),
                                                                poly.points,
                                                                true,
                                                                i == 0 && p0ToInfinity,
                                                                i+2 == points.size() && pnToInfinity,
                                                                tolerance );
        if(!intersections.empty() && !intersectionsTmp.empty()){
            if(intersections.back() == intersectionsTmp.front())
                intersections.pop_back();
            else if (i+2 == points.size() && intersections.front() == intersectionsTmp.back())
                intersectionsTmp.pop_back();
        }
        intersections.insert(intersections.end(), intersectionsTmp.begin(), intersectionsTmp.end());
    }
    return intersections.size();
}

template< typename T, typename >
std::vector<Point> get_intersection(const std::vector<T>& ls, const Polygon& poly){
    std::vector<Point> ret;
    if(ls.empty() || poly.points.empty())
        return ret;
    std::deque<Point>boostLS (ls.begin(), ls.end());
    ret = get_intersection(boostLS, poly);
    return ret;
}


template< typename T1, typename T2, typename >
std::vector<Point> get_intersection(const std::vector<T1>& ls1, const std::vector<T2>& ls2){

    std::vector<Point> ret;
    if(ls1.empty() || ls2.empty())
        return ret;
    std::deque<Point>boostLS1 (ls1.begin(), ls1.end());
    std::deque<Point>boostLS2 (ls2.begin(), ls2.end());
    ret = get_intersection(boostLS1, boostLS2);
    return ret;
}

template< typename T,  typename >
std::vector<Point> get_intersection(const Point& p0,
                                    const Point& p1,
                                    const std::vector<T> &geom,
                                    bool orderToP0,
                                    bool p0ToInfinity,
                                    bool p1ToInfinity,
                                    double tolerance)
{
    std::vector<Point> intersections;
    if(p0 == p1)
        return intersections;
    std::map<double, Point, std::greater<double> > intersections1;
    std::map<double, Point > intersections2;

    for(size_t i = 0 ; i+1 < geom.size() ; ++i){
        Linestring intersections_tmp = get_intersection(p0,
                                                        p1,
                                                        geom.at(i),
                                                        geom.at(i+1),
                                                        true,
                                                        false,
                                                        tolerance);

        for(auto &intersection : intersections_tmp.points){
            double d1 = calc_dist(p1,intersection);
            int location = getLocationInLine(p0, p1, intersection/*, tolerance*/);
            if( location == 0 )
                intersections1[d1] = intersection;
            else{
                if( p0ToInfinity && location == -1 )
                    intersections1[d1] = intersection;
                else if( p1ToInfinity && location == 1 )
                    intersections2[d1] = intersection;
            }
        }

//          Point intersection;
//          if( get_intersection(p0,
//                               p1,
//                               geom.at(i),
//                               geom.at(i+1),
//                               intersection,
//                               true,
//                               false,
//                               tolerance) ){
//              double d1 = calc_dist(p1,intersection);
//              int location = getLocationInLine(p0, p1, intersection, tolerance);
//              if( location == 0 )
//                  intersections1[d1] = intersection;
//              else{
//                  if( p0ToInfinity && location == -1 )
//                      intersections1[d1] = intersection;
//                  else if( p1ToInfinity && location == 1 )
//                      intersections2[d1] = intersection;
//              }
//          }
    }
    for(auto &it : intersections1)
        intersections.push_back(it.second);
    for(auto &it : intersections2)
        intersections.push_back(it.second);

    if(!orderToP0)
        std::reverse(intersections.begin(), intersections.end());

    return intersections;

}

template< typename T, typename >
std::vector<Point> get_intersection(const std::vector<T> &geom,
                                    const Point& p0,
                                    const Point& p1,
                                    bool orderToGeomStart,
                                    bool p0ToInfinity,
                                    bool p1ToInfinity,
                                    double tolerance)
{
    std::vector<Point> intersections;
    if(p0 == p1)
        return intersections;

    for(size_t i = 0 ; i+1 < geom.size() ; ++i){
        Linestring intersections_tmp = get_intersection(p0,
                                                        p1,
                                                        geom.at(i),
                                                        geom.at(i+1),
                                                        true,
                                                        false,
                                                        tolerance);

        for(auto &intersection : intersections_tmp.points){
            double d1 = calc_dist(p1,intersection);
            int location = getLocationInLine(p0, p1, intersection/*, tolerance*/);
            if( location == 0 )
                intersections.emplace_back(intersection);
            else{
                if( p0ToInfinity && location == -1 )
                    intersections.emplace_back(intersection);
                else if( p1ToInfinity && location == 1 )
                    intersections.emplace_back(intersection);
            }
        }
    }

    if(!orderToGeomStart)
        std::reverse(intersections.begin(), intersections.end());

    return intersections;

}


//-------------------------------------------------------------------------------------


template< typename T, typename >
bool getGeometryLimits(const std::vector<T> &geom, double &minX, double &maxX, double &minY, double &maxY){
    if(geom.empty())
        return false;
    minX = geom.at(0).x;
    maxX = geom.at(0).x;
    minY = geom.at(0).y;
    maxY = geom.at(0).y;
    for(auto &p : geom){
        minX = std::min( minX, p.x );
        maxX = std::max( maxX, p.x );
        minY = std::min( minY, p.y );
        maxY = std::max( maxY, p.y );
    }
    return true;
}


template< typename T, typename >
bool getGeometryBoundingBox(const std::vector<T> &geom, Polygon &bbox){
    if(geom.empty())
        return false;
    double minX, maxX, minY, maxY;
    if( !getGeometryLimits(geom, minX, maxX, minY, maxY) )
        return false;

    bbox.points.resize(5);
    bbox.points.at(0) = Point(minX, minY);
    bbox.points.at(1) = Point(minX, maxY);
    bbox.points.at(2) = Point(maxX, maxY);
    bbox.points.at(3) = Point(maxX, minY);
    bbox.points.at(4) = Point(minX, minY);

    return true;
}


template< typename T, typename >
long getLeftmostPointInd(const std::vector<T>& points){
    long ind = -1;
    double ref = std::numeric_limits<double>::max();
    for (size_t i = 0 ; i < points.size() ; ++i){
        if( ref > points.at(i).x ){
            ref = points.at(i).x;
            ind = i;
        }
    }
    return ind;
}


template< typename T, typename >
long getRightmostPointInd(const std::vector<T>& points){
    long ind = -1;
    double ref = std::numeric_limits<double>::lowest();
    for (size_t i = 0 ; i < points.size() ; ++i){
        if( ref < points.at(i).x ){
            ref = points.at(i).x;
            ind = i;
        }
    }
    return ind;
}


template< typename T, typename >
long getLowermostPointInd(const std::vector<T>& points){
    long ind = -1;
    double ref = std::numeric_limits<double>::max();
    for (size_t i = 0 ; i < points.size() ; ++i){
        if( ref > points.at(i).y ){
            ref = points.at(i).y;
            ind = i;
        }
    }
    return ind;
}


template< typename T, typename >
long getUppermostPointInd(const std::vector<T>& points){
    long ind = -1;
    double ref = std::numeric_limits<double>::lowest();
    for (size_t i = 0 ; i < points.size() ; ++i){
        if( ref < points.at(i).y ){
            ref = points.at(i).y;
            ind = i;
        }
    }
    return ind;
}


template< typename T, typename >
T getLeftmostPoint(const std::vector<T>& points){
    long ind = getLeftmostPointInd(points);
    if(ind < 0)
        throw std::invalid_argument( std::string(__FUNCTION__) + " geom.size must be > 0" );
    return points.at(ind);
}


template< typename T, typename >
T getRightmostPoint(const std::vector<T>& points){
    long ind = getRightmostPointInd(points);
    if(ind < 0)
        throw std::invalid_argument( std::string(__FUNCTION__) + " geom.size must be > 0" );
    return points.at(ind);
}


template< typename T, typename >
T getLowermostPoint(const std::vector<T>& points){
    long ind = getLowermostPointInd(points);
    if(ind < 0)
        throw std::invalid_argument( std::string(__FUNCTION__) + " geom.size must be > 0" );
    return points.at(ind);
}


template< typename T, typename >
T getUppermostPoint(const std::vector<T>& points){
    long ind = getUppermostPointInd(points);
    if(ind < 0)
        throw std::invalid_argument( std::string(__FUNCTION__) + " geom.size must be > 0" );
    return points.at(ind);
}


}
}

#endif //AROLIB_GEOMETRY_HELPER_TCC
