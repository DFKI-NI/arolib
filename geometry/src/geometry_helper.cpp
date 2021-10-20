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

#include "arolib/geometry/geometry_helper.hpp"

BOOST_GEOMETRY_REGISTER_POINT_2D(arolib::Point, double, boost::geometry::cs::cartesian, x, y)
BOOST_GEOMETRY_REGISTER_RING(std::vector<arolib::Point>)
BOOST_GEOMETRY_REGISTER_LINESTRING(std::deque<arolib::Point>)


namespace arolib{
namespace geometry{

bool testEqual(double a, double b)
{
    const double EPSILON = 1e-9;
    return fabs(a - b) < 1e-9;
}

//----------------------------

double calc_dist(const Point &p0, const Point &p1){
    return Point::dist(p0, p1);
}

double calc_manhattan_dist(const Point& p0, const Point& p1, bool includeZ){
    return std::fabs(p0.x - p1.x)
            + std::fabs(p0.y - p1.y)
            + ( includeZ ? std::fabs(p0.z - p1.z) : 0.0 );
}

double calc_area(const Point &p0, const Point &p1, double width)
{
    return width * calc_dist(p0,p1);
}

double calc_area(const Polygon &poly)
{
    return std::fabs( boost::geometry::area(poly.points) );
}

void correct_angle(double &ang, bool inDeg, bool onlyPositive){
    const double _2_pi_ = 2 * M_PI;

    if(inDeg)
        ang = deg2rad( ang );

    if(ang >= 0)
        ang = std::fmod(ang, _2_pi_);
    else
        ang = _2_pi_ - std::fmod(-ang, _2_pi_);

    if(ang >= _2_pi_)
        ang = 0;

    if(!onlyPositive && ang > M_PI)
        ang -= _2_pi_;

    if (inDeg)
        ang = rad2deg(ang);
}

double get_angle(const Point &p0, const Point &p1, bool inDeg, bool limit)
{
    double angle = std::atan2( (p1.y - p0.y) , (p1.x - p0.x) );

    if(angle == -M_PI)
        angle = M_PI;

    if (limit){
        if (angle<-M_PI_2) angle+=M_PI_2;
        else if (angle>M_PI_2) angle-=M_PI_2;
    }
    if (inDeg)
        return rad2deg(angle);

    return angle;
}

double get_angle(const Point &p1_0,
                 const Point &p1_1,
                 const Point &p2_0,
                 const Point &p2_1,
                 bool inDeg,
                 bool limit)
{
    if(p1_0 == p1_1 || p2_0 == p2_1)
        return 0;

    double angle =   std::atan2( (p2_1.y - p2_0.y) , (p2_1.x - p2_0.x) )
                   - std::atan2( (p1_1.y - p1_0.y) , (p1_1.x - p1_0.x) );
//      double angle = std::atan2( (p2_1.y - p2_0.y) , (p2_1.x - p2_0.x) );
//      angle -= std::atan2( (p1_1.y - p1_0.y) , (p1_1.x - p1_0.x) );

    if(angle > M_PI)
        angle -= 2 * M_PI;
    if(angle < - M_PI)
        angle += 2 * M_PI;

    if (limit){
        if (angle<-M_PI_2) angle+=M_PI;
        else if (angle>M_PI_2) angle-=M_PI;
    }
    if (inDeg)
        return rad2deg(angle);

    return angle;

}

double get_angle(const Point &p1, const Point &pivot, const Point &p2, bool inDeg, bool limit)
{
    return get_angle(pivot, p1, pivot, p2, inDeg, limit);
}

double get_bearing(const Point &p0, const Point &p1, bool inUTM){

    Point p0_geo = p0;
    Point p1_geo = p1;

    if(inUTM){
        CoordTransformer::GetInstance().convert_to_geodetic(p0, p0_geo);
        CoordTransformer::GetInstance().convert_to_geodetic(p1, p1_geo);
    }

    return Point::bearing(p0_geo, p1_geo);
}

bool getNormVector(const Point &p0, const Point &p1, double &dX, double &dY, double length){
    double deltaX = p1.x - p0.x;
    double deltaY = p1.y - p0.y;
    double len = sqrt(deltaX*deltaX + deltaY*deltaY);
    if(len == 0){
        dX = 0;
        dY = 0;
        return false;
    }
    dX = length * (deltaY)/len;
    dY = length * (-deltaX)/len;
    return true;
}

bool getNormVector(const Point &p0, const Point &p1, Point &vec, double length)
{
    return getNormVector(p0, p1, vec.x, vec.y, length);
}

bool getParallelVector(const Point &p0, const Point &p1, double &dX, double &dY, const double length){
    double deltaX = p1.x - p0.x;
    double deltaY = p1.y - p0.y;
    double len = sqrt(deltaX*deltaX + deltaY*deltaY);
    if(len == 0){
        dX = 0;
        dY = 0;
        return false;
    }
    dX = length * deltaX/len;
    dY = length * deltaY/len;
    return true;
}

bool getParallelVector(const Point &p0, const Point &p1, Point &vec, const double length)
{
    return getParallelVector(p0, p1, vec.x, vec.y, length);
}

Point getCentroid(const Point &p0, const Point &p1)
{
    return Point( p0.x + 0.5 * (p1.x - p0.x) , p0.y + 0.5 * (p1.y - p0.y) );
}

bool getCentroid(const Polygon& poly, Point &centroid)
{
    try{
        auto polyValidity = isPolygonValid(poly);
        if(polyValidity == PolygonValidity::INVALID_POLYGON){
            std::cout <<"[ERROR : getCentroid] The input polygon is not valid" << std::endl;
            return false;
        }

        typedef boost::geometry::model::d2::point_xy<double> boost_point_t;
        typedef boost::geometry::model::polygon<boost_point_t> boost_polygon_t;

        boost_polygon_t input;

        for (auto && p : poly.points) {
            boost::geometry::append(input, boost_point_t(p.x, p.y));
        }

        if( !isPolygonClosed(poly) )
            boost::geometry::append(input, boost_point_t(poly.points.front().x, poly.points.front().y));

        boost::geometry::model::d2::point_xy<double> p;
        boost::geometry::centroid(input, p);

        centroid.x = p.x();
        centroid.y = p.y();

        return true;
    }
    catch(...){
        return false;
    }
}


double dot(const Point& v, const Point& w)
{
  return v.x * w.x + v.y * w.y;
}

double calc_distance(const Point& v, const Point& w, const Point& p) {
  // Return minimum distance between line segment vw and point p
  const double l2 = calc_dist(v, w);  // i.e. |w-v|^2
  if (l2 == 0.0) return calc_dist(p, v); // v == w case
  // Consider the line extending the segment, parameterized as v + t (w - v).
  // We find projection of point p onto the line.
  // It falls where t = [(p-v) . (w-v)] / |w-v|^2
  const double t = dot(p - v, w - v) / l2;
  if (t < 0.0)  // Beyond the 'v' end of the segment
    return calc_dist(p, v);
  else if (t > 1.0) // Beyond the 'w' end of the segment
    return calc_dist(p, w);
  // Projection falls on the segment
  Point projection = v + (w - v) * t;
  return calc_dist(p, projection);
}


double calc_dist_to_line(const Point &p0, const Point &p1, const Point &p, bool infinite, bool abs)
{
  if (p0 == p1)
      return calc_dist(p0, p);
  double ang = get_angle( p0, p1, p );
  double d_p_p1 = calc_dist(p1, p);
  double dist = d_p_p1 * std::sin(ang);

  if(!infinite){
      double ang1 = get_angle(p0, p, p0, p1, false, false);
      double ang2 = get_angle(p1, p, p1, p0, false, false);
      if( fabs(ang1) > M_PI_2 || fabs(ang2) > M_PI_2 ){
          bool neg = dist < 0;
          dist = std::min( calc_dist(p, p0) , calc_dist(p, p1) );
          if(neg)
              dist = -dist;
      }
  }

  if(abs)
      dist = fabs(dist);

  return dist;
}


double calc_dist_to_line2(const Point& p0,
                          const Point& p1,
                          const Point& p,
                          bool p0ToInfinity,
                          bool p1ToInfinity,
                          bool abs)
{
    if(p0ToInfinity == p1ToInfinity)
        return calc_dist_to_line(p0, p1, p, p0ToInfinity, abs);

    if (p0 == p1)
        return calc_dist(p0, p);

    double dx = p1.x - p0.x;
    double dy = p1.y - p0.y;
    double dist = ( dy*p.x - dx*p.y + p1.x*p0.y - p0.x*p1.y ) / sqrt(dx*dx + dy*dy);

    double ang1 = get_angle(p0, p, p0, p1, false, false);
    double ang2 = get_angle(p1, p, p1, p0, false, false);
    if( fabs(ang1) > M_PI_2 || fabs(ang2) > M_PI_2 ){
        bool neg = dist < 0;

        if(p0ToInfinity && calc_dist(p, p1) < calc_dist(p, p0)){
            dist = calc_dist(p, p1);
            if(neg)
                dist = -dist;
        }
        else if (p1ToInfinity && calc_dist(p, p0) < calc_dist(p, p1) ){
            dist = calc_dist(p, p0);
            if(neg)
                dist = -dist;
        }
    }

    if(abs)
        dist = fabs(dist);

    return dist;

}

Point calc_vector_to_line(const Point &p0, const Point &p1, const Point &p, bool infinite)
{
    Point ret = p;
    double dist = calc_dist_to_line(p0,p1,p,true, false);
    getNormVector(p0,p1,ret,dist);
    ret = Point(0,0)-ret;
    if(infinite)
        return ret;

    double ang1 = get_angle(p0, p, p0, p1, false, false);
    double ang2 = get_angle(p1, p, p1, p0, false, false);

    if( fabs(ang1) > M_PI_2 || fabs(ang2) > M_PI_2 ){
        if( calc_dist(p, p0) < calc_dist(p, p1) )
            ret = p0 - p;
        else
            ret = p1 - p;
    }

    return ret;
}



//-----------------------------------------------------------------

PolygonValidity isPolygonValid(const Polygon &poly){
    if(poly.points.size() < 3)
        return PolygonValidity::INVALID_POLYGON;
    std::vector<Point> ring(poly.points.begin(), poly.points.end());

    bool originallyClosed = poly.points.front() == poly.points.back();
    bool originallyCW = true;
    if(!originallyClosed)
        ring.push_back(poly.points.front());

    boost::geometry::validity_failure_type failure;
    boost::geometry::is_valid(ring, failure);
    if(failure == boost::geometry::failure_wrong_orientation){
        originallyCW = false;
        std::reverse(ring.begin(), ring.end());
        boost::geometry::is_valid(ring, failure);
        if(failure == boost::geometry::failure_wrong_orientation){//boost is failing here sometimes!
            std::cout << "[WARNING : isPolygonValid] Boost is_valid failed: double failure_wrong_orientation" << std::endl;
            failure = boost::geometry::no_failure;
        }
    }

    if(failure == boost::geometry::no_failure){
        if(originallyClosed && originallyCW)
            return PolygonValidity::VALID_CLOSED_CW;
        if(originallyClosed && !originallyCW)
            return PolygonValidity::VALID_CLOSED_CCW;
        if(!originallyClosed && originallyCW)
            return PolygonValidity::VALID_OPEN_CW;
        return PolygonValidity::VALID_OPEN_CCW;
    }
    return PolygonValidity::INVALID_POLYGON;
}

PolygonValidity isPolygonValid(std::vector<Point> ring){
    boost::geometry::validity_failure_type failure;
    boost::geometry::is_valid(ring, failure);
    if(failure == boost::geometry::no_failure)
        return PolygonValidity::VALID_CLOSED_CW;
    if(failure == boost::geometry::failure_not_closed){
        ring.push_back(ring.at(0));
        boost::geometry::is_valid(ring, failure);
        if(failure == boost::geometry::no_failure)
            return PolygonValidity::VALID_OPEN_CW;
        if(failure == boost::geometry::failure_wrong_orientation)
            return PolygonValidity::VALID_OPEN_CCW;
        if(failure == boost::geometry::failure_not_closed){//boost is failing here sometimes!
            std::cout << "[WARNING : isPolygonValid] Boost is_valid failed: double failure_not_closed" << std::endl;
            return PolygonValidity::VALID_OPEN_CW;
        }
    }
    else if(failure == boost::geometry::failure_wrong_orientation){
        std::reverse(ring.begin(), ring.end());
        boost::geometry::is_valid(ring, failure);
        if(failure == boost::geometry::no_failure)
            return PolygonValidity::VALID_CLOSED_CCW;
        if(failure == boost::geometry::failure_not_closed)
            return PolygonValidity::VALID_OPEN_CCW;
        if(failure == boost::geometry::failure_wrong_orientation){//boost is failing here sometimes!
            std::cout << "[WARNING : isPolygonValid] Boost is_valid failed: double failure_wrong_orientation" << std::endl;
            return PolygonValidity::VALID_CLOSED_CCW;
        }
    }
    return PolygonValidity::INVALID_POLYGON;
}

bool isPolygonClosed(const PolygonValidity &polyValidty)
{
    return polyValidty == PolygonValidity::VALID_CLOSED_CW ||
           polyValidty == PolygonValidity::VALID_CLOSED_CCW ;
}
bool isPolygonClosed(const Polygon &poly)
{
    return isPolygonClosed( isPolygonValid(poly) ) ;
}

bool isPolygonClockwise(const PolygonValidity &polyValidty)
{
    return polyValidty == PolygonValidity::VALID_CLOSED_CW ||
            polyValidty == PolygonValidity::VALID_OPEN_CW ;
}

bool isPolygonClockwise(const Polygon &poly)
{
    return isPolygonClockwise( isPolygonValid(poly) ) ;
}

void correct_polygon(Polygon &poly, bool clockwise)
{
    boost::geometry::correct(poly.points);
    if(!clockwise)
        std::reverse(poly.points.begin(), poly.points.end());
}

bool closePolygon(Polygon &poly, double tolerance)
{
    tolerance = std::fabs(tolerance);
      if(poly.points.empty())
          return false;
//      if(poly.points.size() < 3)
//          return false;
    if ( std::fabs( poly.points.at(0).x - poly.points.back().x ) <= tolerance &&
         std::fabs( poly.points.at(0).y - poly.points.back().y ) <= tolerance ){
        poly.points.back() = poly.points.at(0);
        return true;
    }
    poly.points.push_back(poly.points.at(0));
    return true;
}

bool openPolygon(Polygon &poly, double tolerance)
{
    tolerance = std::fabs(tolerance);
    if(poly.points.empty())
        return false;
//      if(poly.points.size() < 4)
//          return false;
    while ( std::fabs( poly.points.at(0).x - poly.points.back().x ) <= tolerance &&
            std::fabs( poly.points.at(0).y - poly.points.back().y ) <= tolerance ){
        poly.points.pop_back();
        if( poly.points.empty() )
            return false;
    }
    return true;
}

//-----------------------------------------------------------------

bool setVectorLength(Point &vec, double length)
{
    if(vec == Point(0,0))
        return false;
    Point vec2 = vec;
    return getParallelVector( Point(0,0), vec2, vec, length );
}

bool setVectorLength(const Point &p0, Point &p1, double length)
{
    if(p0 == p1)
        return false;

    if (length == 0){
        p1 = p0;
        return true;
    }

    Point vec;
    getParallelVector( p0, p1, vec, length );
    p1 = p0 + vec;

    return true;
}

bool move_point_to_dist_to_line(const Point &p0, const Point &p1, Point &p, double d, bool abs)
{
    if(abs)
        d = std::fabs(d);
    double dist = calc_dist_to_line(p0, p1, p, false, abs);
    if( fabs(dist) < 0.0001 )
        return false;
    Point p2 = p + calc_vector_to_line(p0, p1, p, false);
    p = extend_line(p2, p, d-dist);
    double distNew = calc_dist_to_line(p0, p1, p, false, abs);
    if( std::fabs( d - distNew ) > 0.001
            && std::fabs( dist - distNew ) > 0.0001 )
        return move_point_to_dist_to_line(p0, p1, p, d, abs);

    return true;
}

double interp_line(const Point &p0, const Point &p1, double x){
    if (p0.x == p1.x)
        return ( p0.y + (p1.y-p0.y)/2.0 );
    return ( (x - p0.x) * (p1.y-p0.y)/(p1.x-p0.x) + p0.y );
}


Point getPointAtDist(const Pose2D &pose, double dist)
{
    return rotate(pose, Point(dist, 0) + pose, pose.angle);
}

Point extend_line(const Point &p0, const Point &p1, double dist){
    Point new_p1;
    double len;
    double dx = p1.x - p0.x;
    double dy = p1.y - p0.y;
    len = sqrt(dx*dx + dy*dy);

    if(len == 0)
        return p0;
    new_p1.x = p1.x + dist * dx/len;
    new_p1.y = p1.y + dist * dy/len;

    dx = new_p1.x - p0.x;
    dy = new_p1.y - p0.y;
    len = sqrt(dx*dx + dy*dy);

    return new_p1;
}

bool extend_linestring(std::vector<Point>& ls, double dist0, double distn, bool keepExtremaPoints){
    if(ls.size() < 2 || dist0 < 1e-9 || distn < 1e-9 )
        return false;

    if(dist0 > 1e-9){
        if(keepExtremaPoints)
            push_front(ls, Point());
        ls.front() = extend_line( ls.at(keepExtremaPoints+1), ls.at(keepExtremaPoints), dist0 );
    }

    if(distn > 1e-9){
        if(keepExtremaPoints)
            ls.push_back(Point());
        ls.back() = extend_line( r_at(ls, keepExtremaPoints+1), r_at(ls,keepExtremaPoints), distn );
    }

    return true;
}

void offset_line(const Point &p0, const Point &p1, Point &new_p0, Point &new_p1, double offset){
    double len, dx, dy, offset_x, offset_y;
    dx = p1.x - p0.x;
    dy = p1.y - p0.y;
    len = sqrt(dx*dx + dy*dy);

    if(len == 0){
        new_p0 = p0;
        new_p1 = p1;
        return;
    }
    offset_x = offset * (dy)/len;
    offset_y = offset * (-dx)/len;

    new_p0.x = p0.x + offset_x;
    new_p0.y = p0.y + offset_y;
    new_p1.x = p1.x + offset_x;
    new_p1.y = p1.y + offset_y;
}

bool offsetLinestring(const std::vector<Point> &points_in,
                      std::vector<Point> &points_out,
                      double offset,
                      bool keepSamples, int points_per_circle){
    return offsetLinestring_boost(points_in, points_out, offset, keepSamples, points_per_circle);
    //return offsetLinestring_old(points_in, points_out, offset);
}

bool offsetLinestring(const std::vector<Point> &points_in,
                      Polygon &poly_out,
                      double offset1,
                      double offset2,
                      bool endFlat,
                      int points_per_circle){
    return offsetLinestring_boost(points_in, poly_out, offset1, offset2, endFlat, points_per_circle);
}

bool offsetLinestring_boost(const std::vector<Point> &points_in,
                            std::vector<Point> &points_out,
                            double offset,
                            bool keepSamples,
                            int points_per_circle){
    if(points_in.size() < 2){
        std::cout << "[ERROR - offsetLinestring_boost] Input points have less that 2 points" << std::endl;
        return false;
    }

    if(offset == 0){
        points_out = points_in;
        return true;
    }

    points_out.clear();
    double offset1, offset2;
    if(offset > 0){
        offset1 = offset;
        offset2 = 0.5 * offset;
    }
    else{
        offset1 = -0.5 * offset;
        offset2 = -offset;
    }

    Polygon poly;
    if(!offsetLinestring_boost(points_in, poly, offset1, offset2, true, points_per_circle))
        return false;

    if(poly.points.empty())
        return false;

    openPolygon(poly);
    if(offset < 0)
        std::reverse(poly.points.begin(), poly.points.end());

    double ang = offset > 0 ? M_PI_2 : -M_PI_2 ;
    offset = std::fabs(offset);

    Point pRef = rotate(points_in.front(), points_in.at(1), ang);
    setVectorLength(points_in.front(), pRef, offset);
    Point pRef2 = rotate(points_in.back(), r_at(points_in, 1), -ang);
    setVectorLength(points_in.back(), pRef2, offset);

    int ind = getGeomIndex(poly.points, pRef);
    points_out.insert( points_out.end(), poly.points.begin()+ind, poly.points.end() );
    points_out.insert( points_out.end(), poly.points.begin(), poly.points.begin()+ind );

    ind = 0;
    for( ; ind < points_out.size() ; ++ind){
        if( calc_dist(r_at(points_out, ind), pRef2) < offset*1e-3 )
            break;
    }
    pop_back(points_out, ind);

    unsample_linestring(points_out);

    if(keepSamples){
        for(size_t i = 1 ; i+1 < points_in.size() ; ++i){
            const Point & p = points_in.at(i);
            double res = calc_dist( p, points_in.at(i+1) );
            auto ind_new = addSampleToGeometryClosestToPoint(points_out, p, 1, res * 0.01);
//            if(ind_new >= 0){
//                if( calc_dist (points_out.at(ind_new), points_out.at(ind_new+1) ) < 0.99 * res )
//                    points_out.erase( points_out.begin() + ind_new + 1 );
//            }
        }
    }

    return !points_out.empty();
}

bool offsetLinestring_boost(const std::vector<Point> &points_in,
                            Polygon &boundary_out,
                            double offset1,
                            double offset2,
                            bool endFlat,
                            int points_per_circle)
{
    boundary_out.points.clear();
    if(points_in.empty()){
        std::cout << "[ERROR - offsetLinestring_boost] No points given" << std::endl;
        return false;
    }

    if(offset1 < 1e-9 || offset2 < 1e-9){
        std::cout << "[ERROR - offsetLinestring_boost] Invalid offsets" << std::endl;
        return false;
    }

    typedef double coordinate_type;
    typedef boost::geometry::model::point<coordinate_type, 2, boost::geometry::cs::cartesian> boost_point_t;
    typedef boost::geometry::model::polygon<boost_point_t> boost_polygon_t;
    typedef boost::geometry::model::linestring<boost_point_t> boost_linestring_t;

    boost_linestring_t input;
    boost::geometry::model::multi_polygon<boost_polygon_t> result;

    for (auto && p : points_in) {
        boost::geometry::append(input, boost_point_t(p.x, p.y));
    }

    boost::geometry::strategy::buffer::distance_asymmetric<coordinate_type> distance_strategy(offset1, offset2);
    boost::geometry::strategy::buffer::join_round join_strategy_round(points_per_circle);
    boost::geometry::strategy::buffer::join_miter join_strategy_flat;
    boost::geometry::strategy::buffer::end_round end_strategy_round(points_per_circle);
    boost::geometry::strategy::buffer::end_flat end_strategy_flat;
    boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
    boost::geometry::strategy::buffer::side_straight side_strategy;

    if(endFlat){
        if(points_per_circle >= 0)
            boost::geometry::buffer(input, result, distance_strategy,
                                    side_strategy, join_strategy_round, end_strategy_flat, circle_strategy);
        else
            boost::geometry::buffer(input, result, distance_strategy,
                                    side_strategy, join_strategy_flat, end_strategy_flat, circle_strategy);
    }
    else{
        if(points_per_circle >= 0)
            boost::geometry::buffer(input, result, distance_strategy,
                                    side_strategy, join_strategy_round, end_strategy_round, circle_strategy);
        else
            boost::geometry::buffer(input, result, distance_strategy,
                                    side_strategy, join_strategy_flat, end_strategy_round, circle_strategy);
    }

    if(result.empty())
        return false;
    if(result.size() == 1){
        for (auto && p : boost::geometry::exterior_ring(result.at(0))) {
            boundary_out.points.emplace_back(Point(boost::geometry::get<0>(p),
                                                   boost::geometry::get<1>(p), 0));
        }
    }
    else{
        double maxArea = std::numeric_limits<double>::lowest();
        for(size_t i = 0 ; i < result.size() ; ++i){
            Polygon poly;
            for (auto && p : boost::geometry::exterior_ring(result.at(i))) {
                poly.points.emplace_back(Point(boost::geometry::get<0>(p),
                                               boost::geometry::get<1>(p), 0));
            }
            double area = calc_area(poly);
            if(maxArea < area){
                maxArea = area;
                boundary_out = poly;
            }
        }
    }

    openPolygon(boundary_out);
    size_t indMinDist = getGeomIndex(boundary_out.points, points_in.front());

    boundary_out.points.insert(boundary_out.points.end(), boundary_out.points.begin(), boundary_out.points.begin()+indMinDist);
    boundary_out.points.erase(boundary_out.points.begin(), boundary_out.points.begin()+indMinDist);

    closePolygon(boundary_out);

    if(isPolygonValid(boundary_out) == PolygonValidity::INVALID_POLYGON){
        std::cout << "[ERROR - offsetLinestring_boost] Final polygon is no valid" << std::endl;
        return false;
    }

    return true;

}


bool offsetLinestring_old(const std::vector<Point> &ls_in, std::vector<Point> &ls_out, const double &offset)
{
    ls_out.clear();
    std::vector<Point> points_in = ls_in;
    if(points_in.size() < 2)
        return false;
    if(offset == 0){
        ls_out = ls_in;
        return true;
    }
    if(!unsample_linestring(points_in)){
        std::cout << "[Error : offsetLinestring] Error unsampling linestring" << std::endl;
        return false;
    }
    std::vector<Point> segments;
    std::pair<Point, Point> longestSegment;
    double longestDist = 0;
    for(size_t i = 0 ; i+1 < points_in.size(); ++i){
        Point p0, p1;
        offset_line(points_in.at(i), points_in.at(i+1), p0, p1, offset);
        segments.push_back(p0);
        segments.push_back(p1);
        double dist = calc_dist(p0, p1);
        if( longestDist < dist ){
            longestDist = dist;
            longestSegment.first = p0;
            longestSegment.second = p1;
        }
    }

    if(segments.size() == 2)
        ls_out.push_back(segments.at(0));

    for(size_t i = 0 ; i+3 < segments.size(); i+=2){
        Point intersection;
        if( !get_intersection(segments.at(i),
                              segments.at(i+1),
                              segments.at(i+2),
                              segments.at(i+3),
                              intersection,
                              true,
                              true,
                              0) ){
            std::cout << "[Error : offsetLinestring] Error obtaining the intersections between the offset segments" << std::endl;
            return false;
        }
        if(i == 0){
            Point intersection2;
            if(get_intersection(points_in.at(0),
                                segments.at(i),
                                points_in.at(1),
                                intersection,
                                intersection2,
                                false,
                                false,
                                0.0001)){
                segments.erase(segments.begin(),segments.begin()+2);
                points_in.erase(points_in.begin());
                i-=2;
                if(points_in.size() < 3){
                    std::cout << "[Warning : offsetLinestring] No intersection between the offset segments obtained. Longest segment set as output." << std::endl;
                    ls_out.clear();
                    ls_out.push_back( longestSegment.first );
                    ls_out.push_back( longestSegment.second );
                    return true;
                }
            }
            else{
                ls_out.push_back(segments.at(0));
                ls_out.push_back(intersection);
            }

        }
        else
            ls_out.push_back(intersection);
    }
    ls_out.push_back(segments.back());

    while(ls_out.size() > 2 && points_in.size() > 2){
        Point intersection2;
        if(get_intersection(points_in.at( points_in.size()-1 ),
                            ls_out.at( ls_out.size()-1 ),
                            points_in.at( points_in.size()-2 ),
                            ls_out.at( ls_out.size()-2 ),
                            intersection2,
                            false,
                            false,
                            0.0001)){

            segments.erase(segments.end()-2,segments.end());
            ls_out.pop_back();
            ls_out.back() = segments.back();
            points_in.pop_back();
            if(points_in.size() < 3){
                std::cout << "[Warning : offsetLinestring] No intersection between the offset segments obtained. Longest segment set as output." << std::endl;
                ls_out.clear();
                ls_out.push_back( longestSegment.first );
                ls_out.push_back( longestSegment.second );
                return true;
            }
        }
        else
            break;
    }

    if(ls_out.size() < 2)
        return false;

    //remove ears
    for(size_t i = 0 ; i+1 < ls_out.size() ; ++i){
        for(size_t j = i+2 ; j+1 < ls_out.size() ; ++j){
            Point intersection;
            if(get_intersection( ls_out.at(i),
                                 ls_out.at(i+1),
                                 ls_out.at(j),
                                 ls_out.at(j+1),
                                 intersection,
                                 false,
                                 false,
                                 0.001) ){
                ls_out.erase( ls_out.begin() + i+1, ls_out.begin() + j);
                ls_out.at(i+1) = intersection;
            }
        }
    }

    //    //check if offset was too large
    //    for(size_t i = 0 ; i+1 < points_in.size() ; ++i){
    //        for(auto &p : ls_out){
    //            double dist = calc_dist_to_line(points_in.at(i), points_in.at(i+1), p, false, true );
    //            if( dist < std::fabs(offset*0.999)){
    //                std::cout << "[Error : offsetLinestring] Offset is too large." << std::endl;
    //                return false;
    //            }
    //        }
    //    }
    return ls_out.size() > 1;

}


bool offsetPolygon(const Polygon &_poly_in, Polygon &poly_out, double offset, bool inflated, size_t points_per_circle){
    return offsetPolygon_boost(_poly_in, poly_out, offset, inflated, points_per_circle);
}


bool offsetPolygon_boost(const Polygon &_poly_in, Polygon &poly_out, double offset, bool inflated, size_t points_per_circle)
{

    poly_out.points.clear();
    auto polyValidity = isPolygonValid(_poly_in);
    if(polyValidity == PolygonValidity::INVALID_POLYGON){
        std::cout <<"[ERROR : offsetBoundary_boost] The input polygon is not valid" << std::endl;
        return false;
    }

    bool clockwise = isPolygonClockwise(_poly_in);

    offset = std::fabs(offset);
    if(!inflated)
        offset = -offset;

    Polygon poly_in = _poly_in;
    correct_polygon(poly_in);


    openPolygon(poly_in);
    if(poly_in.points.size() < 3){
        std::cout <<"[ERROR : offsetBoundary_boost] The input polygon must have at least 3 different points" << std::endl;
        return false;
    }

    closePolygon(poly_in);
    std::vector<Linestring> offSegments;
    for(size_t i = 0; i+1 < poly_in.points.size(); ++i){

        if(i == 0 || i+2 == poly_in.points.size()){
            Point p0,p1;
            Linestring ls;
            if(inflated){
                offset_line(poly_in.points.at(i),
                            poly_in.points.at(i+1),
                            p0,
                            p1,
                            -offset);
                ls.points.push_back(p0);
                ls.points.push_back(p1);
            }
            else{
                offset_line(poly_in.points.at(i),
                            poly_in.points.at(i+1),
                            p0,
                            p1,
                            offset);
                ls.points.push_back(p0);
                ls.points.push_back(p1);
            }
            offSegments.push_back(ls);
        }
    }

    Point p0Offset;
    if( !get_intersection( offSegments.at(0).points.at(0),
                           offSegments.at(0).points.back(),
                           offSegments.back().points.at(0),
                           offSegments.back().points.back(),
                           p0Offset,
                           false,
                           false) ){
        p0Offset = poly_in.points.front();
    }

    typedef double coordinate_type;
    typedef boost::geometry::model::point<coordinate_type, 2, boost::geometry::cs::cartesian> boost_point_t;
    typedef boost::geometry::model::polygon<boost_point_t> boost_polygon_t;

    boost_polygon_t input;
    boost::geometry::model::multi_polygon<boost_polygon_t> result;

    for (auto && p : poly_in.points) {
        boost::geometry::append(input, boost_point_t(p.x, p.y));
    }

    // Declare strategies
    const double buffer_distance = offset;

    boost::geometry::strategy::buffer::distance_symmetric<coordinate_type> distance_strategy(buffer_distance);
    boost::geometry::strategy::buffer::side_straight side_strategy;

    if(points_per_circle < 1){
        boost::geometry::strategy::buffer::join_miter join_strategy;
        boost::geometry::strategy::buffer::end_flat end_strategy;
        boost::geometry::strategy::buffer::point_square circle_strategy;
        boost::geometry::buffer(input, result, distance_strategy,
                                side_strategy, join_strategy, end_strategy, circle_strategy);
    }
    else{
        boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
        boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
        boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
        boost::geometry::buffer(input, result, distance_strategy,
                                side_strategy, join_strategy, end_strategy, circle_strategy);
    }

    if(result.empty())
        return false;
    if(result.size() == 1){
        for (auto && p : boost::geometry::exterior_ring(result.at(0))) {
            poly_out.points.emplace_back(Point(boost::geometry::get<0>(p),
                                                   boost::geometry::get<1>(p), 0));
        }
    }
    else{
        double maxArea = std::numeric_limits<double>::lowest();
        for(size_t i = 0 ; i < result.size() ; ++i){
            Polygon poly;
            for (auto && p : boost::geometry::exterior_ring(result.at(i))) {
                poly.points.emplace_back(Point(boost::geometry::get<0>(p),
                                               boost::geometry::get<1>(p), 0));
            }
            double area = calc_area(poly);
            if(maxArea < area){
                maxArea = area;
                poly_out = poly;
            }
        }
    }

    openPolygon(poly_out);

    double minDist = calc_dist(p0Offset, poly_out.points.front());
    size_t indMinDist = 0;
    for(size_t i = 1 ; i < poly_out.points.size() ; ++i){
        double dist = calc_dist(p0Offset, poly_out.points.at(i));
        if(minDist > dist){
            minDist = dist;
            indMinDist = i;
        }
    }

    poly_out.points.insert(poly_out.points.end(), poly_out.points.begin(), poly_out.points.begin()+indMinDist);
    poly_out.points.erase(poly_out.points.begin(), poly_out.points.begin()+indMinDist);

    closePolygon(poly_out);

    if(!clockwise)
        std::reverse( poly_out.points.begin() , poly_out.points.end() );

    if(isPolygonValid(poly_out) == PolygonValidity::INVALID_POLYGON){
        std::cout << "[ERROR - offsetBoundary_boost] Final polygon is no valid" << std::endl;
        return false;
    }

    return true;
}

bool offsetPolygon_clipper(const Polygon &_poly_in, Polygon &poly_out, double offset, bool inflated)
{
    poly_out.points.clear();
    auto polyValidity = isPolygonValid(_poly_in);
    if(polyValidity == PolygonValidity::INVALID_POLYGON){
        std::cout <<"[ERROR : offsetBoundary_clipper] The input boundary is not valid" << std::endl;
        return false;
    }

    bool clockwise = isPolygonClockwise(_poly_in);

    const double mult = 100000;
    const double multInv = 1/mult;
    offset = std::fabs(offset*mult);
    if(!inflated)
        offset = -offset;

    Polygon poly_in = _poly_in;
    correct_polygon(poly_in);


    openPolygon(poly_in);
    if(poly_in.points.size() < 3){
        std::cout <<"[ERROR : offsetBoundary_clipper] The input boundary must have at least 3 different points" << std::endl;
        return false;
    }
    closePolygon(poly_in);

    std::vector<Linestring> offSegments;
    for(size_t i = 0; i+1 < poly_in.points.size(); ++i){

        if(i == 0 || i+2 == poly_in.points.size()){
            Point p0,p1;
            Linestring ls;
            if(inflated){
                offset_line(poly_in.points.at(i),
                            poly_in.points.at(i+1),
                            p0,
                            p1,
                            -offset);
                ls.points.push_back(p0);
                ls.points.push_back(p1);
            }
            else{
                offset_line(poly_in.points.at(i),
                            poly_in.points.at(i+1),
                            p0,
                            p1,
                            offset);
                ls.points.push_back(p0);
                ls.points.push_back(p1);
            }
            offSegments.push_back(ls);
        }
    }

    Point p0Offset;
    if( !get_intersection( offSegments.at(0).points.at(0),
                           offSegments.at(0).points.back(),
                           offSegments.back().points.at(0),
                           offSegments.back().points.back(),
                           p0Offset,
                           false,
                           false) ){
        p0Offset = poly_in.points.front();
    }

    ClipperLib::Path pathIn;
    ClipperLib::Paths pathsOut;
    for(auto &p : poly_in.points)
        pathIn << ClipperLib::IntPoint(p.x * mult, p.y * mult);

    ClipperLib::ClipperOffset offsetter;
    offsetter.AddPath(pathIn, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
    offsetter.Execute(pathsOut,offset);

    if(pathsOut.empty())
        return false;

    if(pathsOut.size()==1){
        for(auto &p : pathsOut.front())
            poly_out.points.emplace_back( Point(p.X*multInv, p.Y*multInv) );
    }
    else{
        double maxArea = std::numeric_limits<double>::lowest();
        for(auto &path : pathsOut){
            Polygon poly;
            for(auto &p : path)
                poly.points.push_back( Point(p.X*multInv, p.Y*multInv) );
            double area = calc_area(poly);
            if(maxArea < area){
                maxArea = area;
                poly_out = poly;
            }
        }
    }


    if(poly_out.points.empty())
        return false;

    openPolygon(poly_out);

    double minDist = calc_dist(p0Offset, poly_out.points.front());
    size_t indMinDist = 0;
    for(size_t i = 1 ; i < poly_out.points.size() ; ++i){
        double dist = calc_dist(p0Offset, poly_out.points.at(i));
        if(minDist > dist){
            minDist = dist;
            indMinDist = i;
        }
    }

    poly_out.points.insert(poly_out.points.end(), poly_out.points.begin(), poly_out.points.begin()+indMinDist);
    poly_out.points.erase(poly_out.points.begin(), poly_out.points.begin()+indMinDist);

    closePolygon(poly_out);

    if(clockwise && !isPolygonClockwise(poly_out))
        std::reverse(poly_out.points.begin(), poly_out.points.end());

    if(isPolygonValid(poly_out) == PolygonValidity::INVALID_POLYGON){
        std::cout << "[ERROR - offsetBoundary_clipper] Final polygon is no valid" << std::endl;
        return false;
    }

    return true;

}

int addSampleToGeometryClosestToPoint(std::vector<Point>& geom, const Point& p, size_t maxPoints, double minDist){
    if(geom.empty())
        return -1;
    if(geom.size() == 1)
        return 0;

    const double eps = 1e-3;

    minDist = std::fabs(minDist);
    if(maxPoints == 0)
        maxPoints = std::numeric_limits<size_t>::max();

    std::set<size_t> indexes;
    double d_min = std::numeric_limits<double>::max();
    for(size_t i = 0 ; i+1 < geom.size() ; ++i){
        double d = calc_dist_to_line( geom.at(i), geom.at(i+1), p, false );
        if(d_min > d-eps){
            if(d_min - d > eps)
                indexes = {i};
            else if ( indexes.size()+1 <= maxPoints )
                indexes.insert(i);
            d_min = std::min(d_min, d);
        }
    }

    size_t count = 0;
    for(auto& i : indexes){
        Point sample = p;
        move_point_to_dist_to_line( geom.at(i+count), geom.at(i+1+count), sample, 0, true );
        if( calc_dist(geom.at(i+1+count), sample) > minDist
                && calc_dist(geom.at(i+count), sample) > minDist ){
            if(maxPoints == 1){
                geom.insert( geom.begin()+i+1, sample );
                return i+1;
            }
            else{
                ++count;
                geom.insert( geom.begin()+i+count, sample );
            }
        }
        else if(maxPoints == 1)
            return calc_dist(geom.at(i+1), sample) > calc_dist(geom.at(i), sample) ? i : i+1;
    }

    return count;
}

bool removeSpikes(Polygon &poly)
{
    PolygonValidity polyValidity;
    bool clockwise, closed;
    polyValidity = isPolygonValid(poly);

    if(polyValidity==PolygonValidity::INVALID_POLYGON)
        return false;

    clockwise = isPolygonClockwise(polyValidity);
    closed = isPolygonClosed(polyValidity);

    boost::geometry::correct(poly.points);
    boost::geometry::remove_spikes(poly.points);

    if(!clockwise)
        std::reverse(poly.points.begin(), poly.points.end());
    if(!closed)
        poly.points.pop_back();

    return true;
}

std::vector<Polygon> extractSubPolygons(const Polygon &_poly_in)
{
    Polygon poly_in = _poly_in;
    std::vector<Polygon> polys_out;
    PolygonValidity polyValidity;
    bool reorderNeeded;

    if(!openPolygon(poly_in))
        return polys_out;
    if (poly_in.points.size() < 3)
        return polys_out;

    polyValidity = isPolygonValid(poly_in);
    if (polyValidity==PolygonValidity::INVALID_POLYGON)
        return polys_out;

    if( poly_in.points.size() == 3 ){
        closePolygon(poly_in);
        polys_out.push_back(poly_in);
        return polys_out;
    }

    for(size_t i = 0; i < poly_in.points.size(); ++i){
        size_t index0 = i;
        size_t index1 = (i+1) % poly_in.points.size();
        Polygon ear;
        std::pair<size_t,size_t> earIndexes;
        ear.points.push_back( poly_in.points.at(index1) );
        for(size_t j = 2; j+1 < poly_in.points.size(); ++j){
            size_t index2 = (i+j) % poly_in.points.size();
            size_t index3 = (i+j+1) % poly_in.points.size();
            Point intersection;
            ear.points.push_back( poly_in.points.at(index2) );
            if(j==2)
                earIndexes.first=index2;
            earIndexes.second=index2;
            if( get_intersection( poly_in.points.at(index0),
                                  poly_in.points.at(index1),
                                  poly_in.points.at(index2),
                                  poly_in.points.at(index3),
                                  intersection,
                                  false,
                                  false) ){
                ear.points.push_back(intersection);
                ear.points.push_back( ear.points.at(0) );

                PolygonValidity earValidity = isPolygonValid(ear);
                if( earValidity == PolygonValidity::INVALID_POLYGON )//there exists an ear but in the other direction of the check. it will be removed in another iteration
                    break;

                poly_in.points.at(index1) = intersection;
                if(earIndexes.first > earIndexes.second){
                    poly_in.points.erase( poly_in.points.begin() + earIndexes.first, poly_in.points.end() );
                    poly_in.points.erase( poly_in.points.begin() , poly_in.points.begin() + earIndexes.second + 1 );
                    reorderNeeded = true;
                    i -= (earIndexes.second+1);
                }else{
                    poly_in.points.erase( poly_in.points.begin() + earIndexes.first, poly_in.points.begin() + earIndexes.second + 1 );
                    if(earIndexes.first == 0)
                       reorderNeeded = true;
                    if(earIndexes.second < index0)
                        i -= (earIndexes.second-earIndexes.first+1);
                }
                i--;

//                  std::cout << std::setprecision(12) << "boundary out tmp - ear removed" << std::endl;
//                  for(int i = 0 ; i < poly.points.size() ; ++i)
//                      std::cout << poly.points.at(i).x << ";"  << poly.points.at(i).y << std::endl;
//                  std::cout << "- ear removed" << std::endl;

                break;
            }
        }
        if(poly_in.points.size() < 4)
            break;
    }
    return polys_out;

}


//----------------------------------------------------------------------------

bool checkLinesParallel(const Point &_p0_1,
                        const Point &_p1_1,
                        const Point &_p0_2,
                        const Point &_p1_2,
                        double tolerance)
{
    //translate to improve float-precision during computation, in case the values are too high, but yet close
    Point p0_1 = Point(0,0);
    Point p1_1 = _p1_1 - _p0_1;
    Point p0_2 = _p0_2 - _p0_1;
    Point p1_2 = _p1_2 - _p0_1;
    double tol = fabs(tolerance);
    if (p0_1 == p1_1 || p0_2 == p1_2)
        return false;
    if ( p0_1.x == p1_1.x )
        return ( fabs(p0_2.x-p1_2.x) <= tol );
    double m1 = ( p1_1.y - p0_1.y )/( p1_1.x - p0_1.x );
    double m2 = ( p1_2.y - p0_2.y )/( p1_2.x - p0_2.x );
    return ( fabs(m1-m2) <= tol );
}


//----------------------------------------------------------------------------


bool in_box(const Point &point, double minX, double maxX, double minY, double maxY)
{
    return point.x >= minX && point.x <= maxX
            && point.y >= minY && point.y <= maxY;
}

bool in_polygon(const Point &point, const Polygon &poly)
{
    //boost::geometry::correct(poly.points);
    return boost::geometry::within(point, poly.points);
}

bool in_polygon(const Point &point, const std::vector<Point>& ring)
{
    //boost::geometry::correct(ring);
    return boost::geometry::within(point, ring);
}

bool in_polygon(const std::vector<Point> &points, const Polygon &poly, bool all){
    //boost::geometry::correct(poly.points);
    for(size_t i = 0; i < points.size(); i++){
        bool inside = boost::geometry::within(points[i], poly.points);
        if(all && !inside)
            return false;
        else if(!all && inside)
            return true;
    }
    if(all)
        return true;
    return false;
}

bool in_polygon(const Linestring &ls, const Polygon &poly, bool all){
    return in_polygon(ls.points, poly, all);
}

bool in_polygon(const Polygon &poly1, const Polygon &poly2){
    return in_polygon(poly1.points, poly2);
}

bool checkPointInLine(const Point &p0,
                      const Point &p1,
                      const Point &p,
                      bool infinite_line,
                      double tolerance)
{
    double dist;
    double tol = fabs(tolerance);

    //translate to improve float-precision during computation, in case the values are too high, but yet close
    Point p0_t = Point(0,0);
    Point p1_t = p1 - p0;
    Point p_t = p - p0;

    if (p0_t == p1_t)
        return (p_t == p0_t);

    double yMin = std::min(p0_t.y, p1_t.y);
    double yMax = std::max(p0_t.y, p1_t.y);
    double xMin = std::min(p0_t.x, p1_t.x);
    double xMax = std::max(p0_t.x, p1_t.x);

    //if line is vertical
    if (p0_t.x == p1_t.x){
        if( fabs(p_t.x-p0_t.x) > tol )
            return false;
        if (infinite_line)
            return true;
        return ( p_t.y >= yMin-tol && p_t.y <= yMax+tol );
    }

    dist = calc_dist_to_line(p0_t, p1_t, p_t);
    if(dist > tolerance)
        return false;
    if (infinite_line)
        return true;
    return ( p_t.x >= xMin-tol && p_t.x <= xMax+tol );

}

bool checkPointInLine_2(const Point &p0,
                        const Point &p1,
                        const Point &p,
                        bool infinite_line,
                        double tolerance)
{
    double y;
    double tol = fabs(tolerance);

    //translate to improve float-precision during computation, in case the values are too high, but yet close
    Point p0_t = Point(0,0);
    Point p1_t = p1 - p0;
    Point p_t = p - p0;

    if (p0_t == p1_t)
        return (p_t == p0_t);

    double yMin = std::min(p0_t.y, p1_t.y);
    double yMax = std::max(p0_t.y, p1_t.y);
    double xMin = std::min(p0_t.x, p1_t.x);
    double xMax = std::max(p0_t.x, p1_t.x);

    //if line is vertical
    if (p0_t.x == p1_t.x){
        if( fabs(p_t.x-p0_t.x) > tol )
            return false;
        if (infinite_line)
            return true;
        return ( p_t.y >= yMin-tol && p_t.y <= yMax+tol );
    }

    y = interp_line(p0_t, p1_t, p_t.x);
    if( fabs(y-p_t.y) > tol )
        return false;
    if (infinite_line)
        return true;
    return ( p_t.x >= xMin-tol && p_t.x <= xMax+tol );

}

bool checkPointInRectangle(const Point &reference_corner, double rect_width, double rect_height, const Point &p)
{
    Point pLL = reference_corner;
    Point pUR;

    if (rect_width == 0 || rect_height == 0)
        return false;

    if (rect_width < 0)
        pLL.x += rect_width;

    if (rect_height < 0)
        pLL.y += rect_height;

    pUR = pLL;
    pUR.x += fabs(rect_width);
    pUR.y += fabs(rect_height);

    return ( p.x >= pLL.x && p.y >= pLL.y && p.x <= pUR.x && p.y <= pUR.y );

}

bool checkLineSegmentInRectangle(const Point &reference_corner, double rect_width, double rect_height, const Point &p0, const Point &p1)
{
    Point pLL = reference_corner;
    Point pUR;

    if ( p0 == p1)
        return checkPointInRectangle(reference_corner, rect_width, rect_height, p0);

    if (rect_width == 0 || rect_height == 0)
        return false;

    if (rect_width < 0)
        pLL.x += rect_width;

    if (rect_height < 0)
        pLL.y += rect_height;

    pUR = pLL;
    pUR.x += fabs(rect_width);
    pUR.y += fabs(rect_height);


    //check if any of the points lies inside the rectangle
    if ( (p0.x > pLL.x) && (p0.x < pUR.x) && (p0.y > pLL.y) && (p0.y < pUR.y) )
        return true;
    if ( (p1.x > pLL.x) && (p1.x < pUR.x) && (p1.y > pLL.y) && (p1.y < pUR.y) )
        return true;

    if ( p0.x == p1.x ){//if line is vertical
        if ( (p0.x > pLL.x) && (p0.x < pUR.x) ){
            return !( ( (p0.y >= pUR.y) && (p1.y >= pUR.y) ) ||
                      ( (p0.y <= pLL.y) && (p1.y <= pLL.y) ) );
        }
        else
            return false;
    }

    if ( p0.y == p1.y ){//if line is horizontal
        if ( (p0.y > pLL.y) && (p0.y < pUR.y) ){
            return !( ( (p0.x >= pUR.x) && (p1.x >= pUR.x) ) ||
                      ( (p0.x <= pLL.x) && (p1.x <= pLL.x) ) );
        }
        else
            return false;
    }

    Polygon rect;
    rect.points= { pLL,
                   pLL + Point(0, rect_height),
                   pUR,
                   pLL+ Point(rect_width, 0),
                   pLL };

    return checkLineSegmentInPolygon(rect, p0, p1);
//    return intersects( p0, p1, rect.points );
}

bool checkLineSegmentInPolygon(const Polygon &poly, const Point &p0, const Point &p1)
{
    std::deque<Point> intersectionPoints;

    if (poly.points.empty())
        return false;

    //boost::geometry::correct(poly.points);

    if ( boost::geometry::within(p0,poly.points) )
        return true;
    if ( boost::geometry::within(p1,poly.points) )
        return true;


    boost::geometry::model::linestring<Point> ls;
    boost::geometry::append(ls, p0);
    boost::geometry::append(ls, p1);

    boost::geometry::intersection(poly.points, ls, intersectionPoints);

    return (intersectionPoints.size() > 1); //if size == 1 -> the line is only in contact with the border
}


//----------------------------------------------------------------------------


std::vector<Point> sample_line(const Point &p0, const Point &p1, size_t numSamples)
{
    if(p0 == p1)
        return {p0};

    if(numSamples < 2)
        return {};

    double dist = calc_dist(p0, p1);
    double delta_dist = dist / (numSamples-1);

    std::vector<Point> ret = {p0};
    Point vec;
    getParallelVector(p0, p1, vec, delta_dist);
    for(size_t i = 1 ; i+1 < numSamples ; ++i)
        ret.emplace_back( ret.back() + vec );
    ret.emplace_back( p1 );
    return ret;
}

bool unsample_polygon(Polygon &poly, double tolerance, bool keepFirstPoint)
{
    if (poly.points.empty())
        return false;
    if (poly.points.size() < 3)
        return true;

    tolerance = std::fabs(tolerance);
    std::vector<Point> points;
    if (is_line(poly.points,tolerance)){
        size_t indMed = poly.points.size() / 2;
        points.push_back(poly.points.at(0));
        points.push_back(poly.points.at(indMed));
        points.push_back(poly.points.back());
        poly.points = points;
        return true;
    }

    openPolygon(poly);

    points = poly.points;

    for ( size_t i = 1 ; i <= points.size() ; ++i ){
        size_t ind = getIndexFromContinuousGeometry( points, i);
        size_t ind_next;
        std::vector<Point> vec = { points.at(i-1) , points.at(ind), getPointFromContinuousGeometry(points, i+1, &ind_next) };
        while(vec.size() <= points.size() && is_line( vec, tolerance ))
            vec.emplace_back( getPointFromContinuousGeometry(points, ++ind_next) );

        if(vec.size() == 3)
            continue;

        while( !is_line( std::vector<Point>(vec.rbegin(), vec.rbegin()+3), tolerance ) )
            vec.pop_back();

        double minDist = std::numeric_limits<double>::max();
        for(size_t j = 1 ; j+1 < vec.size() ; ++j){
            double dist = calc_dist( vec.at(j-1), vec.at(j) ) + calc_dist( vec.at(j), vec.at(j+1) );
            if(dist < minDist){
                ind_next = getIndexFromContinuousGeometry( points, ind + j - 1 );
//                  if(ind_next == 0 && keepFirstPoint)
//                      continue;
                minDist = dist;
            }
        }

        if(minDist > std::numeric_limits<double>::max() - 10)
            continue;

        points.erase( points.begin() + ind_next );
        --i;
    }
    if (points.empty())
        return false;

    Point p0 = poly.points.at(0);
    poly.points = points;
//      if ( points.front() != p0 ){
//          if( calc_dist(p0, points.front()) > calc_dist(p0, points.back()) )
//              poly.points.insert( poly.points.begin(), points.back() );
//      }

    closePolygon(poly);
    return true;
}

bool unsample_polygon__old(Polygon &poly, double tolerance, bool keepFirstPoint)
{
    if (poly.points.empty())
        return false;
    if (poly.points.size() < 3)
        return true;

    tolerance = std::fabs(tolerance);
    std::vector<Point> points;
    if (is_line(poly.points,tolerance)){
        size_t indMed = poly.points.size() / 2;
        points.push_back(poly.points.at(0));
        points.push_back(poly.points.at(indMed));
        points.push_back(poly.points.back());
        poly.points = points;
        return true;
    }
    closePolygon(poly);

    points = poly.points;

    if(!keepFirstPoint)
      points.push_back( points.at(1) );

    for ( size_t i = 1 ; i+1 < points.size() ; ++i ){
        std::vector<Point> vec = { points.at(i-1) , points.at(i) , points.at(i+1) };
        if (is_line( vec, tolerance )){
            if(i == 1)
               points.back() = points.at(i+1);
            points.erase( points.begin() + i );
            i--;
        }
        else if( i == points.size()-2 && !keepFirstPoint )
            points.pop_back();
    }
    if (points.empty())
        return false;
    Point p0 = poly.points.at(0);
    poly.points.clear();
    if ( points.back() == p0 )
        poly.points.push_back(p0);
    poly.points.insert( poly.points.end(), points.begin()+1, points.end() );

    closePolygon(poly);
    return true;
}


//----------------------------------------------------------------------------

Polygon createRectangleFromLine(const Point &p1, const Point &p2, const double width, const bool clockwise)
{
    Polygon poly;
    Point p1_plus, p1_minus, p2_plus, p2_minus;

    if (width == 0 || p1==p2)
        return poly;

    offset_line(p1, p2, p1_plus, p2_plus, width/2.0);
    offset_line(p1, p2, p1_minus, p2_minus, -width/2.0);

    poly.points.push_back( p1_plus );
    poly.points.push_back( p2_plus );
    poly.points.push_back( p2_minus );
    poly.points.push_back( p1_minus );
    poly.points.push_back( p1_plus );

    correct_polygon(poly, clockwise);
    return poly;
}


Polygon createPolygon(const Point &p1, const Point &p2, double width, bool clockwise)
{
    return createRectangleFromLine(p1, p2, width, clockwise);
}

Polygon create_circle(const Point &centroid,
                      double radius,
                      size_t samples,
                      double startAngle,
                      double angleRange,
                      bool inDegrees,
                      bool alwaysClockwise)
{
    Polygon circle;
    if(samples == 0)
        return circle;

    radius = std::fabs(radius);

    if(inDegrees)
        angleRange = deg2rad(angleRange);

    if(angleRange > 2*M_PI)
        angleRange = 2*M_PI;
    else if(angleRange < -2*M_PI)
        angleRange = -2*M_PI;

    if(angleRange != 2*M_PI && angleRange != -2*M_PI)
        circle.points.push_back( centroid );

    double delta_angle = angleRange / samples;
    for(unsigned int i = 0 ; i <= samples ; ++i){
        double x = centroid.x + std::cos(startAngle+ delta_angle*i) * radius;
        double y = centroid.y + std::sin(startAngle+ delta_angle*i) * radius;
        circle.points.push_back( Point( x , y ) );
        //circle.points.push_back( create_line(centroid, _radius, delta_angle*i, false) );
    }
    closePolygon(circle);
    if(alwaysClockwise && !isPolygonClockwise(circle))
        std::reverse( circle.points.begin() , circle.points.end() );
    return circle;
}

Point create_line(const Point &p0, double length, double angle, bool inDegrees)
{
    if(length == 0)
        return p0;
    Point p1;
    if(inDegrees)
        angle = deg2rad(angle);

    angle = std::fmod(angle, 2 * M_PI);
    if(angle < 0)
        angle += (2 * M_PI);

    if(angle == 0 || angle == 2 * M_PI){
        p1.y = p0.y;
        p1.x = p0.x + length;
        return p1;
    }
    if(angle == M_PI){
        p1.y = p0.y;
        p1.x = p0.x - length;
        return p1;
    }
    if(angle == M_PI_2){
        p1.x = p0.x;
        p1.y = p0.y + length;
        return p1;
    }
    if(angle == -M_PI_2){
        p1.x = p0.x;
        p1.y = p0.y - length;
        return p1;
    }

    p1.x = p0.x + length / std::cos(angle);
    p1.y = p0.y + length / std::sin(angle);
    return p1;

}

Linestring toLinestring(const Polygon &poly) {
  Linestring ls;
  ls.points.insert(ls.points.begin(),poly.points.begin(), poly.points.end());
  return ls;
}

//----------------------------------------------------------------------------

Point getPointInLineAtDist(const Point& p0, const Point &p1, double dist){
    if(p0 == p1)
        return p0;
    Point v = p1 - p0;
    setVectorLength(v, dist);
    return p0 + v;
}

int getLocationInLine(const Point &p0, const Point &p1, const Point &p, double tolerance)
{
    double length = calc_dist(p0,p1);
    double d0 = calc_dist(p0,p);
    double d1 = calc_dist(p1,p);
    if( d0 <= length+tolerance && d1 <= length+tolerance )
        return 0;
    if( d0 < d1 )
        return -1;
    return 1;
}

//----------------------------------------------------------------------------


bool intersects(const Point& p0, const Point& p1, const Polygon& poly, bool p0ToInfinity, bool p1ToInfinity){
    if(!p0ToInfinity && !p1ToInfinity)
        return !get_intersection(p0, p1, poly).empty();
    return intersects(p0, p1, poly.points, p0ToInfinity, p1ToInfinity);
}

bool intersects(const Polygon &poly1, const Polygon &poly2)
{
//    boost::geometry::correct(poly1.points);
//    boost::geometry::correct(poly2.points);
    return boost::geometry::intersects(poly1.points, poly2.points);

}

bool intersects(const Polygon &poly){
    //boost::geometry::correct(ring);
    return boost::geometry::intersects(poly.points);
}

size_t count_intersections(const Point& p0, const Point& p1, const Polygon& poly, bool p0ToInfinity, bool p1ToInfinity, double tolerance){
    if(!isPolygonClosed(poly))
        return get_intersection( p0,
                             p1,
                             poly.points,
                             true,
                             p0ToInfinity,
                             p1ToInfinity,
                             tolerance ).size();

    Polygon polyEd = poly;
    openPolygon(polyEd);
    return get_intersection( p0,
                             p1,
                             polyEd.points,
                             true,
                             p0ToInfinity,
                             p1ToInfinity,
                             tolerance ).size();
}

bool get_intersection(const Point &p0_1,
                      const Point &p1_1,
                      const Point &p0_2,
                      const Point &p1_2,
                      Point &p_int,
                      bool l1_infinite,
                      bool l2_infinite,
                      double tolerance)
{
    Linestring ls_int = get_intersection(p0_1, p1_1, p0_2, p1_2, l1_infinite, l2_infinite, tolerance);

    if ( ls_int.points.size() != 1 )
        return false;

    p_int = ls_int.points.at(0);
    return true;
}


Linestring get_intersection(const Point &p0_1,
                            const Point &p1_1,
                            const Point &p0_2,
                            const Point &p1_2,
                            bool l1_infinite,
                            bool l2_infinite,
                            double tolerance)
{
    Point p_int;
    Linestring ls_int;
    ls_int.points.clear();

    //if the points of a line are equal (i.e. not really a line)
    if ( p0_1 == p1_1){
        if ( !checkPointInLine(p0_2, p1_2, p0_1, l2_infinite, tolerance) )
            return ls_int;
        ls_int.points.push_back(p0_1);
        return ls_int;
    }
    if ( p0_2 == p1_2){
        if ( !checkPointInLine(p0_1, p1_1, p0_2, l1_infinite, tolerance) )
            return ls_int;
        ls_int.points.push_back(p0_2);
        return ls_int;
    }

    //if lines are parallel
    if ( checkLinesParallel(p0_1, p1_1, p0_2, p1_2, tolerance) ){
        //if both of theme are infinite, there is no intersection (the intersection is either non-existent or infinite)
        if (l1_infinite && l2_infinite)
            return ls_int;

        //if one of them is infinite, the returned intersection (if there is) will be the non-infinite line
        if (l1_infinite){
            if ( checkPointInLine(p0_1, p1_1, p0_2, l1_infinite, tolerance) ){
                ls_int.points.push_back(p0_2);
                ls_int.points.push_back(p1_2);
            }
            return ls_int;
        }
        if (l2_infinite){
            if ( checkPointInLine(p0_2, p1_2, p0_1, l2_infinite, tolerance) ){
                ls_int.points.push_back(p0_1);
                ls_int.points.push_back(p1_1);
            }
            return ls_int;
        }


        //if neither of them is infinite, the returned linestring will be the intersection-line (or single point) between the two lines
        if ( checkPointInLine(p0_2, p1_2, p0_1, l2_infinite, tolerance) )
            ls_int.points.push_back(p0_1);
        if ( checkPointInLine(p0_2, p1_2, p1_1, l2_infinite, tolerance) )
            ls_int.points.push_back(p1_1);
        if (ls_int.points.size() > 1)
            return ls_int;
        if ( checkPointInLine(p0_1, p1_1, p0_2, l1_infinite, tolerance) && p0_2 != ls_int.points.at(0) )
            ls_int.points.push_back(p0_2);
        if (ls_int.points.size() > 1)
            return ls_int;
        if ( checkPointInLine(p0_1, p1_1, p1_2, l1_infinite, tolerance) && p1_2 != ls_int.points.at(0) )
            ls_int.points.push_back(p1_2);
        return ls_int;
    }

//      double den = ( (p0_1.x - p1_1.x)*(p0_2.y - p1_2.y) - (p0_1.y - p1_1.y)*(p0_2.x - p1_2.x) );
//      p_int.x = ( (p0_1.x * p1_1.y - p0_1.y * p1_1.x) * (p0_2.x - p1_2.x) - (p0_1.x - p1_1.x) * (p0_2.x * p1_2.y - p0_2.y * p1_2.x) ) / den;
//      p_int.y = ( (p0_1.x * p1_1.y - p0_1.y * p1_1.x) * (p0_2.y  -p1_2.y) - (p0_1.y - p1_1.y) * (p0_2.x * p1_2.y - p0_2.y * p1_2.x) ) / den;

    //translate to improve float-precision during computation, in case the values are too high, but yet close
    Point p0_1_t = Point(0,0);
    Point p1_1_t = p1_1 - p0_1;
    Point p0_2_t = p0_2 - p0_1;
    Point p1_2_t = p1_2 - p0_1;
    double den = ( (p0_1_t.x - p1_1_t.x)*(p0_2_t.y - p1_2_t.y) - (p0_1_t.y - p1_1_t.y)*(p0_2_t.x - p1_2_t.x) );
    p_int.x = ( (p0_1_t.x * p1_1_t.y - p0_1_t.y * p1_1_t.x) * (p0_2_t.x - p1_2_t.x) - (p0_1_t.x - p1_1_t.x) * (p0_2_t.x * p1_2_t.y - p0_2_t.y * p1_2_t.x) ) / den;
    p_int.y = ( (p0_1_t.x * p1_1_t.y - p0_1_t.y * p1_1_t.x) * (p0_2_t.y  -p1_2_t.y) - (p0_1_t.y - p1_1_t.y) * (p0_2_t.x * p1_2_t.y - p0_2_t.y * p1_2_t.x) ) / den;
    p_int += p0_1;

    if (l1_infinite && l2_infinite){
        ls_int.points.push_back(p_int);
        return ls_int;
    }

//      bool int_in_l1 = checkPointInLine(p0_1, p1_1, p_int, false, tolerance);
//      bool int_in_l2 = checkPointInLine(p0_2, p1_2, p_int, false, tolerance);
    bool int_in_l1 = (std::min(p0_1.x,p1_1.x) - tolerance) <= p_int.x &&
                     (std::max(p0_1.x,p1_1.x) + tolerance) >= p_int.x &&
                     (std::min(p0_1.y,p1_1.y) - tolerance) <= p_int.y &&
                     (std::max(p0_1.y,p1_1.y) + tolerance) >= p_int.y;
    bool int_in_l2 = (std::min(p0_2.x,p1_2.x) - tolerance) <= p_int.x &&
                     (std::max(p0_2.x,p1_2.x) + tolerance) >= p_int.x &&
                     (std::min(p0_2.y,p1_2.y) - tolerance) <= p_int.y &&
                     (std::max(p0_2.y,p1_2.y) + tolerance) >= p_int.y;

    if (l1_infinite){
        if(int_in_l2){
            ls_int.points.push_back(p_int);
            return ls_int;
        }
    }
    if (l2_infinite){
        if(int_in_l1){
            ls_int.points.push_back(p_int);
            return ls_int;
        }
    }
    if(int_in_l1 && int_in_l2){
        ls_int.points.push_back(p_int);
        return ls_int;
    }
    return ls_int;
}

std::vector<Point> get_intersection(const Point &p0, const Point &p1, const Polygon &poly)
{
    using boost_linestring_t = boost::geometry::model::linestring<Point>;

    std::vector<Point> ret;
    std::vector< boost_linestring_t > intersection;
    if (poly.points.empty())
        return ret;

    //boost::geometry::correct(poly.points);

    boost_linestring_t ls;
    boost::geometry::append(ls, p0);
    boost::geometry::append(ls, p1);

    boost::geometry::intersection(ls,poly.points,intersection);
    for (unsigned int i = 0 ; i < intersection.size() ; ++i)
        ret.insert( ret.end(), intersection.at(i).begin(), intersection.at(i).end() );

    return ret;
}

std::vector<Point> get_intersection(const std::deque<Point>& ls, const Polygon& poly){
    std::vector<Point> ret;
    boost::geometry::intersection(ls, poly.points, ret);
    return ret;
}

std::vector<Point> get_intersection(const std::deque<Point>& ls1, const std::deque<Point>& ls2){
    std::vector<Point> ret;
    boost::geometry::intersection(ls1, ls2, ret);
    return ret;
}

std::vector<Polygon> get_intersection(const Polygon &poly1, const Polygon &poly2)
{
    std::vector< std::vector<Point> > intersection;
    std::vector<Polygon> ret;

    if (poly1.points.empty() || poly2.points.empty())
        return ret;

//    boost::geometry::correct(poly1.points);
//    boost::geometry::correct(poly2.points);
    boost::geometry::intersection(poly1.points,poly2.points,intersection);

    for (unsigned int i = 0 ; i < intersection.size() ; ++i){
        ret.push_back(Polygon());
        ret.back().points =intersection.at(i);
    }
    return ret;
}

std::vector<Polygon> get_likely_intersection(const Polygon &poly1, const Polygon &poly2, double offset){
    auto intersections = get_intersection(poly1, poly2);
    if(intersections.empty()){//sometimes boost::geometry::intersection return no intersection erroneously
        Polygon poly2_2;
        offsetPolygon_boost(poly2, poly2_2, offset, false);
        intersections = get_intersection(poly1, poly2_2);
    }
    return intersections;
}

bool getLineSegmentInPolygon(const Polygon &poly, const Point &p0_in, const Point &p1_in, Point &p0_out, Point &p1_out)
{
    std::deque<Point> intersectionPoints;
    bool p0_inside, p1_inside;

    if (poly.points.empty())
        return false;

    //boost::geometry::correct(poly.points);

    p0_inside = boost::geometry::within(p0_in,poly.points);
    p1_inside = boost::geometry::within(p1_in,poly.points);

    if (p0_inside && p1_inside){
        p0_out = p0_in;
        p1_out = p1_in;
        return true;
    }


    boost::geometry::model::linestring<Point> ls;
    boost::geometry::append(ls, p0_in);
    boost::geometry::append(ls, p1_in);

    boost::geometry::intersection(poly.points, ls, intersectionPoints);

    if (intersectionPoints.size() == 0)
        return false;

    double max_dist = std::numeric_limits<double>::lowest();
    double dist;
    if (p0_inside){
        p0_out = p0_in;
        for (size_t i = 0 ; i < intersectionPoints.size() ; i++){
            dist = calc_dist( p0_in, intersectionPoints.at(i) );
            if (dist > max_dist){
                max_dist = dist;
                p1_out = intersectionPoints.at(i);
            }
        }
        return true;
    }
    if (p1_inside){
        p1_out = p1_in;
        for (size_t i = 0 ; i < intersectionPoints.size() ; i++){
            dist = calc_dist( p1_in, intersectionPoints.at(i) );
            if (dist > max_dist){
                max_dist = dist;
                p0_out = intersectionPoints.at(i);
            }
        }
        return true;
    }

    if (intersectionPoints.size() < 2)
        return false;

    for (size_t i = 0 ; i+1 < intersectionPoints.size() ; i++){
        for (size_t j = i+1 ; j < intersectionPoints.size() ; j++){
            dist = calc_dist( intersectionPoints.at(i), intersectionPoints.at(j) );
            if (dist > max_dist){
                p0_out = intersectionPoints.at(i);
                p1_out = intersectionPoints.at(j);
            }
        }
    }

    return true;

}

bool getLineSegmentInRectangle(const Point& reference_corner,
                               double rect_width,
                               double rect_height,
                               const Point& p0_in,
                               const Point& p1_in,
                               Point& p0_out,
                               Point& p1_out,
                               bool infinite_line)
{
    Point p0 = p0_in, p1 = p1_in;

    Point pLL = reference_corner;
    Point pUR;

    std::vector<Point> rect_points, int_points;

    if (rect_width == 0 || rect_height == 0)
        return false;

    if (p0_in == p1_in){
        p0_out = p0_in;
        p1_out = p1_in;
        return checkPointInRectangle(reference_corner, rect_width, rect_height, p0_in);
    }

    //if the line points lie inside the rectangle
    if ( !infinite_line &&
         checkPointInRectangle(reference_corner, rect_width, rect_height, p0_in) &&
         checkPointInRectangle(reference_corner, rect_width, rect_height, p1_in) ){
        p0_out = p0_in;
        p1_out = p1_in;
    }

    if (rect_width < 0)
        pLL.x += rect_width;

    if (rect_height < 0)
        pLL.y += rect_height;

    pUR = pLL;
    pUR.x += fabs(rect_width);
    pUR.y += fabs(rect_height);

    rect_points.push_back( pLL );
    rect_points.push_back( pLL + Point(0, fabs(rect_height) ) );
    rect_points.push_back( pLL + Point(fabs(rect_width), fabs(rect_height) ) );
    rect_points.push_back( pLL + Point(fabs(rect_width), 0 ) );
    rect_points.push_back( pLL );

    for (size_t i = 0 ; i+1 < rect_points.size() ; ++i){
        Linestring ls = get_intersection(p0_in, p1_in, rect_points.at(i), rect_points.at(i+1), infinite_line, false);
        for (unsigned int j = 0 ; j < ls.points.size() ; ++j){
            if ( ls.points.at(j) != rect_points.at(i+1) )
                int_points.push_back(ls.points.at(j));
        }
    }
    if ( int_points.empty() )
        return false;

    if ( int_points.size() < 2 ){
        if ( checkPointInRectangle(reference_corner, rect_width, rect_height, p0_in) ){
            p0_out = p0_in;
            p1_out = int_points.at(0);
            return true;
        }
        p0_out = int_points.at(0);
        p1_out = p1_in;
        return true;
    }

    //***debug
    if (int_points.size() > 2)
        std::cout << "[ERROR : getLineSegmentInRectangle] More than two intersection points found!" << std::endl;
    //***debug

    //keep the direction of the original line
    if ( p0_in.x == p1_in.x ){
        if (p0_in.y > p1_in.y){
            if (int_points.at(0).y > int_points.at(1).y){
                p0_out = int_points.at(0);
                p1_out = int_points.at(1);
            }
            else{
                p0_out = int_points.at(1);
                p1_out = int_points.at(0);
            }
        }
        else{
            if (int_points.at(0).y > int_points.at(1).y){
                p0_out = int_points.at(1);
                p1_out = int_points.at(0);
            }
            else{
                p0_out = int_points.at(0);
                p1_out = int_points.at(1);
            }
        }
        return true;
    }
    if (p0_in.x > p1_in.x){
        if (int_points.at(0).x > int_points.at(1).x){
            p0_out = int_points.at(0);
            p1_out = int_points.at(1);
        }
        else{
            p0_out = int_points.at(1);
            p1_out = int_points.at(0);
        }
    }
    else{
        if (int_points.at(0).x < int_points.at(1).x){
            p0_out = int_points.at(0);
            p1_out = int_points.at(1);
        }
        else{
            p0_out = int_points.at(1);
            p1_out = int_points.at(0);
        }
    }
    return true;
}


void get_union(const Polygon &poly1, const Polygon &poly2, std::vector<PolygonWithHoles> &polys_out)
{
//      std::deque<Point> ring1(poly1.points.begin(), poly1.points.end());
//      std::deque<Point> ring2(poly2.points.begin(), poly2.points.end());
//      std::vector< std::deque<Point> > unionPolys;
//      std::vector<Polygon> ret;
//      Polygon poly;

//      if (poly1.points.empty() || poly2.points.empty())
//          return ret;

//      boost::geometry::correct(ring1);
//      boost::geometry::correct(ring2);
//      boost::geometry::union_(ring1,ring2,unionPolys);
//      for (unsigned int i = 0 ; i < unionPolys.size() ; ++i){
//          std::vector<Point> points(unionPolys.at(i).begin(), unionPolys.at(i).end());
//          poly.points = points;
//          ret.push_back(poly);
//      }
//      return ret;

    polys_out.clear();

    if (poly1.points.empty() || poly2.points.empty())
        return;

    boost_polygon_t boostPoly1 = toBoostPolygon(poly1);
    boost_polygon_t boostPoly2 = toBoostPolygon(poly2);
    std::vector<boost_polygon_t> unionPolys;

    boost::geometry::correct(boostPoly1);
    boost::geometry::correct(boostPoly2);


    boost::geometry::union_(boostPoly1,boostPoly2,unionPolys);

    polys_out.resize( unionPolys.size() );
    for (unsigned int i = 0 ; i < unionPolys.size() ; ++i){
        polys_out.at(i) = fromBoostPolygon(unionPolys.at(i));
    }

}

void get_union(const Polygon &outer1,
               const std::vector<Polygon> &inners1,
               const Polygon &outer2,
               const std::vector<Polygon> &inners2,
               std::vector<PolygonWithHoles> &polys_out)
{
    polys_out.clear();

    if (outer1.points.empty() || outer2.points.empty())
        return;

    boost_polygon_t boostPoly1 = toBoostPolygon(outer1, inners1);
    boost_polygon_t boostPoly2 = toBoostPolygon(outer2, inners2);
    std::vector<boost_polygon_t> unionPolys;

    boost::geometry::correct(boostPoly1);
    boost::geometry::correct(boostPoly2);


//      std::cout << std::setprecision(12) << "boostPoly1" << std::endl;
//      for(int i = 0 ; i < boostPoly1.outer().size() ; ++i)
//          std::cout << boostPoly1.outer().at(i).x() << ";"  << boostPoly1.outer().at(i).y() << std::endl;
//      std::cout << "-boostPoly1" << std::endl;
//      if(!boostPoly1.inners().empty())
//          std::cout << "boostPoly1 inners not empty" << std::endl;


//      std::cout << std::setprecision(12) << "boostPoly2" << std::endl;
//      for(int i = 0 ; i < boostPoly2.outer().size() ; ++i)
//          std::cout << boostPoly2.outer().at(i).x() << ";"  << boostPoly2.outer().at(i).y() << std::endl;
//      std::cout << "-boostPoly2" << std::endl;
//      if(!boostPoly2.inners().empty())
//          std::cout << "boostPoly2 inners not empty" << std::endl;

    boost::geometry::union_(boostPoly1,boostPoly2,unionPolys);

    polys_out.resize( unionPolys.size() );
    for (unsigned int i = 0 ; i < unionPolys.size() ; ++i){
        polys_out.at(i) = fromBoostPolygon(unionPolys.at(i));
    }

}

void subtract_intersection(const Polygon& poly1, const Polygon& poly2, std::vector<PolygonWithHoles> &polys_out){

    polys_out.clear();

    if (poly1.points.empty() || poly2.points.empty())
        return;

    boost_polygon_t boostPoly1 = toBoostPolygon(poly1);
    boost_polygon_t boostPoly2 = toBoostPolygon(poly2);
    std::vector<boost_polygon_t> diffPolys;

    boost::geometry::correct(boostPoly1);
    boost::geometry::correct(boostPoly2);

    boost::geometry::difference(boostPoly1,boostPoly2,diffPolys);

    polys_out.resize( diffPolys.size() );
    for (unsigned int i = 0 ; i < diffPolys.size() ; ++i){
        polys_out.at(i) = fromBoostPolygon(diffPolys.at(i));
    }

}




//----------------------------------------------------------------------------

bool getPolygonLimits(const Polygon &poly, double &minX, double &maxX, double &minY, double &maxY)
{
    return getGeometryLimits(poly.points, minX, maxX, minY, maxY);
}

bool getPolygonBoundingBox(const Polygon &poly, Polygon &bbox)
{
    return getGeometryBoundingBox(poly.points, bbox);
}


long getLeftmostPointIndInPolygon(const Polygon &poly)
{
    return getLeftmostPointInd(poly.points);
}
long getRightmostPointIndInPolygon(const Polygon &poly)
{
    return getRightmostPointInd(poly.points);
}
long getLowermostPointIndInPolygon(const Polygon &poly)
{
    return getLowermostPointInd(poly.points);
}
long getUppermostPointIndInPolygon(const Polygon &poly)
{
    return getUppermostPointInd(poly.points);
}

Point getLeftmostPointInPolygon(const Polygon &poly)
{
    return getLeftmostPoint(poly.points);
}
Point getRightmostPointInPolygon(const Polygon &poly)
{
    return getRightmostPoint(poly.points);
}
Point getLowermostPointInPolygon(const Polygon &poly)
{
    return getLowermostPoint(poly.points);
}
Point getUppermostPointInPolygon(const Polygon &poly)
{
    return getUppermostPoint(poly.points);
}


//----------------------------------------------------------------------------------------------------------------------------------



}

}
