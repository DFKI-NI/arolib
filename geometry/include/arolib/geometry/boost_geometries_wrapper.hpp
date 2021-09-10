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
 
#ifndef AROLIB_BOOST_GEOMETRIES_WRAPPER_HPP
#define AROLIB_BOOST_GEOMETRIES_WRAPPER_HPP

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/geometry/geometries/register/ring.hpp>

#include "arolib/types/polygon.hpp"
#include "arolib/types/linestring.hpp"
#include "arolib/misc/basicconversions.hpp"

//NOTE! do not register boost geometries here, otherewise they cannot be re-register. register them in the cpp's so that it is registered only withing that scope

namespace arolib{
namespace geometry{

typedef boost::geometry::model::d2::point_xy<double> boost_point_t;
typedef boost::geometry::model::linestring<boost_point_t> boost_linestring_t;
typedef boost::geometry::model::polygon<boost_point_t > boost_polygon_t;

/**
 * @brief Convert a vector of points into a boost_linestring_t
 * @param points Points
 * @return boost_linestring_t
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
boost_linestring_t toBoostLinesting(const std::vector<T>& points){
    boost_linestring_t boostLS;
    std::string description = "LINESTRING(";
    for(size_t i = 0 ; i < points.size() ; ++i){
        description += double2string(points.at(i).x) + " " + double2string(points.at(i).y);
        if(i != points.size()-1) description += ",";
    }
    description += ")";

    boost::geometry::read_wkt(description, boostLS);
    return boostLS;
}

/**
 * @brief Convert a PolygonWithHoles into a boost_polygon_t
 * @param poly Polygon
 * @return boost_polygon_t
 */
boost_polygon_t toBoostPolygon(const PolygonWithHoles & poly);

/**
 * @brief Convert an outter polygon and set of inner polygons (holes) into a boost_polygon_t
 * @param outer Outter polygon
 * @param inners Inner polygons (holes)
 * @return boost_polygon_t
 */
boost_polygon_t toBoostPolygon(const Polygon & outer, const std::vector<Polygon> & inners = {});

/**
 * @brief Convert a boost_polygon_t into an arolib PolygonWithHoles
 * @param boostPoly Boost polygon
 * @return PolygonWithHoles
 */
PolygonWithHoles fromBoostPolygon(const boost_polygon_t & boostPoly);

/**
 * @brief Convert a boost_polygon_t into the respective arolib outter polygon and inner polygons (holes)
 * @param boostPoly Boost polygon
 * @param outer [out] Outter polygon
 * @param inners [out] [optional] Inner polygons (holes)
 * @return PolygonWithHoles
 */
void fromBoostPolygon(const boost_polygon_t & boostPoly, Polygon & outter, std::vector<Polygon>* inners = nullptr);

}
}
#endif //AROLIB_BOOST_GEOMETRIES_WRAPPER_HPP
