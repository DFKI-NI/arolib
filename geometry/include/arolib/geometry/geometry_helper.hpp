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
 
#ifndef AROLIB_GEOMETRY_HELPER_HPP
#define AROLIB_GEOMETRY_HELPER_HPP

#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <limits.h>
#include <stdexcept>
#include <functional>

#include "boost_geometries_wrapper.hpp"
#include "3rdParty/clipper/clipper.hpp"

#include "arolib/types/polygon.hpp"
#include "arolib/types/linestring.hpp"
#include "arolib/types/pose2D.hpp"
#include "arolib/types/route_point.hpp"
#include "arolib/types/coordtransformer.hpp"
#include "arolib/types/units.hpp"
#include "arolib/misc/basicconversions.hpp"

namespace arolib{

/**
 * Geometry namespace.
 */
namespace geometry{

/**
 * Polygon validity type.
 */
enum PolygonValidity{
    VALID_OPEN_CW, /**< Valid, Open, Clockwise */
    VALID_CLOSED_CW, /**< Valid, Closed, Clockwise */
    VALID_OPEN_CCW, /**< Valid, Open, Counterclockwise */
    VALID_CLOSED_CCW, /**< Valid, Closed, Counterclockwise */
    INVALID_POLYGON /**< Invalid */
};

/**
 * Test if two doubles are equal.
 * @param a First value
 * @param b Second value
 * @return True if considered equal
 */
bool testEqual(double a, double b);

/**
  calc distance between two points
 * @param p0 First point
 * @param p1 Second point
 * @return Distance
 **/
double calc_dist(const Point& p0, const Point& p1);

/**
  calc manhattan istance between two points
 * @param p0 First point
 * @param p1 Second point
 * @return Manhattan distance
 **/
double calc_manhattan_dist(const Point& p0, const Point& p1, bool includeZ = false);

/**
 * @brief Computes the length of the given geometry by
 *        summing up the lengths of all segments.If a start and end index
 *        is given only the length between these indices is computed.
 * @param geom the geometry
 * @param start_index optional start index
 * @param end_index optional end index
 * @return Geometry length
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
double getGeometryLength(const std::vector<T>&geom,
                         int start_index  = 0,
                         int end_index = -1 );

/**
  calc the area of a line/rectangle made up of two points and a given width
 * @param p0 First point of the line
 * @param p1 Second point of the line
 * @param width Width of the line
 * @return Area
 **/
double calc_area(const Point& p0, const Point& p1, double width);

/**
 * @brief Get the area of a polygon
 * @param poly Polygon
 * @return Area of a polygon
 */
double calc_area(const Polygon& poly);

/**
 * @brief Corrects the angle to be between (-pi,pi] or [0,2pi)
 * @param ang [in/out] angle
 * @param inDeg Is the angle in degrees?
 * @param onlyPositive if true, angle range = [0,2pi), else [-pi,pi]
 */
void correct_angle(double &ang, bool inDeg = false, bool onlyPositive = false);

/**
 * @brief Gets the angle between a line and the horizontal axis
 * @param p0 First point of the line
 * @param p1 Second point of the line
 * @param inDeg If set to true, returns the angle in degrees, otherwise in Radians
 * @param limit If set to true, returns angles between [-pi/2,pi/2], otherwise between [-pi,pi]
 * @return angle
 */
double get_angle(const Point& p0, const Point& p1, bool inDeg = false, bool limit = false);

/**
 * @brief Gets the angle between two vectors
 * @param p1_0 First point of the first vector
 * @param p1_1 Second point of the first vector
 * @param p2_0 First point of the second vector
 * @param p2_1 Second point of the second vector
 * @param inDeg If set to true, returns the angle in degrees, otherwise in Radians
 * @param limit If set to true, returns angles between [-pi/2,pi/2], otherwise between [-pi,pi]
 * @return angle
 */
double get_angle(const Point& p1_0,
                 const Point& p1_1,
                 const Point& p2_0,
                 const Point& p2_1,
                 bool inDeg = false,
                 bool limit = false);

/**
 * @brief Gets the angle between the vectors [pivot, p1] and [pivot, p2]
 * @param p1 Point of the first vector
 * @param pivot Pivot point (common to the 2 vectors)
 * @param p2 Point of the second vector
 * @param inDeg If set to true, returns the angle in degrees, otherwise in Radians
 * @param limit If set to true, returns angles between [-pi/2,pi/2], otherwise between [-pi,pi]
 * @return angle
 */
double get_angle(const Point& p1,
                 const Point& pivot,
                 const Point& p2,
                 bool inDeg = false,
                 bool limit = false);

/**
 * @brief Computes the bearing from a line/vector
 * @param p0 First point
 * @param p1 second point
 * @param inUTM If true, the points are considered to be in UTM
 * @return bearing
 */
double get_bearing(const Point& p0, const Point& p1, bool inUTM);

/**
 * @brief Computes the vector normal to a given line/vector
 * @param p0 First point of input line/vector
 * @param p1 second point of input line/vector
 * @param dX [out] dX of normal vector
 * @param dY [out] dY of normal vector
 * @param length Length of the output vector
 * @return True on success
 */
bool getNormVector(const Point &p0, const Point &p1, double &dX, double &dY, double length = 1);

/**
 * @brief Computes the vector normal to a given line/vector
 * @param p0 First point of input line/vector
 * @param p1 second point of input line/vector
 * @param vec [out] Normal vector ( w.r.t. (0,0) )
 * @param length Length of the output vector
 * @return True on success
 */
bool getNormVector(const Point &p0, const Point &p1, Point&vec, double length = 1);

/**
 * @brief Computes the vector normal to a given linestring := sum o the normal vectors of the linestring subsegments
 * @param points Linestring
 * @param vec [out] Normal vector ( w.r.t. (0,0) )
 * @param length Length of the output vector
 * @return True on success
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
bool getNormVector(const std::vector<T> &points, Point& vec, double length = 1);

/**
 * @brief Computes the vector parallel to a given line/vector
 * @param p0 First point of input line/vector
 * @param p1 second point of input line/vector
 * @param dX [out] dX of parallel vector
 * @param dY [out] dY of parallel vector
 * @param length Length of the output vector
 * @return True on success
 */
bool getParallelVector(const Point &p0, const Point &p1, double &dX, double &dY, const double length = 1);

/**
 * @brief Computes the vector parallel to a given line/vector
 * @param p0 First point of input line/vector
 * @param p1 second point of input line/vector
 * @param vec [out] Parallel vector ( w.r.t. (0,0) )
 * @param length Length of the output vector
 * @return True on success
 */
bool getParallelVector(const Point &p0, const Point &p1, Point&vec, const double length = 1);


/**
 * @brief Computes the centroid of a line/segment
 * @param p0 First point of input line
 * @param p1 second point of input line
 * @return Centroid
 */
Point getCentroid(const Point &p0, const Point &p1);

/**
 * @brief Computes the centroid of a polygon
 * @param poly Input polygon
 * @param centroid [out] Centroid
 * @return True on success
 */
bool getCentroid(const Polygon &poly, Point &centroid);

/**
 * @brief dot computes the dot product of the given points
 * @param v point 1
 * @param w point 2
 * @return the dot product
 */
double dot(const Point& v, const Point& w);

/**
 * @brief Computes the distance of a segment defined by the given points v and w with a given point p
 * @param v segment start
 * @param w segment end
 * @param p the point
 * @return distance
 */
double calc_distance(const Point& v, const Point& w, const Point& p);

/**
 * @brief Get the distance between a point an a line (given as two points)
 * @param p0 First point of the line.
 * @param p1 Second point of the line.
 * @param p Given point.
 * @param infinite The line is taken as an infinite line.
 * @param abs Return the absolute value. If set to false and the point is not in the direction of the nom vectors, the resulting distanc will be negative.
 * @return Distance between point p and the line
 */
double calc_dist_to_line(const Point& p0,
                         const Point& p1,
                         const Point& p,
                         bool infinite = true,
                         bool abs = true);

/**
 * @brief Get the distance between a point an a line (given as two points)
 * @param p0 First point of the line.
 * @param p1 Second point of the line.
 * @param p Given point.
 * @param p0ToInfinity Extend the line to infinity in the side of p0
 * @param p1ToInfinity Extend the line to infinity in the side of p1
 * @param abs Return the absolute value. If set to false and the point is not in the direction of the nom vectors, the resulting distanc will be negative.
 * @return Distance between point p and the line
 */
double calc_dist_to_line2(const Point& p0,
                          const Point& p1,
                          const Point& p,
                          bool p0ToInfinity = true,
                          bool p1ToInfinity = true,
                          bool abs = true);

/**
 * @brief Get the distance between a point an a linestring
 * @param points linestring
 * @param p Given point.
 * @param infinite The line is taken as an infinite line.
 * @param abs Return the absolute value. If set to false and the point is not in the direction of the nom vectors, the resulting distanc will be negative.
 * @return Distance between point p and the line
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
double calc_dist_to_linestring(const std::vector<T>& points, const Point& p, bool infinite = false);

/**
 * @brief Get the discrete frechet distance between two linestrings
 * @param points1 linestring1
 * @param points2 linestring2
 * @return discrete frechet distance between the two linestrings
 */
//template< typename T1, typename T2,
//          typename = typename std::enable_if< std::is_base_of<Point, T1>::value && std::is_base_of<Point, T2>::value, void >::type >
//double calc_discrete_frechet_distance (const std::vector<T1>& points1, const std::vector<T2>& points2);

/**
 * @brief Get the vector ( with origin in [0,0] ) corresponding to the minimum distance a point an a line (given as two points),
 * i.e. the direction in which the point hast to move (with a minimum distance) to intersect the line
 * @param p0 First point of the line.
 * @param p1 Second point of the line.
 * @param p Given point.
 * @param infinite The line is taken as an infinite line.
 * @return Output vector ( with origin in [0,0] )
 */
Point calc_vector_to_line(const Point& p0, const Point& p1, const Point& p, bool infinite = true);

/**
 * @brief Get the minimun distance between 2 consecutive points
 * @param points points
 * @return distance (<0 in error)
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
double getMinSampleDistance(const std::vector<T> &points);

/**
 * @brief Get the maximum distance between 2 consecutive points
 * @param points points
 * @return distance (<0 in error)
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
double getMaxSampleDistance(const std::vector<T> &points);

/**
* @brief Estimates the resolution of the geometry.
* @param geom the geometry
* @param start_index optional start index
* @param end_index optional end index
* @return resolution [m] (<0 in error)
*/
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
double getGeometryResolution(const std::vector<T>& geom);

//----------------------------------------------------------------------

/**
 * @brief Get the validity-state of the polygon
 * @param poly Input polygon
 * @return Validity
 */
PolygonValidity isPolygonValid(const Polygon &poly);

/**
 * @brief Check if a validity-state corresponds to a closed polygon
 * @param polyValidty Validty-state
 * @return True if the polygon is closed
 */
bool isPolygonClosed(const PolygonValidity& polyValidty);

/**
 * @brief Check if a polygon is closed
 * @param poly Input polygon
 * @return True if the polygon is closed
 */
bool isPolygonClosed(const Polygon &poly);

/**
 * @brief Check if a validity-state corresponds to a clockwise polygon
 * @param polyValidty Validty-state
 * @param polyValidty Polygon validty
 * @return True if the polygon is clockwise
 */
bool isPolygonClockwise(const PolygonValidity& polyValidty);

/**
 * @brief Check if a polygon is clockwise
 * @param poly Input polygon
 * @return True if the polygon is clockwise
 */
bool isPolygonClockwise(const Polygon &poly);

/**
 * @brief Corrects the polygone (for boost)
 * @param poly Polygon
 * @return Vector of intersection polygons
 */
void correct_polygon(Polygon& poly, bool clockwise = true);

/**
 * @brief Closes the polygon
 * @param poly [in/out] Polygon to be closed
 * @param _tolerance Tolerance [m] used to compare first and last points
 * @return True on success
 */
bool closePolygon(Polygon &poly, double tolerance = 0.0000001);

/**
 * @brief Opens the polygon
 * @param poly [in/out] Polygon to be opened
 * @param _tolerance Tolerance [m] used to compare first and last points
 * @return True on success
 */
bool openPolygon(Polygon &poly, double tolerance = 0.0000001);


//----------------------------------------------------------------------

/**
 * @brief Changes the length of a vector ( with origin (0,0) ).
 * @param vec [in/out].
 * @param length Desired length. if length < 0, the direction of the vector is inverted
 * @return  True on success
 */
bool setVectorLength(Point& vec, double length);

/**
 * @brief Takes a vector composed by p0 and p1 and changes p1 so that the distance between p0 and p1 is the desired one.
 * @param p0 First point of the line.
 * @param p1 [in/out] Second point of the line.
 * @param length Desired length. if length < 0, the direction of the vector is inverted
 * @return  True on success
 */
bool setVectorLength(const Point& p0, Point& p1, double length);

/**
 * @brief Rotate a point (right-hand conv -> counter-clockwise)
 * @param pivot
 * @param p Point to rotate w.r.t. the pivot
 * @param angle Rotation angle
 * @param inDeg Is the angle in degrees?
 * @return Rotated point
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
T rotate(const Point &pivot, const T &p, double angle, bool inDeg = false);


/**
 * @brief Rotate a geometry (right-hand conv -> counter-clockwise)
 * @param pivot
 * @param points Geometry to rotate w.r.t. the pivot
 * @param angle Rotation angle
 * @param inDeg Is the angle in degrees?
 * @return Rotated geometry
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
std::vector<T> rotate(const Point &pivot, const std::vector<T> &points, double angle, bool inDeg = false);

/**
 * @brief Moves a point to a specific distance from a line (based on the norm vector formed between the line and the point)
 * @param p0 First point of the base line.
 * @param p1 Second point of the base line.
 * @param p Output point point.
 * @param d Desired distance.
 * @return True in success
 */
bool move_point_to_dist_to_line(const Point& p0,
                                const Point& p1,
                                Point& p,
                                double d,
                                bool abs);

/**
   Interpolate a line
 * @param p0 First point of the line.
 * @param p1 Second point of the line.
 * @param x X-value.
 * @return Interpolated y- value for the given x
 **/
double interp_line(const Point& p0, const Point& p1, double x );


/**
 Get a point at a distance from a pose given the pose angle
 * @param pose Pose.
 * @param dist Distance from pose.
 * @return Point
**/
Point getPointAtDist(const Pose2D& pose, double dist );



/**
 Get a new point in a line at given distance to the line's second point
 * @param p0 First point of the line.
 * @param p1 Second point of the line.
 * @param dist Distance from p1.
 * @return Point
 **/
Point extend_line(const Point& p0, const Point&p1, double dist);

/**
 Extends the two extrema of the given linestring
 * @param ls [in/out] Linestring.
 * @param dist0 Distance to the first point of ls.
 * @param distn Distance to the last point of ls.
 * @param keepExtremaPoints Should the original first and last points of the linestring be kept?.
 * @return True on success
 **/
bool extend_linestring(std::vector<Point>& ls, double dist0, double distn, bool keepExtremaPoints = false);

/**
 Offset a line
 * @param p0 First point of the input line.
 * @param p1 Second point of the input line.
 * @param new_p0 [out] First point of the output (offset) line.
 * @param new_p1 [out] Second point of the output (offset) line.
 * @param offset Offset distance.
 **/
void offset_line(const Point& p0, const Point& p1, Point& new_p0, Point& new_p1, double offset);

/**
 Offset a linestring
 * @param points_in Input linestring.
 * @param points_out [out] Output (offset) linestring.
 * @param offset Offset distance (>0 --> left ; <0 --> right).
 * @param keepSamples Map the samples/points of the original linestring into the new one?.
 * @param points_per_circle Number of points per circle for smooth corners in the offset linestring.
 * @return True on success
 **/
bool offsetLinestring(const std::vector<Point> &points_in,
                      std::vector<Point> &points_out,
                      double offset,
                      bool keepSamples = false,
                      int points_per_circle = 18);

/**
 Offset a linestring to create a polygon from it
 * @param points_in Input linestring.
 * @param poly_out [out] Output polygon.
 * @param offset1 Offset distance (left).
 * @param offset2 Offset distance (right).
 * @param endFlat If false, the first and last points in the linestring will also be offset; otherwise, the sides of the output polygon coresponding to those points will be flat.
 * @param points_per_circle Number of points per circle for smooth corners in the offset linestring.
 * @return True on success
 **/
bool offsetLinestring(const std::vector<Point> &points_in,
                      Polygon &poly_out,
                      double offset1,//left
                      double offset2,//right
                      bool endFlat,
                      int points_per_circle = 18);

/**
 Offset a linestring (using boost)
 * @param points_in Input linestring.
 * @param points_out [out] Output (offset) linestring.
 * @param offset Offset distance (>0 --> left ; <0 --> right).
 * @param keepSamples Map the samples/points of the original linestring into the new one?.
 * @param points_per_circle Number of points per circle for smooth corners in the offset linestring.
 * @return True on success
 **/
bool offsetLinestring_boost(const std::vector<Point> &points_in,
                           std::vector<Point> &points_out,
                           double offset,
                           bool keepSamples = false,
                            int points_per_circle = 18);

/**
 Offset a linestring to create a polygon from it (using boost)
 * @param points_in Input linestring.
 * @param poly_out [out] Output polygon.
 * @param offset1 Offset distance (left).
 * @param offset2 Offset distance (right).
 * @param endFlat If false, the first and last points in the linestring will also be offset; otherwise, the sides of the output polygon coresponding to those points will be flat.
 * @param points_per_circle Number of points per circle for smooth corners in the offset linestring.
 * @return True on success
 **/
bool offsetLinestring_boost(const std::vector<Point> &points_in,
                            Polygon &boundary_out,
                            double offset1,
                            double offset2,
                            bool endFlat,
                            int points_per_circle = 18);

/**
 (old implementation) Offset a linestring
 * @param ls_in Input linestring.
 * @param ls_out [out] Output
 * @param offset Offset distance
 * @return True on success
 **/
bool offsetLinestring_old(const std::vector<Point> &ls_in, std::vector<Point> &ls_out, const double &offset);

/**
 Offset a polygon outwards (inflate) or inwards (deflate)
 * @param _poly_in Input polygon.
 * @param poly_out [out] Output (offset) polygon.
 * @param offset Offset distance.
 * @param inflated True-> offset outwards; false -> offset inwards.
 * @param points_per_circle Number of points per circle for smooth corners in the offset polygon.
 * @return True on success
 **/
bool offsetPolygon(const Polygon &_poly_in, Polygon &poly_out, double offset, bool inflated, size_t points_per_circle = 36);


/**
 Offset a polygon outwards (inflate) or inwards (deflate) using boost
 * @param _poly_in Input polygon.
 * @param poly_out [out] Output (offset) polygon.
 * @param offset Offset distance.
 * @param inflated True-> offset outwards; false -> offset inwards.
 * @param points_per_circle Number of points per circle for smooth corners in the offset polygon.
 * @return True on success
 **/
bool offsetPolygon_boost(const Polygon &_poly_in, Polygon &poly_out, double offset, bool inflated, size_t points_per_circle = 36);



/**
 Offset a polygon outwards (inflate) or inwards (deflate) using clipper
 * @param _poly_in Input polygon.
 * @param poly_out [out] Output (offset) polygon.
 * @param offset Offset distance.
 * @param inflated True-> offset outwards; false -> offset inwards.
 * @return True on success
 **/
bool offsetPolygon_clipper(const Polygon &_poly_in, Polygon &poly_out, double offset, bool inflated);


/**
 * @brief Translates a geometry a distance in a given direction
 * @param points geometry
 * @param direction vector containing the distance and direction of translation
 * @return Translated geometry
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
std::vector<T> translate(const std::vector<T>& points, const Point &direction);

/**
 * @brief Translates a geometry a distance in a given direction
 * @param points geometry
 * @param distance distance of translation
 * @param direction vector corresponding to the direction of translation
 * @return Translated geometry
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
std::vector<T> translate(const std::vector<T>& points, double distance, const Point &direction);

/**
 * @brief Adds a point (or more) to the geometry where the distance between the geometry and the given point is minimum
 * @param geom Geometry to add the point.
 * @param p reference point
 * @param maxPoints If there is more than one point, this is the max amount of points added. if =0, all points at the min distance will be added
 * @param minDist if the distance betwen the sample and the next/previous point is lower than thiis value, it wount be added
 * @return If geometry is empty, returns -1; Else, if maxPoints = 1, returns the index to the added point or to the original point closeset to the sample; else returns the number of added points.
 */
int addSampleToGeometryClosestToPoint(std::vector<Point>& geom,
                                      const Point& p,
                                      size_t maxPoints = 0,
                                      double minDist = 1e-3);

/**
 * @brief Remove the spikes of an (unvalid) polygon
 * @param poly [in/out] Polygon
 * @return True on success
 **/
bool removeSpikes(Polygon &poly);

/**
   Extract the subpolygons from an invalid polygon whose boundary intersects itself
 **/
std::vector<Polygon> extractSubPolygons(const Polygon& _poly_in );



//----------------------------------------------------------------------

/**
 * @brief Check if the points belong to the same line
 * @param points Vector of points
 * @param tolerance Tolerance to accept a point as 'in the line'
 * @return Points belong to the same line
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
bool is_line(const std::vector<T>& points, double tolerance = 0.001);

/**
 * @brief Check if the points belong to the same line
 * @param points Vector of points
 * @param tolerance Tolerance to accept a point as 'in the line'
 * @return Points belong to the same line
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
bool is_line2(const std::vector<T>& points, double tolerance = 0.001);

/**
 * @brief Check if the points belong to the same line
 * @param points Vector of points
 * @param tolerance Tolerance to accept a point as 'in the line'
 * @return Points belong to the same line
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
bool is_line3(const std::vector<T>& points, double tolerance = 0.001);

/**
 * @brief Check if the points belong to the same line, based on the angles
 * @param points Vector of points
 * @param tolerance Tolerance to accept a point as 'in the line'
 * @param angTolerance Angulat tolerance to accept a point as 'in the line' (rad)
 * @return Points belong to the same line
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
bool is_line4(const std::vector<T>& points, double tolerance = 0.001, double angTolerance = 0.0004);

/**
 * @brief Check if two lines are parall
 * @param p0_1 First point of the first line
 * @param p1_1 Second point of the first line
 * @param p0_2 First point of the second line
 * @param p1_2 Second point of the second line
 * @param tolerance Tolerance to consider two floats equal
 * @return True if lines are parallel
 */
bool checkLinesParallel(const Point& _p0_1,
                        const Point& _p1_1,
                        const Point& _p0_2,
                        const Point& _p1_2,
                        double tolerance = 0.0001);

//----------------------------------------------------------------------

/**
 * @brief Check if a point is inside a box
 * @param point Point
 * @param minX x-coordinate of the lower-left corner of the box
 * @param maxX x-coordinate of the upper-right corner of the box
 * @param minY y-coordinate of the lower-left corner of the box
 * @param maxY y-coordinate of the upper-right corner of the box
 * @return True if inside
 */
bool in_box(const Point& point, double minX, double maxX, double minY, double maxY);

/**
 * @brief Check if a point is inside a polygon
 * @param point Point
 * @param poly Polygon
 * @return True if inside
 */
bool in_polygon(const Point& point, const Polygon& poly);


/**
 * @brief Check if a point is inside a ring
 * @param point Point
 * @param ring Ring
 * @return True if inside
 */
bool in_polygon(const Point &point, const std::vector<Point> &ring);


/**
 * @brief Check if the points are inside a polygon
 * @param point Point
 * @param poly Polygon
 * @param all If true, the result is true iif all points lie inside the polygon; otherwise, the result is true if at least one of the points lie inside the polygon
 * @return True if inside
 */
bool in_polygon(const std::vector<Point> &points, const Polygon &poly, bool all = true);

/**
 * @brief Check if a linestring is inside a polygon
 * @param ls Linestring
 * @param poly Polygon
 * @param all If true, the result is true iif all points lie inside the polygon; otherwise, the result is true if at least one of the points lie inside the polygon
 * @return True if inside
 */
bool in_polygon(const Linestring& ls, const Polygon& poly, bool all = true);


/**
 * @brief Check if one polygon is inside another polygon
 * @param poly1 Polygon 1
 * @param poly2 Polygon 2
 * @return True if poly1 is inside poly2
 */
bool in_polygon(const Polygon& poly1, const Polygon& poly2);

/**
 * @brief Counts the number of points located inside a Polygon
 * @param points Points
 * @param polygon Polygon
 * @return Number of points located inside the polygon
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
size_t count_points_in_polygon(const std::vector<T>& points, const Polygon& polygon);


/**
 * @brief Get the intersection point between two lines
 * @param p0 First point of the line
 * @param p1 Second point of the line
 * @param p Input point
 * @param infinite_line If set to true, the input line will be considered as an infinite line, otherwise it will be considered finit with the limits being the two input points
 * @param tolerance Tolerance to consider two floats equal
 * @return True if an intersection was found
 */
bool checkPointInLine(const Point& p0,
                      const Point& p1,
                      const Point& p,
                      bool infinite_line = false,
                      double tolerance = 0.0001);

/**
 * @brief Get the intersection point between two lines
 * @param p0 First point of the line
 * @param p1 Second point of the line
 * @param p Input point
 * @param infinite_line If set to true, the input line will be considered as an infinite line, otherwise it will be considered finit with the limits being the two input points
 * @param tolerance Tolerance to consider two floats equal
 * @return True if an intersection was found
 */
bool checkPointInLine_2(const Point& p0,
                        const Point& p1,
                        const Point& p,
                        bool infinite_line = false,
                        double tolerance = 0.0001);

/**
 * @brief Checks if a point lies inside a (horzontal) rectangle
 * @param reference_corner Point corresponding to the reference corner (e.g. lower-left point of the rectangle)
 * @param rect_width Width of the rectangle (wrt reference_corner)
 * @param rect_height Height of the rectangle (wrt reference_corner)
 * @param p Point
 * @return False if the point lies outside the rectangle
 */
bool checkPointInRectangle(const Point& reference_corner,
                           double rect_width,
                           double rect_height,
                           const Point& p);

/**
 * @brief Checks if a segment of the input line lies inside a (horizontal) rectangle
 * @param reference_corner Point corresponding to the reference corner (e.g. lower-left point of the rectangle)
 * @param rect_width Width of the rectangle (wrt reference_corner)
 * @param rect_height Height of the rectangle (wrt reference_corner)
 * @param p0 first point of the input line
 * @param p1 end point of the input line
 * @return False if no part of the line lies inside the rectangle
 */
bool checkLineSegmentInRectangle(const Point& reference_corner,
                                 double rect_width,
                                 double rect_height,
                                 const Point& p0,
                                 const Point& p1);

/**
 * @brief Checks if a segment of the input line lies inside a polygon
 * @param poly Polygon
 * @param p0 first point of the input line
 * @param p1 end point of the input line
 * @return False if no part of the line lies inside the polygon
 */
bool checkLineSegmentInPolygon(const Polygon& poly,
                               const Point& p0,
                               const Point& p1);

//----------------------------------------------------------------------


/**
 * @brief Sample a geometry with a given resolution
 * @param geometry_in Input geometry
 * @param resolution Resolution (distance between consecutive samples)
 * @param minDist Minimum acceptable distance between consecutive samples. If < 0, a default value will be taken
 * @param reverse If true, the sampling process will start from the last point
 * @return Sampled geometry
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
std::vector<Point> sample_geometry(const std::vector<T>& geometry_in, double resolution, double minDist = -1, bool reverse = false);

/**
 * @brief Sample a geometry segment with a given resolution
 * @param geometry_in Input geometry
 * @param resolution Resolution (distance between consecutive samples)
 * @param indFrom Index corresponding to the first point of the segment
 * @param indTo Index corresponding to the last point of the segment (inclusive)
 * @param minDist Minimum acceptable distance between consecutive samples. If < 0, a default value will be taken
 * @return Sampled geometry
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
std::vector<Point> sample_geometry_segment(const std::vector<T>& geometry_in, double resolution, size_t indFrom, size_t indTo, double minDist = -1);


/**
 * @brief Sample the extrema (first and last) segments of a geometry with a given resolution
 * @param geometry_in Input geometry
 * @param resolution Resolution (distance between consecutive samples)
 * @param minDist Minimum acceptable distance between consecutive samples. If < 0, a default value will be taken
 * @return Sampled geometry
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
std::vector<Point> sample_geometry_ends(const std::vector<T>& geometry_in, double resolution, double minDist = -1);


/**
 * @brief Sample a line with a given resolution
 * @param p0 First point of the line
 * @param p1 Second point of the line
 * @param numSamples Number of samples for the sampled geometry
 * @return Sampled geometry
 */
std::vector<Point> sample_line(const Point& p0, const Point& p1, size_t numSamples);


/**
 * @brief Removes repeated consecutive points leaving either the first or the last occurance
 * @param points [in/out] Linestring points to be processed. The points will be edited internally.
 * @param leaveFirst Should the first point always be kept?
 * @param tolerance Tolerance to check if the points are equal
 * @return True in success
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
bool remove_repeated_points(std::vector<T>& points, bool leaveFirst = true, double tolerance = 0.001);

/**
 * @brief Delete all points of a linestring that are not vertices
 * @param points [in/out] Linestring points to be processed. The points will be edited internally.
 * @param tolerance Tolerance to check if the points are equal
 * @param angTolerance Angular tolerance [rad] to accept a point as 'in the same line'
 * @return True in success
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
bool unsample_linestring(std::vector<T>& points, double tolerance = 0.001, double angTolerance = 0.0004);

/**
 * @brief Delete all points of a linestring that are not vertices
 * @param points [in/out] Linestring points to be processed. The points will be edited internally.
 * @param tolerance Tolerance to accept a point as 'in the same line'
 * @return True in success
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
bool unsample_linestring2(std::vector<T>& points, double tolerance = 0.001);

/**
 * @brief (old implementation) Delete all points of a linestring that are not vertices
 * @param points [in/out] Linestring points to be processed. The points will be edited internally.
 * @param tolerance Tolerance to accept a point as 'in the same line'
 * @return True in success
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
bool unsample_linestring__old(std::vector<T>& points, double tolerance = 0.001);

/**
 * @brief Delete all points of a polygon that are not vertices
 * @param poly [in/out] Polygon to be processed. The Polygon will be edited internally and corrected.
 * @param tolerance Tolerance to accept a point as 'in the same line'
 * @param keepFirstPoint Should the first point always be kept?
 * @return True in success
 */
bool unsample_polygon(Polygon& poly, double tolerance = 0.001, bool keepFirstPoint = true);

/**
 * @brief (old implementation) Delete all points of a polygon that are not vertices
 * @param poly [in/out] Polygon to be processed. The Polygon will be edited internally and corrected.
 * @param tolerance Tolerance to accept a point as 'in the same line'
 * @param keepFirstPoint Should the first point always be kept?
 * @return True in success
 */
bool unsample_polygon__old(Polygon& poly, double tolerance = 0.001, bool keepFirstPoint = true);


//-------------------------------------------------------------------------------------

/**
 * @brief Creates a rectangle from a line, where the length of the rectangle is equal to the length of the line,
 * the desired width of the rectangle is given as an input, and the input line bisects the resulting rectangle.
 * (in other words, cretes a rectangle by expanding sideways the input line a distance of width/2)
 * @param p1 first point of the line
 * @param p2 end point of the line
 * @param width desired rectangle width
 * @param clockwise Create in clockwise direction?
 * @return Resulting rectangle
 */
Polygon createRectangleFromLine(const Point & p1, const Point & p2, double width, bool clockwise = true);

/**
 * @brief Creates a rectangle from a line, where the length of the rectangle is equal to the length of the line,
 * the desired width of the rectangle is given as an input, and the input line bisects the resulting rectangle.
 * (in other words, cretes a rectangle by expanding sideways the input line a distance of width/2)
 * @param p1 first point of the line
 * @param p2 end point of the line
 * @param width desired rectangle width
 * @param clockwise Create in clockwise direction?
 * @return Resulting rectangle
 */
Polygon createPolygon(const Point & p1, const Point & p2, double width, bool clockwise = true);

/**
 * @brief Creates a polygon by offseting a linestring
 * @param poly [out] Output polygon
 * @param ls linestring
 * @param width Width
 * @param clockwise Create in clockwise direction?
 * @return True on success
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
bool createPolygon(Polygon &poly, std::vector<T> ls, double width, bool clockwise = true);

/**
 * @brief Creates a circle (or semi-circle).
 *
 * If the output is a semicircle, the first and last points will correspond to the centroid
 * @param centroid centroid
 * @param radius radius
 * @param samples number of samples
 * @param startAngle startAngle w.r.t. the horizontal axis (right hand convention -> counter-clockwise)
 * @param angleRange Angular range (if 2*pi --> complete circle). If <0 -> clockwise ; If >0 -> counter-clockwise (unless alwaysClockwise = true)
 * @param inDegrees Are startAngle and angleRange in degrees?
 * @param alwaysClockwise Create in clockwise direction disregarding the given angular range?
 * @return (semi-) circle closed polygon
 */
Polygon create_circle(const Point &centroid,
                      double radius,
                      size_t samples,
                      double startAngle = 0.0,
                      double angleRange = 2*M_PI, //counter clockwise
                      bool inDegrees = false,
                      bool alwaysClockwise = false);

/**
 * @brief Get the second point of a line based on its first point, length and angle (w.r.t. the x-axis).
 * @param p0 First point of the line
 * @param length Length ofthe line
 * @param angle (w.r.t. the x-axis)
 * @param inDegrees is the given angle in degrees?
 * @return second point of the line
 */
Point create_line(const Point &p0, double length, double angle, bool inDegrees = false);


/**
 * @brief Creates a linestring with the given points
 * @param points points
 * @return linestring
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
Linestring toLinestring(const std::vector<T>& points);

/**
 * @brief Creates a linestring with the points of the polygon
 * @param poly polygon
 * @return linestring
 */
Linestring toLinestring(const Polygon& poly);

//-------------------------------------------------------------------------------------

/**
 * @brief Get the point in a geometry at minimum distance from another point
 * @param points Geometry points
 * @param p Reference point
 * @return Point at minimum distance
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
T getPointInMinDist(const std::vector<T> points, const Point &p);

/**
 * @brief Get the index of the point in a geometry at minimum distance from another point
 * @param points Geometry points
 * @param p Reference point
 * @return Index of the point
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
size_t getPointIndInMinDist(const std::vector<T> points, const Point &p);


/**
 * @brief Get the point in the line that is located at a given distance from the line's first point
 * @param p0 First point of the line
 * @param p1 Second point of the line
 * @param dist Distance from p0
 * @return Resulting point
 */
Point getPointInLineAtDist(const Point& p0, const Point &p1, double dist);

/**
 * @brief Get 1) the point at a given relative distance (relative to the geometry length) from the first point of a line; and 2) the index of the sample before the point
 * @param geom geometry
 * @param d relative distance [0, 1]
 * @return Point and index
 */
template< typename T,
         typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
std::pair<Point, int> getPointAtRelativeDist(const std::vector<T> &geom, double d);

/**
 * @brief Get 1) the point at half of the length of a geometry; and 2) the index of the sample before the point
 * @param geom geometry
 * @return Point at half length and index
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
std::pair<Point, int> getPointAtHalfLength(const std::vector<T> &geom);


/**
 * @brief Get the index of the first point in a geometry equal to a given point
 * @param geom geometry
 * @param p Reference point
 * @return index (<0 if not found)
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
int getPointIndex(const std::vector<T> &geom, const Point &p);

/**
 * @brief Returns the indices of the two points of the given geometry that form the closest segment to a given point
 * @param geom the geometry
 * @param p the point
 * @param [out] predecesser the first index of the identified segment
 * @param [out] sucessor   the second index of the identified segment
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
void getGeomIndices(const std::vector<T>& geom, const Point &p,
                    int &predecessor, int& sucessor);

 /**
  * @brief Returns the index of the point on the given geometry that is nearest to a given point
  * @param geom the geometry
  * @param p the point
  * @param distToLine If true, the index will be searched based on the distance from p to the segments of the geometry; if set to false, the index will be searched based on the distance from p to the points of the geometry
  * @return the first index of the identified segment
  */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
size_t getGeomIndex(const std::vector<T>& geom, const Point &p, bool distToLine = false);

/**
 * @brief Returns the real index corresponding to a given pseudo-index for a continuous geometry geom, where geom[i] := geom[i+size]
 * @param geom Continuous geometry
 * @param ind Pseudo index. Can be <0 or >geom.size
 * @param p_out [out] Corresponding point (optional)
 * @return Real index
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
size_t getIndexFromContinuousGeometry(const std::vector<T>& geom, int ind, Point *p_out = nullptr);

/**
 * @brief Returns the point corresponding to a given pseudo-index for a continuous geometry geom, where geom[i] := geom[i+size]
 * @param geom Continuous geometry
 * @param ind Pseudo index. Can be <0 or >geom.size
 * @param [out] indReal Real index
 * @return point
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
T getPointFromContinuousGeometry(const std::vector<T>& geom, int ind, size_t* indReal = nullptr);

/**
 * @brief Gets the location of a point in a line. It assumes that the point is part of the line!!!
 * @param p0 First point of the line.
 * @param p1 Second point of the line.
 * @param p Point.
 * @param tolerance Tolerance.
 * @return 0 if the points is between p0 and p1; -1 if it is outside the line to the side of p0; 1 if it is outside the line to the side of p1
 */
int getLocationInLine(const Point &p0, const Point &p1, const Point &p, double tolerance = 1e-6);

//-------------------------------------------------------------------------------------

/**
 * @brief Get the part/segment of the given geometry that
 * connects the given indices. The function assumes that the geometry is
 * closed if start_index is greater than end_index.
 * @param start_index Start index
 * @param end_index End index
 * @return Segment of the given geometry that connects start_index and end_index
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
std::vector<Point> getGeometryPart(const std::vector<T>& geom,
                                   int start_index, int end_index);

/**
 * @brief Get the shortest part/segment of a CLOSED geometry that
 * connects the given points. The points do not have to be part of the given
 * geometry.
 * @param p1 start point of the part
 * @param p2 end point of the part
 * @param [out] the found start index
 * @param [out] the found end index
  * @param distToLine If true, the index will be searched based on the distance from p to the segments of the geometry; if set to false, the index will be searched based on the distance from p to the points of the geometry
 * @return Segment the given geometry that connects p1 and p2
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
std::vector<T> getShortestGeometryPart(const std::vector<T>& geom,
                                           const Point &p1, const Point &p2,
                                           size_t &start, size_t &end,
                                           bool distToLine = false);

/**
 * @brief Get the shortest part/segment of a CLOSED geometry that
 * connects the given points. The points do not have to be part of the given
 * geometry.
 * @param p1 start point of the part
 * @param p2 end point of the part
  * @param distToLine If true, the index will be searched based on the distance from p to the segments of the geometry; if set to false, the index will be searched based on the distance from p to the points of the geometry
 * @return Segment the given geometry that connects p1 and p2
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
std::vector<T> getShortestGeometryPart(const std::vector<T>& geom,
                                       const Point &p1, const Point &p2,
                                       bool distToLine = false);

/**
 * @brief Get the longest part/segment of a CLOSED geometry that
 * connects the given points. The points do not have to be part of the given
 * geometry.
 * @param p1 start point of the part
 * @param p2 end point of the part
 * @param [out] the found start index
 * @param [out] the found end index
  * @param distToLine If true, the index will be searched based on the distance from p to the segments of the geometry; if set to false, the index will be searched based on the distance from p to the points of the geometry
 * @return Segment the given geometry that connects p1 and p2
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
std::vector<T> getLongestGeometryPart(const std::vector<T>& geom,
                                           const Point &p1, const Point &p2,
                                           size_t &start, size_t &end,
                                           bool distToLine = false);

/**
 * @brief Get the longest part/segment of a CLOSED geometry that
 * connects the given points. The points do not have to be part of the given
 * geometry.
 * @param p1 start point of the part
 * @param p2 end point of the part
  * @param distToLine If true, the index will be searched based on the distance from p to the segments of the geometry; if set to false, the index will be searched based on the distance from p to the points of the geometry
 * @return Segment the given geometry that connects p1 and p2
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
std::vector<T> getLongestGeometryPart(const std::vector<T>& geom,
                                           const Point &p1, const Point &p2,
                                           bool distToLine = false);

//-------------------------------------------------------------------------------------


/**
 * @brief Check if a line [p0, p1] intersects with a linestring
 * @param p0 First point of the line
 * @param p1 Second point of the line
 * @param points linestring
 * @param p0ToInfinity Extend the line to infinity in the side of p0
 * @param p1ToInfinity Extend the line to infinity in the side of p1
 * @return True if an intersection was found
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
bool intersects(const Point& p0, const Point& p1, const std::vector<T>& points, bool p0ToInfinity = false, bool p1ToInfinity = false);

/**
 * @brief Check if two linestrings intersect
 * @param points1 linestring 1
 * @param points2 linestring 2
 * @param p0ToInfinity Extend the linestring 1 to infinity in the side of the first point ( points1[0] )
 * @param p1ToInfinity Extend the linestring 1 to infinity in the side of the last point ( points1[n-1] )
 * @return True if an intersection was found
 */
template< typename T1, typename T2,
          typename = typename std::enable_if< std::is_base_of<Point, T1>::value && std::is_base_of<Point, T2>::value, void >::type >
bool intersects(const std::vector<T1>& points1, const std::vector<T2>& points2, bool p0ToInfinity = false, bool pnToInfinity = false);

/**
 * @brief Check if a line [p0, p1] intersects with a polygon
 * @param p0 First point of the line
 * @param p1 Second point of the line
 * @param poly polygon
 * @param p0ToInfinity Extend the line to infinity in the side of p0
 * @param p1ToInfinity Extend the line to infinity in the side of p1
 * @return True if an intersection was found
 */
bool intersects(const Point& p0, const Point& p1, const Polygon& poly, bool p0ToInfinity = false, bool p1ToInfinity = false);

/**
 * @brief Check if a linestring intersects with a polygon
 * @param points linestring
 * @param poly polygon
 * @param p0ToInfinity Extend the first segment of the linestring to infinity
 * @param pnToInfinity Extend the last segment of the linestring to infinity
 * @return True if an intersection was found
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
bool intersects(const std::vector<T>& points, const Polygon& poly, bool p0ToInfinity = false, bool pnToInfinity = false);

/**
 * @brief Check if 2 polygons intersect
 * @param poly1 polygon1
 * @param poly2 polygon2
 * @return True if an intersection was found
 */
bool intersects(const Polygon& poly1, const Polygon& poly2);

/**
 * @brief Check if a geometry self-intersects
 * @param geom geometry
 * @return True if the geometry self-intersects
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
bool intersects(const std::vector<T>& geom);

/**
 * @brief Check if a polygon self-intersects
 *
 * Corrects the polygon if necessary
 * @param poly polygon
 * @return True if polygon self-intersects
 */
bool intersects(const Polygon& poly);

/**
 * @brief Count the times a line [p0, p1] intersects with a polygon
 * @param p0 First point of the line
 * @param p1 Second point of the line
 * @param poly polygon
 * @param p0ToInfinity Extend the line to infinity in the side of p0
 * @param p1ToInfinity Extend the line to infinity in the side of p1
 * @param tolerance Tolerance to consider two floats equal
 * @return Number of intersections
 */
size_t count_intersections(const Point& p0, const Point& p1, const Polygon& poly, bool p0ToInfinity = false, bool p1ToInfinity = false, double tolerance = 0.0001);

/**
 * @brief Count the times a linestring intersects with a polygon
 * @param points linestring
 * @param poly polygon
 * @param p0ToInfinity Extend the first segment of the linestring to infinity
 * @param pnToInfinity Extend the last segment of the linestring to infinity
 * @param tolerance Tolerance to consider two floats equal
 * @return Number of intersections
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
size_t count_intersections(const std::vector<T>& points, const Polygon& poly, bool p0ToInfinity = false, bool pnToInfinity = false, double tolerance = 0.0001);

/**
 * @brief Get the intersection point between two lines
 * @param p0_1 First point of the first line
 * @param p1_1 Second point of the first line
 * @param p0_1 First point of the first line
 * @param p1_1 Second point of the first line
 * @param p_int Output intersection point
 * @param l1_infinite If set to true, the input line 1 will be considered as an infinite line, otherwise it will be considered finit with the limits being the two input points
 * @param l2_infinite If set to true, the input line 2 will be considered as an infinite line, otherwise it will be considered finit with the limits being the two input points
 * @param tolerance Tolerance to consider two floats equal
 * @return True if an intersection was found
 */
bool get_intersection(const Point& p0_1,
                      const Point& p1_1,
                      const Point& p0_2,
                      const Point& p1_2,
                      Point& p_int,
                      bool l1_infinite = false,
                      bool l2_infinite = false,
                      double tolerance = 0.0001);
/**
 * @brief Get the intersection points (linestring) between two lines
 *
 * The output linestring will have only one point unless the two lines are parallel and overlapping. In this case, at most one of the lines can be taken as infinite.
 * @param p0_1 First point of the first line
 * @param p1_1 Second point of the first line
 * @param p0_1 First point of the first line
 * @param p1_1 Second point of the first line
 * @param l1_infinite If set to true, the input line 1 will be considered as an infinite line, otherwise it will be considered finit with the limits being the two input points
 * @param l2_infinite If set to true, the input line 2 will be considered as an infinite line, otherwise it will be considered finit with the limits being the two input points
 * @param tolerance Tolerance to consider two floats equal
 * @return Output intersection linestring
 */
Linestring get_intersection(const Point& p0_1,
                            const Point& p1_1,
                            const Point& p0_2,
                            const Point& p1_2,
                            bool l1_infinite = false,
                            bool l2_infinite = false,
                            double tolerance = 0.0001);

/**
 * @brief Get the intersection(s) between a line and a polygon
 * @param p0 First point of the line
 * @param p1 Second point of the line
 * @param poly Polygon
 * @return Vector of intersection Points
 */
std::vector<Point> get_intersection(const Point& p0, const Point& p1, const Polygon& poly);


/**
 * @brief Get the intersection points between a linestring and a polygon
 * @param ls linestring
 * @param poly Polygon
 * @return Vector of intersection points
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
std::vector<Point> get_intersection(const std::vector<T>& ls, const Polygon& poly);

/**
 * @brief Get the intersection points between a linestring and a polygon
 * @param ls linestring
 * @param poly Polygon
 * @return Vector of intersection points
 */
std::vector<Point> get_intersection(const std::deque<Point>& ls, const Polygon& poly);

/**
 * @brief Get the intersection linescting(s) between two linestrings
 * @param ls1 linestring1
 * @param ls2 linestring2
 * @return Vector of intersection linestrings
 */
template< typename T1, typename T2,
          typename = typename std::enable_if< std::is_base_of<Point, T1>::value && std::is_base_of<Point, T2>::value, void >::type >
std::vector< Point > get_intersection(const std::vector<T1>& ls1, const std::vector<T2>& ls2);

/**
 * @brief Get the intersection linescting(s) between two linestrings
 * @param ls1 linestring1
 * @param ls2 linestring2
 * @return Vector of intersection linestrings
 */
std::vector< Point > get_intersection(const std::deque<Point>& ls1, const std::deque<Point>& ls2);

/**
 * @brief Get the intersection(s) between a line and a geometry
 * @param p0 First point of the line
 * @param p1 Second point of the line
 * @param geom Geometry points
 * @param orderToP0 If set to true, the intersection will be ordered starting from the side of p0
 * @param p0ToInfinity Extend the line to infinity in the side of p0
 * @param p1ToInfinity Extend the line to infinity in the side of p1
 * @param tolerance Tolerance to consider two floats equal
 * @return Vector of intersection Points
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
std::vector<Point> get_intersection(const Point& p0,
                                    const Point& p1,
                                    const std::vector<T> &geom,
                                    bool orderToP0 = true,
                                    bool p0ToInfinity = false,
                                    bool p1ToInfinity = false,
                                    double tolerance = 0.0001);

/**
 * @brief Get the intersection(s) between a line and a geometry
 * @param geom Geometry points
 * @param p0 First point of the line
 * @param p1 Second point of the line
 * @param orderToGeomStart If set to true, the intersection will be ordered starting fro the first point of the geometry
 * @param p0ToInfinity Extend the line to infinity in the side of p0
 * @param p1ToInfinity Extend the line to infinity in the side of p1
 * @param tolerance Tolerance to consider two floats equal
 * @return Vector of intersection Points
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
std::vector<Point> get_intersection(const std::vector<T> &geom,
                                    const Point& p0,
                                    const Point& p1,
                                    bool orderToGeomStart = true,
                                    bool p0ToInfinity = false,
                                    bool p1ToInfinity = false,
                                    double tolerance = 0.0001);

/**
 * @brief Get the intersection(s) between 2 polygons
 * @param poly1 Polygon 1
 * @param poly2 Polygon 2
 * @return Vector of intersection polygons
 */
std::vector<Polygon> get_intersection(const Polygon& poly1, const Polygon& poly2);

/**
 * @brief Get the intersection(s) between 2 polygons, where an intersection is expected. If no intersection found, poly2 will be offset and the intersections will be recalculated
 * @param poly1 Polygon 1
 * @param poly2 Polygon 2
 * @param offset offset for second check
 * @return Vector of intersection polygons
 */
std::vector<Polygon> get_likely_intersection(const Polygon& poly1, const Polygon& poly2, double offset);


/**
 * @brief Get the segment of the input line that lies inside a polygon
 * @param poly Polygon
 * @param p0_in first point of the input line
 * @param p1_in end point of the input line
 * @param p0_out first point of the output line
 * @param p1_out end point of the output line
 * @return False if no part of the line lies inside the polygon
 */
bool getLineSegmentInPolygon(const Polygon& poly,
                             const Point& p0_in,
                             const Point& p1_in,
                             Point& p0_out,
                             Point& p1_out);

/**
 * @brief Get the segment of the input line that lies inside a box
 * @param reference_corner Point corresponding to the reference corner (e.g. lower-left point of the rectangle)
 * @param rect_width Length of the rectangle (wrt reference_corner)
 * @param rect_height Height of the rectangle (wrt reference_corner)
 * @param p0_in first point of the input line
 * @param p1_in end point of the input line
 * @param p0_out first point of the output line
 * @param p1_out end point of the output line
 * @param infinite_line If set to true, the input line will be considered as an infinite line, otherwise it will be considered finit with the limits being the two input points
 * @return False if no part of the line lies inside the polygon
 */
bool getLineSegmentInRectangle(const Point& reference_corner,
                               double rect_width,
                               double rect_height,
                               const Point& p0_in,
                               const Point& p1_in,
                               Point& p0_out,
                               Point& p1_out,
                               bool infinite_line = false);

/**
 * @brief Get the union(s) between 2 polygons
 * @param poly1 Polygon 1
 * @param poly2 Polygon 2
 * @return Vector of union polygons
 */
void get_union(const Polygon& poly1, const Polygon& poly2, std::vector <PolygonWithHoles> &polys_out);

/**
 * @brief Get the union(s) between 2 geometries with outter and inner polygons
 * @param outer1 Outer boundary/polygon of the first geometry
 * @param inners1 Inner boundaries/polygons (holes) of the first geometry
 * @param outer2 Outer boundary/polygon of the second geometry
 * @param inners2 Inner boundaries/polygons (holes) of the second geometry
 * @return Vector of union polygons
 */
void get_union(const Polygon& outer1,
               const std::vector<Polygon>& inners1,
               const Polygon& outer2,
               const std::vector<Polygon>& inners2,
               std::vector<PolygonWithHoles> &polys_out);

/**
 * @brief Get the polygons corresponding to the 'difference' between two polygons
 *
 * i.e. Polygons corresponding to the union of two polygons minus their intersection
 * @param poly1 Polygon 1
 * @param poly2 Polygon 2
 * @return Vector of difference polygons
 */
void subtract_intersection(const Polygon& poly1, const Polygon& poly2, std::vector<PolygonWithHoles> &polys_out);

//-------------------------------------------------------------------------------------


/**
 * @brief Get the limits of a geometry (bounding box containing the geometry)
 * @param geom geometry
 * @param [out] minX X-value of the lower-left corner of the bounding box
 * @param [out] minY Y-value of the lower-left corner of the bounding box
 * @param [out] maxX X-value of the upper-right corner of the bounding box
 * @param [out] maxY Y-value of the upper-right corner of the bounding box
 * @return True on success
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
bool getGeometryLimits(const std::vector<T> &geom, double &minX, double &maxX, double &minY, double &maxY);

/**
 * @brief Get the limits of a polygon (bounding box containing the polygon)
 * @param poly polygon
 * @param [out] minX X-value of the lower-left corner of the bounding box
 * @param [out] minY Y-value of the lower-left corner of the bounding box
 * @param [out] maxX X-value of the upper-right corner of the bounding box
 * @param [out] maxY Y-value of the upper-right corner of the bounding box
 * @return True on success
 */
bool getPolygonLimits(const Polygon &poly, double &minX, double &maxX, double &minY, double &maxY);

/**
 * @brief Get the bounding box of a geometry
 * @param poly polygon
 * @param [out] bbox Bounding box
 * @return True on success
 */
bool getPolygonBoundingBox(const Polygon &poly, Polygon &bbox);

/**
 * @brief Get the bounding box of a geometry
 * @param geom geom geometry
 * @param [out] bbox Bounding box
 * @return True on success
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
bool getGeometryBoundingBox(const std::vector<T> &geom, Polygon &bbox);

/**
 * @brief Get the index of the left-most point of a geometry
 * @param points geometry
 * @return Index of the left-most point
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
long getLeftmostPointInd(const std::vector<T>& points);

/**
 * @brief Get the index of the right-most point of a geometry
 * @param points geometry
 * @return Index of the right-most point
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
long getRightmostPointInd(const std::vector<T>& points);

/**
 * @brief Get the index of the lowest point of a geometry
 * @param points geometry
 * @return Index of the lowest point
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
long getLowermostPointInd(const std::vector<T>& points);

/**
 * @brief Get the index of the upper-most point of a geometry
 * @param points geometry
 * @return Index of the upper-most point
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
long getUppermostPointInd(const std::vector<T>& points);


/**
 * @brief Get the left-most point of a geometry
 * @param points geometry
 * @return left-most point
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
T getLeftmostPoint(const std::vector<T>& points);

/**
 * @brief Get the right-most point of a geometry
 * @param points geometry
 * @return right-most point
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
T getRightmostPoint(const std::vector<T>& points);

/**
 * @brief Get the lowest point of a geometry
 * @param points geometry
 * @return lowest point
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
T getLowermostPoint(const std::vector<T>& points);

/**
 * @brief Get the upper-most point of a geometry
 * @param points geometry
 * @return upper-most point
 */
template< typename T,
          typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
T getUppermostPoint(const std::vector<T>& points);

/**
 * @brief Get the index of the left-most point of a polygon
 * @param poly polygon
 * @return Index of the left-most point
 */
long getLeftmostPointIndInPolygon(const Polygon& poly);

/**
 * @brief Get the index of the right-most point of a polygon
 * @param poly polygon
 * @return Index of the right-most point
 */
long getRightmostPointIndInPolygon(const Polygon& poly);

/**
 * @brief Get the index of the lower-most point of a polygon
 * @param poly polygon
 * @return Index of the lower-most point
 */
long getLowermostPointIndInPolygon(const Polygon& poly);

/**
 * @brief Get the index of the upper-most point of a polygon
 * @param poly polygon
 * @return Index of the upper-most point
 */
long getUppermostPointIndInPolygon(const Polygon& poly);

/**
 * @brief Get the left-most point of a polygon
 * @param poly polygon
 * @return left-most point
 */
Point getLeftmostPointInPolygon(const Polygon& poly);

/**
 * @brief Get the right-most point of a polygon
 * @param poly polygon
 * @return right-most point
 */
Point getRightmostPointInPolygon(const Polygon& poly);

/**
 * @brief Get the lower-most point of a polygon
 * @param poly polygon
 * @return lower-most point
 */
Point getLowermostPointInPolygon(const Polygon& poly);

/**
 * @brief Get the upper-most point of a polygon
 * @param poly polygon
 * @return upper-most point
 */
Point getUppermostPointInPolygon(const Polygon& poly);

}

}

#include "arolib/geometry/impl/geometry_helper.tcc"
#endif //AROLIB_GEOMETRY_HELPER_HPP
