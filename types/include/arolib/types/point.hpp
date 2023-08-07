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
 
#ifndef _AROLIB_POINT_H_
#define _AROLIB_POINT_H_

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>

#include <boost/functional/hash.hpp>

#include "arolib/misc/basicconversions.hpp"

namespace arolib {
/**
  * @brief Coordinate point (Cartesian, UTM, geodetic, ...).
  */
class Point {
public:

    double x; /**< X-coordinate (UTM: easting ; geo: longitude) */
    double y; /**< Y-coordinate (UTM: northing ; geo: latitude) */
    double z; /**< Z-coordinate (height) */

    /**
      * @brief Projection type (not really used, too tiring to mantain)
      */
    enum ProjectionType {
        UTM=0, /**< UTM. @externalpage https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system */
        WGS=1 /**< Geodetic */
    };

    /**
      * @brief Struct to get hash value
      */
    struct KeyHash{
        std::size_t operator()(const Point &p) const;
        static std::size_t get(const Point &p, std::size_t seed = 0);
    };

    /**
      * @brief Constructor.
      */
    explicit Point();

    /**
      * @brief Constructor with location parameters
      * @param _x x-coordinate
      * @param _y y-coordinate
      * @param _z z-coordinate
      */
    explicit Point(double _x, double _y, double _z=0);

    /**
      * @brief Destructor.
      */
    virtual ~Point() = default;

    /**
      * @brief Points addition
      *
      * The operation is done for each of the axis (x, y and z), e.g. result.x = this.x + p.x
      * @param p Point to be added to the (local) point
      * @return Result point
      */
    Point operator+(const Point &p) const;

    /**
      * @brief Points subtraction
      *
      * The operation is done for each of the axis (x, y and z), e.g. result.x = this.x - p.x
      * @param p Point to be subtracted from the (local) point
      * @return Result point
      */
    Point operator-(const Point &p) const;

    /**
      * @brief Scalar division
      *
      * The operation is done for each of the axis (x, y and z), e.g. result.x = this.x / s
      * @param s Scalar operand (divisor)
      * @return Result point
      */
    Point operator/(float s) const;

    /**
      * @brief Scalar multiplication
      *
      * The operation is done for each of the axis (x, y and z), e.g. result.x = this.x * s
      * @param s Scalar operand (multiplier/factor)
      * @return Result point
      */
    Point operator*(const double s);

    /**
      * @brief Subtract a point from the (local) point
      *
      * The operation is done for each of the axis (x, y and z), e.g. this.x -= p.x
      * @param p Point to be subtracted to the (local) point
      * @return Reference to this point
      */
    Point& operator-=(const Point &p);

    /**
      * @brief Add a point to the (local) point
      *
      * The operation is done for each of the axis (x, y and z), e.g. this.x += p.x
      * @param p Point to be added to the (local) point
      * @return Reference to this point
      */
    Point& operator+=(const Point &p);

    /**
      * @brief Multiply the point by a scalar
      *
      * The operation is done for each of the axis (x, y and z), e.g. this.x *= s
      * @param s Scalar operand (multiplier/factor)
      * @return Reference to this point
      */
    Point& operator*=(const double s);

    /**
      * @brief Divide the point by a scalar
      *
      * The operation is done for each of the axis (x, y and z), e.g. this.x /= s
      * @param s Scalar operand (divisor)
      * @return Reference to this point
      */
    Point& operator/=(const double s);

    /**
      * @brief Check if the locations of the two points are equal (with a very small tolerance)
      *
      * @todo at the moment, only the distance between the two points in the x-y plane (projection) is checked. In the future it might be needed to include z.
      * @param p Other point to be compared
      * @return True if the location of this point is the same as the one from p.
      */
    bool operator==(const Point &p) const;

    /**
      * @brief Check if the locations of the two points are diferent (with a very small tolerance)
      *
      * @todo at the moment, only the distance between the two points in the x-y plane (projection) is checked. In the future it might be needed to include z.
      * @param p Other point to be compared
      * @return True if the location of this point is different from the one from p.
      */
    bool operator!=(const Point &p) const;

    /**
      * @brief operator < (used for sorting --> no significant 'quantitative/qualitative' meaning)
      *
      * The order of checking is: 1) x-coordinate; 2) y-coordinate; 3) z-coordinate
      * @param p Other point to be compared
      * @return True if the (local) point has lower attributes than p (following the order in the method description)
      */
    bool operator<(const Point &p) const;

    /**
      * @brief Compute the distance between two points
      * @todo Computes the distance of the points projected in the x-y plane. In the future it might be needed to include z.
      * @param a One point
      * @param b Anothe point
      * @return Distance between the two points
      */
    static double dist(const Point& p0, const Point& p1);

    /**
     * @brief Computes the bearing from a line/vector. Assumes the points are in WGS!
     * @param p0 First point
     * @param p1 second point
     * @return bearing
     */
    static double bearing(const Point& p0_geo, const Point& p1_geo);

    /**
      * @brief Get the point attributes as a string
      * @param precision Decimal precision (disregarded if < 0)
      * @param incZ If true, the z-coordinate is included; otherwise, only x and y are included
      * @return String description of the point
      */
    std::string toString(int precision = 10, bool incZ = false) const;

    /**
      * @brief Get the point attributes as a string with CSV format (separator)
      * @param sep Character to be used as separator
      * @param precision Decimal precision (disregarded if < 0)
      * @param incZ If true, the z-coordinate is included; otherwise, only x and y are included
      * @return String description of the point
      */
    std::string toStringCSV(char sep = ';', int precision = 10, bool incZ = false) const;

    /**
      * @brief Print a set of points in a string with CSV format
      * @param points Points to be printed
      * @param sep Character to be used as separator
      * @param precision Decimal precision (disregarded if < 0)
      * @param incZ If true, the z-coordinate is included; otherwise, only x and y are included
      * @return (CSV) String description of the set of points
      */
    static std::string toStringCSV(const std::vector<Point>& points, char sep = ';', int precision = 10, bool incZ = false);

    /**
      * @brief Print a (several) sets of points in a string with CSV format
      * @param points Sets of points to be printed
      * @param sep Character to be used as separator
      * @param precision Decimal precision (disregarded if < 0)
      * @param incZ If true, the z-coordinate is included; otherwise, only x and y are included
      * @return (CSV) String description of the sets of points
      */
    static std::string toStringCSV(const std::vector< std::vector<Point> >& v_points, char sep = ';', int precision = 10, bool incZ = false);

    /**
      * @brief Print a (several) sets of points in a string with CSV format
      * @param points Sets of points to be printed
      * @param sep Character to be used as separator
      * @param precision Decimal precision (disregarded if < 0)
      * @param incZ If true, the z-coordinate is included; otherwise, only x and y are included
      * @return (CSV) String description of the sets of points
      */
    static std::string toStringCSV(const std::vector< const std::vector<Point>* >& v_points, char sep = ';', int precision = 10, bool incZ = false);

    /**
      * @brief Set the point as invalid
      */
    void setInvalid();

    /**
      * @brief Check if the point is invalid
      */
    bool isValid() const;

    /**
      * @brief Get an invalid point
      */
    static Point invalidPoint();

    /**
      * @brief Convert a vector of point-derived classes to a vector of basic points
      * @param in vector of point-derived classes
      * @param ind0 Start index (inclusive)
      * @param indn Stop index (inclusive). if <0 || >=in.size -> indn = in.size-1
      * @return Vector of basic points
      */
    template< typename T,
              typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
    static std::vector<Point> toPoints(const std::vector<T>& in, size_t ind0 = 0, int indn = -1){
        if(indn < 0 || indn >= in.size())
            indn = in.size()-1;
        ind0 = std::min(ind0, in.size());
        std::vector<Point> ret( indn-ind0+1 );
        for(size_t i = 0 ; i < ret.size(); ++i)
            ret[i] = in[i+ind0].point();
        return ret;
    }

    /**
      * @brief Convert a vector of points to a vectors of point-derived classes with default extra parameters
      * @param in Vector of basic points
      * @param ind0 Start index (inclusive)
      * @param indn Stop index (inclusive). if <0 || >=in.size -> indn = in.size-1
      * @return Vector of point-derived classes
      */
    template< typename T,
              typename = typename std::enable_if< std::is_base_of<Point, T>::value, void >::type >
    static std::vector<T> toDerivedPoints(const std::vector<Point>& in, size_t ind0 = 0, int indn = -1){
        if(indn < 0 || indn >= in.size())
            indn = in.size()-1;
        ind0 = std::min(ind0, in.size());
        std::vector<T> ret( indn-ind0+1 );
        for(size_t i = 0 ; i < ret.size(); ++i)
           ret[i].point() = in[i+ind0];
        return ret;
    }


    /**
      * @brief Get a reference to the point (usefull for derived clases)
      * @return Reference to the point
      */
    inline const Point& point() const { return *( this ); }

    /**
      * @brief Get a reference to the point (usefull for derived clases)
      * @return Reference to the point
      */
    inline Point& point() { return *( this ); }
};

/**
  * @brief operator<< to add (print) a point to an output stream
  * @note Only the x- and y- coordinates are included
  * @param ostr Output stream
  * @param pt Point to be added/printed
  * @return Updated output stream
  */
inline std::ostream& operator<< (std::ostream &ostr, const Point& pt) {
    ostr <<  std::setprecision(3) << "(" << pt.x << ", " << pt.y << ")";
    return ostr;
}

using PointVec = std::vector<Point>;
using PointVecVec = std::vector<std::vector<Point>>;

/**
  * @brief Scalar multiplication
  *
  * The operation is done for each of the axis (x, y and z), e.g. result.x = pt.x * s
  * @param pt Point to be multiplied
  * @param s Scalar operand (multiplier/factor)
  * @return Resulting point
  */
inline Point operator* (const Point& pt, double scale) {
    Point result;
    result.x = pt.x * scale;
    result.y = pt.y * scale;
    result.z = pt.z * scale;
    return result;
}


}


#endif
