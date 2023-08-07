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
 
#include "arolib/types/point.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include "arolib/types/units.hpp"

namespace arolib {

std::size_t Point::KeyHash::operator()(const Point &p) const{
    return get(p, 0);
}
std::size_t Point::KeyHash::get(const Point &p, std::size_t seed){
    Point pointTmp( std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest() );
    const Point *pPoint = p.isValid() ? &p : &pointTmp;

    boost::hash_combine(seed, pPoint->x);
    boost::hash_combine(seed, pPoint->y);
    boost::hash_combine(seed, pPoint->z);
    return seed;
}

Point::Point():
    x(0),
    y(0),
    z(0){

}

Point::Point(double _x, double _y, double _z):
    x(_x),
    y(_y),
    z(_z){

}

Point Point::operator+(const Point &p) const
{
    Point res;
    res.x = x + p.x;
    res.y = y + p.y;
    res.z = z + p.z;
    return res;
}

Point Point::operator/(float s) const
{
    Point res;
    res.x = x/s;
    res.y = y/s;
    res.z = z/s;
    return res;
}

Point Point::operator*(const double s)
{
    return Point( x*s, y*s, z*s );
}

Point Point::operator-(const Point &p) const
{
    Point res;
    res.x = x - p.x;
    res.y = y - p.y;
    res.z = z - p.z;
    return res;
}

Point& Point::operator-=(const Point &p)
{
    x -= p.x;
    y -= p.y;
    z -= p.z;
    return *this;
}

Point& Point::operator+=(const Point &p)
{
    x += p.x;
    y += p.y;
    z += p.z;
    return *this;
}

Point& Point::operator*=(const double s)
{
    x *= s;
    y *= s;
    z *= s;
    return *this;
}


Point& Point::operator/=(const double s)
{
    x /= s;
    y /= s;
    z /= s;
    return *this;
}

bool Point::operator==(const Point& p) const
{
    if(!this->isValid() || !p.isValid())
        return false;
    static const double epsilon = 0.00001;
    return ( fabs(x-p.x) < epsilon && fabs(y-p.y) < epsilon );
}

bool Point::operator!=(const Point& p) const
{
    return (!(*this == p));
}

bool Point::operator <(const Point& p) const
{
    static const double epsilon = 0.00001;
    if (fabs(x-p.x) > epsilon)
        return x < p.x;
    else if (fabs(y - p.y) > epsilon)
        return y < p.y;
    else if (fabs(z - p.z) > epsilon)
        return z < p.z;
    return false;
}

double Point::dist(const Point &p0, const Point &p1){
    return sqrt( pow(p1.x - p0.x, 2) + pow(p1.y - p0.y, 2) );
}

double Point::bearing(const Point &p0_geo, const Point &p1_geo)
{
    double teta1 = p0_geo.y * M_PI / 180.0;
    double teta2 = p1_geo.y * M_PI / 180.0;
    double delta2 = (p1_geo.x-p0_geo.x) * M_PI / 180.0;
    double y = sin(delta2) * cos(teta2);
    double x = cos(teta1)*sin(teta2) - sin(teta1)*cos(teta2)*cos(delta2);
    double brng = atan2(y,x);
    brng = rad2deg( brng ) ;// radians to degrees
    brng = std::fmod(brng + 360.0, 360.0);

    return brng;
}



std::string Point::toString(int precision, bool incZ) const
{
    std::stringstream stream;
    if(precision >= 0)
        stream << std::setprecision(precision);
    stream << "(" << x << " , " << y ;
    if(incZ)
        stream << " , " << z;
    stream << ")";
    return stream.str();
}

std::string Point::toStringCSV(char sep, int precision, bool incZ) const
{
    std::stringstream stream;
    if(precision >= 0)
        stream << std::setprecision(precision);
    stream << x << sep << y ;
    if(incZ)
        stream << sep << z;
    return stream.str();
}

std::string Point::toStringCSV(const std::vector<Point> &points, char sep, int precision, bool incZ)
{
    std::string sout;
    for(const auto& p : points)
        sout += ( p.toStringCSV(sep, precision, incZ) + "\n" );
    return sout;
}

std::string Point::toStringCSV(const std::vector<std::vector<Point> > &v_points, char sep, int precision, bool incZ)
{
    std::vector<const std::vector<Point> *> v_points_p(v_points.size());
    for(size_t i = 0 ; i < v_points_p.size() ; ++i)
        v_points_p[i] = &(v_points[i]);
    return toStringCSV(v_points_p, sep, precision, incZ);
}

std::string Point::toStringCSV(const std::vector<const std::vector<Point> *> &v_points, char sep, int precision, bool incZ)
{
    std::string sout;
    size_t maxSize = 0;
    for(const auto& v : v_points)
        maxSize = std::max(maxSize, v->size());

    for(size_t i = 0 ; i < maxSize ; ++i){
        for(const auto& v : v_points){
            if(i < v->size())
                sout += ( v->at(i).toStringCSV(sep, precision, incZ) + ";;" );
            else
                sout += (incZ ? ";;;;" : ";;;");
        }
        sout += "\n";
    }
    return sout;
}

void Point::setInvalid()
{
    x = std::nan("1");

    y = std::nan("1");
}

bool Point::isValid() const
{
    return !std::isnan(x) && !std::isnan(y);
}

Point Point::invalidPoint()
{
    return Point(std::nan("1"), std::nan("1"));
}

}
