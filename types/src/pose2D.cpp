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
 

#include "arolib/types/pose2D.hpp"

namespace arolib {

Pose2D::Pose2D(double _x, double _y, double _angle, double _z):
    Point(_x, _y, _z),
    angle(_angle)
{

}

Pose2D::Pose2D(const Point &point, double _angle):
    Point(point),
    angle(_angle)
{

}

bool Pose2D::operator==(const Pose2D &other) const
{
    static const double epsilon = 0.001;
    return ( this->point() == other.point() && fabs(angle-other.angle) < epsilon );
}

bool Pose2D::operator!=(const Pose2D &other) const
{
    return !(operator ==(other));
}

bool Pose2D::operator<(const Pose2D &other) const
{
    static const double epsilon = 0.001;
    if (this->point() != other.point())
        return this->point() < other.point();
    else if (fabs(angle - other.angle) > epsilon)
        return angle < other.angle;
    return false;
}

std::size_t Pose2D::KeyHash::operator()(const Pose2D &p) const
{
    return get(p, 0);
}

std::size_t Pose2D::KeyHash::get(const Pose2D &p, std::size_t seed)
{
    seed = Point::KeyHash::get( p.point(), seed );
    boost::hash_combine(seed, p.angle);
    return seed;
}


}


