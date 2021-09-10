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
 
#include "arolib/types/fieldaccesspoint.hpp"

namespace arolib {

FieldAccessPoint::AccessPointType FieldAccessPoint::intToAccessPointType(int value){
    if(value == AccessPointType::AP_ENTRY_EXIT)
        return AccessPointType::AP_ENTRY_EXIT;
    else if(value == AccessPointType::AP_ENTRY_ONLY)
        return AccessPointType::AP_ENTRY_ONLY;
    else if(value == AccessPointType::AP_EXIT_ONLY)
        return AccessPointType::AP_EXIT_ONLY;

    throw std::invalid_argument( "The given value does not correspond to any FieldAccessPoint::AccessPointType" );
}

FieldAccessPoint::FieldAccessPoint():
    Point(),
    id(-1),
    accessType(AccessPointType::AP_ENTRY_EXIT)
{

}

FieldAccessPoint::FieldAccessPoint(double _x, double _y, double _z):
    Point(_x, _y, _z),
    id(-1),
    accessType(AccessPointType::AP_ENTRY_EXIT)
{

}

FieldAccessPoint::FieldAccessPoint(const Point &point):
    Point( point ),
    id(-1),
    accessType(AccessPointType::AP_ENTRY_EXIT)
{

}

FieldAccessPoint::FieldAccessPoint(const Point &point,
                                   const FieldAccessPointId_t &_id,
                                   const FieldAccessPoint::AccessPointType &_accessPointType):
    Point( point ),
    id(_id),
    accessType(_accessPointType)
{

}

std::vector<FieldAccessPoint> FieldAccessPoint::fromPoints(const std::vector<Point> points)
{
    std::vector<FieldAccessPoint> ret( points.size() );
    for(size_t i = 0 ; i < points.size(); ++i)
        ret[i] = FieldAccessPoint(points[i]);
    return ret;

}


}
