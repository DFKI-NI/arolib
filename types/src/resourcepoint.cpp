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
 
#include "arolib/types/resourcepoint.hpp"

namespace arolib {

const std::set<ResourcePoint::ResourceType> ResourcePoint::AllResourceTypes = { ResourceType::ResourceType_UNLOADING,
                                                                                ResourceType::ResourceType_LOADING,
                                                                                ResourceType::ResourceType_CHARGING };

ResourcePoint::ResourceType ResourcePoint::intToResourceType(int value)
{
    if(value == ResourceType_UNLOADING)
        return ResourceType_UNLOADING;
    else if(value == ResourceType_LOADING)
        return ResourceType_LOADING;
    else if(value == ResourceType_CHARGING)
        return ResourceType_CHARGING;

    throw std::invalid_argument( "The given value does not correspond to any ResourcePoint::ResourceType" );
}

ResourcePoint::ResourcePoint(double _x, double _y, double _z):
    Point(_x, _y, _z)
{

}

ResourcePoint::ResourcePoint(const Point &point):
    Point( point )
{

}

ResourcePoint::ResourcePoint(const Point &point,
                             const ResourcePointId_t &_id,
                             const std::set<ResourceType> &_resourceTypes,
                             const double &_defaultUnloadingTime,
                             const double &_defaultUnloadingTimePerKg,
                             const Polygon &_geometry):
    Point( point ),
    id(_id),
    resourceTypes(_resourceTypes),
    defaultUnloadingTime(_defaultUnloadingTime),
    defaultUnloadingTimePerKg(_defaultUnloadingTimePerKg),
    geometry(_geometry)
{

}

std::vector<ResourcePoint> ResourcePoint::fromPoints(const std::vector<Point> points)
{
    std::vector<ResourcePoint> ret( points.size() );
    for(size_t i = 0 ; i < points.size(); ++i)
        ret[i] = ResourcePoint(points[i]);
    return ret;
}


}
