/*
 * Copyright 2021-2025 DFKI GmbH
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
 
#ifndef _AROLIB_RESOURCEPOINT_H_
#define _AROLIB_RESOURCEPOINT_H_

#include <set>

#include "arolib/types/polygon.hpp"

namespace arolib {

typedef int ResourcePointId_t; /**< Type of the resource point unique id */

/**
  * @brief Resource point class
  */
class ResourcePoint : public Point {
public:

    /**
      * @brief Type of resource point
      */
    enum ResourceType{
        ResourceType_UNLOADING,
        ResourceType_LOADING,
        ResourceType_CHARGING
    };

    /**
      * @brief Get the ResourceType (enum) from its int value
      */
    static ResourceType intToResourceType(int value);

    /**
      * @brief Get the short string description of the resourceType (enum)
      */
    static std::string resourceTypeToShortStr(ResourceType value);

    static const std::set<ResourceType> AllResourceTypes;/**< Set containing all resource point types */

    /**
      * @brief Constructor
      */
    explicit ResourcePoint() = default;

    /**
      * @brief Copy constructor
      */
    ResourcePoint(const ResourcePoint&) = default;

    /**
      * @brief Constructor with location arguments
      * @param _x x-coordinate
      * @param _y y-coordinate
      * @param _z z-coordinate
      */
    explicit ResourcePoint(double _x, double _y, double _z=0);

    /**
      * @brief Constructor with location arguments
      * @param point Location
      */
    explicit ResourcePoint( const Point & point);

    /**
      * @brief Constructor with arguments
      * @param point Location
      * @param _id Id
      * @param _defaultUnloadingTime Default time [s] a machine needs to unload all its bunker
      * @param _defaultUnloadingTimePerKg Default time a machine needs to unload a Kg of yield [s/Kg]
      * @param _massCapacity Mass capacity [Kg] (if <0 -> inf)
      * @param Polygon Geometry of the resource/silo structure (for future use). Disregarded if empty-polygon.
      */
    explicit ResourcePoint(const Point & point,
                           const ResourcePointId_t &_id,
                           const std::set<ResourceType>& _resourceTypes = AllResourceTypes,
                           float _defaultUnloadingTime = 0,
                           float _defaultUnloadingTimePerKg = 0,
                           float _massCapacity = -1,
                           float _volumeCapacity = -1,
                           const Polygon& _geometry = Polygon() );

    /**
      * @brief Create a vector of resource points from a vector of basic points (with default access point properties)
      * @param points vector of basic points
      * @return Vector of resource points
      */
    static std::vector<ResourcePoint> fromPoints(const std::vector<Point> points);

public:
    ResourcePointId_t id = -1; /**< Resource point unique id */
    std::set<ResourceType> resourceTypes = AllResourceTypes; /**< Resource type(s) */
    float defaultUnloadingTime = 0; /**< Default time [s] a machine needs to unload all its bunker */
    float defaultUnloadingTimePerKg = 0; /**< Default time a machine needs to unload a Kg of yield [s/Kg] */
    float massCapacity = -1; /**< Mass capacity [kg] */
    float volumeCapacity = -1; /**< Volume capacity [m³] */
    Polygon geometry; /**< Geometry of the resource/silo structure (for future use) */
};


inline std::ostream& operator<< (std::ostream &out, const ResourcePoint& data) {
  out << "ResourcePoint {id: " << data.id << " , pt: " << data.point() << "}";
  return out;
}

bool operator==(const ResourcePoint& lhs, const ResourcePoint& rhs);

}

#endif //_AROLIB_RESOURCEPOINT_H_
