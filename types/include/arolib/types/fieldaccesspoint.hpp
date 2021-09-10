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
 
#ifndef _AROLIB_FIELDACCESSPOINT_H_
#define _AROLIB_FIELDACCESSPOINT_H_

#include <iostream>
#include <string>
#include <vector>
#include <cmath>


#include "arolib/types/point.hpp"

namespace arolib {

typedef int FieldAccessPointId_t; /**< Type of the access point unique id */

/**
  * @brief Extended point for field access (entry/exit)
  */
class FieldAccessPoint : public Point {
public:
    /**
      * @brief Type of field access
      */
    enum AccessPointType{
        AP_ENTRY_EXIT, /**< Access point used only to exit the field */
        AP_ENTRY_ONLY, /**< Access point used only to enter the field */
        AP_EXIT_ONLY /**< Access point used enter and exit the field */
    };
    /**
      * @brief Get the AccessPointType (enum) from its int value
      * @param value int value
      * @return AccessPointType
      */
    static AccessPointType intToAccessPointType(int value);

    /**
      * @brief Constructor
      */
    explicit FieldAccessPoint();

    /**
      * @brief Constructor with location arguments
      * @param _x x-coordinate
      * @param _y y-coordinate
      * @param _z z-coordinate
      */
    explicit FieldAccessPoint(double _x, double _y, double _z=0);

    /**
      * @brief Constructor with location arguments
      * @param point Location
      */
    explicit FieldAccessPoint( const Point & point);

    /**
      * @brief Constructor with arguments
      * @param point Location
      * @param _id Id
      * @param _accessPointType Access type
      */
    explicit FieldAccessPoint( const Point & point,
                               const FieldAccessPointId_t & _id,
                               const AccessPointType & _accessPointType = AccessPointType::AP_ENTRY_EXIT );

    /**
      * @brief Create a vector of access points from a vector of basic points (with default access point properties)
      * @param points vector of basic points
      * @return Vector of access points
      */
    static std::vector<FieldAccessPoint> fromPoints(const std::vector<Point> points);

public:
    FieldAccessPointId_t id; /**< Access point unique id */
    AccessPointType accessType; /**< Access type */

};

}


#endif //_AROLIB_FIELDACCESSPOINT_H_
