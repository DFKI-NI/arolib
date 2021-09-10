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
 
#ifndef _AROLIB_POSE2D_H_
#define _AROLIB_POSE2D_H_

#include "arolib/types/point.hpp"

namespace arolib {

/**
  * @brief Pose in 2D (location + angle direction)
  */
class Pose2D : public Point {
public:

    /**
      * @brief Constructor
      */
    explicit Pose2D() = default;

    /**
      * @brief Constructor with location arguments
      * @param _x x-coordinate
      * @param _y y-coordinate
      * @param _angle angle
      * @param _z z-coordinate
      */
    explicit Pose2D(double _x, double _y, double _angle, double _z=0);

    /**
      * @brief Constructor with location arguments
      * @param point Location
      * @param _angle angle
      */
    explicit Pose2D( const Point & point, double _angle = 0);


public:
    double angle = 0;
};

}


#endif //_AROLIB_POSE2D_H_
