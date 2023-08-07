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
      * @brief Copy constructor
      */
    Pose2D(const Pose2D&) = default;

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

    /**
      * @brief Check if 2 poses are equal (with a very small tolerance)
      *
      * @param p Other pose to be compared
      * @return True if equal.
      */
    bool operator==(const Pose2D &other) const;

    /**
      * @brief Check if two poses are diferent (with a very small tolerance)
      *
      * @param p Other pose to be compared
      * @return True if differetnt.
      */
    bool operator!=(const Pose2D &other) const;

    /**
      * @brief operator < (used for sorting --> no significant 'quantitative/qualitative' meaning)
      *
      * The order of checking is: 1) point; 2) angle
      * @param p Other pose to be compared
      * @return True if the (local) pose has lower attributes than p (following the order in the method description)
      */
    bool operator<(const Pose2D &other) const;

    /**
      * @brief Struct to get hash value
      */
    struct KeyHash
    {
      std::size_t operator()(const Pose2D &p) const;
      static std::size_t get(const Pose2D &p, std::size_t seed = 0);
    };


public:
    double angle = 0;
};

inline bool operator==(const Pose2D& lhs, const Pose2D& rhs) {
  return (lhs.angle == rhs.angle) && (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.z == rhs.z);
}

}


#endif //_AROLIB_POSE2D_H_
