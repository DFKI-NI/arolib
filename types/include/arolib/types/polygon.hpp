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
 
#ifndef _AROLIB_POLYGON_H_
#define _AROLIB_POLYGON_H_

#include <iostream>
#include <vector>
#include "point.hpp"

namespace arolib {

/**
  * @brief Polygon class
  */
class Polygon {
public:
    std::vector<Point> points; /**< Perimeter points */
};


/**
  * @brief Polygon (with holes) class
  */
class PolygonWithHoles {
public:
    Polygon outer; /**< Outer perimeter */
    std::vector<Polygon> holes; /**< Holes */
};

inline std::ostream& operator<< (std::ostream &out, const arolib::Polygon& data) {
  out << "Polygon { ";
  for(const auto& point: data.points)
  { out << point << " "; }
  out << "}";
  return out;
}

inline bool operator==(const Polygon& lhs, const Polygon& rhs) {
  return (lhs.points == rhs.points);
}

}

#endif
