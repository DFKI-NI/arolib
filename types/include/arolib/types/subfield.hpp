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
 
#ifndef _AROLIB_SUBFIELD_H_
#define _AROLIB_SUBFIELD_H_
#include <vector>

#include "resourcepoint.hpp"
#include "fieldaccesspoint.hpp"
#include "headland.hpp"
#include "obstacle.hpp"

namespace arolib {

/**
 * @brief Subfield (German: Schlag) class
 */
class Subfield {

public:
    int id; /**< Unique subfield id */
    Polygon boundary_outer; /**< Subfield's outer boundary */
    Polygon boundary_inner; /**< Subfield's inner boundary */
    Headlands headlands; /**< Headlands */
    std::vector<Obstacle> obstacles; /**< Obstacles inside the outer boundary */
    std::vector<Track> tracks; /**< Inner-field tracks */
    std::vector<ResourcePoint> resource_points; /**< Resource points (Silos) where OLVs can download the yield */
    std::vector<FieldAccessPoint> access_points; /**< Field-access points where machines can enter/exit the field */

    std::vector<Linestring> reference_lines; /**< An recorded or predefined reference line that is used to create the inner-field tracks (aka AB-Contour) */
    Point reference_line_A = Point::invalidPoint(); /**< Start-point of reference line in A-B, or A+working_direction modus */
    Point reference_line_B = Point::invalidPoint(); /**< End-point of reference line in A-B modus */
    double working_direction; /**< Working direction in A+working_direction modus [rad] */

};

inline bool operator==(const Subfield& lhs, const Subfield& rhs) {
return (lhs.boundary_outer == rhs.boundary_outer) &&\
        (lhs.boundary_inner == rhs.boundary_inner) &&\
        (lhs.headlands == rhs.headlands) &&\
        (lhs.obstacles == rhs.obstacles) &&\
        (lhs.tracks == rhs.tracks) &&\
        (lhs.resource_points == rhs.resource_points) &&\
        (lhs.access_points == rhs.access_points) &&\
        (lhs.reference_lines == rhs.reference_lines) &&\
        (lhs.working_direction == rhs.working_direction);
}

}


#endif
