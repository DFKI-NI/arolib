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
 
#ifndef _AROLIB_FIELD_H_
#define _AROLIB_FIELD_H_
#include <vector>
#include <string>
#include "subfield.hpp"
#include "polygon.hpp"
namespace arolib {

/**
 * @brief Field class
 */
class Field {
public:

  Polygon outer_boundary; /**< Outer boundery of the whole field */
  std::vector<Subfield> subfields; /**< Subfields (German: Schläge) of the field */
  int id; /**< Unique field id */
  std::string name; /**< Field name */
  std::string filename; /**< Absolute path of the file (almost obsolete) */
  std::vector<Linestring> external_roads; /**< Set of external roads use to drive outside the field */

  /**
   * @brief Clear/reset all the attributes of the field
   */
  void clear();
};

inline bool operator==(const Field& lhs, const Field& rhs) {
  return (lhs.outer_boundary == rhs.outer_boundary) &&\
         (lhs.subfields == rhs.subfields) &&\
        (lhs.external_roads == rhs.external_roads);
}

}

#endif
