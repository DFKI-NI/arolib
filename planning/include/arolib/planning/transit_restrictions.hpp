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
 
#ifndef AROLIB_TRANSIT_RESTRICTIONS_HPP
#define AROLIB_TRANSIT_RESTRICTIONS_HPP


#include <stdexcept>

namespace arolib {

/**
 * @brief Types of operations
 */
enum TransitRestriction {
    NO_TRANSIT_RESTRICTION,
    TRANSIT_ONLY_OVER_WORKED_AREAS,
    TRANSIT_ONLY_OVER_UNWORKED_AREAS
};

/**
  * @brief Get the TransitRestriction (enum) from its int value
  * @param value Int value
  * @return Coresponding TransitRestriction
  */
TransitRestriction intToTransitRestriction(int value);

}

#endif // AROLIB_TRANSIT_RESTRICTIONS_HPP
