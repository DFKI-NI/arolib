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
 
#ifndef _AROLIB_RESOURCEPOINTSTATE_H_
#define _AROLIB_RESOURCEPOINTSTATE_H_

namespace arolib {

/**
  * @brief Class holding the state (dynamic properties) of a resource point
  */
class ResourcePointState{

public:
    /**
      * @brief Constructor
      */
    explicit ResourcePointState() = default;

    double capacityMass = std::nan("1"); /**< Current mass capacity [kg] (if NAN -> unknown) */
    double capacityVolume = std::nan("1"); /**< Current volume capacity [m³] (if NAN -> unknown) */
    double timestamp = 0; /**< Timestamp of when the resource point will become available after planning started (not implemented) */
    bool enabled = true; /**< States whether the resource point is enabled for the current planning */


    inline bool operator==(const ResourcePointState& other) {
        const double eps = 1e-9;
      return (enabled == other.enabled)
              && std::fabs(capacityMass - other.capacityMass) < eps
              && std::fabs(capacityVolume - other.capacityVolume)  < eps
              && std::fabs(timestamp - other.timestamp) < eps;
    }

};


}

#endif //_AROLIB_RESOURCEPOINTSTATE_H_
