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
 
#ifndef _AROLIB_MACHINEDYNAMICINFO_H_
#define _AROLIB_MACHINEDYNAMICINFO_H_

#include <vector>
#include "point.hpp"
#include "machine.hpp"

namespace arolib {

/**
  * @brief Class holding the dynamic properties of a machine
  */
class MachineDynamicInfo{

public:
    /**
      * @brief Constructor
      */
    explicit MachineDynamicInfo() = default;

    Point position = Point(0,0); /**< Position/location of the machine */
    double theta = 0; /**< Orientation angle of the machine w.r.t. the x-axis [Rad] */
    double bunkerMass = 0; /**< Yield mass in the bunker [kg] */
    double bunkerVolume = 0; /**< Yield volume in the bunker [m³] */
    double timestamp = 0; /**< Current timestamp of the machine */

    /**
      * @brief Check if the information is valid
      * @return True if valid
      */
    bool isValid() const;

    /**
      * @brief Check if the information is valid
      * @param mdi Map containing machine dynamic info
      * @return True if valid
      */
    static bool isValid(const std::map<MachineId_t, MachineDynamicInfo>& mdi);

    /**
      * @brief Check if the information is valid
      * @param mdi Vector containing machine dynamic info
      * @return True if valid
      */
    static bool isValid(const std::vector<MachineDynamicInfo>& mdi);

};

inline bool operator==(const MachineDynamicInfo& lhs, const MachineDynamicInfo& rhs) {
    const double eps = 1e-9;
  return (lhs.position == rhs.position)
          && std::fabs(lhs.theta - rhs.theta) < eps
          && std::fabs(lhs.bunkerMass - rhs.bunkerMass)  < eps
          && std::fabs(lhs.bunkerVolume - rhs.bunkerVolume) < eps
          && std::fabs(lhs.timestamp - rhs.timestamp) < eps;
}

}

#endif
