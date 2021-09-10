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
 
#ifndef _AROLIB_MACHINEDYNAMICINFO_H_
#define _AROLIB_MACHINEDYNAMICINFO_H_

#include <vector>
#include "point.hpp"

namespace arolib {

/**
  * @brief Class holding the dynamic properties of a machine
  */
class MachineDynamicInfo{

public:
    /**
      * @brief Constructor
      */
    explicit MachineDynamicInfo():
        position( Point(0,0) ),
        theta(0),
        bunkerMass(0),
        bunkerVolume(0){}

    Point position; /**< Position/location of the machine */
    double theta; /**< Orientation angle of the machine w.r.t. the x-axis [Rad] */
    double bunkerMass; /**< Yield mass in the bunker [kg] */
    double bunkerVolume; /**< Yield volume in the bunker [m³] */

};


}

#endif
