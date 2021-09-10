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
 
#ifndef ARO_FUNCTIONS_HPP
#define ARO_FUNCTIONS_HPP

#include <math.h>
#include <algorithm>

#include "arolib/types/units.hpp"

namespace arolib{

const double defYieldmassPerArea_siloMaize__t_ha = 50; /**< Default yield mass per area ratio for silo maize harvesting [t/ha] */
const double defYieldmassPerArea_siloMaize__kg_m2 = t_ha2Kg_sqrm(defYieldmassPerArea_siloMaize__t_ha);  /**< Default yield mass per area ratio for silo maize harvesting [Kg/m²] */

const double defYieldmassPerArea_winterWheat__t_ha = 11; /**< Default yield mass per area ratio for winter wheat harvesting [t/ha] */
const double defYieldmassPerArea_winterWheat__kg_m2 = t_ha2Kg_sqrm(defYieldmassPerArea_winterWheat__t_ha);  /**< Default yield mass per area ratio for winter wheat harvesting [Kg/m²] */

const double defYieldmassPerArea_sugarBeet__t_ha = 90; /**< Default yield mass per area ratio for sugar beet harvesting [t/ha] */
const double defYieldmassPerArea_sugarBeet__kg_m2 = t_ha2Kg_sqrm(defYieldmassPerArea_sugarBeet__t_ha); /**< Default yield mass per area ratio for sugar beet harvesting [Kg/m²] */

const double defDrymatter_siloMaize = 35; /**< Default drymatter % for silo maize harvesting [%] */
const double machine_power_default = 400; /**< Default machine power [kW] */

}

#endif // ARO_FUNCTIONS_HPP
