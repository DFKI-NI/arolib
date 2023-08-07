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
 
#ifndef _AROLIB_MATERIALFLOWTYPE_HPP_
#define _AROLIB_MATERIALFLOWTYPE_HPP_

#include <stdexcept>

namespace arolib {

/**
 * @brief Types of operations
 */
enum MaterialFlowType {
    INPUT_MATERIAL_FLOW,
    OUTPUT_MATERIAL_FLOW,
    NEUTRAL_MATERIAL_FLOW
};

/**
  * @brief Get the MaterialFlowType (enum) from its int value
  * @param value Int value
  * @return Coresponding MaterialFlowType
  */
MaterialFlowType intToMaterialFlowType(int value);

}


#endif //_AROLIB_MATERIALFLOWTYPE_HPP_
