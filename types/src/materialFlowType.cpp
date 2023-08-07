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
 
#include "arolib/types/materialFlowType.hpp"

namespace arolib {

MaterialFlowType intToMaterialFlowType(int value){
    if(value == INPUT_MATERIAL_FLOW)
        return INPUT_MATERIAL_FLOW;
    else if(value == OUTPUT_MATERIAL_FLOW)
        return OUTPUT_MATERIAL_FLOW;
    else if(value == NEUTRAL_MATERIAL_FLOW)
        return NEUTRAL_MATERIAL_FLOW;

    throw std::invalid_argument( "The given value does not correspond to any MaterialFlowType" );
}


}
