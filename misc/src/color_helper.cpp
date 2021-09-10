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
 
#include "arolib/misc/color_helper.hpp"

namespace arolib{

Range2RGBColorType intToRange2RGBColorType(int value)
{

    if(value == Range2RGBColorType::COLOR_GRAY)
        return Range2RGBColorType::COLOR_GRAY;
    else if(value == Range2RGBColorType::COLOR_HEAT)
        return Range2RGBColorType::COLOR_HEAT;
    else if(value == Range2RGBColorType::COLOR_GREEN2RED)
        return Range2RGBColorType::COLOR_GREEN2RED;
    else if(value == Range2RGBColorType::COLOR_RED2GREEN)
        return Range2RGBColorType::COLOR_RED2GREEN;

    throw std::invalid_argument( "The given value does not correspond to any Range2RGBColorType" );
}

}
