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
 

#include "arolib/planning/transit_restrictions.hpp"

namespace arolib{

TransitRestriction intToTransitRestriction(int value)
{
    if(value == TransitRestriction::NO_TRANSIT_RESTRICTION)
        return TransitRestriction::NO_TRANSIT_RESTRICTION;
    else if(value == TransitRestriction::TRANSIT_ONLY_OVER_WORKED_AREAS)
        return TransitRestriction::TRANSIT_ONLY_OVER_WORKED_AREAS;
    else if(value == TransitRestriction::TRANSIT_ONLY_OVER_UNWORKED_AREAS)
        return TransitRestriction::TRANSIT_ONLY_OVER_UNWORKED_AREAS;

    throw std::invalid_argument( "The given value does not correspond to any TransitRestriction" );
}

}

