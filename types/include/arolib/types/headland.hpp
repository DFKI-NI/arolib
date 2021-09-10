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
 
#ifndef _AROLIB_HEADLAND_H_
#define _AROLIB_HEADLAND_H_
#include <vector>

#include "polygon.hpp"
#include "point.hpp"
#include "resourcepoint.hpp"
#include "fieldaccesspoint.hpp"
#include "linestring.hpp"
#include "headlandpoint.hpp"
#include "track.hpp"

namespace arolib {

/**
 * @brief Complete headland class (headland surrounding subfield)
 */
class CompleteHeadland {
public:
    double headlandWidth = -1; /**< Headland width */
    Polygon middle_track; /**< Central track in the headland (might not correspond to any headland track).*/
    std::vector<Track> tracks; /**< Tracks */
    std::pair<Polygon, Polygon> boundaries; /**< Outer and inner boundaries of the headland. */
};

/**
 * @brief Headlands class
 */
class Headlands {

public:
    CompleteHeadland complete; /**< Complete/surrounding headland */

    /**
     * @brief Check if a complete headland exits
     * @return True if a complete headland exits
     */
    virtual bool hasCompleteHeadland() const;
};

}


#endif //_AROLIB_HEADLAND_H_
