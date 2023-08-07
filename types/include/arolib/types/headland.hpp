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
 
#ifndef _AROLIB_HEADLAND_H_
#define _AROLIB_HEADLAND_H_
#include <vector>

#include "polygon.hpp"
#include "point.hpp"
#include "resourcepoint.hpp"
#include "fieldaccesspoint.hpp"
#include "linestring.hpp"
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
 * @brief Partial headland class
 */
class PartialHeadland {
public:
    static const int NoConnectingHeadlandId;

    int id; /**< Headland id */
    Polygon boundary; /**< Headland boundary*/
    std::vector<Track> tracks; /**< Tracks*/
    std::pair<int, int> connectingHeadlandIds = std::make_pair(NoConnectingHeadlandId, NoConnectingHeadlandId); /**< If the partial headland is a connecting headland, these are the ids of the headlands being connected*/

    /**
     * @brief Check if the headland is a connecting headland based on the connectingHeadlandIds
     * @param twoSided if false, it will be enough to be connected from one side
     * @return True if the headland is a connecting headland
     */
    virtual bool isConnectingHeadland(bool twoSided = true) const;

    /**
     * @brief Get the headland connection sequence to connect two partial headlands
     * @param hls Headlands
     * @param ind_from Index of the first headland
     * @param ind_to Index of the last headland
     * @return Connection sequences (indexes)
     */
    static std::vector<std::vector<size_t>> getHeadlandConnectionSequences(const std::vector<PartialHeadland>& hls, size_t ind_from, size_t ind_to);
};

/**
 * @brief Headlands class
 */
class Headlands {

public:
    CompleteHeadland complete; /**< Complete/surrounding headland */
    std::vector<PartialHeadland> partial; /**< Partial headlands */

    /**
     * @brief Check if a complete headland exits
     * @return True if a complete headland exits
     */
    virtual bool hasCompleteHeadland() const;

    /**
     * @brief Clear headlands geometries
     */
    virtual void clear();
};

inline bool operator==(const CompleteHeadland& lhs, const CompleteHeadland& rhs) {
  return (lhs.middle_track == rhs.middle_track) && (lhs.tracks == rhs.tracks) && (lhs.boundaries == rhs.boundaries);
}

inline bool operator==(const PartialHeadland& lhs, const PartialHeadland& rhs) {
  return (lhs.boundary == rhs.boundary) && (lhs.tracks == rhs.tracks);
}

inline bool operator==(const Headlands& lhs, const Headlands& rhs) {
  return (lhs.complete == rhs.complete) && (lhs.partial == rhs.partial);
}

}


#endif //_AROLIB_HEADLAND_H_
