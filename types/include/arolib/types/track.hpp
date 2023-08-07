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
 
#ifndef _AROLIB_TRACK_H_
#define _AROLIB_TRACK_H_

#include "polygon.hpp"

namespace arolib {

/**
  * @brief Track class
  */
class Track  {
public:
    using TrackPoint_t = Point;

    /**
      * @brief Track types
      */
    enum TrackType{
        MAIN_IF,
        SEC_IF,
        MAIN_HL,
        SEC_HL
    };

    /**
      * @brief Get the TrackType (enum) from its int value
      */
    static TrackType intToTrackType(int value);

    int id; /**< track id . For infield, 0<=id<DeltaHLTrackId ; for headland, id>=DeltaHLTrackId*/
    TrackType type = MAIN_IF; /**< track type */
    std::vector<TrackPoint_t> points; /**< central track points */
    Polygon boundary; /**< boundary of the track */
    double width = -1; /**< track width */

    static const int DeltaHLTrackId = 1000; /**< The headland tracks will have unique ids starting from these values, to differenciate them from inner-field tracks */

    /**
      * @brief Check if the track is from the inner-field
      * @return True if the track is from the inner-field
      */
    bool isInfieldTrack() const;

    /**
      * @brief Check if a track id corresponds to a track of the inner-field
      * @param trackId Track id
      * @return True if the track id corresponds to a track of the inner-field
      */
    static bool isInfieldTrack(int trackId);

    /**
      * @brief Check if the track is from the headland
      * @return True if the track is from the headland
      */
    bool isHeadlandTrack() const;

    /**
      * @brief Check if a track id corresponds to a track of the headland
      * @param trackId Track id
      * @return True if the track id corresponds to a track of the headland
      */
    static bool isHeadlandTrack(int trackId);

    /**
      * @brief Check if the track is a main track
      * @return True if the track is a main track
      */
    bool isMainTrack() const;


    /**
      * @brief Print the points of a set of tracks in a string with CSV format
      * @param tracks Tracks to be printed
      * @param sep Character to be used as separator
      * @param precision Decimal precision (disregarded if < 0)
      * @param incZ If true, the z-coordinate is included; otherwise, only x and y are included
      * @return (CSV) String description of the set of points
      */
    static std::string toStringCSV(const std::vector<Track>& tracks, char sep = ';', int precision = 10, bool incZ = false);
};

inline bool operator==(const Track& lhs, const Track& rhs) {
  return (lhs.type == rhs.type) &&\
         (lhs.points == rhs.points) &&\
        (lhs.boundary == rhs.boundary);
}

}

#endif
