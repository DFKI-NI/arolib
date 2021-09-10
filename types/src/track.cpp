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
 
#include "arolib/types/track.hpp"

namespace arolib {

Track::TrackType Track::intToTrackType(int value){
    if(value == TrackType::MAIN_IF)
        return TrackType::MAIN_IF;
    if(value == TrackType::SEC_IF)
        return TrackType::SEC_IF;
    if(value == TrackType::MAIN_HL)
        return TrackType::MAIN_HL;
    if(value == TrackType::SEC_HL)
        return TrackType::SEC_HL;
    throw std::invalid_argument( "The given value does not correspond to any Track::TrackType" );
}

bool Track::isInfieldTrack() const
{
    return type == MAIN_IF || type == SEC_IF;
    //return id >= 0 && id < DeltaHLTrackId;
}

bool Track::isInfieldTrack(int trackId)
{
    return trackId >= 0 && trackId < DeltaHLTrackId;
}

bool Track::isHeadlandTrack() const
{
    return type == MAIN_HL || type == SEC_HL;
    //return id >= DeltaHLTrackId;
}

bool Track::isHeadlandTrack(int trackId)
{
    return trackId >= DeltaHLTrackId;
}

bool Track::isMainTrack() const
{
    return type == MAIN_IF || type == MAIN_HL;
}

}
