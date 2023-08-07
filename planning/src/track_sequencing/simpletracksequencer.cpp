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
 
#include "arolib/planning/track_sequencing/simpletracksequencer.hpp"

#include <stdexcept>
#include <algorithm>

namespace arolib{

SimpleTrackSequencer::SimpleTrackSequencer(LogLevel logLevel) :
    ITrackSequencer(__FUNCTION__, logLevel)
{

}

AroResp SimpleTrackSequencer::computeSequences(const Subfield &subfield,
                                               const std::vector<Machine> &machines,
                                               const TrackSequencerSettings& settings,
                                               std::map<MachineId_t, std::vector<TrackInfo> > &sequences,
                                               const Pose2D* initRefPose,
                                               const std::set<size_t> &excludeTrackIndexes)
{
    sequences.clear();

    if(machines.empty())
        return AroResp(1, "No machines given");

    if(subfield.tracks.empty())
        return AroResp(1, "No tracks given");

    for(auto& track : subfield.tracks){
        if(track.points.empty())
            return AroResp(1, "One or more tracks have no points");
    }

    std::vector<size_t> trackInds;
    for(size_t i = 0 ; i < subfield.tracks.size() ; ++i){
        if(excludeTrackIndexes.find(i) == excludeTrackIndexes.end())
            trackInds.push_back(i);
    }
    if(trackInds.empty())
        return AroResp(1, "No tracks left after excludeding given tracks");

    bool tracksInReverse, trackPointsInReverse;
    areTracksInReverse(subfield, initRefPose, tracksInReverse, trackPointsInReverse);
    int indMachine = 0;
    for(size_t i = 0; i < trackInds.size(); ++i) {
        auto machineId = machines.at(indMachine).id;
        indMachine = get_index_from_cyclic_container(machines, indMachine+1);
        auto& seq = sequences[machineId];
        TrackInfo info;
        if(tracksInReverse)
            info.trackIndex = r_at(trackInds, i);
        else
            info.trackIndex = trackInds.at(i);
        if(seq.empty())
            info.trackPointsDirection = trackPointsInReverse ? TrackPointsDirection::REVERSE : TrackPointsDirection::FORWARD;
        else
            info.trackPointsDirection = TrackPointsDirection::UNDEF;
        seq.push_back(info);
    }
    return AroResp::ok();

}


void SimpleTrackSequencer::areTracksInReverse(const Subfield &subfield, const Point *initRefPoint, bool & tracksInReverse, bool & trackPointsInReverse)
{
    tracksInReverse = false;
    trackPointsInReverse = false;
    if(!initRefPoint)
        return;
    if(!initRefPoint->isValid())
        return;

    double dist0_0 = geometry::calc_dist(subfield.tracks.front().points.front(), *initRefPoint);
    double dist0_n = geometry::calc_dist(subfield.tracks.front().points.back(), *initRefPoint);
    double distn_0 = geometry::calc_dist(subfield.tracks.back().points.front(), *initRefPoint);
    double distn_n = geometry::calc_dist(subfield.tracks.back().points.back(), *initRefPoint);

    if( std::min(dist0_0, dist0_n) <= std::min(distn_0, distn_n) ){
        tracksInReverse = false;
        trackPointsInReverse = dist0_0 > dist0_n;
    }
    else{
        tracksInReverse = true;
        trackPointsInReverse = distn_0 > distn_n;
    }
}

} // namespace arolib
