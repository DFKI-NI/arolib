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
 
#include "arolib/planning/track_sequencing/tracksequenceradjacentnext.hpp"

#include <stdexcept>
#include <algorithm>

namespace arolib{

TrackSequencerAdjacentNext::TrackSequencerAdjacentNext(LogLevel logLevel) :
    ITrackSequencer(__FUNCTION__, logLevel)
{
    m_useConnOverBoundaryAsReference = true;
}

AroResp TrackSequencerAdjacentNext::computeSequences(const Subfield &subfield,
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
        if(track.points.size() < 2)
            return AroResp(1, "One or more tracks have less than 2 points");
    }

    std::vector<size_t> trackInds;
    std::map<size_t, size_t> realToVecIndexMap;
    for(size_t i = 0 ; i < subfield.tracks.size() ; ++i){
        if(excludeTrackIndexes.find(i) != excludeTrackIndexes.end()
                || subfield.tracks.at(i).points.size() < 2)
            continue;
        realToVecIndexMap[i] = trackInds.size();
        trackInds.push_back(i);
    }
    if(trackInds.empty())
        return AroResp(1, "No tracks left after excludeding given tracks");


    std::vector<size_t> extremaTrackInds;
    if(settings.limitStartToExtremaTracks)
        extremaTrackInds = geometry::getInfieldExtremaTracksIndexes(subfield, excludeTrackIndexes);

    auto& trackIndsStart = ( extremaTrackInds.empty() ? trackInds : extremaTrackInds );


    auto nextTrackIndexes = getFirstTracksIndexes(subfield, initRefPose, trackIndsStart, machines.size());// holds indexes corresponding to trackInds, not the the track index in the tracks vectors
    if(nextTrackIndexes.empty())
        return AroResp(1, "Error obtaining the initial track(s)");


    if(&trackIndsStart != &trackInds){//search the indexes in trackInds corresponding to nextTrackIndexes which correspond to trackIndsStart
        for(size_t i = 0 ; i < nextTrackIndexes.size() ; ++i){
            auto& indInfo = nextTrackIndexes.at(i);
            int ind = -1;
            for(size_t j = 0 ; j < trackInds.size() ; ++j){
                if(trackInds.at(j) == extremaTrackInds.at(indInfo.first)){
                    ind = j;
                    break;
                }
            }
            if(ind < 0)
                return AroResp(1, "Error obtaining track indexes from extrema track indexes");
            indInfo = std::make_pair( ind, indInfo.second );
        }
    }


    std::set<size_t> assignedTrackIndexes; // corresponding to trackInds, not the the track index in the tracks vectors
    std::map<MachineId_t, Polygon> limitBoundaries;
    for(size_t i = 0 ; i < nextTrackIndexes.size() ; ++i){
        auto& indInfo = nextTrackIndexes.at(i);
        auto machine = machines.at(i);
        sequences[machine.id].push_back( ITrackSequencer::TrackInfo( trackInds.at( indInfo.first ), indInfo.second ) );
        assignedTrackIndexes.insert( indInfo.first );
        if(m_tracksConnector)
            limitBoundaries[machine.id] = m_tracksConnector->getExtendedLimitBoundary(subfield, std::max(0.0, machine.getTurningRadius()) );
        else
            limitBoundaries[machine.id] = getDefTracksConnector()->getExtendedLimitBoundary(subfield, std::max(0.0, machine.getTurningRadius()) );
    }

    std::map<size_t, std::set<size_t> > adjacentTracksList = geometry::getAdjacentTracksList(subfield.tracks, true);

    Polygon connBoundary;
    if(m_useConnOverBoundaryAsReference){
        if(subfield.boundary_inner.points.size() > 3)
            connBoundary = subfield.boundary_inner;
        else if(subfield.boundary_outer.points.size() > 3)
            connBoundary = subfield.boundary_outer;
    }

    int indMachine = 0;
    while(assignedTrackIndexes.size() < trackInds.size()){
        auto machine = machines.at(indMachine);
        size_t indPrevTrack = nextTrackIndexes.at(indMachine).first;
        bool prevTrackInReverse = nextTrackIndexes.at(indMachine).second == ITrackSequencer::REVERSE;

        std::pair<int, bool> nextTrackInfo = getNextAdjacentTrack(subfield,
                                                                  trackInds,
                                                                  realToVecIndexMap,
                                                                  assignedTrackIndexes,
                                                                  adjacentTracksList,
                                                                  machine,
                                                                  indPrevTrack,
                                                                  prevTrackInReverse,
                                                                  sequences[machine.id],
                                                                  limitBoundaries[machine.id],
                                                                  connBoundary,
                                                                  settings.useMachineTurningRad);

        if(nextTrackInfo.first < 0)//no adjacents remaining or not correctly reachable -> try with closest next
            nextTrackInfo = getNextTrack(subfield,
                                         trackInds,
                                         assignedTrackIndexes,
                                         machine,
                                         indPrevTrack,
                                         prevTrackInReverse,
                                         sequences[machine.id],
                                         limitBoundaries[machine.id],
                                         connBoundary,
                                         settings.useMachineTurningRad);


        if(nextTrackInfo.first < 0){
            logger().printDebug(__FUNCTION__, "IF tracks sequences so far...");
            for(auto& it : sequences){
                logger().printDebug("", "\t Machine id : " + std::to_string(it.first) + ":");
                for(TrackInfo& info : it.second)
                    logger().printDebug("", "\t\t" + std::to_string(info.trackIndex) + "("  + std::to_string(info.trackPointsDirection) + ")");
            }
            logger().printDebug(__FUNCTION__, "IF tracks sequences so far (end)");

            return AroResp(1, "Error obtaining connection to next track from track with index " + std::to_string( trackInds.at(indPrevTrack) ));
        }

        nextTrackIndexes.at(indMachine) = std::make_pair( (size_t)nextTrackInfo.first, nextTrackInfo.second ? ITrackSequencer::REVERSE : ITrackSequencer::FORWARD );
        assignedTrackIndexes.insert( nextTrackInfo.first );
        sequences[machine.id].push_back( ITrackSequencer::TrackInfo( trackInds.at( nextTrackIndexes.at(indMachine).first ), nextTrackIndexes.at(indMachine).second ) );

        indMachine = get_index_from_cyclic_container(nextTrackIndexes, indMachine+1);
    }

    {
        logger().printDebug(__FUNCTION__, "Resulting IF tracks sequences ...");
        for(auto& it : sequences){
            logger().printDebug("", "\t Machine id : " + std::to_string(it.first) + ":");
            for(TrackInfo& info : it.second)
                logger().printDebug("", "\t\t" + std::to_string(info.trackIndex) + "("  + std::to_string(info.trackPointsDirection) + ")");
        }
        logger().printDebug(__FUNCTION__, "Resulting IF tracks sequences (end)");
    }

    return AroResp::ok();

}

std::pair<int, bool> TrackSequencerAdjacentNext::getNextAdjacentTrack(const Subfield &subfield,
                                                                      const std::vector<size_t>& trackInds,
                                                                      const std::map<size_t, size_t>& realToVecIndexMap,
                                                                      const std::set<size_t>& assignedTrackIndexes,
                                                                      const std::map<size_t, std::set<size_t> >& adjacentTracksList,
                                                                      const Machine& machine,
                                                                      size_t indPrevTrack,
                                                                      bool prevTrackInReverse,
                                                                      const std::vector<TrackInfo>& sequence,
                                                                      const Polygon& limitBoundary,
                                                                      const Polygon &connBoundary,
                                                                      bool useMachineTurningRad){



    const auto& trackFrom = subfield.tracks.at( trackInds.at(indPrevTrack) );
    const auto& trackFromPts = trackFrom.points;

    std::map<int, std::vector<size_t>> adjacentTracks;// holds indexes corresponding to trackInds, not the the track index in the tracks vectors
    auto it_adjL = adjacentTracksList.find(indPrevTrack);
    if(it_adjL != adjacentTracksList.end()){
        for(auto ind : it_adjL->second){
            auto it_ind = realToVecIndexMap.find(ind);
            if(it_ind == realToVecIndexMap.end()
                    || assignedTrackIndexes.find(ind) != assignedTrackIndexes.end())
                continue;
            const size_t& vInd = it_ind->second;

            //sort adjacent tracks based on track directions
            if(sequence.size() > 1){
                //check the "directions" between the previous visited tracks and potential next tracks
                Point directionPrev = getSequenceAverageDirection(subfield, sequence);
                auto& nextTrackPts = subfield.tracks.at( ind ).points;
                Point direction = geometry::getDirectionBetweenTracks(trackFromPts, nextTrackPts);
                int angle = std::fabs( geometry::get_angle(directionPrev, Point(0,0), direction, true) );
                adjacentTracks[angle].push_back(vInd);
            }
            else
                adjacentTracks[-1].push_back(vInd);
        }
    }

    for(auto& it_adj : adjacentTracks){
        double minDist = std::numeric_limits<double>::max();
        int minInd = -1;
        bool minIndRev;
        for(auto ind : it_adj.second){

            const auto& trackNext = subfield.tracks.at( trackInds.at(ind) );
            const auto& trackNextPts = trackNext.points;

            bool nextInReverse = geometry::calc_dist(trackFromPts.front(), trackNextPts.front()) + geometry::calc_dist(trackFromPts.back(), trackNextPts.back())
                    > geometry::calc_dist(trackFromPts.front(), trackNextPts.back()) + geometry::calc_dist(trackFromPts.back(), trackNextPts.front());
            if(!prevTrackInReverse)
                nextInReverse = !nextInReverse;

            PointVec connPath0, connPathn;
            double dist0TC, distnTC;
            std::pair<double, bool> connInfo = computeConnectionDistances(m_tracksConnector,
                                                                          subfield,
                                                                          machine,
                                                                          trackFrom,
                                                                          trackNext,
                                                                          prevTrackInReverse,
                                                                          minDist,
                                                                          limitBoundary,
                                                                          useMachineTurningRad,
                                                                          false,
                                                                          !nextInReverse, nextInReverse,
                                                                          connPath0, connPathn,
                                                                          dist0TC, distnTC);
            if(nextInReverse)
                std::swap(dist0TC, distnTC);

            if(connInfo.first < -1e-9 || dist0TC < -1e-9 || dist0TC > minDist) //accept adjacent iif the closest track-end is connectable
                continue;
            minDist = dist0TC;
            minInd = ind;
            minIndRev = nextInReverse;
        }
        if(minInd >= 0)
            return std::make_pair(minInd, minIndRev);
    }
    //not found -> try searching the closest adjacent
    std::set<size_t> assignedTrackIndexesEd(trackInds.begin(), trackInds.end());
    for(auto& it_adj : adjacentTracks){
        for(auto ind : it_adj.second)
            assignedTrackIndexesEd.erase(ind);
    }

    return TrackSequencerClosestNext::getNextTrack(subfield,
                                                   trackInds,
                                                   assignedTrackIndexes,
                                                   machine,
                                                   indPrevTrack,
                                                   prevTrackInReverse,
                                                   sequence,
                                                   limitBoundary,
                                                   connBoundary,
                                                   useMachineTurningRad);

}


} // namespace arolib
