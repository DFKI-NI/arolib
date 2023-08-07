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
 
#include "arolib/planning/track_sequencing/tracksequencerspiral.hpp"

#include <stdexcept>
#include <algorithm>

namespace arolib{


TrackSequencerSpiral::SequenceStrategy TrackSequencerSpiral::intToSequenceStrategy(int value)
{
    if(value == SequenceStrategy::INNER_TO_OUTER)
        return SequenceStrategy::INNER_TO_OUTER;
    else if(value == SequenceStrategy::OUTER_TO_INNER)
        return SequenceStrategy::OUTER_TO_INNER;

    throw std::invalid_argument( "The given value does not correspond to any TrackSequencer::SequenceStrategy" );
}

TrackSequencerSpiral::TrackSequencerSpiral(LogLevel logLevel) :
    ITrackSequencer(__FUNCTION__, logLevel)
{
}

AroResp TrackSequencerSpiral::computeSequences(const Subfield &subfield,
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

    int start_track = 0;

    std::vector<size_t> trackInds;
    for(size_t i = 0 ; i < subfield.tracks.size() ; ++i){
        if(excludeTrackIndexes.find(i) == excludeTrackIndexes.end())
            trackInds.push_back(i);
    }
    if(trackInds.empty())
        return AroResp(1, "No tracks left after excludeding given tracks");

    int nr_tracks_per_bed = m_tracks_per_machine * machines.size();

    bool tracksInReverse, trackPointsInReverse;
    areTracksInReverse(subfield, initRefPose, tracksInReverse, trackPointsInReverse);

    std::vector<Bed> beds;

    //iteratively compute beds until the remaining tracks are less than the tracks per bed
    while((beds.size()+1) * nr_tracks_per_bed <= trackInds.size()) {
        addBed(start_track, machines,nr_tracks_per_bed, m_tracks_per_machine, trackInds, beds);
        start_track = beds.back().end_track + 1;
    }

    int num_remaining_tracks = trackInds.size() - start_track;//get number of remaining tracks

    logger().printOut(LogLevel::DEBUG, __FUNCTION__, std::string("BEDS GENERATION: ")
                      + "\t trackInds.size() = " + std::to_string(trackInds.size()) + "\n"
                      + std::string("\t --1-- \n")
                      + "\t nr_tracks_per_bed = " + std::to_string(nr_tracks_per_bed) + "\n"
                      + "\t nr_tracks_per_machine_bed = " + std::to_string(m_tracks_per_machine) + "\n"
                      + "\t beds.size() = " + std::to_string(beds.size()) + "\n"
                      + "\t Nr tracks so far = " + std::to_string(beds.size() * nr_tracks_per_bed) + "\n"
                      + std::string("\t --2-- \n")
                      + "\t num_remaining_tracks = " + std::to_string(num_remaining_tracks) + "\n"
                      + "\t start_track = " + std::to_string(start_track) );

    if(num_remaining_tracks % machines.size() == 0 ||
       num_remaining_tracks / machines.size() > 0) {// there is more than one track per machine left --> compute another bed with adjusted tracks_per_machine parameter
        int tracks_per_machine = num_remaining_tracks / machines.size();

//          if(start_track < 0)
//              start_track = start_track + nr_tracks_per_bed - 1;

//        logger().printOut(LogLevel::DEBUG, __FUNCTION__,
//                            "\t start_track (2) = " + std::to_string(start_track) + "\n"
//                          + "\t nr_tracks_per_machine_bed = " + std::to_string(tracks_per_machine) );

        //addBed(start_track, machines, nr_tracks_per_bed);
        addBed( start_track, machines, tracks_per_machine*machines.size(), tracks_per_machine, trackInds, beds );
        num_remaining_tracks -= (tracks_per_machine * machines.size());
    }


    if(num_remaining_tracks > 0) {// there are still tracks left (less than the number of machines) --> use the first machine to process all of them
        start_track = trackInds.size() - num_remaining_tracks;
        // if unprocessed tracks are remaining, just harvest them with the first harvester of the group
        std::vector<Machine> first_harvester;
        first_harvester.push_back(machines.at(0));
        addBed(start_track,first_harvester, num_remaining_tracks, num_remaining_tracks, trackInds, beds);
    }

    if(tracksInReverse) {
        logger().printOut(LogLevel::ERROR,__FUNCTION__, "Inverting tracks order...");
        for (size_t i = 0 ; i < beds.size() ; ++i){
            beds.at(i).start_track = trackInds.size() - 1 - beds.at(i).start_track;
            beds.at(i).end_track = trackInds.size() - 1 - beds.at(i).end_track;
            for (size_t j = 0 ; j < beds.at(i).track_indexes.size() ; ++j){
                beds.at(i).track_indexes.at(j) = trackInds.size() - 1 - beds.at(i).track_indexes.at(j);
            }
        }
    }

    //adjust the bed indexes to the real subfield track indexes
    for(auto& bed : beds){
        bed.start_track = trackInds.at(bed.start_track);
        bed.end_track = trackInds.at(bed.end_track);
        for(auto& ind : bed.track_indexes)
            ind = trackInds.at(ind);
    }


    logger().printOut(LogLevel::DEBUG, __FUNCTION__, std::string("\n")
                      + "\t # tracks = " + std::to_string(trackInds.size()) + "\n"
                      + "\t # machines = " + std::to_string(machines.size()) + "\n"
                      + "\t # tracks per machine = " + std::to_string(m_tracks_per_machine) + "\n"
                      + "\t # beds = " + std::to_string(beds.size()) + "\n"
                      + "\t # tracks per bed = " + std::to_string(nr_tracks_per_bed) );


    std::map<MachineId_t, std::pair<Point, Point> > lastMachinePoints; //pair <first visited track point, last visited track point>
    Point refPoint0, refPointn;
    for (size_t i = 0; i < beds.size(); ++i) {

        for (size_t j = 0; j < beds.at(i).machine_ids.size(); ++j) {
            auto machineId = beds.at(i).machine_ids.at(j);
            TrackInfo info;
            info.trackIndex = beds.at(i).track_indexes.at(j);

            auto& p0 = subfield.tracks.at(info.trackIndex).points.front();
            auto& pn = subfield.tracks.at(info.trackIndex).points.back();

            if(lastMachinePoints.empty()){//first track, we know how it goes
                info.trackPointsDirection = trackPointsInReverse ? TrackPointsDirection::REVERSE : TrackPointsDirection::FORWARD;
                if(trackPointsInReverse){
                    info.trackPointsDirection = TrackPointsDirection::REVERSE;
                    refPoint0 = pn;
                    refPointn = p0;
                    lastMachinePoints[machineId] = std::make_pair( pn, p0 );
                }
                else{
                    info.trackPointsDirection = TrackPointsDirection::FORWARD;
                    refPoint0 = p0;
                    refPointn = pn;
                    lastMachinePoints[machineId] = std::make_pair( p0, pn );
                }
            }
            else if(lastMachinePoints.find(machineId) == lastMachinePoints.end()){
                //must go in the same direction as the firts machine
                double dist1 = geometry::calc_dist(refPoint0, p0) + geometry::calc_dist(refPointn, pn);
                double dist2 = geometry::calc_dist(refPoint0, pn) + geometry::calc_dist(refPointn, p0);
                if( dist1 <= dist2 ){
                    info.trackPointsDirection = TrackPointsDirection::FORWARD;
                    lastMachinePoints[machineId] = std::make_pair( p0, pn );
                }
                else{
                    info.trackPointsDirection = TrackPointsDirection::REVERSE;
                    lastMachinePoints[machineId] = std::make_pair( pn, p0 );
                }

            }
            else{
                //must go in opposite direction from the last machine track
                auto ptsPrev = lastMachinePoints[machineId];
                double dist1 = geometry::calc_dist(ptsPrev.second, p0) + geometry::calc_dist(ptsPrev.first, pn);
                double dist2 = geometry::calc_dist(ptsPrev.second, pn) + geometry::calc_dist(ptsPrev.first, p0);
                if( dist1 <= dist2 ){
                    info.trackPointsDirection = TrackPointsDirection::FORWARD;
                    lastMachinePoints[machineId] = std::make_pair( p0, pn );
                }
                else{
                    info.trackPointsDirection = TrackPointsDirection::REVERSE;
                    lastMachinePoints[machineId] = std::make_pair( pn, p0 );
                }
            }

            sequences[machineId].push_back(info);
        }
    }


    return AroResp::ok();

}

bool TrackSequencerSpiral::setTracksPerMachinePerBed(size_t tracks_per_machine) {
    if(tracks_per_machine == 0)
        return false;
    m_tracks_per_machine = tracks_per_machine;
    return true;
}


void TrackSequencerSpiral::areTracksInReverse(const Subfield &subfield, const Point *initRefPoint, bool & tracksInReverse, bool & trackPointsInReverse)
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

void TrackSequencerSpiral::addBed(int start_track,
                                  std::vector<Machine> machines,
                                  int tracks_per_bed,
                                  int tracks_per_machine,
                                  const std::vector<size_t> &trackInds,
                                  std::vector<Bed>& beds) {
  Bed bed;
  bed.start_track = start_track;
  if(m_strategy == OUTER_TO_INNER) {
      computeBed(bed, machines, tracks_per_bed, tracks_per_machine, trackInds);
      std::reverse(bed.track_indexes.begin(), bed.track_indexes.end());
  }
  else {
      computeBed(bed, machines, tracks_per_bed, tracks_per_machine, trackInds);
  }
  beds.push_back(bed);
}


void TrackSequencerSpiral::computeBed(Bed &bed,
                                const std::vector<Machine>& machines,
                                int nr_tracks_per_bed,
                                int tracks_per_machine,
                                const std::vector<size_t> &trackInds)
{
    //@TODO: In this moment, the beds and their (sub)sequences are computed in order to minimize the distances traveled by the harvesters between beds.
    //       This, however, doesn't take into accout whether the harvester can only download at one of it sides.
    //       In the future, the 'unloading sides' should be taken into account, and the direction in which the sequence is computed in the bed but be calculated based on those directions (disregarding the distance to travel between beds)


    int v = 0;
    int cur_track;
    int nr_tracks = 0;
    int bed_start;//holds the (delta) index of the first track to be harvested in the bed (w.r.t. the bed's first track). If bed_start = 0 --> the first track to be harvested is the bed's first track
    int dir = 1; //used so that the last track of the bed is the closest to the next bed in IN_TO_OUT strategy, or that the first track of the bed is the closest to the previous bed in OUT_TO_IN strategy

    if(nr_tracks_per_bed < 1e-6) {//compute the nr_tracks_per_bed based on the working group and the number of tracks per machine per bed
        nr_tracks_per_bed = tracks_per_machine * machines.size();
    }

    if( nr_tracks_per_bed > trackInds.size() - bed.start_track ){//if there are not enoght track left, adjust nr_tracks_per_bed and tracks_per_machine according to the number of remaining tracks
        nr_tracks_per_bed = trackInds.size() - bed.start_track;
        tracks_per_machine = nr_tracks_per_bed / machines.size();
    }

    bed.end_track = bed.start_track + nr_tracks_per_bed -1;

    /**
    if(bed.start_track == bed.end_track) {
        bed.track_idx.push_back(bed.start_track);
        bed.machine_id.push_back(machines.at(0).id);
        return;
    }
    **/

    // get the first track (between 0 and nr_tracks_per_bed) for the first machine in this bed
    if(tracks_per_machine%2 == 0) {//even number of tracks per machine
        bed_start = nr_tracks_per_bed/2;
        if(m_strategy == INNER_TO_OUTER){
            bed_start -= 1;
            dir *= -1;
        }
    }
    else {//odd number of tracks per machine
        bed_start = (tracks_per_machine/2) * machines.size();
        if(m_strategy == OUTER_TO_INNER){
            bed_start += machines.size() -1;
            dir *= -1;
        }
        if(bed_start < 0)
            bed_start = 0;
    }

    // iterativly add tracks to the bed
    while(nr_tracks < nr_tracks_per_bed) {
        for(int i = 0; i < machines.size(); i++) {
            bed.machine_ids.push_back(machines.at(i).id);
            if(v%2 != 0) {
                cur_track = bed_start + bed.start_track + dir * ( machines.size() - i - 1 - ( (v/2 + 1) * machines.size() ) );
            } else {
                cur_track = bed_start + bed.start_track + dir * ( i + v/2 * machines.size() );
            }
            bed.track_indexes.push_back(cur_track);
            nr_tracks++;

//              if (nr_tracks >= nr_tracks_per_bed)
//                  break;
        }
        v++;
    }

}

} // namespace arolib
