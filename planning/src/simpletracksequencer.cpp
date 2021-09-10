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
 
#include "arolib/planning/simpletracksequencer.hpp"

#include <stdexcept>
#include <algorithm>

namespace arolib{


SimpleTrackSequencer::SequenceStrategy SimpleTrackSequencer::intToSequenceStrategy(int value)
{
    if(value == SequenceStrategy::MEANDER)
        return SequenceStrategy::MEANDER;
    else if(value == SequenceStrategy::INNER_TO_OUTER)
        return SequenceStrategy::INNER_TO_OUTER;
    else if(value == SequenceStrategy::OUTER_TO_INNER)
        return SequenceStrategy::OUTER_TO_INNER;

    throw std::invalid_argument( "The given value does not correspond to any TrackSequencer::SequenceStrategy" );
}

SimpleTrackSequencer::SimpleTrackSequencer(LogLevel logLevel) :
    LoggingComponent(logLevel, __FUNCTION__)
{
    m_subfield.id = -1;
}

int SimpleTrackSequencer::getNumberMachines() { return m_machines.size(); }

int SimpleTrackSequencer::getID(size_t machine_ind) {
    if (machine_ind >= m_machines.size() )
        return -1;
    return m_machines.at(machine_ind).id;
}

bool SimpleTrackSequencer::computeSequence(Subfield *subfield,
                                           std::vector<Machine> machines) {
    int start_track = 0;

    if(subfield != NULL) {//rewrite the saved working subfield
        setSubfield(*subfield);
    }

    if(machines.size() > 0) {//add machines to the saved working group
        for (int i = 0; i < machines.size(); ++i) {
            addMachine(machines.at(i));
        }
    }

    if(m_machines.empty() || m_subfield.tracks.empty()) {
        m_logger.printOut(LogLevel::ERROR,__FUNCTION__, "Unable to compute sequence: subfield or machines are not set");
        return false;
    }

    std::vector<size_t> trackInds;
    for(size_t i = 0 ; i < m_subfield.tracks.size() ; ++i){
        if(m_excludeTrackIndexes.find(i) == m_excludeTrackIndexes.end())
            trackInds.push_back(i);
    }
    if(trackInds.empty()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No tracks left after excludeding given tracks");
        return false;
    }

    int nr_tracks_per_bed = m_tracks_per_machine * m_machines.size();

    if (m_strategy == MEANDER) {
        // do everything in the "getSequence" function (we don't need any bed separation)
       return true;
    }

    //iteratively compute beds until the remaining tracks are less than the tracks per bed
    while((m_beds.size()+1) * nr_tracks_per_bed <= trackInds.size()) {
        addBed(start_track, m_machines,nr_tracks_per_bed, m_tracks_per_machine, trackInds);
        start_track = m_beds.back().end_track + 1;
    }

    int num_remaining_tracks = trackInds.size() - start_track;//get number of remaining tracks

    m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, std::string("BEDS GENERATION: ")
                      + "\t trackInds.size() = " + std::to_string(trackInds.size()) + "\n"
                      + std::string("\t --1-- \n")
                      + "\t nr_tracks_per_bed = " + std::to_string(nr_tracks_per_bed) + "\n"
                      + "\t nr_tracks_per_machine_bed = " + std::to_string(m_tracks_per_machine) + "\n"
                      + "\t m_beds.size() = " + std::to_string(m_beds.size()) + "\n"
                      + "\t Nr tracks so far = " + std::to_string(m_beds.size() * nr_tracks_per_bed) + "\n"
                      + std::string("\t --2-- \n")
                      + "\t num_remaining_tracks = " + std::to_string(num_remaining_tracks) + "\n"
                      + "\t start_track = " + std::to_string(start_track) );

    if(num_remaining_tracks % m_machines.size() == 0 ||
       num_remaining_tracks / m_machines.size() > 0) {// there is more than one track per machine left --> compute another bed with adjusted tracks_per_machine parameter
        int tracks_per_machine = num_remaining_tracks / m_machines.size();

//          if(start_track < 0)
//              start_track = start_track + nr_tracks_per_bed - 1;

//        m_logger.printOut(LogLevel::DEBUG, __FUNCTION__,
//                            "\t start_track (2) = " + std::to_string(start_track) + "\n"
//                          + "\t nr_tracks_per_machine_bed = " + std::to_string(tracks_per_machine) );

        //addBed(start_track, m_machines, nr_tracks_per_bed);
        addBed( start_track, m_machines, tracks_per_machine*m_machines.size(), tracks_per_machine, trackInds );
        num_remaining_tracks -= (tracks_per_machine * m_machines.size());
    }


    if(num_remaining_tracks > 0) {// there are still tracks left (less than the number of machines) --> use the first machine to process all of them
        start_track = trackInds.size() - num_remaining_tracks;
        // if unprocessed tracks are remaining, just harvest them with the first harvester of the group
        std::vector<Machine> first_harvester;
        first_harvester.push_back(m_machines.at(0));
        addBed(start_track,first_harvester, num_remaining_tracks, num_remaining_tracks, trackInds);
    }

    if(m_inverse_track_order) {
        m_logger.printOut(LogLevel::ERROR,__FUNCTION__, "Inverting tracks order...");
        for (size_t i = 0 ; i < m_beds.size() ; ++i){
            m_beds.at(i).start_track = trackInds.size() - 1 - m_beds.at(i).start_track;
            m_beds.at(i).end_track = trackInds.size() - 1 - m_beds.at(i).end_track;
            for (size_t j = 0 ; j < m_beds.at(i).track_indexes.size() ; ++j){
                m_beds.at(i).track_indexes.at(j) = trackInds.size() - 1 - m_beds.at(i).track_indexes.at(j);
            }
        }
    }

    //adjust the bed indexes to the real subfield track indexes
    for(auto& bed : m_beds){
        bed.start_track = trackInds.at(bed.start_track);
        bed.end_track = trackInds.at(bed.end_track);
        for(auto& ind : bed.track_indexes)
            ind = trackInds.at(ind);
    }


    m_logger.printOut(LogLevel::INFO, __FUNCTION__, std::string("\n")
                      + "\t # tracks = " + std::to_string(trackInds.size()) + "\n"
                      + "\t # machines = " + std::to_string(m_machines.size()) + "\n"
                      + "\t # tracks per machine = " + std::to_string(m_tracks_per_machine) + "\n"
                      + "\t # beds = " + std::to_string(m_beds.size()) + "\n"
                      + "\t # tracks per bed = " + std::to_string(nr_tracks_per_bed) );

    return true;
}

std::vector<int> SimpleTrackSequencer::getSequence(int machine_id) {

    std::vector<int> sequence;
    if(m_beds.size() == 0 && (m_strategy == MEANDER && m_machines.size() == 0)) {// the sequence is not computed yet compute it --> compute it
        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Computing sequence...");
        if(!computeSequence()){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error computing sequence");
            return sequence;
        }
        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Sequence computed");
    }

    if (m_strategy == MEANDER) {
        int thisMachineNR = -1;
        for (size_t j=0; j < m_machines.size(); j++) {
            if (m_machines.at(j).id == machine_id) {
                thisMachineNR = j;
                break;
            }
        }
        if (thisMachineNR == -1){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Requested machine_id not found in working group");
            return sequence;
        }

        std::vector<size_t> trackInds;
        for(size_t i = 0 ; i < m_subfield.tracks.size() ; ++i){
            if(m_excludeTrackIndexes.find(i) == m_excludeTrackIndexes.end())
                trackInds.push_back(i);
        }
        if(trackInds.empty()){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No tracks left after excludeding given tracks");
            return sequence;
        }

        for(size_t i = 0; i < trackInds.size(); i += m_machines.size()) {
            if (i + thisMachineNR < trackInds.size()){
                if(m_inverse_track_order)
                    sequence.push_back( r_at(trackInds, i + thisMachineNR ) );
                else
                    sequence.push_back( trackInds.at(i + thisMachineNR) );
            }
        }

    }
    else {//use the computed beds
        for (size_t i = 0; i < m_beds.size(); ++i) {
            for (size_t j = 0; j < m_beds.at(i).machine_ids.size(); ++j) {
                if(m_beds.at(i).machine_ids.at(j) == machine_id) {
                    sequence.push_back(m_beds.at(i).track_indexes.at(j));
                }
            }
        }
    }

    return sequence;
}

void SimpleTrackSequencer::addBed(int start_track, std::vector<Machine> machines, int tracks_per_bed, int tracks_per_machine, const std::vector<size_t> &trackInds) {
  Bed bed;
  bed.start_track = start_track;
  if(m_strategy == OUTER_TO_INNER) {
      computeBed(bed, machines, tracks_per_bed, tracks_per_machine, trackInds);
      std::reverse(bed.track_indexes.begin(), bed.track_indexes.end());
  }
  else {
      computeBed(bed, machines, tracks_per_bed, tracks_per_machine, trackInds);
  }
  m_beds.push_back(bed);
}


void SimpleTrackSequencer::computeBed(Bed &bed,
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
