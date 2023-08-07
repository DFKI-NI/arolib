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
 
#ifndef _AROLIB_TRACKSEQUENCERSPIRAL_HPP
#define _AROLIB_TRACKSEQUENCERSPIRAL_HPP

#include <map>

#include "arolib/planning/track_sequencing/tracksequencer.hpp"
#include "arolib/geometry/geometry_helper.hpp"

namespace arolib {

  /**
   * @brief Generates the track sequences that the harvesters must follow to harvest the inner-field based on the sequence strategy
   */
  class TrackSequencerSpiral : public ITrackSequencer
  {
  public:
      /**
       * @brief Part of the inner-field comprising a set of consecutive tracks, in cases where the the complete inner-field hast to be harvested in batches
       */
      struct Bed
      {
          int start_track; /**< Index of the first track of the bed */
          int end_track; /**< Index of the last track of the bed */
          std::vector<MachineId_t> machine_ids; /**< Id of the machine that corresponds to respective track_idx */
          std::vector<int> track_indexes; /**< Indexes of the tracks of the bed sorted by start time (sequence) */
      };

      /**
       * @brief Strategy to compute the track sequences.
       */
      enum SequenceStrategy{
         INNER_TO_OUTER, /**< (for batch/beds harvesting) The harvester(s) will start in the track located in the middle of the bed and harvest outwards in a spiral fashion. */
         OUTER_TO_INNER /**< (for batch/beds harvesting) The harvester(s) will start in a track located in one of the sides of the bed harvest inwards in a spiral fashion. */
      };
      /**
        * @brief Get the SequenceStrategy (enum) from its int value
        */
      static SequenceStrategy intToSequenceStrategy(int value);

      /**
       * @brief Constructor
       * @param logLevel Log level
       */
      explicit TrackSequencerSpiral(LogLevel logLevel = LogLevel::INFO);

      /**
       * @brief Compute the sequences
       * @param subfield subfield
       * @param machines Working group
       * @param excludeTrackIndexes Indexes of the tracks that should be excluded
       * @param [out] sequences Sequences <machine_id, track_id sequence>
       * @param initRefPose Pose used to select the first track in the sequence (disregarded if NULL or invalid)
      * @return AroResp with error id (0:=OK) and message
       */
      virtual AroResp computeSequences(const Subfield &subfield,
                                       const std::vector<Machine>& machines,
                                       const TrackSequencerSettings& settings,
                                       std::map<MachineId_t, std::vector<ITrackSequencer::TrackInfo>>& sequences,
                                       const Pose2D* initRefPose = nullptr,
                                       const std::set<size_t>& excludeTrackIndexes = {}) override;

      /**
       * @brief Set the sequence strategy
       * @param strategy Sequence strategy
       */
      inline void setStrategy(SequenceStrategy strategy) {m_strategy = strategy;}

      /**
       * @brief Get the current sequence strategy
       * @param strategy Current sequence strategy
       */
      inline SequenceStrategy getStrategy() const  {return m_strategy;}

      /**
       * @brief Set the number of tracks per machine per bed
       * @param tracks_per_machine Number of tracks per machine per bed
       */
      bool setTracksPerMachinePerBed(size_t tracks_per_machine);

      /**
       * @brief Get the number of tracks per machine per bed
       * @param Number of tracks per machine per bed
       */
      inline size_t getTracksPerMachinePerBed() const  {return m_tracks_per_machine;}



  private:

      /**
       * @brief Check if the tracks are to be worked starting from the first track or the last based on the reference inital point
       * @param subfield subfield
       * @param initRefPoint Point used to select the first track in the sequence (disregarded if NULL or invalid)
       * @param [out] tracksInReverse Tracks in reverse
       * @param [out] trackPointsInReverse Tracks points in reverse
       */
      void areTracksInReverse(const Subfield& subfield, const Point* initRefPoint, bool &tracksInReverse, bool &trackPointsInReverse);

      /**
       * @brief Computes a bed and adds it to the internal set of beds
       * @param start_track Index of the first track of the bed
       * @param machines Machines assigned to the bed
       * @param tracks_per_bed Number of tracks in the bed (if <= 0, it is calculated based on 'tracks_per_machine' and 'machines')
       * @param tracks_per_machine Number of tracks per machine in the bed (disregarded if tracks_per_bed > 0)
       * @param trackIds Vector holding the (non excluded) track idexes (in ascending order)
       * @param beds Beds
       */
      void addBed(int start_track, std::vector<Machine> machines, int tracks_per_bed, int tracks_per_machine, const std::vector<size_t>& trackInds, std::vector<Bed> &beds);

      /**
       * @brief Compute a bed
       * @param [in/out] bed Bed to be computed (its start_track must be set already)
       * @param tracks_per_bed Number of tracks in the bed (if <= 0, it is calculated based on 'tracks_per_machine' and 'machines')
       * @param tracks_per_machine Number of tracks per machine in the bed (disregarded if tracks_per_bed > 0)
       */
      void computeBed(Bed &bed, const std::vector<Machine> &machines, int tracks_per_bed, int tracks_per_machine, const std::vector<size_t> &trackInds);


  private:
      SequenceStrategy m_strategy = INNER_TO_OUTER; /**< Strategy to compute the track sequences */
      size_t m_tracks_per_machine = 3; /**< Number of tracks per machine per bed */

  };

}
#endif // _AROLIB_TRACKSEQUENCERSPIRAL_HPP
