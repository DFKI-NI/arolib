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
 
#ifndef _AROLIB_TRACKSEQUENCERADJACENTNEXT_HPP
#define _AROLIB_TRACKSEQUENCERADJACENTNEXT_HPP

#include <map>

#include "arolib/planning/track_connectors/infieldtracksconnectordef.hpp"
#include "arolib/planning/track_sequencing/tracksequencerclosestnext.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/field_geometry_processing.hpp"

namespace arolib {

  /**
   * @brief Generates the track sequences that the primary machine must follow to cover the inner-field.
   */
  class TrackSequencerAdjacentNext : virtual public ITrackSequencer, protected TrackSequencerClosestNext
  {
  public:
      /**
       * @brief Constructor
       * @param logLevel Log level
       */
      explicit TrackSequencerAdjacentNext(LogLevel logLevel = LogLevel::INFO);


      /**
       * @brief Compute the sequences
       * @param subfield subfield
       * @param machines Working group
       * @param excludeTrackIndexes Indexes of the tracks that should be excluded
       * @param [out] sequences Sequences <machine_id, tracks_sequence>
       * @param initRefPose Pose used to select the first track in the sequence (disregarded if NULL or invalid)
       * @return AroResp with error id (0:=OK) and message
       */
      virtual AroResp computeSequences(const Subfield &subfield,
                                       const std::vector<Machine>& machines,
                                       const TrackSequencerSettings& settings,
                                       std::map<MachineId_t, std::vector<ITrackSequencer::TrackInfo>>& sequences,
                                       const Pose2D* initRefPose = nullptr,
                                       const std::set<size_t>& excludeTrackIndexes = {}) override;


  protected:

      /**
       * @brief Compute the sequences
       * @param subfield subfield
       * @param trackInds Vector holding the real indexes of the tracks used in the serach
       * @param realToVecIndexMap Mapping of real track indexes -> index in trackInds
       * @param assignedTrackIndexes Set of the track indexes already assigned
       * @param adjacentTracksList Map holding the list of indexes of the track adjacent to each track (the map key is the index of trackInds)
       * @param machine Machine
       * @param indPrevTrack Index of trackInds corresponding to the previous track
       * @param prevTrackInReverse Is the previous track in reverse?
       * @param sequence Tracks sequence for the machine
       * @param limitBoundary Limit boundary
       * @param connBoundary Connection boundary (disregarded if empty)
       * @param useMachineTurningRad Flag stating if the machine turning radius must be used in the search
       * @return Index of the next adjacent track, flag stating if the next track is in reverse
       */
      std::pair<int, bool> getNextAdjacentTrack(const Subfield &subfield,
                                                const std::vector<size_t>& trackInds,
                                                const std::map<size_t, size_t> &realToVecIndexMap,
                                                const std::set<size_t>& assignedTrackIndexes,
                                                const std::map<size_t, std::set<size_t> >& adjacentTracksList,
                                                const Machine& machine,
                                                size_t indPrevTrack,
                                                bool prevTrackInReverse,
                                                const std::vector<TrackInfo>& sequence,
                                                const Polygon& limitBoundary,
                                                const Polygon& connBoundary,
                                                bool useMachineTurningRad);


  protected:

  };

}
#endif // _AROLIB_TRACKSEQUENCERADJACENTNEXT_HPP
