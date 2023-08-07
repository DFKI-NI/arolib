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
 
#ifndef _AROLIB_SIMPLETRACKSEQUENCER_HPP
#define _AROLIB_SIMPLETRACKSEQUENCER_HPP

#include <map>

#include "arolib/planning/track_sequencing/tracksequencer.hpp"
#include "arolib/geometry/geometry_helper.hpp"

namespace arolib {

  /**
   * @brief Generates the track sequences that the harvesters must follow to harvest the inner-field based on the sequence strategy
   */
  class SimpleTrackSequencer : public ITrackSequencer
  {
  public:
      /**
       * @brief Constructor
       * @param logLevel Log level
       */
      explicit SimpleTrackSequencer(LogLevel logLevel = LogLevel::INFO);

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

  private:

      /**
       * @brief Check if the tracks are to be worked starting from the first track or the last based on the reference inital point
       * @param subfield subfield
       * @param initRefPoint Point used to select the first track in the sequence (disregarded if NULL or invalid)
       * @param [out] tracksInReverse Tracks in reverse
       * @param [out] trackPointsInReverse Tracks points in reverse
       */
      void areTracksInReverse(const Subfield& subfield, const Point* initRefPoint, bool &tracksInReverse, bool &trackPointsInReverse);

  };

}
#endif // _AROLIB_SIMPLETRACKSEQUENCER_HPP
