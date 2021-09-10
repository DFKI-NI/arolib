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
 
#ifndef _AROLIB_SIMPLETRACKSEQUENCER_HPP
#define _AROLIB_SIMPLETRACKSEQUENCER_HPP

#include <map>

#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/logger.h"
#include "arolib/types/field.hpp"
#include "arolib/types/machine.hpp"

namespace arolib {

  /**
   * @brief Generates the track sequences that the harvesters must follow to harvest the inner-field based on the sequence strategy
   */
  class SimpleTrackSequencer : public LoggingComponent
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
       MEANDER, /**< The harvester(s) will start from one side of the field and harvest the tracks in order (either forward or reverse) until the other side of the field */
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
    explicit SimpleTrackSequencer(LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Add a machine (harvester) to the working group
     * @param machine Machine
     */
    inline void addMachine(Machine &machine) {m_machines.push_back(machine);}

    /**
     * @brief Get the amount of machines in the working group
     * @return Amount of machines in the working group
     */
    int getNumberMachines();

    /**
     * @brief Get the id of the machine located in the given index (in the working group vector)
     * @param machine_ind Machine index (in the working group vector)
     * @return Id of the machine located in the given index ( <0 of index is invali)
     */
    int getID(size_t machine_ind);

    /**
     * @brief Compute the sequences
     * @param subfield (~optional) Working subfield (overwrites the currently saved subfield). If = NULL, the currently saved subfield is used.
     * @param machines (~optional) This machines will be ADDED to the currently saved working grupot prior to the squence calculation
     * @return True on success
     */
    bool computeSequence(Subfield *subfield = NULL,
                         std::vector<Machine> machines = std::vector<Machine>());

    /**
     * @brief Get the computed track sequence for the given machine.
     * @param machine_id Machine id
     * @return Track sequence for the given machine: vector containing the track indexes in order of harvesting
     */
    std::vector<int> getSequence(int machine_id);

    /**
     * @brief Set the working subfield
     * @param subfield Processed field (inc. tracks)
     */
    inline void setSubfield(Subfield &subfield) {m_subfield = subfield;}

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
     * @brief Set whether the thacks should be processed in inverse order or not (related to the order they are saved in the subfield's track vector).
     * @param inverse Flag stating inverse order
     */
    inline void setInverseTrackOrder(const bool& inverse) {m_inverse_track_order = inverse;}

    /**
     * @brief Ser the amount of tracks per machine per one bed.
     * @param tracks Amount of tracks per machine per one bed.
     */
    inline void setTracksPerMachine(int tracks) {m_tracks_per_machine = tracks;}


    /**
     * @brief Set the indexes of the tracks to be excluded.
     * @param excludeTrackIndexes Indexes of the tracks to be excluded
     */
    inline void setExcludeTrackIndexes(const std::set<size_t>& excludeTrackIndexes ) {
        m_excludeTrackIndexes = excludeTrackIndexes;
    }

  private:

    /**
     * @brief Computes a bed and adds it to the internal set of beds
     * @param start_track Index of the first track of the bed
     * @param machines Machines assigned to the bed
     * @param tracks_per_bed Number of tracks in the bed (if <= 0, it is calculated based on 'tracks_per_machine' and 'machines')
     * @param tracks_per_machine Number of tracks per machine in the bed (disregarded if tracks_per_bed > 0)
     * @param trackIds Vector holding the (non excluded) track idexes (in ascending order)
     */
    void addBed(int start_track, std::vector<Machine> machines, int tracks_per_bed, int tracks_per_machine, const std::vector<size_t>& trackInds);

    /**
     * @brief Compute a bed
     * @param [in/out] bed Bed to be computed (its start_track must be set already)
     * @param tracks_per_bed Number of tracks in the bed (if <= 0, it is calculated based on 'tracks_per_machine' and 'machines')
     * @param tracks_per_machine Number of tracks per machine in the bed (disregarded if tracks_per_bed > 0)
     */
    void computeBed(Bed &bed, const std::vector<Machine> &machines, int tracks_per_bed, int tracks_per_machine, const std::vector<size_t> &trackInds);

    Subfield m_subfield; /**< Working subfield */
    std::vector<Machine> m_machines; /**< Working group */
    int m_tracks_per_machine; /**< Number of tracks per machine per bed */
    SequenceStrategy m_strategy = OUTER_TO_INNER; /**< Strategy to compute the track sequences */
    /// if true the bed pattern is computed from tracks.size to zero not vice versa.
    bool m_inverse_track_order = false; /**< Flag stating whether the tracks should be processed in inverse order or not (related to the order they are saved in the subfield's track vector) */
    std::vector<Bed> m_beds;  /**< Computed beds */
    std::set<size_t> m_excludeTrackIndexes; /**< Indexes of the track to be excluded */

  };

}
#endif // _AROLIB_SIMPLETRACKSEQUENCER_HPP
