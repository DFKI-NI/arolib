/*
 * Copyright 2021-2025 DFKI GmbH
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
 
#ifndef AROLIB_SIMPLEHARVESTERPLANNER_H
#define AROLIB_SIMPLEHARVESTERPLANNER_H

#include "track_sequencing/simpletracksequencer.hpp"
#include "edge_calculators/edgeSpeedCalculator.hpp"
#include "arolib/types/route.hpp"
#include "arolib/types/machinedynamicinfo.hpp"

namespace arolib{


/**
 * @brief Class used to plan preliminary base routes for inner-field processing (harvesting, seeding, etc)
 */
class SimpleBaseRoutesPlanner : public LoggingComponent{

public:
    /**
     * @brief constructor
     * @param logLevel Log level
     */
    explicit SimpleBaseRoutesPlanner(const LogLevel &logLevel = LogLevel::INFO);

    /**
     * @brief Set the working subfield
     * @param subfield Subfield
      * @return AroResp with error id (0:=OK) and message
     */
    AroResp setSubfield(const Subfield &subfield);

    /**
     * @brief Add a harvester to the working group
     * @param machine Harvester machine
      * @return AroResp with error id (0:=OK) and message
     */
    AroResp addMachine(const Machine& machine);

    /**
     * @brief Set the Infield TracksConnector to be used.
     * @param connector Infield TracksConnector to be used (if nullptr, it will use the default one of the route assembler)
     */
    inline void setInfieldTrackConnector(std::shared_ptr<IInfieldTracksConnector> connector){
        m_tracks_connector = connector;
    }

    /**
     * @brief Set the Infield TrackSequencer to be used.
     * @param track_sequencer Infield TrackSequencer to be used.
     */
    inline void setInfieldTrackSequencer(std::shared_ptr<ITrackSequencer> track_sequencer) {
        if(track_sequencer) m_track_sequencer = track_sequencer;
    }

    /**
     * @brief Set the Infield TrackSequencer settings to be used.
     * @param track_sequencer Infield TrackSequencer settings to be used.
     */
    inline void setInfieldTrackSequencerSettings(const ITrackSequencer::TrackSequencerSettings& settings) {
        m_track_sequencer_settings = settings;
    }

    /**
     * @brief Set the indexes of the tracks to be excluded.
     * @param excludeTrackIndexes Indexes of the tracks to be excluded
     */
    inline void setExcludeTrackIndexes(const std::set<size_t>& excludeTrackIndexes ) {
        m_excludeTrackIndexes = excludeTrackIndexes;
    }


    /**
     * @brief Set the initial reference pose.
     *
     * If set, inverse flags InverseTrackOrder and FirstTrackInversePointOrder might be disregarded
     * @param pose initial reference pose (disregarded if invalid)
     */
    inline void setInitRefPoses(const std::map<MachineId_t, Pose2D>& poses ) {
        m_initRefPoses = poses;
    }


    /**
     * @brief Set the flag to reuse the the track connection paths saved in the sequencer (if available) instead of recomputing them with the tracks connector.
     * @param enable If true, it will reuse the track connection paths saved in the sequencer (if available) instead of recomputing them with the tracks connector.
     */
    inline void setReuseSequencerConnections(bool enable) {
        m_reuse_sequencer_connections = enable;
    }


    /**
     * @brief Get the planned route of a given machine
     *
     * If the routes have not been planned yet, calling the method will compute the routes for all machines first.
     * @param machine_id Id of the machine assigned to it
     * @param [out] route Planned route
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Speed calculator (working edges).
     * @param [in/out*] edgeSpeedCalculator Speed calculator (transit edges).
      * @return AroResp with error id (0:=OK) and message
     */
    AroResp getRoute(int machine_id,
                     Route& route,
                     std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                     std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                     std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorTransit);

private:

    /**
     * @brief Plans and generates the harvester routes
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Speed calculator (working edges).
     * @param [in/out*] edgeSpeedCalculator Speed calculator (transit edges).
      * @return AroResp with error id (0:=OK) and message
     */
    AroResp plan(std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                 std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                 std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorTransit);

protected:

   std::shared_ptr<ITrackSequencer> m_track_sequencer; /**< Inner-field track sequencer. */
   ITrackSequencer::TrackSequencerSettings m_track_sequencer_settings;/**< Inner-field track sequencer settings. */
   Subfield m_subfield; /**< Working subfield. */
   std::map<MachineId_t, Machine> m_machinesMap; /**< Working group. */
   std::vector<Machine> m_machines; /**< Working group. */
   std::map<MachineId_t, Route> m_machines_routes; /**< Planned routes. <machine id, route> */
   std::set<size_t> m_excludeTrackIndexes; /**< Indexes of the tracks that will be disregarded. */
   std::map<MachineId_t, Pose2D> m_initRefPoses = {}; /**< Initial reference poses per machine. */
   std::shared_ptr<IInfieldTracksConnector> m_tracks_connector = nullptr; /**< Infield tracks' connector */

   bool m_has_subfield; /**< Was the subfield already set?. */
   bool m_has_machine; /**< There exists at least one machine in the working group. */

   bool m_reuse_sequencer_connections = true; /**< If true, it will reuse the track connection paths saved in the sequencer (if available) instead of recomputing them with the tracks connector. */

};

}



#endif //AROLIB_SIMPLEHARVESTERPLANNER_H
