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
 
#ifndef AROLIB_SIMPLEHARVESTERPLANNER_H
#define AROLIB_SIMPLEHARVESTERPLANNER_H

#include "arolib/types/route.hpp"
#include "track_sequencing/simpletracksequencer.hpp"
#include "arolib/types/field.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/cartography/common.hpp"
#include "arolib/planning/planningworkspace.h"
#include "arolib/planning/routeassembler.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/logger.h"

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
     * @brief Set whether the thacks should be processed in inverse order or not (related to the order they are saved in the subfield's track vector).
     * @param inverse Flag stating inverse order
     */
    inline void setInverseTrackOrder(bool inverse) {
        m_tracks_in_reverse = inverse;
    }

    /**
     * @brief Set whether the first track should be worrked in inverse (points) order.
     * @param inverse Flag stating inverse order
     */
    inline void setFirstTrackInversePointOrder(bool inverse) {
        m_first_track_inverse = inverse;
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
    inline void setInitRefPose(const Pose2D& pose ) {
        m_initRefPose = pose;
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

    Pose2D getInitRefPoseForSequencer();

protected:

   std::shared_ptr<ITrackSequencer> m_track_sequencer; /**< Inner-field track sequencer. */
   ITrackSequencer::TrackSequencerSettings m_track_sequencer_settings;/**< Inner-field track sequencer settings. */
   Subfield m_subfield; /**< Working subfield. */
   std::map<MachineId_t, Machine> m_machinesMap; /**< Working group. */
   std::vector<Machine> m_machines; /**< Working group. */
   std::map<MachineId_t, Route> m_machines_routes; /**< Planned routes. <machine id, route> */
   bool m_tracks_in_reverse = false; /**< Flag stating whether the tracks shoud start from the front (false) or back (true). */
   bool m_first_track_inverse = false; /**< Flag stating whether the tracks shoud start with inverse point order. */
   std::set<size_t> m_excludeTrackIndexes; /**< Indexes of the tracks that will be disregarded. */
   Pose2D m_initRefPose = Pose2D(Point::invalidPoint(), 0); /**< Initial reference pose. */

   bool m_has_subfield; /**< Was the subfield already set?. */
   bool m_has_machine; /**< There exists at least one machine in the working group. */

};

}



#endif //AROLIB_SIMPLEHARVESTERPLANNER_H
