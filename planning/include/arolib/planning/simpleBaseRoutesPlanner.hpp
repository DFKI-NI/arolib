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
 
#ifndef AROLIB_SIMPLEHARVESTERPLANNER_H
#define AROLIB_SIMPLEHARVESTERPLANNER_H

#include "arolib/types/route.hpp"
#include "simpletracksequencer.hpp"
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
     */
    void setSubfield(arolib::Subfield subfield);

    /**
     * @brief Add a harvester to the working group
     * @param machine Harvester machine
     */
    void addMachine(arolib::Machine machine);

    /**
     * @brief Set the SequenceStrategy to be used.
     * @param strategy Sequence strategy
     */
    inline void setStrategy(SimpleTrackSequencer::SequenceStrategy strategy) {
        m_track_sequencer.setStrategy(strategy);
    }

    /**
     * @brief Set whether the thacks should be processed in inverse order or not (related to the order they are saved in the subfield's track vector).
     * @param inverse Flag stating inverse order
     */
    inline void setInverseTrackOrder(bool inverse) {
        m_track_sequencer.setInverseTrackOrder(inverse);
    }

    /**
     * @brief Set whether the first track should be worrked in inverse (points) order.
     * @param inverse Flag stating inverse order
     */
    inline void setFirstTrackInversePointOrder(bool inverse) {
        m_first_track_inverse = inverse;
    }

    /**
     * @brief Set the number of tracks per machine per bed (for SequenceStrategy = INNER_TO_OUTER/OUTER_TO_INNER).
     * @param nr Number of tracks per machine per bed
     */
    inline void setTracksPerMachine(int nr) {
        m_track_sequencer.setTracksPerMachine(nr);

    }

    /**
     * @brief Set the indexes of the tracks to be excluded.
     * @param excludeTrackIndexes Indexes of the tracks to be excluded
     */
    inline void setExcludeTrackIndexes(const std::set<size_t>& excludeTrackIndexes ) {
        m_track_sequencer.setExcludeTrackIndexes( excludeTrackIndexes );
    }

    /**
     * @brief Get the planned route of a given machine
     *
     * If the routes have not been planned yet, calling the method will compute the routes for all machines first.
     * @param machine_id Id of the machine assigned to it
     * @param [out] route Planned route
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Speed calculator
     * @return True on success
     */
    bool getRoute(int machine_id,
                  Route& route,
                  IEdgeMassCalculator & edgeMassCalculator,
                  IEdgeSpeedCalculator & edgeSpeedCalculator);

    /**
     * @brief Get the planned route of a given machine
     *
     * If the routes have not been planned yet, calling the method will compute the routes for all machines first.
     * Note: the machines used for the planning (if applicable) correspond to the ones added using 'addMachine', and not the ones in the planning workspace
     * @param machine_id Id of the machine assigned to it
     * @param [in/out] pw Planning workspace containing the planning data. Some of its fields might get updated after planning.
     * @param [out] route Planned route
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Speed calculator.
     * @return True on success
     */
    bool getRoute(int machine_id,
                  PlanningWorkspace &pw,
                  Route& route,
                  IEdgeMassCalculator & edgeMassCalculator,
                  IEdgeSpeedCalculator & edgeSpeedCalculator);

private:

    /**
     * @brief Plans and generates the harvester routes
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Speed calculator.
     * @return True on success
     */
    bool plan(IEdgeMassCalculator & edgeMassCalculator,
              IEdgeSpeedCalculator & edgeSpeedCalculator);

    /**
     * @brief Planns and generates the harvester routes
     * @param [in/out] pw Planning workspace containing the planning data. Some of its fields might get updated after planning.
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Speed calculator.
     * @return True on success
     */
    bool plan(PlanningWorkspace &pw,
              IEdgeMassCalculator & edgeMassCalculator,
              IEdgeSpeedCalculator & edgeSpeedCalculator);

    /**
     * @brief Add an inner-field inter-track (headland) connection to a route
     *
     * To be called inbetween the addition of two tracks
     * @param routeAss Route-assembler being used to generate the route
     * @param nextTrackId Id of the nex track
     * @param reverseTrack Will the next track be added in reverse order? (for TrackConnectionStrategy!=MEANDER only)
     * @param speed Machine speed in the connection
     * @param limitBoundary Limit boundary
     * @return True on success
     */
    bool addHeadlandConnection(RouteAssembler &routeAss,
                               int nextTrackId,
                               bool reverseTrack,
                               double speed,
                               const Polygon &limitBoundary);

protected:

    /**
     * @brief Flag used internally to know whether the plannning is being done using a Planning Workspace or not
     */
   enum CalcGridValueOption{
       CALC_DIRECT, /**< Make computations directly without a planning workspace */
       CALC_PW /**< Make computations using the planning workspace */
   };

   SimpleTrackSequencer m_track_sequencer; /**< Inner-field track sequencer. */
   Subfield m_subfield; /**< Working subfield. */
   std::map<MachineId_t, Machine> m_machines; /**< Working group. */
   std::map<MachineId_t, Route> m_machines_routes; /**< Planned routes. <machine id, route> */
   bool m_first_track_inverse = false; /**< Flag stating whether the tracks shoud start with inverse point orderp. */

   bool m_has_subfield; /**< Was the subfield already set?. */
   bool m_has_machine; /**< There exists at least one machine in the working group. */

   CalcGridValueOption m_calcGridValueOption = CALC_DIRECT; /**< By default = CALC_DIRECT. Change to CALC_PW (and back) done by corresponding methods */
   PlanningWorkspace* m_planningWorkspace = nullptr; /**< Pointer to the planning workspace (if NULL, no methods with a planning workspace as parameter were called) */

};

}



#endif //AROLIB_SIMPLEHARVESTERPLANNER_H
