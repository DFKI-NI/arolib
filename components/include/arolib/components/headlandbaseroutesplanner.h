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
 
#ifndef AROLIB_HEADLANDBASEROUTESPLANNER_H
#define AROLIB_HEADLANDBASEROUTESPLANNER_H

#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <math.h>
#include <string>
#include <fstream>
#include <memory>
#include <functional>

#include "arolib/misc/logger.h"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/field_geometry_processing.hpp"
#include "arolib/planning/generalplanningparameters.hpp"
#include "arolib/cartography/common.hpp"
#include "arolib/cartography/sharedgridsmanager.hpp"
#include "arolib/planning/simpleBaseRoutesPlanner.hpp"
#include "arolib/misc/basic_responses.h"

namespace arolib {

/**
 * @brief This Class plans the base-routes for headland processing like harvester routes (only headland, without the inner-field)
 */
class HeadlandBaseRoutesPlanner : public LoggingComponent, protected PlanningWorkspaceAccessor
{
public:

    /**
     * @brief Strategy used to set the order in which the machines will be assigned the tracks
     */
    enum MachineOrderStrategy{
        AUTO,/**< Let the planner select the order */
        KEEP_ORDER_RELATIVE_TO_FIRST_TRACK__STRICT,/**< The given machine order will be kept, where the first machine will be assigned the first track*/
        KEEP_ORDER_RELATIVE_TO_FIRST_WORKING_TRACK__STRICT,/**< The given machine order will be kept, where the first machine will be assigned the first track that will be worked*/
        KEEP_ORDER_RELATIVE_TO_FIRST_WORKING_TRACK,/**< If possible, the planner will select the machine to work the first track that will be worked. Afterwards, the machines will be assigned based on the given order (where the second track to be worked will be assigned to the machine following the machine assigned the first working track)*/
    };

    /**
      * @brief Get the MachineOrderStrategy (enum) from its int value
      * @brief value Int value
      * @return MachineOrderStrategy
      */
    static MachineOrderStrategy intToMachineOrderStrategy(int value);

    /**
     * @brief Restriction to plan transit segments over worked or unworked tracks
     */
    enum WorkedAreaTransitRestriction{
        NO_RESTRICTION,/**< No restrictions */
        TRANSIT_ONLY_OVER_UNWORKED_AREA,/**< The transit segments of the routes will be planned only over tracks that have not been planned to be worked */
        TRANSIT_ONLY_OVER_WORKED_AREA,/**< The transit segments of the routes will be planned only over tracks that have been planned to be worked */
        FROM_MACHINE_TYPE,/**< Select based on the machine type */
    };

    /**
      * @brief Get the WorkedAreaTransitRestriction (enum) from its int value
      * @brief value Int value
      * @return WorkedAreaTransitRestriction
      */
    static WorkedAreaTransitRestriction intToWorkedAreaTransitRestriction(int value);

    /**
     * @brief Base-routes headland planner parameters
     */
    struct PlannerParameters : public virtual FieldGeneralParameters, public virtual GridComputationSettings{
        bool startFromOutermostTrack = true; /**< Should the first track to be worked be the outer-most or the inner-most track?*/
        bool finishWithOutermostTrack = false; /**< Should the last track to be worked be the outer-most or the inner-most track? (only for partial headlands)*/
        bool clockwise = true; /**< Work the tracks in clockwise direction? (only for complete/surounding headland) */
        double speedMultiplier = 1; /**< This will be multiplied to the calculated machine working speed to obtain the speed to be used while working the headland */
        bool removeInitialWorkedSegments = false; /**< If true, the initial segments of the base routes that have been already worked will be removed; otherwise the corresponding route points will have a timestamp < 0 */
        bool restrictToBoundary = true; /**< If true, the mass calculation will check the intersection with the field outer boundary */
        bool monitorPlannedAreas = false; /**< If true, the mass calculation will check for the intersection with planned areas */
        MachineOrderStrategy machineOrderStrategy = MachineOrderStrategy::AUTO; /**< Strategy used to set the order in which the machines will be assigned the tracks */
        WorkedAreaTransitRestriction workedAreaTransitRestriction = WorkedAreaTransitRestriction::FROM_MACHINE_TYPE; /**< Restriction to plan transit segments over worked or unworked tracks (for planning on connected partial headlands) */
        PlannerParameters() = default;

        /**
         * @brief Parse the parameters from a string map, starting from a default PlannerParameters
         * @param [out] param Parameters
         * @param map String map containing the parameter values
         * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the default values will remain)
         * @return True on success
         */
        static bool parseFromStringMap( PlannerParameters& params, const std::map<std::string, std::string>& map, bool strict = false);

        /**
         * @brief Parse the parameters to a string map
         * @param param Parameters
         * @return String map containing the parameter values
         */
        static std::map<std::string, std::string> parseToStringMap( const PlannerParameters& params);
    };

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit HeadlandBaseRoutesPlanner(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Generate the headland base routes.
     *
     * The massFactorMap is a gridmap containing the factor/multiplier to be to be applied on the mass calculation (used on top of the edgeMassCalculator). It could be based on the areas that have been worked already (e.g. the inner-field), or in general a correction for the mass calculation.
     * If restrictToBoundary = true, the planner automatically factors out the areas outside of the field, hence these areas must not be factored out in this massFactorMap.
     * The massFactorMap must also factor out worked-areas corresponding to the workedAreaMap.
     * The workedAreaMap is used only to estimate the initial planning parameters for partially worked fields (incl. in which track and track-point to start working), but not for mass calculation (hence the need to include the corresponding mass factor in the massFactorMap. Likewise, the massFactorMap is not used to estimate the initial planning parameters.
     *
     * @param [in/out] subfield Subfield containing the necessary data (inc. tracks). It might be updated after the planning.
     * @param workinggroup Machines used for planning
     * @param plannerParameters Planner parameters
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Working speed calculator
     * @param [out] routes Resulting planned routes
     * @param [in/out] massFactorMap Gridmap containing the factor/multiplier to be to be applied on the mass calculation (used on top of the edgeMassCalculator). If set, it will be updated based on the resulting base routes (the cells corresponding to the routes will be set to 0.0). If set but not allocated, it will be initialized by the planner. See method description for more information
     * @param machineCurrentStates (optional) Machine current states used to decide where to start working (disregarded if null).
     * @param initRefPose (optional) Reference pose to decide where to start working (disregarded if null or if the starting point can be obtained from the workedAreaMap or the machineCurrentStates).
     * @param outFieldInfo (optional) Out-of-field information used to decide where to start working (disregarded if null, or if initRefPoint was given, or if the starting point can be obtained from the workedAreaMap or the machineCurrentStates).
     * @param remainingAreaMap (optional) Remaining-area map/grid, where cell values of 1 := unworked; 0 := worked; noValue := not worked. If null -> the whole area is considered unworked. See method description for more information.
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp plan(const Subfield &subfield,
                 const std::vector<Machine> &workinggroup,
                 const PlannerParameters & plannerParameters,
                 std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                 std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                 std::vector<Route> & routes,
                 std::shared_ptr<ArolibGrid_t> massFactorMap = nullptr,
                 const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates = nullptr,
                 const Pose2D * initRefPose = nullptr,
                 const OutFieldInfo* outFieldInfo = nullptr,
                 std::shared_ptr<const ArolibGrid_t> remainingAreaMap = nullptr);

    /**
     * @brief Set shared CellsInfoManager to record cells data
     * @param cim CellsInfoManager.
     */
    virtual void setGridCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim);


    /**
     * @brief Get the WorkedAreaTransitRestriction to be used in planning based on the griven WorkedAreaTransitRestriction setting and the machine types
     * @param war User WorkedAreaTransitRestriction setting.
     * @param machines machines.
     * @return WorkedAreaTransitRestriction to be used in planning.
     */
    static WorkedAreaTransitRestriction getWorkedAreaTransitRestriction(const WorkedAreaTransitRestriction & war, const std::map<MachineId_t, Machine> & machines );

protected:

    /**
     * @brief General internal headland parameters (for both complete and partial headlands)
     */
    struct InternalParametersGeneral
    {
        gridmap::SharedGridsManager gm; /**< Gridmaps manager (holding the needed gridmaps) */
        std::shared_ptr<ArolibGrid_t> massFactorMapWA; /**< Mass-factor gridmap (worked area) */
        gridmap::SharedGridsManager::PreciseCalculationOption precision_wam = gridmap::SharedGridsManager::PRECISE_ONLY_IF_AVAILABLE; /**< Precision used for computations with the worked/remaining-area map */
        gridmap::SharedGridsManager::PreciseCalculationOption precision_massMap = gridmap::SharedGridsManager::PRECISE_ONLY_IF_AVAILABLE; /**< Precision used for computations with the mass map */
        WorkedAreaTransitRestriction workedAreaTransitRestriction; /**< WorkedAreaTransitRestriction */
        std::vector<PolygonWithHoles> workedPolys; /**< Vector holding the polygong that are considered to be fully worked */
        const Polygon* boundary = nullptr; /**< Boundary used to obtain the mass */
    };

    /**
     * @brief Internal parameters for planning on complete headland
     */
    struct InternalParametersForCompleteHL: public InternalParametersGeneral
    {
        bool keepGivenClockwise = true; /**< Flag stating whether the user clockwise setting must be respected or not */
        bool reverseTracksOrder; /**< Flag stating whether the tracks must be analized in reverse order or not (w.r.t. the order in which they are in the respective headland tracks' vector) */
        size_t indFirstTrack; /**< Index of the first track of the process (might be a completelly worked track) */
        size_t indFirstTrackFirstPoint; /**< Index of the first point of the first track of the process (might be a completelly worked track) */
        size_t indFirstWorkingTrack; /**< Index of the first track that needs to be worked */
        size_t indFirstWorkingTrackPoint; /**< Index of the first point that needs to be worked, of the first track that needs to be worked */
        size_t indFirstWorkingTrackFirstPoint; /**< Index of the first point of the first track that needs to be worked */
        bool clockwise; /**< Flag stating whether the headland will be worke clockwise or not */
        bool trackIsClockwise; /**< Flag stating whether the first track to be worked is clockwise or not */
        bool trackIsPartiallyWorked = false; /**< Flag stating whether the first track to be worked is partially worked or not */
        MachineId_t startingMachineId; /**< Id of the machine working the first track to be worked */
    };


    /**
     * @brief Internal parameters for planning on partial headlands
     */
    struct InternalParametersForPartialHLs: public InternalParametersGeneral
    {
        /**
         * @brief Partial headland states
         */
        enum HeadlandState{
            PHL_WORKED,
            PHL_NOT_WORKED,
            PHL_PARTIALLY_WORKED
        };

        /**
         * @brief Partial headland Track states
         */
        enum TrackState{
            TRACK_WORKED,
            TRACK_NOT_WORKED,
            TRACK_PARTIALLY_WORKED
        };

        /**
         * @brief Internal base parameters for a partial headland
         */
        struct HLParamsBase{
            size_t indFirstTrack; /**< Index of the first track of the working process for this headland (might be a completelly worked track) */
            size_t indFirstWorkingTrack; /**< Index of the first track to be worked in this headland */
            size_t indFirstWorkingTrackPoint; /**< Index of the first point of the first track to be worked in this headland */
            size_t indFirstWorkingTrackPointToWork; /**< Index of the first point to be worked of the first track to be worked in this headland */
            bool firstWorkingTrackInReverse; /**< Flag stating whether the first track to be worked must be worked in reverse (point) order or not */
            int indTransitTrackStart = -1; /**< Index corresponding to the transit track that must be driven before starting to work the headland */
            int indTransitTrackFinish = -1; /**< Index corresponding to the transit track that must be driven after working the headland (to connect to the next headland) */
            int nextWorkingHL = -1; /**< Index of the next headland to be worked */
            bool trackIsPartiallyWorked = false; /**< Flag stating whether the first track to be worked is partially worked or not */
        };

        /**
         * @brief Internal parameters for a partial headland
         */
        struct HLParams : public HLParamsBase{
            friend struct arolib::HeadlandBaseRoutesPlanner::InternalParametersForPartialHLs;
            bool tracksInwards; /**< Flag stating whether the thatck of the partial headland are in inwards order or not */
            std::vector<bool> trackDirectionFlags; /**< Flags stating whether the track points are in reverse direction (true) or the same direction (false) as the points in track[0] */
            bool startFromOutermostTrack; /**< Flag stating whether the headland is worked starting from the outermost track or the innermost track */
            HeadlandState state;  /**< Working state of the headland */
            std::vector<TrackState> trackStates;  /**< Working states of the tracks */
            std::vector<size_t> tracksToWork;  /**< Indexes of the tracks that have to be worked (following the order of headland.tracks) */
            std::map<size_t, std::pair<size_t, size_t>> partiallyWorkedTrackPointsInds;  /**< Start and end point indexes of the segment of the track that sill have to be worked */

            /**
             * @brief Update the parameters for a partial headland from some given base parameters
             * @param base Base parameters
             */
            void update(const HLParamsBase& base);
        };

        /**
         * @brief Parameters of the different options for potential first working headlands
         */
        struct FirstWorkingHeadlandOptionParams{
            size_t indHL; /**< Headland index */
            size_t indFirstWorkingTrack; /**< Index of the first working track (in the process, might be already worked) */
            size_t indLastWorkingTrack; /**< Index of the last working track */
            size_t indFirstTrackToWork; /**< Index of the first track to be worked */
            size_t indFirstTrack; /**< Index of the first track in the process (might be transit) */
            size_t indLastTrack; /**< Index of the last track in the process (might be transit) */
            bool withStartTransitTrack; /**< Flag stating if the option includes transit track before starting working the headland */
            bool withEndTransitTrack; /**< Flag stating if the includes a transit track after starting the headland to connect with the next headland */
            bool firstWorkingTrackInReverse; /**< Flag stating if the first first working track is in reverse (point) order */
            bool firstTrackToWorkInReverse; /**< Flag stating if the first track to work is in reverse (point) order */
            int side; /**< Side of the headland being checked (0: starting at firstTrack.points[0]; 1: starting at firstTrack.points[end] */
            bool finishesInSameSide; /**< Flag stating if the process in the headland would finish in the same side where it started */
            size_t indFirstTrackPt; /**< Index FirstTrack of the first point in the process (might be transit) */
            size_t indLastTrackPt; /**< Index in LastTrack of the last point in the process (might be transit) */
            size_t indFirstPointToWork; /**< Index of the first point to be worked in firstTrackToWork */
            size_t indHLNext; /**< Index of the next headland */
            bool hlNextIsWorked; /**< Flag stating if the next headland is completelly worked */
            int indHLPrev; /**< Index of the previous headland */
        };


        int indFirstHeadland = -1; /**< Index of the first headland of the process (might be a completelly worked headland) */
        int indFirstWorkingHeadland = -1; /**< Index of the first headland to be worked */
        std::vector<size_t> sortedPartialHLs; /**< Vector holding the indexes of the partial headlands sorted in order of work */
        std::vector<size_t> potentialFirstWorkingHeadlands; /**< Indexes of the headlands that are potentially the first headland to be worked */
        std::vector<HLParams> hlParams; /**< Internal parameters of all partial headlands */
    };

    /**
     * @brief Get the working group as a id-machine map (holding only working-type machines)
     * @param workinggroup Machines
     */
    std::map<MachineId_t, Machine> getWorkingGroup(const std::vector<Machine> &workinggroup);

    /**
     * @brief Generate the headland base routes for complete headland.
     * @param subfield Subfield containing the necessary data (inc. tracks).
     * @param workinggroup Machines used for planning
     * @param machines Working-type machines
     * @param plannerParameters Planner parameters
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Working speed calculator
     * @param initRefPose (optional) Reference pose to decide where to start working (disregarded if null or if the starting point can be obtained from the workedAreaMap or the machineCurrentStates).
     * @param machineCurrentStates (optional) Machine current states used to decide where to start working (disregarded if null).
     * @param outFieldInfo (optional) Out-of-field information used to decide where to start working (disregarded if null, or if initRefPoint was given, or if the starting point can be obtained from the workedAreaMap or the machineCurrentStates).
     * @param [in/out] ip Internal planning parameters.
     * @param [out] routes Resulting planned routes
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp planForCompleteHL(const Subfield &subfield,
                              const std::vector<Machine> &workinggroup,
                              const std::map<MachineId_t, Machine> &machines,
                              const PlannerParameters & plannerParameters,
                              IEdgeMassCalculator & edgeMassCalculator,
                              IEdgeSpeedCalculator & edgeSpeedCalculator,
                              const Pose2D * initRefPose,
                              const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                              const OutFieldInfo* outFieldInfo,
                              InternalParametersForCompleteHL & ip,
                              std::vector<Route> & routes);

    /**
     * @brief Generate the headland base routes for partial headlands.
     * @param subfield Subfield containing the necessary data (inc. tracks).
     * @param machine Working machine
     * @param plannerParameters Planner parameters
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Working speed calculator
     * @param initRefPose (optional) Reference pose to decide where to start working (disregarded if null or if the starting point can be obtained from the workedAreaMap or the machineCurrentStates).
     * @param machineCurrentStates (optional) Machine current states used to decide where to start working (disregarded if null).
     * @param outFieldInfo (optional) Out-of-field information used to decide where to start working (disregarded if null, or if initRefPoint was given, or if the starting point can be obtained from the workedAreaMap or the machineCurrentStates).
     * @param [in/out] ip Internal planning parameters.
     * @param [out] routes Resulting planned routes
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp planForPartialHLs(const Subfield &subfield,
                              const Machine &machine,
                              const PlannerParameters & plannerParameters,
                              IEdgeMassCalculator & edgeMassCalculator,
                              IEdgeSpeedCalculator & edgeSpeedCalculator,
                              const Pose2D * initRefPose,
                              const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                              const OutFieldInfo* outFieldInfo,
                              InternalParametersForPartialHLs &ip,
                              std::vector<Route> & routes);

    /**
     * @brief Initialize the internal (general) planning parameters.
     *
     * Initializes the grid manager with the avalable grids, updates the workedAreaTransitRestriction to be used in the rest of planning, and initializes the boundary for mass computations
     *
     * @param [in/out] ip Internal planning parameters.
     * @param subfield Subfield containing the necessary data (inc. tracks).
     * @param machines Working machines
     * @param plannerParameters Planner parameters
     * @param [in/out] massFactorMap Gridmap containing the factor/multiplier to be to be applied on the mass calculation (used on top of the edgeMassCalculator). If set, it will be updated based on the resulting base routes (the cells corresponding to the routes will be set to 0.0). If set but not allocated, it will be initialized by the planner. See method description for more information
     * @param remainingAreaMap (optional) Remaining-area map/grid, where cell values of 1 := unworked; 0 := worked; noValue := not worked. If null -> the whole area is considered unworked. See method description for more information.
     * @param lh LoggersHandler
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp initInternalGeneralParameters(InternalParametersGeneral& ip,
                                          const Subfield &subfield,
                                          const std::map<MachineId_t, Machine> &machines,
                                          const PlannerParameters & plannerParameters,
                                          std::shared_ptr<ArolibGrid_t> massFactorMap,
                                          std::shared_ptr<const ArolibGrid_t> remainingAreaMap,
                                          LoggersHandler &lh);

    /**
     * @brief Get the starting internal planning parameters (for complete headland).
     *
     * Initializes the following parameters of InternalParametersForCompleteHL:
     *      * First track in the process: indFirstTrack, indFirstTrackFirstPoint, indFirstTrackFirstPoint
     *      * First track to be worked: indFirstWorkingTrack, trackIsPartiallyWorked, trackIsClockwise, indFirstWorkingTrackFirstPoint
     *      * Headland working direction: clockwise
     *      * Other: startingMachineId
     *
     * @param subfield Subfield containing the necessary data (inc. tracks).
     * @param machines Working-type machines
     * @param plannerParameters Planner parameters
     * @param pInitRefPose (optional) Reference pose to decide where to start working .
     * @param machineCurrentStates (optional) Machine current states used to decide where to start working (disregarded if null).
     * @param outFieldInfo (optional) Out-of-field information used to decide where to start working (disregarded if null, or if initRefPoint was given, or if the starting point can be obtained from the workedAreaMap or the machineCurrentStates).
     * @param [in/out] ip Internal planning parameters.
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp getStartingParameters_completeHL(const Subfield &subfield,
                                             const std::map<MachineId_t, Machine> &machines,
                                             const PlannerParameters & plannerParameters,
                                             const Pose2D * pInitRefPose,
                                             const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                             const OutFieldInfo* outFieldInfo,
                                             InternalParametersForCompleteHL & ip);

    /**
     * @brief Get the starting internal planning parameters (for complete headland) based on the worked area.
     *
     * Initializes the following parameters of InternalParametersForCompleteHL: ip.indFirstWorkingTrack
     * @param tracks Headland tracks.
     * @param machines Working-type machines
     * @param plannerParameters Planner parameters
     * @param [in/out] ip Internal planning parameters.
     * @param [out] potentialFirstWorkingTrackPointIdxs Vector containing the indexes of the points which can be considered as potential start working point (of the FirstWorkingTrack).
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp getStartingParameters_completeHL_workedArea(const std::vector<Track> &tracks,
                                                        const std::map<MachineId_t, Machine> &machines,
                                                        const PlannerParameters & plannerParameters,
                                                        InternalParametersForCompleteHL & ip,
                                                        std::vector<size_t>& potentialFirstWorkingTrackPointIdxs);

    /**
     * @brief Get the index of the first point to be worked from the first track to be worked (from a vector of potentiall FirstWorkingTrackPoint indexes) in complete headland based on the current states of the machines.
     * @param subfield Subfield.
     * @param firstWorkingTrack First working track.
     * @param machines Working-type machines
     * @param machineCurrentStates (optional) Machine current states used to decide where to start working (disregarded if null).
     * @param [out] refPoseFromMachine Reference initial pose obtained from the initial machine states.
     * @param potentialFirstWorkingTrackPointIdxs Vector containing the indexes of the points which can be considered as potential start working point.
     * @param [in/out] ip Internal planning parameters.
     * @param [out] indStartingPoint Index of potentialFirstWorkingTrackPointIdxs (!) corresponding to the first point to be worked (<0 if not found).
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp getStartingParameters_completeHL_machineStates(const Subfield &subfield,
                                                           const Track& firstWorkingTrack,
                                                           const std::map<MachineId_t, Machine> &machines,
                                                           const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                           Pose2D& refPoseFromMachine,
                                                           const std::vector<size_t> &potentialFirstWorkingTrackPointIdxs,
                                                           InternalParametersForCompleteHL & ip,
                                                           int & indStartingPoint);

    /**
     * @brief Get the index of the first point to be worked from the first track to be worked (from a vector of potentiall FirstWorkingTrackPoint indexes) in complete headland based on a reference initial pose.
     * @param firstWorkingTrack First working track.
     * @param machines Working-type machines
     * @param initRefPose Reference pose to decide where to start working.
     * @param potentialFirstWorkingTrackPointIdxs Vector containing the indexes of the points which can be considered as potential start working point.
     * @param [in/out] ip Internal planning parameters.
     * @param [out] indStartingPoint Index of potentialFirstWorkingTrackPointIdxs (!) corresponding to the first point to be worked (<0 if not found).
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp getStartingParameters_completeHL_refPose(const Track& firstWorkingTrack,
                                                     const std::map<MachineId_t, Machine> &machines,
                                                     const Pose2D& initRefPose,
                                                     const std::vector<size_t>& potentialFirstWorkingTrackPointIdxs,
                                                     InternalParametersForCompleteHL & ip,
                                                     int & indStartingPoint);

    /**
     * @brief Get the index of the first point to be worked from the first track to be worked (from a vector of potentiall FirstWorkingTrackPoint indexes) in complete headland based on the access points and out-of-field information.
     * @param subfield Subfield.
     * @param firstWorkingTrack First working track.
     * @param machines Working-type machines
     * @param machineCurrentStates (optional) Machine current states used to decide where to start working (disregarded if null).
     * @param outFieldInfo (optional) Out-of-field information used to decide where to start working (disregarded if null, or if initRefPoint was given, or if the starting point can be obtained from the workedAreaMap or the machineCurrentStates).
     * @param potentialFirstWorkingTrackPointIdxs Vector containing the indexes of the points which can be considered as potential start working point.
     * @param [in/out] ip Internal planning parameters.
     * @param [out] indStartingPoint Index of potentialFirstWorkingTrackPointIdxs (!) corresponding to the first point to be worked (<0 if not found).
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp getStartingParameters_completeHL_outFieldInfo(const Subfield &subfield,
                                                          const Track& firstWorkingTrack,
                                                          const std::map<MachineId_t, Machine> &machines,
                                                          const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                                          const OutFieldInfo& outFieldInfo,
                                                          const std::vector<size_t> &potentialFirstWorkingTrackPointIdxs,
                                                          InternalParametersForCompleteHL & ip,
                                                          int & indStartingPoint);

    /**
     * @brief Initialize internal planning parameters related to starting-points (for complete headland).
     *
     * Initializes the following parameters of InternalParametersForCompleteHL: indFirstWorkingTrackFirstPoint and indFirstTrackFirstPoint
     *
     * @param subfield Subfield.
     * @param potentialFirstWorkingTrackPointIdxs Vector containing the indexes of the points which can be considered as potential start working point.
     * @param indStartingPoint Index of potentialFirstWorkingTrackPointIdxs (!) corresponding to the first point to be worked.
     * @param [in/out] ip Internal planning parameters.
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp getStartingTrackPointIdx_completeHL(const Subfield &subfield,
                                                const std::vector<size_t> &potentialFirstWorkingTrackPointIdxs,
                                                size_t indStartingPoint,
                                                InternalParametersForCompleteHL & ip);

    /*
     * @brief Get the machine ids sorted in the order in which they will work the tracks (relative to the first track, not to the firts track to be worked)
     * @param subfield Subfield.
     * @param workinggroup workinggroup.
     * @param machines Working-type machines.
     * @param machineOrderStrategy Machine order strategy
     * @param ip Internal planning parameters.
     * @param machineCurrentStates (optional) Machine current states used to decide where to start working (disregarded if null).
     * @param outFieldInfo (optional) Out-of-field information used to decide where to start working (disregarded if null, or if initRefPoint was given, or if the starting point can be obtained from the workedAreaMap or the machineCurrentStates).
     * @return Sorted machine ids
    */
    std::vector<MachineId_t> sortMachines_completeHL(const Subfield &subfield,
                                                     const std::vector<Machine> &workinggroup,
                                                     const std::map<MachineId_t, Machine> &machines,
                                                     MachineOrderStrategy machineOrderStrategy,
                                                     const InternalParametersForCompleteHL & ip,
                                                     const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                                     const OutFieldInfo* outFieldInfo);

    /*
     * @brief Generate the base routes for complete headland based on previously computed internal parameters (for complete headland)
     * @param subfield Subfield.
     * @param machines Working-type machines.
     * @param sortedMachineIds Machine ids sorted in the order in which they will work the tracks (relative to the first track, not to the firts track to be worked)
     * @param plannerParameters Planner parameters
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Working speed calculator
     * @param machineCurrentStates (optional) Machine current states used to decide where to start working (disregarded if null).
     * @param [in/out] ip Internal planning parameters.
     * @param [out] routes Resulting planned routes
     * @return AroResp with error id (0:=OK) and message
    */
    AroResp generateRoutes_completeHL(const Subfield &subfield,
                                      const std::map<MachineId_t, Machine> &machines,
                                      const std::vector<MachineId_t> &sortedMachineIds,
                                      const PlannerParameters & plannerParameters,
                                      IEdgeMassCalculator & edgeMassCalculator,
                                      IEdgeSpeedCalculator & edgeSpeedCalculator,
                                      const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                      InternalParametersForCompleteHL &ip,
                                      std::vector<Route> & routes);

    /*
     * @brief Add a route segment coresponding to a given complete-headland track to the base route
     *
     * @param [in/out] route Base route
     * @param machine Working machine.
     * @param track Track
     * @param from Index of the first track point to be added
     * @param to Index of the last track point to be added (if <1 -> last point in the track)
     * @param isWorking Flag stating if the segment if a working segment
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Working speed calculator
     * @param [in/out] ip Internal planning parameters.
     * @param plannerParameters Planner parameters
     * @param machineCurrentStates (optional) Machine current states used to decide where to start working (disregarded if null).
    */
    void addTrackPointsToRoute_completeHL(Route &route,
                                          const Machine& machine,
                                          const Track &track, int indFrom, int indTo,
                                          bool isWorking,
                                          IEdgeMassCalculator & edgeMassCalculator,
                                          IEdgeSpeedCalculator & edgeSpeedCalculator,
                                          InternalParametersForCompleteHL &ip,
                                          const PlannerParameters &plannerParameters,
                                          const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates);

    /*
     * @brief Initialize the partial headland internal parameters
     *
     * Initializes the following HLParams of InternalParametersForPartialHLs for all headlands
     *      * tracksInwards
     *      * trackDirectionFlags
     *      * HeadlandState state
     *      * trackStates
     *
     * @param subfield Subfield.
     * @param [in/out] ip Internal planning parameters.
     * @return AroResp with error id (0:=OK) and message
    */
    AroResp initPartialHeadlandsParams(const Subfield &subfield,
                                       InternalParametersForPartialHLs &ip);

    /*
     * @brief Pre-sorts the partial headland indexes to obtain the order in which all headlands can be travrsed based on their connections
     *
     * Initializes the following parameters of InternalParametersForPartialHLs: sortedPartialHLs
     * @param subfield Subfield.
     * @param [in/out] ip Internal planning parameters.
     * @return AroResp with error id (0:=OK) and message
    */
    AroResp presortPartialHeadlands(const Subfield &subfield,
                                   InternalParametersForPartialHLs &ip);

    /**
     * @brief Get the starting internal planning parameters (for partial headlands).
     *
     * Initializes the following parameters of InternalParametersForPartialHLs:
     *      * General: potentialFirstWorkingHeadlands, indFirstWorkingHeadland,
     *      * Initializes the following parameters of HLParams for the first headland to be worked:
     *          * First track in the process: indFirstTrack,
     *          * First track to be worked: indFirstWorkingTrack, firstWorkingTrackInReverse, indFirstWorkingTrackPoint
     *          * Transit (non-working) tracks: indTransitTrackStart, indTransitTrackFinish
     *          * General: nextWorkingHL
     *
     * @param subfield Subfield containing the necessary data (inc. tracks).
     * @param machine Working machine
     * @param plannerParameters Planner parameters
     * @param pInitRefPose (optional) Reference pose to decide where to start working .
     * @param machineCurrentStates (optional) Machine current states used to decide where to start working (disregarded if null).
     * @param outFieldInfo (optional) Out-of-field information used to decide where to start working (disregarded if null, or if initRefPoint was given, or if the starting point can be obtained from the workedAreaMap or the machineCurrentStates).
     * @param [in/out] ip Internal planning parameters.
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp getStartingParameters_partialHLs(const Subfield &subfield,
                                             const Machine &machine,
                                             const PlannerParameters & plannerParameters,
                                             const Pose2D * pInitRefPose,
                                             const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                             const OutFieldInfo* outFieldInfo,
                                             InternalParametersForPartialHLs & ip);


    /**
     * @brief Get the starting internal planning parameters (for partial headlands).
     *
     * Initializes the following parameters of InternalParametersForPartialHLs:
     *      * General: potentialFirstWorkingHeadlands, indFirstWorkingHeadland,
     *      * Initializes the following parameters of HLParams for the first headland to be worked:
     *          * First track in the process: indFirstTrack,
     *          * First track to be worked: indFirstWorkingTrack, firstWorkingTrackInReverse, indFirstWorkingTrackPoint
     *          * Transit (non-working) tracks: indTransitTrackStart, indTransitTrackFinish
     *          * General: nextWorkingHL
     *
     * @param subfield Subfield containing the necessary data (inc. tracks).
     * @param machine Working machine
     * @param plannerParameters Planner parameters
     * @param pInitRefPose (optional) Reference pose to decide where to start working .
     * @param machineCurrentStates (optional) Machine current states used to decide where to start working (disregarded if null).
     * @param outFieldInfo (optional) Out-of-field information used to decide where to start working (disregarded if null, or if initRefPoint was given, or if the starting point can be obtained from the workedAreaMap or the machineCurrentStates).
     * @param [in/out] ip Internal planning parameters.
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp getFirstWorkingHeadlandParameters(const Subfield &subfield,
                                               const Machine &machine,
                                               const PlannerParameters & plannerParameters,
                                               const Pose2D * pInitRefPose,
                                               const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                               const OutFieldInfo* outFieldInfo,
                                               InternalParametersForPartialHLs & ip);

    /**
     * @brief Get the starting internal planning parameters (for partial headlands) based on the initial machine state.
     *
     * On success, initializes the following parameters of InternalParametersForPartialHLs:
     *      * General: potentialFirstWorkingHeadlands, indFirstWorkingHeadland,
     *      * Initializes the following parameters of HLParams for the first headland to be worked:
     *          * First track in the process: indFirstTrack,
     *          * First track to be worked: indFirstWorkingTrack, firstWorkingTrackInReverse, indFirstWorkingTrackPoint
     *          * Transit (non-working) tracks: indTransitTrackStart, indTransitTrackFinish
     *          * General: nextWorkingHL
     *
     * @param subfield Subfield containing the necessary data (inc. tracks).
     * @param machine Working machine
     * @param plannerParameters Planner parameters
     * @param pInitRefPose (optional) Reference pose to decide where to start working.
     * @param machineCurrentStates Machine current states used to decide where to start working (disregarded if null).
     * @param [in/out] ip Internal planning parameters.
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp getFirstWorkingHeadlandParameters_machineState(const Subfield &subfield,
                                                           const Machine &machine,
                                                           const PlannerParameters & plannerParameters,
                                                           const Pose2D * pInitRefPose,
                                                           const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                           InternalParametersForPartialHLs &ip);

    /**
     * @brief Get the starting internal planning parameters (for partial headlands) based on the initial referense pose.
     *
     * On success, initializes the following parameters of InternalParametersForPartialHLs:
     *      * General: potentialFirstWorkingHeadlands, indFirstWorkingHeadland,
     *      * Initializes the following parameters of HLParams for the first headland to be worked:
     *          * First track in the process: indFirstTrack,
     *          * First track to be worked: indFirstWorkingTrack, firstWorkingTrackInReverse, indFirstWorkingTrackPoint
     *          * Transit (non-working) tracks: indTransitTrackStart, indTransitTrackFinish
     *          * General: nextWorkingHL
     *
     * @param subfield Subfield containing the necessary data (inc. tracks).
     * @param machine Working machine
     * @param plannerParameters Planner parameters
     * @param initRefPose Reference pose to decide where to start working.
     * @param [in/out] ip Internal planning parameters.
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp getFirstWorkingHeadlandParameters_refPose(const Subfield &subfield,
                                                      const Machine &machine,
                                                      const PlannerParameters & plannerParameters,
                                                      const Pose2D &initRefPose,
                                                      InternalParametersForPartialHLs & ip);

    /**
     * @brief Get the starting internal planning parameters (for partial headlands) based on the field access points and the out-of-field information.
     *
     * On success, initializes the following parameters of InternalParametersForPartialHLs:
     *      * General: potentialFirstWorkingHeadlands, indFirstWorkingHeadland,
     *      * Initializes the following parameters of HLParams for the first headland to be worked:
     *          * First track in the process: indFirstTrack,
     *          * First track to be worked: indFirstWorkingTrack, firstWorkingTrackInReverse, indFirstWorkingTrackPoint
     *          * Transit (non-working) tracks: indTransitTrackStart, indTransitTrackFinish
     *          * General: nextWorkingHL
     *
     * @param subfield Subfield containing the necessary data (inc. tracks).
     * @param machine Working machine
     * @param plannerParameters Planner parameters
     * @param pInitRefPose (optional) Reference pose to decide where to start working.
     * @param machineCurrentStates (optional) Machine current states used to decide where to start working (disregarded if null).
     * @param outFieldInfo (optional) Out-of-field information used to decide where to start working (disregarded if null, or if initRefPoint was given, or if the starting point can be obtained from the workedAreaMap or the machineCurrentStates).
     * @param [in/out] ip Internal planning parameters.
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp getFirstWorkingHeadlandParameters_outFieldInfo(const Subfield &subfield,
                                                           const Machine &machine,
                                                           const PlannerParameters & plannerParameters,
                                                           const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                                           const OutFieldInfo& outFieldInfo,
                                                           InternalParametersForPartialHLs & ip);

    /**
     * @brief Get all options of starting parameters for first working headland from the list of pre-computed PotentialFirstWorking headlands.
     * @param subfield Subfield containing the necessary data (inc. tracks).
     * @param plannerParameters Planner parameters
     * @param startHLFromOutermostTrack startHLFromOutermostTrack
     * @param [in/out] ip Internal planning parameters.
     * @param cb Callback sending the parameters for a given option.
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp checkPotentialFirstWorkingHLsOptions(const Subfield &subfield,
                                                 const PlannerParameters & plannerParameters,
                                                 bool startHLFromOutermostTrack,
                                                 InternalParametersForPartialHLs & ip,
                                                 const std::function<void(InternalParametersForPartialHLs::FirstWorkingHeadlandOptionParams& optionParams)> &cb);

    /**
     * @brief Update the headland parameters (HLParams) for the first headland to be worked.
     *
     * @param subfield Subfield containing the necessary data (inc. tracks).
     * @param indHL Index of the headland to be updated
     * @param ip Internal planning parameters.
     * @param [in/out] hlParams HLParams of the headland to be updated.
     * @param [in/out] firstWorkingTrackInd Index of the first working track
     * @param [in/out] firstTrackToWorkInd Index of the first track to be worked
     * @param [in/out] firstTrackInd Index of the first track in the process
     * @param [in/out] lastTrackInd Index of the last track in the working process
     * @param [in/out] withStartTransitTrack Flag stating if a transit track is needed before working the headland
     * @param [in/out] withEndTransitTrack  Flag stating if a transit track is needed after working the headland
     * @param [in/out] firstTrackToWorkInReverse Flag stating if the first track to be worked worked in reverse (point) order
     * @param [in/out] indFirstWorkingTrackPointToWork Index of the first point to be worked
     * @param [in/out] indHLNext Index of the headland to be worked after the given headland
     * @return AroResp with error id (0:=OK) and message
     */
    void updateHLParamsForFirstWorkingHeadland(const Subfield &subfield,
                                               size_t indHL,
                                               const InternalParametersForPartialHLs & ip,
                                               InternalParametersForPartialHLs::HLParamsBase& hlParams,
                                               size_t firstWorkingTrackInd,
                                               size_t firstTrackToWorkInd,
                                               size_t firstTrackInd,
                                               size_t lastTrackInd,
                                               bool withStartTransitTrack,
                                               bool withEndTransitTrack,
                                               size_t indFirstWorkingTrackPointToWork,
                                               bool firstTrackToWorkInReverse,
                                               size_t indHLNext);

    /**
     * @brief Update the headland parameters (HLParams) for all headlands (except the headland to be worked first).
     *
     * @param subfield Subfield containing the necessary data (inc. tracks).
     * @param plannerParameters Planner parameters
     * @param [in/out] ip Internal planning parameters.
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp updateHLParamsForNonFirstWorkingHeadlands(const Subfield &subfield,
                                                      const PlannerParameters & plannerParameters,
                                                      InternalParametersForPartialHLs & ip);

    /**
     * @brief Get the index of the first track to be worked in a (completelly) non-worked HL
     *
     * @param subfield Subfield
     * @param indHL Index of the headland
     * @param ip Internal planning parameters.
     * @param startHLFromOutermostTrack startHLFromOutermostTrack
     * @param onlyToBeWorked If true, it will exclude initial tracks in the precess which are worked already
     * @return Index of the first track to be worked (<0 if all tracks are worked)
     */
    static int getFirstPartialHeadlandWorkingTrackInd(const Subfield &subfield, size_t indHL, const InternalParametersForPartialHLs & ip, bool startHLFromOutermostTrack, bool onlyToBeWorked);

    /**
     * @brief Get the index of the last track to be worked in a (completelly) non-worked HL
     *
     * @param indHL Index of the headland
     * @param ip Internal planning parameters.
     * @param startHLFromOutermostTrack startHLFromOutermostTrack
     * @param plannerParameters Planner parameters
     */
    static int getLastPartialHeadlandWorkingTrackInd(size_t indHL, const InternalParametersForPartialHLs & ip, bool startHLFromOutermostTrack);

    /**
     * @brief Get the index of the transit track to be driven before/after a given working track
     *
     * @param subfield Subfield containing the necessary data (inc. tracks).
     * @param ip Internal planning parameters.
     * @param indHL Index of the headland
     * @param indWorkingTrackRef Index of the (reference) working track
     * @return Index of the transit track to be driven before/after a given working track
     */
    static size_t getPartialHeadlandTransitTrackInd(const Subfield &subfield,
                                                    const InternalParametersForPartialHLs &ip,
                                                    size_t indHL,
                                                    size_t indWorkingTrackRef);

    /**
     * @brief Find the 1st headland in the process based on the 1st working headland, following the order of the presorted sequence ip.sortedPartialHLs. The sequence ip.sortedPartialHLs will be updated to the final sequence.
     * @param subfield Subfield containing the necessary data (inc. tracks).
     * @param [in/out] ip Internal planning parameters.
     */
    AroResp getFirstHeadlandAndSortHeadlands(const Subfield &subfield,
                                             InternalParametersForPartialHLs & ip);


    /*
     * @brief Generate the base route for the partial headland based on previously computed internal parameters
     * @param subfield Subfield.
     * @param machine Working machine.
     * @param plannerParameters Planner parameters
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Working speed calculator
     * @param machineCurrentStates (optional) Machine current states used to decide where to start working (disregarded if null).
     * @param [in/out] ip Internal planning parameters.
     * @param [out] route Resulting planned route
     * @return AroResp with error id (0:=OK) and message
    */
    AroResp generateRoute_partialHLs(const Subfield &subfield,
                                      const Machine &machine,
                                      const PlannerParameters & plannerParameters,
                                      IEdgeMassCalculator & edgeMassCalculator,
                                      IEdgeSpeedCalculator & edgeSpeedCalculator,
                                      const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                      InternalParametersForPartialHLs &ip,
                                      Route & route);


    /*
     * @brief Add a route segment coresponding to a given headland to the base route
     * @param subfield Subfield.
     * @param machine Working machine.
     * @param indHL Index of the headland
     * @param plannerParameters Planner parameters
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Working speed calculator
     * @param machineCurrentStates (optional) Machine current states used to decide where to start working (disregarded if null).
     * @param [in/out] ip Internal planning parameters.
     * @param [in/out] route Base route
     * @return AroResp with error id (0:=OK) and message
    */
    AroResp addRouteSegments_partialHL(const Subfield &subfield,
                                       const Machine &machine,
                                       size_t indHL,
                                       const PlannerParameters & plannerParameters,
                                       IEdgeMassCalculator & edgeMassCalculator,
                                       IEdgeSpeedCalculator & edgeSpeedCalculator,
                                       const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                       InternalParametersForPartialHLs &ip,
                                       Route & route);

    /*
     * @brief Add a route segment coresponding to a given partial-headland track to the base route
     *
     * @param [in/out] route Base route
     * @param machine Working machine.
     * @param track Track
     * @param indFrom Index of the first track point to be added
     * @param indTo Index of the last track point to be added (if < indFrom, the points are added in reverse)
     * @param isWorking Flag stating if the segment if a working segment
     * @param isTransit Flag stating if the segment if a transit segment
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Working speed calculator
     * @param [in/out] ip Internal planning parameters.
     * @param plannerParameters Planner parameters
     * @param machineCurrentStates (optional) Machine current states used to decide where to start working (disregarded if null).
     * @param extraSpeedFactor (extra) factor to be applied to the speed in that segment.
    */
    void addTrackPointsToRoute_partialHLs(Route& route,
                                          const Machine& machine,
                                          const Track& track,
                                          size_t indFrom, size_t indTo,
                                          bool isWorking,
                                          bool isTransit,
                                          IEdgeMassCalculator &edgeMassCalculator,
                                          IEdgeSpeedCalculator &edgeSpeedCalculator,
                                          InternalParametersForPartialHLs & ip,
                                          const PlannerParameters & plannerParameters,
                                          const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                          double extraSpeedFactor = 1);

    /*
     * @brief Add a route segment coresponding to connection between two partial headland tracks
     *
     * @param [in/out] route Base route
     * @param machine Working machine.
     * @param nextTrack Next track
     * @param nextTrackInReverse Flag stating if the next track will be addde in reverse point-order
     * @param removeLastPoint Flag stating if the last point of the connecting segment should be removed
     * @param isWorking Flag stating if the segment if a working segment
     * @param isTransit Flag stating if the segment if a transit segment
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Working speed calculator
     * @param [in/out] ip Internal planning parameters.
     * @param plannerParameters Planner parameters
     * @param machineCurrentStates (optional) Machine current states used to decide where to start working (disregarded if null).
    */
    void addInterTrackConnection_partialHLs(Route& route,
                                            const Machine& machine,
                                            const Track& nextTrack,
                                            bool nextTrackInReverse,
                                            bool removeLastPoint,
                                            bool isWorking,
                                            bool isTransit,
                                            IEdgeMassCalculator &edgeMassCalculator,
                                            IEdgeSpeedCalculator &edgeSpeedCalculator,
                                            InternalParametersForPartialHLs &ip,
                                            const PlannerParameters &plannerParameters,
                                            const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates);

    /*
     * @brief Add a route segment coresponding to connection between two partial headlands
     *
     * @param [in/out] route Base route
     * @param subfield Subfield.
     * @param machine Working machine.
     * @param indHLFrom Index of the source headland
     * @param indHLTo Index of the target headland
     * @param [in/out] ip Internal planning parameters.
     * @param removeLastPoint Flag stating if the last point of the connecting segment should be removed
     * @param plannerParameters Planner parameters
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Working speed calculator
     * @param machineCurrentStates (optional) Machine current states used to decide where to start working (disregarded if null).
    */
    void addInterHeadlandConnection_partialHLs(Route& route,
                                               const Subfield &subfield,
                                               const Machine& machine,
                                               size_t indHLFrom, size_t indHLTo,
                                               InternalParametersForPartialHLs &ip,
                                               bool removeLastPoint,
                                               const PlannerParameters &plannerParameters,
                                               IEdgeMassCalculator &edgeMassCalculator,
                                               IEdgeSpeedCalculator &edgeSpeedCalculator,
                                               const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates);

    /*
     * @brief Get the index of a headland (from subfield.headlands.partial) based on a reference headland index (from subfield.headlands.partial) and a given displacement in ip.sortedPartialHLs
     *
     * @param indHLRef Reference headland index (from subfield.headlands.partial)
     * @param deltaInd Headland displacement in the headland sequence ip.sortedPartialHLs (if < 0, the sequence will be checked in reverse order)
     * @param ip Internal planning parameters.
     * @return Index on the sequence at a desired deltaInd.
    */
    static int getHeadlandIndexFromSortedList(size_t indHLRef, int deltaInd, const InternalParametersForPartialHLs &ip);

    /*
     * @brief Get the index of ip.sortedPartialHLs corresponding to an actual (!) headland index (from subfield.headlands.partial)
     *
     * @param indHL Actual (!) headland index (from subfield.headlands.partial)
     * @param ip Internal planning parameters.
     * @return Index of ip.sortedPartialHLs corresponding to indHL
    */
    static int getSortedListIndexForHeadland(size_t indHL, const InternalParametersForPartialHLs & ip);


    /**
     * @brief Checks if a segment is worked based on the RemainingArea map/grid
     * @param p0 First point of the segment
     * @param p1 Second point of the segment
     * @param width Width of the segment
     * @param [in/out] ip Internal planning parameters.
     * @return True if the segment is considered to be worked
     */
    static bool isSegmentWorked(const Point &p0,
                                const Point &p1,
                                double width,
                                InternalParametersGeneral & ip);

    /**
     * @brief Checks if a partial headland is completelly worked based on the pre-computed headland states
     * @param indHLRef Reference headland index (from subfield.headlands.partial)
     * @param deltaInd Headland displacement in the headland sequence ip.sortedPartialHLs (if < 0, the sequence will be checked in reverse order)
     * @param ip Internal planning parameters.
     * @param [out] (optional) Index of the headland (from subfield.headlands.partial) corresponding to indHLRef + deltaInd
     * @return True if the partial headland is considered to be worked
     */
    static bool isHeadlandWorked(size_t indHLRef, int deltaInd, const InternalParametersForPartialHLs & ip, size_t *indHL = nullptr);

    /**
     * @brief Checks if a partial headland partially worked
     * @param indHL Headland index
     * @param ip Internal planning parameters.
     * @return True if the partial headland is considered to be partially worked
     */
    static bool isPartialHeadlandPartiallyWorked(const Subfield &subfield,
                                                 size_t indHL,
                                                 const InternalParametersForPartialHLs& ip);

    /**
     * @brief Computes the mass to be worked in a segment
     * @param p0 First point of the segment
     * @param p1 Second point of the segment
     * @param width Width of the segment
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out] ip Internal planning parameters.
     * @param [out] isWorked Flag stating if the segment is worked.
     * @return Mass to be worked in a segment
     */
    static float getMass(const Point &p0,
                         const Point &p1,
                         double width,
                         IEdgeMassCalculator & edgeMassCalculator,
                         InternalParametersGeneral &ip,
                         bool &isWorked);

    /**
     * @brief Updates the ip.workedPolys from a worked segment
     * @param points Segment points
     * @param width Width of the segment
     * @param [in/out] ip Internal planning parameters.
     */
    static void updateWorkedAreas(const std::vector<Point> &points,
                                  double width,
                                  InternalParametersGeneral &ip);

    /**
     * @brief Get a segment from a complete headland track
     * @param track Track
     * @param ip Internal planning parameters.
     * @param from Index of the first track point to be added
     * @param to Index of the last track point to be added (if <1 -> last point in the track)
     * @param closedTrack Flag stating if the track is closed
     * @param [out] Flag stating if the track points were added in reverse order (based on the track direction and the process direction - clockwise)
     * @return Segment points
     */
    static std::vector<Point> getTrackSegment_completeHL(const Track &track,
                                                         const InternalParametersForCompleteHL & ip,
                                                         size_t from, int to,
                                                         bool closedTrack,
                                                         bool &inReverse);

    /**
     * @brief Get a segment from a partial headland track
     * @param track Track
     * @param ip Internal planning parameters.
     * @param from Index of the first track point to be added
     * @param to Index of the last track point to be added (if > from -> reverse order)
     * @param [out] Flag stating if the track points were added in reverse order
     * @return Segment points
     */
    static std::vector<Point> getTrackSegment_partialHL(const Track &track,
                                                        const InternalParametersForPartialHLs & ip,
                                                        size_t from, size_t to,
                                                        bool &inReverse);

    /**
     * @brief Get an unworked segment from the first track to be worked in a complete headland
     * @param subfield Subfield.
     * @param ip Internal planning parameters.
     * @param [out] Flag stating if the track points were added in reverse order (based on the track direction and the process direction - clockwise)
     * @return Segment points
     */
    static std::vector<Point> getUnworkedTrackSegment_completeHL(const Subfield &subfield,
                                                                 const InternalParametersForCompleteHL & ip,
                                                                 bool &inReverse);


    /**
     * @brief Get the index of ip.sortedPartialHLs (!) corresponding to the first partially worked headland found in the sequence
     * @param subfield Subfield.
     * @param [in/out] ip Internal planning parameters.
     * @return Index of ip.sortedPartialHLs (!) corresponding to the first partially worked headland found in the sequence (<0 if not found)
     */
    static std::vector<size_t> getIndexesOfPartiallyWorkedHeadlands(InternalParametersForPartialHLs &ip);

    /**
     * @brief Get the index of the headland closest to a given point, where the returning indx corresponds to a headland either after or before a given reference headland on the headland sequence ip.sortedPartialHLs
     * @param subfield Subfield.
     * @param indHL Reference headland index.
     * @param pt Reference point.
     * @param ip Internal planning parameters.
     * @return Index of the headland closest to a given point (following the sequence)
     */
    static size_t getNextHLIndexClosestToPoint(const Subfield &subfield,
                                               size_t indHL,
                                               const Point& pt,
                                               const InternalParametersForPartialHLs & ip);

    /**
     * @brief Get the transit speed for a given machine and bunker mass
     * @param machine Machine.
     * @param bunkerMass Bunker mass
     * @return Transit speed
     */
    double getTransitSpeed(const Machine& machine,
                           double bunkerMass);


protected:
    static const double ThresholdIsWorked;/**< Threshold [0,1] used to consider an area worked or not */
    static const std::string RemainingAreaMapName;/**< Name of the worked-area gridmap (given by the user) */
    static const std::string MassFactorMapName;/**< Name of the worked-area gridmap (given by the user) */

    std::shared_ptr<gridmap::GridCellsInfoManager> m_cim = nullptr; /**< Shared GridCellsInfoManager */

};

}

#endif // AROLIB_HEADLANDBASEROUTESPLANNER_H
