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
 
#ifndef AROLIB_BASEROUTESPLANNER_H
#define AROLIB_BASEROUTESPLANNER_H

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
#include "arolib/planning/track_connectors/infieldtracksconnectordef.hpp"
#include "arolib/cartography/common.hpp"
#include "arolib/cartography/sharedgridsmanager.hpp"
#include "arolib/planning/simpleBaseRoutesPlanner.hpp"
#include "arolib/misc/basic_responses.h"
#include "arolib/components/headlandbaseroutesplanner.h"
#include "arolib/components/infieldbaseroutesplanner.h"

namespace arolib {

/**
 * @brief This Class plans the base-routes for field processing like harvester routes (including headland and inner-field)
 */
class BaseRoutesPlanner : public LoggingComponent
{
public:

    /**
     * @brief Base-routes planner parameters
     */
    struct PlannerParameters : public virtual FieldGeneralParameters, public virtual GridComputationSettings{

        bool workHeadlandFirst = true; /**< Should the headland be worked before the inner-field? (might be disregarded depending on workedAreaTransitRestriction)*/
        HeadlandBaseRoutesPlanner::WorkedAreaTransitRestriction workedAreaTransitRestriction; /**< Restriction to plan transit segments over worked or unworked tracks */

        bool startHeadlandFromOutermostTrack; /**< Should the first track to be worked in the headland be the outer-most or the inner-most track?*/
        bool finishHeadlandWithOutermostTrack; /**< Should the last track to be worked in the headland be the outer-most or the inner-most track? (only for partial headlands)*/
        bool headlandClockwise; /**< Work the headland tracks in clockwise direction? (only for complete/surounding headland) */
        bool restrictToBoundary; /**< If true, the mass calculation in the headland will check the intersection with the field outer boundary */
        bool monitorPlannedAreasInHeadland; /**< If true, the mass calculation in the headland will check for the intersection with planned areas */
        double headlandSpeedMultiplier; /**< This will be multiplied to the calculated machine working speed to obtain the speed to be used while working the headland */
        HeadlandBaseRoutesPlanner::MachineOrderStrategy headlandMachineOrderStrategy; /**< Strategy used to set the order in which the machines will be assigned the headland tracks (when the headland is worked first)*/

        bool limitStartToExtremaTracks = true; /**< Is the selection of the starting track in the infield limited to a track located at an extrema? */
        bool useMachineTurningRadInTrackSequencer = false; /**< Should the machine turning radius be used in the tracks sequencer computations? */
        bool infieldInverseTrackOrder; /**< Invert the order of the tracks (might be disregarded based on the curent worked state of the field, the locations of the machines or the reference point)*/
        bool infieldInversePointsOrder; /**< Invert the order of the points of the first track (might be disregarded based on the curent worked state of the field, the locations of the machines or the reference point) */

        PlannerParameters();

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

        /**
         * @brief Obtain the corresponding Headland planner parameters
         * @return Headland planner parameters
         */
        HeadlandBaseRoutesPlanner::PlannerParameters toHeadlandPlannerParameters() const;

        /**
         * @brief Obtain the corresponding Inner-field planner parameters
         * @return Inner-field planner parameters
         */
        InfieldBaseRoutesPlanner::PlannerParameters toInfieldPlannerParameters() const;

        /**
         * @brief Set the corresponding parameters from Headland planner parameters
         * @param params Headland planner parameters
         */
        void fromHeadlandPlannerParameters(const HeadlandBaseRoutesPlanner::PlannerParameters& params);

        /**
         * @brief Set the corresponding parameters from Inner-field planner parameters
         * @param params Inner-field planner parameters
         */
        void fromInfieldPlannerParameters(const InfieldBaseRoutesPlanner::PlannerParameters& params);
    };

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit BaseRoutesPlanner(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Generate the field base routes.
     *
     * The massFactorMap is a gridmap containing the factor/multiplier to be to be applied on the mass calculation (used on top of the edgeMassCalculator). It could be based on the areas that have been worked already (e.g. the inner-field), or in general a correction for the mass calculation.
     * The planner automatically factors out the areas outside of the field, hence these areas must not be factored out in this massFactorMap.
     * The massFactorMap must also factor out worked-areas corresponding to the workedAreaMap.
     * The workedAreaMap is used only to estimate the initial planning parameters for partially worked fields (incl. in which track and track-point to start working), but not for mass calculation (hence the need to include the corresponding mass factor in the massFactorMap. Likewise, the massFactorMap is not used to estimate the initial planning parameters.
     *
     * @param [in/out] subfield Subfield containing the necessary data (inc. tracks). It might be updated after the planning.
     * @param workinggroup Machines used for planning
     * @param plannerParameters Planner parameters
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculatorHeadland Working speed calculator (for headland)
     * @param [in/out*] edgeSpeedCalculatorInfield Working speed calculator (for innerfield)
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
                 std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorHeadland,
                 std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorInfield,
                 std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorTransit,
                 std::vector<Route> & routes,
                 std::shared_ptr<ArolibGrid_t> massFactorMap = nullptr,
                 const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates = nullptr,
                 const Pose2D * initRefPose = nullptr,
                 const OutFieldInfo* outFieldInfo = nullptr,
                 std::shared_ptr<const ArolibGrid_t> remainingAreaMap = nullptr);

    /**
     * @brief Set the Infield TrackSequencer to be used.
     * @param track_sequencer Infield TrackSequencer to be used. (if null -> sets default)
     */
    virtual void setInfieldTrackSequencer(std::shared_ptr<ITrackSequencer> track_sequencer);

    /**
     * @brief Set the TracksConnector to be used to connect headland and inner field.
     * @param connector TracksConnector to be used to connect headland and inner field. (if null -> sets default)
     */
    virtual void setTrackConnector_headland2infield(std::shared_ptr<IInfieldTracksConnector> connector);

    /**
     * @brief Set the TracksConnector to be used in the inner field.
     * @param connector TracksConnector to be used in the inner field. (if null -> sets default)
     */
    virtual void setTrackConnector_infield(std::shared_ptr<IInfieldTracksConnector> connector);

    /**
     * @brief Set shared CellsInfoManager to record cells data
     * @param cim CellsInfoManager.
     */
    virtual void setGridCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim);

protected:
    /**
     * @brief Set the default Infield TrackSequencer to be used.
     */
    virtual void setDefInfieldTrackSequencer();

    /**
     * @brief Set the defalt TracksConnector to be used to connect headland and inner field.
     */
    virtual void setDefTrackConnector_headland2infield();

    /**
     * @brief Set the default TracksConnector to be used in the inner field.
     */
    virtual void setDefTrackConnector_infield();


    /**
     * @brief Remove the initial route points from tracks considered to be fully worked
     * @param [in/out] routes Routes to be updated
     */
    void removeWorkedTracks(std::vector<Route>& routes);


    /**
     * @brief Remove the initial route points considered to be fully worked
     * @param [in/out] route Route to be updated
     * @return Route containing the removed rout epoints
     */
    Route removeWorkedPoints(Route& route);

    /**
     * @brief Remove the initial route points considered to be fully worked
     * @param [in/out] routes Routes to be updated
     * @return Routes containing the removed rout epoints
     */
    std::vector<Route> removeWorkedPoints(std::vector<Route>& routes);

    /**
     * @brief Get the final pose of the route whose last route-point has the lowest (valid) timestamp
     * @param routes Routes
     * @return Pose (invalid point if not valid pose was obtained)
     */
    Pose2D getRefPose(const std::vector<Route> &routes);


    /**
     * @brief Update the final position and orientation of the machine states based on the coresponding route
     * @param routes Routes
     * @param [in/out] machineStates Machine states to be updated
     */
    void updatedMachineStates(const std::vector<Route> &routes, std::map<MachineId_t, MachineDynamicInfo> &machineStates);


    /**
     * @brief Get the updted MassFactorMap removing the inner-field region
     * @param subfield Subfield
     * @param machines Machines
     * @param routes_if Routes corresponding to inner-field work
     * @param massFactorMap Base massFactorMap (if nullptr -> disregarded)
     * @return Updted MassFactorMap
     */
    std::shared_ptr<ArolibGrid_t> getUpdatedMassFactorMap_headland(const Subfield &subfield,
                                                                   const std::map<MachineId_t, Machine>& machines,
                                                                   const std::vector<Route> &routes_if,
                                                                   std::shared_ptr<const ArolibGrid_t> massFactorMap);


    /**
     * @brief Get the updted MassFactorMap removing the headland region
     * @param subfield Subfield
     * @param machines Machines
     * @param routes_hl Routes corresponding to headland work
     * @param massFactorMap Base massFactorMap (if nullptr -> disregarded)
     * @return Updted MassFactorMap
     */
    std::shared_ptr<ArolibGrid_t> getUpdatedMassFactorMap_infield(const Subfield &subfield,
                                                                  const std::map<MachineId_t, Machine>& machines,
                                                                  const std::vector<Route> &routes_hl,
                                                                  std::shared_ptr<const ArolibGrid_t> massFactorMap);

    /**
     * @brief Connect the headland and inner-field routes for each machine (headland routes first)
     * @param subfield Subfield
     * @param routes_hl Routes corresponding to headland work
     * @param routes_if Routes corresponding to inner-field work
     * @param edgeSpeedCalculatorTransit Edge speed calculator used for the connection transit
     * @param machines Machines
     * @param [out] routes Connected routes
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp connectHeadlandAndInfieldRoutes(const Subfield &subfield,
                                            std::vector<Route> &routes_hl,
                                            std::vector<Route> &routes_if,
                                            std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorTransit,
                                            const std::map<MachineId_t, Machine>& machines,
                                            std::vector<Route>& routes);

    /**
     * @brief Connect the inner-field and headland routes for each machine (inner-field routes first)
     * @param subfield Subfield
     * @param routes_hl Routes corresponding to headland work
     * @param routes_if Routes corresponding to inner-field work
     * @param edgeSpeedCalculatorTransit Edge speed calculator used for the connection transit
     * @param machines Machines
     * @param [out] routes Connected routes
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp connectInfieldAndHeadlandRoutes(const Subfield &subfield,
                                            std::vector<Route>& routes_if,
                                            std::vector<Route>& routes_hl,
                                            std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorTransit,
                                            const std::map<MachineId_t, Machine>& machines,
                                            std::vector<Route>& routes);

protected:
    std::shared_ptr<gridmap::GridCellsInfoManager> m_cim = nullptr;/**< Grid-cells-info manager */
    std::shared_ptr<ITrackSequencer> m_tracksSequencer;/**< Infield tracks' sequencer */
    std::shared_ptr<IInfieldTracksConnector> m_tracksConnector_hl2if = nullptr; /**< Tracks' connector (Headland -> Infield) */
    std::shared_ptr<IInfieldTracksConnector> m_tracksConnector_if = nullptr; /**< Tracks' connector (infiend)*/

};

}

#endif // AROLIB_BASEROUTESPLANNER_H
