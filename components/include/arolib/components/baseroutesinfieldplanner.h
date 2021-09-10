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
 
#ifndef AROLIB_BASEROUTESINFIELDPLANNER_H
#define AROLIB_BASEROUTESINFIELDPLANNER_H

#include <unistd.h>
#include <iostream>
#include <math.h>
#include <string>
#include <fstream>

#include "arolib/misc/logger.h"
#include "arolib/types/coordtransformer.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/field_geometry_processing.hpp"
#include "arolib/planning/generalplanningparameters.hpp"
#include "arolib/planning/simpletracksequencer.hpp"
#include "arolib/planning/infieldtracksconnector.hpp"
#include "arolib/cartography/common.hpp"
#include "arolib/planning/simpleBaseRoutesPlanner.hpp"
#include "arolib/planning/planningworkspace.h"
#include "arolib/misc/basic_responses.h"
#include "arolib/geometry/field_geometry_processing.hpp"

namespace arolib {

/**
 * @brief This Class plans the base-routes for inner-field processing like harvester routes (only inner-field, without the headland)
 */
class BaseRoutesInfieldPlanner : public LoggingComponent, protected PlanningWorkspaceAccessor
{
public:

    /**
     * @brief Base-routes infield planner parameters
     */
    struct PlannerParameters : public virtual FieldGeneralParameters, public virtual GridComputationSettings{
        int tracksPerMachine = 1; /**< Number of tracks per machine per bed */
        bool inverseTrackOrder = false; /**< Invert the order of the tracks (might be disregarded based on the curent worked state of the field, the locations of the machines or the reference point)*/
        bool inversePointsOrder = false; /**< Invert the order of the points of the first track (might be disregarded based on the curent worked state of the field, the locations of the machines or the reference point) */
        double sampleResolutionHeadland = 0; /**< Resolution for the headland segments */
        SimpleTrackSequencer::SequenceStrategy sequenceStrategy = SimpleTrackSequencer::MEANDER; /**< Track sequence strategy */
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
    explicit BaseRoutesInfieldPlanner(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Generate the infield harvester routes.
     * @param [in/out] subfield Subfield containing the necessary data (inc. tracks). It might be updated after the planning.
     * @param workinggroup Machines used for planning
     * @param plannerParameters Planner parameters
     * @param remainingArea_map Remaining (unworked) -area map/grid
     * @param machineCurrentStates Map containing the current states of the machines
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param edgeSpeedCalculator (in/out*) Speed calculator
     * @param [out] routes Resulting planned routes
     * @param _initRefPoint (optional) Vector containing the initial reference point (only the first one is used). If set, remainingArea_map is disregarded when checking the best place to start the routes
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp plan(Subfield &subfield,
                 const std::vector<Machine> &workinggroup,
                 const PlannerParameters & plannerParameters,
                 const ArolibGrid_t &remainingArea_map,
                 const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                 IEdgeMassCalculator & edgeMassCalculator,
                 IEdgeSpeedCalculator & edgeSpeedCalculator,
                 std::vector<Route> & routes,
                 const std::vector<Pose2D> &_initRefPoint = {});

    /**
     * @brief Generate the infield harvester routes.
     * @param [in/out] pw Planning workspace containing the necessary data (subfield, etc) for planning, as well as the resulting planned routes.
     * @param subfieldIdx Index of the subfield (in pw) that will be planned
     * @param plannerParameters Planner parameters
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param edgeSpeedCalculator (in/out*) Speed calculator
     * @param _initRefPoint Vector containing the initial reference point (only the first one is used). if set, remainingArea_map is disregarded when checking the best place to start the routes
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp plan(PlanningWorkspace &pw,
                 size_t subfieldIdx,
                 const PlannerParameters & plannerParameters,
                 IEdgeMassCalculator & edgeMassCalculator,
                 IEdgeSpeedCalculator & edgeSpeedCalculator,
                 const std::vector<Pose2D> &_initRefPoint = {});

protected:

    /**
     * @brief Get the indexes of the tracks that are completelly worked
     * @param subfield Subfield
     * @param remainingArea_map Remaining (unworked) -area map/grid
     * @param bePrecise Be precise with remainingArea_map?
     * @return indexes of the tracks that are completelly worked
     */
    std::set<size_t> getExcludeTrackIndexes(const Subfield &subfield, const ArolibGrid_t &remainingArea_map, bool bePrecise);


    /**
     * @brief Sets the inverseTrackOrder and inversePointsOrderStart  based on the RemainingArea map/grid and the machines' current states (locations).
     *
     * @param subfield subfield
     * @param excludeTrackIndexes Indexes of the tracks that are completelly worked
     * @param workinggroup Machines used for planning
     * @param plannerParameters Planner parameters
     * @param remainingArea_map Remaining (unworked) -area map/grid
     * @param machineCurrentStates Map containing the current states of the machines
     * @param [out] inverseTrackOrder Process the tracks in inverse order
     * @param [out] inversePointsOrderStart Process the points of the 1st track in inverse order
     * @return True on success.
     */
    bool getInverseFlagsBasedOnRemainingArea(Subfield &subfield,
                                             const std::set<size_t> &excludeTrackIndexes,
                                             const std::vector<Machine> workinggroup,
                                             const PlannerParameters & plannerParameters,
                                             const ArolibGrid_t &remainingArea_map,
                                             const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                             bool& inverseTrackOrder,
                                             bool& inversePointsOrderStart);

    /**
     * @brief Sets the inverseTrackOrder and inversePointsOrderStart based on the machines' current locations.
     *
     * It only checks the first and last unworked tracks of the subfield
     * @param subfield subfield
     * @param excludeTrackIndexes Indexes of the tracks that are completelly worked
     * @param workinggroup Machines used for planning
     * @param plannerParameters Planner parameters
     * @param machineCurrentStates Map containing the current states of the machines
     * @param [out] inverseTrackOrder Process the tracks in inverse order
     * @param [out] inversePointsOrderStart Process the points of the 1st track in inverse order
     * @return True on success.
     */
    bool getInverseFlagsBasedOnMachineLocation(Subfield &subfield,
                                               const std::set<size_t> &excludeTrackIndexes,
                                               const std::vector<Machine> workinggroup,
                                               const PlannerParameters & plannerParameters,
                                               const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                               bool& inverseTrackOrder,
                                               bool& inversePointsOrderStart);

    /**
     * @brief Sets the inverseTrackOrder and inversePointsOrderStart based on the machines' current locations.
     * @param subfield subfield
     * @param excludeTrackIndexes Indexes of the tracks that are completelly worked
     * @param workinggroup Machines used for planning
     * @param plannerParameters Planner parameters
     * @param [out] inverseTrackOrder Process the tracks in inverse order
     * @param [out] inversePointsOrderStart Process the points of the 1st track in inverse order
     * @return True on success.
     */
    bool getInverseFlagsBasedOnReferencePoint(Subfield &subfield,
                                              const std::set<size_t> &excludeTrackIndexes,
                                              const Pose2D &refPoint,
                                              const std::vector<Machine> workinggroup,
                                              const PlannerParameters & plannerParameters,
                                              bool& inverseTrackOrder,
                                              bool& inversePointsOrderStart);


    /**
     * @brief Checks if a segment is worked based on the RemainingArea map/grid
     * @param p0 First point of the segment
     * @param p1 Second point of the segment
     * @param width Width of the segment
     * @param remainingArea_map Remaining (unworked) -area map/grid
     * @param bePrecise Calculate the percentage of worked area preciselly
     * @return True if the segment is considered to be worked
     */
    bool isWorked(const Point &p0, const Point &p1, double width, const ArolibGrid_t &remainingArea_map, bool bePrecise);

    /**
     * @brief Checks if a segment is worked based on the RemainingArea map/grid (checking the boundary)
     * @param boundary Boundary
     * @param p0 First point of the segment
     * @param p1 Second point of the segment
     * @param width Width of the segment
     * @param remainingArea_map Remaining (unworked) -area map/grid
     * @param bePrecise Calculate the percentage of worked area preciselly
     * @return >0 -> the segment is considered to be worked; 0 -> not worked; <0 -> unknown
     */
    int isWorked(const Polygon& boundary, const Point &p0, const Point &p1, double width, const ArolibGrid_t &remainingArea_map, bool bePrecise);

    /**
     * @brief Checks if a segment has biomass based on the biomass-proportion map/grid
     * @param p0 First point of the segment
     * @param p1 Second point of the segment
     * @param workingWidth Width of the segment
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @return True if the segment has biomass
     */
    bool hasBiomass(const Point &p0, const Point &p1, double workingWidth, IEdgeMassCalculator & edgeMassCalculator);


    /**
     * @brief Generate the infield harvester routes.
     * @param subfield Subfield
     * @param excludeTrackIndexes Indexes of the tracks that are completelly worked
     * @param workinggroup Machines used for planning
     * @param plannerParameters Planner parameters
     * @param inverseTrackOrder Process the tracks in inverse order?
     * @param inversePointsOrderStart Process the points of the 1st track in inverse order?
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param edgeSpeedCalculator (in/out*) Speed calculator
     * @param [out] routes Generated harvester routes
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generateBaseRoutes(const Subfield& subfield,
                               const std::set<size_t> &excludeTrackIndexes,
                               const std::vector<Machine> &workinggroup,
                               const PlannerParameters &plannerParameters,
                               bool inverseTrackOrder,
                               bool inversePointsOrderStart,
                               IEdgeMassCalculator & edgeMassCalculator,
                               IEdgeSpeedCalculator & edgeSpeedCalculator,
                               std::vector<Route> & routes);

    /**
     * @brief Adjust the infield routes based on the biomass and remaining-area maps (and, if necesary, the machines' current states and other parameters)
     * @param [in/out] routes Route to be adjusted (overwritten with the adjusted routes)
     * @param subfield Subfield
     * @param workinggroup Machines used for planning
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param remainingArea_map Remaining (unworked) -area map/grid
     * @param plannerParameters Planner parameters
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp adjustBaseRoutes(std::vector<Route> & routes,
                             const Subfield& subfield,
                             const std::vector<Machine> &workinggroup,
                             IEdgeMassCalculator & edgeMassCalculator,
                             const ArolibGrid_t &remainingArea_map,
                             const PlannerParameters & plannerParameters);


    /**
     * @brief Gets the headland part (segment) connecting 2 points
     * @param headland Headland points
     * @param headlandPoint0 First point to use for the connection
     * @param headlandPoint1 Second point to use for the connection
     * @param includeP0 If true, the return vector will include headlandPoint0
     * @param includeP1 If true, the return vector will include headlandPoint1
     * @param sampleResolution Resulution for the headland part
     * @param longest If true, it will return the longest segment connecting the 2 points; if false, it will return the shortest one
     * @return Headland part (segment) connecting the 2 points
     */
    std::vector<Point> getHeadlandPart(const std::vector<Point> &headland,
                                       const Point& headlandPoint0,
                                       const Point& headlandPoint1,
                                       bool includeP0,
                                       bool includeP1,
                                       double sampleResolution,
                                       bool longest = false);

    /**
     * @brief Gets the headland part (segment) connecting 2 points using a control point to select whether the connection must be the shortest or the longest segment
     * @param headland Headland points
     * @param headlandPoint0 First point to use for the connection
     * @param headlandPoint1 Second point to use for the connection
     * @param control_point Control point to select whether the connection must be the shortest or the longest segment. The selected connection must not include this control point!
     * @param includeP0 If true, the return vector will include headlandPoint0
     * @param includeP1 If true, the return vector will include headlandPoint1
     * @param sampleResolution Resulution for the headland part
     * @return Headland part (segment) connecting the 2 points
     */
    std::vector<Point> getHeadlandSidesConnection(const std::vector<Point> &headland,
                                                  const Point &headlandPoint0,
                                                  const Point &headlandPoint1,
                                                  const Point &control_point,
                                                  bool includeP0,
                                                  bool includeP1,
                                                  double sampleResolution);

protected:

    /**
     * @brief Flag used internally to know whether the plannning is being done using a Planning Workspace or not
     */
    enum CalcGridValueOption{
        CALC_DIRECT, /**< Make computations directly without a planning workspace */
        CALC_PW /**< Make computations using the planning workspace */
    };

    const double m_unsamplingTolerance = 0.1;/**< Tolerance to unsample linestrings/polygones */
    CalcGridValueOption m_calcGridValueOption = CALC_DIRECT; /**< By default = CALC_DIRECT. Change to CALC_PW (and back) done by corresponding methods */
    PlanningWorkspace* m_planningWorkspace = nullptr;/**< Pointer to the planning workspace (if NULL, no methods with a planning workspace as parameter were called) */

    static const double m_thresholdIsWorked;/**< Threshold [0,1] sued to consider an area worked or not*/
};

}

#endif // AROLIB_BASEROUTESINFIELDPLANNER_H
