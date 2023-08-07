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
 
#ifndef AROLIB_INFIELDBASEROUTESPLANNER_H
#define AROLIB_INFIELDBASEROUTESPLANNER_H

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
#include "arolib/planning/track_sequencing/simpletracksequencer.hpp"
#include "arolib/planning/track_connectors/infieldtracksconnectordef.hpp"
#include "arolib/cartography/common.hpp"
#include "arolib/planning/simpleBaseRoutesPlanner.hpp"
#include "arolib/misc/basic_responses.h"

namespace arolib {

/**
 * @brief This Class plans the base-routes for inner-field processing like harvester routes (only inner-field, without the headland)
 */
class InfieldBaseRoutesPlanner : public LoggingComponent
{
public:

    /**
     * @brief Base-routes infield planner parameters
     */
    struct PlannerParameters : public virtual FieldGeneralParameters, public virtual GridComputationSettings, public virtual ITrackSequencer::TrackSequencerSettings{
        bool inverseTrackOrder = false; /**< Invert the order of the tracks (might be disregarded based on the curent worked state of the field, the locations of the machines or the reference point)*/
        bool inversePointsOrder = false; /**< Invert the order of the points of the first track (might be disregarded based on the curent worked state of the field, the locations of the machines or the reference point) */
        double sampleResolutionHeadland = 0; /**< Resolution for the headland segments */
        bool removeInitialWorkedSegments = false; /**< If true, the initial segments of the base routes that have been already worked will be removed; otherwise the corresponding route points will have a timestamp < 0 */
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
    explicit InfieldBaseRoutesPlanner(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Generate the infield base routes.
     * @param [in/out] subfield Subfield containing the necessary data (inc. tracks). It might be updated after the planning.
     * @param workinggroup Machines used for planning
     * @param plannerParameters Planner parameters
     * @param remainingArea_map Remaining (unworked) -area map/grid
     * @param machineCurrentStates Map containing the current states of the machines
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Speed calculator (working edges).
     * @param [in/out*] edgeSpeedCalculator Speed calculator (transit edges).
     * @param [out] routes Resulting planned routes
     * @param _initRefPoint (optional) Vector containing the initial reference point (only the first one is used). If set, remainingArea_map is disregarded when checking the best place to start the routes
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp plan(const Subfield &subfield,
                 const std::vector<Machine> &workinggroup,
                 const PlannerParameters & plannerParameters,
                 std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                 std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                 std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorTransit,
                 std::vector<Route> & routes,
                 const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates = nullptr,
                 const Pose2D * initRefPose = nullptr,
                 std::shared_ptr<const ArolibGrid_t> massFactorMap = nullptr,
                 std::shared_ptr<const ArolibGrid_t> remainingAreaMap = nullptr);

    /**
     * @brief Set the Infield TrackSequencer to be used.
     * @param track_sequencer Infield TrackSequencer to be used.
     */
    virtual void setInfieldTrackSequencer(std::shared_ptr<ITrackSequencer> track_sequencer);

    /**
     * @brief Set the Infield TracksConnector to be used.
     * @param connector Infield TracksConnector to be used.
     */
    virtual void setInfieldTrackConnector(std::shared_ptr<IInfieldTracksConnector> connector);

    /**
     * @brief Set shared CellsInfoManager to record cells data
     * @param cim CellsInfoManager.
     */
    virtual void setGridCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim);

protected:

    /**
     * @brief Get the indexes of the tracks that are completelly worked
     * @param subfield Subfield
     * @param remainingArea_map Remaining (unworked) -area map/grid
     * @param bePrecise Be precise with remainingArea_map?
     * @return indexes of the tracks that are completelly worked
     */
    std::set<size_t> getExcludeTrackIndexes(const Subfield &subfield, std::shared_ptr<const ArolibGrid_t> remainingAreaMap, bool bePrecise);


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
    bool getInverseFlagsBasedOnRemainingArea(const Subfield &subfield,
                                             const std::set<size_t> &excludeTrackIndexes,
                                             const std::vector<Machine> workinggroup,
                                             const PlannerParameters & plannerParameters,
                                             const ArolibGrid_t &remainingArea_map,
                                             const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
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
    bool getInverseFlagsBasedOnMachineLocation(const Subfield &subfield,
                                               const std::set<size_t> &excludeTrackIndexes,
                                               const std::vector<Machine> workinggroup,
                                               const PlannerParameters & plannerParameters,
                                               const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
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
    bool getInverseFlagsBasedOnReferencePoint(const Subfield &subfield,
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
    bool isWorked(const Point &p0, const Point &p1, double width, const ArolibGrid_t &remainingAreaMap, bool bePrecise);

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
    int isWorked(const Polygon& boundary, const Point &p0, const Point &p1, double width, const ArolibGrid_t &remainingAreaMap, bool bePrecise);

    /**
     * @brief Checks if a segment has biomass based on the biomass-proportion map/grid
     * @param p0 First point of the segment
     * @param p1 Second point of the segment
     * @param workingWidth Width of the segment
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @return True if the segment has biomass
     */
    bool hasBiomass(const Point &p0, const Point &p1, double workingWidth, std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator);


    /**
     * @brief Generate the infield harvester routes.
     * @param subfield Subfield
     * @param excludeTrackIndexes Indexes of the tracks that are completelly worked
     * @param workinggroup Machines used for planning
     * @param plannerParameters Planner parameters
     * @param inverseTrackOrder Process the tracks in inverse order?
     * @param inversePointsOrderStart Process the points of the 1st track in inverse order?
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Speed calculator (working edges).
     * @param [in/out*] edgeSpeedCalculator Speed calculator (transit edges).
     * @param [out] routes Generated harvester routes
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generateBaseRoutes(const Subfield& subfield,
                               const std::set<size_t> &excludeTrackIndexes,
                               const std::vector<Machine> &workinggroup,
                               const PlannerParameters &plannerParameters,
                               bool inverseTrackOrder,
                               bool inversePointsOrderStart,
                               const Pose2D * initRefPose,
                               std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                               std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                               std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorTransit,
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
                             std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                             std::shared_ptr<const ArolibGrid_t> remainingAreaMap,
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

    std::shared_ptr<gridmap::GridCellsInfoManager> m_cim = nullptr;/**< Grid-cells-info manager */
    const double m_unsamplingTolerance = 0.1;/**< Tolerance to unsample linestrings/polygons */
    static const double m_thresholdIsWorked;/**< Threshold [0,1] sued to consider an area worked or not*/
    std::shared_ptr<ITrackSequencer> m_tracksSequencer;/**< Infield tracks' sequencer */
    std::shared_ptr<IInfieldTracksConnector> m_tracksConnector = nullptr; /**< Infield tracks' connector */

    /**
     * @brief Internal edge-mass calculator
     */
    class InternalMassCalculator: public IEdgeMassCalculator{
    public:
        /**
         * @brief Constructor
         * @param base Base EdgeMassCalculator
         * @param massFactorMap Mass factor gridmap
         * @param cim Grids manager
         * @param precision Precision option to perform map/grid operations
         */
        InternalMassCalculator(std::shared_ptr<IEdgeMassCalculator> base,
                               std::shared_ptr<const ArolibGrid_t> massFactorMap,
                               std::shared_ptr<gridmap::GridCellsInfoManager> cim,
                               gridmap::SharedGridsManager::PreciseCalculationOption precision = gridmap::SharedGridsManager::PRECISE_ONLY_IF_AVAILABLE);

        /**
         * @brief Compute edge mass
         */
        virtual double calcMass (const Point& p0, const Point& p1, double width);

    private:
        gridmap::SharedGridsManager m_gridsManager; /**< Grids manager*/
        std::shared_ptr<IEdgeMassCalculator> m_base; /**< Base EdgeMassCalculator*/
        std::shared_ptr<const ArolibGrid_t> m_factorMap; /**< Factor gridmap */
        gridmap::SharedGridsManager::PreciseCalculationOption m_precision /**< Precision option to perform map/grid operations */;
    };
};

}

#endif // AROLIB_INFIELDBASEROUTESPLANNER_H
