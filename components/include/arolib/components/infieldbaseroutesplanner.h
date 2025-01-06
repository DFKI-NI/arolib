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
 
#ifndef AROLIB_INFIELDBASEROUTESPLANNER_H
#define AROLIB_INFIELDBASEROUTESPLANNER_H

#include "arolib/types/route.hpp"
#include "arolib/types/machinedynamicinfo.hpp"
#include "arolib/types/outfieldinfo.hpp"
#include "arolib/cartography/gridcellsinfomanager.hpp"
#include "arolib/planning/edge_calculators/edgeSpeedCalculator.hpp"
#include "arolib/planning/track_connectors/infieldtracksconnector.hpp"
#include "arolib/planning/track_sequencing/tracksequencer.hpp"
#include "arolib/planning/generalplanningparameters.hpp"
#include "arolib/planning/workedareaanalyst.hpp"

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
     * @param _initRefPoses Map containing the initial reference poses to be used to compute the first tracks to be worked. If set (!nullptr), the machines current locations and remainingArea_map are disregarded when checking the best place to start the routes.
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Speed calculator (working edges).
     * @param [in/out*] edgeSpeedCalculator Speed calculator (transit edges).
     * @param [out] routes Resulting planned routes
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
                 const std::map<MachineId_t, Pose2D> *_initRefPoses = nullptr,
                 std::shared_ptr<const ArolibGrid_t> massFactorMap = nullptr,
                 std::shared_ptr<const ArolibGrid_t> remainingAreaMap = nullptr);

    /**
     * @brief Generate the infield base routes.
     * @param [in/out] subfield Subfield containing the necessary data (inc. tracks). It might be updated after the planning.
     * @param workinggroup Machines used for planning
     * @param plannerParameters Planner parameters
     * @param remainingArea_map Remaining (unworked) -area map/grid
     * @param machineCurrentStates Map containing the current states of the machines
     * @param _initRefPose (optional) Initial reference poses to be used to compute the first tracks to be worked. If valid, the machines current locations and remainingArea_map are disregarded when checking the best place to start the routes.
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param [in/out*] edgeSpeedCalculator Speed calculator (working edges).
     * @param [in/out*] edgeSpeedCalculator Speed calculator (transit edges).
     * @param [out] routes Resulting planned routes
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
                 const Pose2D& initRefPose = Pose2D(Point::invalidPoint()),
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

    enum TrackWorkedState{
        TRACK_WORKED,
        TRACK_NOT_WORKED,
        TRACK_PARTIALLY_WORKED
    };

    struct TrackInfo{
        TrackWorkedState workedState = TRACK_NOT_WORKED;  /**< Worked states of the track */
        int workingDirection = 0;  /**< 0: not known; 1: in track's points order; -1: in track's points reverse order; 2: potentially in track's points order; -2: potentially in track's points reverse order */
        int indFirstWorkingPointFwd = -1;  /**< Index of the first point to work in forward direction */
        int indFirstWorkingPointRev = -1;  /**< Index of the first point to work in reverse direction */
    };

    struct TracksInfo{
        std::vector<TrackInfo> tracksInfo;  /**< Tracks info */
        std::set<size_t> excludeTrackIndexes;  /**< Tracks to be excluded */
        std::set<size_t> partiallyWorkedTrackIndexes;  /**< Partially worked */
        std::map<MachineId_t, size_t> indFirstTrack;  /**< Indexes of the first track that should be worked */
        TracksInfo(size_t numTracks): tracksInfo(numTracks){}
    };

    /**
     * @brief Initialize the tracks' info
     * @param subfield Subfield
     * @param remainingArea_map Remaining (unworked) -area map/grid
     * @param bePrecise Be precise with remainingArea_map?
     * @return Tracks info
     */
    static TracksInfo initTracksInfo(const Subfield &subfield, WorkedAreaAnalyst& waa, bool bePrecise);

    /**
     * @brief Select a partially worked track as the first track.
     *
     * @param subfield subfield
     * @param [in/out] tracksInfo Tracks info
     * @param workinggroup Machines used for planning
     * @param initRefPoses Current initial reference poses
     * @return True on success.
     */
    static bool updateFirstTrackFromPartiallyWorkedTracks(const Subfield &subfield,
                                                          TracksInfo &tracksInfo,
                                                          const std::vector<Machine> workinggroup,
                                                          const std::map<MachineId_t, Pose2D> &initRefPoses);

    /**
     * @brief Select the fisrt track to work based on the partially worked tracks and machines that are near the worked segments of those tracks.
     *
     * @param subfield subfield
     * @param [in/out] tracksInfo Tracks info
     * @param workinggroup Machines used for planning
     * @param machineCurrentStates Map containing the current states of the machines
     * @param initRefPoses Current initial reference poses
     * @return True on success.
     */
    static bool updateFirstTrackInfoFromMachinesNearPartiallyWorkedTracks(const Subfield &subfield,
                                                                          TracksInfo &tracksInfo,
                                                                          const std::vector<Machine> workinggroup,
                                                                          const std::map<MachineId_t, MachineDynamicInfo>& machineCurrentStates,
                                                                          const std::map<MachineId_t, Pose2D> &initRefPoses);

    /**
     * @brief Get a reference initial pose based on the current machine locations and the distance to the tracks.
     *
     * @param subfield subfield
     * @param [in/out] tracksInfo Tracks info
     * @param workinggroup Machines used for planning
     * @param machineCurrentStates Map containing the current states of the machines
     * @param initRefPoses Current initial reference poses
     * @return True on success.
     */
    static bool completeInitRefPosesFromMachinesLocations(const Subfield &subfield,
                                                          TracksInfo &tracksInfo,
                                                          const std::vector<Machine> workinggroup,
                                                          const std::map<MachineId_t, MachineDynamicInfo>& machineCurrentStates,
                                                          std::map<MachineId_t, Pose2D> &initRefPoses);


    /**
     * @brief Checks if a segment is worked based on the RemainingArea map/grid
     * @param p0 First point of the segment
     * @param p1 Second point of the segment
     * @param width Width of the segment
     * @param waa WorkedAreaAnalyst
     * @param bePrecise Calculate the percentage of worked area preciselly
     * @return first: worked state; second: computed value (to be read as worked value: i.e., 0:= not-worked and 1:= worked ; NAN if no valid value was computed)
     */
    static std::pair<WorkedAreaAnalyst::WorkedState, float> isSegmentWorked(const Point &p0, const Point &p1, double width, WorkedAreaAnalyst &waa, bool bePrecise);

    /**
     * @brief Checks if a segment is worked based on the RemainingArea map/grid (checking the boundary)
     * @param boundary Boundary
     * @param p0 First point of the segment
     * @param p1 Second point of the segment
     * @param width Width of the segment
     * @param waa WorkedAreaAnalyst
     * @param bePrecise Calculate the percentage of worked area preciselly
     * @return first: worked state; second: computed value (to be read as worked value: i.e., 0:= not-worked and 1:= worked ; NAN if no valid value was computed)
     */
    static std::pair<WorkedAreaAnalyst::WorkedState, float> isSegmentWorked(const Polygon& boundary, const Point &p0, const Point &p1, double width, WorkedAreaAnalyst &waa, bool bePrecise);

    /**
     * @brief Checks if a segment has biomass based on the biomass-proportion map/grid
     * @param p0 First point of the segment
     * @param p1 Second point of the segment
     * @param workingWidth Width of the segment
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @return True if the segment has biomass
     */
    static bool hasBiomass(const Point &p0, const Point &p1, double workingWidth, std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator);


    /**
     * @brief Generate the infield harvester routes.
     * @param subfield Subfield
     * @param excludeTrackIndexes Indexes of the tracks that are completelly worked
     * @param workinggroup Machines used for planning
     * @param plannerParameters Planner parameters
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
                               const std::map<MachineId_t, Pose2D> &initRefPoses,
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
     * @param waa WorkedAreaAnalyst
     * @param plannerParameters Planner parameters
     * @return AroResp with error id (0:=OK) and message
     */
    static AroResp adjustBaseRoutes(std::vector<Route> & routes,
                                    const Subfield& subfield,
                                    const std::vector<Machine> &workinggroup,
                                    std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                    WorkedAreaAnalyst &waa,
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
    static std::vector<Point> getHeadlandPart(const std::vector<Point> &headland,
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
    static std::vector<Point> getHeadlandSidesConnection(const std::vector<Point> &headland,
                                                         const Point &headlandPoint0,
                                                         const Point &headlandPoint1,
                                                         const Point &control_point,
                                                         bool includeP0,
                                                         bool includeP1,
                                                         double sampleResolution);

protected:

    std::shared_ptr<gridmap::GridCellsInfoManager> m_cim = nullptr;/**< Grid-cells-info manager */
    static const double m_unsamplingTolerance;/**< Tolerance to unsample linestrings/polygons */
    static const double m_thresholdIsWorkedLB;/**< Lower bound threshold [0,1] used to consider an area worked or not*/
    static const double m_thresholdIsWorkedUB;/**< Upper bound threshold [0,1] used to consider an area worked or not*/
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
        gridmap::SharedGridsManager m_gridsManager; /**< Grids manager */
        std::shared_ptr<IEdgeMassCalculator> m_base; /**< Base EdgeMassCalculator */
        std::shared_ptr<const ArolibGrid_t> m_factorMap; /**< Factor gridmap */
        gridmap::SharedGridsManager::PreciseCalculationOption m_precision /**< Precision option to perform map/grid operations */;
    };
};

}

#endif // AROLIB_INFIELDBASEROUTESPLANNER_H
