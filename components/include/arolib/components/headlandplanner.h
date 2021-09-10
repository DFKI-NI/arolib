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
 
#ifndef AROLIB_HEADLANDPLANNER_H
#define AROLIB_HEADLANDPLANNER_H

#include <unistd.h>
#include <iostream>
#include <math.h>
#include <string>
#include <functional>

#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/basic_responses.h"
#include "arolib/types/machine.hpp"
#include "arolib/types/machinedynamicinfo.hpp"
#include "arolib/types/coordtransformer.hpp"
#include "arolib/types/headlandroute.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/field_geometry_processing.hpp"
#include "arolib/planning/edgeMassCalculator.hpp"
#include "arolib/planning/edgeSpeedCalculator.hpp"
#include "arolib/planning/planningworkspace.h"
#include "arolib/planning/headlandrouteassembler.hpp"
#include "arolib/planning/aro_functions.hpp"
#include "arolib/planning/generalplanningparameters.hpp"
#include "arolib/cartography/common.hpp"
#include "arolib/io/io_xml.hpp"
#include "arolib/io/io_kml.hpp"
#include "arolib/components/tracksgenerator.h"


namespace arolib {

/**
 * @brief Class used to generate the headland geometries (inner-field boundaries, headland tracks, etc.) and the harvester routes for headland harvesting
 */
class HeadlandPlanner : public LoggingComponent, protected PlanningWorkspaceAccessor
{
public:

    /**
     * @brief Strategy used to sample the headland tracks
     */
    enum TracksSamplingStrategy{
        SIMPLE_TRACK_SAMPLING,/**< The sampling starts with the first point of the track */
        SAMPLE_TRACKS_PERPENDICULARILY,/**< The sampling is done using the line perpendicular to the previous (inwards) track in each oo its (sampled) points */
        SAMPLE_TRACKS_WITH_CLOSEST_SAMPLES/**< The sampling is done using by adding points in the track that are closest to the previous track (sampled) points */
    };

    /**
      * @brief Get the TracksSamplingStrategy (enum) from its int value
      * @brief value Int value
      * @return TracksSamplingStrategy
      */
    static TracksSamplingStrategy intToTracksSamplingStrategy(int value);

    /**
     * @brief Headland planner parameters
     *
     * * If there are machines assigned to process the headland:
     *      * if headlandWidth > 0 --> the headland width will be the lower multiple of the working_widths of the machines that is >= headlandWidth
     *      * if headlandWidth <= 0 --> the headland width will depend on the machines working widths and the widthMultiplier
     *      * numTracks is disregarded
     *
     * * If no machines are assigned:
     *      * headlandWidth must be > 0
     *      * numTracks will be the number of HL tracks, where the distance between tracks = headlandWidth/numTracks (iif numTracks > 0)
     *      * widthMultiplier is disregarded
     */
    struct PlannerParameters : public virtual FieldGeneralParameters, public virtual GridComputationSettings{
        double headlandWidth = 0; /**< Headland width. If <= 0, the headland width is calculated using 'widthMultiplier' and the working widths of the machines */
        double widthMultiplier = 1; /**< Disregarded if headlandWidth > 0; otherwise, it is the times the sum of all harvesters' working widths must fit in the headland (i.e. defining the headland width) */
        double speedMultiplier = 1; /**< This will be multiplied to the calculated harvester speed to obtain the harvester speed to be used in the headland routes */
        size_t numTracks = 0;/**< If no machines are assigned for HL processing, generates this number of tracks based on headlandWidth, without taking into account the machines (i.e. generates tracks without routes)  */
        double trackDistance = -1; /**< desired distance between tracks (disregarded if a working group is given) */
        double sampleResolution = -1; /**< Sample resolution for the headland tracks and routes. If <= 0, it will be automatically calculated using the machines lengths */
        bool clockwise = true; /**< Generate polygons (tracks, boundaries, ...) clockwise */
        bool sortMachinesByWidth = true; /**< Sort the harvesters by working width (widest fisrt) before generating the tracks and routes (the widest machine will harvest the most outwards headland track) */
        TracksSamplingStrategy tracksSamplingStrategy = TracksSamplingStrategy::SAMPLE_TRACKS_PERPENDICULARILY; /**< Strategy used to sample the headland tracks */

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
    explicit HeadlandPlanner(const LogLevel &logLevel = LogLevel::INFO);

    /**
     * @brief Generate the subfield's infield boundary, planned headland, and (if applicable) headland tracks and (harvester) routes.
     *
     * Headland tracks and harvester routes will only be generated if there exist harvesters assigned to the headland in workinggroup.
     * If no there are no harvesters assigned to the headland in workinggroup, the infield boundary and planned headland will be generated to match plannerParameters.headlandWidth;
     * If there exist harvesters assigned to the headland in workinggroup, infield boundary and planned headland (and other geometries) will be generated so that the headland width is >= plannerParameters.headlandWidth, and it fits to the machines widths (i.e. there are always complete tracks to harvest, equal to the machine's working width)
     * @param [in/out] subfield Subfield containing the necessary data (inc. outer boundary). It will be updated after the planning.
     * @param workinggroup Machines used for planning (only harvesters assigned to the headland are taken into account)
     * @param outFieldInfo Out-of-field information (inc. arrival times)
     * @param plannerParameters Planner parameters
     * @param [out] routes Resulting planned harvester routes (if applicable, i.e. there were valid harvesters assigned to harvest the headland)
     * @param edgeMassCalculator Edge mass calculator
     * @param edgeSpeedCalculator Edge speed calculator
     * @param initRefPoint (optional) Vector containing the initial reference point (only the first one is used) used to set the location where the routes must begin
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp planHeadlandSubfield(Subfield &subfield,
                                 const std::vector<arolib::Machine>& workinggroup,
                                 const OutFieldInfo& outFieldInfo,
                                 const PlannerParameters& plannerParameters,
                                 std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                 std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                 std::vector< HeadlandRoute >& routes,
                                 const std::vector<Point>& initRefPoint = {});

    /**
     * @brief Generate the subfield's infield boundary, planned headland, and (if applicable) headland tracks and (harvester) routes.
     *
     * Headland tracks and harvester routes will only be generated if there exist harvesters assigned to the headland in workinggroup.
     * If no there are no harvesters assigned to the headland in workinggroup, the infield boundary and planned headland will be generated to match plannerParameters.headlandWidth;
     * If there exist harvesters assigned to the headland in workinggroup, infield boundary and planned headland (and other geometries) will be generated so that the headland width is >= plannerParameters.headlandWidth, and it fits to the machines widths (i.e. there are always complete tracks to harvest, equal to the machine's working width)
     * @param [in/out] subfield Subfield containing the necessary data (inc. outer boundary). It will be updated after the planning.
     * @param workinggroup Machines used for planning (only harvesters assigned to the headland are taken into account)
     * @param outFieldInfo Out-of-field information (inc. arrival times)
     * @param plannerParameters Planner parameters
     * @param remainingAreaMap Remaining (unworked) -area map/grid
     * @param edgeMassCalculator Edge mass calculator
     * @param edgeSpeedCalculator Edge speed calculator
     * @param [out] routes Resulting planned harvester routes (if applicable, i.e. there were valid harvesters assigned to harvest the headland)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp planHeadlandSubfield(Subfield &subfield,
                                 const std::vector<arolib::Machine>& workinggroup,
                                 const OutFieldInfo& outFieldInfo,
                                 const PlannerParameters& plannerParameters,
                                 const ArolibGrid_t& remainingAreaMap,
                                 std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                 std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                 std::vector< HeadlandRoute >& routes);

    /**
     * @brief For all subfields in the field, it generate the infield boundary, planned headland, and (if applicable) headland tracks and (harvester) routes.
     *
     * Headland tracks and harvester routes will only be generated if there exist harvesters assigned to the headland in workinggroup.
     * If no there are no harvesters assigned to the headland in workinggroup, the infield boundary and planned headland will be generated to match plannerParameters.headlandWidth;
     * If there exist harvesters assigned to the headland in workinggroup, infield boundary and planned headland (and other geometries) will be generated so that the headland width is >= plannerParameters.headlandWidth, and it fits to the machines widths (i.e. there are always complete tracks to harvest, equal to the machine's working width)
     * It attempts to plan the subfields so that the distance between consecutuve subfields is minimized
     * @param [in/out] field Field with subfields containing the necessary data (inc. outer boundary). It will be updated after the planning.
     * @param workinggroup Machines used for planning (only harvesters assigned to the headland are taken into account)
     * @param outFieldInfo Out-of-field information (inc. arrival times)
     * @param plannerParameters Planner parameters
     * @param edgeMassCalculator Edge mass calculator
     * @param edgeSpeedCalculator Edge speed calculator
     * @param [out] routes Resulting planned harvesters routes (one set of routes per subfield) (if applicable, i.e. there were valid harvesters assigned to harvest the headland). at the moment, the key is the subfield index in the vector
     * @param initRefPoint (optional) Vector containing the initial reference point (only the first one is used) used to set the location where the routes must begin
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp planHeadlandField(Field &field,
                              const std::vector<arolib::Machine>& workinggroup,
                              const OutFieldInfo& outFieldInfo,
                              const PlannerParameters& plannerParameters,
                              std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                              std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                              std::map<int, std::vector<HeadlandRoute> > &routes,
                              const std::vector<Point>& initRefPoint = {});

    /**
     * @brief For all subfields in the field, it generate the infield boundary, planned headland, and (if applicable) headland tracks and (harvester) routes.
     *
     * Headland tracks and harvester routes will only be generated if there exist harvesters assigned to the headland in workinggroup.
     * If no there are no harvesters assigned to the headland in workinggroup, the infield boundary and planned headland will be generated to match plannerParameters.headlandWidth;
     * If there exist harvesters assigned to the headland in workinggroup, infield boundary and planned headland (and other geometries) will be generated so that the headland width is >= plannerParameters.headlandWidth, and it fits to the machines widths (i.e. there are always complete tracks to harvest, equal to the machine's working width)
     * In the case of planning the complete field, it attempts to plan the subfields so that the distance between consecutuve subfields is minimized
     * @param [in/out] field Field with subfields containing the necessary data (inc. outer boundary). It will be updated after the planning.
     * @param workinggroup Machines used for planning (only harvesters assigned to the headland are taken into account)
     * @param outFieldInfo Out-of-field information (inc. arrival times)
     * @param plannerParameters Planner parameters
     * @param remainingAreaMap Remaining (unworked) -area map/grid
     * @param edgeMassCalculator Edge mass calculator
     * @param edgeSpeedCalculator Edge speed calculator
     * @param [out] routes Resulting planned harvesters routes (one set of routes per subfield) (if applicable, i.e. there were valid harvesters assigned to harvest the headland). at the moment, the key is the subfield index in the vector
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp planHeadlandField(Field &field,
                              const std::vector<arolib::Machine>& workinggroup,
                              const OutFieldInfo& outFieldInfo,
                              const PlannerParameters& plannerParameters,
                              const ArolibGrid_t& remainingAreaMap,
                              std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                              std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                              std::map<int, std::vector<HeadlandRoute> > &routes);

    /**
     * Plans/replans the headland for a field or single subfield.
     *
     * Headland tracks and harvester routes will only be generated if there exist harvesters assigned to the headland in workinggroup.
     * If no there are no harvesters assigned to the headland in workinggroup, the infield boundary and planned headland will be generated to match plannerParameters.headlandWidth;
     * If there exist harvesters assigned to the headland in workinggroup, infield boundary and planned headland (and other geometries) will be generated so that the headland width is >= plannerParameters.headlandWidth, and it fits to the machines widths (i.e. there are always complete tracks to harvest, equal to the machine's working width)
     * @param [in/out] pw Planning workspace containing the planning data. If the pw has a remaining area map and the corresponding harvester routes, it makes a replanning using them.
     * @param edgeMassCalculator Edge mass calculator
     * @param edgeSpeedCalculator Edge speed calculator
     * @param subfieldIdx subfield index. if <0, plan for all subfields
     * @param initRefPoint initial reference / starting point (vector with 0 or 1 input). Important iif no remainingarea map is allocated. If empty, calculates automatically starting point
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp planHeadland(PlanningWorkspace &pw,
                         const PlannerParameters& plannerParameters,
                         std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                         std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                         int subfieldIdx = -1,
                         const std::vector<Point>& initRefPoint = {});


protected:

    /**
     * @brief Get the harvesters assigned to harvest the headland.
     * @param machines Working group (all machines)
     * @return Harvesters assigned to harvest the headland
     */
    std::vector<Machine> getAssignedHarvesters(const std::vector<Machine>& machines) const;

    /**
     * @brief Obtain the desired headland width based on the planner parameters.
     *
     * If plannerParameters.headlandWidth > 0, it returns plannerParameters.headlandWidth; otherwise, it calculates it from the machines with and plannerParameters.widthMultiplier
     * @param plannerParameters Planner parameters
     * @param machines Working group
     * @return Headland width
     */
    double getHeadlandWidth(const PlannerParameters& plannerParameters,
                            const std::vector<Machine>& machines);

    /**
     * @brief Reorder the outer boundary points so that the first one is the closest to an access point or the lastRoutePoint (if given)
     * @param [in/out] sf Subfield to be updated
     * @param lastRoutePoint (optional) Pointer to the last route-point from a previous plan (e.g. from an adjacent subfield)
     * @return True on success
     */
    bool reorderSubfieldPoints(Subfield & sf, const Point * lastRoutePoint = NULL);


    /**
     * @brief Get the best starting point based on the access points and the corresponding arrival times
     * @param sf Subfield
     * @param workinggroup Machines
     * @param outFieldInfo Out-of-field information (inc. arrival times)
     * @return Staring point (default = first point of the outer boundary)
     */
    Point getStartingPoint(const Subfield &sf,
                           const std::vector<arolib::Machine>& workinggroup,
                           const OutFieldInfo& outFieldInfo);

    /**
     * @brief For all subfields in the field (in pw), it generate the infield boundary, planned headland, and (if applicable) headland tracks and (harvester) routes.
     * @param [in/out] pw Planning workspace containing the planning data. If the pw has a remaining area map and the corresponding harvester routes, it makes a replanning using them.
     * @param plannerParameters Planner parameters
     * @param initRefPoint initial reference / starting point (vector with 0 or 1 input). Important iif no remainingarea map is allocated. If empty, calculates automatically starting point
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp processField(PlanningWorkspace &pw,
                         const PlannerParameters& plannerParameters,
                         std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                         std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                         const std::vector<Point>& initRefPoint);

    /**
     * @brief For all subfields in the field (in pw), it generate the infield boundary, planned headland, and (if applicable) headland tracks and (harvester) routes.
     *
     * The initial reference point is calculated automatically based in the remaining-area map/grid (if existent)
     * @param [in/out] pw Planning workspace containing the planning data. If the pw has a remaining area map and the corresponding harvester routes, it makes a replanning using them.
     * @param plannerParameters Planner parameters
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp processField(PlanningWorkspace &pw,
                         const PlannerParameters& plannerParameters,
                         std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                         std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator);

    /**
     * @brief It generates the infield boundary, planned headland, and (if applicable) headland tracks and (harvester) routes for the given subfield (in pw).
     * @param [in/out] pw Planning workspace containing the planning data. If the pw has a remaining area map and the corresponding harvester routes, it makes a replanning using them.
     * @param subfieldIdx Index of the subfield to be processed
     * @param plannerParameters Planner parameters
     * @param initRefPoint initial reference / starting point (vector with 0 or 1 input). Important iif no remainingarea map is allocated. If empty, calculates automatically starting point
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp processSubfield_pw(size_t subfieldIdx,
                               const PlannerParameters& plannerParameters,
                               std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                               std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                               const std::vector<Point>& initRefPoint);

    /**
     * @brief It generates the infield boundary, planned headland, and (if applicable) headland tracks and (harvester) routes for the given subfield (in pw).
     *
     * The initial reference point is calculated automatically based in the remaining-area map/grid (if existent)
     * @param [in/out] pw Planning workspace containing the planning data. If the pw has a remaining area map and the corresponding harvester routes, it makes a replanning using them.
     * @param subfieldIdx Index of the subfield to be processed
     * @param plannerParameters Planner parameters
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp processSubfield_pw(size_t subfieldIdx,
                               const PlannerParameters& plannerParameters,
                               std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                               std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator);

    /**
     * @brief It generates the infield boundary, planned headland, and (if applicable) headland tracks and (harvester) routes for the given subfield (in pw).
     * @param [in/out] pw Planning workspace containing the planning data. If the pw has a remaining area map and the corresponding harvester routes, it makes a replanning using them.
     * @param subfieldIdx Index of the subfield to be processed
     * @param plannerParameters Planner parameters
     * @param headlandWidth Headland width
     * @param machines Working group
     * @param startingPoint initial reference / starting point
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp processSubfield_pw(size_t subfieldIdx,
                               const PlannerParameters& plannerParameters,
                               double headlandWidth,
                               const std::vector<arolib::Machine>& machines,
                               std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                               std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                               arolib::Point startingPoint);

    /**
     * @brief It generates the infield boundary, planned headland, and (if applicable) headland tracks and (harvester) routes for the given subfield.
     * @param [in/out] subfield subfield to be processed
     * @param headlandWidth Headland width
     * @param machines Working group
     * @param edgeMassCalculator Edge mass calculator
     * @param edgeSpeedCalculator Edge speed calculator
     * @param [out] routes Planned headland routes
     * @param startingPoint initial reference / starting point
     * @param settings Planner parameters
     * @return AroResp with error id (0:=OK) and message
     */
    bool processSubfield(arolib::Subfield& subfield,
                         double headlandWidth,
                         const std::vector<arolib::Machine>& machines,
                         std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                         std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                         std::vector< arolib::HeadlandRoute >& routes,
                         arolib::Point startingPoint,
                         const PlannerParameters& settings);

    /**
     * @brief It generates the infield boundary, planned headland, and (if applicable) headland tracks and (harvester) routes for the given subfield.
     *
     * The headland width is calculated automatically using the settings.widthMultiplier and the working widths of the assigned harvesters
     * @param [in/out] subfield subfield to be processed
     * @param machines Working group
     * @param edgeMassCalculator Edge mass calculator
     * @param edgeSpeedCalculator Edge speed calculator
     * @param [out] routes Planned headland routes
     * @param startingPoint initial reference / starting point
     * @param settings Planner parameters
     * @return AroResp with error id (0:=OK) and message
     */
    bool processSubfield(arolib::Subfield& subfield,
                         const std::vector<arolib::Machine>& machines,
                         std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                         std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                         std::vector< arolib::HeadlandRoute >& routes,
                         arolib::Point startingPoint,
                         const PlannerParameters& settings);

    /**
     * @brief Adjusts the routes (and timestamps) based on the worked area, setting timestamp=-1 in the initial segments of the route that have been worked
     * @param [in/out] routes Routes
     * @param machines Working group
     * @param plannerParameters Planner parameters
     * @param remainingAreaMap Remaining (unworked) -area map/grid
     */
    void adjustRoutes(std::vector< arolib::HeadlandRoute >& routes,
                      const std::vector<arolib::Machine>& machines,
                      const ArolibGrid_t& remainingAreaMap,
                      const PlannerParameters& plannerParameters);

    /**
     * @brief Adjusts the routes (and timestamps) based on the worked area, setting timestamp=-1 in the initial segments of the route that have been worked
     * @param [in/out] pw Planning workspace containing the necessary data (subfield, maps, etc.).
     * @param subfieldIdx Index of the subfield (in pw)
     * @param plannerParameters Planner parameters
     * @param remainingAreaMap Remaining (unworked) -area map/grid
     */
    void adjustRoutes(PlanningWorkspace &pw,
                      size_t subfieldIdx,
                      const PlannerParameters& plannerParameters);

    /**
     * @brief Offsets a boundary (in- or out- wards)
     * @param boundary_in Polygon to be offset
     * @param boundary_out Resulting offset polygon
     * @param offset Offset distance
     * @param inflated If true, the boundary_in will be inflated (offset outwards); otherwise, the boundary_in will be deflated (offset inwards)
     * @param strategy Strategy/method to perform the offset operation
     * @return True on success
     */
    bool offsetBoundary(const Polygon& boundary_in,
                         Polygon& boundary_out,
                         double offset,
                         bool inflated);

    /**
     * @brief Generates the tracks and harvester routes for a subfield
     * @param [in/out] subfield subfield to be processed
     * @param machines Working group
     * @param [out] tracks Generated tracks
     * @param yield_map Yield-proportion map/grid (values in t/ha)
     * @param dryness_map Drymatter map/grid
     * @param startingPoint initial reference / starting point
     * @param headlandWidth Headland width
     * @param settings Planner parameters
     * @param [out] routes Generated headland routes
     */
    void generateTracksAndRoutes(arolib::Subfield& subfield,
                                 const std::vector<Machine> &machines,
                                 std::vector<std::tuple<Polygon, size_t, int> > &tracks,
                                 std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                 std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                 const Point &startingPoint,
                                 double headlandWidth,
                                 const PlannerParameters& settings,
                                 std::vector<HeadlandRoute> &routes);

    /**
     * @brief (not used at the moment) Add reference samples to the tracks
     * @param [in/out] tracks tracks to be processed
     * @param resolution Sample resolution for the tracks
     */
    void addReferenceSamplesToTracks(std::vector<std::tuple<Polygon, size_t, int> > &tracks,
                                     double resolution);

    /**
     * @brief (not used at the moment) Add reference samples to the tracks (old implementation)
     * @param [in/out] tracks tracks to be processed
     * @param resolution Sample resolution for the tracks
     */
    void addReferenceSamplesToTracks_old(std::vector<std::tuple<Polygon, size_t, int> > &tracks,
                                     double resolution);

    /**
     * @brief Sample the tracks using perpendicular vectors (obtained from previous sampled tracks)
     *
     * It samples the most inward track first (reference track). For each sample in this track, it calculates the vector perpendicular to the samples segment (starting at the sample) and obtains the intersection of this perpendicular vector with the next track (appending it as a sample of the next track).
     * When all samples of the reference track are processed, the next track is sampled (without loosing the samples generated with the perpendicular vectors). The new sampled track is now taken as the reference track to process the following track in the subfield.
     * @param [in/out] tracks tracks to be processed
     * @param resolution Sample resolution for the tracks
     * @param headlandWidth Headland width
     */
    void sampleTracksPerpendicularly(std::vector<std::tuple<Polygon, size_t, int> > &tracks,
                                     double resolution,
                                     double headlandWidth);

    /**
     * @brief Sample the tracks by adding samples as close as possible to the samples in the previous (inwards) track
     *
     * It samples the most inward track first (reference track). For each sample in this track, it calculates the point in the next track closest to it (appending it as a sample of the next track).
     * When all samples of the reference track are processed, the next track is sampled (without loosing the samples generated with the perpendicular vectors). The new sampled track is now taken as the reference track to process the following track in the subfield.
     * @param [in/out] tracks tracks to be processed
     * @param resolution Sample resolution for the tracks
     * @param headlandWidth Headland width
     */
    void sampleTracksWithClosestPoints(std::vector<std::tuple<Polygon, size_t, int> > &tracks,
                                       double resolution);


    /**
     * @brief Checks if replanning is needed based on the remaining-area map/grid and the planned routes (do the planned routes concurr with the remaining-area map?)
     * @param [in/out] routes Planned routes. It might be updated (e.g. if a route is completelly worked, it is removed)
     * @param workingGroup Working group
     * @param remainingAreaMap Remaining (unworked) -area map/grid
     * @param [out] newInitRefPoint New initial reference / starting point in case replanning is needed
     * @return True if replanning is needed
     */
    bool replanningNeeded(std::vector< arolib::HeadlandRoute >& routes,
                          const std::vector<Machine> &workingGroup,
                          const ArolibGrid_t &remainingAreaMap,
                          const PlannerParameters& settings,
                          arolib::Point& newInitRefPoint);



    /**
     * @brief Checks if a segment is worked based on the RemainingArea map/grid
     * @param p0 First point of the segment
     * @param p1 Second point of the segment
     * @param workingWidth Width of the segment
     * @param remainingAreaMap Remaining (unworked) -area map/grid
     * @return True if the segment is considered to be worked
     */
    bool isWorked(const Point &p0,
                  const Point &p1,
                  double width,
                  const ArolibGrid_t &remainingAreaMap,
                  bool bePrecise = false);

protected:

    /**
     * @brief Flag used internally to know whether the plannning is being done using a Planning Workspace or not
     */
    enum CalcGridValueOption{
        CALC_DIRECT, /**< Make computations directly without a planning workspace */
        CALC_PW /**< Make computations using the planning workspace */
    };
    CalcGridValueOption m_calcGridValueOption = CALC_DIRECT; /**< By default = CALC_DIRECT. Change to CALC_PW (and back) done by corresponding methods */
    PlanningWorkspace* m_planningWorkspace = nullptr; /**< Pointer to the planning workspace (if NULL, no methods with a planning workspace as parameter were called) */

    static const double m_thresholdIsWorked;/**< Threshold [0,1] sued to consider an area worked or not*/

};

}

#endif // AROLIB_HEADLANDPLANNER_H
