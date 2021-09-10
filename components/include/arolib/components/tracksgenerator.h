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
 
#ifndef AROLIB_TRACKGENERATOR_H
#define AROLIB_TRACKGENERATOR_H

#include <unistd.h>
#include <iostream>
#include <math.h>
#include <string>
#include "arolib/cartography/common.hpp"
#include "arolib/geometry/field_geometry_processing.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/planning/planningworkspace.h"

#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/logger.h"
#include "arolib/misc/basic_responses.h"
#include "arolib/misc/basicconversions.hpp"

namespace arolib {

/**
 * @brief Generate IF (inner-field, so without HL) tracks
 */
class TracksGenerator : public LoggingComponent, protected PlanningWorkspaceAccessor
{
public:

    /**
     * @brief Strategy to shift the reference line
     */
    enum ShiftingStrategy{
        TRANSLATE_TRACKS, /**< Translate the reference line using a direction vector (if direction is not set, it is calculated from the normal vectors to the reference line/linestrin segments); this strategy keeps the form and length of the reference-line linestring */
        OFFSET_TRACKS /**< Offset the reference-line linestring. This strategy DOES NOT keep the form and length of the reference-line linestring unless it is a straight line */
    };

    /**
      * @brief Get the ShiftingStrategy (enum) from its int value
      * @param value Int value
      * @return ShiftingStrategy
      */
    static ShiftingStrategy intToShiftingStrategy(int value);

    /**
     * @brief Strategy to order the tracks
     */
    enum TrackOrderStrategy{
        LEFTMOST_TRACK_FIRST, /**< The track (beteen the fisrt and last computed tracks) with the left-most point will be the first track in the subfield's vector (track 0) */
        LONGEST_TRACK_FIRST, /**< The longest track (beteen the fisrt and last computed tracks) will be the first track in the subfield's vector (track 0) */
        EXTRA_TRACK_LAST /**< If an extra track was added at the begining or end of the calculated tracks (i.e. an extra track was added to harvest a final partial track with remaining yield), this thack will be the last track in the subfield's vector (track n-1); if there is no extra track, the default strategy is used */
    };

    /**
      * @brief Get the TrackOrderStrategy (enum) from its int value
      * @param value Int value
      * @return TrackOrderStrategy
      */
    static TrackOrderStrategy intToTrackOrderStrategy(int value);

    /**
     * @brief Strategy to sample the tracks
     */
    enum TrackSamplingStrategy{
        DEFAULT_SAMPLING, /**< at the moment = MIN_DIST_BETWEEN_TRACKS  */
        START_AT_TRACK_START, /**< Perform a normal sampling starting from the first point of the track */
        MIN_DIST_BETWEEN_TRACKS /**< Perform a sampling so that the distance between two samples from adjacent tracks is minimum (i.e. the edges connecting the tracks are perpendicular to the corresponding track segment) */
    };

    /**
      * @brief Get the TrackSamplingStrategy (enum) from its int value
      * @param value Int value
      * @return TrackSamplingStrategy
      */
    static TrackSamplingStrategy intToTrackSamplingStrategy(int value);

    /**
     * @brief Tracks' generator parameters
     */
    struct TracksGeneratorParameters{
        double trackDistance = -1; /**< Distance between tracks */
        double sampleResolution = 0; /**< Track resolution [m] (distance between track points) */
        ShiftingStrategy shiftingStrategy = ShiftingStrategy::TRANSLATE_TRACKS;/**< Strategy to shift the reference line/linestring */
        Point direction = Point(0,0); /**< Direction in which to translate the reference line/linestring (iif shiftingStrategy = TRANSLATE_TRACKS); if = (0,0) -> it is calculated from the normal vectors to the reference line/linestring segments */
        TrackOrderStrategy trackOrderStrategy = TrackOrderStrategy::LONGEST_TRACK_FIRST; /**< Strategy to order the tracks */
        TrackSamplingStrategy trackSamplingStrategy = TrackSamplingStrategy::DEFAULT_SAMPLING; /**< Strategy to sample the tracks */
        bool checkForRemainingTracks = false; /**< Check if it is necesary to add extra tracks at the begining and/or end to (partially) harvest some area with remaining yield */
        bool useRefLineAsCentralLine = true; /**< If the given reference line lies inside the boundary, it will be read as a desired reference central linestring */
        double trackAreaThreshold = 0.2; /**< Relative area threshold (0, 0.99) used to decide whether a track should be removed or not */
        bool onlyUntilBoundaryIntersection = false; /**< If true, the tracks will be generated so that the central points start/end at the boundary intersection (i.e. no complete coverage guaranteed) */

        /**
         * @brief Default constructor
         */
        explicit TracksGeneratorParameters() = default;

        /**
         * @brief Constructor
         * @param _trackDistance Distance between tracks
         * @param _sampleResolution Track resolution [m] (distance between track points)
         */
        explicit TracksGeneratorParameters(double _trackDistance,
                                           double _sampleResolution);


        /**
         * @brief Parse the parameters from a string map, starting from a default TracksGeneratorParameters
         * @param [out] param Parameters
         * @param map String map containing the parameter values
         * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the default values will remain)
         * @return True on success
         */
        static bool parseFromStringMap( TracksGeneratorParameters& params, const std::map<std::string, std::string>& map, bool strict = false);

        /**
         * @brief Parse the parameters to a string map
         * @param param Parameters
         * @return String map containing the parameter values
         */
        static std::map<std::string, std::string> parseToStringMap( const TracksGeneratorParameters& params);
    };

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit TracksGenerator(const LogLevel &logLevel = LogLevel::INFO);

    /**
     * @brief Generate the itracks for a given boundary (simple generation).
     * @param boundary Boundary
     * @param refLine Reference linestring
     * @param distances Distances between tracks (if more than one, the distances will be set cyclically)
     * @param tracksGeneratorParameters Tracks' generator parameters/settings
     * @param [out] tracks Generated tracks
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generateTracks(Polygon boundary,
                           std::vector<Point> refLine,
                           const std::vector<double> & distances,
                           const TracksGeneratorParameters &tracksGeneratorParameters,
                           std::vector<Track> & tracks);

    /**
     * @brief Generate the inner-field tracks for all subfields in a field.
     * @param [in/out] field Field to be processed (must include necessary data for each subfield, inc. reference lines)
     * @param tracksGeneratorParameters Tracks' generator parameters/settings
     * @param referenceLineIndexes (optional) Indexes of the reference lines to be used in each subfield. If no index is given for a subfield, the first reference line (ind = 0) is used
     * @param yieldmap Yield-proportion map/grid (values in t/ha)
     * @param remainingAreaMap Remaining (unharvested) -area map/grid
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generateTracks(Field &field,
                           const TracksGeneratorParameters &tracksGeneratorParameters,
                           const std::vector<size_t> &referenceLineIndexes = std::vector<size_t>(0),
                           const ArolibGrid_t& yieldMap = ArolibGrid_t(),
                           const ArolibGrid_t& remainingAreaMap = ArolibGrid_t());

    /**
     * @brief Generate the inner-field tracks of a subfield.
     * @param [in/out] subfield Subield to be processed (must include necesary data, inc. reference lines)
     * @param tracksGeneratorParameters Tracks' generator parameters/settings
     * @param referenceLineIndex Index of the reference lines to be used
     * @param yieldmap Yield-proportion map/grid (values in t/ha)
     * @param remainingAreaMap Remaining (unharvested) -area map/grid
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generateTracks(Subfield &subfield,
                                 const TracksGeneratorParameters &tracksGeneratorParameters,
                                 const size_t &referenceLineIndex = 0,
                                 const ArolibGrid_t& yieldMap = ArolibGrid_t(),
                                 const ArolibGrid_t& remainingAreaMap = ArolibGrid_t());

    /**
     * @brief Generate the inner-field tracks for all subfields in a field.
     * @param [in/out] pw Planning workspace containing the necessary data ( field with all necesary data for each subfield (inc. reference lines)) for infield tracks' generation; The subfields in the pw will be updated accordingly.
     * @param tracksGeneratorParameters Tracks' generator parameters/settings
     * @param referenceLineIndexes (optional) Indexes of the reference lines to be used in each subfield. If no index is given for a subfield, the first reference line (ind = 0) is used
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generateTracks(PlanningWorkspace& pw,
                                 const TracksGeneratorParameters &tracksGeneratorParameters,
                                 const std::vector<size_t> &referenceLineIndexes = std::vector<size_t>(0) );

    /**
     * @brief Generate the inner-field tracks of a subfield.
     * @param [in/out] pw Planning workspace containing the necessary data ( field with all necesary data for each subfield to be processed (inc. reference lines)) for infield tracks' generation; The subfields in the pw will be updated accordingly.
     * @param subfieldIdx subfield index. If <0, generates tracks for the complete field
     * @param tracksGeneratorParameters Tracks' generator parameters/settings
     * @param referenceLineIndex Index of the reference line to be used. If subfieldIdx < 0, applies to all subfields
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generateTracks(PlanningWorkspace& pw,
                                 size_t subfieldIdx,
                                 const TracksGeneratorParameters &tracksGeneratorParameters,
                                 const size_t &referenceLineIndex = 0 );

protected:

    /**
     * @brief Struct holding information of the tracks geometries
     */
    struct TrackGeometries{
        std::vector<Point> side1; /**< Points of one side of the track */
        std::vector<Point> side2; /**< Points of the other side of the track */
        std::vector<Point> center; /**< Central track points */
        double width = -1; /**< Track width */
    };

    /**
     * @brief Translates the given reference line to the best position (partially or completelly) inside the boundary
     *
     * Used when the reference line is not (partially or completelly) inside the boundary
     * @param boundary Boundary
     * @param refLine Reference line/linestring
     * @param [out] refLineTrans Translated reference line/linestring
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp relocateReferenceLine(const Polygon &boundary,
                                   const std::vector<Point>& refLine,
                                   std::vector<Point>& refLineTrans);

    /**
     * @brief Generated the tracks' geonetries by translating the reference line a given direction and distances
     *
     * This method keeps the form and length of the reference line/linestring
     * @param boundary Boundary
     * @param trackDistances Distances that the reference lines will be translated to generate the tracks (the order will be kept in the positive direction)
     * @param refLine Reference line/linestring
     * @param direction Translation direction vector
     * @param refLineIsCenter Flag stating whether the given refLine corresponds to the center line of the track or not
     * @param tracks [out] (temporary) resulting set of tracks' geometries (without intersections with the boundary)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp translateReferenceLine(const Polygon &boundary,
                                   const std::vector<double> &trackDistances,
                                   const std::vector<Point> &refLine,
                                   const Point & direction,
                                   bool refLineIsCenter,
                                   std::vector<TrackGeometries> & tracks);

    /**
     * @brief Generated the subfield tracks by translating the linestring along a given axis (direction vector)
     *
     * This method keeps the form and length of the reference line/linestring
     * @param [in/out] subfield Subfield to be processed
     * @param trackDistance Distance between adjacent tracks
     * @param refLine Reference line/linestring
     * @param direction Translation direction vector
     * @param startAtRefLine If true, the track generation will be done so that a track is generated where the reference line is; if false, the initial tracks will be generated at a distance +/- trackDistance/2 from the given reference line
     * @param checkLastTrack Check if it is necesary to add extra tracks at the begining and/or end to (partially) harvest some area with remaining yield
     * @param checkForSingleTrackCase If set to true and no tracks were generated at the end, it will recall the methods with trackDistance/2 (for the cases where the infield was so small that no tracks were generated with the original trackDistance)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp translateReferenceLine_old(Subfield &subfield,
                                         double trackDistance,
                                         const Linestring & refLine,
                                         const Point & direction,
                                         bool startAtRefLine,
                                         bool checkLastTrack,
                                         bool checkForSingleTrackCase = true);

    /**
     * @brief Generated the subfield tracks by offseting the linestring
     *
     * Unless the reference line/linestring is a straight line, this method WILL NOT keep the form and length of the reference line/linestring (think of a U-shaped linestring: the offset in one direction will generate a bigger U, whilest on the other direction it will generate a smaller U (or even just a line or point))
     * @param boundary Boundary
     * @param trackWidths Tracks' widths (the order will be kept in the positive direction)
     * @param refLine Reference line/linestring
     * @param refLineIsCenter Flag stating whether the given refLine corresponds to the center line of the track or not
     * @param tracksGeometries [out] (temporary) resulting set of tracks' geometries (without intersections with the boundary)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp offsetReferenceLine(const Polygon &boundary,
                                const std::vector<double>& trackWidths,
                                const std::vector<Point> & refLine,
                                bool refLineIsCenter,
                                std::vector<TrackGeometries> & tracks);

    /**
     * @brief Generated the subfield tracks by offseting the linestring
     *
     * Unless the reference line/linestring is a straight line, this method WILL NOT keep the form and length of the reference line/linestring (think of a U-shaped linestring: the offset in one direction will generate a bigger U, whilest on the other direction it will generate a smaller U (or even just a line or point))
     * @param [in/out] subfield Subfield to be processed
     * @param trackDistance Distance between adjacent tracks
     * @param refLine Reference line/linestring
     * @param direction Translation direction vector
     * @param startAtRefLine If true, the track generation will be done so that a track is generated where the reference line is; if false, the initial tracks will be generated at a distance +/- trackDistance/2 from the given reference line
     * @param checkLastTrack Check if it is necesary to add extra tracks at the begining and/or end to (partially) harvest some area with remaining yield
     * @param checkForSingleTrackCase If set to true and no tracks were generated at the end, it will recall the methods with trackDistance/2 (for the cases where the infield was so small that no tracks were generated with the original trackDistance)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp offsetReferenceLine_old(Subfield &subfield,
                                      double trackDistance,
                                      const Linestring & _refLine,
                                      bool startAtRefLine,
                                      bool checkLastTrack,
                                      bool checkForSingleTrackCase = true);

    /**
     * @brief Connects the current tracks to the boundary
     * @param boundary boundary
     * @param [in/out] subfield Tracks' geometries
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp connectTracksToBoundary(const Polygon &boundary,
                                    std::vector<TrackGeometries>& tracks, bool onlyUntilBoundaryIntersection);

    /**
     * @brief Create tracks from its computed geometries
     * @param tracks_geom tracks geometries
     * @param [out] tracks tracks
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp createTracks(std::vector<TrackGeometries>& tracks_geom,
                        std::vector<Track>& tracks);

    /**
     * @brief Remove tracks that practically lay outside the boundary
     * @param boundary boundary
     * @param [out] tracks tracks
     * @param areaThreshold values = (0, 0.99). If the track area lying inside the boundary is lower than areaThreshold * track area, the track is removed
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp removeEmptyTracks(const Polygon &boundary,
                              std::vector<Track>& tracks,
                              double areaThreshold = 0.2);

    /**
     * @brief Orders the subfield tracks following the given strategy
     * @param [in/out] tracks tracks
     * @param strategy strategy
     * @return AroResp with error id (0:=OK) and message
     */
    void orderTracks(std::vector<Track> &tracks, TrackOrderStrategy strategy = TrackOrderStrategy::LONGEST_TRACK_FIRST);

    /**
     * @brief Connects the current tracks of the subfield to its inner boundary
     * @param [in/out] subfield Subfield containing tracks
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp connectTracksToInnerBoundary(Subfield &subfield);

    /**
     * @brief Connects the current tracks of the subfield to its planned headland ('track' in the middle of the headland)
     * @param [in/out] subfield Subfield containing tracks
     * @param strategy Strategy to make the connection to the headland track
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp connectTracksToHeadland(Subfield &subfield,
                                          const arolib::geometry::HeadlandConnectionStrategy& strategy);

    /**
     * @brief Samples the current tracks of the subfield
     * @param [in/out] tracks Tracks to sampled
     * @param settings Tracks' generator parameters/settings (containing the strategy to sample the tracks)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp sampleTracks(std::vector<Track> &tracks, const TracksGeneratorParameters& settings, const Polygon &boundary);

    /**
     * @brief Samples the current tracks of the subfield
     * @param [in/out] tracks Tracks to sampled
     * @param settings Tracks' generator parameters/settings (containing the strategy to sample the tracks)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp sampleTracks1(std::vector<Track> &tracks, const TracksGeneratorParameters& settings, const Polygon &boundary);

    /**
     * @brief Samples the current tracks of the subfield
     *
     * [warning] more tests needed
     * @param [in/out] tracks Tracks to sampled
     * @param settings Tracks' generator parameters/settings (containing the strategy to sample the tracks)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp sampleTracks2(std::vector<Track> &tracks, const TracksGeneratorParameters& settings, const Polygon &boundary);

    /**
     * @brief Remove the tracks that are completely harvested
     *
     * @warning: by removing harvested tracks there will be a gap between the tracks and the headland which might be undesired in further planning steps (e.g. graph generation)
     * @param [in/out] subfield Subfield containing tracks
     * @param workingWidth Working width (used to calculte the edge rectangle)
     * @param yieldmap Yield-proportion map/grid (values in t/ha)
     * @param remainingAreaMap Remaining (unharvested) -area map/grid
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp removeHarvestedTracks(Subfield &subfield,
                                        double workingWidth,
                                        const ArolibGrid_t& yieldMap,
                                        const ArolibGrid_t& remainingAreaMap);

    /**
     * @brief Check if a track is completelly harvested
     * @param points Track points
     * @param workingWidth Working width (used to calculte the edge rectangle)
     * @param yieldmap Yield-proportion map/grid (values in t/ha)
     * @param remainingAreaMap Remaining (unharvested) -area map/grid
     * @return True if the track is completelly harvested
     */
    bool isTrackHarvested(const std::vector<Point> &points,
                              double workingWidth,
                              const ArolibGrid_t& yieldMap,
                              const ArolibGrid_t& remainingAreaMap);

    /**
     * @brief Translates all geometries of a subfield among a given axis and distance (direction vector)
     *
     * Translated geometries include subfield boundaries, tracks, given reference line, etc.
     * This method is used to translate the subfield to the origin (0,0) and back, to avoid performing calculations with large numbers (@TODO: check if it is still necesary, although for debug is nicer this way)
     * @param [in/out] subfield Subfield to be translated
     * @param refLineIndex Index of the reference that will be translated (all other reference lines stay the same)
     * @param vec Vector containing the direction and distance of translation
     * @return True if the track is completelly harvested
     */
    void translateAll(Subfield &subfield, const size_t &refLineIndex, const Point& vec);

    /**
     * @brief Check if a track with the given (temporary) geometries lies inside the boundary
     */
    bool isTrackInBoundary(const Polygon& boundary, TrackGeometries track, double extDist = -1);

    /**
     * @brief Creates a track
     * @param boundary Infield boundary
     * @param side1 Points corresponding to one side of the track (inside the boundary)
     * @param side1 Points corresponding to the side of the track (inside the boundary)
     * @return Track
     */
    Track createTrack(const Polygon& boundary, const std::vector<Point>& side1, const std::vector<Point>& side2, double res, double width, Track::TrackType type);


protected:

    /**
     * @brief Flag used internally to know whether the plannning is being done using a Planning Workspace or not
     */
    enum CalcGridValueOption{
        CALC_DIRECT, /**< Make computations directly without a planning workspace */
        CALC_PW /**< Make computations using the planning workspace */
    };

    std::vector< std::pair<std::string, std::vector<Point> > > m_pointsOutput; /**< for debug */
    const double m_unsamplingTolerance = 0.1; /**< Tolerance for geometry unsampling (refer to geometry helper) */
    const bool m_doInitialTranslation = false;/**< If set to true, all the geometries of the subfield will be translated to the origin before making any computations (to avoid dealing with huge values). The geometries are translated back after all computations are done (see translateAll method) */

    PlanningWorkspace* m_planningWorkspace = nullptr;/**< Pointer to the planning workspace (if NULL, no methods with a planning workspace as parameter were called) */
    CalcGridValueOption m_calcGridValueOption = CALC_DIRECT;/**< By default = CALC_DIRECT. Change to CALC_PW (and back) done by corresponding methods */

    const double m_multWidth_checkLastTrack = 0.4;/**< multiplier to be used to check (and generate if necesary) if extra tracks are necessary at the begining and/or end to (partially) harvest some area with remaining yield  */

};

}

#endif // AROLIB_TRACKGENERATOR_H
