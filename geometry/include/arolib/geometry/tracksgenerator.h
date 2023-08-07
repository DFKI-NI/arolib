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
 
#ifndef AROLIB_TRACKGENERATOR_H
#define AROLIB_TRACKGENERATOR_H

#include <unistd.h>
#include <iostream>
#include <math.h>
#include <string>
#include "arolib/cartography/common.hpp"
#include "arolib/geometry/field_geometry_processing.hpp"
#include "arolib/geometry/geometry_helper.hpp"

#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/logger.h"
#include "arolib/misc/basic_responses.h"
#include "arolib/misc/basicconversions.hpp"

namespace arolib {

namespace geometry{

/**
 * @brief Generator of tracks
 */
class TracksGenerator : public LoggingComponent
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
        double trackRelativeAreaThreshold = 0.2; /**< Relative area threshold (0, 0.99) used to decide whether a track should be removed or not */
        double trackAreaThreshold = -1; /**< If > 1, the tracks whose are intersecting the boundary is lower that this value, will be disregarded */
        bool onlyUntilBoundaryIntersection = false; /**< If true, the tracks will be generated so that the central points start/end at the boundary intersection (i.e. no complete coverage guaranteed) */
        bool splitBoundaryIntersectingTracks = false; /**< If true, the tracks that intersect with the boundary will be splitted into more tracks */
        double trackThresholdArea = -1; /**< If > 1, the tracks whose are intersecting the boundary is lower that this value, will be disregarded */

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
     * @brief Generate the tracks for a given boundary (simple generation).
     * @param boundary Boundary
     * @param refLine Reference linestring
     * @param trackWidths Track widths (if more than one, the width will be set cyclically). Disregards tracksGeneratorParameters.trackDistance
     * @param settings Tracks' generator parameters/settings
     * @param [out] tracks Generated tracks
     * @param [out] pTrackWidthsIndexes (optional) Vector containing the indexes of 'widths' corresponding to 'tracks' (disregarded if null)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generateTracks(Polygon boundary,
                           std::vector<Point> refLine,
                           const std::vector<double> & trackWidths,
                           const TracksGeneratorParameters &settings,
                           std::vector<Track> & tracks,
                           std::vector<size_t> *pTrackWidthsIndexes = nullptr);

    /**
     * @brief Generate the tracks for a given boundary (simple generation).
     * @param boundary Boundary
     * @param refLine Reference linestring
     * @param tracksGeneratorParameters Tracks' generator parameters/settings
     * @param [out] tracks Generated tracks
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generateTracks(Polygon boundary,
                           std::vector<Point> refLine,
                           const TracksGeneratorParameters &tracksGeneratorParameters,
                           std::vector<Track> & tracks);

    /**
     * @brief Generate the inner-field tracks for all subfields in a field.
     * @param [in/out] field Field to be processed (must include necessary data for each subfield, inc. reference lines)
     * @param settings Tracks' generator parameters/settings
     * @param referenceLineIndexes (optional) Indexes of the reference lines to be used in each subfield. If no index is given for a subfield, the first reference line (ind = 0) is used
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generateTracks(Field &field,
                           const TracksGeneratorParameters &settings,
                           const std::vector<size_t> &referenceLineIndexes = std::vector<size_t>(0));

    /**
     * @brief Generate the inner-field tracks of a subfield.
     * @param [in/out] subfield Subield to be processed (must include necesary data, inc. reference lines)
     * @param settings Tracks' generator parameters/settings
     * @param referenceLineIndex Index of the reference lines to be used
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generateTracks(Subfield &subfield,
                           const TracksGeneratorParameters &settings,
                           const size_t &referenceLineIndex = 0);

protected:

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
     * @brief Generates the tracks from a given boundary, reference line, and tracks widths
     * @param boundary boundary
     * @param trackWidths Track widths
     * @param [out] trackWidthsIndexes Vector containing the indexes of 'widths' corresponding to 'tracks'
     * @param refLine Reference line
     * @param refLineIsCenter Does the reference line correspond to a track center linestring
     * @param settings Tracks' generator parameters/settings
     * @param [out] tracks Generated tracks
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generateTracks(const Polygon &boundary,
                           const std::vector<double> &trackWidths,
                           std::vector<size_t> &trackWidthsIndexes,
                           std::vector<Point> refLine,
                           bool refLineIsCenter,
                           const TracksGeneratorParameters &settings,
                           std::vector<Track> &tracks);

    /**
     * @brief Remove the points at the start or end of a linestring that 'surpass' the given set of boundaries
     * @param boundaries boundaries
     * @param [in/out] ls Linestring
     * @param fromFront The points to remove are from the beginning (true) or end (false) of the linestring?
     */
    void removeLinestringPointsToFitBoundaries(const std::vector<const Polygon*> &boundaries, std::vector<Point>& ls, bool fromFront);

    /**
     * @brief Extends a given reference line to fit the boundary and translates it
     * @param boundary boundary
     * @param refLine Reference line
     * @param distance Translation distance
     * @param direction Translation direction
     * @param distExtension Distance to extend the reference line
     * @param [out] Translated linestring
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp translateAndExtendReferenceLine(const Polygon &boundary,
                                            const std::vector<Point> &refLine,
                                            double distance,
                                            const Point & direction,
                                            double distExtension,
                                            std::vector<Point> & ls);


    /**
     * @brief Extends a given reference line to fit the boundary and offsets it
     * @param boundary boundary
     * @param refLine Reference line
     * @param distance Offset distance
     * @param distExtension Distance to extend the reference line
     * @param [out] Offset linestring
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp offesetAndExtendReferenceLine(const Polygon &boundary,
                                          const std::vector<Point> &refLine,
                                          double distance,
                                          double distExtension,
                                          std::vector<Point> & ls);

    /**
     * @brief Creates tracks from a given reference line
     * @param boundary boundary
     * @param refLine Reference line
     * @param width Track width
     * @param settings Tracks' generator parameters/settings
     * @param [out] tracks Generated tracks
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp createTracksFromReferenceLine(const Polygon &boundary,
                                          const std::vector<Point> &refLine,
                                          double width,
                                          const TracksGeneratorParameters &settings,
                                          std::vector<Track> & tracks);

    /**
     * @brief Remove tracks that overlap with the next/previous track
     * @param boundary boundary
     * @param [in/out] tracks tracks
     * @param [in/out] trackDistanceIndexes Vector containing the indexes of 'distances' corresponding to 'tracks'
     * @param areaThreshold values = (0, 0.99). If the track overlapping area is lower than areaThreshold * track area, the track is removed
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp removeOverlappingTracks(const Polygon &boundary,
                                    std::vector<Track>& tracks,
                                    std::vector<size_t> &trackDistanceIndexes,
                                    const TracksGenerator::TracksGeneratorParameters &settings,
                                    double areaRelativeThreshold = 0.2,
                                    double areaThreshold = -1);

    /**
     * @brief Orders the subfield tracks following the given strategy
     * @param [in/out] tracks tracks
     * @param [in/out] trackDistanceIndexes Vector containing the indexes of 'distances' corresponding to 'tracks'
     * @param strategy strategy
     * @return AroResp with error id (0:=OK) and message
     */
    void orderTracks(std::vector<Track> &tracks,
                     std::vector<size_t> &trackDistanceIndexes,
                     TrackOrderStrategy strategy = TrackOrderStrategy::LONGEST_TRACK_FIRST);

    /**
     * @brief Samples the current tracks of the subfield
     * @param [in/out] tracks Tracks to sample
     * @param [in/out] trackDistanceIndexes Vector containing the indexes of 'distances' corresponding to 'tracks'
     * @param settings Tracks' generator parameters/settings (containing the strategy to sample the tracks)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp sampleTracks(std::vector<Track> &tracks,
                         std::vector<size_t> &trackDistanceIndexes,
                         const TracksGeneratorParameters& settings,
                         const Polygon &boundary);

    /**
     * @brief Samples the current tracks of the subfield
     * @param [in/out] tracks Tracks to sample
     * @param [in/out] trackDistanceIndexes Vector containing the indexes of 'distances' corresponding to 'tracks'
     * @param settings Tracks' generator parameters/settings (containing the strategy to sample the tracks)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp sampleTracks1(std::vector<Track> &tracks,
                          std::vector<size_t> &trackDistanceIndexes,
                          const TracksGeneratorParameters& settings,
                          const Polygon &boundary);

    /**
     * @brief Samples the current tracks of the subfield
     * @param [in/out] tracks Tracks to sample
     * @param [in/out] trackDistanceIndexes Vector containing the indexes of 'distances' corresponding to 'tracks'
     * @param settings Tracks' generator parameters/settings (containing the strategy to sample the tracks)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp sampleTracks2(std::vector<Track> &tracks,
                          std::vector<size_t> &trackDistanceIndexes,
                          const TracksGeneratorParameters& settings,
                          const Polygon &boundary);

    /**
     * @brief Samples the current tracks of the subfield
     *
     * [warning] more tests needed
     * @param [in/out] tracks Tracks to sample
     * @param [in/out] trackDistanceIndexes Vector containing the indexes of 'distances' corresponding to 'tracks'
     * @param settings Tracks' generator parameters/settings (containing the strategy to sample the tracks)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp sampleTracks3(std::vector<Track> &tracks,
                          std::vector<size_t> &trackDistanceIndexes,
                          const TracksGeneratorParameters& settings,
                          const Polygon &boundary);

    void preSampleAdjacentTracks(std::vector<Track> &tracks,
                                 size_t indRefTrack,
                                 const std::set<size_t> &exclude, const std::set<size_t> &parents,
                                 std::set<size_t> &preSampled,
                                 const std::map<size_t, std::set<size_t> >& adjacencyList,
                                 double resolution);

    /**
     * @brief Samples the points of a track
     * @param [in/out] points Points to sample
     * @param [in/out] resolution Resolution
     * @param bisectSegment If the distance between 2 points is less than 2xresolutiontrue, if true, it will add the point in the middle; otherwise it will add the point at a distance "resolution" from the reference point.
     */
    void sampleTrackPoints(std::vector<Point> &points,
                           double resolution,
                           bool bisectSegment = false);


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


    double getAreaThresholdForMultipleIntersections(double trackWidth, double res);


protected:

    std::vector< std::pair<std::string, std::vector<Point> > > m_pointsOutput; /**< for debug */
    const double m_unsamplingTolerance = 0.1; /**< Tolerance for geometry unsampling (refer to geometry helper) */
    const bool m_doInitialTranslation = false;/**< If set to true, all the geometries of the subfield will be translated to the origin before making any computations (to avoid dealing with huge values). The geometries are translated back after all computations are done (see translateAll method) */

    const double m_multWidth_checkLastTrack = 0.4;/**< multiplier to be used to check (and generate if necesary) if extra tracks are necessary at the begining and/or end to (partially) harvest some area with remaining yield  */

    static const double m_unsampleAngTol; /**< Angular tolerance used for unsampling the tracks */
};

}

}

#endif // AROLIB_TRACKGENERATOR_H
