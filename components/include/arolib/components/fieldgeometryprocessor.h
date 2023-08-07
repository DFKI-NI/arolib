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
 
#ifndef AROLIB_FILELGEOMETRYPROCESSOR_H
#define AROLIB_FILELGEOMETRYPROCESSOR_H

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
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/field_geometry_processing.hpp"
#include "arolib/geometry/tracksgenerator.h"
#include "arolib/planning/edge_calculators/edgeMassCalculator.hpp"
#include "arolib/planning/edge_calculators/edgeSpeedCalculator.hpp"
#include "arolib/planning/planningworkspace.h"
#include "arolib/planning/aro_functions.hpp"
#include "arolib/planning/generalplanningparameters.hpp"
#include "arolib/cartography/common.hpp"
#include "arolib/io/io_xml.hpp"
#include "arolib/io/io_kml.hpp"


namespace arolib {

/**
 * @brief Class used to process the field geometries: generate the headland geometries (inner-field boundaries, headland tracks, etc.) and the inner-field tracks
 */
class FieldGeometryProcessor : public LoggingComponent, protected PlanningWorkspaceAccessor
{
public:

    /**
     * @brief Headland type
     */
    enum HeadlandType{
        SURROUNDING_HEADLAND,/**< Complete headland surrounding the inner-field*/
        SIDES_HEADLANDS,/**< Partial headlands located at the extrema of the inner-field tracks */
    };

    /**
      * @brief Get the HeadlandType (enum) from its int value
      * @brief value Int value
      * @return HeadlandType
      */
    static HeadlandType intToHeadlandType(int value);


    /**
     * @brief Headland generation settings
     *
     * * if headlandWidth <= 0: the headland width will correspond to trackWidth * numTracks
     *      * trackWidth must be > 0
     *      * numTracks must be > 0
     *
     * * if headlandWidth > 0:
     *      * if numTracks > 0: the tracks' width will correspond headlandWidth / numTracks
     *      * if numTracks <= 0:
     *          * if trackWidth <= 0: No tracks are generated
     *          * if trackWidth > 0: the headland width will be recalculated so that it is a multiple of trackWidth
     */
    struct HeadlandParameters{

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
         * @brief Strategy used to sample the headland tracks
         */
        enum SideHeadlandGenerationStrategy{
            SIDEHL_STRATEGY_DEF,/**<  */
            SIDEHL_STRATEGY_1,/**< Most stable */
            SIDEHL_STRATEGY_2,/**< Not very stable, but in some fields it might yield better headlands avoiding innecesary combinartion into one longer partial headland */
            SIDEHL_STRATEGY_3/**< Not stable */
        };

        /**
          * @brief Get the SideHeadlandGenerationStrategy (enum) from its int value
          * @brief value Int value
          * @return SideHeadlandGenerationStrategy
          */
        static SideHeadlandGenerationStrategy intToSideHeadlandGenerationStrategy(int value);

        /**
         * @brief Strategy used to sample the headland tracks
         */
        enum SideHeadlandTracksGenerationStrategy{
            SIDEHLTRACKS_STRATEGY_DEF,/**< Default */
            SIDEHLTRACKS_FROM_OUTER_BOUNDARY,/**< Generates the tracks starting from the outer-field boundary. Ensures that the areas near the field boundary are covered, but might fail in the generation of the inner-most tracks */
            SIDEHLTRACKS_FROM_INNER_BOUNDARY,/**< Generates the tracks starting from the inner-field boundary. Mostly stable, but might miss area near the field boundary */
            SIDEHLTRACKS_CHECK_ALL,/**< Computes with both strategies and selects the one that fits best */
        };

        /**
          * @brief Get the SideHeadlandTracksGenerationStrategy (enum) from its int value
          * @brief value Int value
          * @return SideHeadlandTracksGenerationStrategy
          */
        static SideHeadlandTracksGenerationStrategy intToSideHeadlandTracksGenerationStrategy(int value);

        double headlandWidth = 0; /**< Headland width. If <= 0, the headland width is calculated using 'widthMultiplier' and the working widths of the machines */
        size_t numTracks = 0;/**< If no machines are assigned for HL processing, generates this number of tracks based on headlandWidth, without taking into account the machines (i.e. generates tracks without routes)  */
        double trackWidth = -1; /**< tracks' width, i.e. desired distance between tracks*/
        double sampleResolution = -1; /**< Sample resolution for the headland tracks and routes. */
        double headlandConnectionTrackWidth = -1; /**< Width of the track connecting the partial headlands. if = 0, no connecting headland will be generated; if < 0, the same track width of the partial headlands will be used*/
        bool trimPartialHeadlandTrackEnds = true; /**< If true, the tracks of the main partial headlands will not reach the headland boundary*/
        TracksSamplingStrategy tracksSamplingStrategy = TracksSamplingStrategy::SAMPLE_TRACKS_PERPENDICULARILY; /**< Strategy used to sample the headland tracks */
        SideHeadlandGenerationStrategy sideHLGenerationStrategy = SideHeadlandGenerationStrategy::SIDEHL_STRATEGY_DEF; /**< Strategy used to generate the side headlands */
        SideHeadlandTracksGenerationStrategy sideHLTracksGenerationStrategy = SideHeadlandTracksGenerationStrategy::SIDEHLTRACKS_STRATEGY_DEF; /**< Strategy used to generate the side headlands' tracks */
        bool limitIFTracksNotOverMainHL = true; /**< Attept to limit how many IF tracks do not end in a main headland to 1 or 2 (side headlands only) */
        double sideHLAngThresholdBoundIFTracks = 45; /**< Angular threshold (in degrees, between 0 and 90) used to obtain the reference linestring for the main headlands when comparing the angle between the IF tracks and the boundary. If <0, uses default */

        HeadlandParameters() = default;

        /**
         * @brief Parse the parameters from a string map, starting from a default PlannerParameters
         * @param [out] param Parameters
         * @param map String map containing the parameter values
         * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the default values will remain)
         * @return True on success
         */
        static bool parseFromStringMap( HeadlandParameters& params, const std::map<std::string, std::string>& map, bool strict = false);

        /**
         * @brief Parse the parameters to a string map
         * @param param Parameters
         * @return String map containing the parameter values
         */
        static std::map<std::string, std::string> parseToStringMap(const HeadlandParameters &params);

    };


    /**
     * @brief Tracks' generator parameters
     */
    struct InfieldParameters : public virtual geometry::TracksGenerator::TracksGeneratorParameters{

        /**
         * @brief Parse the parameters from a string map, starting from a default TracksGeneratorParameters
         * @param [out] param Parameters
         * @param map String map containing the parameter values
         * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the default values will remain)
         * @return True on success
         */
        static bool parseFromStringMap( InfieldParameters& params, const std::map<std::string, std::string>& map, bool strict = false);

        /**
         * @brief Parse the parameters to a string map
         * @param param Parameters
         * @return String map containing the parameter values
         */
        static std::map<std::string, std::string> parseToStringMap( const InfieldParameters& params);
    };

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit FieldGeometryProcessor(const LogLevel &logLevel = LogLevel::INFO);


    /**
     * @brief Generate the geoemtries for the subfield's surrounding headland and inner-field.
     * @param [in/out] subfield Subfield containing the necessary data (inc. outer boundary). It will be updated after the planning.
     * @param headlandParameters headland parameters
     * @param infieldParameters infield parameters
     * @param referenceLineIndex Index of the reference lines to be used
     * @param initRefPoint (optional) Reference point used to choose where the headland tracks start/end (if null or invalid, it will take either an entry point or the first point of the subfield boundary)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp processSubfieldWithSurroundingHeadland(Subfield &subfield,
                                                   const HeadlandParameters& headlandParameters,
                                                   const InfieldParameters &infieldParameters,
                                                   const size_t &referenceLineIndex = 0,
                                                   const Point* initRefPoint = nullptr);

    /**
     * @brief Generate the geoemtries for the subfield's surrounding headland and inner-field.
     * @param [in/out] subfield Subfield containing the necessary data (inc. outer boundary). It will be updated after the planning.
     * @param headlandParameters headland parameters
     * @param infieldParameters infield parameters
     * @param initRefPoint Reference point used to choose where the headland tracks start/end (if invalid, it will take either an entry point or the first point of the subfield boundary)
     * @param referenceLineIndex Index of the reference lines to be used
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp processSubfieldWithSurroundingHeadland(Subfield &subfield,
                                                   const HeadlandParameters& headlandParameters,
                                                   const InfieldParameters &infieldParameters,
                                                   const Point &initRefPoint,
                                                   const size_t &referenceLineIndex = 0);


    /**
     * @brief Generate the geoemtries for the subfield's side headlands and inner-field.
     * @param [in/out] subfield Subfield containing the necessary data (inc. outer boundary). It will be updated after the planning.
     * @param headlandParameters headland parameters
     * @param infieldParameters infield parameters
     * @param referenceLineIndex Index of the reference lines to be used
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp processSubfieldWithSideHeadlands(Subfield &subfield,
                                             const HeadlandParameters& headlandParameters,
                                             const InfieldParameters &infieldParameters,
                                             const size_t &referenceLineIndex = 0);

    /**
     * @brief Generate the subfield's infield boundary, surrounding headland, and (if applicable) headland tracks.
     * @param [in/out] subfield Subfield containing the necessary data (inc. outer boundary). It will be updated after the planning.
     * @param headlandParameters headland parameters
     * @param initRefPoint (optional) Reference point used to choose where the tracks start/end (if null, it will take either an entry point or the first point of the subfield boundary)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generateSurroundingHeadland(Subfield &subfield,
                                        const HeadlandParameters& headlandParameters,
                                        const Point* initRefPoint = nullptr);


    /**
     * Generates the partial headlands based on an infield reference line so that the headlands are located in the extrema of the IF tracks.
     * @param subfield subfield
     * @param headlandParameters headland parameters
     * @param refLine Reference line for the IF tracks
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generateSideHeadlands(Subfield &subfield,
                                  const HeadlandParameters& headlandParameters,
                                  Linestring refLine,
                                  double trackDistanceIF);



    /**
     * @brief Generate the inner-field tracks for all subfields in a field.
     * @param [in/out] field Field to be processed (must include necessary data for each subfield, inc. reference lines)
     * @param infieldParameters infield parameters
     * @param referenceLineIndexes (optional) Indexes of the reference lines to be used in each subfield. If no index is given for a subfield, the first reference line (ind = 0) is used
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp processInfield(Field &field,
                           const InfieldParameters &infieldParameters,
                           const std::vector<size_t> &referenceLineIndexes = std::vector<size_t>(0));

    /**
     * @brief Generate the inner-field tracks of a subfield.
     * @param [in/out] subfield Subield to be processed (must include necesary data, inc. reference lines)
     * @param infieldParameters infield parameters
     * @param referenceLineIndex Index of the reference lines to be used
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp processInfield(Subfield &subfield,
                           const InfieldParameters &infieldParameters,
                           const size_t &referenceLineIndex = 0);

    /**
     * @brief Generate the inner-field tracks of a subfield.
     * @param [in/out] subfield Subield to be processed (must include necesary data, inc. reference lines)
     * @param infieldParameters infield parameters
     * @param trackDistances Distances between tracks (if more than one, the distances will be set cyclically). Disregards infieldParameters.trackDistance
     * @param referenceLineIndex Index of the reference lines to be used
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp processInfield(Subfield &subfield,
                           const InfieldParameters &infieldParameters,
                           const std::vector<double> & trackDistances,
                           const size_t &referenceLineIndex = 0);


protected:

    /**
     * @brief Obtain the headland width, tracks' width and number of tracks based on the planner parameters.
     *
     * @param plannerParameters Planner parameters
     * @param [out] headlandWidth Headland width
     * @param [out] trackWidth Track width
     * @param [out] numTracks Number of tracks
     * @return True on success
     */
    AroResp getHeadlandParameters(const HeadlandParameters& plannerParameters,
                                  double& headlandWidth,
                                  double& trackWidth,
                                  size_t& numTracks);

    /**
     * @brief Reorder the outer boundary points so that the first one is the closest to an access point or the lastPoint (if given)
     * @param [in/out] sf Subfield to be updated
     * @param lastPoint (optional) Pointer to the last point from a previous plan (e.g. from an adjacent subfield)
     * @return True on success
     */
    bool reorderSubfieldPoints(Subfield & sf, const Point * lastPoint = nullptr);


    /**
     * @brief Get the best starting point based on the access points
     * @param sf Subfield
     * @param tracksIF Inner field tracks
     * @return Staring point (default = first point of the outer boundary)
     */
    Point getStartingPoint(const Subfield &sf, const std::vector<Track> &tracksIF);

    /**
     * @brief It generates the infield boundary, planned headland, and (if applicable) headland tracks and (harvester) routes for the given subfield.
     * @param [in/out] subfield subfield to be processed
     * @param headlandWidth Headland width
     * @param trackWidth Track width
     * @param numTracks Number of tracks
     * @param startingPoint initial reference / starting point
     * @param settings Planner parameters
     * @param tracksIF Infield tracks used to sample the inmost headlsnd track
     * @return AroResp with error id (0:=OK) and message
     */
    bool processSubfieldSurroundingHeadland(Subfield& subfield,
                                            double headlandWidth,
                                            double trackWidth,
                                            size_t numTracks,
                                            Point startingPoint,
                                            const HeadlandParameters& settings,
                                            const std::vector<Track>& tracksIF = {});

    /**
     * @brief Add initial samples to the track using the infield tracks as reference
     * @param [in/out] trackHL Headland track to be processed
     * @param tracksIF Reference infield tracks
     * @param sampleResHL Sample resolution for the HL tracks
     */
    void sampleSurroundingTrackBasedOnIFTracks(Track& trackHL,
                                               const std::vector<Track>& tracksIF,
                                               double sampleResHL);

    /**
     * @brief Sample the tracks using perpendicular vectors (obtained from previous sampled tracks)
     *
     * It samples the most inward track first (reference track). For each sample in this track, it calculates the vector perpendicular to the samples segment (starting at the sample) and obtains the intersection of this perpendicular vector with the next track (appending it as a sample of the next track).
     * When all samples of the reference track are processed, the next track is sampled (without loosing the samples generated with the perpendicular vectors). The new sampled track is now taken as the reference track to process the following track in the subfield.
     * @param [in/out] headland Headland to be processed
     * @param resolution Sample resolution for the tracks
     * @param headlandWidth Headland width
     */
    void sampleHeadlandTracksPerpendicularly(CompleteHeadland& headland,
                                             double resolution,
                                             double headlandWidth);

    /**
     * @brief Sample the tracks by adding samples as close as possible to the samples in the previous (inwards) track
     *
     * It samples the most inward track first (reference track). For each sample in this track, it calculates the point in the next track closest to it (appending it as a sample of the next track).
     * When all samples of the reference track are processed, the next track is sampled (without loosing the samples generated with the perpendicular vectors). The new sampled track is now taken as the reference track to process the following track in the subfield.
     * @param [in/out] headland to be processed
     * @param resolution Sample resolution for the tracks
     * @param headlandWidth Headland width
     */
    void sampleHeadlandTracksWithClosestPoints(CompleteHeadland& headland,
                                               double resolution);

    /**
     * @brief Sample the tracks of the partial headland based on the infield tracks
     * @param [in/out] subfield subfield
     * @param resolution Sample resolution for the tracks
     * @param headlandWidth Headland width
     */
    void samplePartialHLTracksBasedOnIFTracks(Subfield &subfield, double resolution);

    /**
     * @brief Adjust and remove the IF tracks needed for side headland generation based on their length and the HL width
     * @param boundary boundary
     * @param [in/out] tracksIF reference inner-field tracks (tracks that are disregarded to compute the hl_points are removed)
     * @param headlandWidth Headland width
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp adjustIFTracksForSideHLGeneration(Polygon &boundary,
                                              std::vector<Track> &tracksIF,
                                              double headlandWidth,
                                              const HeadlandParameters &hlParams);


    /**
     * @brief Generates the reference lines that will be initially used to create the partial headlands based on some reference inner-field tracks
     * @param boundary boundary
     * @param [in/out] tracksIF reference inner-field tracks (tracks that are disregarded to compute the hl_points are removed)
     * @param headlandWidth Headland width
     * @param trackDistanceIF Distance between inner-field tracks
     * @param plannerParameters Planner parameters
     * @param [out] hl_points_1 Reference line for headland 1
     * @param [out] hl_points_2 Reference line for headland 2
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp createPartialHLInitialReferenceLines(Polygon &boundary,
                                                 std::vector<Track> &tracksIF,
                                                 double headlandWidth,
                                                 double trackDistanceIF,
                                                 const HeadlandParameters& plannerParameters,
                                                 std::vector<Point>& hl_points_1,
                                                 std::vector<Point>& hl_points_2,
                                                 size_t maxRemoveTracks,
                                                 std::pair<size_t, size_t> countRemovedTracks = std::make_pair(0,0));

    /**
     * @brief Takes two overlaping partial headlands and creates a combined headland with the given headland width
     * @param subfield Subfield
     * @param hl1 Partial headland boundary (side 1)
     * @param hl2 Partial headland boundary (side 2)
     * @param intersectionPoly Intersection polygon between hl1 and hl2
     * @param if_tracks Inner-field tracks used to generate the partial headlands
     * @param headlandWidth Headland width
     * @param [out] combinedHL Combined headland boundary
     * @param [out] hl_points_combined Reference points for the combined headland
     * @return True on success
     */
    bool combineOverlappingHeadlands(const Subfield& subfield,
                                     std::vector<Point> hl_points_1,
                                     std::vector<Point> hl_points_2,
                                     const Polygon& hl1,
                                     const Polygon& hl2,
                                     Polygon &intersectionPoly,
                                     std::vector<Track> if_tracks,
                                     double headlandWidth,
                                     Polygon& combinedHL,
                                     std::vector<Point> &hl_points_combined);

    /**
     * @brief Generates the tracks of the partial headlands (from inner-most track to outer-most track) using a TracksGenerator
     * @param [in/out] subfield subfield to be processed
     * @param trackDistance Distance between tracks
     * @param plannerParameters Planner parameters
     * @param expectedNumTracks Expected number of tracks
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generatePartialHLTracksWithTracksGenerator(Subfield& subfield,
                                                       double trackDistance,
                                                       const HeadlandParameters& plannerParameters,
                                                       size_t expectedNumTracks);

    /**
     * @brief Generates the tracks of the partial headlands (from outer-most track to inner-most track) using a TracksGenerator
     * @param [in/out] subfield subfield to be processed
     * @param trackDistance Distance between tracks
     * @param plannerParameters Planner parameters
     * @param expectedNumTracks Expected number of tracks
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generatePartialHLTracksWithTracksGenerator(Subfield& subfield,
                                                       const std::vector<Point>& hl_points_1,
                                                       const std::vector<Point>& hl_points_2,
                                                       double trackDistance,
                                                       const HeadlandParameters& plannerParameters,
                                                       size_t expectedNumTracks);

    /**
     * @brief Generates the tracks of the partial headlands using a TracksGenerator and a given reference line
     * @param boundary boundary
     * @param refLineHL Reference line
     * @param trackDistance Distance between tracks
     * @param sampleResolution Sample resolution for the headland tracks
     * @param isRefLineInwards Is the given reference line towards the innermost part of the headland?
     * @param useRefLineAsCentralLine Use the given reference line as the track central line?
     * @param expectedNumTracks Expected number of tracks
     * @param headlandNumber Headland number
     * @param [out] tracks Generated tracks
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp generatePartialHLTracksWithTracksGenerator(const Polygon& boundary,
                                                       const std::vector<Point>& refLineHL,
                                                       double trackDistance,
                                                       double sampleResolution,
                                                       bool isRefLineInwards,
                                                       bool useRefLineAsCentralLine,
                                                       size_t expectedNumTracks,
                                                       size_t headlandNumber,
                                                       std::vector<Track>& tracks);


    /**
     * @brief Generates partial headland(s) connecting the partial headlands located at the extrema of the IF tracks)
     * @param [in/out] subfield subfield to be processed
     * @param trackWidth Width of the track for the connecting headland
     * @param sampleResolution Sample resolution for the connecting headland track
     * @param headlandWidth Width of the original partial headlands
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp addConnectingHeadlands(Subfield& subfield,
                                   double trackWidth,
                                   double sampleResolution,
                                   double headlandWidth);

    /**
     * @brief Adjust the track ends of main headlands
     * @param [in/out] subfield subfield to be processed
     * @param trackWidth Width of the track for the connecting headland
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp adjustMainHeadlandTrackEnds(Subfield& subfield,
                                       double trackWidth);

protected:
    static const double m_unsampleAngTol; /**< Angular tolerance to unsample linestrings */
};


}

#endif // AROLIB_FILELGEOMETRYPROCESSOR_H
