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
 
#include "arolib/components/fieldgeometryprocessor.h"
namespace arolib {

//const double FieldGeometryProcessor::m_unsampleAngTol = deg2rad(0.3);
const double FieldGeometryProcessor::m_unsampleAngTol = deg2rad(0.05);

FieldGeometryProcessor::HeadlandType FieldGeometryProcessor::intToHeadlandType(int value)
{
    if(value == HeadlandType::SURROUNDING_HEADLAND)
        return HeadlandType::SURROUNDING_HEADLAND;
    else if(value == HeadlandType::SIDES_HEADLANDS)
        return HeadlandType::SIDES_HEADLANDS;

    throw std::invalid_argument( "The given value does not correspond to any FieldGeometryProcessor::HeadlandType" );

}

FieldGeometryProcessor::HeadlandParameters::TracksSamplingStrategy FieldGeometryProcessor::HeadlandParameters::intToTracksSamplingStrategy(int value)
{
    if(value == HeadlandParameters::TracksSamplingStrategy::SIMPLE_TRACK_SAMPLING)
        return HeadlandParameters::TracksSamplingStrategy::SIMPLE_TRACK_SAMPLING;
    else if(value == HeadlandParameters::TracksSamplingStrategy::SAMPLE_TRACKS_PERPENDICULARILY)
        return HeadlandParameters::TracksSamplingStrategy::SAMPLE_TRACKS_PERPENDICULARILY;
    else if(value == HeadlandParameters::TracksSamplingStrategy::SAMPLE_TRACKS_WITH_CLOSEST_SAMPLES)
        return HeadlandParameters::TracksSamplingStrategy::SAMPLE_TRACKS_WITH_CLOSEST_SAMPLES;

    throw std::invalid_argument( "The given value does not correspond to any FieldGeometryProcessor::HeadlandSettings::TracksSamplingStrategy" );
}

FieldGeometryProcessor::HeadlandParameters::SideHeadlandGenerationStrategy FieldGeometryProcessor::HeadlandParameters::intToSideHeadlandGenerationStrategy(int value)
{
    if(value == HeadlandParameters::SideHeadlandGenerationStrategy::SIDEHL_STRATEGY_DEF)
        return HeadlandParameters::SideHeadlandGenerationStrategy::SIDEHL_STRATEGY_DEF;
    else if(value == HeadlandParameters::SideHeadlandGenerationStrategy::SIDEHL_STRATEGY_1)
        return HeadlandParameters::SideHeadlandGenerationStrategy::SIDEHL_STRATEGY_1;
    else if(value == HeadlandParameters::SideHeadlandGenerationStrategy::SIDEHL_STRATEGY_2)
        return HeadlandParameters::SideHeadlandGenerationStrategy::SIDEHL_STRATEGY_2;
    else if(value == HeadlandParameters::SideHeadlandGenerationStrategy::SIDEHL_STRATEGY_3)
        return HeadlandParameters::SideHeadlandGenerationStrategy::SIDEHL_STRATEGY_3;

    throw std::invalid_argument( "The given value does not correspond to any FieldGeometryProcessor::HeadlandSettings::SideHeadlandGenerationStrategy" );

}

FieldGeometryProcessor::HeadlandParameters::SideHeadlandTracksGenerationStrategy FieldGeometryProcessor::HeadlandParameters::intToSideHeadlandTracksGenerationStrategy(int value)
{
    if(value == HeadlandParameters::SideHeadlandTracksGenerationStrategy::SIDEHLTRACKS_STRATEGY_DEF)
        return HeadlandParameters::SideHeadlandTracksGenerationStrategy::SIDEHLTRACKS_STRATEGY_DEF;
    else if(value == HeadlandParameters::SideHeadlandTracksGenerationStrategy::SIDEHLTRACKS_FROM_OUTER_BOUNDARY)
        return HeadlandParameters::SideHeadlandTracksGenerationStrategy::SIDEHLTRACKS_FROM_OUTER_BOUNDARY;
    else if(value == HeadlandParameters::SideHeadlandTracksGenerationStrategy::SIDEHLTRACKS_FROM_INNER_BOUNDARY)
        return HeadlandParameters::SideHeadlandTracksGenerationStrategy::SIDEHLTRACKS_FROM_INNER_BOUNDARY;
    else if(value == HeadlandParameters::SideHeadlandTracksGenerationStrategy::SIDEHLTRACKS_CHECK_ALL)
        return HeadlandParameters::SideHeadlandTracksGenerationStrategy::SIDEHLTRACKS_CHECK_ALL;

    throw std::invalid_argument( "The given value does not correspond to any FieldGeometryProcessor::HeadlandSettings::SideHeadlandTracksGenerationStrategy" );
}

bool FieldGeometryProcessor::HeadlandParameters::parseFromStringMap(HeadlandParameters &params, const std::map<std::string, std::string> &map, bool strict)
{
    FieldGeometryProcessor::HeadlandParameters tmp;

    int tracksSamplingStrategy = FieldGeometryProcessor::HeadlandParameters::SAMPLE_TRACKS_PERPENDICULARILY;
    int sideHLGenerationStrategy = FieldGeometryProcessor::HeadlandParameters::SIDEHL_STRATEGY_DEF;
    int sideHLTracksGenerationStrategy = FieldGeometryProcessor::HeadlandParameters::SIDEHLTRACKS_STRATEGY_DEF;
    std::map<std::string, double*> dMap = { {"headlandWidth" , &tmp.headlandWidth},
                                            {"trackWidth" , &tmp.trackWidth},
                                            {"sampleResolution" , &tmp.sampleResolution},
                                            {"headlandConnectionTrackWidth" , &tmp.headlandConnectionTrackWidth},
                                            {"sideHLAngThresholdBoundIFTracks" , &tmp.sideHLAngThresholdBoundIFTracks} };
    std::map<std::string, bool*> bMap = { {"trimPartialHeadlandTrackEnds" , &tmp.trimPartialHeadlandTrackEnds},
                                          {"limitIFTracksNotOverMainHL" , &tmp.limitIFTracksNotOverMainHL}};
    std::map<std::string, size_t*> uiMap = { {"numTracks" , &tmp.numTracks} };
    std::map<std::string, int*> enumMap = { {"tracksSamplingStrategy" , &tracksSamplingStrategy},
                                            {"sideHLGenerationStrategy" , &sideHLGenerationStrategy},
                                            {"sideHLTracksGenerationStrategy" , &sideHLTracksGenerationStrategy} };

    if( !setValuesFromStringMap( map, dMap, strict)
            || !setValuesFromStringMap( map, bMap, strict)
            || !setValuesFromStringMap( map, uiMap, strict)
            || !setValuesFromStringMap( map, enumMap, strict) )
        return false;

    tmp.tracksSamplingStrategy = FieldGeometryProcessor::HeadlandParameters::intToTracksSamplingStrategy( tracksSamplingStrategy );
    tmp.sideHLGenerationStrategy = FieldGeometryProcessor::HeadlandParameters::intToSideHeadlandGenerationStrategy( sideHLGenerationStrategy );
    tmp.sideHLTracksGenerationStrategy = FieldGeometryProcessor::HeadlandParameters::intToSideHeadlandTracksGenerationStrategy( sideHLTracksGenerationStrategy );

    params = tmp;
    return true;
}

std::map<std::string, std::string> FieldGeometryProcessor::HeadlandParameters::parseToStringMap(const FieldGeometryProcessor::HeadlandParameters &params)
{
    std::map<std::string, std::string> ret;
    ret["headlandWidth"] = double2string( params.headlandWidth );
    ret["trackWidth"] = double2string( params.trackWidth );
    ret["sampleResolution"] = double2string( params.sampleResolution );
    ret["headlandConnectionTrackWidth"] = double2string( params.headlandConnectionTrackWidth );
    ret["sideHLAngThresholdBoundIFTracks"] = double2string( params.sideHLAngThresholdBoundIFTracks );
    ret["trimPartialHeadlandTrackEnds"] = std::to_string( params.trimPartialHeadlandTrackEnds );
    ret["limitIFTracksNotOverMainHL"] = std::to_string( params.limitIFTracksNotOverMainHL );
    ret["numTracks"] = std::to_string( params.numTracks );
    ret["tracksSamplingStrategy"] = std::to_string( params.tracksSamplingStrategy );
    ret["sideHLTracksGenerationStrategy"] = std::to_string( params.sideHLTracksGenerationStrategy );
    return ret;
}

bool FieldGeometryProcessor::InfieldParameters::parseFromStringMap(FieldGeometryProcessor::InfieldParameters &params, const std::map<std::string, std::string> &map, bool strict)
{
    FieldGeometryProcessor::InfieldParameters tmp;

    if( !TracksGeneratorParameters::parseFromStringMap(tmp, map, strict) )
        return false;

    params = tmp;
    return true;
}

std::map<std::string, std::string> FieldGeometryProcessor::InfieldParameters::parseToStringMap(const FieldGeometryProcessor::InfieldParameters &params)
{
    std::map<std::string, std::string> ret, subMap;
    subMap = TracksGeneratorParameters::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );
    return ret;
}




FieldGeometryProcessor::FieldGeometryProcessor(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{

}

AroResp FieldGeometryProcessor::processSubfieldWithSurroundingHeadland(Subfield &subfield,
                                                                       const HeadlandParameters &headlandParameters,
                                                                       const InfieldParameters &infieldParameters,
                                                                       const size_t &referenceLineIndex,
                                                                       const Point *initRefPoint)
{
    Subfield sfTmp = subfield;
    HeadlandParameters hlParamsTmp;
    double headlandWidth;
    double trackWidth;
    size_t numTracks;   

    subfield.headlands.clear();

    auto aroResp = getHeadlandParameters(headlandParameters,
                                         headlandWidth,
                                         trackWidth,
                                         numTracks);
    if(aroResp.isError())
        return aroResp;

    hlParamsTmp.headlandWidth = headlandWidth;
    hlParamsTmp.trackWidth = hlParamsTmp.numTracks = 0;

    //Initially generate the headland and IF boundary for the IF tracks generation
    aroResp = generateSurroundingHeadland(sfTmp, hlParamsTmp);
    if(aroResp.isError()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating initial headland");
        return AroResp(1, "Error generating initial headland: " + aroResp.msg);
    }

    //compute the IF geometries (tracks, )
    aroResp = processInfield(sfTmp, infieldParameters, referenceLineIndex);
    if(aroResp.isError()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating infield geometries");
        return AroResp(1, "Error generating infield geometries: " + aroResp.msg);
    }

    //re-order subfield boundary points
    if(!reorderSubfieldPoints(subfield, nullptr)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Problem reordering outer boundary points of subfield");
        return AroResp(1, "Problem reordering outer boundary points of subfield");
    }

    Point startingPoint;

    if(initRefPoint && initRefPoint->isValid())
        startingPoint = *initRefPoint;
    else
        startingPoint = getStartingPoint(subfield, sfTmp.tracks);

    //(re-)generate the final headland geometries based on the IF tracks
    if(! processSubfieldSurroundingHeadland( subfield,
                                             headlandWidth,
                                             trackWidth,
                                             numTracks,
                                             startingPoint,
                                             headlandParameters,
                                             sfTmp.tracks) ){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating headlands");
        return AroResp(1, "Error generating headlands");
    }

    //@note all the generated IF geometries must be copied
    subfield.tracks = sfTmp.tracks;

    return AroResp(0, "OK");

}

AroResp FieldGeometryProcessor::processSubfieldWithSurroundingHeadland(Subfield &subfield,
                                                                       const HeadlandParameters& headlandParameters,
                                                                       const InfieldParameters &infieldParameters,
                                                                       const Point &initRefPoint,
                                                                       const size_t &referenceLineIndex)
{
    return processSubfieldWithSurroundingHeadland(subfield,
                                                  headlandParameters,
                                                  infieldParameters,
                                                  referenceLineIndex,
                                                  &initRefPoint);
}

AroResp FieldGeometryProcessor::processSubfieldWithSideHeadlands(Subfield &subfield, const HeadlandParameters &headlandParameters, const InfieldParameters &infieldParameters, const size_t &referenceLineIndex)
{
    if(subfield.reference_lines.size() <=  referenceLineIndex) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid reference line index '" + std::to_string(referenceLineIndex) + " : Nr. of reference lines is " + std::to_string(subfield.reference_lines.size()));
        return AroResp(1, "Invalid reference line index" );
    }

    subfield.headlands.clear();

    auto hlParams = headlandParameters;
    hlParams.sampleResolution = -1;

    if(headlandParameters.sampleResolution > 1e-9)
        hlParams.trimPartialHeadlandTrackEnds = false;

    auto aroResp = generateSideHeadlands(subfield, hlParams, subfield.reference_lines.at(referenceLineIndex), /*0.5 **/ infieldParameters.trackDistance);
    if(aroResp.isError()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating headlands");
        return AroResp(1, "Error generating headlands: " + aroResp.msg);
    }

    aroResp = processInfield(subfield, infieldParameters, referenceLineIndex);
    if(aroResp.isError()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating infield geometries");
        return AroResp(1, "Error generating infield geometries: " + aroResp.msg);
    }

    if(headlandParameters.sampleResolution > 1e-9)
        samplePartialHLTracksBasedOnIFTracks(subfield, headlandParameters.sampleResolution);

    if(headlandParameters.trimPartialHeadlandTrackEnds && hlParams.trimPartialHeadlandTrackEnds != headlandParameters.trimPartialHeadlandTrackEnds){
        double headlandWidth;
        double trackWidth;
        size_t numTracks;

        aroResp = getHeadlandParameters(headlandParameters, headlandWidth, trackWidth, numTracks);
        if(aroResp.isError())
            return AroResp::LoggingResp(1, "Error obtaining HL papameters: " + aroResp.msg, m_logger, LogLevel::ERROR, __FUNCTION__);

        double trackWidthConn = headlandParameters.headlandConnectionTrackWidth;
        if(trackWidthConn < -1e-5)
            trackWidthConn = trackWidth;
        aroResp = adjustMainHeadlandTrackEnds(subfield, trackWidthConn > 1e-5 ? trackWidthConn : trackWidth);
        if(aroResp.isError())
            return AroResp::LoggingResp(1, "Error adjusting track ends of main partial headlands: " + aroResp.msg, m_logger, LogLevel::ERROR, __FUNCTION__);
    }

    return AroResp(0, "OK");

}


AroResp FieldGeometryProcessor::generateSurroundingHeadland(Subfield &subfield,
                                                            const HeadlandParameters& headlandParameters,
                                                            const Point* initRefPoint)
{
    logger().printOut(LogLevel::INFO, __FUNCTION__, "Planning headland for a single subfield");

    subfield.headlands.clear();

    double headlandWidth;
    double trackWidth;
    size_t numTracks;

    auto aroResp = getHeadlandParameters(headlandParameters,
                                         headlandWidth,
                                         trackWidth,
                                         numTracks);
    if(aroResp.isError())
        return aroResp;

    try {

        if(!reorderSubfieldPoints(subfield, nullptr)){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Problem reordering outer boundary points of subfield");
            return AroResp(1, "Problem reordering outer boundary points of subfield");
        }

        Point startingPoint;

        if(initRefPoint)
            startingPoint = *initRefPoint;
        else
            startingPoint = getStartingPoint(subfield, subfield.tracks);

        if(! processSubfieldSurroundingHeadland( subfield,
                                                 headlandWidth,
                                                 trackWidth,
                                                 numTracks,
                                                 startingPoint,
                                                 headlandParameters ) ){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error processing subfield.");
            return AroResp(1, "Error processing subfield");
        }

        return AroResp(0, "OK");
    }
    catch (std::exception &e) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what());
        return AroResp(1, e.what());
    }
}

AroResp FieldGeometryProcessor::generateSideHeadlands(Subfield &subfield,
                                                      const HeadlandParameters &headlandParameters,
                                                      Linestring refLine,
                                                      double trackDistanceIF)
{
    subfield.headlands.clear();

    Subfield sfTmp;//temporary subfield

    arolib::geometry::unsample_linestring(refLine.points);
    if(refLine.points.size() < 2){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid reference line");
        return AroResp(1, "Invalid reference line");
    }
    if(trackDistanceIF <= 0){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid track distance (IF)");
        return AroResp(1, "Invalid track distance (IF)");
    }

    double headlandWidth;
    double trackWidth;
    size_t numTracks;

    auto aroResp = getHeadlandParameters(headlandParameters, headlandWidth, trackWidth, numTracks);
    if(aroResp.isError())
        return aroResp;

    double trackWidthConn = headlandParameters.headlandConnectionTrackWidth;
    if(trackWidthConn < -1e-5)
        trackWidthConn = trackWidth;

    if(std::fabs(trackWidthConn - trackWidth) < 1e-5 && numTracks == 1){
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "The single-track partial headlands will connect with tracks of the same width. Creating a complete headland instead");
        auto params_ed = headlandParameters;
        params_ed.headlandWidth = headlandWidth;
        params_ed.numTracks = numTracks;
        params_ed.trackWidth = trackWidth;
        return generateSurroundingHeadland(subfield, params_ed, nullptr);
    }

//    if(trackWidthConn >= headlandWidth){
//        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The track width of the connecting headland must be lower than the headland width");
//        return AroResp(1, "The track width of the connecting headland must be lower than the headland width");
//    }


    sfTmp.boundary_outer = subfield.boundary_outer;

    //obtain the temporary IF tracks
    geometry::TracksGenerator trGen;
    trGen.logger().setParent(loggerPtr());
    FieldGeometryProcessor::InfieldParameters tgParams;
    tgParams.sampleResolution = -1;
    tgParams.trackAreaThreshold = 0.9;
    tgParams.onlyUntilBoundaryIntersection = true;
    tgParams.splitBoundaryIntersectingTracks = false;
    tgParams.trackSamplingStrategy = geometry::TracksGenerator::START_AT_TRACK_START;
    tgParams.checkForRemainingTracks = true;

    size_t divTrackDistance = 1;
    size_t maxRemovedTracks = 1 + divTrackDistance;
    //size_t maxRemovedTracks = 10 + divTrackDistance;

    aroResp = trGen.generateTracks(sfTmp.boundary_outer, refLine.points, {trackDistanceIF/divTrackDistance}, tgParams, sfTmp.tracks);
    if(aroResp.isError()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating temporary IF tracks: " + aroResp.msg);
        return AroResp(1, "Error generating temporary IF tracks: " + aroResp.msg);
    }

    geometry::unsample_polygon(sfTmp.boundary_outer);

    aroResp = adjustIFTracksForSideHLGeneration(sfTmp.boundary_outer, sfTmp.tracks, headlandWidth, headlandParameters);
    if(aroResp.isError())
        return aroResp;

    std::vector<Point> hl_points_1, hl_points_2;
    aroResp = createPartialHLInitialReferenceLines(sfTmp.boundary_outer, sfTmp.tracks,
                                                   headlandWidth, trackDistanceIF, headlandParameters,
                                                   hl_points_1, hl_points_2,
                                                   maxRemovedTracks);
    if(aroResp.isError())
        return aroResp;

    Polygon hl1, hl2;

    arolib::geometry::offsetLinestring( hl_points_1, hl1, headlandWidth, headlandWidth, true, 9 );
    arolib::geometry::offsetLinestring( hl_points_2, hl2, headlandWidth, headlandWidth, true, 9 );

    arolib::geometry::closePolygon(hl1);
    arolib::geometry::closePolygon(hl2);

    arolib::geometry::unsample_polygon(hl1);
    arolib::geometry::unsample_polygon(hl2);

    arolib::geometry::correct_polygon(hl1);
    arolib::geometry::correct_polygon(hl2);

    for(int hl_side = 1 ; hl_side <= 2 ; ++hl_side){
        auto& hlPoly = (hl_side == 1 ? hl1 : hl2);
        auto& hlPts = (hl_side == 1 ? hl_points_1 : hl_points_2);
        auto intersections = geometry::get_intersection(sfTmp.boundary_outer, hlPoly);
        if(intersections.empty()){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating boundary of headland " + std::to_string(hl_side) + " (no valid intersection with boundary)");
            return AroResp(1, "Error generating boundary of headland " + std::to_string(hl_side) + " (no valid intersection with boundary)");
        }

        hlPoly = intersections.front();
        if(intersections.size() > 1){//in case there exist more than one intersections because of the extensions...
            Point control_pt = hlPts.size() > 2 ? hlPts.at(1) : geometry::getCentroid(hlPts.at(0), hlPts.at(1));
            double minDist = geometry::calc_dist_to_linestring(hlPoly.points, control_pt);
            for(size_t i = 1 ; i < intersections.size() ; ++i){
                double dist = geometry::calc_dist_to_linestring(intersections.at(i).points, control_pt);
                if(dist < minDist){
                    hlPoly = intersections.at(i);
                    minDist = dist;
                }
            }
        }
    }

    //get headlands' boundaries
    auto intersections1 = arolib::geometry::get_intersection(hl1, subfield.boundary_outer);
    if(intersections1.size() != 1){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error processing partial headlands (Intersection hl1 and boundary)");
        return AroResp(1, "Error processing partial headlands (Intersection hl1 and boundary)");
    }

    auto intersections2 = arolib::geometry::get_intersection(hl2, subfield.boundary_outer);
    if(intersections2.size() != 1){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error processing partial headlands (Intersection hl2 and boundary)");
        return AroResp(1, "Error processing partial headlands (Intersection hl2 and boundary)");
    }

    //check if the headlands intersect
    auto intersections_1_2 = arolib::geometry::get_intersection(intersections1.front(), intersections2.front());
    if(intersections_1_2.size() > 1){//@todo can it be possible to have more than one intersection and still intersect only in one side?
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "The partial headlands overlap in both sides. Creating a complete headland instead");
        auto params_ed = headlandParameters;
        params_ed.headlandWidth = headlandWidth;
        params_ed.numTracks = numTracks;
        params_ed.trackWidth = trackWidth;
        return generateSurroundingHeadland(subfield, params_ed, nullptr);
    }
    if(intersections_1_2.size() == 1){//the headlands intersect in one side -> combine into one propper headland
        Polygon combinedHL;
        std::vector<Point> hl_points_combined;
        if(!combineOverlappingHeadlands(subfield,
                                        hl_points_1,
                                        hl_points_2,
                                        intersections1.front(), intersections2.front(),
                                        intersections_1_2.front(), sfTmp.tracks, headlandWidth,
                                        combinedHL, hl_points_combined)){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error combining overlapping headlands");
            return AroResp(1, "Error combining overlapping headlands");
        }

        std::swap(intersections1.front(), combinedHL);
        std::swap(hl_points_1, hl_points_combined);
        intersections2.clear();

        //generate IF boundary
        std::vector<PolygonWithHoles> polys_tmp;
        Polygon hl1_ed = intersections1.front();
        geometry::offsetPolygon(intersections1.front(), hl1_ed, 1e-3, true, 0);//inflate a bit to avoid spikes and other errors
        arolib::geometry::subtract_intersection( sfTmp.boundary_outer, hl1_ed, polys_tmp );
        if(polys_tmp.size() != 1){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating IF boundary from overlapping headlands. The headland width might be too high for the field.");
            return AroResp(1, "Error generating IF boundary (side 1)");
        }
        subfield.boundary_inner = polys_tmp.front().outer;
    }
    else{
        //generate IF boundary
        std::vector<PolygonWithHoles> polys_tmp;
        Polygon hl1_ed = hl1, hl2_ed = hl2;
        geometry::offsetPolygon(intersections1.front(), hl1_ed, 1e-3, true, 0);//inflate a bit to avoid spikes and other errors
        arolib::geometry::subtract_intersection( sfTmp.boundary_outer, hl1_ed, polys_tmp );
        if(polys_tmp.size() != 1){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating IF boundary (side 1). The headland width might be too high for the field.");
            return AroResp(1, "Error generating IF boundary (side 1)");
        }
        sfTmp.boundary_inner = polys_tmp.front().outer;
        geometry::offsetPolygon(intersections2.front(), hl2_ed, 1e-3, true, 0);
        arolib::geometry::subtract_intersection( sfTmp.boundary_inner, hl2_ed, polys_tmp );
        if(polys_tmp.size() != 1){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating IF boundary (side 2). The headland width might be too high for the field.");
            return AroResp(1, "Error generating IF boundary (side 2)");
        }
        subfield.boundary_inner = polys_tmp.front().outer;
    }


    //reset subfield headlands and add new partial headlands
    subfield.headlands = Headlands();
    subfield.headlands.partial.push_back( PartialHeadland() );
    subfield.headlands.partial.back().id = subfield.headlands.partial.size()-1;
    subfield.headlands.partial.back().boundary = intersections1.front();
    if(!intersections2.empty()){
        subfield.headlands.partial.push_back( PartialHeadland() );
        subfield.headlands.partial.back().id = subfield.headlands.partial.size()-1;
        subfield.headlands.partial.back().boundary = intersections2.front();
    }

    //reduce extension points from hl_points
    double distExtrema = 2*trackWidth;
    if( geometry::calc_dist_to_linestring(subfield.boundary_outer.points, hl_points_1.front()) > 0.01 )
        hl_points_1.front() = geometry::getPointInLineAtDist(hl_points_1.at(1), hl_points_1.front(), distExtrema);
    if( geometry::calc_dist_to_linestring(subfield.boundary_outer.points, hl_points_1.back()) > 0.01 )
        hl_points_1.back() = geometry::getPointInLineAtDist(r_at(hl_points_1, 1), hl_points_1.back(), distExtrema);
    if( geometry::calc_dist_to_linestring(subfield.boundary_outer.points, hl_points_2.front()) > 0.01 )
        hl_points_2.front() = geometry::getPointInLineAtDist(hl_points_2.at(1), hl_points_2.front(), distExtrema);
    if( geometry::calc_dist_to_linestring(subfield.boundary_outer.points, hl_points_2.back()) > 0.01 )
        hl_points_2.back() = geometry::getPointInLineAtDist(r_at(hl_points_2, 1), hl_points_2.back(), distExtrema);

    auto sideHLTracksGenerationStrategy = headlandParameters.sideHLTracksGenerationStrategy;
    if(sideHLTracksGenerationStrategy == HeadlandParameters::SIDEHLTRACKS_STRATEGY_DEF)
        sideHLTracksGenerationStrategy = HeadlandParameters::SIDEHLTRACKS_FROM_OUTER_BOUNDARY;

    if(sideHLTracksGenerationStrategy == HeadlandParameters::SIDEHLTRACKS_CHECK_ALL){
        aroResp = generatePartialHLTracksWithTracksGenerator(subfield, hl_points_1, hl_points_2, trackWidth, headlandParameters, numTracks);
        sfTmp = subfield;
        auto aroResp2 = generatePartialHLTracksWithTracksGenerator(sfTmp, trackWidth, headlandParameters, numTracks);
        if(!aroResp.isError() && !aroResp2.isError()){
            for(size_t i = 0 ; i < subfield.headlands.partial.size() ; ++i){
                auto& hl_stg1 = subfield.headlands.partial.at(i);
                auto& hl_stg2 = sfTmp.headlands.partial.at(i);
                for(size_t j = 0 ; j < hl_stg1.tracks.size() && j < hl_stg2.tracks.size() ; ++j){
                    if( geometry::getGeometryLength(r_at(hl_stg1.tracks, j).points)
                            < geometry::getGeometryLength(r_at(hl_stg2.tracks, j).points) )//select the longest track
                        r_at(hl_stg1.tracks, j) = r_at(hl_stg2.tracks, j);
                }
            }
        }
        else if(!aroResp2.isError()){
            subfield = sfTmp;
            aroResp = aroResp2;
        }
    }
    else{
        if(sideHLTracksGenerationStrategy == HeadlandParameters::SIDEHLTRACKS_FROM_OUTER_BOUNDARY)
            aroResp = generatePartialHLTracksWithTracksGenerator(subfield, hl_points_1, hl_points_2, trackWidth, headlandParameters, numTracks);
        else
            aroResp = generatePartialHLTracksWithTracksGenerator(subfield, trackWidth, headlandParameters, numTracks);
        if(aroResp.isError()){//try again with the other strategy
            logger().printOut(LogLevel::WARNING, __FUNCTION__, "Error generating the headland tracks with the given strategy. Trying with another one.");
            if(sideHLTracksGenerationStrategy == HeadlandParameters::SIDEHLTRACKS_FROM_OUTER_BOUNDARY)
                aroResp = generatePartialHLTracksWithTracksGenerator(subfield, trackWidth, headlandParameters, numTracks);
            else
                aroResp = generatePartialHLTracksWithTracksGenerator(subfield, hl_points_1, hl_points_2, trackWidth, headlandParameters, numTracks);
        }
    }

    if(aroResp.isError())
        return aroResp;

    if(trackWidthConn > 1e-5){
        aroResp = addConnectingHeadlands(subfield, trackWidthConn, headlandParameters.sampleResolution, headlandWidth);
        if(aroResp.isError())
            return aroResp;
    }

    if( headlandParameters.trimPartialHeadlandTrackEnds ){
        aroResp = adjustMainHeadlandTrackEnds(subfield, trackWidthConn > 1e-5 ? trackWidthConn : trackWidth);
        if(aroResp.isError())
            return aroResp;
    }

    return AroResp::ok();

}

AroResp FieldGeometryProcessor::processInfield(Field &field, const InfieldParameters &infieldParameters, const std::vector<size_t> &referenceLineIndexes)
{
    try {
        if (infieldParameters.trackDistance == 0) {
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid track width (=0)" );
            return AroResp(1, "Invalid track width (=0)" );
        }

        if (referenceLineIndexes.size() != 0 && referenceLineIndexes.size() != field.subfields.size()) {
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid number of reference line indexes (must be 0 or equal to the number of subfields)" );
            return AroResp(1, "Invalid number of reference line indexes (must be 0 or equal to the number of subfields)" );
        }

        for (int i = 0; i < field.subfields.size(); ++i) {
            size_t referenceLineIndex = 0;
            if(!referenceLineIndexes.empty())
                referenceLineIndex = referenceLineIndexes.at(i);

            Subfield &sf = field.subfields.at(i);
            // check for reference line
            if(sf.reference_lines.empty()) {
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Subfield " + std::to_string(i) + " has no reference lines.");
                return AroResp(1, "Subfield " + std::to_string(i) + " has no reference lines." );
            }
            if(sf.reference_lines.size() <=  referenceLineIndex) {
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid reference line index '" + std::to_string(referenceLineIndex) + "' for subfield " + std::to_string(i) +
                         " : Nr. of reference lines is " + std::to_string(sf.reference_lines.size()));
                return AroResp(1, "Invalid reference line index" );
            }

            logger().printOut(LogLevel::INFO, __FUNCTION__, "Shifting subfield " + std::to_string(i) + "...");
            AroResp compResp = processInfield(sf,
                                              infieldParameters,
                                              referenceLineIndex);

            if (compResp.errorID > 0){
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Problem with subfield " + std::to_string(i) );
                return AroResp(1, "Problem with subfield " + std::to_string(i) + ": " + compResp.msg );
            }
        }

        return AroResp(0, "OK" );
    }
    catch (std::exception &e) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what() );
        return AroResp(1, e.what() );
    }
}

AroResp FieldGeometryProcessor::processInfield(Subfield &subfield, const InfieldParameters &infieldParameters, const size_t &referenceLineIndex)
{
    if(subfield.reference_lines.size() <=  referenceLineIndex) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid reference line index '" + std::to_string(referenceLineIndex) + " : Nr. of reference lines is " + std::to_string(subfield.reference_lines.size()));
        return AroResp(1, "Invalid reference line index" );
    }
    geometry::TracksGenerator tg;
    tg.logger().setParent(loggerPtr());
    return tg.generateTracks(subfield.boundary_inner,
                             subfield.reference_lines.at(referenceLineIndex).points,
                             {infieldParameters.trackDistance},
                             infieldParameters,
                             subfield.tracks);
}

AroResp FieldGeometryProcessor::processInfield(Subfield &subfield,
                                               const InfieldParameters &infieldParameters,
                                               const std::vector<double> &trackDistances,
                                               const size_t &referenceLineIndex)
{
    if(subfield.reference_lines.size() <=  referenceLineIndex) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid reference line index '" + std::to_string(referenceLineIndex) + " : Nr. of reference lines is " + std::to_string(subfield.reference_lines.size()));
        return AroResp(1, "Invalid reference line index" );
    }
    geometry::TracksGenerator tg;
    tg.logger().setParent(loggerPtr());
    return tg.generateTracks(subfield.boundary_inner,
                             subfield.reference_lines.at(referenceLineIndex).points,
                             trackDistances,
                             infieldParameters,
                             subfield.tracks);
}


AroResp FieldGeometryProcessor::getHeadlandParameters(const HeadlandParameters &plannerParameters, double &headlandWidth, double &trackWidth, size_t &numTracks)
{
    if(plannerParameters.headlandWidth <= 0){
        if(plannerParameters.numTracks <= 0 || plannerParameters.trackWidth < 1e-9){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid set of headland parameters");
            return AroResp(1, "Invalid set of headland parameters");
        }
        trackWidth = plannerParameters.trackWidth;
        numTracks = plannerParameters.numTracks;
        headlandWidth = trackWidth * numTracks;
    }
    else if(plannerParameters.numTracks > 0) {
        headlandWidth = plannerParameters.headlandWidth;
        numTracks = plannerParameters.numTracks;
        trackWidth = headlandWidth / numTracks;
    }
    else{
        headlandWidth = plannerParameters.headlandWidth;
        numTracks = 0;
        if(plannerParameters.trackWidth < 1e-9){
            trackWidth = headlandWidth;
        }
        else if(plannerParameters.trackWidth > 1e-9){
            trackWidth = plannerParameters.trackWidth;
            numTracks = headlandWidth / trackWidth;
            if(numTracks == 0){
                headlandWidth = trackWidth;
                numTracks = 1;
            }
            else if( headlandWidth - numTracks * trackWidth > 0.05*trackWidth){
                ++numTracks;
                headlandWidth = trackWidth * numTracks;
            }
        }
    }

    return AroResp::ok();

}

bool FieldGeometryProcessor::reorderSubfieldPoints(Subfield &sf, const Point *lastPoint){
    if(sf.boundary_outer.points.size() < 3)
        return false;

    arolib::geometry::openPolygon(sf.boundary_outer);
    size_t indSP = 0;
    double distSP = std::numeric_limits<double>::max();

    //Get the point in the outer boundary that is closest to a field access point
    for(size_t i = 0 ; i < sf.access_points.size() ; ++i){
        size_t ind = arolib::geometry::getPointIndInMinDist(sf.boundary_outer.points, sf.access_points.at(i));
        double dist = arolib::geometry::calc_dist(sf.boundary_outer.points.at(ind), sf.access_points.at(i));
        if(distSP > dist){
            indSP = ind;
            distSP = dist;
        }

    }

    //Get the point in the outer boundary that is closest to the given lastRoutePoint (iif the distance is shorter than the one for the closest field access point)
    if(lastPoint){
        size_t ind = arolib::geometry::getPointIndInMinDist(sf.boundary_outer.points, *lastPoint);
        double dist = arolib::geometry::calc_dist(sf.boundary_outer.points.at(ind), *lastPoint);
        if(distSP > dist){
            indSP = ind;
            distSP = dist;
        }
    }

    //reorder the outer boundary points so that the first point is the previously calculated closest point
    if(indSP != 0){
        std::vector<Point> pointsTmp;
        pointsTmp.insert( pointsTmp.end(), sf.boundary_outer.points.begin(), sf.boundary_outer.points.begin()+indSP );
        sf.boundary_outer.points.erase( sf.boundary_outer.points.begin(), sf.boundary_outer.points.begin()+indSP );
        sf.boundary_outer.points.insert( sf.boundary_outer.points.end(), pointsTmp.begin(), pointsTmp.end() );
    }
    arolib::geometry::closePolygon(sf.boundary_outer);

    return true;

}

Point FieldGeometryProcessor::getStartingPoint(const Subfield &sf, const std::vector<Track>& tracksIF)
{
    Point startingPoint = sf.boundary_outer.points.front();

    if(!tracksIF.empty()){//obtain the field access point closest to the first or last IF track

        double minDist = std::numeric_limits<double>::max();
        for(int i = 0 ; i < 2 ; ++i){
            const auto& track_points = ( i == 0 ? tracksIF.front().points : tracksIF.back().points );
            for(int j = 0 ; j < 2 ; ++j){
                const auto& pt = ( j == 0 ? track_points.front() : track_points.back() );
                for(auto &fap : sf.access_points){
                    double dist = arolib::geometry::calc_dist(pt, fap);
                    //update the overall minimum arrival time for all harvesters and the corresponding field access point
                    if(minDist > dist){
                        minDist = dist;
                        startingPoint = fap.point();
                    }
                }
            }
        }

        return startingPoint;
    }

    //obtain the field access point closest to the first boundary point
    double minDist = std::numeric_limits<double>::max();
    for(auto &fap : sf.access_points){
        double dist = arolib::geometry::calc_dist(sf.boundary_outer.points.front(), fap);
        //update the overall minimum arrival time for all harvesters and the corresponding field access point
        if(minDist > dist){
            minDist = dist;
            startingPoint = fap.point();
        }
    }


    return startingPoint;
}

bool FieldGeometryProcessor::processSubfieldSurroundingHeadland(Subfield& subfield,
                                                                double headlandWidth,
                                                                double trackWidth,
                                                                size_t numTracks,
                                                                Point startingPoint,
                                                                const HeadlandParameters& settings,
                                                                const std::vector<Track> &tracksIF) {

    if(subfield.boundary_outer.points.size() < 3)
        return false;

    geometry::openPolygon(subfield.boundary_outer);

    //get the index of the point in the outer boundary that is closest to the initial reference point and reorder the boundary points if needed
    size_t indSP = geometry::getPointIndInMinDist(subfield.boundary_outer.points, startingPoint);
    if(indSP != 0){
        std::vector<Point> pointsTmp;
        pointsTmp.insert( pointsTmp.end(), subfield.boundary_outer.points.begin(), subfield.boundary_outer.points.begin()+indSP );
        subfield.boundary_outer.points.erase( subfield.boundary_outer.points.begin(), subfield.boundary_outer.points.begin()+indSP );
        subfield.boundary_outer.points.insert( subfield.boundary_outer.points.end(), pointsTmp.begin(), pointsTmp.end() );
    }
    arolib::geometry::closePolygon(subfield.boundary_outer);

    //obtain the inner boundary from the outer boundary using the resulting headland width
    if(!geometry::offsetPolygon(subfield.boundary_outer,
                                subfield.boundary_inner,
                                headlandWidth,
                                false) ){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting the outer boundary to obtain the inner boundary");
        return false;
    }

    //obtain the planned headland (linestring between the outer and the inner boundaries) from the outer boundary using the resulting headland width / 2
    if(!geometry::offsetPolygon(subfield.boundary_outer,
                                subfield.headlands.complete.middle_track,
                                headlandWidth*0.5,
                                false) ){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting the outer boundary to obtain the headland");
        return false;
    }

    bool offsetRoutesFromOuterBoundary = true;
    int trackIdRef = Track::DeltaHLTrackId;//start at 'DeltaHLTrackId' to avoid problems with inner field tracks
    Polygon outer_tmp = subfield.boundary_outer, inner_tmp;

    for(size_t i = 0 ; i < numTracks ; ++i){
        Polygon headlandTrack_tmp;

        if(offsetRoutesFromOuterBoundary){//the new track is generated using the outer boundary polygone and the corresponding offset distance (sum of current and previous machine working widths)
            if ( !geometry::offsetPolygon(subfield.boundary_outer,
                                          headlandTrack_tmp,
                                          trackWidth * (0.5 + i),
                                          false) ){
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting the boundary to obtain track " + std::to_string(i) + ". Tracks are incomplete!");
                subfield.headlands.complete.tracks.pop_back();
                return false;
            }
        }
        else{//the new track is generated using the previously generated track and the corresponding offset distance (current machine working width)
            if ( !geometry::offsetPolygon(outer_tmp,
                                          headlandTrack_tmp,
                                          trackWidth * 0.5,
                                          false) ){
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting the boundary of track " + std::to_string(i-1) + " to obtain the next headland track (1st offset). Routes are incomplete!");
                subfield.headlands.complete.tracks.pop_back();
                return false;
            }

            if(i+1 < numTracks){//calculate the reference boundary to be used to generate the next track
                if ( !geometry::offsetPolygon(outer_tmp,
                                              inner_tmp,
                                              trackWidth,
                                              false) ){
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting the boundary of track " + std::to_string(i-1) + " to obtain the next headland track (2nd offset). Routes are incomplete!");
                    subfield.headlands.complete.tracks.pop_back();
                    return false;
                }
                outer_tmp = inner_tmp;
            }

        }


        geometry::unsample_polygon(headlandTrack_tmp, 0.1);//the sampling is done later
        subfield.headlands.complete.tracks.emplace_back(Track());
        auto& track = subfield.headlands.complete.tracks.back();
        track.type = Track::MAIN_HL;
        track.id = trackIdRef + i;
        track.width = trackWidth;
        track.points = headlandTrack_tmp.points;

    }

    if(!tracksIF.empty() && !subfield.headlands.complete.tracks.empty())
        sampleSurroundingTrackBasedOnIFTracks(subfield.headlands.complete.tracks.back(), tracksIF, settings.sampleResolution);

    if(settings.sampleResolution > 1e-9){
        if(settings.tracksSamplingStrategy == HeadlandParameters::TracksSamplingStrategy::SAMPLE_TRACKS_PERPENDICULARILY)
            sampleHeadlandTracksPerpendicularly(subfield.headlands.complete, settings.sampleResolution, headlandWidth);
        else if(settings.tracksSamplingStrategy == HeadlandParameters::TracksSamplingStrategy::SAMPLE_TRACKS_WITH_CLOSEST_SAMPLES)
            sampleHeadlandTracksWithClosestPoints(subfield.headlands.complete, settings.sampleResolution);
        else if(settings.tracksSamplingStrategy == HeadlandParameters::TracksSamplingStrategy::SIMPLE_TRACK_SAMPLING){
            for(auto& track :subfield.headlands.complete.tracks)
                track.points = arolib::geometry::sample_geometry( track.points, settings.sampleResolution, 0.15*settings.sampleResolution);
        }
    }

    subfield.headlands.complete.headlandWidth = headlandWidth;
    subfield.headlands.complete.boundaries.first = subfield.boundary_outer;
    subfield.headlands.complete.boundaries.second = subfield.boundary_inner;

    return true;
}

void FieldGeometryProcessor::sampleSurroundingTrackBasedOnIFTracks(Track &trackHL, const std::vector<Track> &tracksIF, double sampleResHL)
{
    for(auto& track : tracksIF){
        double res = (sampleResHL > 1e-9 ? sampleResHL : track.width);
        if(track.points.size() < 2)
            continue;
        for(int side = 0 ; side < 2 ; side++){
            Point p0 = (side == 0 ? track.points.front() : track.points.back());
            Point p1 = (side == 0 ? track.points.at(1) : r_at(track.points, 1));
            auto intersections = geometry::get_intersection(p0, p1, trackHL.points, true, true, false);
            for(auto& pt : intersections)
                geometry::addSampleToGeometryClosestToPoint(trackHL.points, pt, 1, 0.25 * res);
            auto indNew = geometry::addSampleToGeometryClosestToPoint(trackHL.points, p0, 1, 0.15 * res);
        }
    }

    for(int side = 0 ; side < 2 ; side++){
        for(size_t i = 0 ; i < tracksIF.size() ; ++i){
            size_t trackInd = ( side == 0 ? i : tracksIF.size()-1-i );
            size_t prevTrackInd = ( side == 0 ? i-1 : tracksIF.size()-i );
            double res = (sampleResHL > 1e-9 ? sampleResHL : tracksIF.at(trackInd).width);
            for(size_t j = 1 ; j+1 < tracksIF.at(trackInd).points.size() ; ++j){
                const auto& pt = tracksIF.at(trackInd).points.at(j);
                if(i == 0){
                    geometry::addSampleToGeometryClosestToPoint(trackHL.points, pt, -1, 0.15 * res);
                    continue;
                }
                auto distPrev = geometry::calc_dist_to_linestring(tracksIF.at(prevTrackInd).points, pt);
                auto distHL = geometry::calc_dist_to_linestring(trackHL.points, pt);
                if(distPrev > distHL)
                    geometry::addSampleToGeometryClosestToPoint(trackHL.points, pt, -1, 0.4 * res);
            }
        }
    }
}


void FieldGeometryProcessor::sampleHeadlandTracksPerpendicularly(CompleteHeadland &headland, double resolution, double headlandWidth)
{
    if(headland.tracks.empty())
        return;

    const double minDeltaSample = 0.3*resolution;
    const double minDeltaPerpSample = 0.15*resolution;

    if(headland.tracks.size() == 1){//sample normally
        //geometry::unsample_linestring(headland.tracks.front().points, 0.001, m_unsampleAngTol);
        headland.tracks.front().points = geometry::sample_geometry(headland.tracks.front().points, resolution, minDeltaSample, false, true);
        return;
    }

    double multAngle = 1;//multiplier used to obtain the rotation angle (for the perpendicular vectors) depending on the polygon direction
    Polygon trackPoly;
    trackPoly.points = headland.tracks.back().points;

    if(!geometry::isPolygonClockwise(trackPoly, true))
        multAngle = -1;


    //geometry::unsample_linestring(headland.tracks.back().points, 0.001, m_unsampleAngTol);

    for(int i = headland.tracks.size()-2 ; i >= 0 ; --i){
        Track &track = headland.tracks.at(i);
        Track &trackRef = headland.tracks.at(i+1);

        geometry::unsample_linestring(track.points, 0.001, m_unsampleAngTol);

        trackRef.points = arolib::geometry::sample_geometry(trackRef.points, resolution, minDeltaSample, false, true);
        std::vector< std::pair<Point, Point> > perpLines;
        std::vector< Point > refPoints;

        if(trackRef.points.size() > 3){//get the perpendicular points iif
            {
                auto& p0 = trackRef.points.front();
                auto& p1 = trackRef.points.at(1);
                auto& p2 = *( trackRef.points.end()-2 );

                //get the angle between the 2 (consecutive) segments to which p0 belongs (p0 being the connection between the 2 segments)
                double ang1 = ( multAngle>0 ?
                                    arolib::geometry::get_angle(p0, p1, p0, p2, true) :
                                    arolib::geometry::get_angle(p0, p2, p0, p1, true) );
                if( ang1 < 0 || ang1 >= 180){
                    Point pp = arolib::geometry::rotate(p0, p1, M_PI_2 * multAngle);
                    perpLines.push_back( std::make_pair(p0, pp) );
                }
                else if(ang1 > 175)
                    refPoints.emplace_back(p0);
            }
            for(size_t ii = 1 ; ii+1 < trackRef.points.size() ; ++ii){
                auto& p0 = trackRef.points.at(ii);
                auto& p1 = trackRef.points.at(ii+1);
                auto& p2 = trackRef.points.at(ii-1);

                //get the angle between the 2 (consecutive) segments to which p0 belongs (p0 being the connection between the 2 segments)
                double ang1 = ( multAngle>0 ?
                                    arolib::geometry::get_angle(p0, p1, p0, p2, true) :
                                    arolib::geometry::get_angle(p0, p2, p0, p1, true) );
                if( ang1 > 0 && ang1 < 180){
                    if(ang1 > 175)
                        refPoints.emplace_back(p0);
                    continue;
                }
                if( std::fabs( arolib::geometry::get_angle(p0, p1, p0, p2, true) ) < 170){
                    Point pp0 = arolib::geometry::rotate(p0, p2, M_PI_2 * -multAngle);
                    perpLines.push_back( std::make_pair(p0, pp0) );
                }
                Point pp = arolib::geometry::rotate(p0, p1, M_PI_2 * multAngle);
                perpLines.push_back( std::make_pair(p0, pp) );
            }
            {//add last ref perp point
                auto& p0 = trackRef.points.front();
                auto& p1 = trackRef.points.at(1);
                auto& p2 = *( trackRef.points.end()-2 );
                double ang1 = ( multAngle>0 ?
                                    arolib::geometry::get_angle(p0, p1, p0, p2, true) :
                                    arolib::geometry::get_angle(p0, p2, p0, p1, true) );
                if( ang1 < 0 || ang1 >= 180-1e-3){
                    if( std::fabs( arolib::geometry::get_angle(p0, p1, p0, p2, true) ) < 170){
                        Point pp0 = arolib::geometry::rotate(p0, p2, M_PI_2 * -multAngle);
                        perpLines.push_back( std::make_pair(p0, pp0) );
                    }
                }
            }
        }

        size_t refIndex = 0;
        bool leaveFirstPoint = false;
        for(int j = -1 ; j+1 < track.points.size() && refIndex < perpLines.size(); ++j){

            const auto& p0 = ( j < 0 ? r_at(track.points, 1) : track.points.at(j) );
            const auto& p1 = track.points.at(j+1);

            auto& p0_ref = perpLines.at(refIndex).first;
            auto& p1_ref = perpLines.at(refIndex).second;
            Point sample;

            if( arolib::geometry::get_intersection( p0_ref, p1_ref, p0, p1, sample, true )
                    && std::fabs( arolib::geometry::get_angle(p0_ref, p1_ref, p0_ref, sample) ) < M_PI_2
                    && arolib::geometry::calc_dist(p0_ref, sample) < 0.5*headlandWidth ){

                bool distOK0 = ( arolib::geometry::calc_dist(sample, p0) > minDeltaPerpSample );
                bool distOK1 = ( arolib::geometry::calc_dist(sample, p1) > minDeltaPerpSample );

                if( distOK0 && distOK1 ){

                    const Point& p3 = ( j+2 < track.points.size() ? track.points.at(j+2) : track.points.at(1) );

                    if( arolib::geometry::calc_dist(sample, p1) < resolution
                            && arolib::geometry::is_line(std::vector<Point>{sample, p1, p3}, 0.1) ){
                        if(j+2 == track.points.size()){
                            if(!leaveFirstPoint){
                                track.points.front() = track.points.back() = sample;
                                --j;
                            }
                            else
                                track.points.insert( track.points.begin()+j+1, sample );
                        }
                        else{
                            track.points.at(j+1) = sample;
                            --j;
                        }
                    }
                    else{
                        if(j < 0){
                            track.points.insert( track.points.end()-1, sample );
                            --j;
                        }
                        else
                            track.points.insert( track.points.begin()+j+1, sample );
                    }
                }
                else{
                    if( (j == 0 && !distOK0) || (j == -1 && !distOK1) )
                        leaveFirstPoint = true;
                    --j;
                }
                ++refIndex;
            }
        }

        for(auto& p :  refPoints)
            arolib::geometry::addSampleToGeometryClosestToPoint(track.points, p, 1, minDeltaPerpSample);
    }

    headland.tracks.front().points = geometry::sample_geometry(headland.tracks.front().points, resolution, minDeltaSample);
}

void FieldGeometryProcessor::sampleHeadlandTracksWithClosestPoints(CompleteHeadland &headland, double resolution)
{
    if(headland.tracks.empty())
        return;

    const double minDeltaSample = 0.15*resolution;
    const double minDeltaPerpSample = 0.1*resolution;

    if(headland.tracks.size() == 1){//sample normally
        headland.tracks.front().points = geometry::sample_geometry(headland.tracks.front().points, resolution, minDeltaSample);
        return;
    }

    double multAngle = 1;
    Polygon trackPoly;
    trackPoly.points = headland.tracks.back().points;

    if(!geometry::isPolygonClockwise(trackPoly, true))
        multAngle = -1;

    for(int i = headland.tracks.size()-2 ; i >= 0 ; --i){
        headland.tracks.at(i+1).points = arolib::geometry::sample_geometry(headland.tracks.at(i+1).points, resolution, minDeltaSample);
        for(size_t j = 0 ; j+1 < headland.tracks.at(i+1).points.size() ; ++j)
            geometry::addSampleToGeometryClosestToPoint(headland.tracks.at(i).points, headland.tracks.at(i+1).points.at(j), 0, minDeltaPerpSample);
    }
    headland.tracks.front().points = geometry::sample_geometry(headland.tracks.front().points, resolution, minDeltaSample);

}

void FieldGeometryProcessor::samplePartialHLTracksBasedOnIFTracks(Subfield &subfield, double resolution)
{
    for(auto& hl : subfield.headlands.partial){
        if(!hl.tracks.empty())
            geometry::unsample_linestring(hl.tracks.back().points, 0.1 * hl.tracks.back().width, m_unsampleAngTol);
    }

    for(auto& iftrack : subfield.tracks){
        if(iftrack.points.empty())
            continue;
        for(int side = 0 ; side < 2 ; side++){
            const Point& pt = ( side == 0 ? iftrack.points.front() : iftrack.points.back() );
            for(auto& hl : subfield.headlands.partial){
                if(!hl.isConnectingHeadland())
                    continue;
                if(hl.tracks.empty())
                    continue;
                auto& hltrack = hl.tracks.back();
                if( geometry::calc_dist_to_linestring(hltrack.boundary.points, pt, false) < 0.5 * hltrack.width ){
                    geometry::addSampleToGeometryClosestToPoint(hltrack.points, pt, 1, 0.2 * resolution);
                    geometry::addSampleToGeometryClosestToPoint(hltrack.points, pt, 0, 0.5 * resolution);
                    if(iftrack.points.size() > 2){
                        const auto& p1 = ( side == 0 ? iftrack.points.at(1) : r_at(iftrack.points, 1) );
                        auto intersections = geometry::get_intersection(pt, p1, hltrack.points, true, true, true);
                        if(!intersections.empty())
                            geometry::addSampleToGeometryClosestToPoint(hltrack.points, intersections.front(), 1,  0.5 * resolution);
                    }
                }

            }
        }
    }

    auto extremaInds = geometry::getInfieldExtremaTracksIndexes(subfield);
    std::set<size_t> extremaIndsPlus;
    for(auto& trackind : extremaInds){
        extremaIndsPlus.insert(trackind);
        if(trackind > 0)
            extremaIndsPlus.insert(trackind - 1);
        if(trackind+1 < subfield.tracks.size())
            extremaIndsPlus.insert(trackind + 1);
    }

    for(auto& trackind : extremaIndsPlus){
        const auto& iftrack = subfield.tracks.at(trackind);
        for(size_t i = 0 ; i < iftrack.points.size() ; ++i){
            const Point& pt = iftrack.points.at(i);
            for(auto& hl : subfield.headlands.partial){
                if(hl.tracks.empty())
                    continue;
                if(!hl.isConnectingHeadland() && (i == 0 || i+1 == iftrack.points.size()))
                    continue;

                auto& hltrack = hl.tracks.back();
                if( geometry::calc_dist_to_linestring(hltrack.boundary.points, pt, false) > 0.75 * iftrack.width )
                    continue;

                bool addClosest = true;
                if(hl.isConnectingHeadland() && iftrack.points.size() > 1){
                    Point pRef1 = geometry::rotate(pt, ( i == 0 ? iftrack.points.at(i+1) : r_at(iftrack.points, 1) ), M_PI_2);
                    pRef1 = geometry::getPointInLineAtDist(pt, pRef1, 1.5 * iftrack.width );
                    Point pRef2 = geometry::getPointInLineAtDist(pt, pRef1, - 1.5 * iftrack.width );
                    auto intersections = geometry::get_intersection(pRef1, pRef2, hltrack.points, false, false, false);
                    for(auto& pInt : intersections)
                        addClosest &= ( geometry::addSampleToGeometryClosestToPoint(hltrack.points, pInt, 1, 0.25 * resolution) > 0 );
                }

                if(addClosest)
                    geometry::addSampleToGeometryClosestToPoint(hltrack.points, pt, 1, 0.5 * resolution);
            }
        }

    }

    for(auto& hl : subfield.headlands.partial){
        if(hl.tracks.empty())
            continue;
        hl.tracks.back().points = geometry::sample_geometry(hl.tracks.back().points, resolution, 0.5 * resolution);
        for(size_t i = 1 ; i < hl.tracks.size() ; ++i){
            Track& track = r_at(hl.tracks, i);
            Track& track2 = r_at(hl.tracks, i-1);
            geometry::unsample_linestring(track.points, 0.1 * track.width, m_unsampleAngTol);
            for(auto& pt : track2.points)
                geometry::addSampleToGeometryClosestToPoint(track.points, pt, 0, 0.5 * resolution);
            track.points = geometry::sample_geometry(track.points, resolution, 0.5 * resolution);
        }
    }
}

AroResp FieldGeometryProcessor::adjustIFTracksForSideHLGeneration(Polygon &boundary, std::vector<Track> &tracksIF, double headlandWidth, const HeadlandParameters& hlParams){

    Polygon boundary2;
    if(hlParams.trackWidth > 1e-9 && !geometry::offsetPolygon(boundary, boundary2, hlParams.trackWidth, false))
        boundary2.points.clear();
    for(int tracks_side = 1 ; tracks_side <= 2 ; ++tracks_side){
        if(tracksIF.empty())
            break;
        auto & track = ( tracks_side == 1 ? tracksIF.front().points : tracksIF.back().points );
        arolib::geometry::unsample_linestring(track);

        bool removeTrack = track.size() < 2 ;

        if(!removeTrack){
            if(hlParams.trackWidth > 1e-9)
               removeTrack =  arolib::geometry::getGeometryLength(track) <= headlandWidth + hlParams.trackWidth;
            else
                removeTrack =  arolib::geometry::getGeometryLength(track) <= 2*headlandWidth;
        }

        if(!removeTrack && boundary2.points.size() > 3){
            std::vector<Point> intersections = geometry::get_intersection(track, boundary2.points);
            removeTrack = intersections.size() < 2;
        }

        if(removeTrack){
            if(tracks_side == 1)
                pop_front(tracksIF);
            else
                tracksIF.pop_back();
            --tracks_side;
            continue;
        }

        std::vector<Point> intersections = geometry::get_intersection(track, boundary.points);
        std::vector<int> intersectionIdxs{0};
        for(size_t j = 0 ; j < intersections.size() ; ++j){
            if(geometry::calc_dist(intersections.at(j), track.front()) < 1e-5
                    || geometry::calc_dist(intersections.at(j), track.back()) < 1e-5 )
                continue;
            int ind = geometry::addSampleToGeometryClosestToPoint(track, intersections.at(j), 1);
            for (size_t k = 0 ; k < intersectionIdxs.size() ; k++){
                auto& idx = r_at(intersectionIdxs, k);
                if(idx >= ind)
                    ++idx;
                else{
                    intersectionIdxs.insert(intersectionIdxs.begin()+(intersectionIdxs.size()-k), ind);
                    break;
                }
            }
        }
        intersectionIdxs.push_back(track.size()-1);

        auto isMiddlePointInPoly = [&boundary, &track, &intersectionIdxs](size_t j, int deltaInd)->bool{
            int ind0 = deltaInd + intersectionIdxs.at(j);
            int ind1 = deltaInd + intersectionIdxs.at(j+1);
            if(ind0 > ind1)
                std::swap(ind0, ind1);
            std::vector<Point> segment(track.begin()+ind0, track.begin()+ind1+1);
            return geometry::in_polygon( geometry::getPointAtHalfLength(segment).first, boundary );
        };

        int deltaInd = 0;
        while(track.size() > 1){
            int indNextIntersection = -1;
            for(size_t j = 0 ; j+1 < intersectionIdxs.size() ; ++j){
                if(!isMiddlePointInPoly(j, -deltaInd)){indNextIntersection = j;
                    break;
                }
            }
            if(indNextIntersection < 0)
                break;

            int ind0 = deltaInd + intersectionIdxs.at(indNextIntersection);
            int ind1 = deltaInd + intersectionIdxs.at(indNextIntersection+1);
            if(ind0 == 0){//first segment is outside the boundary --> remove first track point (i.e. first segment)
                deltaInd = ind1;
                pop_front(track, 1);
                intersectionIdxs.erase(intersectionIdxs.begin(), intersectionIdxs.begin()+indNextIntersection+1);
                continue;
            }
            if(geometry::getGeometryLength(track, 0, ind0) <= headlandWidth ){//remove points corresponding to the short segment inside the boundary and the segment outside the boundary
                deltaInd = ind1;
                pop_front(track, ind1 - ind0 + 1);
                intersectionIdxs.erase(intersectionIdxs.begin(), intersectionIdxs.begin()+indNextIntersection+1);
                continue;
            }
            break;
        }
        while(track.size() > 1){
            int indNextIntersection = -1;
            for(size_t j = intersectionIdxs.size()-1 ; j > 0 ; --j){
                if(!isMiddlePointInPoly(j-1, -deltaInd)){
                    indNextIntersection = j;
                    break;
                }
            }
            if(indNextIntersection < 0)
                break;

            int ind0 = deltaInd + intersectionIdxs.at(indNextIntersection);
            int ind1 = deltaInd + intersectionIdxs.at(indNextIntersection-1);
            if(ind0+1 == track.size()){//last segment is outside the boundary --> remove last track point (i.e. last segment)
                track.pop_back();
                intersectionIdxs.erase(intersectionIdxs.begin()+indNextIntersection, intersectionIdxs.end());
                continue;
            }
            if(geometry::getGeometryLength(track, ind0) <= headlandWidth ){//remove points corresponding to the short segment inside the boundary and the segment outside the boundary
                pop_back(track, ind0-ind1+1);
                intersectionIdxs.erase(intersectionIdxs.begin()+indNextIntersection, intersectionIdxs.end());
                continue;
            }
            break;
        }

        if(track.size() < 2){
            if(tracks_side == 1)
                pop_front(tracksIF);
            else
                tracksIF.pop_back();
            --tracks_side;
            continue;
        }
    }
    if(tracksIF.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating temporary IF tracks: no tracks left");
        return AroResp(1, "Error generating temporary IF tracks: no tracks left");
    }
    return AroResp::ok();
}

AroResp FieldGeometryProcessor::createPartialHLInitialReferenceLines(Polygon &boundary,
                                                                     std::vector<Track> &tracksIF,
                                                                     double headlandWidth,
                                                                     double trackDistanceIF,
                                                                     const HeadlandParameters &plannerParameters,
                                                                     std::vector<Point> &hl_points_1,
                                                                     std::vector<Point> &hl_points_2,
                                                                     size_t maxRemoveTracks,
                                                                     std::pair<size_t, size_t> countRemovedTracks){

    hl_points_1.clear();
    hl_points_2.clear();

    auto boundary_us = boundary;
    geometry::unsample_linestring(boundary_us.points, 0.001, deg2rad(0.3));

    if(tracksIF.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating headland reference points: no IF tracks left");
        return AroResp(1, "Error generating headland reference points: no IF tracks left");
    }

    if(tracksIF.size() == 1){//weird case
        Point pRef = arolib::geometry::getPointInLineAtDist( tracksIF.front().points.front(), tracksIF.front().points.at(1), trackDistanceIF );
        hl_points_1.emplace_back(tracksIF.front().points.front());
        hl_points_1.front() = arolib::geometry::rotate(tracksIF.front().points.front(), pRef, M_PI_2);
        hl_points_1.push_back( arolib::geometry::rotate(tracksIF.front().points.front(), pRef, -M_PI_2) );

        pRef = arolib::geometry::getPointInLineAtDist( tracksIF.front().points.back(), r_at(tracksIF.front().points, 1), trackDistanceIF );
        hl_points_2.emplace_back(tracksIF.front().points.back());
        hl_points_2.front() = arolib::geometry::rotate(tracksIF.front().points.back(), pRef, M_PI_2);
        hl_points_2.push_back( arolib::geometry::rotate(tracksIF.front().points.back(), pRef, -M_PI_2) );
    }
    else{
        geometry::addSampleToGeometryClosestToPoint(boundary.points, tracksIF.front().points.front());
        geometry::addSampleToGeometryClosestToPoint(boundary.points, tracksIF.front().points.back());
        geometry::addSampleToGeometryClosestToPoint(boundary.points, tracksIF.back().points.front());
        geometry::addSampleToGeometryClosestToPoint(boundary.points, tracksIF.back().points.back());

        geometry::addSampleToGeometryClosestToPoint(boundary_us.points, tracksIF.front().points.front());
        geometry::addSampleToGeometryClosestToPoint(boundary_us.points, tracksIF.front().points.back());
        geometry::addSampleToGeometryClosestToPoint(boundary_us.points, tracksIF.back().points.front());
        geometry::addSampleToGeometryClosestToPoint(boundary_us.points, tracksIF.back().points.back());

        Point controlPt1,  controlPt2;
        if(tracksIF.size() > 2){
            controlPt1 = tracksIF.at( (int)(0.5 * tracksIF.size()) ).points.front();
            controlPt2 = tracksIF.at( (int)(0.5 * tracksIF.size()) ).points.back();
        }
        else{
            controlPt1 = geometry::getCentroid(tracksIF.front().points.front(), tracksIF.back().points.front());
            controlPt2 = geometry::getCentroid(tracksIF.front().points.back(), tracksIF.back().points.back());
        }

        auto path1 = geometry::getShortestGeometryPart(boundary_us.points, tracksIF.front().points.front(), tracksIF.back().points.front());
        auto path2 = geometry::getLongestGeometryPart(boundary_us.points, tracksIF.front().points.front(), tracksIF.back().points.front());
        if(geometry::calc_dist_to_linestring(path1, controlPt1, false) < geometry::calc_dist_to_linestring(path2, controlPt1, false))
            hl_points_1 = path1;
        else
            hl_points_1 = path2;

        path1 = geometry::getShortestGeometryPart(boundary_us.points, tracksIF.front().points.back(), tracksIF.back().points.back());
        path2 = geometry::getLongestGeometryPart(boundary_us.points, tracksIF.front().points.back(), tracksIF.back().points.back());
        if(geometry::calc_dist_to_linestring(path1, controlPt2, false) < geometry::calc_dist_to_linestring(path2, controlPt2, false))
            hl_points_2 = path1;
        else
            hl_points_2 = path2;

        //reorganize hl_points so that the first point corresponds to the first track
        if( geometry::calc_dist(hl_points_1.front(), tracksIF.front().points.front()) > geometry::calc_dist(hl_points_1.back(), tracksIF.front().points.front()) )
            std::reverse(hl_points_1.begin(), hl_points_1.end());
        if( geometry::calc_dist(hl_points_2.front(), tracksIF.front().points.back()) > geometry::calc_dist(hl_points_2.back(), tracksIF.front().points.back()) )
            std::reverse(hl_points_2.begin(), hl_points_2.end());

        double angTreshold_deg = plannerParameters.sideHLAngThresholdBoundIFTracks > -1e-9 ? plannerParameters.sideHLAngThresholdBoundIFTracks : 45; //70; //45; //12;

        //remove the points that would generate a headland parallel to the 1st and last tracks and adjust the hl reference points
        for(int hl_side = 0 ; hl_side < 2 ; ++hl_side){
            auto& hl_points = ( hl_side == 0 ? hl_points_1 : hl_points_2 );
            for(int line_side = 0 ; line_side < 2 ; ++line_side){
                Point p0_tr, p1_tr;
                if(hl_side == 0){
                    if(line_side == 0){
                        p0_tr = tracksIF.front().points.front();
                        p1_tr = tracksIF.front().points.at(1);
                    }
                    else{
                        p0_tr = tracksIF.back().points.front();
                        p1_tr = tracksIF.back().points.at(1);
                    }
                }
                else{
                    if(line_side == 0){
                        p0_tr = tracksIF.front().points.back();
                        p1_tr = r_at( tracksIF.front().points, 1 );
                    }
                    else{
                        p0_tr = tracksIF.back().points.back();
                        p1_tr = r_at( tracksIF.back().points, 1 );
                    }
                }

                Point p0_hl = ( line_side == 0 ? hl_points.at(1) : r_at(hl_points, 1) );
                Point p1_hl = ( line_side == 0 ? hl_points.front() : hl_points.back() );

                double ang_deg = geometry::get_angle(p0_tr, p1_tr, p0_hl, p1_hl, true);

                if( std::fabs(ang_deg) < angTreshold_deg ){
                    if(line_side == 0 && (!plannerParameters.limitIFTracksNotOverMainHL || countRemovedTracks.first < maxRemoveTracks)){
                        pop_front(tracksIF);
                        return createPartialHLInitialReferenceLines(boundary, tracksIF, headlandWidth, trackDistanceIF,
                                                                    plannerParameters, hl_points_1, hl_points_2,
                                                                    maxRemoveTracks, std::make_pair(countRemovedTracks.first+1, countRemovedTracks.second));
                    }
                    else if(line_side == 1 && (!plannerParameters.limitIFTracksNotOverMainHL || countRemovedTracks.second < maxRemoveTracks)){
                        tracksIF.pop_back();
                        return createPartialHLInitialReferenceLines(boundary, tracksIF, headlandWidth, trackDistanceIF,
                                                                    plannerParameters, hl_points_1, hl_points_2,
                                                                    maxRemoveTracks, std::make_pair(countRemovedTracks.first, countRemovedTracks.second+1));
                    }
                }
            }
        }

        auto sideHLGenerationStrategy = plannerParameters.sideHLGenerationStrategy;
        if(plannerParameters.sideHLGenerationStrategy == HeadlandParameters::SIDEHL_STRATEGY_DEF)
            sideHLGenerationStrategy = HeadlandParameters::SIDEHL_STRATEGY_1;

        if(sideHLGenerationStrategy == HeadlandParameters::SIDEHL_STRATEGY_1){
            //double extensionDist = headlandWidth;
            double extensionDist = geometry::getGeometryLength(boundary.points);
            arolib::geometry::extend_linestring( hl_points_1, extensionDist, extensionDist, true );
            arolib::geometry::extend_linestring( hl_points_2, extensionDist, extensionDist, true );
        }
        else if(sideHLGenerationStrategy == HeadlandParameters::SIDEHL_STRATEGY_2){
            auto getExtensionPoint = [&](const Point &hl_extrema, const Point& trackPt0, const Point& trackPt1, std::vector<Point> adjTrack)->Point{
                Point pRef1 = geometry::getPointInLineAtDist( trackPt0, geometry::rotate(trackPt0, trackPt1, M_PI_2), trackDistanceIF );
                Point pRef2 = geometry::getPointInLineAtDist( trackPt0, geometry::rotate(trackPt0, trackPt1, -M_PI_2), trackDistanceIF );

                if( geometry::calc_dist_to_linestring(adjTrack, pRef1) > geometry::calc_dist_to_linestring(adjTrack, pRef2) )
                    std::swap(pRef1, pRef2);
                pRef1 = trackPt0 - pRef1;

                //double extensionDist = headlandWidth;
                double extensionDist = geometry::getGeometryLength(boundary.points);
                geometry::setVectorLength(pRef1, extensionDist);
                return hl_extrema + pRef1;
            };

            push_front( hl_points_1, getExtensionPoint( hl_points_1.front(), tracksIF.front().points.front(), tracksIF.front().points.at(1),    tracksIF.at(1).points ) );
            hl_points_1.push_back( getExtensionPoint( hl_points_1.back(),    tracksIF.back().points.front(),  tracksIF.back().points.at(1),     r_at(tracksIF, 1).points ) );
            push_front( hl_points_2, getExtensionPoint( hl_points_2.front(), tracksIF.front().points.back(),  r_at(tracksIF.front().points, 1), tracksIF.at(1).points ) );
            hl_points_2.push_back( getExtensionPoint( hl_points_2.back(),    tracksIF.back().points.back(),   r_at(tracksIF.back().points, 1), r_at(tracksIF, 1).points ) );

        }
        else{
            auto getExtensionPoint = [&](const Point& pRef, std::vector<Point> prevTrack)->Point{
                Point pExt = pRef;
                arolib::geometry::extend_linestring( prevTrack, 1e6, 1e6 );
                int ind = arolib::geometry::addSampleToGeometryClosestToPoint(prevTrack, pRef, 1);
                if(ind >= 0 & ind < prevTrack.size())
                    pExt = arolib::geometry::extend_line( prevTrack.at(ind), pRef, headlandWidth );
                return pExt;
            };
            push_front( hl_points_1, getExtensionPoint( tracksIF.front().points.front(), tracksIF.at(1).points ) );
            hl_points_1.push_back( getExtensionPoint( tracksIF.back().points.front(), r_at(tracksIF, 1).points ) );
            push_front( hl_points_2, getExtensionPoint( tracksIF.front().points.back(), tracksIF.at(1).points ) );
            hl_points_2.push_back( getExtensionPoint( tracksIF.back().points.back(), r_at(tracksIF, 1).points ) );
        }

    }
    return AroResp::ok();
}

bool FieldGeometryProcessor::combineOverlappingHeadlands(const Subfield &subfield,
                                                         std::vector<Point> hl_points_1,
                                                         std::vector<Point> hl_points_2,
                                                         const Polygon &hl1, const Polygon &hl2,
                                                         Polygon &intersectionPoly,
                                                         std::vector<Track> if_tracks,
                                                         double headlandWidth,
                                                         Polygon &combinedHL, std::vector<Point> &hl_points_combined)
{
    Polygon boundary_tmp = subfield.boundary_outer;
    hl_points_combined.clear();

    int hasExtension1_front = geometry::calc_dist_to_linestring(subfield.boundary_outer.points, hl_points_1.front()) > 0.01;
    int hasExtension1_back = geometry::calc_dist_to_linestring(subfield.boundary_outer.points, hl_points_1.back()) > 0.01;
    int hasExtension2_front = geometry::calc_dist_to_linestring(subfield.boundary_outer.points, hl_points_2.front()) > 0.01;
    int hasExtension2_back = geometry::calc_dist_to_linestring(subfield.boundary_outer.points, hl_points_2.back()) > 0.01;

    auto getRefPoint = [&intersectionPoly](bool hasExtension, const Point& p0, const Point& p1) -> Point{
        if(!hasExtension)
            return p0;
        auto ind = geometry::addSampleToGeometryClosestToPoint(intersectionPoly.points, p1, 1);
        if(ind < 0)
            return p0;
        Point pTmp = intersectionPoly.points.at(ind);
        PointVec seg = {p0, p1};
        ind = geometry::addSampleToGeometryClosestToPoint(seg, pTmp, 1);
        if(ind < 0)
            return p0;
        return seg.at(ind);
    };

    Point refPoint1_front = getRefPoint(hasExtension1_front, hl_points_1.front(), hl_points_1.at(1));
    Point refPoint1_back = getRefPoint(hasExtension1_back, hl_points_1.back(), r_at(hl_points_1, 1));
    Point refPoint2_front = getRefPoint(hasExtension2_front, hl_points_2.front(), hl_points_2.at(1));
    Point refPoint2_back = getRefPoint(hasExtension2_back, hl_points_2.back(), r_at(hl_points_2, 1));

    //check in which side of the HL_points is the overlap -> reverse hl_points so that the first points do not correspond to the overlapped side
    if( geometry::calc_dist_to_linestring(intersectionPoly.points, refPoint1_front)
            < geometry::calc_dist_to_linestring(intersectionPoly.points, refPoint1_back) ){
        std::reverse(hl_points_1.begin(), hl_points_1.end());
        std::swap(refPoint1_front, refPoint1_back);
    }
    if( geometry::calc_dist_to_linestring(intersectionPoly.points, refPoint2_front)
            < geometry::calc_dist_to_linestring(intersectionPoly.points, refPoint2_back) ){
        std::reverse(hl_points_2.begin(), hl_points_2.end());
        std::swap(refPoint2_front, refPoint2_back);
    }

    if( geometry::calc_dist_to_linestring(intersectionPoly.points, if_tracks.front().points.front())
            + geometry::calc_dist_to_linestring(intersectionPoly.points, if_tracks.front().points.back())
            <
            geometry::calc_dist_to_linestring(intersectionPoly.points, if_tracks.back().points.front())
            + geometry::calc_dist_to_linestring(intersectionPoly.points, if_tracks.back().points.back()) )
    {
        std::reverse(if_tracks.begin(), if_tracks.end());
    }

    geometry::addSampleToGeometryClosestToPoint(boundary_tmp.points, if_tracks.front().points.front());
    geometry::addSampleToGeometryClosestToPoint(boundary_tmp.points, if_tracks.front().points.back());

    auto conn_1 = geometry::getShortestGeometryPart(subfield.boundary_outer.points,
                                                    if_tracks.front().points.front(),
                                                    if_tracks.front().points.back());
    auto conn_2 = geometry::getLongestGeometryPart(subfield.boundary_outer.points,
                                                   if_tracks.front().points.front(),
                                                   if_tracks.front().points.back());

    if( geometry::calc_dist_to_linestring(conn_1, if_tracks.back().points.front())
            > geometry::calc_dist_to_linestring(conn_2, if_tracks.back().points.front()) )
        std::swap(conn_1, conn_2);

    Polygon hl_tmp;
    if(!geometry::offsetLinestring(conn_1, hl_tmp, headlandWidth, headlandWidth, false)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating initial HL boundary");
        return false;
    }

    auto intersections = geometry::get_intersection(subfield.boundary_outer, hl_tmp);
    if(intersections.size() != 1){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error intersecting the initial HL boundary with the subfield boundary");
        return false;
    }

    combinedHL = intersections.front();
    hl_points_combined.clear();

    std::vector<PolygonWithHoles> unionPolys;
    geometry::get_union(hl1, hl2, unionPolys);
    if(unionPolys.size() != 1){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error obtaining union of headlands");
        return false;
    }
    auto hl12 = unionPolys.front().outer;
    geometry::get_union(hl12, combinedHL, unionPolys);
    if(unionPolys.size() != 1){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error obtaining union of headlands (2)");
        return false;
    }
    combinedHL = unionPolys.front().outer;

    if(hl_points_1.size() < 4 - hasExtension1_front - hasExtension1_back ||
            hl_points_2.size() < 4 - hasExtension2_front - hasExtension2_back){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The hl reference lines have too few points");
        return false;
    }

    if(hasExtension1_front)
        hl_points_combined.emplace_back( hl_points_1.front() );


    geometry::addSampleToGeometryClosestToPoint(boundary_tmp.points, hl_points_1.at(hasExtension1_front));
    geometry::addSampleToGeometryClosestToPoint(boundary_tmp.points, hl_points_2.at(hasExtension2_front));

    geometry::addSampleToGeometryClosestToPoint(boundary_tmp.points, refPoint1_front);
    geometry::addSampleToGeometryClosestToPoint(boundary_tmp.points, refPoint2_front);

    conn_1 = geometry::getShortestGeometryPart(boundary_tmp.points,
                                               hl_points_1.at(hasExtension1_front),
                                               hl_points_2.at(hasExtension2_front));
    conn_2 = geometry::getLongestGeometryPart(boundary_tmp.points,
                                              hl_points_1.at(hasExtension1_front),
                                              hl_points_2.at(hasExtension2_front));

    if( geometry::calc_dist_to_linestring(conn_1, refPoint1_back)
            < geometry::calc_dist_to_linestring(conn_2, refPoint2_back) )
        hl_points_combined.insert(hl_points_combined.end(), conn_1.begin(), conn_1.end());
    else
        hl_points_combined.insert(hl_points_combined.end(), conn_2.begin(), conn_2.end());


    if(hasExtension2_front)
        hl_points_combined.emplace_back( hl_points_2.front() );

    return true;
}

AroResp FieldGeometryProcessor::generatePartialHLTracksWithTracksGenerator(Subfield &subfield,
                                                                           double trackDistance,
                                                                           const HeadlandParameters &plannerParameters,
                                                                           size_t expectedNumTracks)
{
    if(trackDistance < 1e-9 || expectedNumTracks == 0)
        return AroResp::ok();

    for(size_t hl_side = 1 ; hl_side <= 2 ; ++hl_side){
        PartialHeadland* hl = ( hl_side == 1 ? &subfield.headlands.partial.front() : &subfield.headlands.partial.back() );

        auto hl_boundary = hl->boundary;
        auto & hl_boundary_pts = hl->boundary.points;

        //workaround to check which points of the IF boundary are adjacent to the HL boundary (in a perfect world it would be ~0)
        double refDist = 0.1;
        for(auto& pt : subfield.boundary_inner.points){
            double dist = geometry::calc_dist_to_linestring(hl_boundary_pts, pt, false);
            if( dist < refDist )
                refDist = dist;
        }
        refDist = std::max(1e-5, refDist);
        refDist *= 10;
        //refDist *= 10;

        //get the segment of the inner boundary adjascent to the headland boundary (taking into account that the order of the points of the IF boundary is not known)

        //get index of a point that is not adjascent
        int ind_out = -1;
        for(size_t i = 0 ; i+1 < hl_boundary_pts.size() ; ++i){
            double dist = geometry::calc_dist_to_linestring(subfield.boundary_inner.points, hl_boundary_pts.at(i), false);
            if( dist > refDist ){
                ind_out = i;
                break;
            }
        }
        if(ind_out < 0){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error obtaining intersection points between infield boundary and headland #" + std::to_string(hl_side) + " (init ind_out)");
            return AroResp(1, "Error obtaining intersection points between infield boundary and headland #" + std::to_string(hl_side) + " (init ind_out)");
        }

        //get index of a point that is adjascent
        int ind_in = -1;
        for(size_t i = 1+ind_out ; i < hl_boundary_pts.size()+ind_out ; ++i){
            if(i+1 == hl_boundary_pts.size())
                continue;
            size_t ind = geometry::getIndexFromContinuousGeometry(hl_boundary_pts, i);
            double dist = geometry::calc_dist_to_linestring(subfield.boundary_inner.points, hl_boundary_pts.at(ind), false);
            if( dist < refDist ){
                if(ind+1 == hl_boundary_pts.size())
                    ind_in = 0;
                else
                    ind_in = ind;
                break;
            }
        }
        if(ind_in < 0){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error obtaining intersection points between infield boundary and headland #" + std::to_string(hl_side) + " (ind_in)");
            return AroResp(1, "Error obtaining intersection points between infield boundary and headland #" + std::to_string(hl_side) + " (ind_in)");
        }

        //get index of a point that is adjascent
        ind_out = -1;
        for(size_t i = 1+ind_in ; i < hl_boundary_pts.size()+ind_in ; ++i){
            size_t ind = geometry::getIndexFromContinuousGeometry(hl_boundary_pts, i);
            double dist = geometry::calc_dist_to_linestring(subfield.boundary_inner.points, hl_boundary_pts.at(ind), false);
            if( dist > refDist ){

                //sometimes fails -> check next one (if the next one is adjacent, this was a false positive
                size_t ind_next = geometry::getIndexFromContinuousGeometry(hl_boundary_pts, i+1);
                if(ind_next+1 == hl_boundary_pts.size())
                    ind_next = 0;
                if(ind_next != ind_in){
                    double dist2 = geometry::calc_dist_to_linestring(subfield.boundary_inner.points, hl_boundary_pts.at(ind_next), false);
                    if( dist2 < refDist )//false positive
                        continue;
                }

                ind_out = ind-1;
                break;
            }
        }
        if(ind_out < 0 || ind_out == ind_in){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error obtaining intersection points between infield boundary and headland #" + std::to_string(hl_side) + " (ind_out)");
            return AroResp(1, "Error obtaining intersection points between infield boundary and headland #" + std::to_string(hl_side) + " (ind_out)");
        }

        std::vector<Point> refLineHL = geometry::getShortestGeometryPart(hl_boundary.points, ind_in, ind_out);
        if(refLineHL.size() < 2 || refLineHL.size() == hl_boundary.points.size()){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating the reference line for headland #" + std::to_string(hl_side));
            return AroResp(1, "Error generating the reference line for headland #" + std::to_string(hl_side));
        }

        geometry::unsample_linestring(refLineHL);

        bool useRefLineAsCentralLine = false;
//        std::vector<Point> refLineHL1, refLineHL2;
//        geometry::offsetLinestring(refLineHL, refLineHL1, 0.5*trackDistance);
//        geometry::offsetLinestring(refLineHL, refLineHL2, -0.5*trackDistance);

//        refLineHL1 = geometry::sample_geometry( refLineHL1, 0.2 * geometry::getGeometryLength(refLineHL1) );
//        refLineHL2 = geometry::sample_geometry( refLineHL2, 0.2 * geometry::getGeometryLength(refLineHL2) );

//        if(geometry::count_points_in_polygon(refLineHL1, subfield.boundary_inner) < geometry::count_points_in_polygon(refLineHL2, subfield.boundary_inner))
//            std::swap(refLineHL1, refLineHL2);

//        if(geometry::getGeometryLength(refLineHL1) > geometry::getGeometryLength(refLineHL)){
//            refLineHL = refLineHL1;
//            geometry::unsample_linestring(refLineHL);
//            useRefLineAsCentralLine = true;
//        }
//        else
//            useRefLineAsCentralLine = false;

        //obtain the HL tracks
        auto resp = generatePartialHLTracksWithTracksGenerator(hl->boundary, refLineHL, trackDistance, plannerParameters.sampleResolution,
                                                               true, useRefLineAsCentralLine, expectedNumTracks, hl_side, hl->tracks);
        if(resp.isError()){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating tracks of headland #" + std::to_string(hl_side) + ": " + resp.msg);
            return AroResp(1, "Error generating tracks of headland #" + std::to_string(hl_side) + ": " + resp.msg);
        }

        for(auto& track : hl->tracks){

            auto trackPts = track.points;
            if(trackPts.size() < 2)
                return AroResp(1, "Error generating tracks of headland #" + std::to_string(hl_side) + ": one or more tracks have less than 2 points");

            auto deltaDist = std::min( 0.1, 0.1 * geometry::calc_dist(trackPts.front(), trackPts.at(1)) );
            trackPts.front() = geometry::getPointInLineAtDist(trackPts.front(), trackPts.at(1), deltaDist);
            deltaDist = std::min( 0.1, 0.1 * geometry::calc_dist(trackPts.back(), r_at(trackPts, 1)) );
            trackPts.back() = geometry::getPointInLineAtDist(trackPts.back(), r_at(trackPts, 1), deltaDist);

            if(geometry::intersects(trackPts, subfield.boundary_outer.points)){
                return AroResp(1, "Error generating tracks of headland #" + std::to_string(hl_side) + ": one or more tracks intersect with the boundary");
            }
        }
    }

    return AroResp::ok();

}

AroResp FieldGeometryProcessor::generatePartialHLTracksWithTracksGenerator(Subfield &subfield,
                                                                           const std::vector<Point>& hl_points_1,
                                                                           const std::vector<Point>& hl_points_2,
                                                                           double trackDistance,
                                                                           const HeadlandParameters &plannerParameters,
                                                                           size_t expectedNumTracks)
{
    if(trackDistance < 1e-9 || expectedNumTracks == 0)
        return AroResp::ok();

    for(size_t hl_side = 1 ; hl_side <= subfield.headlands.partial.size() ; ++hl_side){
        PartialHeadland* hl = ( hl_side == 1 ? &subfield.headlands.partial.front() : &subfield.headlands.partial.back() );
        auto& refLineHL = ( hl_side == 1 ? hl_points_1 : hl_points_2 );

        //obtain the HL tracks
        auto resp = generatePartialHLTracksWithTracksGenerator(hl->boundary, refLineHL, trackDistance, plannerParameters.sampleResolution,
                                                               false, false, expectedNumTracks, hl_side, hl->tracks);
        if(resp.isError()){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating tracks of headland #" + std::to_string(hl_side) + ": " + resp.msg);
            return AroResp(1, "Error generating tracks of headland #" + std::to_string(hl_side) + ": " + resp.msg);
        }
    }

    return AroResp::ok();

}

AroResp FieldGeometryProcessor::generatePartialHLTracksWithTracksGenerator(const Polygon &boundary,
                                                                           const std::vector<Point> &refLineHL,
                                                                           double trackDistance,
                                                                           double sampleResolution,
                                                                           bool isRefLineInwards,
                                                                           bool useRefLineAsCentralLine,
                                                                           size_t expectedNumTracks,
                                                                           size_t headlandNumber,
                                                                           std::vector<Track> &tracks)
{
    geometry::TracksGenerator trGen;
    trGen.logger().setParent(loggerPtr());
    FieldGeometryProcessor::InfieldParameters tgParams;
    tgParams.sampleResolution = sampleResolution;
    tgParams.onlyUntilBoundaryIntersection = true;
    tgParams.splitBoundaryIntersectingTracks = false;
    tgParams.checkForRemainingTracks = false;
    tgParams.shiftingStrategy = geometry::TracksGenerator::ShiftingStrategy::OFFSET_TRACKS;
    tgParams.useRefLineAsCentralLine = useRefLineAsCentralLine;

//    Polygon boundaryEd;
//    if(!geometry::offsetPolygon(boundary, boundaryEd, 0.5 * trackDistance, true))
//        boundaryEd = boundary;

    auto& boundaryEd = boundary;

    auto resp = trGen.generateTracks(boundaryEd, refLineHL, {trackDistance}, tgParams, tracks);
    if(resp.isError()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating tracks: " + resp.msg);
        return AroResp(1, "Error generating tracks: " + resp.msg);
    }

    for(size_t i = 0 ; i < tracks.size() ; ++i){
        if(tracks.at(i).points.size() < 2){
            tracks.erase(tracks.begin()+i);
            i--;
        }
    }

    while(tracks.size() > expectedNumTracks){
        auto intersectionsFront = geometry::get_intersection(tracks.front().boundary, boundary);
        auto intersectionsBack = geometry::get_intersection(tracks.back().boundary, boundary);
        double areaFront = 0, areaBack = 0;
        for(auto& poly : intersectionsFront)
            areaFront += geometry::calc_area(poly);
        for(auto& poly : intersectionsBack)
            areaBack += geometry::calc_area(poly);
        if(areaFront < areaBack)
            pop_front(tracks);
        else
            tracks.pop_back();
    }

    if(!tracks.empty()){
        double distFirst = geometry::calc_dist_to_linestring(refLineHL, geometry::getPointAtHalfLength(tracks.front().points).first, false);
        double distLast = geometry::calc_dist_to_linestring(refLineHL, geometry::getPointAtHalfLength(tracks.back().points).first, false);
        if(isRefLineInwards){
            if( distFirst < distLast )
                std::reverse(tracks.begin(), tracks.end());//reverse so that the 1st track is the outermost track
        }
        else{
            if( distFirst > distLast )
                std::reverse(tracks.begin(), tracks.end());//reverse so that the 1st track is the outermost track
        }
    }

    for(size_t i = 0 ; i < tracks.size() ; ++i){
        if(geometry::intersects(tracks.at(i).points)){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating tracks: a track self intersects");
            return AroResp(1, "Error generating tracks: a track self intersects");
        }
        for(size_t j = i+1 ; j < tracks.size() ; ++j){
            if(geometry::intersects(tracks.at(i).points, tracks.at(j).points, false, false)){
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating tracks: tracks intersect");
                return AroResp(1, "Error generating tracks: tracks intersect");
            }
        }
        tracks.at(i).id = Track::DeltaHLTrackId + 1000 * headlandNumber + i;
    }
    return AroResp::ok();

}

AroResp FieldGeometryProcessor::addConnectingHeadlands(Subfield &subfield, double trackWidth, double sampleResolution, double headlandWidth){

    if(headlandWidth < trackWidth){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "A connecting track with a width higher than the headland width is not supported at the moment");
        return AroResp(1, "A connecting track with a width higher than the headland width is not supported at the moment");
    }

    Polygon boundaryTmp;
    if(!geometry::offsetPolygon(subfield.boundary_outer, boundaryTmp, (trackWidth-1e-3), false)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting outer boundary");
        return AroResp(1, "Error offsetting outer boundary");
    }

    std::vector<PolygonWithHoles> diffPolys;
    geometry::subtract_intersection(subfield.boundary_inner, boundaryTmp, diffPolys);
    if(diffPolys.size() < subfield.headlands.partial.size()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error subtracting IF boundary from polygon to obtain boundaries of connecting headlands");
        return AroResp(1, "Error subtracting IF boundary from polygon to obtain boundaries of connecting headlands");
    }
    if(diffPolys.size() > subfield.headlands.partial.size()){
        std::sort(diffPolys.begin(), diffPolys.end(), [](const PolygonWithHoles& a, const PolygonWithHoles& b){
            return geometry::calc_area(a.outer) > geometry::calc_area(b.outer);
        });
    }

    Polygon trackPoly;
    if(!geometry::offsetPolygon(subfield.boundary_outer, trackPoly, 0.5 * trackWidth, false)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting outer boundary (2)");
        return AroResp(1, "Error offsetting outer boundary (2)");
    }

    //compute new IF boundary
    std::vector<Polygon> intersectionPolys = geometry::get_intersection(subfield.boundary_inner, boundaryTmp);
    if(intersectionPolys.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error obtaining new IF boundary");
        return AroResp(1, "Error obtaining new IF boundary");
    }
    std::sort(intersectionPolys.begin(), intersectionPolys.end(), [](const Polygon& a, const Polygon& b){
        return geometry::calc_area(a) > geometry::calc_area(b);
    });
    subfield.boundary_inner = intersectionPolys.front();
    geometry::unsample_linestring(subfield.boundary_inner.points);
    geometry::correct_polygon(subfield.boundary_inner);
    geometry::removeSpikes(subfield.boundary_inner);
    if(geometry::isPolygonValid(subfield.boundary_inner) == geometry::PolygonValidity::INVALID_POLYGON){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "New inner-field boundary is an invalid polygon");
        return AroResp(1, "New inner-field boundary is an invalid polygon");
    }


    size_t numHLs = subfield.headlands.partial.size();
    for(size_t i = 0 ; i < numHLs ; ++i){
        subfield.headlands.partial.push_back(PartialHeadland());
        auto& hl = subfield.headlands.partial.back();
        hl.id = subfield.headlands.partial.size()-1;
        hl.boundary = diffPolys.at(i).outer;

        if(numHLs == 1)
            hl.connectingHeadlandIds = std::make_pair(subfield.headlands.partial.front().id,
                                                      subfield.headlands.partial.front().id);
        else
            hl.connectingHeadlandIds = std::make_pair(subfield.headlands.partial.at(0).id,
                                                      subfield.headlands.partial.at(1).id);

        geometry::unsample_polygon(hl.boundary);
        geometry::remove_repeated_points(hl.boundary.points);
        geometry::correct_polygon(hl.boundary);
        geometry::removeSpikes(hl.boundary);
        if(geometry::isPolygonValid(hl.boundary) == geometry::PolygonValidity::INVALID_POLYGON){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Boundary of connecting headland is an invalid polygon");
            return AroResp(1, "Boundary of connecting headland is an invalid polygon");
        }

        auto intersections = geometry::get_intersection(hl.boundary.points, trackPoly.points);
        if(intersections.size() != 2){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error generating track of connecting headlad: intersections != 2");
            return AroResp(1, "Error generating track of connecting headlad: intersections != 2");
        }

        auto ind1 = geometry::addSampleToGeometryClosestToPoint(trackPoly.points, intersections.front(), 1);
        auto ind2 = geometry::addSampleToGeometryClosestToPoint(trackPoly.points, intersections.back(), 1);
        if(ind1 >= ind2)
            ++ind1;

        hl.tracks.emplace_back(Track());
        auto& track = hl.tracks.back();
        track.id = Track::DeltaHLTrackId + 1000 * subfield.headlands.partial.size();
        track.width = trackWidth;

        track.points = geometry::getGeometryPart(trackPoly.points, ind1, ind2, false, false);
        if( !geometry::in_polygon(geometry::getPointAtHalfLength(track.points).first, hl.boundary) )
            track.points = geometry::getGeometryPart(trackPoly.points, ind2, ind1, true, false);

        geometry::unsample_linestring(track.points, 0.1 * trackWidth, m_unsampleAngTol);
        if(sampleResolution > 1e-9)
            track.points = geometry::sample_geometry(track.points, sampleResolution, 0.25 * sampleResolution);

        geometry::offsetLinestring(track.points, track.boundary, 0.5*trackWidth, 0.5*trackWidth, true);
    }

    return AroResp(0, "OK" );
}

AroResp FieldGeometryProcessor::adjustMainHeadlandTrackEnds(Subfield &subfield, double trackWidth){

    const double eps = 1.2;
    for(auto& hl : subfield.headlands.partial){
        if(hl.isConnectingHeadland() || hl.tracks.empty())
            continue;

        Polygon boundaryFront, boundaryBack;
        if( hl.tracks.size() == 1 ){
            boundaryFront = geometry::create_circle( hl.tracks.front().points.front(), trackWidth, 36 );
            boundaryBack = geometry::create_circle( hl.tracks.front().points.back(), trackWidth, 36 );
        }
        else{
            PointVec ptsFront, ptsBack;
            for(auto& track : hl.tracks){
                ptsFront.push_back(track.points.front());
                ptsBack.push_back(track.points.back());
            }

            geometry::offsetLinestring(ptsFront, boundaryFront, 0.5*trackWidth, 0.5*trackWidth, false, 18);
            geometry::offsetLinestring(ptsBack, boundaryBack, 0.5*trackWidth, 0.5*trackWidth, false, 18);
        }

        for(size_t i_track = 0 ; i_track < hl.tracks.size() ; ++i_track){
            auto& track = hl.tracks.at(i_track);
            bool erase = true;
            for(int side = 0 ; side < 2 ; side++){
                for(size_t i = 0 ; i+1 < track.points.size() ; ++i){

                    PointVec intersections;
                    if(side == 0)
                        intersections = geometry::get_intersection( track.points.at(i+1), track.points.at(i), boundaryFront.points, true, false, false );
                    else
                        intersections = geometry::get_intersection( r_at(track.points, i+1), r_at(track.points, i), boundaryBack.points, true, false, false );
                    if(intersections.empty())
                        continue;

                    if(side == 0){
                        track.points.at(i) = intersections.front();
                        pop_front(track.points, i);
                    }
                    else{
                        r_at(track.points, i) = intersections.front();
                        pop_back(track.points, i);
                    }

                    erase = false;
                }
                if(erase)
                    break;
            }

            Polygon hlBoundaryEd;
            if( !erase && geometry::offsetPolygon(hl.boundary, hlBoundaryEd, 0.25*trackWidth, false) ){
                int ind0 = -1;
                for(size_t side = 0 ; side < 2 ; ++side){
                    for(size_t i = 0 ; i+i < track.points.size() ; ++i){
                        Point& pt = (side == 0 ? track.points.at(i) : r_at(track.points, i));
                        if(geometry::in_polygon(pt, hlBoundaryEd)){
                            ind0 = i;
                            break;
                        }
                    }
                    if(ind0 < 0){
                        erase = true;
                        break;
                    }
                    if (ind0 == 0)
                        continue;
                    Point& pt1 = (side == 0 ? track.points.at(ind0) : r_at(track.points, ind0));
                    Point& pt2 = (side == 0 ? track.points.at(ind0-1) : r_at(track.points, ind0-1));
                    PointVec intersections = geometry::get_intersection(pt1, pt2, hlBoundaryEd.points, true, false, false);
                    if(!intersections.empty()){
                        pt2 = intersections.front();
                        ind0--;
                    }
                    if (ind0 == 0)
                        continue;
                    if(side == 0)
                        pop_front(track.points, ind0);
                    else
                        pop_back(track.points, ind0);

                }

            }

            if(erase){
                hl.tracks.erase(hl.tracks.begin()+i_track);
                --i_track;
                continue;
            }

            for(size_t side = 0 ; side < 2 ; ++side){
                const Point& p0 = ( side == 0 ? track.points.front() : track.points.back() );
                size_t indStop = 1;
                for(; indStop < track.points.size() ; ++indStop){
                    const Point& p1 = ( side == 0 ? track.points.at(indStop) : r_at(track.points, indStop) );
                    if(indStop + 1 < track.points.size()){
                        const Point& p2 = ( side == 0 ? track.points.at(indStop+1) : r_at(track.points, indStop+1) );
                        if( !geometry::is_line(PointVec{p0, p1, p2}) )
                            break;
                    }
                    if( geometry::calc_dist(p0, p1) > eps*0.5*trackWidth )
                        break;
                }

                if(indStop == track.points.size())
                    --indStop;

                if(side == 0)
                    track.points.erase( track.points.begin()+1, track.points.begin()+ indStop );
                else
                    track.points.erase( track.points.end()-indStop, track.points.end()-1 );

                Point p1 = ( side == 0 ? track.points.at(1) : r_at(track.points, 1) );
                double dist = geometry::calc_dist(p0, p1);
                if( dist <= 0.5*trackWidth )
                    continue;
                if(side == 0)
                    track.points.insert( track.points.begin()+1, geometry::getPointInLineAtDist(p0, p1, 0.5*trackWidth) );
                else
                    track.points.insert( track.points.end()-1, geometry::getPointInLineAtDist(p0, p1, 0.5*trackWidth) );

            }
        }
    }
    return AroResp::ok();
}

}
