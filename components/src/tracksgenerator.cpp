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
 
#include "arolib/components/tracksgenerator.h"

#include <chrono>

namespace arolib {

TracksGenerator::ShiftingStrategy TracksGenerator::intToShiftingStrategy(int value)
{
    if(value == ShiftingStrategy::TRANSLATE_TRACKS)
        return ShiftingStrategy::TRANSLATE_TRACKS;
    else if(value == ShiftingStrategy::OFFSET_TRACKS)
        return ShiftingStrategy::OFFSET_TRACKS;

    throw std::invalid_argument( "The given value does not correspond to any TracksGenerator::ShiftingStrategy" );
}

TracksGenerator::TrackOrderStrategy TracksGenerator::intToTrackOrderStrategy(int value)
{
    if(value == TrackOrderStrategy::LEFTMOST_TRACK_FIRST)
        return TrackOrderStrategy::LEFTMOST_TRACK_FIRST;
    else if(value == TrackOrderStrategy::LONGEST_TRACK_FIRST)
        return TrackOrderStrategy::LONGEST_TRACK_FIRST;
    else if(value == TrackOrderStrategy::EXTRA_TRACK_LAST)
        return TrackOrderStrategy::EXTRA_TRACK_LAST;

    throw std::invalid_argument( "The given value does not correspond to any TracksGenerator::TrackOrderStrategy" );

}

TracksGenerator::TrackSamplingStrategy TracksGenerator::intToTrackSamplingStrategy(int value)
{
    if(value == TrackSamplingStrategy::DEFAULT_SAMPLING)
        return TrackSamplingStrategy::DEFAULT_SAMPLING;
    else if(value == TrackSamplingStrategy::START_AT_TRACK_START)
        return TrackSamplingStrategy::START_AT_TRACK_START;
    else if(value == TrackSamplingStrategy::MIN_DIST_BETWEEN_TRACKS)
        return TrackSamplingStrategy::MIN_DIST_BETWEEN_TRACKS;

    throw std::invalid_argument( "The given value does not correspond to any TracksGenerator::TrackSamplingStrategy" );
}

TracksGenerator::TracksGeneratorParameters::TracksGeneratorParameters(double _trackDistance, double _sampleResolution):
    trackDistance(_trackDistance),
    sampleResolution(_sampleResolution)
{

}

bool TracksGenerator::TracksGeneratorParameters::parseFromStringMap(TracksGenerator::TracksGeneratorParameters &params, const std::map<std::string, std::string> &map, bool strict)
{
    TracksGenerator::TracksGeneratorParameters tmp;

    try{

        int shiftingStrategy, trackOrderStrategy, trackSamplingStrategy;
        std::map<std::string, double*> dMap = { {"trackDistance" , &tmp.trackDistance},
                                                {"sampleResolution" , &tmp.sampleResolution},
                                                {"direction.x" , &tmp.direction.x},
                                                {"direction.y" , &tmp.direction.y},
                                                {"trackAreaThreshold" , &tmp.trackAreaThreshold} };
        std::map<std::string, bool*> bMap = { {"checkForRemainingTracks" , &tmp.checkForRemainingTracks},
                                              {"useRefLineAsCentralLine" , &tmp.useRefLineAsCentralLine},
                                              {"onlyUntilBoundaryIntersection" , &tmp.onlyUntilBoundaryIntersection}};
        std::map<std::string, int*> enumMap = { {"shiftingStrategy" , &shiftingStrategy},
                                                {"trackOrderStrategy" , &trackOrderStrategy},
                                                {"trackSamplingStrategy" , &trackSamplingStrategy} };

        if( !setValuesFromStringMap( map, dMap, strict)
                || !setValuesFromStringMap( map, bMap, strict)
                || !setValuesFromStringMap( map, enumMap, strict) )
            return false;

        tmp.shiftingStrategy = intToShiftingStrategy( shiftingStrategy );
        tmp.trackOrderStrategy = intToTrackOrderStrategy( trackOrderStrategy );
        tmp.trackSamplingStrategy = intToTrackSamplingStrategy( trackSamplingStrategy );

    } catch(...){ return false; }

    params = tmp;
    return true;
}

std::map<std::string, std::string> TracksGenerator::TracksGeneratorParameters::parseToStringMap(const TracksGenerator::TracksGeneratorParameters &params)
{
    std::map<std::string, std::string> ret;

    ret["trackDistance"] = double2string( params.trackDistance );
    ret["sampleResolution"] = double2string( params.sampleResolution );
    ret["direction.x"] = double2string( params.direction.x );
    ret["direction.y"] = double2string( params.direction.y );
    ret["trackAreaThreshold"] = double2string( params.trackAreaThreshold );
    ret["checkForRemainingTracks"] = std::to_string( params.checkForRemainingTracks );
    ret["useRefLineAsCentralLine"] = std::to_string( params.useRefLineAsCentralLine );
    ret["onlyUntilBoundaryIntersection"] = std::to_string( params.onlyUntilBoundaryIntersection );
    ret["shiftingStrategy"] = std::to_string( params.shiftingStrategy );
    ret["trackOrderStrategy"] = std::to_string( params.trackOrderStrategy );
    ret["trackSamplingStrategy"] = std::to_string( params.trackSamplingStrategy );

    return ret;
}

TracksGenerator::TracksGenerator(const arolib::LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{

}

AroResp TracksGenerator::generateTracks(Polygon boundary,
                                        std::vector<Point> refLine,
                                        const std::vector<double> &distances,
                                        const TracksGenerator::TracksGeneratorParameters &settings,
                                        std::vector<Track> &tracks)
{
    try {
        AroResp compResp;
        tracks.clear();

        geometry::correct_polygon(boundary);
        if( arolib::geometry::isPolygonValid(boundary) == arolib::geometry::PolygonValidity::INVALID_POLYGON )
            return AroResp(1, "The boundary is not valid");
        if(distances.empty())
            return AroResp(1, "No distances given.");
        for(auto & d : distances){
            if(d <= 0)
                return AroResp(1, "Invalid distance " + std::to_string(d));
        }
        geometry::unsample_linestring(refLine);
        if(refLine.size() < 2)
            return AroResp(1, "The (unsampled) reference line has less than two points");

        bool isRefLineCenter = settings.useRefLineAsCentralLine;

        //check if the reference line lies partially or completelly inside the inner boundary of the subfield (if so, use it as it is; otherwise translate it to a suitable position)
        if( !geometry::in_polygon(refLine, boundary, false) && !geometry::intersects(refLine, boundary, false, false) ){
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "None of the points of the reference line lie within the inner boundary of the subfield, nor the reference line intersects with the inner boundary. The reference line will be translated to the best fitting position.");
            compResp = relocateReferenceLine(boundary, refLine, refLine);
            if( compResp.isError() ){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error translating the reference line: " + compResp.msg);
                return AroResp(1, "Error translating the reference line: " + compResp.msg );
            }
            isRefLineCenter = false;
        }

        std::vector<TrackGeometries> tracks_geom;

        if(settings.shiftingStrategy == ShiftingStrategy::TRANSLATE_TRACKS){
            Point direction = settings.direction;
            if(refLine.size() == 2)
                direction = Point(0,0);
            std::string directionStr = "";
            if(direction == Point(0,0))
                directionStr = " (calculated from the normal vectors to the reference line segments) ";

            if(direction == Point(0,0)){
                m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Calculating the direction of track shifting based on the normal vectors to the reference line segments... ");
                if( !arolib::geometry::getNormVector(refLine, direction) ){
                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error calculating the direction of track shifting based on the normal vectors to the reference line segments");
                    return AroResp(1, "Error calculating the direction of track shifting based on the normal vectors to the reference line segments" );
                }

            }

            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Translating reference line with a track distance of " + std::to_string(settings.trackDistance) + " m" +
                     " and direction " + direction.toString() + directionStr);
            compResp = translateReferenceLine(boundary,
                                              distances,
                                              refLine,
                                              direction,
                                              isRefLineCenter,
                                              tracks_geom);
            if( compResp.isError() ){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error translating the reference line: " + compResp.msg);
                return AroResp(1, "Error translating the reference line: " + compResp.msg );
            }
        }
        else{
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Offsetting reference line with a track distance of " + std::to_string(settings.trackDistance) + " m");
            compResp = offsetReferenceLine(boundary,
                                           distances,
                                           refLine,
                                           isRefLineCenter,
                                           tracks_geom);
            if( compResp.isError() ){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting the reference line: " + compResp.msg);
                return AroResp(1, "Error offsetting the reference line: " + compResp.msg );
            }
        }
        m_logger.printOut(LogLevel::INFO, __FUNCTION__, std::to_string(tracks_geom.size()) + " initial track geometries generated.");

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Connecting tracks to boundary...");
        compResp = connectTracksToBoundary(boundary, tracks_geom, settings.onlyUntilBoundaryIntersection);
        if( compResp.isError() ){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error connecting tracks to boundary: " + compResp.msg);
            return AroResp(1, "Error connecting tracks to boundary: " + compResp.msg );
        }

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Creating tracks from geometries...");
        compResp = createTracks(tracks_geom, tracks);
        if( compResp.isError() ){
            tracks.clear();
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error creating tracks from geometries: " + compResp.msg);
            return AroResp(1, "Error creating tracks from geometries: " + compResp.msg );
        }

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Removing 'empty' tracks...");
        compResp = removeEmptyTracks(boundary, tracks, settings.trackAreaThreshold);
        if( compResp.isError() ){
            tracks.clear();
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error removing 'empty' tracks: " + compResp.msg);
            return AroResp(1, "Error removing 'empty' tracks: " + compResp.msg );
        }

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Ordering tracks...");
        orderTracks(tracks, settings.trackOrderStrategy);

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Sampling tracks...");
        compResp = sampleTracks(tracks, settings, boundary);
        if( compResp.errorID > 0 ){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error sampling tracks: " + compResp.msg);
            return AroResp(1, "Error sampling tracks: " + compResp.msg );
        }

        return AroResp(0, "OK" );
    }
    catch (std::exception &e) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what() );
        return AroResp(1, e.what() );
    }

}

AroResp TracksGenerator::generateTracks(Field &field,
                                              const TracksGeneratorParameters &tracksGeneratorParameters,
                                              const std::vector<size_t> &referenceLineIndexes,
                                              const ArolibGrid_t &yieldMap,
                                              const ArolibGrid_t &remainingAreaMap) {


    LoggingComponent::LoggersHandler lh(true);//will be reset on destruction
    LoggingComponent::setTemporalLoggersParent(lh, *this, yieldMap, remainingAreaMap);
    try {
        if (tracksGeneratorParameters.trackDistance == 0) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid track width (=0)" );
            return AroResp(1, "Invalid track width (=0)" );
        }

        if (referenceLineIndexes.size() != 0 && referenceLineIndexes.size() != field.subfields.size()) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid number of reference line indexes (must be 0 or equal to the number of subfields)" );
            return AroResp(1, "Invalid number of reference line indexes (must be 0 or equal to the number of subfields)" );
        }

        for (int i = 0; i < field.subfields.size(); ++i) {
            size_t referenceLineIndex = 0;
            if(!referenceLineIndexes.empty())
                referenceLineIndex = referenceLineIndexes.at(i);

            Subfield &sf = field.subfields.at(i);
            // check for reference line
            if(sf.reference_lines.empty()) {
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Subfield " + std::to_string(i) + " has no reference lines.");
                return AroResp(1, "Subfield " + std::to_string(i) + " has no reference lines." );
            }
            if(sf.reference_lines.size() <=  referenceLineIndex) {
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid reference line index '" + std::to_string(referenceLineIndex) + "' for subfield " + std::to_string(i) +
                         " : Nr. of reference lines is " + std::to_string(sf.reference_lines.size()));
                return AroResp(1, "Invalid reference line index" );
            }

            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Shifting subfield " + std::to_string(i) + "...");
            AroResp compResp = generateTracks(sf,
                                              tracksGeneratorParameters,
                                              referenceLineIndex,
                                              yieldMap,
                                              remainingAreaMap);

            if (compResp.errorID > 0){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Problem with subfield " + std::to_string(i) );
                return AroResp(1, "Problem with subfield " + std::to_string(i) + ": " + compResp.msg );
            }
        }

        return AroResp(0, "OK" );
    }
    catch (std::exception &e) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what() );
        return AroResp(1, e.what() );
    }
}

AroResp TracksGenerator::generateTracks(Subfield &subfield,
                                              const TracksGeneratorParameters &tracksGeneratorParameters,
                                              const size_t &referenceLineIndex,
                                              const ArolibGrid_t &yieldMap,
                                              const ArolibGrid_t &remainingAreaMap)
{

    //test!
    return generateTracks(subfield.boundary_inner, subfield.reference_lines.at(referenceLineIndex).points, {tracksGeneratorParameters.trackDistance}, tracksGeneratorParameters, subfield.tracks);

    //previous method bellow | | |
    //                       V V V

    Point pTrans0 = Point(0,0);

    LoggingComponent::LoggersHandler lh(true);//will be reset on destruction
    LoggingComponent::setTemporalLoggersParent(lh, *this, yieldMap, remainingAreaMap);

    try {
        AroResp compResp;
        if (tracksGeneratorParameters.trackDistance <= 0) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid track width (<=0)" );
            return AroResp(1, "Invalid track width (<=0)" );
        }

        if (tracksGeneratorParameters.sampleResolution <= 0) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid sample resolution (<=0)" );
            return AroResp(1, "Invalid sample resolution (<=0)" );
        }

        if(subfield.boundary_inner.points.size() < 3) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The inner boundary of the subfield has less than 3 points.");
            return AroResp(1, "The inner boundary of the subfield has less than 3 points." );
        }
        if(arolib::geometry::isPolygonValid(subfield.boundary_inner) == arolib::geometry::PolygonValidity::INVALID_POLYGON){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The inner boundary of the subfield is not valid.");
            return AroResp(1, "The inner boundary of the subfield is not valid." );
        }

        // check for reference line
        if(subfield.reference_lines.empty()) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The subfield has no reference lines.");
            return AroResp(1, "The subfield has no reference lines." );
        }
        if(subfield.reference_lines.size() <=  referenceLineIndex) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid reference line index '" + std::to_string(referenceLineIndex) + " : Nr. of reference lines is " + std::to_string(subfield.reference_lines.size()));
            return AroResp(1, "Invalid reference line index" );
        }
        if(subfield.reference_lines.at(referenceLineIndex).points.size() < 2 ) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Reference line has less than 2 points.");
            return AroResp(1, "Reference line has less than 2 points." );
        }

        if (m_doInitialTranslation)//translate all important geometries of the subfield to the origin (0,0)
            pTrans0 = subfield.boundary_inner.points.at(0);
        translateAll(subfield, referenceLineIndex, pTrans0 * -1);

        Linestring refLine = subfield.reference_lines.at(referenceLineIndex);

        //remove repeated consecutive points
        geometry::unsample_linestring(refLine.points);


        //check if the reference line lies partially or completelly inside the inner boundary of the subfield (if so, use it as it is; otherwise translate it to a suitable position)
        bool startAtRefLine = arolib::geometry::in_polygon(refLine, subfield.boundary_inner, false) || arolib::geometry::intersects(refLine.points, subfield.boundary_inner, false, false);
        if( !startAtRefLine ){
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "None of the points of the reference line lie within the inner boundary of the subfield, nor the reference line intersects with the inner boundary. The reference line will be translated to the best fitting position.");
            compResp = relocateReferenceLine(subfield.boundary_inner, subfield.reference_lines.at(referenceLineIndex).points, refLine.points);
            if( compResp.errorID > 0 ){
                translateAll(subfield, referenceLineIndex, pTrans0);//(if necessary) translate all the translated subfield geometries back to their original location
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error translating the reference line: " + compResp.msg);
                return AroResp(1, "Error translating the reference line: " + compResp.msg );
            }
        }

        if(tracksGeneratorParameters.shiftingStrategy == ShiftingStrategy::TRANSLATE_TRACKS){
            Point direction = tracksGeneratorParameters.direction;
            if(refLine.points.size() == 2)
                direction = Point(0,0);
            std::string directionStr = "";
            if(direction == Point(0,0))
                directionStr = " (calculated from the normal vectors to the reference line segments) ";

            if(direction == Point(0,0)){
                m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Calculating the direction of track shifting based on the normal vectors to the reference line segments... ");
                auto reLineSampled = geometry::sample_geometry(refLine.points, tracksGeneratorParameters.sampleResolution);
                if( !arolib::geometry::getNormVector(reLineSampled, direction) ){
                    translateAll(subfield, referenceLineIndex, pTrans0);//(if necessary) translate all the translated subfield geometries back to their original location
                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error calculating the direction of track shifting based on the normal vectors to the reference line segments");
                    return AroResp(1, "Error calculating the direction of track shifting based on the normal vectors to the reference line segments" );
                }

            }

            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Translating reference line with a track distance of " + std::to_string(tracksGeneratorParameters.trackDistance) + " m" +
                     " and direction " + direction.toString() + directionStr);
            compResp = translateReferenceLine_old(subfield,
                                              tracksGeneratorParameters.trackDistance,
                                              refLine,
                                              direction,
                                              startAtRefLine,
                                              tracksGeneratorParameters.checkForRemainingTracks);
            if( compResp.errorID > 0 ){
                translateAll(subfield, referenceLineIndex, pTrans0);//(if necessary) translate all the translated subfield geometries back to their original location
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error translating the reference line: " + compResp.msg);
                return AroResp(1, "Error translating the reference line: " + compResp.msg );
            }
        }
        else{
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Offsetting reference line with a track distance of " + std::to_string(tracksGeneratorParameters.trackDistance) + " m");
            compResp = offsetReferenceLine_old(subfield,
                                           tracksGeneratorParameters.trackDistance,
                                           refLine,
                                           startAtRefLine,
                                           tracksGeneratorParameters.checkForRemainingTracks);
            if( compResp.errorID > 0 ){
                translateAll(subfield, referenceLineIndex, pTrans0);//(if necessary) translate all the translated subfield geometries back to their original location
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting the reference line: " + compResp.msg);
                return AroResp(1, "Error offsetting the reference line: " + compResp.msg );
            }
        }
        m_logger.printOut(LogLevel::INFO, __FUNCTION__, std::to_string(subfield.tracks.size()) + " generated.");

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Connecting tracks to inner boundary...");
        compResp = connectTracksToInnerBoundary(subfield);
        if( compResp.errorID > 0 ){
            translateAll(subfield, referenceLineIndex, pTrans0);//(if necessary) translate all the translated subfield geometries back to their original location
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error connecting tracks to inner boundary: " + compResp.msg);
            return AroResp(1, "Error connecting tracks to inner boundary: " + compResp.msg );
        }

        orderTracks(subfield.tracks, tracksGeneratorParameters.trackOrderStrategy);

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Connecting tracks to headland...");

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Sampling tracks...");
        compResp = sampleTracks(subfield.tracks, tracksGeneratorParameters, subfield.boundary_inner);
        if( compResp.errorID > 0 ){
            translateAll(subfield, referenceLineIndex, pTrans0);//(if necessary) translate all the translated subfield geometries back to their original location
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error sampling tracks: " + compResp.msg);
            return AroResp(1, "Error sampling tracks: " + compResp.msg );
        }

        translateAll(subfield, referenceLineIndex, pTrans0);//(if necessary) translate all the translated subfield geometries back to their original location
        return AroResp(0, "OK" );

    } catch (std::exception &e) {
        translateAll(subfield, referenceLineIndex, pTrans0);//(if necessary) translate all the translated subfield geometries back to their original location
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what() );
        return AroResp(1, std::string("Exception cought: ") + e.what() );
    }

}

AroResp TracksGenerator::generateTracks(PlanningWorkspace &pw, const TracksGenerator::TracksGeneratorParameters &tracksGeneratorParameters, const std::vector<size_t> &referenceLineIndexes)
{
    const auto &yieldmap = getMaps(pw).at( PlanningWorkspace::GridType::YIELD );
    const auto &remainingAreaMap = getMaps(pw).at( PlanningWorkspace::GridType::REMAINING_AREA );

    m_calcGridValueOption = CALC_PW;
    m_planningWorkspace = &pw;
    auto ret = generateTracks(getField(pw), tracksGeneratorParameters, referenceLineIndexes, yieldmap, remainingAreaMap);

    m_planningWorkspace = nullptr;
    m_calcGridValueOption = CALC_DIRECT;

    return ret;
}

AroResp TracksGenerator::generateTracks(PlanningWorkspace &pw,
                                              size_t subfieldIdx,
                                              const TracksGenerator::TracksGeneratorParameters &tracksGeneratorParameters,
                                              const size_t &referenceLineIndex)
{

    if (subfieldIdx >= getField(pw).subfields.size()) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfield index" );
        return AroResp(1, "Invalid subfield index" );
    }
    Subfield& subfield = getField(pw).subfields.at(subfieldIdx);
    const auto &yieldmap = getMaps(pw).at( PlanningWorkspace::GridType::YIELD );
    const auto &remainingAreaMap = getMaps(pw).at( PlanningWorkspace::GridType::REMAINING_AREA );

    m_calcGridValueOption = CALC_PW;
    m_planningWorkspace = &pw;

    auto ret = generateTracks(subfield, tracksGeneratorParameters, referenceLineIndex, yieldmap, remainingAreaMap);

    m_planningWorkspace = nullptr;
    m_calcGridValueOption = CALC_DIRECT;

    return ret;
}


AroResp TracksGenerator::relocateReferenceLine(const Polygon &boundary, const std::vector<Point> &refLine, std::vector<Point> &refLineTrans)
{
    std::vector<Point> refLineTmp;
    const std::vector<Point>* pRefLine = &refLine;

    if(&refLine == &refLineTrans){
        refLineTmp = *pRefLine;
        pRefLine = &refLineTmp;
    }

    refLineTrans.clear();

    if(boundary.points.empty())
        return AroResp(1, "Invalid subfield inner boundary." );

    if(pRefLine->size() < 2)
        return AroResp(1, "Invalid reference line." );

    double tunnelRad = 1;
    double maxBenefit = 0; //benefit = segment_length + c * sum[ 1/dist_point_to_refline ]
    Point trans(0,0);

    double c = 1;
    size_t indStart = 0, indEnd = 0, indRef = 0;

    for(size_t i = 0 ; i < boundary.points.size() ; ++i){//try translating the ref line to each one of the points of the boundary
        for(size_t j = 0 ; j < pRefLine->size() ; ++j){//try translating the ref line based on each one of its points
            std::vector<Point> refPointsTmp = arolib::geometry::translate(*pRefLine, boundary.points.at(i) - pRefLine->at(j));
            double seg_length = 0, sumDistPointsInv = 0;
            Point prevPoint = boundary.points.at(i);

            size_t indIB;
            for(size_t ii = 1 ; ii < boundary.points.size() ; ++ii){
                indIB = (i+ii)%boundary.points.size();
                double dist = arolib::geometry::calc_dist_to_linestring(refPointsTmp, boundary.points.at(indIB), true);
                if(dist > tunnelRad)
                    break;
                seg_length += arolib::geometry::calc_dist( prevPoint, boundary.points.at(indIB) );
                sumDistPointsInv += (c * (1-dist));
                prevPoint = boundary.points.at(indIB);

            }

            if( seg_length > 0 && maxBenefit < seg_length + sumDistPointsInv ){
                maxBenefit = seg_length + sumDistPointsInv;
                trans = boundary.points.at(i) - pRefLine->at(j);
                indStart = i;
                indEnd = indIB-1;
                indRef = j;
            }
        }
    }

    m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "indRef = " + std::to_string(indRef) +
                                            " ; indStart = " + std::to_string(indStart) +
                                            " ; indEnd = " + std::to_string(indEnd));

    if(pRefLine->size() == 2){//so it looks better, but shouldn't matter
        if( arolib::geometry::calc_dist( pRefLine->front(), boundary.points.at(indStart) ) + arolib::geometry::calc_dist( pRefLine->back(), boundary.points.at(indEnd) )
                < arolib::geometry::calc_dist( pRefLine->back(), boundary.points.at(indStart) ) + arolib::geometry::calc_dist( pRefLine->front(), boundary.points.at(indEnd) ) )
            refLineTrans = arolib::geometry::translate(*pRefLine, boundary.points.at(indStart) - pRefLine->front());
        else
            refLineTrans = arolib::geometry::translate(*pRefLine, boundary.points.at(indStart) - pRefLine->back());
    }
    else
        refLineTrans = arolib::geometry::translate(*pRefLine, trans);

    return AroResp(0, "OK" );

}

AroResp TracksGenerator::translateReferenceLine(const Polygon &boundary,
                                                const std::vector<double>& trackDistances,
                                                const std::vector<Point> & refLine,
                                                const Point & direction,
                                                bool refLineIsCenter,
                                                std::vector<TrackGeometries> & tracks)
{
    auto getTrackDistance = [](const std::vector<double>& trackDistances, int ind)->double{
        while(ind < 0)
            ind += trackDistances.size();
        return trackDistances.at( ind % trackDistances.size() );
    };

    auto computeTrackWidth = [](TrackGeometries& track){
        track.width = -1;
        if(track.center.empty())
            return;
        if(track.side1.size() < 2 && track.side2.size() < 2)
            return;

        double minDist = std::numeric_limits<double>::max();
        if(track.side1.size() >= 2){
            for(auto& p : track.center)
                minDist = std::min(minDist, geometry::calc_dist_to_linestring(track.side1, p, false));
        }
        if(track.side2.size() >= 2){
            for(auto& p : track.center)
                minDist = std::min(minDist, geometry::calc_dist_to_linestring(track.side2, p, false));
        }

        track.width = 2 * minDist;
    };

    tracks.clear();
    if( arolib::geometry::isPolygonValid(boundary) == arolib::geometry::PolygonValidity::INVALID_POLYGON )
        return AroResp(1, "The inner boundary of the subfield is not valid");
    if(refLine.size() < 2)
        return AroResp(1, "The reference line has less than two points");
    if(trackDistances.empty())
        return AroResp(1, "No track distances given.");
    for(auto & d : trackDistances){
        if(d <= 0)
            return AroResp(1, "Invalid track distance " + std::to_string(d));
    }

    //deflate a bit the inner boundary to avoid problems with location of points too close the original boundary (if the points are located over the boundary, they will be considered as out-of-the-boundary)
    Polygon boundaryEd;
    if( !arolib::geometry::offsetPolygon(boundary, boundaryEd, 0.001, false) ){
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error offseting the inner boundary. Trying again...");
        boundaryEd = boundary;
        Polygon polyTmp;
        if( !arolib::geometry::offsetPolygon(boundary, polyTmp, 0.1, false) || !arolib::geometry::offsetPolygon(polyTmp, boundaryEd, 0.099, true) ){
            m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error offseting the inner boundary (2nd try)");
            boundaryEd = boundary;
        }
    }
    double boundaryLength = geometry::getGeometryLength(boundary.points);

    std::vector<Point> refLine0 = refLine;//reference linestring for translation
    if(refLineIsCenter){//the given reference line corresponds to the center line of the track -> an initial translation of -distance/2 must be done
        double transDist = trackDistances.front();
        Point transDir = direction * -1;
        if( !arolib::geometry::setVectorLength(transDir, 0.5 * transDist) )
            return AroResp(1, "Invalid direction vector or translation distance");
        refLine0 = arolib::geometry::translate(refLine0, transDir);
    }

    for(int side = -1 ; side < 2 ; side += 2){
        int indDistance = side == 1 ? 0 : -1;
        auto refLineTmp = refLine0;

        while(true){//translate in the given direction until the translated track is outside the boundary
            TrackGeometries track;
            track.side1 = refLineTmp;

            //translate the (current) reference linestring
            double transDist = getTrackDistance(trackDistances, indDistance);
            indDistance += side;
            Point transDir = direction * side;
            if( !arolib::geometry::setVectorLength(transDir, transDist) )
                return AroResp(1, "Invalid direction vector or translation distance");
            track.side2 = arolib::geometry::translate(refLineTmp, transDir);

            if( !arolib::geometry::setVectorLength(transDir, 0.5 * transDist) )
                return AroResp(1, "Invalid direction vector or translation distance");
            track.center = arolib::geometry::translate(refLineTmp, transDir);

            if( !isTrackInBoundary(boundaryEd, track, boundaryLength) )
                break;

            computeTrackWidth(track);
            if(track.width <= 0)
                return AroResp(1, "Error computing track width");

            tracks.emplace_back(track);

            //update refLine1 for next iteration
            refLineTmp = track.side2;
        }

        if(side == -1)
            std::reverse(tracks.begin(), tracks.end());

    }

    return AroResp(0, "OK");

}


AroResp TracksGenerator::translateReferenceLine_old(Subfield &subfield,
                                                      double trackDistance,
                                                      const Linestring &refLine,
                                                      const Point &direction,
                                                      bool startAtRefLine,
                                                      bool checkLastTrack,
                                                      bool checkForSingleTrackCase)
{
    subfield.tracks.clear();
    if( arolib::geometry::isPolygonValid(subfield.boundary_inner) == arolib::geometry::PolygonValidity::INVALID_POLYGON )
        return AroResp(1, "The inner boundary of the subfield is not valid");
    if(refLine.points.size() < 2)
        return AroResp(1, "The reference line has less than two points");
    if(trackDistance == 0)
        return AroResp(1, "Invalid track distance.");

    Point dirPos = direction;// direction(+magnitude) vector for translation in 'positive/forward' direction
    if( !arolib::geometry::setVectorLength(dirPos, trackDistance) )
        return AroResp(1, "Invalid direction vector.");

    if(!startAtRefLine)//if the translation should NOT be done starting at the given reference line, set (temporarily) the length to half the given track distance (i.e. the first tracks will be generated at +/- trackDistance/2 from the given reference line)
        arolib::geometry::setVectorLength(dirPos, trackDistance*0.5);
    Point dirNeg = dirPos*-1;// direction(+magnitude) vector for translation in 'negative/backward' direction (oposite to dirPos)

    std::vector<Point> refPos = arolib::geometry::translate(refLine.points, dirNeg);//reference linestring for translation in 'positive/forward' direction
    std::vector<Point> refNeg;//reference linestring for translation in 'negative/backward' direction
    if(startAtRefLine)
        refNeg = refLine.points;
    else
        refNeg = arolib::geometry::translate(refLine.points, dirPos);

    //set again the magnitudes of the vectors to the desired trackDistance
    arolib::geometry::setVectorLength(dirPos, trackDistance);
    arolib::geometry::setVectorLength(dirNeg, trackDistance);

    //deflate a bit the inner boundary to avoid problems with location of points too close the original boundary (if the points are located over the boundary, they will be considered as out-of-the-boundary)
    Polygon boundaryEd;
    if( !arolib::geometry::offsetPolygon(subfield.boundary_inner, boundaryEd, 0.001, false) ){
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error offseting the inner boundary.");
        boundaryEd = subfield.boundary_inner;
    }


    //perform track translation/generation in 'negative/backward' direction
    bool normalCheck = true;
    while (true){//stop when the translated linestring does NOT lie partially or completelly inside the inner boundary
        if(normalCheck)//translate the (current) reference linestring normally
            refNeg = arolib::geometry::translate(refNeg, dirNeg);
        else{//translate the (current) reference linestring a bit on the opposite direction (for a last check)
            auto dirLast = dirPos;// direction(+magnitude) vector for the last translation (in this case in 'positive/forward' direction (direction dirPos) since the reference linestring in now the last translated linestring which is lying outside the inner boundary (i.e. we must reverse a bit))
            arolib::geometry::setVectorLength(dirLast, m_multWidth_checkLastTrack*trackDistance);
            refNeg = arolib::geometry::translate(refNeg, dirLast);
        }

//        if(!arolib::geometry::in_polygon(refNeg, subfield.boundary_inner, false))
//            break;

        //check if the new (translated) linestring intersects with (lies partially or completelly inside) the inner boundary
        if(!arolib::geometry::intersects(refNeg, boundaryEd, true, true)){
            if (!normalCheck || !checkLastTrack)//if no extra check must be done or it was already done in the last iteration, stop translating linestrings
                break;
            normalCheck = false;//make a (last) special check in the next iteration to see if an extra track is needed
            continue;
        }

        //make a second check with another method (sometimes the fists one fails)
        if(arolib::geometry::count_intersections(refNeg, boundaryEd, true, true) < 2){
            if (!normalCheck || !checkLastTrack)//if no extra check must be done or it was already done in the last iteration, stop translating linestrings
                break;
            normalCheck = false;//make a (last) special check in the next iteration to see if an extra track is needed
            continue;
        }

        //add translated linestring to tracks
        subfield.tracks.push_back( createTrack(subfield.boundary_inner, refNeg, {}, -1, trackDistance, Track::MAIN_IF) );


        if(!normalCheck)//no more linestrings to translate
            break;
    }

    std::reverse(subfield.tracks.begin(), subfield.tracks.end());

    //perform track translation/generation in 'positive/forward' direction
    normalCheck = true;
    while (true){
        if(normalCheck)//translate the (current) reference linestring normally
            refPos = arolib::geometry::translate(refPos, dirPos);
        else{//translate the (current) reference linestring a bit on the opposite direction (for a last check)
            auto dirLast = dirNeg;// direction(+magnitude) vector for the last translation (in this case in 'negative/backward' direction (direction dirNeg) since the reference linestring in now the last translated linestring which is lying outside the inner boundary (i.e. we must reverse a bit))
            arolib::geometry::setVectorLength(dirLast, m_multWidth_checkLastTrack*trackDistance);
            refPos = arolib::geometry::translate(refPos, dirLast);
        }

//        if(!arolib::geometry::in_polygon(refPos, subfield.boundary_inner, false))
//            break;

        //check if the new (translated) linestring intersects with (lies partially or completelly inside) the inner boundary
        if(!arolib::geometry::intersects(refPos, boundaryEd, true, true)){
            if (!normalCheck || !checkLastTrack)//if no extra check must be done or it was already done in the last iteration, stop translating linestrings
                break;
            normalCheck = false;//make a (last) special check in the next iteration to see if an extra track is needed
            continue;
        }

        //make a second check with another method (sometimes the fists one fails)
        if(arolib::geometry::count_intersections(refPos, boundaryEd, true, true) < 2){
            if (!normalCheck || !checkLastTrack)//if no extra check must be done or it was already done in the last iteration, stop translating linestrings
                break;
            normalCheck = false;//make a (last) special check in the next iteration to see if an extra track is needed
            continue;
        }

        //add translated linestring to tracks
        subfield.tracks.push_back( createTrack(subfield.boundary_inner, refPos, {}, -1, trackDistance, Track::MAIN_IF) );

        if(!normalCheck)//no more linestrings to translate
            break;
    }

    if(subfield.tracks.empty()){
        if(checkForSingleTrackCase){
            m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "No tracks were generated (maybe the inner boundary is too small). Attempting to generate one single track...");
            return translateReferenceLine_old(subfield,
                                          trackDistance*0.5,
                                          refLine,
                                          direction,
                                          startAtRefLine,
                                          checkLastTrack,
                                          false);
        }
        return AroResp(1, "No tracks were computed.");
    }

    return AroResp(0, "OK");

}

AroResp TracksGenerator::offsetReferenceLine(const Polygon &boundary,
                                             const std::vector<double> &trackWidths,
                                             const std::vector<Point> &refLine,
                                             bool refLineIsCenter,
                                             std::vector<TracksGenerator::TrackGeometries> &tracks)
{
    auto getTrackWidth = [](const std::vector<double>& trackWidths, int ind)->double{
        while(ind < 0)
            ind += trackWidths.size();
        return trackWidths.at( ind % trackWidths.size() );
    };


    tracks.clear();
    if( arolib::geometry::isPolygonValid(boundary) == arolib::geometry::PolygonValidity::INVALID_POLYGON )
        return AroResp(1, "The inner boundary of the subfield is not valid");
    if(refLine.size() < 2)
        return AroResp(1, "The reference line has less than two points");
    if(trackWidths.empty())
        return AroResp(1, "No track distances given.");
    for(auto & d : trackWidths){
        if(d <= 0)
            return AroResp(1, "Invalid track distance " + std::to_string(d));
    }

    //deflate a bit the inner boundary to avoid problems with location of points too close the original boundary (if the points are located over the boundary, they will be considered as out-of-the-boundary)
    Polygon boundaryEd;
    if( !arolib::geometry::offsetPolygon(boundary, boundaryEd, 0.001, false) ){
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error offseting the inner boundary.");
        boundaryEd = boundary;
    }
    double boundaryLength = geometry::getGeometryLength(boundary.points);

    std::vector<Point> refLine0 = refLine;//reference linestring

    //extend refLine in the sides
    geometry::extend_linestring(refLine0, boundaryLength, boundaryLength);

    if(refLineIsCenter){//the given reference line corresponds to the center line of the track -> an initial offset of -width/2 must be done
        auto refLineTmp = refLine0;
        if( !geometry::offsetLinestring(refLineTmp, refLine0, -trackWidths.front()) )
            return AroResp(1, "Error offseting the reference line half of the (1st) track width.");
    }


    for(int side = -1 ; side < 2 ; side += 2){
        int indWidth = side == 1 ? 0 : -1;
        auto refLineTmp = refLine0;

        while(true){//translate in the given direction until the translated track is outside the boundary
            TrackGeometries track;
            track.side1 = refLineTmp;

            //translate the (current) reference linestring
            double width = getTrackWidth(trackWidths, indWidth) * side;
            indWidth += side;
            if( !geometry::offsetLinestring(refLineTmp, track.side2, width) )
                return AroResp(1, "Error offseting the temporary reference line.");

            if( !geometry::offsetLinestring(refLineTmp, track.center, 0.5 * width) )
                return AroResp(1, "Error offseting the temporary reference line half of the track width.");

            if( !isTrackInBoundary(boundaryEd, track, boundaryLength) )
                break;

            track.width = std::fabs(width);
            tracks.emplace_back(track);

            //update refLine1 for next iteration
            refLineTmp = track.side2;
        }

        if(side == -1)
            std::reverse(tracks.begin(), tracks.end());

    }

    return AroResp(0, "OK");


}

AroResp TracksGenerator::offsetReferenceLine_old(Subfield &subfield,
                                                   double trackDistance,
                                                   const Linestring &_refLine,
                                                   bool startAtRefLine,
                                                   bool checkLastTrack,
                                                   bool checkForSingleTrackCase)
{

    subfield.tracks.clear();
    if( arolib::geometry::isPolygonValid(subfield.boundary_inner) == arolib::geometry::PolygonValidity::INVALID_POLYGON )
        return AroResp(1, "The inner boundary of the subfield is not valid");
    if(_refLine.points.size() < 2)
        return AroResp(1, "The reference line has less than two points");
    if(trackDistance == 0)
        return AroResp(1, "Invalid track distance.");

    Linestring refLine = _refLine;

    //extend first and last segment of the reference line/linestring (the extension distance is calculated based on the size of the subfield)
    double minX, maxX, minY, maxY;
    if ( !arolib::geometry::getPolygonLimits(subfield.boundary_inner, minX, maxX, minY, maxY) )
        return AroResp(1, "Error obtaining the limits of the inner boundary." );
    double extendDistance = arolib::geometry::calc_dist( Point(minX, minY), Point(maxX, maxY) )
                            * std::max( 1.0, std::fabs(trackDistance) );
    Point newPoint;
    newPoint = arolib::geometry::extend_line( refLine.points.at(1), refLine.points.at(0), extendDistance );
    refLine.points.insert(refLine.points.begin(), newPoint);
    newPoint = arolib::geometry::extend_line( refLine.points.at( refLine.points.size()-2 ), refLine.points.back(), extendDistance );
    refLine.points.push_back(newPoint);


    std::vector<Point> refPos, refNeg; //reference linestrings for translation in 'positive'/'negative' directions

    double initialOffset = trackDistance;
    if(!startAtRefLine)//if the translation should NOT be done starting at the given reference line, set the initla offset to be half the given track distance (i.e. the first tracks will be generated at +/- trackDistance/2 from the given reference line)
        initialOffset *= 0.5;

    //make initla offsets in positive and negative direction to obtain the initial refPos and refNeg
    if( !arolib::geometry::offsetLinestring(refLine.points, refPos, -initialOffset) )
            return AroResp(1, "Error offseting the reference line half of the track distance (negative direction).");

    if(startAtRefLine)
        refNeg = refLine.points;
    else{
        if( !arolib::geometry::offsetLinestring(refLine.points, refNeg, initialOffset) )
                return AroResp(1, "Error offseting the reference line (half of the track distance (positive direction).");
    }

    //perform track offsetting/generation in 'negative/backward' direction
    bool normalCheck = true;
    while (true){
        std::vector<Point> points;
        if(normalCheck){//offset the (current) reference linestring normally
            if( !arolib::geometry::offsetLinestring(refNeg, points, -trackDistance) )
                return AroResp(1, "Error offseting the reference (negative direction).");
        }
        else{//offset the (current) reference linestring a bit on the opposite direction (for a last check)
            if( !arolib::geometry::offsetLinestring(refNeg, points, m_multWidth_checkLastTrack*trackDistance) )// the offset distance for the last translation (in this case in 'positive' direction since the (current) reference linestring in now the last offset linestring which is lying outside the inner boundary (i.e. we must reverse a bit))
                return AroResp(1, "Error offseting the reference (positive direction - neg last).");
        }
        refNeg = points;

        //        if(!arolib::geometry::in_polygon(ls.points, subfield.boundary_inner, false))
        //            break;

        //check if the new (offset) linestring intersects with (lies partially or completelly inside) the inner boundary
        if(!arolib::geometry::intersects(points, subfield.boundary_inner, true, true)){
            if (!normalCheck || !checkLastTrack)//if no extra check must be done or it was already done in the last iteration, stop offsetting linestrings
                break;
            normalCheck = false;//make a (last) special check in the next iteration to see if an extra track is needed
            continue;
        }
        //make a second check with another method (sometimes the fists one fails)
        if(arolib::geometry::count_intersections(points, subfield.boundary_inner, true, true) < 2){
            if (!normalCheck || !checkLastTrack)//if no extra check must be done or it was already done in the last iteration, stop offsetting linestrings
                break;
            normalCheck = false;//make a (last) special check in the next iteration to see if an extra track is needed
            continue;
        }

        //add ofsfet linestring to tracks
        subfield.tracks.push_back( createTrack(subfield.boundary_inner, points, {}, -1, trackDistance, Track::MAIN_IF) );

        if(!normalCheck)//no more linestrings to offset
            break;
    }

    std::reverse(subfield.tracks.begin(), subfield.tracks.end());

    //perform trackoffsetting/generation in 'positive/forward' direction
    normalCheck = true;
    while (true){
        std::vector<Point> points;
        if(normalCheck){//offset the (current) reference linestring normally
            if( !arolib::geometry::offsetLinestring(refPos, points, trackDistance) )
                return AroResp(1, "Error offseting the reference (positive direction).");
        }
        else{//offset the (current) reference linestring a bit on the opposite direction (for a last check)
            if( !arolib::geometry::offsetLinestring(refPos, points, -m_multWidth_checkLastTrack*trackDistance) )// the offset distance for the last translation (in this case in 'negative' direction since the (current) reference linestring in now the last offset linestring which is lying outside the inner boundary (i.e. we must reverse a bit))
                return AroResp(1, "Error offseting the reference (negative direction - pos last).");
        }
        refPos = points;

        //        if(!arolib::geometry::in_polygon(ls.points, subfield.boundary_inner, false))
        //            break;

        //check if the new (offset) linestring intersects with (lies partially or completelly inside) the inner boundary
        if(!arolib::geometry::intersects(points, subfield.boundary_inner, true, true)){
            if (!normalCheck || !checkLastTrack)//if no extra check must be done or it was already done in the last iteration, stop offsetting linestrings
                break;
            normalCheck = false;//make a (last) special check in the next iteration to see if an extra track is needed
            continue;
        }
        //make a second check with another method (sometimes the fists one fails)
        if(arolib::geometry::count_intersections(points, subfield.boundary_inner, true, true) < 2){
            if (!normalCheck || !checkLastTrack)//if no extra check must be done or it was already done in the last iteration, stop offsetting linestrings
                break;
            normalCheck = false;//make a (last) special check in the next iteration to see if an extra track is needed
            continue;
        }

        //add ofsfet linestring to tracks
        subfield.tracks.push_back( createTrack(subfield.boundary_inner, points, {}, -1, trackDistance, Track::MAIN_IF) );

        if(!normalCheck)//no more linestrings to offset
            break;
    }

    if(subfield.tracks.empty()){
        if(checkForSingleTrackCase){
            m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "No tracks were generated (maybe the inner boundary is too small). Attempting to generate one single track...");
            return offsetReferenceLine_old(subfield,
                                       trackDistance*0.5,
                                       _refLine,
                                       startAtRefLine,
                                       checkLastTrack,
                                       false);
        }
        return AroResp(1, "No tracks were computed.");
    }

    return AroResp(0, "OK");

}

AroResp TracksGenerator::connectTracksToBoundary(const Polygon &boundary,
                                                 std::vector<TracksGenerator::TrackGeometries> &tracks,
                                                 bool onlyUntilBoundaryIntersection)
{
    auto addIntersectionPoints = [](std::vector<Point> & line, const std::vector<Point> & intersectionPoints){
        for(auto& p : intersectionPoints)
            geometry::addSampleToGeometryClosestToPoint(line, p);
    };

    auto removePointsOutsideBoundary = [](std::vector<Point> & line, const Polygon & boundary, int ind1 = -1, int ind2 = -1){
        int indRemove = 0;
        ind2 = line.size() - 1 - ind2;
        for(size_t i = 0 ; i < line.size() ; ++i){
            if( geometry::in_polygon( r_at(line, i), boundary ) || i == ind2 ){
                indRemove = line.size() - i;
                break;
            }
        }
        line.erase( line.begin()+indRemove, line.end() );

        indRemove = line.size();
        for(size_t i = 0 ; i < line.size() ; ++i){
            if( geometry::in_polygon( line.at(i), boundary ) || i == ind1 ){
                indRemove = i;
                break;
            }
        }
        line.erase( line.begin(), line.begin()+indRemove );
    };

    auto addSampleToLinestring = [](std::vector<Point>& ls, const Point& p, int& minInd, int& maxInd){
        auto size_prev = ls.size();
        int ind = geometry::addSampleToGeometryClosestToPoint(ls, p, 1);
        minInd = std::min(minInd, ind);
        if(size_prev == ls.size() || ind > maxInd)
            maxInd = std::max(maxInd, ind);
        else
            ++maxInd;
    };

    auto getNewCentralPoints = [](TrackGeometries& track)->bool{
        if( ( track.side1.empty() && track.side2.empty() ) || track.center.size() < 2 )
            return false;

        const std::vector<Point>& refSide = track.side1.empty() ? track.side2 : track.side1 ;
        std::vector<Point>& otherSide = track.side1.empty() ? track.side1 : track.side2 ;
        otherSide = track.center;
        track.width *= 0.5;

        bool ok = geometry::offsetLinestring(refSide, track.center, 0.5*track.width, false, 0)
                    && geometry::calc_dist( otherSide.front(), track.center.front() ) < geometry::calc_dist( refSide.front(), otherSide.front() );//@todo check if this actually works to check if the offset direction is the correct one

        if(!ok){//try again to the other side
            ok = geometry::offsetLinestring(refSide, track.center, -0.5*track.width, false, 0)
                    && geometry::calc_dist( otherSide.front(), track.center.front() ) < geometry::calc_dist( refSide.front(), otherSide.front() );//@todo check if this actually works to check if the offset direction is the correct one
        }


        return ok;
    };

    double extDist = geometry::getGeometryLength(boundary.points);

    //deflate a bit the inner boundary to avoid problems with location of points too close the original boundary (if the points are located over the boundary, they will be considered as out-of-the-boundary)
    Polygon boundaryEd;    
    if( !arolib::geometry::offsetPolygon(boundary, boundaryEd, 0.001, false) ){
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error offseting the inner boundary. Trying again...");
        boundaryEd = boundary;
        Polygon polyTmp;
        if( !arolib::geometry::offsetPolygon(boundary, polyTmp, 0.1, false) || !arolib::geometry::offsetPolygon(polyTmp, boundaryEd, 0.099, true) ){
            m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error offseting the inner boundary (2nd try)");
            boundaryEd = boundary;
        }
    }

    bool checkForIntermediateTrack = true;

    //for(auto& track : tracks){
    for(size_t i = 0 ; i < tracks.size() ; ++i){
        auto& track = tracks.at(i);

        bool checkForIntermediateTrack_tmp = checkForIntermediateTrack && (i == 0 || i == tracks.size()-1 );
        checkForIntermediateTrack = true;

        geometry::extend_linestring(track.side1, extDist, extDist);
        geometry::extend_linestring(track.side2, extDist, extDist);
        geometry::extend_linestring(track.center, extDist, extDist);

        auto intersectionPoints1 = geometry::get_intersection(track.side1, boundaryEd.points);
        auto intersectionPoints2 = geometry::get_intersection(track.side2, boundaryEd.points);
        auto intersectionPointsC = geometry::get_intersection(track.center, boundaryEd.points);

        //add intersection points to lines
        addIntersectionPoints(track.side1, intersectionPoints1);
        addIntersectionPoints(track.side2, intersectionPoints2);
        addIntersectionPoints(track.center, intersectionPointsC);

        //remove points in the side lines' extremas that lie outside the boundary
        if(intersectionPoints1.size() < 2)
            track.side1.clear();
        else
            removePointsOutsideBoundary(track.side1, boundary);

        if(intersectionPoints2.size() < 2)
            track.side2.clear();
        else
            removePointsOutsideBoundary(track.side2, boundary);

        if(onlyUntilBoundaryIntersection && intersectionPointsC.size() < 2){//should we do this also when onlyUntilBoundaryIntersection?
            if(!checkForIntermediateTrack_tmp || !getNewCentralPoints(track) ){
                tracks.erase( tracks.begin() + i );
            }
            else
                checkForIntermediateTrack = false;

            --i;
            continue;

        }

        //@todo if there is still something to cover, the central track should be replaced with the intermediate linestring between the current central track and the side that intersects


        if(track.side1.size() < 2 && track.side2.size() < 2){//special case
            if(intersectionPointsC.size() < 2){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid track geometries (no intersections with boundary)");
                return AroResp(1, "Invalid track geometries (no intersections with boundary)");
            }
            removePointsOutsideBoundary(track.center, boundary);
            if(track.center.size() < 2){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid track geometries (unexpected error)");
                return AroResp(1, "Invalid track geometries (unexpected error)");
            }
            continue;
        }

        //add samples perpendicular to the extrema of both line sides
        int ind1 = track.center.size();
        int ind2 = -1;
        if(!onlyUntilBoundaryIntersection){
            if(track.side1.size() >= 2){
                addSampleToLinestring(track.center, track.side1.front(), ind1, ind2);
                addSampleToLinestring(track.center, track.side1.back(), ind1, ind2);
            }
            if(track.side2.size() >= 2){
                addSampleToLinestring(track.center, track.side2.front(), ind1, ind2);
                addSampleToLinestring(track.center, track.side2.back(), ind1, ind2);
            }
        }

        auto centerLineTmp = track.center;

        //remove points in the central line extremas that lie outside the boundary
        removePointsOutsideBoundary(track.center, boundary, ind1, ind2);

        //@todo if the central line does not lie inside the bounday (or only ery little), the tracks should be cumputed so that the entral track is more or less in the "center" of the remaining area

        if(!track.center.empty())
            continue;
        else if(onlyUntilBoundaryIntersection){
            tracks.erase( tracks.begin() + i );
            --i;
            continue;
        }

        //shouldn't reach this case, oder?

        if(track.side1.empty() && track.side2.empty()){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No points remaining in track geometries");
            return AroResp(1, "No points remaining in track geometries");
        }

        track.center = centerLineTmp;
        std::vector<Point> refLine = track.side1.size() > 0 ? track.side1 : track.side2;

        int ind = geometry::addSampleToGeometryClosestToPoint(track.center, refLine.front(), 1);
        if(ind >= 0 && ind < track.center.size())
            track.center.erase( track.center.begin(), track.center.begin() + ind );

        ind = geometry::addSampleToGeometryClosestToPoint(track.center, refLine.back(), 1);
        if(ind >= 0 && ind < track.center.size())
            track.center.erase( track.center.begin() + ind + 1, track.center.end() );

        if( track.center.size() < 2 ){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Could not adjust central line from side line");
            return AroResp(1, "Could not adjust central line from side line");
        }

    }

    return AroResp::ok();

}

AroResp TracksGenerator::createTracks(std::vector<TracksGenerator::TrackGeometries> &tracks_geom, std::vector<Track> &tracks)
{

    tracks.resize( tracks_geom.size() );
    for(size_t i = 0 ; i < tracks.size() ; ++i){
        auto& track = tracks.at(i);
        auto& track_geom = tracks_geom.at(i);

        track = Track();

        if(track_geom.width <= 0){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid track width for track " + std::to_string(i));
            return AroResp(1, "Invalid track width for track " + std::to_string(i));
        }

        track.width = track_geom.width;
        track.points = track_geom.center;

        if(!geometry::offsetLinestring(track.points, track.boundary, 0.5 * track.width, 0.5 * track.width, true, 10)){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error computing boundary for track " + std::to_string(i));
            return AroResp(1, "Error computing boundary for track " + std::to_string(i));
        }

        geometry::correct_polygon(track.boundary);
    }

    return AroResp::ok();

}

AroResp TracksGenerator::removeEmptyTracks(const Polygon &boundary, std::vector<Track> &tracks, double areaThreshold)
{
    areaThreshold = std::min( 0.99, areaThreshold );
    if(areaThreshold < 1e-3)
        return AroResp::ok();

    for(size_t i = 0 ; i < tracks.size() ; ++i){
        const auto& trackPoints = tracks.at(i).points;
        const auto& trackBoundary = tracks.at(i).boundary;
        if(trackBoundary.points.empty())
            continue;

        if( geometry::get_intersection(trackPoints, boundary.points).size() > 1 )//the central linestring is inside the boundary, do not remove
            continue;

        double trackArea = geometry::calc_area(trackBoundary);
        if(trackArea <= 0)
            continue;
        auto intersectionPolys = geometry::get_intersection(boundary, trackBoundary);
        double intArea = 0;

        for(auto& poly : intersectionPolys)
            intArea += geometry::calc_area(poly);

        if(intArea / trackArea < areaThreshold){
            tracks.erase(tracks.begin()+i);
            --i;
        }

    }
    return AroResp::ok();
}

void TracksGenerator::orderTracks(std::vector<Track> &tracks, TrackOrderStrategy strategy)
{
    if(strategy == TrackOrderStrategy::EXTRA_TRACK_LAST){
        //check if there were extra tracks generated at the begining/end of the tracks
        bool disregardStrategy = true;
        if(tracks.size() > 2){
            double dist_0 = std::numeric_limits<double>::max();
            double dist_1 = dist_0;

            const auto points_0_0 = tracks.front().points;
            const auto points_0_1 = tracks.at(1).points;
            const auto points_n_0 = tracks.back().points;
            const auto points_n_1 = (tracks.end()-2)->points;

            for(size_t i = 0 ; i < points_0_0.size() ; ++i)
                dist_0 = std::min( dist_0, arolib::geometry::calc_dist_to_linestring(points_0_1, points_0_0.at(i), true) );

            for(size_t i = 0 ; i < points_n_0.size() ; ++i)
                dist_1 = std::min( dist_1, arolib::geometry::calc_dist_to_linestring(points_n_1, points_0_0.at(i), true) );

            if (std::fabs(dist_0 - dist_1) > 0.1 * std::max(dist_0, dist_1) ){//the are extra tracks --> order the tracks accordingly
                disregardStrategy = false;
                if(dist_1 > dist_0)
                    std::reverse(tracks.begin(), tracks.end());
            }
        }

        if(disregardStrategy)//no extra tracks --> use the default strategy in TracksGeneratorParameters
            strategy = TracksGeneratorParameters().trackOrderStrategy;
    }

    if(strategy == TrackOrderStrategy::LEFTMOST_TRACK_FIRST){
        Point l_first = arolib::geometry::getLeftmostPoint(tracks.front().points);
        Point r_first = arolib::geometry::getRightmostPoint(tracks.front().points);
        Point l_last = arolib::geometry::getLeftmostPoint(tracks.back().points);
        Point r_last = arolib::geometry::getRightmostPoint(tracks.back().points);

        if(l_last.x < l_first.x && r_last.x < r_first.x)
            std::reverse(tracks.begin(), tracks.end());
    }

    else if(strategy == TrackOrderStrategy::LONGEST_TRACK_FIRST){
        if( arolib::geometry::getGeometryLength(tracks.front().points) < arolib::geometry::getGeometryLength(tracks.back().points)  )
            std::reverse(tracks.begin(), tracks.end());
    }

    //remove tracks without points ad set the track ids
    for(size_t i = 0 ; i < tracks.size() ; ++i){
        if(tracks.at(i).points.size() < 2){
            tracks.erase( tracks.begin()+i );
            --i;
            continue;
        }
        tracks.at(i).id = i;
    }
}

AroResp TracksGenerator::connectTracksToInnerBoundary(Subfield &subfield)
{
    try{

        double tol = 0.0001;
        arolib::geometry::closePolygon(subfield.boundary_inner);
        for(size_t trackNr = 0 ; trackNr < subfield.tracks.size() ; ++trackNr){
            auto &track = subfield.tracks.at(trackNr);
            if(track.points.size() < 2)
                return AroResp(1, "Track " + std::to_string(trackNr) + " has less than 2 points");

            arolib::geometry::unsample_linestring(track.points, m_unsamplingTolerance);

            //extend first segment if necessary
            std::vector<Point> intersections;
            while(1){
                intersections = arolib::geometry::get_intersection(track.points.at(0),
                                                 track.points.at(1),
                                                 subfield.boundary_inner.points,
                                                 true,
                                                 true,
                                                 false,
                                                 tol);
                if(!intersections.empty() || track.points.size() == 2)
                    break;

                //remove the segment if it does not intersect with the inner boundary
                track.points.erase( track.points.begin() );
                if(track.points.size() < 2)
                    return AroResp(1, "Error obtaining the intersections of track " + std::to_string(trackNr) + " with the inner boundary (first segment).");
            }
            if(!intersections.empty()){
                if( arolib::geometry::getLocationInLine(track.points.at(0), track.points.at(1), intersections.front()/*, tol*/) == -1
                        || intersections.front() == track.points.at(0)){
                    track.points.insert( track.points.begin(),
                                         arolib::geometry::extend_line(track.points.at(1), intersections.front(), 10));
                }
            }
            else{
                intersections = arolib::geometry::get_intersection(track.points.at(0),
                                                 track.points.at(1),
                                                 subfield.boundary_inner.points,
                                                 true,
                                                 false,
                                                 true,
                                                 tol);
                if(intersections.empty())
                    return AroResp(1, "Error obtaining the intersections of track " + std::to_string(trackNr) + " with the inner boundary (first segment - second try).");
                track.points.at(0) = track.points.at(1);
                track.points.at(1) = intersections.front();
                track.points.push_back( arolib::geometry::extend_line(track.points.at(0), track.points.at(1), 1) );
            }

            //extend last segment if necessary
            while(1){
                intersections = arolib::geometry::get_intersection(track.points.back(),
                                                 track.points.at( track.points.size()-2 ),
                                                 subfield.boundary_inner.points,
                                                 true,
                                                 true,
                                                 false,
                                                 tol);
                if(!intersections.empty())
                    break;

                //remove the segment if it does not intersect with the inner boundary
                track.points.pop_back();
                if(track.points.size() < 2)
                    return AroResp(1, "Error obtaining the intersections of track " + std::to_string(trackNr) + " with the inner boundary (last segment).");
            }
            if( arolib::geometry::getLocationInLine(track.points.back(), track.points.at( track.points.size()-2 ), intersections.front()/*, tol*/) == -1
                    || intersections.front() == track.points.back()){
                track.points.push_back( arolib::geometry::extend_line(track.points.at( track.points.size()-2 ), intersections.front(), 10) );
            }

            std::vector<Point> &pointsOld = track.points;
            std::vector<Point> pointsNew;
            int countIntersections = 0;
            for(size_t i = 0 ; i+1 < pointsOld.size() ; ++i){
                intersections = arolib::geometry::get_intersection(pointsOld.at(i),
                                                 pointsOld.at(i+1),
                                                 subfield.boundary_inner.points,
                                                 true,
                                                 false,
                                                 false,
                                                 tol);
                if(i != 0)
                    pointsNew.push_back(pointsOld.at(i));
                if(intersections.empty())
                    continue;
                countIntersections += intersections.size();
                pointsNew.insert( pointsNew.end(), intersections.begin(), intersections.end() );

            }

            if(countIntersections < 2)
                return AroResp(1, "Error obtaining the intersections of track " + std::to_string(trackNr) + " with the inner boundary (" + std::to_string(countIntersections) + " found)");

            arolib::geometry::remove_repeated_points(pointsNew);
            track.points = pointsNew;
        }

        return AroResp(0, "OK");

    }
    catch(std::exception &e){
        return AroResp(1, std::string("Exception cought: ") + e.what());
    }
}

AroResp TracksGenerator::connectTracksToHeadland(Subfield &subfield, const arolib::geometry::HeadlandConnectionStrategy &strategy)
{
    arolib::geometry::openPolygon(subfield.headlands.complete.middle_track);
    std::vector<Point> headland = subfield.headlands.complete.middle_track.points;
    arolib::geometry::closePolygon(subfield.headlands.complete.middle_track);
    arolib::geometry::closePolygon(subfield.boundary_inner);
    arolib::geometry::closePolygon(subfield.boundary_outer);
    const auto& boundary = subfield.boundary_inner;

    bool addConnectionsToHeadland = false;

    for(size_t tr = 0 ; tr < subfield.tracks.size(); ++tr){
        auto &track = subfield.tracks.at(tr);
        if(track.points.size() < 2)
            return AroResp(1, "Track " + std::to_string(track.id) + " has less than 2 points");

        std::vector<Point> &pointsOld = track.points;
        std::vector<Point> pointsNew;

        for(size_t i = 0 ; i+1 < pointsOld.size() ; ++i){

            Point med = arolib::geometry::getCentroid(pointsOld.at(i), pointsOld.at(i+1));

            if(arolib::geometry::in_polygon(med, boundary)){//check if the segment centroid (middle point) is inside the inner boundary
                //add the point(s) to the (new) track
                if(pointsNew.empty())
                    pointsNew.push_back(pointsOld.at(i));
                pointsNew.push_back(pointsOld.at(i+1));
            }
            else{//the centroid is outside the inner boundary. we have to check if an intra-track connection through the headland is needed, or we can simply send directly the track as it is (because the part of the track lying outside the inner field goes over the headland anyway)

                if(pointsNew.size() < 2)
                    continue;

                //find the (next) segment whose centroid is inside the inner boundary
                size_t indEnd = 0;
                for(size_t j = i+1 ; j+1 < pointsOld.size() ; ++j){
                    med = arolib::geometry::getCentroid(pointsOld.at(j), pointsOld.at(j+1));
                    if(arolib::geometry::in_polygon(med, boundary)){
                        indEnd = j;
                        break;
                    }
                }

                if(indEnd == 0 || indEnd+1 == pointsOld.size())//the remaining part of the track is outside the inner boundary --> dont add more points to this track
                    break;

                //check if the segment of the track that is lying outside the inner boundary intersects with the outer boundary
                bool intersectsWithOB = false;
                for(size_t j = i ; j < indEnd ; ++j){
                    if(!arolib::geometry::get_intersection(pointsOld.at(j), pointsOld.at(j+1), subfield.boundary_outer.points).empty()){
                        intersectsWithOB = true;
                        break;
                    }
                }

                if( intersectsWithOB ){//--> we have to connect the two subsegments through the headland
                    if(!arolib::geometry::get_intersection(pointsOld.at(i), pointsOld.at(indEnd), subfield.boundary_outer.points).empty()){
                        std::vector<Point> connection;


                        if( !getConnectionToHeadland(headland,
                                                     pointsNew.at( pointsNew.size()-2 ),
                                                     pointsNew.back(),
                                                     pointsOld.at(indEnd),
                                                     pointsOld.at(indEnd+1),
                                                     connection,
                                                     strategy,
                                                     addConnectionsToHeadland,
                                                     m_unsamplingTolerance) )
                            return AroResp(1, "Error connecting internal segments of track " + std::to_string(track.id) + " through the headland");


                        //add connection through headland to the track
                        pointsNew.insert( pointsNew.end(), connection.begin(), connection.end() );
                    }
                }
                else////TODO should we still insert the points from the segments that is outside the inner boundary (for repeteability)? or simply connect directly with the next point inside the inner boundary? If the points should be inserted, this if is not necessary
                    pointsNew.insert( pointsNew.end(), pointsOld.begin()+i, pointsOld.begin()+indEnd );

                pointsNew.push_back(pointsOld.at(indEnd));
                i = indEnd-1;
            }

        }
        pointsOld = pointsNew;
        if(pointsOld.size() < 2)
            return AroResp(1, "Track " + std::to_string(track.id) + " has less than 2 points after processing");
    }

    return AroResp(0, "OK");
}

AroResp TracksGenerator::sampleTracks(std::vector<Track> &tracks, const TracksGenerator::TracksGeneratorParameters &settings, const Polygon& boundary){
    return sampleTracks1(tracks, settings, boundary);
}

AroResp TracksGenerator::sampleTracks1(std::vector<Track> &tracks, const TracksGenerator::TracksGeneratorParameters &settings, const Polygon& boundary)
{
    const double epsRes = 0.1;

    double resolution = settings.sampleResolution > 0 ? settings.sampleResolution : 0.5 * std::numeric_limits<double>::max();


    for(auto &track : tracks){
        arolib::geometry::unsample_linestring(track.points);
        auto intersectionPoints = geometry::get_intersection(boundary.points, track.points);
        for(auto& p : intersectionPoints)
            geometry::addSampleToGeometryClosestToPoint(track.points, p);
    }

    while(!tracks.empty() && tracks.front().points.size() < 2)
        tracks.erase( tracks.begin() );

    if(tracks.empty())
        return AroResp(1, "Tracks are empty");

    if(settings.trackSamplingStrategy == START_AT_TRACK_START){//simple sampling
        for(auto &track : tracks){
            track.points = arolib::geometry::sample_geometry(track.points, resolution);
        }
        return AroResp(0, "OK");
    }

    bool fwd = arolib::geometry::getGeometryLength(tracks.front().points) >= arolib::geometry::getGeometryLength(tracks.back().points);

    Point pivot = tracks.front().points.front();
    Point tip = tracks.front().points.at(1);
    auto trackId_base = tracks.front().id;
    if(!fwd){//if the last track is longer that the first one, take it as reference/base
        pivot = tracks.back().points.front();
        tip = tracks.back().points.at(1);
        trackId_base = tracks.back().id;
    }
    Point pPerp = arolib::geometry::rotate( pivot, tip, 90, true );

    //the line (pivot, pPerp) is a line perpendicular to the first segment/edge of the reference/base track.
    //the intersections of this line with the other tracks will be used as reference point to sample the intersected tracks

    for(auto &track : tracks){
        if(trackId_base == track.id){//if it is the reference/base track, make a simple sampling
            track.points = arolib::geometry::sample_geometry(track.points, resolution);
            continue;
        }

        //check if the line (pivot, pPerp) intersects the current track. If the line intersects the track, use the intersection as reference point for sampling
        auto intersections = arolib::geometry::get_intersection(pivot, pPerp, track.points, true, true, true);
        if(intersections.empty()){//if the line does not intersect the track...

            //get the intersection of line (pivot, pPerp) with the extended track (first and last segments/edges of the track are 'extended infinitelly' to obtain the intersection points)
            Point pInt1, pInt2;
            bool int1OK, int2OK = false;
            int1OK = arolib::geometry::get_intersection(pivot, pPerp,
                                      track.points.front(), track.points.at(1),
                                      pInt1, true, true);
            if(track.points.size() > 2)
                int2OK = arolib::geometry::get_intersection(pivot, pPerp,
                                          track.points.back(), *(track.points.end()-2),
                                          pInt2, true, true);

            if(!int1OK && !int2OK){//if still no intersection are found, make a simple sampling
                m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Unable to find intersection if track with id " + std::to_string(track.id)
                                                                 + " with the reference line perpendicular to track " + std::to_string(trackId_base));
                track.points = arolib::geometry::sample_geometry(track.points, resolution);
                continue;
            }

            //check which one of the intersections is suitable to be used to obtain the reference point for sampling, and calculate the distance from this point to the track (the point is in the extended section, hence not part of the track)
            double distToTrack;
            if(int1OK && int2OK){
                double d1 = arolib::geometry::calc_dist_to_line(track.points.front(), track.points.at(1), pInt1, false);
                double d2 = arolib::geometry::calc_dist_to_line(track.points.back(), *(track.points.end()-2), pInt2, false);
                if(d1 > d2)
                    int1OK = false;
                else
                    int2OK = false;
                distToTrack = std::min(d1, d2);
            }
            else if (int1OK)
                distToTrack = arolib::geometry::calc_dist_to_line(track.points.front(), track.points.at(1), pInt1, false);
            else
                distToTrack = arolib::geometry::calc_dist_to_line(track.points.back(), *(track.points.end()-2), pInt2, false);

            double deltaDist = std::fmod(distToTrack, resolution);
            deltaDist = resolution - deltaDist;

            if( std::fabs(deltaDist) >= arolib::geometry::getGeometryLength(track.points) )
                continue;

            //obtain the reference point for sampling based on the suitable intersection point and its distance to the track, and add it to the previously empty intersections (holding the reference point for sampling)
            std::vector<Point> segTemp;
            if(int1OK)
                segTemp = {track.points.front(), track.points.at(1)};
            else
                segTemp = {track.points.back(), *(track.points.end()-2)};

            Point pVec;
            if(!arolib::geometry::getParallelVector(segTemp.front(), segTemp.at(1), pVec, deltaDist)){
                m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Unable to obtain parallel vector to first/last secment.");
                track.points = arolib::geometry::sample_geometry(track.points, resolution);
                continue;
            }
            intersections.push_back( segTemp.front() + pVec );
        }

        auto pInt = intersections.front();//reference point for sampling. unless it is a very unusual reference linestring, there should be only one intersection

        //get the index of the first point of the segment where the reference point for sampling (pInt) belongs (i.e. the point is somewhere between this point and the next)
        size_t ind = 0;
        for(;ind+1 < track.points.size() ; ++ind){
            if(arolib::geometry::checkPointInLine( track.points.at(ind),
                                 track.points.at(ind+1),
                                 pInt) )//pInt in the line composed by points[ind] and points[ind+1] (i.e. between the 2 points)
                break;
        }

        //divide the track in two segments (points before and after the reference point for sampling (pInt))

        //obtain the segment of points located in the track before pInt, reverse them and sample them
        std::vector<Point> pointsBack ( track.points.begin(), track.points.begin()+ind+1 );
        pointsBack.emplace_back(pInt);
        std::reverse(pointsBack.begin(), pointsBack.end());
        pointsBack = arolib::geometry::sample_geometry(pointsBack, resolution);
        for(size_t i = 0 ; i+2 < pointsBack.size() ; ++i){
            if( arolib::geometry::calc_dist(pointsBack.at(i), pointsBack.at(i+1)) < epsRes * resolution )
                pointsBack.erase( pointsBack.begin() + i+1 );
        }

        //obtain the segment of points located in the track after pInt, and sample them
        std::vector<Point> pointsFront = {pInt};
        pointsFront.insert(pointsFront.end(), track.points.begin()+ind+1, track.points.end() );
        pointsFront = arolib::geometry::sample_geometry(pointsFront, resolution);
        for(size_t i = 0 ; i+2 < pointsFront.size() ; ++i){
            //remove second-from-last point if the length of the last segment is too low
            if( arolib::geometry::calc_dist(pointsFront.at(i), pointsFront.at(i+1)) < epsRes * resolution )
                pointsFront.erase( pointsFront.begin() + i+1 );
        }

        //update the track points with the combination of the sampled previous/backwards segment (reversed) and the next/forward segment
        track.points.clear();
        track.points.insert(track.points.end(), pointsBack.rbegin(), pointsBack.rend()-1);
        track.points.insert(track.points.end(), pointsFront.begin(), pointsFront.end());

        //remove second and/or second-from-last points if the length of the first and last segment is too low
        if(track.points.size() > 2
                && arolib::geometry::calc_dist(track.points.front(), track.points.at(1)) < epsRes * resolution)
            track.points.erase( track.points.begin()+1 );
        if(track.points.size() > 2
                && arolib::geometry::calc_dist(track.points.back(), *(track.points.end()-2) ) < epsRes * resolution)
            track.points.erase( (track.points.end()-2) );
    }
    return AroResp(0, "OK");

}

AroResp TracksGenerator::sampleTracks2(std::vector<Track> &tracks, const TracksGenerator::TracksGeneratorParameters &settings, const Polygon& boundary)
{
    const double epsRes = 0.1;

    for(auto &track : tracks){
        arolib::geometry::unsample_linestring(track.points);
        auto intersectionPoints = geometry::get_intersection(boundary.points, track.points);
        for(auto& p : intersectionPoints)
            geometry::addSampleToGeometryClosestToPoint(track.points, p);
    }

    while(!tracks.empty() && tracks.front().points.size() < 2)
        tracks.erase( tracks.begin() );

    if(tracks.empty())
        return AroResp(1, "Tracks are empty");

    if( settings.sampleResolution <= 0)
        return AroResp(0, "OK");


    //double resolution = settings.sampleResolution > 0 ? settings.sampleResolution : 0.5 * std::numeric_limits<double>::max();
    double resolution = settings.sampleResolution;

    if(settings.trackSamplingStrategy == START_AT_TRACK_START){//simple sampling
        for(auto &track : tracks){
            track.points = arolib::geometry::sample_geometry(track.points, resolution);
        }
    }

    bool inv = geometry::getGeometryLength(tracks.front().points) < geometry::getGeometryLength(tracks.back().points);

    for(int dir = 0 ; dir < 2 ; ++dir){
        for(size_t i = 0 ; i+1 < tracks.size() ; ++i){
            auto& refTrack = ((bool)dir == inv) ? tracks.at(i) : r_at(tracks, i);

            if(i == 0 && dir == 0)
                refTrack.points = geometry::sample_geometry(refTrack.points, resolution, epsRes * resolution);
            else{
                refTrack.points = geometry::sample_geometry_ends(refTrack.points, resolution, epsRes * resolution);
                refTrack.points = geometry::sample_geometry(refTrack.points, resolution, 0.5 * resolution);
            }


            for(size_t k = 0 ; k < refTrack.points.size() ; ++k){
                double dist1 = k > 0 ? geometry::calc_dist(refTrack.points.at(k), refTrack.points.at(k-1)) : 2*resolution;
                double dist2 = k+1 < refTrack.points.size() ? geometry::calc_dist(refTrack.points.at(k), refTrack.points.at(k+1)) : 2*resolution;
                if(dist1 < 0.95 * resolution || dist2 < 0.95 * resolution)
                    continue;

                for(size_t j = i+1 ; j < tracks.size() ; ++j){
                    auto& track = ((bool)dir == inv) ? tracks.at(j) : r_at(tracks, j);
                    geometry::addSampleToGeometryClosestToPoint(track.points, refTrack.points.at(k), 1, 0.95 * settings.sampleResolution);
                }
            }



        }
    }

    return AroResp(0, "OK");
}

AroResp TracksGenerator::removeHarvestedTracks(Subfield &subfield,
                                                     double workingWidth,
                                                     const ArolibGrid_t &yieldMap,
                                                     const ArolibGrid_t &remainingAreaMap)
{
    for( size_t i = 0, originalIndex = 0 ; i < subfield.tracks.size() ; ++i, ++originalIndex){
        if(isTrackHarvested(subfield.tracks.at(i).points,
                       workingWidth,
                       yieldMap,
                       remainingAreaMap)){
            m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Removing track " + std::to_string(originalIndex));
            subfield.tracks.erase( subfield.tracks.begin()+i );
            i--;
        }
    }
    if(subfield.tracks.empty())
        return AroResp(1, "No tracks remaining");

    return AroResp(0, "OK");
}

bool TracksGenerator::isTrackHarvested( const std::vector<Point> &points,
                                   double workingWidth,
                                   const ArolibGrid_t &yieldMap,
                                   const ArolibGrid_t &remainingAreaMap)
{
    const double yieldThreshold = 1e-3;
    const double remainingAreaThreshold = 1e-2;

    if(points.size() < 2)
        return true;

    if(!yieldMap.isAllocated() && remainingAreaMap.isAllocated())
        return false;

    Polygon poly;
    if( !arolib::geometry::createPolygon(poly, points, workingWidth) )
        return true;

    bool errorTmp;
    if(remainingAreaMap.isAllocated()){
        double remainingArea = remainingAreaMap.getPolygonComputedValue( poly,
                                                                         ArolibGrid_t::AVERAGE_TOTAL,
                                                                         true,
                                                                         &errorTmp );

        if (errorTmp || remainingArea < remainingAreaThreshold)
            return true;
    }

    if(yieldMap.isAllocated()){
        double yield = yieldMap.getPolygonComputedValue( poly,
                                                         ArolibGrid_t::AVERAGE_TOTAL,
                                                         true,
                                                         &errorTmp );

        if (errorTmp || yield < yieldThreshold)
            return true;
    }

    return false;
}

void TracksGenerator::translateAll(Subfield &subfield, const size_t &refLineIndex, const Point &vec)
{
    if(vec.x == 0 && vec.y == 0)
        return;

    subfield.boundary_inner.points = arolib::geometry::translate(subfield.boundary_inner.points, vec);
    subfield.boundary_outer.points = arolib::geometry::translate(subfield.boundary_outer.points, vec);
    subfield.headlands.complete.middle_track.points = arolib::geometry::translate(subfield.headlands.complete.middle_track.points, vec);
    subfield.reference_lines.at(refLineIndex).points = arolib::geometry::translate(subfield.reference_lines.at(refLineIndex).points, vec);

    for (auto &track : subfield.tracks)
        track.points = arolib::geometry::translate(track.points, vec);

}

bool TracksGenerator::isTrackInBoundary(const Polygon &boundary, TracksGenerator::TrackGeometries track, double extDist)
{
    bool usePolygonIntersection = true;
    if(extDist < 0)
        extDist = geometry::getGeometryLength(boundary.points);

    if(usePolygonIntersection && extDist > 1e-6){
        geometry::extend_linestring(track.side1, extDist, extDist);
        geometry::extend_linestring(track.side2, extDist, extDist);
        geometry::extend_linestring(track.center, extDist, extDist);
    }

    //check using polygon intersection
    if(usePolygonIntersection){
        Polygon trackPoly;
        trackPoly.points = track.side1;
        trackPoly.points.insert( trackPoly.points.end(), track.side2.rbegin(), track.side2.rend());
        geometry::correct_polygon(trackPoly);
        return geometry::intersects(boundary, trackPoly);
    }

    //check intersections of the track geometries with the boundary
    if( !geometry::intersects(track.side1, boundary, true, true)
            && !geometry::intersects(track.side2, boundary, true, true)
            && !geometry::intersects(track.center, boundary, true, true) )
        return false;

    //make a second check with another method (sometimes the fists one fails)
    if(geometry::count_intersections(track.side1, boundary, true, true) < 2
            && geometry::count_intersections(track.side2, boundary, true, true) < 2
            && geometry::count_intersections(track.center, boundary, true, true) < 2)
        return false;

    return true;
}

Track TracksGenerator::createTrack(const Polygon &boundary, const std::vector<Point> &side1, const std::vector<Point> &side2, double res, double width, Track::TrackType type)
{
    Track track;
    track.type = type;
    track.width = width;

    //@todo temporary solution
    track.points.reserve(side1.size());
    for(auto & p : side1)
        track.points.emplace_back(p);
//    if(!track.points.empty())
//        track.points.front().type = track.points.back().type = TrackPoint::TP_BOUNDARY_INTERSECTION;

    return track;
}

}
