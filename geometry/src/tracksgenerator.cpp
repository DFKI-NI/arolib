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
 
#include "arolib/geometry/tracksgenerator.h"

#include <chrono>

namespace arolib {

namespace geometry {

const double TracksGenerator::m_unsampleAngTol = deg2rad(0.1);

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

    int shiftingStrategy, trackOrderStrategy, trackSamplingStrategy;
    std::map<std::string, double*> dMap = { {"trackDistance" , &tmp.trackDistance},
                                            {"sampleResolution" , &tmp.sampleResolution},
                                            {"direction.x" , &tmp.direction.x},
                                            {"direction.y" , &tmp.direction.y},
                                            {"trackRelativeAreaThreshold" , &tmp.trackRelativeAreaThreshold},
                                            {"trackAreaThreshold" , &tmp.trackAreaThreshold} };
    std::map<std::string, bool*> bMap = { {"checkForRemainingTracks" , &tmp.checkForRemainingTracks},
                                          {"useRefLineAsCentralLine" , &tmp.useRefLineAsCentralLine},
                                          {"onlyUntilBoundaryIntersection" , &tmp.onlyUntilBoundaryIntersection},
                                          {"splitBoundaryIntersectingTracks" , &tmp.splitBoundaryIntersectingTracks} };
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
    ret["trackRelativeAreaThreshold"] = double2string( params.trackRelativeAreaThreshold );
    ret["trackAreaThreshold"] = double2string( params.trackAreaThreshold );
    ret["checkForRemainingTracks"] = std::to_string( params.checkForRemainingTracks );
    ret["useRefLineAsCentralLine"] = std::to_string( params.useRefLineAsCentralLine );
    ret["onlyUntilBoundaryIntersection"] = std::to_string( params.onlyUntilBoundaryIntersection );
    ret["splitBoundaryIntersectingTracks"] = std::to_string( params.splitBoundaryIntersectingTracks );
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
                                        const std::vector<double> &trackWidths,
                                        const TracksGenerator::TracksGeneratorParameters &settings,
                                        std::vector<Track> &tracks,
                                        std::vector<size_t> *pTrackWidthsIndexes)
{
    std::vector<size_t> trackDistanceIndexesTmp;
    std::vector<size_t>& trackDistanceIndexes = pTrackWidthsIndexes ? *pTrackWidthsIndexes : trackDistanceIndexesTmp;

    try {
        AroResp compResp;
        tracks.clear();

        geometry::correct_polygon(boundary);
        if( arolib::geometry::isPolygonValid(boundary) == arolib::geometry::PolygonValidity::INVALID_POLYGON )
            return AroResp(1, "The boundary is not valid");
        if(trackWidths.empty())
            return AroResp(1, "No distances given.");
        for(auto & d : trackWidths){
            if(d <= 0)
                return AroResp(1, "Invalid distance " + std::to_string(d));
        }
        geometry::unsample_linestring(refLine);
        if(refLine.size() < 2)
            return AroResp(1, "The (unsampled) reference line has less than two points");

        bool refLineIsCenter = settings.useRefLineAsCentralLine;

        //check if the reference line lies partially or completelly inside the inner boundary of the subfield (if so, use it as it is; otherwise translate it to a suitable position)
        {
            Polygon boundary_tmp;
            if(!geometry::offsetPolygon(boundary, boundary_tmp, 1e-3, true))
                boundary_tmp = boundary;
            if( !geometry::in_polygon(refLine, boundary_tmp, false) && !geometry::intersects(refLine, boundary_tmp, false, false) ){
                logger().printOut(LogLevel::INFO, __FUNCTION__, "None of the points of the reference line lie within the inner boundary of the subfield, nor the reference line intersects with the inner boundary. The reference line will be translated to the best fitting position.");
                compResp = relocateReferenceLine(boundary, refLine, refLine);
                if( compResp.isError() ){
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error translating the reference line: " + compResp.msg);
                    return AroResp(1, "Error translating the reference line: " + compResp.msg );
                }
                refLineIsCenter = false;
            }
        }

        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Creating track geometries...");
        compResp = generateTracks(boundary,
                                  trackWidths,
                                  trackDistanceIndexes,
                                  refLine,
                                  refLineIsCenter,
                                  settings,
                                  tracks);
        if( compResp.isError() ){
            tracks.clear();
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating track geometries: " + compResp.msg);
            return AroResp(1, "Error creating track geometries: " + compResp.msg );
        }

        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Removing 'overlapping' tracks...");
        compResp = removeOverlappingTracks(boundary, tracks, trackDistanceIndexes, settings, settings.trackRelativeAreaThreshold, settings.trackAreaThreshold);
        if( compResp.isError() ){
            tracks.clear();
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error removing 'overlapping' tracks: " + compResp.msg);
            return AroResp(1, "Error removing 'overlapping' tracks: " + compResp.msg );
        }

        if( tracks.empty() ){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "No tracks were generated");
            return AroResp(1, "No tracks were generated" );
        }

        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Ordering tracks...");
        orderTracks(tracks, trackDistanceIndexes, settings.trackOrderStrategy);

        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Sampling tracks...");
        compResp = sampleTracks(tracks, trackDistanceIndexes, settings, boundary);
        if( compResp.errorID > 0 ){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error sampling tracks: " + compResp.msg);
            return AroResp(1, "Error sampling tracks: " + compResp.msg );
        }

        return AroResp(0, "OK" );
    }
    catch (std::exception &e) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what() );
        return AroResp(1, e.what() );
    }

}

AroResp TracksGenerator::generateTracks(Polygon boundary, std::vector<Point> refLine, const TracksGeneratorParameters &tracksGeneratorParameters, std::vector<Track> &tracks)
{
    return generateTracks(boundary,
                          refLine,
                          {tracksGeneratorParameters.trackDistance},
                          tracksGeneratorParameters,
                          tracks);
}

AroResp TracksGenerator::generateTracks(Field &field,
                                        const TracksGeneratorParameters &settings,
                                        const std::vector<size_t> &referenceLineIndexes) {


    try {
        if (settings.trackDistance == 0) {
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

            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Shifting subfield " + std::to_string(i) + "...");
            AroResp compResp = generateTracks(sf,
                                              settings,
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

AroResp TracksGenerator::generateTracks(Subfield &subfield,
                                              const TracksGeneratorParameters &settings,
                                              const size_t &referenceLineIndex)
{

    if(subfield.reference_lines.size() <=  referenceLineIndex) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid reference line index '" + std::to_string(referenceLineIndex) + " : Nr. of reference lines is " + std::to_string(subfield.reference_lines.size()));
        return AroResp(1, "Invalid reference line index" );
    }
    return generateTracks(subfield.boundary_inner, subfield.reference_lines.at(referenceLineIndex).points, {settings.trackDistance}, settings, subfield.tracks);

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

    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "indRef = " + std::to_string(indRef) +
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

AroResp TracksGenerator::generateTracks(const Polygon &boundary,
                                        const std::vector<double> &trackWidths,
                                        std::vector<size_t> &trackWidthsIndexes,
                                        std::vector<Point> refLine,
                                        bool refLineIsCenter,
                                        const TracksGeneratorParameters &settings,
                                        std::vector<Track> &tracks)
{
    tracks.clear();
    trackWidthsIndexes.clear();
    AroResp aroResp(1, "No tracks");

    Point direction = settings.direction;
    if(settings.shiftingStrategy == ShiftingStrategy::TRANSLATE_TRACKS){
        if(refLine.size() == 2)
            direction = Point(0,0);
        std::string directionStr = "";
        if(direction == Point(0,0))
            directionStr = " (calculated from the normal vectors to the reference line segments) ";

        if(direction == Point(0,0)){
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Calculating the direction of track shifting based on the normal vectors to the reference line segments... ");
            if( !arolib::geometry::getNormVector(refLine, direction) ){
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error calculating the direction of track shifting based on the normal vectors to the reference line segments");
                return AroResp(1, "Error calculating the direction of track shifting based on the normal vectors to the reference line segments" );
            }

        }
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Translating reference line with a track distance of " + std::to_string(settings.trackDistance) + " m" +
                 " and direction " + direction.toString() + directionStr);
    }

    if(!refLineIsCenter){
        std::vector<Point> ls;
        if(settings.shiftingStrategy == ShiftingStrategy::TRANSLATE_TRACKS)
            aroResp = translateAndExtendReferenceLine(boundary,
                                                      refLine,
                                                      0.5 * std::fabs(trackWidths.front()),
                                                      direction,
                                                      -1,
                                                      ls);
        else
            aroResp = offesetAndExtendReferenceLine(boundary,
                                                    refLine,
                                                    0.5 * std::fabs(trackWidths.front()),
                                                    std::fabs(trackWidths.front()),
                                                    ls);

        if(aroResp.isError() || ls.size() < 2)
            return AroResp(1, "Error during initial translation/offeset of reference line");
        refLine = ls;
    }

    double boundaryLength = getGeometryLength(boundary.points);
    int side = 0;
    size_t distInd = 0;
    double distTotal = 0;
    std::vector<Track> sideTracks;
    std::vector<size_t> sideTrackDistanceIndexes;
    while(side > -2){
        std::vector<Point> ls;
        std::vector<Track> subTracks;

        size_t distIndNext = get_index_from_cyclic_container(trackWidths, distInd + side);
        double dist = side * 0.5 * ( std::fabs(trackWidths.at(distInd)) + std::fabs(trackWidths.at(distIndNext)) );

        if(settings.shiftingStrategy == ShiftingStrategy::TRANSLATE_TRACKS)
            aroResp = translateAndExtendReferenceLine(boundary,
                                                      refLine,
                                                      dist + distTotal,
                                                      direction,
                                                      boundaryLength,
                                                      ls);
        else
            aroResp = offesetAndExtendReferenceLine(boundary,
                                                    refLine,
                                                    dist + distTotal,
                                                    boundaryLength,
                                                    ls);
        distTotal += dist;

        if(aroResp.isError() || ls.size() < 2)
            return AroResp(1, "Error during translation/offeset of reference line (side = " + std::to_string(side) + ")");

        aroResp = createTracksFromReferenceLine(boundary,
                                                ls,
                                                std::fabs(trackWidths.at(distInd)),
                                                settings,
                                                subTracks);

        if(aroResp.isError())
            return AroResp(1, "Error creating track(s) for reference line (side = " + std::to_string(side) + "): " + aroResp.msg);

        //try sorting tracks so that the order corresponds to the points order of the ref line
        if(subTracks.size() > 1){
            auto subTracksTmp = subTracks;
            std::multimap<double, size_t> tracksSorted;
            for(size_t j = 0 ; j < subTracks.size(); ++j){
                auto& track = subTracks.at(j);
                auto ls_tmp = ls;
                if(side < 0){//they will be reversed later
                    int ind = addSampleToGeometryClosestToPoint(ls_tmp, track.points.back());
                    if(ind < 0 || ind >= ls_tmp.size())
                        return AroResp(1, "Error sorting sub-tracks");
                    tracksSorted.insert( std::make_pair( getGeometryLength(ls_tmp, ind, -1) , j ) );
                }
                else{
                    int ind = addSampleToGeometryClosestToPoint(ls_tmp, track.points.front());
                    if(ind < 0 || ind >= ls_tmp.size())
                        return AroResp(1, "Error sorting sub-tracks");
                    tracksSorted.insert( std::make_pair( getGeometryLength(ls_tmp, 0, ind) , j ) );
                }
            }
            subTracks.clear();
            for(auto& track_it : tracksSorted){
                subTracks.emplace_back( subTracksTmp.at(track_it.second) );
            }
        }

        sideTracks.insert(sideTracks.end(), subTracks.begin(), subTracks.end());
        for(size_t j = 0 ; j < subTracks.size() ; ++j)
            sideTrackDistanceIndexes.push_back(distInd);
        distInd = distIndNext;

        if(side == 0 || subTracks.empty()){
            distTotal = 0;
            if(side < 0){
                tracks.insert(tracks.begin(), sideTracks.rbegin(), sideTracks.rend());
                trackWidthsIndexes.insert(trackWidthsIndexes.begin(), sideTrackDistanceIndexes.rbegin(), sideTrackDistanceIndexes.rend());
            }
            else{
                tracks.insert(tracks.end(), sideTracks.begin(), sideTracks.end());
                trackWidthsIndexes.insert(trackWidthsIndexes.end(), sideTrackDistanceIndexes.begin(), sideTrackDistanceIndexes.end());
            }

            sideTracks.clear();
            sideTrackDistanceIndexes.clear();
            if(side == 0)
                side = 1;
            else
                side -= 2;
            distInd = get_index_from_cyclic_container(trackWidths, side);
            continue;
        }
    }

    return aroResp;

}

void TracksGenerator::removeLinestringPointsToFitBoundaries(const std::vector<const Polygon *> &boundaries, std::vector<Point> &ls, bool fromFront){
    if(ls.empty())
        return;

    //        int ind_sel = fromFront ? ls.size() : -1;
    //        Point p_sel = Point::invalidPoint();
    //        auto ls_tmp = ls;
    //        for(auto boundary : boundaries){
    //            for(auto& pt: boundary->points){
    //                auto sizePrev = ls_tmp.size();
    //                int ind = addSampleToGeometryClosestToPoint(ls_tmp, pt, 1);
    //                if(ind < 0 || ind >= ls_tmp.size())
    //                    continue;
    //                if(ls_tmp.size() > sizePrev && ind <= ind_sel)
    //                    ind_sel++;;
    //                if( fromFront ? ind <= ind_sel : ind > ind_sel ){
    //                    ind_sel = ind;
    //                    p_sel = pt;
    //                }
    //            }
    //        }
    //        if(!p_sel.isValid()){
    //            ls.clear();
    //            return;
    //        }

    //        int ind = addSampleToGeometryClosestToPoint(ls, p_sel, 1);
    //        if(fromFront && ind > 0)
    //            ls.erase(ls.begin(), ls.begin()+ind);
    //        else if(!fromFront && ind < ls.size()-1)
    //            ls.erase(ls.begin()+ind+1, ls.end());


    double minDist = std::numeric_limits<double>::max();
    Point p_min = Point::invalidPoint();
    for(auto boundary : boundaries){
        for(auto& pt: boundary->points){
            auto ls_tmp = ls;
            int ind = addSampleToGeometryClosestToPoint(ls_tmp, pt, 1);
            if(ind < 0 || ind >= ls_tmp.size())
                continue;
            auto dist = fromFront ? getGeometryLength(ls_tmp, 0, ind) : getGeometryLength(ls_tmp, ind, -1);
            if(minDist > dist){
                minDist = dist;
                p_min = pt;
            }
        }
    }
    if(!p_min.isValid()){
        ls.clear();
        return;
    }

    int ind = addSampleToGeometryClosestToPoint(ls, p_min, 1);
    if(fromFront && ind > 0)
        ls.erase(ls.begin(), ls.begin()+ind);
    else if(!fromFront && ind < ls.size()-1)
        ls.erase(ls.begin()+ind+1, ls.end());
}

AroResp TracksGenerator::translateAndExtendReferenceLine(const Polygon &boundary,
                                                         const std::vector<Point> &refLine,
                                                         double distance,
                                                         const Point &direction,
                                                         double distExtension,
                                                         std::vector<Point> &ls)
{
    auto refLineTmp = refLine;
    unsample_linestring(refLineTmp);
    if(refLineTmp.size() < 2)
        return AroResp(1, "Invalide reference line");

    if(intersects(refLineTmp))
        return AroResp(1, "Reference line self intersects");

    distExtension = std::fabs(distExtension);
    if (distExtension > 1e-6){
        if(!extend_linestring(refLineTmp, distExtension, 0, false))
            return AroResp(1, "Error extending linestring (1)");
        removeLinestringPointsToFitBoundaries({&boundary}, refLineTmp, true);
        if(refLineTmp.size() < 2)
            return AroResp(1, "Error adjusting extended linestring (1)");

        if(!extend_linestring(refLineTmp, 0, distExtension, false))
            return AroResp(1, "Error extending linestring (2)");
        removeLinestringPointsToFitBoundaries({&boundary}, refLineTmp, false);
        if(refLineTmp.size() < 2)
            return AroResp(1, "Error adjusting extended linestring (2)");

        if(intersects(refLineTmp)){
            Point p_int;
            if( !get_intersection(refLineTmp.front(), refLineTmp.at(1), refLineTmp.back(), r_at(refLineTmp, 1), p_int) )
                return AroResp(1, "Extended linestring self intersects");
            refLineTmp.front() = p_int;
            refLineTmp.back() = p_int;
        }
    }

    if (std::fabs(distance) > 1e-6){
        Point transDir = direction;
        if(distance < 0)
            transDir = direction * -1;
        if( !arolib::geometry::setVectorLength(transDir, std::fabs(distance)) )
            return AroResp(1, "Invalid direction vector or translation distance");
        ls = arolib::geometry::translate(refLineTmp, transDir);
        if(ls.size() < 2)
            return AroResp(1, "Error translating linestring");
    }
    else
        ls = refLineTmp;

    return AroResp::ok();

}

AroResp TracksGenerator::offesetAndExtendReferenceLine(const Polygon &boundary,
                                                       const std::vector<Point> &refLine,
                                                       double distance,
                                                       double distExtension,
                                                       std::vector<Point> &ls)
{
    auto refLineTmp = refLine;
    unsample_linestring(refLineTmp);
    if(refLineTmp.size() < 2)
        return AroResp(1, "Invalide reference line");

    if(intersects(refLineTmp))
        return AroResp(1, "Reference line self intersects");

    distExtension = std::fabs(distExtension);
    if (distExtension > 1e-6){
        if(!extend_linestring(refLineTmp, distExtension, 0, false))
            return AroResp(1, "Error extending linestring (1)");
        removeLinestringPointsToFitBoundaries({&boundary}, refLineTmp, true);
        if(refLineTmp.size() < 2)
            return AroResp(1, "Error adjusting extended linestring (1)");

        if(!extend_linestring(refLineTmp, 0, distExtension, false))
            return AroResp(1, "Error extending linestring (2)");
        removeLinestringPointsToFitBoundaries({&boundary}, refLineTmp, false);
        if(refLineTmp.size() < 2)
            return AroResp(1, "Error adjusting extended linestring (2)");

        if(intersects(refLineTmp)){
            Point p_int;
            if( !get_intersection(refLineTmp.front(), refLineTmp.at(1), refLineTmp.back(), r_at(refLineTmp, 1), p_int) )
                return AroResp(1, "Extended linestring self intersects");
            refLineTmp.front() = p_int;
            refLineTmp.back() = p_int;
        }
    }

    if (std::fabs(distance) > 1e-6){
        if( !geometry::offsetLinestring(refLineTmp, ls, distance) || ls.size() < 2 )
            return AroResp(1, "Error offseting reference line.");
        unsample_linestring(ls, 0.1);
        if(intersects(ls)){
            Point p_int;
            if( !get_intersection(ls.front(), ls.at(1), ls.back(), r_at(ls, 1), p_int) )
                return AroResp(1, "Offset linestring self intersects");
            ls.front() = p_int;
            ls.back() = p_int;
        }
    }
    else
        ls = refLineTmp;

    return AroResp::ok();
}

AroResp TracksGenerator::createTracksFromReferenceLine(const Polygon &boundary,
                                                       const std::vector<Point> &refLine,
                                                       double width,
                                                       const TracksGeneratorParameters &settings,
                                                       std::vector<Track> &tracks)
{
    tracks.clear();

    PolygonWithHoles polyTrack0;

    auto adjustLinestringUntilIntersection = [](const std::vector<const Polygon*> &boundaries, std::vector<Point>& ls, bool fromFront){
        if(ls.empty())
            return;

        double minDist = std::numeric_limits<double>::max();
        Point p_min = Point::invalidPoint();
        for(auto boundary : boundaries){
            std::vector<Point> intersections = get_intersection(ls, boundary->points);
            for(auto& p_int: intersections){
                auto ls_tmp = ls;
                int ind = addSampleToGeometryClosestToPoint(ls_tmp, p_int, 1);
                if(ind < 0 || ind >= ls_tmp.size())
                    continue;
                auto dist = fromFront ? getGeometryLength(ls_tmp, 0, ind) : getGeometryLength(ls_tmp, ind, -1);
                if(minDist > dist){
                    minDist = dist;
                    p_min = p_int;
                }
            }
        }
        if(!p_min.isValid()){
            ls.clear();
            return;
        }

        int ind = addSampleToGeometryClosestToPoint(ls, p_min, 1);
        if(fromFront && ind > 0)
            ls.erase(ls.begin(), ls.begin()+ind);
        else if(!fromFront && ind < ls.size()-1)
            ls.erase(ls.begin()+ind+1, ls.end());
    };


    auto tracksTmp = tracks;

    //first compute tracks spliting them to see if some segments must be removed
    double trackRelativeAreaThreshold = std::max(0.0, std::min(0.99, settings.trackRelativeAreaThreshold) );

    std::vector<const Polygon*> boundaries;
    bool okOffset = geometry::offsetLinestring(refLine, polyTrack0, 0.5*width, 0.5*width, true);
    if( !okOffset || polyTrack0.outer.points.size() < 4 )
        return AroResp(1, "Error offseting reference line to get polygon");
    auto intersectionPolys = get_intersection(boundary, polyTrack0);
    if(intersectionPolys.empty()) //track is outside the boundary
        return AroResp::ok();
    intersectionPolys = get_intersection(boundary, polyTrack0);
    for(auto& poly : intersectionPolys){
        tracksTmp.push_back(Track());
        auto& track = tracksTmp.back();
        track.points = refLine;
        if(settings.onlyUntilBoundaryIntersection){
            adjustLinestringUntilIntersection({&poly.outer}, track.points, true);
            adjustLinestringUntilIntersection({&poly.outer}, track.points, false);
        }
        else{
            removeLinestringPointsToFitBoundaries({&poly.outer}, track.points, true);
            removeLinestringPointsToFitBoundaries({&poly.outer}, track.points, false);
        }
        if(track.points.size() < 2){
            tracksTmp.pop_back();
            continue;
        }

        if(settings.trackAreaThreshold < 1e-6 && trackRelativeAreaThreshold < 1e-6){
            boundaries.push_back(&poly.outer);
            continue;
        }

        PolygonWithHoles trackPoly;
        bool okOffset = geometry::offsetLinestring(track.points, trackPoly, 0.5*width, 0.5*width, true);
        if( !okOffset || trackPoly.outer.points.size() < 4 )
            return AroResp(1, "Error offseting adjuested reference line to get polygon");

        double trackArea = calc_area(trackPoly);
        double validArea = calc_area( get_intersection(boundary, trackPoly) );

        if( trackArea < 1e-9 ||
                ( settings.trackAreaThreshold > 1e-6 && validArea < settings.trackAreaThreshold ) ||
                ( trackRelativeAreaThreshold > 1e-6 && validArea/trackArea < trackRelativeAreaThreshold )){
            tracksTmp.pop_back();
            continue;
        }

        boundaries.push_back(&poly.outer);
    }

    if(boundaries.empty()) //no valid tracks
        return AroResp::ok();


    if(settings.onlyUntilBoundaryIntersection){
        tracks.push_back(Track());
        auto& track = tracks.back();
        track.points = refLine;
        adjustLinestringUntilIntersection(boundaries, track.points, true);
        adjustLinestringUntilIntersection(boundaries, track.points, false);
        if(track.points.size() < 2){
            tracks.pop_back();
            return AroResp::ok();
        }
    }
    else{
        if(!settings.splitBoundaryIntersectingTracks){
            tracks.push_back(Track());
            auto& track = tracks.back();
            track.points = refLine;
            removeLinestringPointsToFitBoundaries(boundaries, track.points, true);
            removeLinestringPointsToFitBoundaries(boundaries, track.points, false);
            if(track.points.size() < 2){
                tracks.pop_back();
                return AroResp::ok();
            }
        }
        else{
            tracks = tracksTmp; //the work was done already
        }
    }

    for(auto& track : tracks){
        if( !geometry::offsetLinestring(track.points, track.boundary, 0.5*width, 0.5*width, true) || track.boundary.points.size() < 4 )
            return AroResp(1, "Error obtaining track boundary");
        track.type = Track::MAIN_IF;
        track.width = width;

        //add samples corresponding to the intersections with the boundary
        auto intersections = get_intersection(track.points, boundary.points);
        for(auto p : intersections)
            addSampleToGeometryClosestToPoint(track.points, p, 1);
    }

    return AroResp::ok();

}

AroResp TracksGenerator::removeOverlappingTracks(const Polygon &boundary, std::vector<Track> &tracks, std::vector<size_t> &trackDistanceIndexes, const TracksGeneratorParameters &settings, double areaRelativeThreshold, double areaThreshold)
{
    areaRelativeThreshold = std::min( 0.99, areaRelativeThreshold );
    if(areaRelativeThreshold < 1e-3 || tracks.size() < 2)
        return AroResp::ok();

    for(int side = 0 ; side < 2 ; ++side){
        if(tracks.size() < 2)
            return AroResp::ok();

        auto& track = side == 0 ? tracks.front() : tracks.back();
        auto& trackNext = side == 0 ? tracks.at(1) : r_at(tracks, 1);
        bool remove = false;

        if(geometry::intersects(track.points, trackNext.points))
            remove = true;
        else{
            double areaTrack = geometry::calc_area(track.boundary);//area of the track
            remove = true;

            auto boundaryIntersections = geometry::get_intersection(track.boundary, boundary);//intersection(s) of the track with the boundary
            if(boundaryIntersections.size() == 1){
                double areaIntersectionBoundary = geometry::calc_area(boundaryIntersections.front());//area of the intersection of the track with the boundary
                double areaIntersectionTrack = 0;//area of the intersection of the track with the boundary AND with the next track
                auto trackIntersections = geometry::get_intersection(boundaryIntersections.front(), trackNext.boundary);//intersection(s) of the intersection with the next track
                for(auto &poly2 : trackIntersections)
                    areaIntersectionTrack += geometry::calc_area(poly2);
                double validTrackArea = areaIntersectionBoundary - areaIntersectionTrack;
                remove = validTrackArea / areaTrack < areaRelativeThreshold;
            }
            else if (boundaryIntersections.size() > 1){
                double areaThresholdLocal = std::max( areaThreshold, getAreaThresholdForMultipleIntersections(track.width, settings.sampleResolution) );
                double areaIntersectionBoundarySum = 0;//area of the intersection of the track with the boundary
                double areaIntersectionTrackSum = 0;//area of the intersection of the track with the boundary AND with the next track
                for(auto& poly : boundaryIntersections){//special case where the track is divided
                    double areaIntersectionBoundary = geometry::calc_area(poly);
                    double areaIntersectionTrack = 0;//area of the intersection of the track with the boundary AND with the next track
                    auto trackIntersections = geometry::get_intersection(boundaryIntersections.front(), trackNext.boundary);//intersection(s) of the intersection with the next track
                    for(auto &poly2 : trackIntersections)
                        areaIntersectionTrack += geometry::calc_area(poly2);
                    double validTrackArea = areaIntersectionBoundary - areaIntersectionTrack;
                    if(validTrackArea > areaThresholdLocal){//@todo: if the track is intersected by the boundary (i.e. there exists more than one intersection polys), it might happen that part of the track must be remove, but not all. (see also removeEmptyTracks)
                        remove = false; //workaround: at least one part of the track has some intersection area -> do not remove
                        break;
                    }
                    areaIntersectionBoundarySum += areaIntersectionBoundary;
                    areaIntersectionTrackSum += areaIntersectionTrack;
                }
                if(remove){//just in case, check the sum
                    double validTrackArea = areaIntersectionBoundarySum - areaIntersectionTrackSum;
                    remove = validTrackArea / areaTrack < areaRelativeThreshold;
                }
            }
        }

        if(remove){
            if(side == 0){
                pop_front(tracks);
                pop_front(trackDistanceIndexes);
            }
            else{
                tracks.pop_back();
                trackDistanceIndexes.pop_back();
            }
        }
    }

    return AroResp::ok();
}

void TracksGenerator::orderTracks(std::vector<Track> &tracks, std::vector<size_t> &trackDistanceIndexes, TrackOrderStrategy strategy)
{
    if(tracks.size() < 2)
        return;

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
                if(dist_1 > dist_0){
                    std::reverse(tracks.begin(), tracks.end());
                    std::reverse(trackDistanceIndexes.begin(), trackDistanceIndexes.end());
                }
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

        if(l_last.x < l_first.x && r_last.x < r_first.x){
            std::reverse(tracks.begin(), tracks.end());
            std::reverse(trackDistanceIndexes.begin(), trackDistanceIndexes.end());
        }
    }

    else if(strategy == TrackOrderStrategy::LONGEST_TRACK_FIRST){
        if( arolib::geometry::getGeometryLength(tracks.front().points) < arolib::geometry::getGeometryLength(tracks.back().points)  ){
            std::reverse(tracks.begin(), tracks.end());
            std::reverse(trackDistanceIndexes.begin(), trackDistanceIndexes.end());
        }
    }

    //remove tracks without points and set the track ids
    for(size_t i = 0 ; i < tracks.size() ; ++i){
        if(tracks.at(i).points.size() < 2){
            tracks.erase( tracks.begin()+i );
            trackDistanceIndexes.erase( trackDistanceIndexes.begin()+i );
            --i;
            continue;
        }
        tracks.at(i).id = i;
    }
}

AroResp TracksGenerator::sampleTracks(std::vector<Track> &tracks, std::vector<size_t> &trackDistanceIndexes, const TracksGenerator::TracksGeneratorParameters &settings, const Polygon& boundary){
    return sampleTracks1(tracks, trackDistanceIndexes, settings, boundary);
}

AroResp TracksGenerator::sampleTracks1(std::vector<Track> &tracks, std::vector<size_t> &trackDistanceIndexes, const TracksGenerator::TracksGeneratorParameters &settings, const Polygon& boundary)
{
    const double epsRes = 0.1;

    double resolution = settings.sampleResolution > 0 ? settings.sampleResolution : 0.5 * std::numeric_limits<double>::max();


    for(auto &track : tracks){//unsample and add intersections with boundary
        arolib::geometry::unsample_linestring(track.points, 0.1*track.width, m_unsampleAngTol);
        auto intersectionPoints = geometry::get_intersection(boundary.points, track.points);
        for(auto& p : intersectionPoints)
            geometry::addSampleToGeometryClosestToPoint(track.points, p);
    }

    for(size_t i = 0 ; i < tracks.size() ; ++i){//remove invalid tracks
        if(tracks.at(i).points.size() < 2){
            tracks.erase( tracks.begin()+i );
            trackDistanceIndexes.erase( trackDistanceIndexes.begin()+i );
            --i;
        }
    }

    if(tracks.empty())
        return AroResp(1, "Tracks are empty");

    if(settings.trackSamplingStrategy == START_AT_TRACK_START){//simple sampling
        for(auto &track : tracks)
            sampleTrackPoints(track.points, resolution, true);
        return AroResp(0, "OK");
    }

    std::multimap<double, size_t, std::greater<double>> tracksSortedLength;
    for(size_t i = 0 ; i < tracks.size() ; ++i)
        tracksSortedLength.insert( std::make_pair(geometry::getGeometryLength(tracks.at(i).points), i) );

    std::map<size_t, std::set<size_t> > adjacencyList = getAdjacentTracksList(tracks, true);
    std::set<size_t> sampledTracks;

    int indRef = -1;
    for(auto it_t : tracksSortedLength){
        auto& trackInd = it_t.second;

        size_t sizePrev = tracks.at(trackInd).points.size();
        sampleTrackPoints(tracks.at(trackInd).points, resolution, true);
        sampledTracks.insert(trackInd);

        auto it_adj = adjacencyList.find(trackInd);
        if(it_adj == adjacencyList.end())
            continue;
        if(it_adj->second.empty())
            continue;

        if(indRef < 0){
            indRef = trackInd;
            sizePrev = 0;
        }

        if( tracks.at(trackInd).points.size() != sizePrev ){
            std::set<size_t> preSampled;
            preSampleAdjacentTracks(tracks, trackInd, sampledTracks, {}, preSampled, adjacencyList, resolution);
        }
    }

    //sample remaining tracks if any
    for(size_t i = 0 ; i < tracks.size() ; ++i){
        if(sampledTracks.find(i) == sampledTracks.end())
            sampleTrackPoints(tracks.at(i).points, resolution, true);
    }

    return AroResp(0, "OK");

}

AroResp TracksGenerator::sampleTracks2(std::vector<Track> &tracks, std::vector<size_t> &trackDistanceIndexes, const TracksGenerator::TracksGeneratorParameters &settings, const Polygon& boundary)
{
    const double epsRes = 0.1;

    double resolution = settings.sampleResolution > 0 ? settings.sampleResolution : 0.5 * std::numeric_limits<double>::max();


    for(auto &track : tracks){
        arolib::geometry::unsample_linestring(track.points, 0.1*track.width, m_unsampleAngTol);
        auto intersectionPoints = geometry::get_intersection(boundary.points, track.points);
        for(auto& p : intersectionPoints)
            geometry::addSampleToGeometryClosestToPoint(track.points, p);
    }

    while(!tracks.empty() && tracks.front().points.size() < 2){
        tracks.erase( tracks.begin() );
        trackDistanceIndexes.erase( trackDistanceIndexes.begin() );
    }

    if(tracks.empty())
        return AroResp(1, "Tracks are empty");

    if(settings.trackSamplingStrategy == START_AT_TRACK_START){//simple sampling
        for(auto &track : tracks)
            sampleTrackPoints(track.points, resolution, true);
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
            sampleTrackPoints(track.points, resolution);
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
            int1OK &= geometry::calc_dist(pInt1, track.points.front()) < geometry::calc_dist(pInt1, track.points.at(1));
            if(track.points.size() > 2){
                int2OK = arolib::geometry::get_intersection(pivot, pPerp,
                                                            track.points.back(), r_at(track.points, 1),
                                                            pInt2, true, true);
                int2OK &= geometry::calc_dist(pInt1, track.points.back()) < geometry::calc_dist(pInt1, r_at(track.points, 1));
            }

            if(!int1OK && !int2OK){//if still no intersection are found, make a simple sampling
                logger().printOut(LogLevel::WARNING, __FUNCTION__, "Unable to find intersection if track with id " + std::to_string(track.id)
                                                                 + " with the reference line perpendicular to track " + std::to_string(trackId_base));
                sampleTrackPoints(track.points, resolution);
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
                segTemp = {track.points.back(), r_at(track.points, 1)};

            if(deltaDist < geometry::getGeometryLength(segTemp)){

                Point pVec;
                if(!arolib::geometry::getParallelVector(segTemp.front(), segTemp.at(1), pVec, deltaDist)){
                    logger().printOut(LogLevel::WARNING, __FUNCTION__, "Unable to obtain parallel vector to first/last secment.");
                    sampleTrackPoints(track.points, resolution);
                    continue;
                }
                intersections.push_back( segTemp.front() + pVec );
            }

        }

        //get the index of the first point of the segment where the reference point for sampling (pInt) belongs (i.e. the point is somewhere between this point and the next)
        int ind = -1;
        Point pInt;
        if(!intersections.empty()){
            pInt = intersections.front();//reference point for sampling. unless it is a very unusual reference linestring, there should be only one intersection

            for(size_t i = 0; i+1 < track.points.size() ; ++i){
                if(arolib::geometry::checkPointInLine( track.points.at(i),
                                                       track.points.at(i+1),
                                                       pInt) ){//pInt in the line composed by points[ind] and points[ind+1] (i.e. between the 2 points)
                    ind = i;
                    break;
                }
            }

        }
        if(ind >= 0){

            //divide the track in two segments (points before and after the reference point for sampling (pInt))

            //obtain the segment of points located in the track before pInt, reverse them and sample them
            std::vector<Point> pointsBack ( track.points.begin(), track.points.begin()+ind+1 );
            pointsBack.emplace_back(pInt);
            std::reverse(pointsBack.begin(), pointsBack.end());
            sampleTrackPoints(pointsBack, resolution);
            for(size_t i = 0 ; i+2 < pointsBack.size() ; ++i){
                if( arolib::geometry::calc_dist(pointsBack.at(i), pointsBack.at(i+1)) < epsRes * resolution )
                    pointsBack.erase( pointsBack.begin() + i+1 );
            }

            //obtain the segment of points located in the track after pInt, and sample them
            std::vector<Point> pointsFront = {pInt};
            pointsFront.insert(pointsFront.end(), track.points.begin()+ind+1, track.points.end() );
            sampleTrackPoints(pointsFront, resolution);
            for(size_t i = 0 ; i+2 < pointsFront.size() ; ++i){
                //remove second-from-last point if the length of the last segment is too low
                if( arolib::geometry::calc_dist(pointsFront.at(i), pointsFront.at(i+1)) < epsRes * resolution )
                    pointsFront.erase( pointsFront.begin() + i+1 );
            }

            //update the track points with the combination of the sampled previous/backwards segment (reversed) and the next/forward segment
            track.points.clear();
            track.points.insert(track.points.end(), pointsBack.rbegin(), pointsBack.rend()-1);
            track.points.insert(track.points.end(), pointsFront.begin(), pointsFront.end());
        }
        else{//no intersection or for some reason the intersection was not right -> normal sampling
            sampleTrackPoints(track.points, resolution);
        }

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

AroResp TracksGenerator::sampleTracks3(std::vector<Track> &tracks, std::vector<size_t> &trackDistanceIndexes, const TracksGenerator::TracksGeneratorParameters &settings, const Polygon& boundary)
{
    const double epsRes = 0.1;

    for(auto &track : tracks){
        arolib::geometry::unsample_linestring(track.points, 0.1 * track.width, m_unsampleAngTol);
        auto intersectionPoints = geometry::get_intersection(boundary.points, track.points);
        for(auto& p : intersectionPoints)
            geometry::addSampleToGeometryClosestToPoint(track.points, p);
    }

    while(!tracks.empty() && tracks.front().points.size() < 2){
        tracks.erase( tracks.begin() );
        trackDistanceIndexes.erase( trackDistanceIndexes.begin() );
    }

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

void TracksGenerator::preSampleAdjacentTracks(std::vector<Track> &tracks,
                                              size_t indRefTrack,
                                              const std::set<size_t>& exclude,
                                              const std::set<size_t>& parents,
                                              std::set<size_t>& preSampled,
                                              const std::map<size_t, std::set<size_t> > &adjacencyList,
                                              double resolution)
{
    auto it_adj = adjacencyList.find(indRefTrack);
    if(it_adj == adjacencyList.end())
        return;

    std::multimap<double, size_t, std::greater<double>> sortedAdj;

    for(auto indAdj : it_adj->second){
        if(indAdj == indRefTrack
                || exclude.find(indAdj) != exclude.end()
                || parents.find(indAdj) != parents.end())
            continue;
        if(preSampled.find(indAdj) == preSampled.end()){
            sortedAdj.insert( std::make_pair( getGeometryLength(tracks.at(indAdj).points), indAdj ) );
            preSampled.insert(indAdj);
        }

        auto& refPts = tracks.at(indRefTrack).points;
        for(size_t i = 0 ; i < tracks.at(indRefTrack).points.size() ; ++i){
            double d1 = 0, d2 = 0;
            if(i > 0)
                d1 = calc_dist( refPts.at(i), refPts.at(i-1) );
            if(i+1 < refPts.size())
                d2 = calc_dist( refPts.at(i), refPts.at(i+1) );
            if(d1 > 0.8 * resolution || d2 > 0.8 * resolution)
                addSampleToGeometryClosestToPoint( tracks.at(indAdj).points, refPts.at(i), 1, 0.2*resolution );
        }
    }

    auto parentsNew = parents;
    parentsNew.insert(indRefTrack);
    for(auto& it : sortedAdj){
        auto& trackInd = it.second;
        preSampleAdjacentTracks(tracks,
                                trackInd,
                                exclude,
                                parentsNew,
                                preSampled,
                                adjacencyList,
                                resolution);
    }
}

void TracksGenerator::sampleTrackPoints(std::vector<Point> &points, double resolution, bool bisectSegment)
{
    const double minDist = 0.2 * resolution;
    double removeTH = ( arolib::geometry::getGeometryLength(points) > resolution ? 0.1 * resolution : 1e-5 );
    points = arolib::geometry::sample_geometry(points, resolution, minDist, false, bisectSegment);
    arolib::geometry::remove_repeated_points(points, true, removeTH);
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
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Removing track " + std::to_string(originalIndex));
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

double TracksGenerator::getAreaThresholdForMultipleIntersections(double trackWidth, double res)
{
    if(res < 1e-6)
        return 0.5 * trackWidth * trackWidth;
    return std::min(res, 0.5 * trackWidth) * trackWidth;
}

}

}
