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
 

#include "arolib/planning/track_connectors/infieldtracksconnectordef.hpp"

namespace arolib{

namespace ag = arolib::geometry;


InfieldTracksConnectorDef::InfieldTracksConnectorDef(std::shared_ptr<Logger> parentLogger)
    : IInfieldTracksConnector(__FUNCTION__, parentLogger)
{

}

std::vector<Point> InfieldTracksConnectorDef::getConnection(const Machine &machine,
                                                            const Pose2D &_pose_start,
                                                            const Pose2D &_pose_end,
                                                            double turningRad,
                                                            const std::pair<double, double> &extraDist,
                                                            const Polygon &_limitBoundary,
                                                            const Polygon &infieldBoundary,
                                                            const Headlands &headlands,
                                                            double speed_start,
                                                            double maxConnectionLength) const
{
    std::vector<Point> ret;

    auto limitBoundary = _limitBoundary;
    if(ag::isPolygonValid(limitBoundary) != ag::PolygonValidity::VALID_CLOSED_CW)
        limitBoundary.points.clear();

    //get turning rad for computations
    turningRad = getTurningRad(machine, turningRad);

    //adjust start/end poses
    Pose2D pose_start, pose_end;
    getExtendedPoses(_pose_start, _pose_end, extraDist, pose_start, pose_end);

    if( !limitBoundary.points.empty() ){
        if(turningRad > 1e-9){
            auto polyTmp = limitBoundary;
            if(!ag::offsetPolygon(polyTmp, limitBoundary, 0.01*turningRad, true))
                limitBoundary = polyTmp;
        }
        if( !ag::in_polygon(pose_start, limitBoundary) || !ag::in_polygon(pose_end, limitBoundary) ){
            logger().printError(__FUNCTION__, "(extended) start and/or end poses lie outside the limit boundary");
            return ret;
        }
    }

    if(!checkForMinPossibleLength(pose_start, pose_end, turningRad, limitBoundary, infieldBoundary, maxConnectionLength))
        return ret;

    if(turningRad > 1e-9){
        //try connecting directly using dubins path
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Trying to connect directly using dubins path...");
        ret = InfieldTracksConnectorDubins::getDubinsPathConnection(pose_start, pose_end, turningRad, {&limitBoundary}, infieldBoundary, maxConnectionLength, false, nullptr);
        if(!ret.empty()){
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Connected directly using dubins path..");
            return ret;
        }
//        ret = InfieldTracksConnectorDubins::getDubinsPathConnection(pose_start, pose_end, turningRad, {&limitBoundary}, infieldBoundary, maxConnectionLength, true, nullptr);
//        if(!ret.empty()){
//            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Connected directly using dubins path (2)..");
//            return ret;
//        }
    }
    else{
        //try connecting directly
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Trying to connect directly...");
        if( isPathValid({pose_start.point(), pose_end.point()}, limitBoundary, infieldBoundary, maxConnectionLength) ){
            ret.emplace_back(pose_start.point());
            ret.emplace_back(pose_end.point());
            return ret;
        }
    }

    if(!headlands.complete.tracks.empty() || !headlands.complete.middle_track.points.empty()){//try connecting over complete/surrounding headland
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Trying to connect over complete/surrounding headland...");
        ret = getConnectionOverHeadland(headlands.complete,
                                        pose_start,
                                        pose_end,
                                        turningRad,
                                        limitBoundary,
                                        infieldBoundary,
                                        maxConnectionLength);

        if(!ret.empty()){
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Connected over complete/surrounding headland");
            return ret;
        }
    }

    if(!headlands.partial.empty()){//try connecting over partial headlands
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Trying to connect over partial headlands...");
        ret = getConnectionOverHeadland(headlands.partial,
                                        pose_start,
                                        pose_end,
                                        machine,
                                        turningRad,
                                        limitBoundary,
                                        infieldBoundary,
                                        maxConnectionLength);

        if(!ret.empty()){
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Connected over partial headlands");
            return ret;
        }
    }


    //try connecting outside the IF boundary
    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Trying to connect outside the IF boundary...");
    ret = getConnectionOutsideBoundary(infieldBoundary,
                                       pose_start,
                                       pose_end,
                                       turningRad,
                                       limitBoundary,
                                       infieldBoundary,
                                       maxConnectionLength);

    if(!ret.empty()){
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Connected outside the IF boundary");
        return ret;
    }

    return ret;


}

Polygon InfieldTracksConnectorDef::getExtendedLimitBoundary(const Subfield &sf, double turningRad) const
{
    //test
    return IInfieldTracksConnector::getExtendedLimitBoundary(sf, turningRad);



    double minWidth = 3 * turningRad;

    auto boundary = sf.boundary_outer;
    auto boundaryIF = sf.boundary_inner;

    geometry::correct_polygon(boundary);
    geometry::correct_polygon(boundaryIF);

    bool boundaryValid = geometry::isPolygonValid(boundary) != geometry::PolygonValidity::INVALID_POLYGON;
    bool boundaryIFValid = geometry::isPolygonValid(boundaryIF) != geometry::PolygonValidity::INVALID_POLYGON;

    if(!boundaryValid && !boundaryIFValid)
        return Polygon();

    if(!boundaryValid){
        if(turningRad < 1e-6)
            return Polygon();
        if(!geometry::offsetPolygon(boundaryIF, boundary, minWidth, true, 0))
            return Polygon();
        return boundary;
    }

    if(turningRad < 1e-6)
        return boundary;

    if( boundaryValid && boundaryIFValid ){//normal case
        Polygon boundaryIFExt;
        if(geometry::offsetPolygon(boundaryIF, boundaryIFExt, minWidth, true, 0)){

            double maxArea = -1;
            int indUnion = -1;
            std::vector<PolygonWithHoles> unionPolys;
            geometry::get_union(boundary, boundaryIFExt, unionPolys);
            for(size_t i = 0 ; i < unionPolys.size() ; ++i){
                double area = geometry::calc_area(unionPolys.at(i).outer);
                if(area > maxArea){
                    maxArea = area;
                    indUnion = i;
                }
            }
            if(indUnion >= 0)
                return unionPolys.at(indUnion).outer;
        }
    }

    auto getTotalWidth = [](const std::vector<Track> &tracks) -> double{
        double width = 0;
        for(auto& track : tracks)
            width += std::max(0.0, track.width);
        return width;
    };

    bool allWidthsOK = true;
    for(auto& hl : sf.headlands.partial){
        double width = getTotalWidth(hl.tracks);
        if(width > 1e-6 && width < minWidth){
            allWidthsOK = false;
            break;
        }
    }


    auto getPolygonOffset = [&getTotalWidth, minWidth, allWidthsOK, turningRad](const std::vector<Track> &tracks) -> double{
        double width = getTotalWidth(tracks);
        if(width > 1e-6 && width < minWidth)
            return minWidth - width;
        return allWidthsOK ? 0 : 0.5 * turningRad;
    };

    auto extendBoundary = [&boundary](const Polygon& poly){
        std::vector<PolygonWithHoles> unionPolys;
        geometry::get_union(boundary, poly, unionPolys);
        if(unionPolys.empty())
            return;
        if(unionPolys.size() == 1){
            boundary = unionPolys.front().outer;
            return;
        }

        double maxArea = -1;
        int indMaxArea = 0;
        for(size_t i = 0 ; i < unionPolys.size() ; ++i){
            double area = geometry::calc_area(unionPolys.at(i).outer);
            if(area > maxArea){
                maxArea = area;
                indMaxArea = i;
            }
        }
        boundary = unionPolys.at(indMaxArea).outer;
    };

    double offset = getPolygonOffset(sf.headlands.complete.tracks);
    if(offset > 1e-3){
        Polygon poly;
        if(geometry::offsetPolygon(sf.boundary_outer, poly, offset, true, 0))
            return poly;
    }

    for(auto& hl : sf.headlands.partial){
        offset = getPolygonOffset(hl.tracks);
        if(offset > 1e-3){
            Polygon poly;
            if(geometry::offsetPolygon(hl.boundary, poly, offset, true, 0))
                extendBoundary(poly);
        }
    }

    return boundary;
}

std::vector<Point> InfieldTracksConnectorDef::getConnectionOverTracks(const Pose2D &pose_start,
                                                                      const Pose2D &pose_end,
                                                                      double turningRad,
                                                                      const std::vector<std::vector<Point> > &tracks,
                                                                      const Polygon &limitBoundary,
                                                                      const Polygon &infieldBoundary,
                                                                      double maxConnectionLength) const{
    return getConnectionOverTracks_1_2(pose_start, pose_end, turningRad, tracks, limitBoundary, infieldBoundary, maxConnectionLength);
}

std::vector<Point> InfieldTracksConnectorDef::getConnectionOverTracks_1_1(const Pose2D &pose_start,
                                                                          const Pose2D &pose_end,
                                                                          double turningRad,
                                                                          const std::vector<std::vector<Point> > &tracks,
                                                                          const Polygon &limitBoundary,
                                                                          const Polygon &infieldBoundary,
                                                                          double maxConnectionLength) const
{
    std::vector<Point> ret;
    std::map<double, std::vector<Point>> tracksSorted = getConnectionOverTracks_adjustAndSortTracks(pose_start, pose_end, turningRad, tracks);
    if(turningRad < 1e-9){
        if(!tracksSorted.empty())
            std::swap(ret, tracksSorted.begin()->second);
        return ret;
    }

    const double searchRad = 3 * turningRad;

    auto getSegmentConnection = [&](PointVec seg, bool isStart)->PointVec{
        static const InfieldTracksConnectorDubins::ExtensionParameters extParams = InfieldTracksConnectorDubins::ExtensionParameters::NoExtention();
        Pose2D pose0 = isStart ? pose_start : pose_end;
        double minLength = std::numeric_limits<double>::max();
        if(!isStart){
            pose0.angle += M_PI;
            std::reverse(seg.begin(), seg.end());
        }

        int indMin = -1;
        PointVec ret;
        for(size_t i = 0; i+1 < seg.size() ; ++i){
            Pose2D posen( r_at(seg, i+1), ag::get_angle(r_at(seg, i+1), r_at(seg, i)) );

            if(ag::calc_dist(pose0, posen) > searchRad)
                continue;

            double length;
            auto path0 = InfieldTracksConnectorDubins::getDubinsPathConnection(pose0, posen, turningRad, {&limitBoundary}, infieldBoundary, maxConnectionLength, false, &extParams, &length);
            if(path0.empty())
                continue;
            if(length > minLength)
                break;
            indMin = seg.size()-i-2;
            std::swap(ret, path0);
        }
        if(indMin < 0)
            return ret;
        ret.insert(ret.end(), seg.begin()+indMin+1, seg.end());
        if(!isStart)
            std::reverse(ret.begin(), ret.end());
        return ret;
    };

    for(auto& tr_it : tracksSorted){
        PointVec& track = tr_it.second;

        if(track.size() < 2)
            continue;

        size_t indEnd = track.size() - 2;
        for(size_t i = 0; i+1 < track.size(); ++i){
            auto& pt = track.at(i);
            double dist = ag::calc_dist(pose_start, pt);
            if(dist >= searchRad){
                indEnd = i;
                break;
            }
        }

        PointVec seg(track.begin(), track.begin()+indEnd+2);
        size_t segPtsCount = seg.size();
        seg = ag::sample_geometry( seg, turningRad * 0.25 );

        PointVec seg0 = getSegmentConnection(seg, true);
        if(seg0.empty())
            continue;

        pop_front(track, segPtsCount);
        track.insert(track.begin(), seg0.begin(), seg0.end());

        if(track.size() < 2)
            continue;

        indEnd = 1;
        for(size_t i = track.size()-1; i > 0; --i){
            auto& pt = track.at(i);
            double dist = ag::calc_dist(pose_end, pt);
            if(dist >= searchRad){
                indEnd = i;
                break;
            }
        }

        seg = PointVec(track.begin()+indEnd-1, track.end());
        segPtsCount = seg.size();
        seg = ag::sample_geometry( seg, turningRad * 0.25 );

        PointVec segn = getSegmentConnection(seg, false);
        if(segn.empty())
            continue;

        pop_back(track, segPtsCount);
        track.insert(track.end(), segn.begin(), segn.end());

//        ag::unsample_linestring(track);
//        if(!isPathValid(track, limitBoundary, infieldBoundary, maxConnectionLength)){
//            continue;
//        }

//        if(turningRad > 1e-6){//@todo this step is relativelly time expensive
//            ag::PathSmoother ppss;
//            track = ppss.smoothenPath(track, turningRad);
//        }

//        if(track.empty())
//            continue;

        ag::unsample_linestring(track);
        if(isPathValid(track, Polygon(), Polygon(), maxConnectionLength)){//no boundaries: we assume the given track fullfilled that condition
            ret = track;
            break;
        }
    }

    return ret;
}


std::vector<Point> InfieldTracksConnectorDef::getConnectionOverTracks_1_2(const Pose2D &pose_start,
                                                                          const Pose2D &pose_end,
                                                                          double turningRad,
                                                                          const std::vector<std::vector<Point> > &tracks,
                                                                          const Polygon &limitBoundary,
                                                                          const Polygon &infieldBoundary,
                                                                          double maxConnectionLength) const
{
    double turningRadEd = std::max(0.0, turningRad);

    auto isPointValid = [turningRadEd](const Pose2D& pose, const Point& p, const Point& p2, bool towardsPose) -> bool{
        //@todo this check can be improved

        //change reference frame so that the pose is the origin and the x_axis is direction of the pose
        double rotAngle = towardsPose ? -(pose.angle + M_PI) : -pose.angle;
        Point p0_transf = p - pose.point();
        Point p1_transf = p2 - pose.point();
        Point pRef0 = Point(0,0);
        p0_transf = ag::rotate(pRef0, p0_transf, rotAngle);
        p1_transf = ag::rotate(pRef0, p1_transf, rotAngle);

        if( p0_transf.x < 0 )//it is behind the pose : too strict?
            return false;

        double ang1 = ag::get_angle(p0_transf, p1_transf);
        double ang2 = ag::get_angle(pRef0, p0_transf);

        //check y-axis position condition 1
        if( ang1 > 1e-3 && ang2 < 1e-3 )
            return false;
        if( ang1 < 1e-3 && ang2 > 1e-3 )
            return false;

        if( std::fabs(ang1) > M_PI_2 ){
            auto ang1_ed = std::fabs(ang1) - M_PI_2;
            double ang1_tan = std::fabs(ang1) > M_PI_2+1e-3 ? std::tan(ang1) : std::numeric_limits<double>::max();
            double x_thershold_1 = turningRadEd * std::cos(ang1_ed);
            double x_thershold_2 = turningRadEd * ( std::fabs(ang1_tan) + 0.1 );
            double y_thershold = turningRadEd * ( 1 + std::sin(ang1_ed) );

            if( std::fabs(p0_transf.x) >= x_thershold_1 &&
                    std::fabs(p0_transf.x) <= x_thershold_2 &&
                    std::fabs(p0_transf.y) >= y_thershold )
                return true;

            return false;
        }


        //check 'save' x-axis position condition
        if( p0_transf.x >= 2*turningRadEd - std::sin( std::fabs(ang1) ) )
            return true;

        //move to y_axist positive quadrant
        p0_transf.y = std::fabs( p0_transf.y );
        p1_transf.y = std::fabs( p1_transf.y );
        ang1 = std::fabs(ang1);
        ang2 = std::fabs(ang2);

        //now we work in the (+,+) quadrant

        //check x-axis position contition 1
        if( p0_transf.x < turningRadEd * std::sin(ang1) )
            return false;

        //check y-axis position contition 1
        if( p0_transf.y < turningRadEd * (1 - std::cos(ang1) ) )
            return false;

        //check with respect to a 2xrad circle tangent to pRef0
        if( ag::calc_dist( p0_transf, Point(0, 2*turningRadEd) ) < 2*turningRadEd-1e-5 )
            return false;

        return true;


        //---

//        const Point *pP1 = towardsPose ? &p2 : &p;
//        const Point *pP2 = towardsPose ? &p : &p2;
//        Point pPose1 = ag::getPointAtDist(pose, 1.0);
//        Point pPose2 = ag::getPointAtDist(pose, towardsPose ? -1.0 : 1.0);

//        double ang1 = ag::get_angle(pose, pPose1, *pP1, *pP2);
//        if( std::fabs(ang1) > M_PI_2 * 1.1 )
//            return false;

//        double ang2 = ag::get_angle(p, pose, pPose2);
//        if( std::fabs(ang2) > M_PI_2 )
//            return false;

//        double dist1 = ag::calc_dist(pose, p);
//        double distCmpr1 = std::fabs(ang1) * M_1_PI * 2 * turningRadEd;
//        if(dist1 < distCmpr1)
//            return false;
//        double distCmpr2 = dist1 * std::cos(ang2);
//        if(distCmpr2 < turningRadEd)
//            return false;
//        return true;

        //---

        //return ag::calc_dist(pose, p) > turningRad;
    };

    Polygon limitBoundaryFinalCheck;
    if(limitBoundary.points.size() > 3){
        double offset = turningRad > 1e-9 ? 0.1 * std::min(turningRad, 1.0) : 0.1;
        if(!ag::offsetPolygon(limitBoundary, limitBoundaryFinalCheck, offset, true))
            limitBoundaryFinalCheck = limitBoundary;
    }

    Polygon infieldBoundaryFinalCheck;
    if(infieldBoundary.points.size() > 3){
        double offset = turningRad > 1e-9 ? 0.1 * std::min(turningRad, 1.0) : 0.1;
        if(!ag::offsetPolygon(infieldBoundary, infieldBoundaryFinalCheck, offset, false))
            infieldBoundaryFinalCheck = infieldBoundary;
    }

    std::vector<Point> ret;
    std::map<double, std::vector<Point>> tracksSorted = getConnectionOverTracks_adjustAndSortTracks(pose_start, pose_end, turningRad, tracks);
    for(auto& tr_it : tracksSorted){
        auto& track = tr_it.second;
        track = ag::sample_geometry(track, turningRad > 1e-9 ? turningRad * 0.25 : 0.05 * ag::getGeometryLength(track) );

        if(turningRad > 1e-9){
            bool found = false;
            for(size_t i = 0; i+1 < track.size() ; ++i){
                if(isPointValid(pose_start, track.at(i), track.at(i+1), false)){
                    track.erase(track.begin(), track.begin()+i);
                    found = true;
                    break;
                }
            }
            if(!found)
                continue;

            found = false;
            for(int i = track.size()-1; i > 0; --i){
                if(isPointValid(pose_end, track.at(i), track.at(i-1), true)){
                    track.erase(track.begin()+(i+1), track.end());
                    found = true;
                    break;
                }
            }
            if(!found)
                continue;
        }

        if(track.size() < 2)
            continue;


        PointVec seg0 = {pose_start.point()};
        PointVec segn = {track.back()};

        if(turningRad > 0){
            seg0.push_back( ag::getPointAtDist(pose_start, 0.5*turningRad) );
            segn.push_back( ag::getPointAtDist(pose_end, -0.5*turningRad) );
        }

        seg0.push_back( track.front() );
        segn.push_back( pose_end.point() );

        if(!isPathValid(seg0, limitBoundary, infieldBoundary, maxConnectionLength)
                || !isPathValid(segn, limitBoundary, infieldBoundary, maxConnectionLength)){
            continue;
        }

        track.insert(track.begin(), seg0.begin(), seg0.end()-1);
        track.insert(track.end(), segn.begin()+1, segn.end());

        ag::unsample_linestring(track);

        if(!isPathValid(track, limitBoundaryFinalCheck, infieldBoundaryFinalCheck, maxConnectionLength))//we use more relaxed boundaries (we assume that the intermediate track points comply with boundaries)
            continue;

        if(turningRad > 1e-6){//@todo this step is relativelly time expensive
            ag::PathSmoother ppss;
            track = ppss.smoothenPath(track, turningRad);
        }

        if(track.empty())
            continue;

        if(isPathValid(track, limitBoundaryFinalCheck, infieldBoundaryFinalCheck, maxConnectionLength)){//we use more relaxed boundaries (we assume that the intermediate track points comply with boundaries)
            ret = track;
            break;
        }
    }

    return ret;
}



std::vector<Point> InfieldTracksConnectorDef::getConnectionOverTracks_2(const Pose2D &pose_start,
                                                                        const Pose2D &pose_end,
                                                                        double turningRad,
                                                                        const std::vector<std::vector<Point> > &tracks,
                                                                        const Polygon &limitBoundary,
                                                                        const Polygon &infieldBoundary,
                                                                        double maxConnectionLength) const
{

    std::vector<Point> ret;
    std::map<double, std::vector<Point>> tracksSorted;

    double extraDist = (turningRad > 1e-6 ? 2 * turningRad : 1.0);
    Point p2_start = ag::getPointAtDist(pose_start, -extraDist);
    Point p2_end = ag::getPointAtDist(pose_end, extraDist);

    for(auto & track : tracks){
        auto track_ed = track;
        ag::unsample_linestring(track_ed);

        size_t indNoInt = 0;
        for(; indNoInt+1 < track_ed.size() ; ++indNoInt){
            std::vector<Point> segment = {track_ed.at(indNoInt), track.at(indNoInt+1)};
            auto intersections = ag::get_intersection(segment, pose_start.point(), p2_start, false, false, false);
            if(intersections.empty())
                break;
        }
        if(indNoInt+1 >= track_ed.size())
            continue;
        if(indNoInt > 0)
            pop_front(track_ed, indNoInt);

        indNoInt = 0;
        for(; indNoInt+1 < track_ed.size() ; ++indNoInt){
            std::vector<Point> segment = {r_at(track_ed, indNoInt), r_at(track_ed, indNoInt+1)};
            auto intersections = ag::get_intersection(segment, pose_end.point(), p2_end, false, false, false);
            if(intersections.empty())
                break;
        }
        if(indNoInt+1 >= track_ed.size())
            continue;
        if(indNoInt > 0)
            pop_back(track_ed, indNoInt);

        if(track_ed.size() < 2)
            continue;

        auto dist1 = ag::calc_dist(track_ed.front(), pose_start);
        auto dist2 = ag::calc_dist(track_ed.back(), pose_end);
        auto dist3 = ag::getGeometryLength(track_ed);

        tracksSorted[dist1 + dist2 + dist3] = track_ed;
    }

    double minLength = (maxConnectionLength < 1e-9 ? std::numeric_limits<double>::max() : maxConnectionLength);
    double searchRadiusOut = 3 * turningRad;

    for(auto& tr_it : tracksSorted){

        std::vector<Point>& track = tr_it.second;
        double pathLength;

        if(track.size() < 2)
            continue;

        if(turningRad < 1e-9){
            std::vector<Point> path;
            path.push_back(pose_start);
            path.insert(path.end(), track.begin(), track.end());
            path.push_back(pose_end);
            pathLength = ag::getGeometryLength(path);
            if(minLength <= pathLength)
                continue;

            ag::unsample_linestring(path);
            if(!isPathValid(path, limitBoundary, infieldBoundary, -1))//the maxConnectionLength is already checked in the minLength
                continue;

            ret = path;
            minLength = pathLength;
            continue;
        }

        track = ag::sample_geometry(track, turningRad > 1e-9 ? turningRad : 1 );

        size_t indStartMax = 0;
        size_t indEndMin = track.size()-1;

        for(; indStartMax+1 < track.size() ; ++indStartMax){
            if( ag::calc_dist(pose_start, track.at(indStartMax)) > searchRadiusOut )
                break;
        }

        for(; indEndMin > 0; --indEndMin){
            if( ag::calc_dist(pose_end, track.at(indEndMin)) > searchRadiusOut )
                break;
        }

        if(indStartMax == 0)
            indStartMax++;

        if(indEndMin == track.size()-1)
            indEndMin--;

        if( indEndMin > indStartMax && ag::getGeometryLength(track, indStartMax-1, indEndMin+1) > minLength )//check the minimum length that the path could have
            continue;

        for(size_t i = 0 ; i < indStartMax ; ++i){
            double dist0 = ag::calc_dist(pose_start, track.at(i));
            Pose2D pose0(track.at(i), ag::get_angle(track.at(i), track.at(i+1)));
            double angRef = ag::get_angle( pose_start, ag::getPointAtDist(pose_start, 1),
                                            pose0, ag::getPointAtDist(pose0, 1) );
            double searchRadiusIn = turningRad * (1 - std::fabs(angRef) / M_PI );
            if( dist0 < searchRadiusIn )
                continue;

            double length0;

//            auto path0 = ag::calcDubinsPath(10, pose_start, pose0, turningRad, ag::DubinsParams::SHORTEST, &length0);
//            if(path0.empty())
//                continue;
//            if(!isPathValid(path0, limitBoundary, infieldBoundary, -1))//the maxConnectionLength is already checked in the minLength
//                continue;

            InfieldTracksConnectorDubins::ExtensionParameters extParams = InfieldTracksConnectorDubins::ExtensionParameters::NoExtention();
            auto path0 = InfieldTracksConnectorDubins::getDubinsPathConnection(pose_start, pose0, turningRad, {&limitBoundary}, infieldBoundary, -1, false, &extParams, &length0);//the maxConnectionLength is already checked in the minLength
            if(path0.empty()){
//                path0 = InfieldTracksConnectorDubins::getDubinsPathConnection(pose_start, pose0, turningRad, {&limitBoundary}, infieldBoundary, -1, true, &extParams, &length0);//the maxConnectionLength is already checked in the minLength
//                if(path0.empty())
                    continue;
            }


            for(size_t j = track.size()-1 ; j > indEndMin && j >= i ; --j){
                double distn = ag::calc_dist(pose_end, track.at(j));
                Pose2D posen(track.at(j), ag::get_angle(track.at(j-1), track.at(j)));
                double angRef = ag::get_angle( pose_end, ag::getPointAtDist(pose_end, 1),
                                                posen, ag::getPointAtDist(posen, 1) );
                double searchRadiusIn = turningRad * (1 - std::fabs(angRef) / M_PI) ;
                if( distn < searchRadiusIn )
                    continue;

                double segLength = ag::getGeometryLength(track, i, j);
                if(minLength <= length0 + segLength)
                    continue;

                double lengthn;

//                auto pathn = ag::calcDubinsPath(10, posen, pose_end, turningRad, ag::DubinsParams::SHORTEST, &lengthn);
//                if(pathn.empty())
//                    continue;


                auto pathn = InfieldTracksConnectorDubins::getDubinsPathConnection(posen, pose_end, turningRad, {}, Polygon(), -1, false, &extParams, &lengthn);//the validity will be checked on the complete path
                if(pathn.empty()){
//                    pathn = InfieldTracksConnectorDubins::getDubinsPathConnection(posen, pose_end, turningRad, {}, Polygon(), -1, true, &extParams, &lengthn);//the validity will be checked on the complete path
//                    if(path0.empty())
                        continue;
                }


                double pathLength = length0 + segLength + lengthn;
                if(minLength > pathLength){
                    auto path = path0;
                    if(j-i > 1)
                        path.insert(path.end(), track.begin()+i+1, track.begin()+j);
                    path.insert(path.end(), pathn.begin(), pathn.end());

                    ag::unsample_linestring(path);
                    if(!isPathValid(path, limitBoundary, infieldBoundary, -1))//the maxConnectionLength is already checked in the minLength
                        continue;

                    ret = path;
                    minLength = pathLength;
                }

            }
        }
    }

    return ret;
}

std::map<double, std::vector<Point> > InfieldTracksConnectorDef::getConnectionOverTracks_adjustAndSortTracks(const Pose2D &pose_start, const Pose2D &pose_end, double turningRad, const std::vector<std::vector<Point> > &tracks) const{

    std::map<double, std::vector<Point>> tracksSorted;

    for(auto & track : tracks){
        auto track_ed = track;
        ag::unsample_linestring(track_ed);

        if(track_ed.size() < 2)
            continue;

        double extensionDist = std::max(1.0, 2*turningRad);

        int ind1 = -1;
        Point pRef0 = pose_start.point();
        Point pRef1 = ag::getPointAtDist(pose_start, extensionDist);
        std::vector<Point> intersections = ag::get_intersection(pRef0, pRef1, track_ed, true, false, false);

        if(!intersections.empty()){
            for(auto& pInt : intersections){
                int ind = ag::addSampleToGeometryClosestToPoint(track_ed, pInt, 1);
                if(ind < 0 || ind > track_ed.size())
                    continue;
                if(ind1 < 0 || ind <= ind1)
                    ind1 = ind;
            }
        }
        else{
            ind1 = ag::addSampleToGeometryClosestToPoint(track_ed, pose_start, 1);
            if(ind1 < 0 || ind1 > track_ed.size())
                continue;

            //note: it is not enough to get the ind1 from addSampleToGeometryClosestToPoint because, in the cases where the track contains a same track twise (fwd and bwd), the unsampling might cause that the expencted ind1 is not obtaines
            pRef1 = track_ed.at(ind1);
            if(pRef0 != pRef1){
                pRef1 = ag::getPointInLineAtDist(pRef0, pRef1, extensionDist);
                intersections = ag::get_intersection(pRef0, pRef1, track_ed, true, false, false);
                if(!intersections.empty()){
                    ind1 = -1;
                    for(auto& pInt : intersections){
                        int ind = ag::addSampleToGeometryClosestToPoint(track_ed, pInt, 1);
                        if(ind < 0 || ind > track_ed.size())
                            continue;
                        if(ind1 < 0 || ind <= ind1)
                            ind1 = ind;
                    }
                }
            }
        }
        if(ind1 < 0 || ind1 > track_ed.size())
            continue;

        auto track_ed_rev = track_ed;//in case the segments of the track are "repeated")
        std::reverse(track_ed_rev.begin(), track_ed_rev.end());

        int ind2 = -1;
        pRef0 = pose_end.point();
        pRef1 = ag::getPointAtDist(pose_end, -extensionDist);
        intersections = ag::get_intersection(pRef0, pRef1, track_ed_rev, true, false, false);

        if(!intersections.empty()){
            for(auto& pInt : intersections){
                int ind = ag::addSampleToGeometryClosestToPoint(track_ed_rev, pInt, 1);
                if(ind < 0 || ind > track_ed_rev.size())
                    continue;
                if(ind2 < 0 || ind <= ind2)
                    ind2 = ind;
            }
        }
        else{
            ind2 = ag::addSampleToGeometryClosestToPoint(track_ed_rev, pose_end, 1);
            if(ind2 < 0 || ind2 > track_ed_rev.size())
                continue;
            pRef1 = track_ed_rev.at(ind2);
            if(pRef0 != pRef1){
                pRef1 = ag::getPointInLineAtDist(pRef0, pRef1, extensionDist);
                intersections = ag::get_intersection(pRef0, pRef1, track_ed_rev, true, false, false);
                if(!intersections.empty()){
                    ind2 = -1;
                    for(auto& pInt : intersections){
                        int ind = ag::addSampleToGeometryClosestToPoint(track_ed_rev, pInt, 1);
                        if(ind < 0 || ind > track_ed_rev.size())
                            continue;
                        if(ind2 < 0 || ind <= ind2)
                            ind2 = ind;
                    }
                }
            }
        }
        if(ind2 < 0 || ind2 > track_ed_rev.size())
            continue;

        ind2 = track_ed_rev.size() - 1 - ind2;

        if(track_ed.size() != track_ed_rev.size()){
            track_ed.insert(track_ed.begin()+ind2, track_ed_rev.at( track_ed_rev.size() - 1 - ind2 ));
            if(ind2 <= ind1 )
                ind1++;
        }

        if(ind1 > ind2){
            track_ed.erase( track_ed.begin() + (ind1+1), track_ed.end() );
            track_ed.erase( track_ed.begin(), track_ed.begin() + ind2 );
            std::reverse(track_ed.begin(), track_ed.end());
        }
        else{
            track_ed.erase( track_ed.begin() + (ind2+1), track_ed.end() );
            track_ed.erase( track_ed.begin(), track_ed.begin() + ind1 );
        }
        if(track_ed.size() < 2)
            continue;


        auto dist1 = ag::calc_dist(track_ed.front(), pose_start);
        auto dist2 = ag::calc_dist(track_ed.back(), pose_end);
        auto dist3 = ag::getGeometryLength(track_ed);

        tracksSorted[dist1 + dist2 + dist3] = track_ed;
    }
    return tracksSorted;
}

std::vector<Point> InfieldTracksConnectorDef::getConnectionOverHeadland(const CompleteHeadland &hl,
                                                                        const Pose2D &pose_start,
                                                                        const Pose2D &pose_end,
                                                                        double turningRad,
                                                                        const Polygon &limitBoundary,
                                                                        const Polygon &infieldBoundary,
                                                                        double maxConnectionLength) const
{
    std::vector<Point> ret;

    if(!hl.tracks.empty()){
        //try using closest valid track from surrounding headland
        std::vector<std::vector<Point> > tracks;
        tracks.reserve(hl.tracks.size() * 2);
        for(auto & track : hl.tracks){
            Polygon poly;
            poly.points = track.points;
            if( !ag::isPolygonClosed(poly) )
                continue;
            ag::correct_polygon(poly);
            ag::unsample_polygon(poly);
            ag::addSampleToGeometryClosestToPoint(poly.points, pose_start, 1);
            ag::addSampleToGeometryClosestToPoint(poly.points, pose_end, 1);
            tracks.emplace_back( ag::getShortestGeometryPart(poly.points, pose_start, pose_end) );
            tracks.emplace_back( ag::getLongestGeometryPart(poly.points, pose_start, pose_end) );
        }

        ret = getConnectionOverTracks(pose_start, pose_end, turningRad, tracks, limitBoundary, infieldBoundary, maxConnectionLength);
        if(!ret.empty())
            return ret;

        ret = getConnectionOverTracks_2(pose_start, pose_end, turningRad, tracks, limitBoundary, infieldBoundary, maxConnectionLength);
        if(!ret.empty())
            return ret;
    }


    if(hl.middle_track.points.size() > 2){
        //try using middle track from surrounding headland
        std::vector<std::vector<Point> > tracks;
        if( ag::isPolygonClosed(hl.middle_track) ){
            Polygon poly;
            poly.points = hl.middle_track.points;
            ag::correct_polygon(poly);
            ag::unsample_polygon(poly);
            ag::addSampleToGeometryClosestToPoint(poly.points, pose_start, 1);
            ag::addSampleToGeometryClosestToPoint(poly.points, pose_end, 1);
            tracks.emplace_back( ag::getShortestGeometryPart(poly.points, pose_start, pose_end) );
            tracks.emplace_back( ag::getLongestGeometryPart(poly.points, pose_start, pose_end) );
            ret = getConnectionOverTracks(pose_start, pose_end, turningRad, tracks, limitBoundary, infieldBoundary, maxConnectionLength);
            if(!ret.empty())
                return ret;

            ret = getConnectionOverTracks_2(pose_start, pose_end, turningRad, tracks, limitBoundary, infieldBoundary, maxConnectionLength);
            if(!ret.empty())
                return ret;
        }
    }

    return ret;
}

std::vector<Point> InfieldTracksConnectorDef::getConnectionOverHeadland(const std::vector<PartialHeadland> &hls,
                                                                        const Pose2D &pose_start,
                                                                        const Pose2D &pose_end,
                                                                        const Machine machine,
                                                                        double turningRad,
                                                                        const Polygon &limitBoundary,
                                                                        const Polygon &infieldBoundary,
                                                                        double maxConnectionLength) const
{
    if(hls.empty())
        return {};

    int hl_ind_start = -1, hl_ind_end = -1;

    //search for the headlands adjacent to the start and end poses
    for(size_t i = 0 ; i < hls.size() ; ++i){
        Polygon boundary;
        const auto& hl = hls.at(i);
        if(!ag::offsetPolygon( hl.boundary, boundary, std::max(0.1, 0.1*turningRad), true ))
            boundary = hl.boundary;
        if(hl_ind_start < 0 && ag::in_polygon(pose_start, boundary))
            hl_ind_start = i;
        if(hl_ind_end < 0 && ag::in_polygon(pose_end, boundary))
            hl_ind_end = i;
    }
    if(hl_ind_start < 0 || hl_ind_end < 0)
        return {};

    std::vector< std::vector<Point> > tracks;
    std::vector< const Polygon* > limitPolys = {&limitBoundary};

    if(hl_ind_start == hl_ind_end)//connect using only the headland tracks
        return getConnectionOverSamePartialHeadland(hls, hls.at(hl_ind_start), pose_start, pose_end, turningRad, limitBoundary, infieldBoundary, maxConnectionLength);

    //check if there is one or more headlands connecting hl_ind_start and hl_ind_end

    PointVec ret = getConnectionOverAdjacentHeadlands(hls, hl_ind_start, hl_ind_end,
                                                      pose_start, pose_end, turningRad,
                                                      limitBoundary, infieldBoundary, maxConnectionLength);
    if(!ret.empty())
        maxConnectionLength = std::min(maxConnectionLength, ag::getGeometryLength(ret));


    //check if there is a cyclic headland connection sequence
    auto connectionSequences = PartialHeadland::getHeadlandConnectionSequences(hls, hl_ind_start, hl_ind_start);
    std::vector<size_t> cyclicSequence;
    for(auto& seq : connectionSequences){
        for(auto ind: seq){
            if(ind == hl_ind_end){
                cyclicSequence = seq;
                break;
            }
        }
        if(!cyclicSequence.empty())
            break;
    }
    if(!cyclicSequence.empty()){
        std::vector< std::vector<Point> > tracksTmp;
        getTracksFromHeadlandSequence(hls, cyclicSequence, tracksTmp, turningRad, maxConnectionLength);
        tracks = getSubTracksFromClosedTracks(tracksTmp, pose_start, pose_end);
    }
    else{
        connectionSequences = PartialHeadland::getHeadlandConnectionSequences(hls, hl_ind_start, hl_ind_end);
        getTracksFromHeadlandSequences(hls, connectionSequences, tracks, turningRad, maxConnectionLength);
    }


    if(tracks.empty())
        return ret;

    //try connecting using the tracks
    auto ret_1 = getConnectionOverTracks(pose_start, pose_end, turningRad, tracks, limitBoundary, infieldBoundary, maxConnectionLength);
    if(!ret_1.empty())
        maxConnectionLength = ag::getGeometryLength(ret_1) / 1.5;
    auto ret_2 = getConnectionOverTracks_2(pose_start, pose_end, turningRad, tracks, limitBoundary, infieldBoundary, maxConnectionLength);
    if(!ret_2.empty())
        return ret_2;
    if(!ret_1.empty())
        return ret_1;

    return ret;
}

std::vector<Point> InfieldTracksConnectorDef::getConnectionOutsideBoundary(Polygon boundary,
                                                                           const Pose2D &pose_start,
                                                                           const Pose2D &pose_end,
                                                                           double turningRad,
                                                                           const Polygon &limitBoundary,
                                                                           const Polygon &infieldBoundary,
                                                                           double maxConnectionLength) const
{
    std::vector<Point> ret;
    if(boundary.points.size() < 3)
        return ret;

    double offsetDist1 = 0, offsetDist2 = 0;
    std::pair<double, double> offsetLimits {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    bool expand = true;

    for(int ii = 0 ; ii < 2 ; ii ++){
        Pose2D pose = ii == 0 ? pose_start : pose_end;
        double *offsetDist = ii == 0 ? &offsetDist1 : &offsetDist2;

        if(ii!=0){
            pose.angle -= M_PI;
            ag::correct_angle(pose.angle);
        }

        int ind1 = ag::addSampleToGeometryClosestToPoint(boundary.points, pose, 1);
        if(ind1 < 0 || ind1 > boundary.points.size())
            return ret;

        auto dist = ag::calc_dist(boundary.points.at(ind1), pose);
        double angle1 = ag::get_angle( ag::getPointAtDist(pose, 1), pose, boundary.points.at(ind1) );

        double refDist = turningRad * ( 1 + std::fabs( std::cos(angle1) ) );

        if(ag::in_polygon(pose, boundary)){
            if( std::fabs(angle1) > M_PI_2 ){//boundary must be shrinked
                *offsetDist = -1 * (dist + refDist);
                expand = false;
                offsetLimits.first = 0;
            }
            else {
                if(dist < refDist){//boundary must be expanded
                    *offsetDist = refDist - dist;
                    offsetLimits.second = 0;
                }
                else
                    offsetLimits.second = std::min(offsetLimits.second, dist - refDist);

            }
        }
        else{
            if( std::fabs(angle1) >= M_PI_2 ){//boundary must be expanded
                *offsetDist = dist + refDist;
                offsetLimits.second = 0;
            }
            else {
                if(dist < refDist){//boundary must be shrinked
                    *offsetDist = -1 * (refDist - dist);
                    expand = false;
                    offsetLimits.first = 0;
                }
                else
                    offsetLimits.first = std::min(offsetLimits.first, dist - refDist);
            }
        }
    }

    if( (offsetDist1 < -1e-9 && offsetDist2 > 1e-9) || (offsetDist2 < -1e-9 && offsetDist1 > 1e-9) )//we cannot expand AND shrink
        return ret;

    offsetDist1 = std::max( std::fabs(offsetDist1), std::fabs(offsetDist2) );
    if(!expand)
        offsetLimits.first = offsetLimits.second;
    offsetDist2 = 0;

    if(offsetDist1 > offsetLimits.first){
        offsetDist2 = 0.1 * (offsetDist1 - offsetLimits.first);
        offsetDist1 = 0.5 * (offsetDist1 + offsetLimits.first);
    }

    CompleteHeadland hl;
    Polygon boundaryEd;

    if(offsetDist1 > 1e-9){
        if( !ag::offsetPolygon(boundary, boundaryEd, offsetDist1, expand, 0) )
            return ret;
    }
    else
        boundaryEd = boundary;

    ag::correct_polygon(boundaryEd);
    if(ag::isPolygonValid(boundaryEd) != ag::PolygonValidity::VALID_CLOSED_CW)
        return ret;

    hl.tracks.push_back(Track());
    hl.tracks.back().points = boundaryEd.points;

    if(offsetDist2 > 1e-9){
        if( ag::offsetPolygon(boundary, boundaryEd, offsetDist1 + offsetDist2, expand, 0) ){
            ag::correct_polygon(boundaryEd);
            hl.tracks.push_back(Track());
            hl.tracks.back().points = boundaryEd.points;
        }
        if( ag::offsetPolygon(boundary, boundaryEd, offsetDist1 - offsetDist2, expand, 0) ){
            ag::correct_polygon(boundaryEd);
            hl.tracks.push_back(Track());
            hl.tracks.back().points = boundaryEd.points;
        }
    }

    ret = getConnectionOverHeadland(hl, pose_start, pose_end, turningRad, limitBoundary, infieldBoundary, maxConnectionLength);
    if(!ret.empty())
        return ret;

    return ret;
}

std::vector<Point> InfieldTracksConnectorDef::getConnectionOverSamePartialHeadland(const std::vector<PartialHeadland> &hls,
                                                                                   const PartialHeadland& hl,
                                                                                   const Pose2D& pose_start,
                                                                                   const Pose2D& pose_end,
                                                                                   double turningRad,
                                                                                   const Polygon& limitBoundary,
                                                                                   const Polygon& infieldBoundary,
                                                                                   double maxConnectionLength) const
{

    auto getExtraPointInAdjacentHL = [&hls, &hl, &turningRad, &limitBoundary](const Pose2D& poseRef, int trackInd, std::map<int, Point>& knownExtraPoses) -> Point{
        auto it1 = knownExtraPoses.find(trackInd);
        if(it1 != knownExtraPoses.end() && it1->second.isValid())
            return it1->second;

        Polygon boundaryEd;
        const Polygon* b1 = nullptr;
        const Polygon* b2 = nullptr;
        for(auto& hl2 : hls){
            if(hl.connectingHeadlandIds.first == hl2.id)
                b1 = &hl2.boundary;
            if(hl.connectingHeadlandIds.second == hl2.id)
                b2 = &hl2.boundary;
        }

        const Polygon* boundary = nullptr;
        if(b1 && b2)
            boundary = ( ag::calc_dist_to_linestring(b1->points, poseRef) < ag::calc_dist_to_linestring(b2->points, poseRef) ? b1 : b2 );
        else if(b1)
            boundary = b1;
        else if(b1)
            boundary = b2;

        if(boundary){
            if(!ag::offsetPolygon(*boundary, boundaryEd, turningRad, false)){
                if(!ag::offsetPolygon(*boundary, boundaryEd, 0.5 * turningRad, false)){
                    boundaryEd.points.clear();
                }
            }
        }

        if(!boundaryEd.points.empty()){
            int ind = ag::addSampleToGeometryClosestToPoint(boundaryEd.points, poseRef, 1);
            if(ind >= 0){
                knownExtraPoses[trackInd] = boundaryEd.points.at(ind);
                return boundaryEd.points.at(ind);
            }
        }


        if(boundary && boundary->points.size() > 3){
            Point extraPt = ag::getPointAtDist(poseRef, turningRad);
            boundaryEd = *boundary;
            ag::closePolygon(boundaryEd);
            int ind = ag::addSampleToGeometryClosestToPoint(boundaryEd.points, extraPt, 1);
            if(ind >= 0){
                Point centroid;
                if(ag::getCentroid(boundaryEd, centroid)){
                    extraPt = ag::getPointInLineAtDist( boundaryEd.points.at(ind), centroid, turningRad + std::min(0.1, 0.01 * turningRad) );
                    if( limitBoundary.points.empty() || ag::in_polygon(extraPt, limitBoundary) ){
                        knownExtraPoses[trackInd] = extraPt;
                        return extraPt;
                    }
                }
            }
        }

        Point extraPt = ag::getPointAtDist(poseRef, turningRad + std::min(0.1, 0.01 * turningRad));
        if( limitBoundary.points.empty() || ag::in_polygon(extraPt, limitBoundary) ){
            knownExtraPoses[trackInd] = extraPt;
            return extraPt;
        }

        extraPt = ag::getPointAtDist(poseRef, 0.5 * turningRad);
        if( ag::in_polygon(extraPt, limitBoundary) ){
            knownExtraPoses[trackInd] = extraPt;
            return extraPt;
        }

        return Point::invalidPoint();
    };

    std::vector< std::vector<Point> > tracks;

    if(hl.tracks.empty())
        return {};

//        if(turningRad > 1e-6){
//            double pathLength;
//            ret = getConnectionOverSamePartialHeadlandWithDubins(hl, pose_start, pose_end, turningRad, limitBoundary, infieldBoundary, maxConnectionLength, pathLength);
//            if(!ret.empty())
//                maxConnectionLength = pathLength;
//        }

    tracks.reserve(hl.tracks.size());
    for(auto & tr : hl.tracks){
        auto pts = tr.points;
        int ind1 = ag::addSampleToGeometryClosestToPoint(pts, pose_start);
        int ind2 = ag::addSampleToGeometryClosestToPoint(pts, pose_end);
        if(ind2 <= ind1)
            std::reverse(pts.begin(), pts.end());
        tracks.emplace_back(pts);
    }

    if(hl.isConnectingHeadland(false)){//add connected hl tracks
        std::map<int, Point> knownExtraPoses;
        tracks.reserve(tracks.size() + 2 * ( hl.tracks.size() - 1) * 3 - 1);
        for(size_t i = 0 ; i < hl.tracks.size() ; ++i){
            const auto& track1 = hl.tracks.at(i).points;
            if(track1.size() < 2)
                continue;
            for(size_t j = i ; j < hl.tracks.size() && j <= i+2 ; ++j){
                const auto& track2 = hl.tracks.at(j).points;
                if(track2.size() < 2)
                    continue;
                Point tr1_front_0 = track1.front();
                Point tr1_front_1 = track1.at(1);
                Point tr1_back_0 = track1.back();
                Point tr1_back_1 = r_at(track1, 1);

                Point tr2_front_0 = track1.front();
                Point tr2_front_1 = track1.at(1);
                Point tr2_back_0 = track1.back();
                Point tr2_back_1 = r_at(track1, 1);

                bool track2InReverse = false;

                if( ag::calc_dist(tr1_front_0, tr2_front_0) + ag::calc_dist(tr1_back_0, tr2_back_0) >  ag::calc_dist(tr1_front_0, tr2_back_0) + ag::calc_dist(tr1_back_0, tr2_front_0) ){
                    std::swap(tr2_front_0, tr2_back_0);
                    std::swap(tr2_front_1, tr2_back_1);
                    track2InReverse = true;
                }

                for(int side = 0 ; side < 2 ; ++side){
                    const Point& p1_0 = ( side == 0 ? tr1_back_0 : tr1_front_0 );
                    const Point& p1_1 = ( side == 0 ? tr1_back_1 : tr1_front_1 );
                    const Point& p2_0 = ( side == 0 ? tr2_back_0 : tr2_front_0 );
                    const Point& p2_1 = ( side == 0 ? tr2_back_1 : tr2_front_1 );

                    Pose2D pose1 ( p1_0 , ag::get_angle(p1_1, p1_0) );
                    Pose2D pose2 ( p2_0 , ag::get_angle(p2_0, p2_1) );

                    bool addExtraPt1 = false;
                    bool addExtraPt2 = false;

                    auto connPath = InfieldTracksConnectorDubins::getDubinsPathConnection(pose1, pose2, turningRad, {&limitBoundary}, infieldBoundary, maxConnectionLength, false, nullptr);
                    if(connPath.empty()){
                        //try adding an extra point to track1
                        Point extraPt1 = getExtraPointInAdjacentHL( pose1, (side == 0 ? 1 : -1)*(i+1) , knownExtraPoses);
                        Point extraPt2;
                        Pose2D pose1_ed, pose2_ed;
                        if(extraPt1.isValid()){
                            pose1_ed = Pose2D( extraPt1 , ag::get_angle(p1_0, extraPt1) );
                            connPath = InfieldTracksConnectorDubins::getDubinsPathConnection(pose1_ed, pose2, turningRad, {&limitBoundary}, infieldBoundary, maxConnectionLength, false, nullptr);
                        }
                        if(!connPath.empty())
                            addExtraPt1 = true;
                        else {
                            //try adding an extra point to track2
                            if(track2InReverse)
                                extraPt2 = getExtraPointInAdjacentHL( pose2, (side == 0 ? -1 : 1)*(j+1), knownExtraPoses);
                            else
                                extraPt2 = getExtraPointInAdjacentHL( pose2, (side == 0 ? 1 : -1)*(j+1) , knownExtraPoses);
                            if(extraPt2.isValid()){
                                pose2_ed = Pose2D( extraPt2 , ag::get_angle(extraPt2, p2_0) );
                                connPath = InfieldTracksConnectorDubins::getDubinsPathConnection(pose1, pose2_ed, turningRad, {&limitBoundary}, infieldBoundary, maxConnectionLength, false, nullptr);
                                if(!connPath.empty())
                                    addExtraPt2 = true;
                            }
                        }
                        if( connPath.empty() && extraPt1.isValid() && extraPt2.isValid() ){ //try adding an extra point to track1 and track2
                            connPath = InfieldTracksConnectorDubins::getDubinsPathConnection(pose1_ed, pose2_ed, turningRad, {&limitBoundary}, infieldBoundary, maxConnectionLength, false, nullptr);
                            addExtraPt1 = addExtraPt2 = true;
                        }
                        if(connPath.empty())
                            continue;
                    }

                    std::vector<Point> newTrack;
                    int deltaInd1 = (addExtraPt1 ? 0 : 1);
                    int deltaInd2 = (addExtraPt2 ? 0 : 1);
                    newTrack.reserve(track1.size() + track2.size() + connPath.size() - deltaInd1 - deltaInd2);
                    if(side == 0){
                        newTrack.insert(newTrack.end(), track1.begin(), track1.end()-deltaInd1);
                        newTrack.insert(newTrack.end(), connPath.begin(), connPath.end());
                        if(track2InReverse)
                            newTrack.insert(newTrack.end(), track2.begin()+deltaInd2, track2.end());
                        else
                            newTrack.insert(newTrack.end(), track2.rbegin()+deltaInd2, track2.rend());
                    }
                    else{
                        newTrack.insert(newTrack.end(), track1.rbegin(), track1.rend()-deltaInd1);
                        newTrack.insert(newTrack.end(), connPath.begin(), connPath.end());
                        if(track2InReverse)
                            newTrack.insert(newTrack.end(), track2.rbegin()+deltaInd2, track2.rend());
                        else
                            newTrack.insert(newTrack.end(), track2.begin()+deltaInd2, track2.end());
                    }

                    tracks.emplace_back( newTrack );
                    std::reverse(newTrack.begin(), newTrack.end());
                    tracks.emplace_back( newTrack );
                }
            }
        }
    }

    auto getEndIndex = [&limitBoundary, &infieldBoundary, &turningRad](const std::vector<Point>& track)->int{

        if(track.size() < 2)
            return -1;
        if(turningRad < 1e-9)
            return track.size() -1;
        for(int i = track.size()-1; i > 0 ; --i){
            Point pt = ag::getPointInLineAtDist(track.at(i), track.at(i-1), -turningRad);
            if( !limitBoundary.points.empty() && !ag::in_polygon(pt, limitBoundary) )
                continue;
            if( !infieldBoundary.points.empty() && ag::in_polygon(pt, infieldBoundary) )
                continue;
            return i;
        }
        return -1;
    };

    auto getStartIndex = [&limitBoundary, &infieldBoundary, &turningRad](std::vector<Point>& track, const Pose2D& poseRef, int endIndex)->int{

        if(turningRad < 1e-9)
            return 0;
        int ind = ag::addSampleToGeometryClosestToPoint(track, poseRef, 1);
        if(ind < 0)
            return endIndex;
        const auto& ptRef = track.at(ind);
        for(int i = endIndex-1; i > 0 ; --i){
            const auto& pt = track.at(i);
            if( ag::calc_dist(pt, ptRef) < 2*turningRad )
                return i+1;
        }
        return 0;
    };

    auto getCutIndexes = [&turningRad](std::vector<Point>& track1, int startInd1, int endInd1, int& cutInd1,
                                       std::vector<Point>& track2, int startInd2, int endInd2, int& cutInd2){

        cutInd1 = cutInd2 = -1;
        Point startPt1 = track1.at(startInd1);
        Point startPt2 = track2.at(startInd2);
        Point endPt1 = track1.at(endInd1);
        Point endPt2 = track2.at(endInd2);
        int ind1 = ag::addSampleToGeometryClosestToPoint(track1, startPt2, 1);
        int ind2 = ag::addSampleToGeometryClosestToPoint(track2, startPt1, 1);
        if(ind1 <= 0 || ind2 <= 0)
            return;
        if(ind1 <= startInd1){
            cutInd1 = ind1;
            ind1 = ag::addSampleToGeometryClosestToPoint(track1, endPt2, 1);
            if(ind1 <= cutInd1)
                cutInd2 = endInd2;
            else
                cutInd2 = startInd2;
        }
        else{
            cutInd2 = ind2;
            ind2 = ag::addSampleToGeometryClosestToPoint(track2, endPt1, 1);
            if(ind2 <= cutInd2)
                cutInd1 = endInd1;
            else
                cutInd1 = startInd1;
        }
    };

    if(!hl.tracks.empty()){
        tracks.reserve(tracks.size() + 2 * ( hl.tracks.size() - 1) * 3 - 1);
        for(int i = 0 ; i < hl.tracks.size() ; ++i){
            auto track1_fwd = hl.tracks.at(i).points;

            if(track1_fwd.size() < 2)
                continue;

            auto track1_rev = track1_fwd;
            std::reverse(track1_rev.begin(), track1_rev.end());


            for(int side = 0 ; side < 2 ; ++side){

                std::vector<Point>& track1 = ( side == 0 ? track1_fwd : track1_rev );

                if(ag::getLocationInLine(track1.front(), track1.back(), pose_start) > 0)
                    continue;

                int indEnd1 = getEndIndex(track1);
                if(indEnd1 < 0)
                    continue;
                int indStart1 = getStartIndex(track1, pose_start, indEnd1);

                for(int j = i-2 ; j < (int)hl.tracks.size() && j <= i+2 ; ++j){
                    if(j < 0)
                        continue;

                    auto track2 = hl.tracks.at(j).points;
                    if(track2.size() < 2)
                        continue;

                    Point tr1_front_0 = track1.front();
                    Point tr1_back_0 = track1.back();

                    Point tr2_front_0 = track2.front();
                    Point tr2_back_0 = track2.back();

                    if( ag::calc_dist(tr1_front_0, tr2_front_0) + ag::calc_dist(tr1_back_0, tr2_back_0) >  ag::calc_dist(tr1_front_0, tr2_back_0) + ag::calc_dist(tr1_back_0, tr2_front_0) )
                        std::reverse(track2.begin(), track2.end());

                    if(ag::getLocationInLine(track2.front(), track2.back(), pose_end) > 0)
                        continue;

                    int indEnd2 = getEndIndex(track2);
                    if(indEnd2 < 0)
                        continue;

                    int indStart2 = getStartIndex(track2, pose_end, indEnd2);

                    auto track1ed = track1;
                    auto track2ed = track2;

                    int indCut1, indCut2;
                    getCutIndexes(track1ed, indStart1, indEnd1, indCut1, track2ed, indStart2, indEnd2, indCut2);

                    if(indCut1 < 0 || indCut1 >= track1.size() || indCut2 < 0 || indCut2 >= track2.size())
                        continue;

                    if(indCut1 > 0)
                        track1ed.erase(track1ed.begin()+indCut1+1, track1ed.end());
                    if(indCut2 > 0)
                        track2ed.erase(track2ed.begin()+indCut2+1, track2ed.end());


                    const Point& p1_n = track1ed.back();
                    const Point& p1_prev = ( track1ed.size() > 1 ? r_at(track1ed, 1) : pose_start.point() );
                    const Point& p2_n = track2ed.back();
                    const Point& p2_prev = ( track2ed.size() > 1 ? r_at(track2ed, 1) : pose_end.point() );

                    Pose2D pose1 ( p1_n , ag::get_angle(p1_prev, p1_n) );
                    Pose2D pose2 ( p2_n , ag::get_angle(p2_n, p2_prev) );

                    InfieldTracksConnectorDubins::ExtensionParameters extParams = InfieldTracksConnectorDubins::ExtensionParameters::NoExtention();
                    auto connPath = InfieldTracksConnectorDubins::getDubinsPathConnection(pose1, pose2, turningRad, {&limitBoundary}, infieldBoundary, maxConnectionLength, false, &extParams);
                    if(connPath.empty())
                        continue;

                    tracks.push_back(std::vector<Point>());
                    std::vector<Point>& newTrack = tracks.back();
                    newTrack.reserve(track1ed.size() + track2ed.size() + connPath.size() - 2);
                    newTrack.insert(newTrack.end(), track1ed.begin(), track1ed.end()-1);
                    newTrack.insert(newTrack.end(), connPath.begin(), connPath.end());
                    newTrack.insert(newTrack.end(), track2ed.rbegin()+1, track2ed.rend());

                }
            }
        }
    }

    if(tracks.empty())
        return {};

    //try connecting using the tracks
    auto ret_1 = getConnectionOverTracks(pose_start, pose_end, turningRad, tracks, limitBoundary, infieldBoundary, maxConnectionLength);
    if(!ret_1.empty())
        maxConnectionLength = ag::getGeometryLength(ret_1) / 1.5;
    auto ret_2 = getConnectionOverTracks_2(pose_start, pose_end, turningRad, tracks, limitBoundary, infieldBoundary, maxConnectionLength);
    if(!ret_2.empty())
        return ret_2;
    if(!ret_1.empty())
        return ret_1;

    return {};

}

std::vector<Point> InfieldTracksConnectorDef::getConnectionOverSamePartialHeadlandWithDubins(const PartialHeadland &hl,
                                                                                             const Pose2D &pose_start,
                                                                                             const Pose2D &pose_end,
                                                                                             double turningRad,
                                                                                             const Polygon &limitBoundary,
                                                                                             const Polygon &infieldBoundary,
                                                                                             double maxConnectionLength,
                                                                                             double& pathLength) const
{
    std::vector<Point> ret;
    if(hl.tracks.empty())
        return ret;
    auto trackRef = hl.tracks.at( hl.tracks.size() / 2 ).points;
    if(trackRef.empty())
        return ret;
    ag::unsample_linestring(trackRef);
    int ind0 = ag::addSampleToGeometryClosestToPoint(trackRef, pose_start, 1);
    if(ind0 < 0)
        return ret;
    auto size_prev = trackRef.size();
    int indn = ag::addSampleToGeometryClosestToPoint(trackRef, pose_end, 1);
    if(indn < 0)
        return ret;
    if(size_prev < trackRef.size() && indn <= ind0)
        ++ind0;
    if(ind0 == indn)
        return ret;
    bool inReverse = ind0 > indn;

    Pose2D intermediatePose;
    intermediatePose.point() = ag::getPointAtHalfLength(trackRef).first;
    int indMid = ag::addSampleToGeometryClosestToPoint(trackRef, intermediatePose, 1);
    if(indMid < 1 || indMid+1 >= trackRef.size())
        return ret;

    if(!inReverse)
        intermediatePose.angle = ag::get_angle( trackRef.at(indMid), trackRef.at(indMid+1) );
    else
        intermediatePose.angle = ag::get_angle( trackRef.at(indMid), trackRef.at(indMid-1) );

    double l0, ln;
    InfieldTracksConnectorDubins::ExtensionParameters extParams = InfieldTracksConnectorDubins::ExtensionParameters::NoExtention();

    auto path0 = InfieldTracksConnectorDubins::getDubinsPathConnection(pose_start, intermediatePose, turningRad, {&limitBoundary}, infieldBoundary, maxConnectionLength, false, &extParams, &l0);
    if(path0.empty())
        return ret;

    auto pathn = InfieldTracksConnectorDubins::getDubinsPathConnection(intermediatePose, pose_end, turningRad, {&limitBoundary}, infieldBoundary, maxConnectionLength, false, &extParams, &ln);
    if(pathn.empty())
        return ret;

    pathLength = l0 + ln;
    ret = path0;
    ret.insert(ret.end(), pathn.begin()+1, pathn.end() );
    return ret;
}

std::vector<Point> InfieldTracksConnectorDef::getConnectionOverAdjacentHeadlands(const std::vector<PartialHeadland>& hls,
                                                                                 size_t hl_ind_start,
                                                                                 size_t hl_ind_end,
                                                                                 const Pose2D& pose_start,
                                                                                 const Pose2D& pose_end,
                                                                                 double turningRad,
                                                                                 const Polygon& limitBoundary,
                                                                                 const Polygon& infieldBoundary,
                                                                                 double maxConnectionLength) const{

    std::vector<Point> ret;

    InfieldTracksConnectorDubins::ExtensionParameters dubinsConnExtParams = InfieldTracksConnectorDubins::getDefaultExtensionParameters(turningRad);
    dubinsConnExtParams.distPasses = std::max((size_t)3, dubinsConnExtParams.distPasses);

    auto connectionSequences = PartialHeadland::getHeadlandConnectionSequences(hls, hl_ind_start, hl_ind_end);
    for(auto& seq : connectionSequences){
        if(seq.size() != 2)
            continue;

        for(int side1 = 0 ; side1 < 2 ; side1++){
            auto& hl = side1 == 0 ? hls.at(hl_ind_start) : hls.at(hl_ind_end);

            if( !hl.isConnectingHeadland() || hl.tracks.empty() )
                continue;

            auto& track = hl.tracks.back().points;
            if(track.size() < 2)
                continue;

            Pose2D pose1, pose2;
            if(side1 == 0 ){
                pose1 = pose_end;
                pose1.angle += M_PI;
                pose2 = pose_start;
                pose2.angle += M_PI;
            }
            else{
                pose1 = pose_start;
                pose2 = pose_end;
            }

            for(int side2 = 0 ; side2 < 2 ; side2++){
                Pose2D poseTmp;
                if(side2 == 0){
                    poseTmp.point() = track.front();
                    poseTmp.angle = ag::get_angle( track.front(), track.at(1) );
                }
                else{
                    poseTmp.point() = track.back();
                    poseTmp.angle = ag::get_angle( track.back(), r_at(track, 1) );
                }

                double path1Length;
                auto path1 = InfieldTracksConnectorDubins::getDubinsPathConnection(pose1, poseTmp, turningRad, {&limitBoundary}, infieldBoundary, maxConnectionLength, false, &dubinsConnExtParams, &path1Length);
                if(path1.empty())
                    continue;

                double maxConnectionLengthTmp = maxConnectionLength;
                if(maxConnectionLengthTmp > 1e-9)
                    maxConnectionLengthTmp = std::max(maxConnectionLength - path1Length, 0.001);

                auto path2 = InfieldTracksConnectorDubins::getDubinsPathConnection(poseTmp, pose2, turningRad, {&limitBoundary}, infieldBoundary, maxConnectionLength, false, &dubinsConnExtParams);
                if(path2.empty()){
                    PointVecVec tracks {track};
                    if(side2 == 0)
                        std::reverse(tracks.back().begin(), tracks.back().end());
                    path2 = getConnectionOverTracks(poseTmp, pose2, turningRad, tracks, limitBoundary, infieldBoundary, maxConnectionLengthTmp);
                }

                if(path2.empty())
                    continue;

                ret = path1;
                ret.insert(ret.end(), path2.begin()+1, path2.end());
                if(side1 == 0)
                    std::reverse(ret.begin(), ret.end());

                maxConnectionLength = ag::getGeometryLength(ret);
            }
        }
    }
    return ret;
}

std::vector<std::vector<Point> > InfieldTracksConnectorDef::getSubTracksFromClosedTracks(const std::vector<Track> &tracks_in, const Pose2D &pose_start, const Pose2D &pose_end)
{
    std::vector< const std::vector<Point>* > tracks_tmp;
    for(const auto& track : tracks_in){
        tracks_tmp.emplace_back( &track.points );
    }
    return getSubTracksFromClosedTracks(tracks_tmp, pose_start, pose_end);
}

std::vector<std::vector<Point> > InfieldTracksConnectorDef::getSubTracksFromClosedTracks(const std::vector<std::vector<Point> > &tracks_in,
                                                                                         const Pose2D &pose_start, const Pose2D &pose_end)
{
    std::vector< const std::vector<Point>* > tracks_tmp;
    for(const auto& track : tracks_in){
        tracks_tmp.emplace_back( &track );
    }
    return getSubTracksFromClosedTracks(tracks_tmp, pose_start, pose_end);
}

std::vector<std::vector<Point>> InfieldTracksConnectorDef::getSubTracksFromClosedTracks(const std::vector< const std::vector<Point>* > &tracks_in,
                                                                                        const Pose2D &pose_start, const Pose2D &pose_end)
{
    std::vector<std::vector<Point>> ret;
    ret.reserve(tracks_in.size()*2);

    for (auto pTrack : tracks_in){
        const auto& track = *pTrack;

        if(track.size() < 2)
            continue;

        Polygon poly;
        poly.points = track;
        if( !ag::isPolygonClosed(poly, true) ){
            continue;
        }
        ag::correct_polygon(poly);

        //ag::unsample_polygon(poly);


        Point pIntStart = pose_start.point();
        Point pRef1 = ag::getPointAtDist(pose_start, 1);
        std::vector<Point> intersections = ag::get_intersection(pIntStart, pRef1, poly.points, true, false, true);
        if(!intersections.empty()){
            pIntStart = intersections.front();
            ag::addSampleToGeometryClosestToPoint(poly.points, pIntStart, 1);
        }
        else
            pIntStart = Point::invalidPoint();

        Point pIntEnd = pose_end.point();
        pRef1 = ag::getPointAtDist(pose_end, -1);
        intersections = ag::get_intersection(pIntEnd, pRef1, poly.points, true, false, true);
        if(!intersections.empty()){
            pIntEnd = intersections.front();
            ag::addSampleToGeometryClosestToPoint(poly.points, pIntEnd, 1);
        }
        else
            pIntEnd = Point::invalidPoint();


        ag::addSampleToGeometryClosestToPoint(poly.points, pose_start, 1);
        ag::addSampleToGeometryClosestToPoint(poly.points, pose_end, 1);

        double maxShortestLength = -1, maxLongestLength = -1;
        std::vector<Point> shortestPath, longestPath;
        for(int side_start = 0 ; side_start < 2 ; ++side_start){
            const auto& pRefStart = ( side_start == 0 ? pose_start.point() : pIntStart );
            if(!pRefStart.isValid())
                continue;
            for(int side_end = 0 ; side_end < 2 ; ++side_end){
                const auto& pRefEnd = ( side_start == 0 ? pose_end.point() : pIntEnd );
                if(!pRefEnd.isValid())
                    continue;
                auto path = ag::getShortestGeometryPart(poly.points, pRefStart, pRefEnd);
                double pathLength = ag::getGeometryLength(path);
                if( maxShortestLength < pathLength ){
                    maxShortestLength = pathLength;
                    shortestPath = path;
                }

                path = ag::getLongestGeometryPart(poly.points, pRefStart, pRefEnd);
                pathLength = ag::getGeometryLength(path);
                if( maxLongestLength < pathLength ){
                    maxLongestLength = pathLength;
                    longestPath = path;
                }
            }
        }
        if(!shortestPath.empty())
            ret.emplace_back( shortestPath );
        if(!longestPath.empty())
            ret.emplace_back( longestPath );
    }
    return ret;

}

void InfieldTracksConnectorDef::getTracksFromHeadlandSequence(const std::vector<PartialHeadland> &hls,
                                                              const std::vector<size_t> &sequence,
                                                              std::vector<std::vector<Point> > &tracks,
                                                              double turningRad,
                                                              double maxConnectionLength)
{
    tracks.clear();
    try{
        std::vector<PartialHeadland> hls_sorted = sortHeadlandsFollowingSequence(hls, sequence);
        size_t maxNumTracks = 1;
        for(int i = 0 ; i < hls_sorted.size() ; ++i){
            auto& hl = hls_sorted.at(i);
            if(hl.tracks.empty())
                return;
            if(!hl.isConnectingHeadland()){
                maxNumTracks = std::max(maxNumTracks, hl.tracks.size());
                if(i > 0){
                    auto pRef = hls_sorted.at(i-1).tracks.front().points.back();
                    for(auto& track: hl.tracks){
                        int ind = geometry::addSampleToGeometryClosestToPoint(track.points, pRef, 1);
                        if(ind > 0)
                            track.points.erase(track.points.begin(), track.points.begin()+ind);
                        pRef = track.points.front();
                    }
                }
                if(i+1 < hls_sorted.size()){
                    auto pRef = hls_sorted.at(i+1).tracks.front().points.front();
                    for(auto& track: hl.tracks){
                        int ind = geometry::addSampleToGeometryClosestToPoint(track.points, pRef, 1);
                        if(ind >= 0 && ind+1 < track.points.size())
                            track.points.erase(track.points.begin()+ind+1, track.points.end());
                        pRef = track.points.back();
                    }
                }
            }
        }

        tracks.resize(maxNumTracks, {});
        bool isCyclic = sequence.size() > 1 && sequence.front() == sequence.back();
        for(size_t i = 0 ; i < tracks.size() ; ++i){
            auto& track_out = tracks.at(i);
            double intemediateLength = -1; //holds the length of the intermediate track segments (i.e. excluding the tracks from the 1st and last hl)
            for(int j = 0 ; j < hls_sorted.size(); ++j){
                size_t size_prev = track_out.size();
                auto& hl = hls_sorted.at(j);
                size_t ind_track = std::min(i, hl.tracks.size()-1);
                auto& trackPts = hl.tracks.at( ind_track ).points;
                if(trackPts.empty())
                    continue;

                if(hl.isConnectingHeadland()){
                    track_out.insert(track_out.end(), trackPts.begin(), trackPts.end());
                }

                else{
                    //if necessary, add extra points before and after the track so that the connection is better

                    std::vector<Point> extraPointsPrev = {trackPts.front()};
                    int ind_hl_prev = (j == 0 && isCyclic ? hls_sorted.size()-1 : j-1);
                    if(ind_hl_prev >= 0){
                        auto& hl_prev = hls_sorted.at(ind_hl_prev);
                        size_t ind_track_ref = ind_track;
                        size_t ind_track_prev = std::min(i, hl_prev.tracks.size()-1);
                        auto& track_prev = hl_prev.tracks.at( ind_track_prev );
                        while(ind_track_ref > 0 && geometry::calc_dist(extraPointsPrev.back(), track_prev.points.back()) > 1.5 * hl.tracks.at(ind_track_ref).width){
                            --ind_track_ref;
                            extraPointsPrev.push_back( hl.tracks.at(ind_track_ref).points.front() );
                        }
                    }

                    track_out.insert(track_out.end(), extraPointsPrev.rbegin(), extraPointsPrev.rend());
                    track_out.insert(track_out.end(), trackPts.begin()+1, trackPts.end());

                    size_t ind_hl_next = ( j+1 >= hls_sorted.size() && isCyclic ? 0 : j+1 );
                    if(ind_hl_next < hls_sorted.size()){
                        auto& hl_next = hls_sorted.at(ind_hl_next);
                        size_t ind_track_ref = ind_track;
                        size_t ind_track_next = std::min(i, hl_next.tracks.size()-1);
                        auto& track_next = hl_next.tracks.at( ind_track_next );
                        while(ind_track_ref > 0 && geometry::calc_dist(track_out.back(), track_next.points.front()) > 1.5 * hl.tracks.at(ind_track_ref).width){
                            --ind_track_ref;
                            track_out.push_back( hl.tracks.at(ind_track_ref).points.back() );
                        }
                    }
                }
                if(maxConnectionLength > 1e-9 && intemediateLength > -1e-6 && j < hls_sorted.size() && track_out.size() > size_prev){
                    intemediateLength += ag::getGeometryLength(track_out, size_prev);
                    if(intemediateLength - 2*turningRad > maxConnectionLength){
                        track_out.clear();
                        break;
                    }
                }
                if(track_out.size() != size_prev)
                    intemediateLength = 0;
            }
        }

        for(size_t i = 0; i < tracks.size() ; ++i){
            auto& track = tracks.at(i);
            if(track.empty()){
                tracks.erase(tracks.begin()+i);
                --i;
                continue;
            }
            if(isCyclic)//close the track
                track.push_back(track.front());
        }
    }
    catch(...){
        tracks.clear();
    }
}

void InfieldTracksConnectorDef::getTracksFromHeadlandSequences(const std::vector<PartialHeadland> &hls,
                                                               const std::vector<std::vector<size_t> > &connectionSequences,
                                                               std::vector<std::vector<Point> > &tracks,
                                                               double turningRad,
                                                               double maxConnectionLength)
{
    tracks.clear();
    for(auto& seq : connectionSequences){
        std::vector<std::vector<Point> > tracksTmp;
        getTracksFromHeadlandSequence(hls, seq, tracksTmp, turningRad, maxConnectionLength);
        tracks.insert(tracks.end(), tracksTmp.begin(), tracksTmp.end());
    }

}

std::vector<PartialHeadland> InfieldTracksConnectorDef::sortHeadlandsFollowingSequence(const std::vector<PartialHeadland> &hls, const std::vector<size_t> &sequence)
{
    std::vector<PartialHeadland> hls_sorted;
    if(sequence.empty())
        return hls_sorted;
    try{
        size_t hlsSize = sequence.size() - ( sequence.front() == sequence.back() );
        hls_sorted.reserve(hlsSize);
        for(size_t i = 0 ; i < hlsSize ; ++i)
            hls_sorted.push_back(hls.at( sequence.at(i) ));


        //reorder tracks
        for(int i = 0 ; i < hls_sorted.size() ; ++i){
            auto& hl = hls_sorted.at(i);
            if(hl.tracks.size() < 2)
                continue;

            auto& track0 = hl.tracks.front();
            auto& trackn = hl.tracks.back();

            //reorder the tracks so that the first one is the one closest one to the previous/next headland

            auto& hl_ref = ( i > 0 ? hls_sorted.at(i-1) : hls_sorted.at(i+1) );
            double dist0 = geometry::calc_dist_to_linestring(hl_ref.boundary.points, track0.points.front());
            double distn = geometry::calc_dist_to_linestring(hl_ref.boundary.points, trackn.points.front());
            double dist0_2 = geometry::calc_dist_to_linestring(hl_ref.boundary.points, track0.points.back());
            double distn_2 = geometry::calc_dist_to_linestring(hl_ref.boundary.points, trackn.points.back());

            if( dist0+distn > dist0_2+distn_2){//we have to check both sides because we do not which one is the adjacen to the reference (next/prev) headland
                std::swap(dist0, dist0_2);
                std::swap(distn, distn_2);
            }

            if( dist0 > distn )
                std::reverse(hl.tracks.begin(), hl.tracks.end());

        }

        //reorder track points
        for(int i = 0 ; i < hls_sorted.size() ; ++i){
            auto& hl = hls_sorted.at(i);
            for(int j = 0 ; j < hl.tracks.size() ; ++j){
                auto& track = hl.tracks.at(j);
                auto& trackPts = track.points;
                if(trackPts.empty()){
                    hl.tracks.erase( hl.tracks.begin()+j );
                    --j;
                    continue;
                }
                if(i > 0){
                    Point refPoint = hls_sorted.at(i-1).tracks.front().points.back();
                    double dist0 = geometry::calc_dist(refPoint, trackPts.front());
                    double distn = geometry::calc_dist(refPoint, trackPts.back());
                    if( dist0 > distn )
                        std::reverse(trackPts.begin(), trackPts.end());
                }
                else if (i+1 < hls_sorted.size()){
                    Point refPoint = hls_sorted.at(i+1).tracks.front().points.back();
                    double dist0 = geometry::calc_dist(refPoint, trackPts.front());
                    double distn = geometry::calc_dist(refPoint, trackPts.back());
                    if( distn > dist0 )
                        std::reverse(trackPts.begin(), trackPts.end());
                }
            }
        }

    }
    catch(...){
        hls_sorted.clear();
    }
    return hls_sorted;
}


}

