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
 

#include "arolib/planning/infieldtracksconnector.hpp"

namespace arolib{

namespace ag = arolib::geometry;

IInfieldTracksConnector::IInfieldTracksConnector(const std::string &childName, Logger *parentLogger)
    : LoggingComponent(parentLogger, childName)
{

}

std::vector<Point> IInfieldTracksConnector::getConnection(const Subfield &sf,
                                                          const Machine &machine,
                                                          const Pose2D &pose_start,
                                                          const Pose2D &pose_end,
                                                          double turningRad,
                                                          double extraDist,
                                                          double speed_start,
                                                          double maxConnectionLength) const
{
    return getConnection( machine,
                          pose_start,
                          pose_end,
                          turningRad,
                          extraDist,
                          sf.boundary_outer,
                          sf.boundary_inner,
                          sf.headlands,
                          speed_start,
                          maxConnectionLength);
}

std::vector<Point> IInfieldTracksConnector::getConnection(const Machine &machine,
                                                          const Pose2D &pose_start,
                                                          const std::vector<Point> &track,
                                                          bool checkBothSides,
                                                          double turningRad,
                                                          double extraDist,
                                                          const Polygon &limitBoundary,
                                                          const Polygon &infieldBoundary,
                                                          const Headlands &headlands,
                                                          double speed_start,
                                                          double maxConnectionLength) const
{
    std::vector<Point> ret;
    auto track_us = track;
    if( !ag::unsample_linestring(track_us) || track_us.size() < 2 )
        return ret;

    Pose2D pose_end;
    pose_end.point() = track_us.front();
    pose_end.angle = ag::get_angle(track_us.front(), track_us.at(1));
    ret = getConnection(machine,
                        pose_start,
                        pose_end,
                        turningRad,
                        extraDist,
                        limitBoundary,
                        infieldBoundary,
                        headlands,
                        speed_start,
                        maxConnectionLength);

    if(!checkBothSides)
        return ret;

    pose_end.point() = track_us.back();
    pose_end.angle = ag::get_angle(track_us.back(), r_at(track_us, 1));

    if(!ret.empty()){
        auto length1 = ag::getGeometryLength(ret);
        if(maxConnectionLength > 1e-9)
            maxConnectionLength = std::min(maxConnectionLength, length1);
        else
            maxConnectionLength = length1;
    }

    std::vector<Point> ret2 = getConnection(machine,
                                            pose_start,
                                            pose_end,
                                            turningRad,
                                            extraDist,
                                            limitBoundary,
                                            infieldBoundary,
                                            headlands,
                                            speed_start,
                                            maxConnectionLength);

    if(ret2.empty())
        return ret;
    if(ret.empty() || ag::getGeometryLength(ret) > ag::getGeometryLength(ret2))
        ret = ret2;

    return ret;

}

InfieldTracksConnectorDef::InfieldTracksConnectorDef(Logger *parentLogger)
    : IInfieldTracksConnector(__FUNCTION__, parentLogger)
{

}

std::vector<Point> InfieldTracksConnectorDef::getConnection(const Machine &machine,
                                                            const Pose2D &_pose_start,
                                                            const Pose2D &_pose_end,
                                                            double turningRad,
                                                            double extraDist,
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

    //adjust turning radius
    if(turningRad < 1e-9)
        turningRad = machine.getTurningRadius();
    turningRad = std::max(0.0, turningRad);

    //adjust start/end poses
    extraDist = std::max(0.0, extraDist);
    auto pose_start = _pose_start;
    auto pose_end = _pose_end;
    if(extraDist > 0){
        pose_start.point() = ag::rotate(pose_start, Point(extraDist, 0) + pose_start, pose_start.angle);
        pose_end.point() = ag::rotate(pose_end, Point(extraDist, 0) + pose_end, pose_end.angle - M_PI);
    }

    if( !limitBoundary.points.empty() ){
        auto polyTmp = limitBoundary;
        ag::offsetPolygon(polyTmp, limitBoundary, 0.01*turningRad, true);
        if( !ag::in_polygon(pose_start, limitBoundary) || !ag::in_polygon(pose_end, limitBoundary) ){
            logger().printError(__FUNCTION__, "(extended) start and/or end poses lie outside the limit boundary");
            return ret;
        }
    }

    //try connecting directly using dubins path
    ret = ag::calcDubinsPath(pose_start, pose_end, turningRad, 10, true);

    if(!ret.empty()){
        //ag::unsample_linestring(ret);
        if( isPathValid(ret, limitBoundary, infieldBoundary, maxConnectionLength) ){
            return ret;
        }
    }

    //try connecting over complete/surrounding headland
    ret = getConnectionOverHeadland(headlands.complete,
                                    pose_start,
                                    pose_end,
                                    turningRad,
                                    limitBoundary,
                                    infieldBoundary,
                                    maxConnectionLength);

    if(!ret.empty())
        return ret;


    //try connecting outside the IF boundary
    ret = getConnectionOutsideBoundary(infieldBoundary,
                                       pose_start,
                                       pose_end,
                                       turningRad,
                                       limitBoundary,
                                       infieldBoundary,
                                       maxConnectionLength);

    if(!ret.empty())
        return ret;

    return ret;


}

bool InfieldTracksConnectorDef::isPathValid(const std::vector<Point> &path, const Polygon &limitBoundary, const Polygon &infieldBoundary, double maxConnectionLength) const
{
    if(maxConnectionLength > 1e-9 && ag::getGeometryLength(path) > maxConnectionLength)
        return false;

    if(!limitBoundary.points.empty()){
        if( ag::intersects(path, limitBoundary.points, false, false) ){
            return false;
        }
    }

    Polygon boundaryEd;
    if( ag::offsetPolygon(infieldBoundary, boundaryEd, 1e-3, false) && !boundaryEd.points.empty()){
        if( ag::intersects(path, boundaryEd, false, false) ){
            return false;
        }
    }

    return true;
}

std::vector<Point> InfieldTracksConnectorDef::getConnectionOverTracks(const Pose2D &pose_start,
                                                                      const Pose2D &pose_end,
                                                                      double turningRad,
                                                                      const std::vector<std::vector<Point> > &tracks,
                                                                      const Polygon &limitBoundary,
                                                                      const Polygon &infieldBoundary,
                                                                      double maxConnectionLength) const
{

    auto isPointValid = [turningRad](const Pose2D& pose, const Point& p, const Point& p2, bool towardsPose) -> bool{
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
            double x_thershold_1 = turningRad * std::cos(ang1_ed);
            double x_thershold_2 = turningRad * ( std::fabs(ang1_tan) + 0.1 );
            double y_thershold = turningRad * ( 1 + std::sin(ang1_ed) );

            if( std::fabs(p0_transf.x) >= x_thershold_1 &&
                    std::fabs(p0_transf.x) <= x_thershold_2 &&
                    std::fabs(p0_transf.y) >= y_thershold )
                return true;

            return false;
        }


        //check 'save' x-axis position condition
        if( p0_transf.x >= 2*turningRad - std::sin( std::fabs(ang1) ) )
            return true;

        //move to y_axist positive quadrant
        p0_transf.y = std::fabs( p0_transf.y );
        p1_transf.y = std::fabs( p1_transf.y );
        ang1 = std::fabs(ang1);
        ang2 = std::fabs(ang2);

        //now we work in the (+,+) quadrant

        //check x-axis position contition 1
        if( p0_transf.x < turningRad * std::sin(ang1) )
            return false;

        //check y-axis position contition 1
        if( p0_transf.y < turningRad * (1 - std::cos(ang1) ) )
            return false;

        //check with respect to a 2xrad circle tangent to pRef0
        if( ag::calc_dist( p0_transf, Point(0, 2*turningRad) ) < 2*turningRad-1e-5 )
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
//        double distCmpr1 = std::fabs(ang1) * M_1_PI * 2 * turningRad;
//        if(dist1 < distCmpr1)
//            return false;
//        double distCmpr2 = dist1 * std::cos(ang2);
//        if(distCmpr2 < turningRad)
//            return false;
//        return true;

        //---

        //return ag::calc_dist(pose, p) > turningRad;
    };

    std::vector<Point> ret;
    std::map<double, std::vector<Point>> tracksSorted;

    for(auto & track : tracks){
        auto track_ed = track;
        ag::unsample_linestring(track_ed);

        if(track.size() < 2)
            continue;

        int ind1 = ag::addSampleToGeometryClosestToPoint(track_ed, pose_start, 1);
        if(ind1 < 0 || ind1 > track_ed.size())
            continue;
        auto size_prev = track_ed.size();
        int ind2 = ag::addSampleToGeometryClosestToPoint(track_ed, pose_end, 1);
        if(ind2 < 0 || ind2 > track_ed.size())
            continue;
        if(size_prev != track_ed.size() && ind2 < ind1 )
            ind1++;

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
    for(auto& tr_it : tracksSorted){
        auto& track = tr_it.second;
        track = ag::sample_geometry(track, turningRad > 1e-9 ? turningRad * 0.25 : 0.05 * ag::getGeometryLength(track) );

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
                found = true; break;
            }
        }
        if(!found)
            continue;
        if(track.size() < 2)
            continue;

        if(turningRad > 0){
            push_front(track, ag::getPointAtDist(pose_start, 0.5*turningRad));
            track.push_back(ag::getPointAtDist(pose_end, -0.5*turningRad));
        }
        push_front(track, pose_start.point());
        track.push_back(pose_end.point());

        ag::unsample_linestring(track);
        if(!isPathValid(track, limitBoundary, infieldBoundary, maxConnectionLength)){
            continue;
        }

        if(turningRad > 1e-6){//@todo this steps is relativelly time expensive
            ag::PathSmoother ppss;
            track = ppss.smoothenPath(track, turningRad);
        }

        if(track.empty())
            continue;

        ag::unsample_linestring(track);
        if(isPathValid(track, limitBoundary, infieldBoundary, maxConnectionLength)){
            ret = track;
            break;
        }
    }

    return ret;
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

    //try using middle track from surrounding headland
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
    }

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


}

