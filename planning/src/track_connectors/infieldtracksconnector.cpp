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
 

#include "arolib/planning/track_connectors/infieldtracksconnector.hpp"

namespace arolib{

namespace ag = arolib::geometry;

IInfieldTracksConnector::IInfieldTracksConnector(const std::string &childName, std::shared_ptr<Logger> parentLogger)
    : LoggingComponent(parentLogger, childName)
{

}

std::vector<Point> IInfieldTracksConnector::getConnection(const Subfield &sf,
                                                          const Machine &machine,
                                                          const Pose2D &pose_start,
                                                          const Pose2D &pose_end,
                                                          double turningRad,
                                                          const std::pair<double, double> &extraDist,
                                                          double speed_start,
                                                          double maxConnectionLength) const
{


    //adjust start/end poses
    Pose2D pose_start_ext, pose_end__ext;
    this->getExtendedPoses(pose_start, pose_end, extraDist, pose_start_ext, pose_end__ext);
    if( !this->checkForMinPossibleLength(pose_start_ext, pose_end__ext, turningRad,
                                         sf.boundary_outer, sf.boundary_inner, maxConnectionLength) )
        return {};

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
                                                          const std::pair<double, double> &extraDist,
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

    Pose2D pose_end_1, pose_end_2;
    pose_end_1.point() = track_us.front();
    pose_end_1.angle = ag::get_angle(track_us.front(), track_us.at(1));

    if(checkBothSides){
        pose_end_2.point() = track_us.back();
        pose_end_2.angle = ag::get_angle(track_us.back(), r_at(track_us, 1));

        if(ag::calc_dist(pose_start, pose_end_1) > ag::calc_dist(pose_start, pose_end_2))
            std::swap(pose_end_1, pose_end_2);
    }

    if(maxConnectionLength > 1e-9){
        if(ag::calc_dist(pose_start, pose_end_1) >maxConnectionLength)
            return ret;
    }

    ret = getConnection(machine,
                        pose_start,
                        pose_end_1,
                        turningRad,
                        extraDist,
                        limitBoundary,
                        infieldBoundary,
                        headlands,
                        speed_start,
                        maxConnectionLength);

    if(!checkBothSides)
        return ret;

    if(!ret.empty()){
        auto length1 = ag::getGeometryLength(ret);
        if(maxConnectionLength > 1e-9)
            maxConnectionLength = std::min(maxConnectionLength, length1);
        else
            maxConnectionLength = length1;

        if(maxConnectionLength < ag::calc_dist(pose_start, pose_end_2) + 2*turningRad)
            return ret;
    }
    else if(maxConnectionLength > 1e-9 && ag::calc_dist(pose_start, pose_end_2) > maxConnectionLength)
        return ret;

    std::vector<Point> ret2 = getConnection(machine,
                                            pose_start,
                                            pose_end_2,
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

Polygon IInfieldTracksConnector::getExtendedLimitBoundary(const Subfield &sf, double /*turningRad*/) const
{
    Polygon ret;
    auto boundary = sf.boundary_outer;
    ag::correct_polygon(ret);
    if(ag::isPolygonValid(boundary) == ag::PolygonValidity::INVALID_POLYGON)
        return ret;

    if(!ag::offsetPolygon(boundary, ret, 0.01, true, 0))
        ret = boundary;
    return ret;
}

double IInfieldTracksConnector::getTurningRad(const Machine &machine, double turningRad){
    if(std::fabs(turningRad) < 1e-9)//no turning rad
        turningRad = -1;
    else{
        if(turningRad < -1e-9)//from machine
            turningRad = machine.getTurningRadius();
        turningRad = std::max(0.0, turningRad);
    }
    return turningRad;
}

bool IInfieldTracksConnector::isPathValid(const std::vector<Point> &path, const Polygon &limitBoundary, const Polygon &infieldBoundary, double maxConnectionLength)
{
    return isPathValid(path, {&limitBoundary}, infieldBoundary, maxConnectionLength);
}

bool IInfieldTracksConnector::isPathValid(const std::vector<Point> &path, const std::vector<const Polygon *> &limitBoundaries, const Polygon &infieldBoundary, double maxConnectionLength)
{
    if(path.empty())
        return false;

    if(maxConnectionLength > 1e-9 && ag::getGeometryLength(path) > maxConnectionLength)
        return false;

    for(auto &limitBoundary : limitBoundaries){
        if(!limitBoundary->points.empty()){
            if( ag::intersects(path, limitBoundary->points, false, false) ){
                return false;
            }
        }
    }

    auto boundary_if = getInfieldBoundaryForValidation(infieldBoundary, path.front(), path.back());

    if( !boundary_if.points.empty() && ag::intersects(path, boundary_if, false, false) )
        return false;

    return true;
}

Polygon IInfieldTracksConnector::getInfieldBoundaryForValidation(const Polygon &infieldBoundary, const Point &p0, const Point &pn)
{
    const double eps = 0.1;
    double offset = eps;
    Polygon poly;
    if(infieldBoundary.points.size() < 4)
        return poly;
    double d0 = ag::calc_dist_to_linestring(infieldBoundary.points, p0);
    double dn = ag::calc_dist_to_linestring(infieldBoundary.points, pn);

    if( d0 < eps || dn < eps || ag::in_polygon(p0, infieldBoundary) || ag::in_polygon(pn, infieldBoundary) )
        offset = std::max(d0, dn) + eps;

    if(!ag::offsetPolygon(infieldBoundary, poly, offset, false))
        poly = infieldBoundary;

    return poly;
}

void IInfieldTracksConnector::getExtendedPoses(const Pose2D& pose_start, const Pose2D& pose_end, const std::pair<double, double>& extraDist,
                                               Pose2D& pose_start_ed, Pose2D& pose_end_ed) const
{
    pose_start_ed = pose_start;
    pose_end_ed = pose_end;
    double extraDist0 = std::max(0.0, extraDist.first);
    double extraDistn = std::max(0.0, extraDist.second);
    if(extraDist0 > 0)
        pose_start_ed.point() = ag::getPointAtDist(pose_start, extraDist0);
    if(extraDistn > 0)
        pose_end_ed.point() = ag::getPointAtDist(pose_end, -extraDistn);
}

bool IInfieldTracksConnector::checkForMinPossibleLength(const Pose2D &pose_start,
                                                        const Pose2D &pose_end,
                                                        double turningRad,
                                                        const Polygon &limitBoundary,
                                                        const Polygon &infieldBoundary,
                                                        double maxConnectionLength) const
{
    if(maxConnectionLength < 1e-9)
        return true;
    if(turningRad > 1e-9){
        auto length = ag::calcDubinsPathLength(pose_start, pose_end, turningRad);
        if(length > 0)
            return length <= maxConnectionLength;
    }

    return ag::calc_dist(pose_start, pose_end) <= maxConnectionLength;
}

}

