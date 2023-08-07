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
 

#include "arolib/planning/track_connectors/infieldtracksconnectordubinsorgraphbased.hpp"

namespace arolib{

namespace ag = arolib::geometry;


InfieldTracksConnectorDubinsOrGraphBased::InfieldTracksConnectorDubinsOrGraphBased(std::shared_ptr<Logger> parentLogger)
    : InfieldTracksConnectorGraphBased(parentLogger)
{

}

std::vector<Point> InfieldTracksConnectorDubinsOrGraphBased::getConnection(const Machine &machine,
                                                                           const Pose2D &_pose_start,
                                                                           const Pose2D &_pose_end,
                                                                           double _turningRad,
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

    double turningRad = _turningRad;

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
        ret = InfieldTracksConnectorDubins::getDubinsPathConnection(pose_start, pose_end, turningRad, {&limitBoundary}, infieldBoundary, maxConnectionLength, true, nullptr);
        if(!ret.empty()){
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Connected directly using dubins path (2)..");
            return ret;
        }
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

    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Trying to connect using graph-based connector...");
    ret = InfieldTracksConnectorGraphBased::getConnection(machine,
                                                          _pose_start,
                                                          _pose_end,
                                                          _turningRad,
                                                          extraDist,
                                                          _limitBoundary,
                                                          infieldBoundary,
                                                          headlands,
                                                          speed_start,
                                                          maxConnectionLength);
    return ret;

}



}

