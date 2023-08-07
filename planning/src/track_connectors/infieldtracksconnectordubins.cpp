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
 

#include "arolib/planning/track_connectors/infieldtracksconnectordubins.hpp"

namespace arolib{

namespace ag = arolib::geometry;

InfieldTracksConnectorDubins::ExtensionParameters InfieldTracksConnectorDubins::ExtensionParameters::NoExtention()
{
    ExtensionParameters ret;
    ret.deltaAng0 = ret.deltaDist0 =  ret.deltaAngn = ret.deltaDistn = -1;
    return ret;
}


InfieldTracksConnectorDubins::InfieldTracksConnectorDubins(std::shared_ptr<Logger> parentLogger)
    : IInfieldTracksConnector(__FUNCTION__, parentLogger)
{

}

std::vector<Point> InfieldTracksConnectorDubins::getConnection(const Machine &machine,
                                                               const Pose2D &_pose_start,
                                                               const Pose2D &_pose_end,
                                                               double turningRad,
                                                               const std::pair<double, double> &extraDist,
                                                               const Polygon &_limitBoundary,
                                                               const Polygon &infieldBoundary,
                                                               const Headlands &/*headlands*/,
                                                               double /*speed_start*/,
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
        ret = getDubinsPathConnection(pose_start, pose_end, turningRad, {&limitBoundary}, infieldBoundary, maxConnectionLength, false, nullptr);
        if(!ret.empty()){
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Connected directly using dubins path..");
            return ret;
        }
        if(!m_onlyShortest){
            ret = getDubinsPathConnection(pose_start, pose_end, turningRad, {&limitBoundary}, infieldBoundary, maxConnectionLength, true, nullptr);
            if(!ret.empty()){
                logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Connected directly using dubins path (2)..");
                return ret;
            }
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

    return ret;

}

void InfieldTracksConnectorDubins::setOnlyShortest(bool onlyShortest)
{
    m_onlyShortest = onlyShortest;
}


std::vector<Point> InfieldTracksConnectorDubins::getDubinsPathConnection(const Pose2D &pose_start,
                                                                         const Pose2D &pose_end,
                                                                         double turningRad,
                                                                         const std::vector<const Polygon *> &limitBoundaries,
                                                                         const Polygon &infieldBoundary,
                                                                         double maxConnectionLength,
                                                                         bool testAll,
                                                                         const ExtensionParameters *extParams,
                                                                         double *length)
{


    ExtensionParameters defExtParams;
    const ExtensionParameters * usedExtParams = extParams;
    if(!usedExtParams){
        usedExtParams = &defExtParams;
        defExtParams = getDefaultExtensionParameters(turningRad);
    }

    std::vector<Point> ret;

    for(int extDistStartType = 0 ; extDistStartType < usedExtParams->distPasses+1 && ret.empty() ; ++extDistStartType){
        if(extDistStartType > 0 && usedExtParams->deltaDist0 < 1e-9)
            break;

        double deltaDist0 = usedExtParams->deltaDist0 * extDistStartType;

        for(int extAngStartType = 0 ; extAngStartType < 3 && ret.empty() ; ++extAngStartType){
            if(extAngStartType > 0 && usedExtParams->deltaAng0 < 1e-9)
                break;

            double deltaAng0 = 0;
            if(extAngStartType != 0){
                int mult = 0.5 * (extAngStartType+1);
                if (extAngStartType%2 == 1){
                    deltaAng0 = usedExtParams->deltaAng0 * mult;
                }
                else{
                    deltaAng0 = usedExtParams->deltaAng0 * -mult;
                }
            }

            Pose2D pose0 = pose_start;
            pose0.angle += deltaAng0;
            pose0.point() = ag::getPointAtDist(pose0, deltaDist0);

            if(pose0.point() != pose_start.point()){
                PointVec seg = {pose_start.point(), pose0.point()};
                if( !isPathValid(seg, limitBoundaries, infieldBoundary, maxConnectionLength) )
                    continue;
            }

            double dist0 = ag::calc_dist(pose_start, pose0);
            double maxConnLength0 = ( maxConnectionLength > 1e-9 ? std::max(1e-6, maxConnectionLength - dist0) : maxConnectionLength );

            for(int extDistEndType = 0 ; extDistEndType < usedExtParams->distPasses+1 && ret.empty() ; ++extDistEndType){
                if(extDistEndType > 0 && usedExtParams->deltaDistn < 1e-9)
                    break;

                double deltaDistn = usedExtParams->deltaDistn * extDistEndType;

                for(int extAngEndType = 0 ; extAngEndType < 2*(usedExtParams->distPasses)+1 && ret.empty() ; ++extAngEndType){
                    if(extAngEndType > 0 && usedExtParams->deltaAngn < 1e-9)
                        break;

                    double deltaAngn = 0;
                    if(extAngEndType != 0){
                        int mult = 0.5 * (extAngEndType+1);
                        if (extAngEndType%2 == 1){
                            deltaAngn = usedExtParams->deltaAngn * mult;
                        }
                        else{
                            deltaAngn = usedExtParams->deltaAngn * -mult;
                        }
                    }

                    Pose2D posen = pose_end;
                    posen.angle += deltaAngn;
                    posen.point() = ag::getPointAtDist(posen, -deltaDistn);

                    if(posen.point() != pose_end.point()){
                        PointVec seg = {pose_end.point(), posen.point()};
                        if( !isPathValid(seg, limitBoundaries, infieldBoundary, maxConnLength0) )
                            continue;
                    }

                    double distn = ag::calc_dist(pose_end, posen);
                    double maxConnLength = ( maxConnLength0 > 1e-9 ? std::max(1e-6, maxConnLength0 - distn) : maxConnLength0 );


                    std::multimap<double, std::vector<Point>> paths;

                    int iPathType = testAll ? 0 : (int) ag::DubinsParams::PathType::SHORTEST;
                    int iPathTypen = testAll ? -1 + (int) ag::DubinsParams::PathType::SHORTEST : (int) ag::DubinsParams::PathType::SHORTEST;
                    for(; iPathType <= iPathTypen; ++iPathType){
                        auto pathType = ag::DubinsParams::intToPathType(iPathType);
                        double lengthTmp;
                        auto pathTmp = ag::calcDubinsPath(pose0, posen, turningRad, 30, true, pathType, &lengthTmp);
                        if(pathTmp.empty())
                            continue;
                        if( maxConnLength > 1e-9 && lengthTmp > maxConnLength)
                            continue;
                        if(pose0.point() != pose_start.point())
                            push_front(pathTmp, pose_start.point());
                        if(posen.point() != pose_end.point())
                            pathTmp.push_back(pose_end.point());
                        paths.insert( std::make_pair(lengthTmp + dist0 + distn, pathTmp) );
                    }

                    for(auto& it_path : paths){
                        if( isPathValid(it_path.second, limitBoundaries, infieldBoundary, maxConnectionLength) ){
                            ret = it_path.second;
                            if(length)
                                *length = it_path.first;
                            return ret;
                        }
                    }
                }
            }
        }
    }

    return ret;
}

InfieldTracksConnectorDubins::ExtensionParameters InfieldTracksConnectorDubins::getDefaultExtensionParameters(double turningRad){
    ExtensionParameters defExtParams;
    defExtParams.deltaAng0 = defExtParams.deltaAngn = deg2rad(15);
    if(turningRad > 1e-9)
        defExtParams.deltaDist0 = defExtParams.deltaDistn = std::max(turningRad, 0.01);
    else
        defExtParams.deltaDist0 = defExtParams.deltaDistn = -1;
    defExtParams.distPasses = 2;
    defExtParams.angPasses = 1;
    return defExtParams;
}



}

