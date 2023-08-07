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
 
#include "arolib/components/infieldbaseroutesplanner.h"

namespace arolib {


const double InfieldBaseRoutesPlanner::m_thresholdIsWorked = 0.5;

bool InfieldBaseRoutesPlanner::PlannerParameters::parseFromStringMap(InfieldBaseRoutesPlanner::PlannerParameters &params,
                                                                     const std::map<std::string, std::string> &map, bool strict)
{
    InfieldBaseRoutesPlanner::PlannerParameters tmp;

    if( !FieldGeneralParameters::parseFromStringMap(tmp, map, strict) )
        return false;
    if( !GridComputationSettings::parseFromStringMap(tmp, map, strict) )
        return false;
    if( !ITrackSequencer::TrackSequencerSettings::parseFromStringMap(tmp, map, strict) )
        return false;

    std::map<std::string, double*> dMap = { {"sampleResolutionHeadland" , &tmp.sampleResolutionHeadland} };
    std::map<std::string, bool*> bMap = { {"inverseTrackOrder" , &tmp.inverseTrackOrder},
                                          {"inversePointsOrder" , &tmp.inversePointsOrder},
                                          {"removeInitialWorkedSegments" , &tmp.removeInitialWorkedSegments} };



    if( !setValuesFromStringMap( map, dMap, strict)
            || !setValuesFromStringMap( map, bMap, strict) )
        return false;

    params = tmp;
    return true;
}

std::map<std::string, std::string> InfieldBaseRoutesPlanner::PlannerParameters::parseToStringMap(const InfieldBaseRoutesPlanner::PlannerParameters &params)
{
    std::map<std::string, std::string> ret, subMap;
    subMap = FieldGeneralParameters::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );
    subMap = GridComputationSettings::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );
    subMap = ITrackSequencer::TrackSequencerSettings::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );

    ret["sampleResolutionHeadland"] = double2string( params.sampleResolutionHeadland );
    ret["inverseTrackOrder"] = std::to_string( params.inverseTrackOrder );
    ret["inversePointsOrder"] = std::to_string( params.inversePointsOrder );
    ret["removeInitialWorkedSegments"] = std::to_string( params.removeInitialWorkedSegments );

    return ret;

}

InfieldBaseRoutesPlanner::InfieldBaseRoutesPlanner(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{
    m_tracksSequencer = std::make_shared<SimpleTrackSequencer>();
    m_tracksSequencer->logger().setParent(loggerPtr());

    m_tracksConnector = std::make_shared<InfieldTracksConnectorDef>();
    m_tracksConnector->logger().setParent(loggerPtr());
}

AroResp InfieldBaseRoutesPlanner::plan(const Subfield &subfield,
                                       const std::vector<Machine> &workinggroup,
                                       const PlannerParameters &plannerParameters,
                                       std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                       std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                       std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorTransit,
                                       std::vector<Route> &routes,
                                       const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                       const Pose2D *initRefPose,
                                       std::shared_ptr<const ArolibGrid_t> massFactorMap,
                                       std::shared_ptr<const ArolibGrid_t> remainingAreaMap)
{

    if(!edgeMassCalculator)
        return AroResp::LoggingResp(1, "A mass calculator must be given", m_logger, LogLevel::ERROR, __FUNCTION__);

    LoggingComponent::LoggersHandler lh(true);//will be reset on destruction

    if(remainingAreaMap)
        LoggingComponent::setTemporalLoggersParent(lh, *this, *remainingAreaMap);

    auto initRefPoint = initRefPose;
    bool inverseFlagsSet = false;
    bool inverseTrackOrder = plannerParameters.inverseTrackOrder;
    bool inversePointsOrderStart = plannerParameters.inversePointsOrder;

    std::shared_ptr<InternalMassCalculator> edgeMassCalculatorExtended = std::make_shared<InternalMassCalculator>( edgeMassCalculator,
                                                                                                                   massFactorMap,
                                                                                                                   m_cim,
                                                                                                                   plannerParameters.bePreciseWithMatterMap ? gridmap::SharedGridsManager::PRECISE : gridmap::SharedGridsManager::PRECISE_ONLY_IF_AVAILABLE );

    //Get the indexes of the tracks that are completelly worked
    std::set<size_t> excludeTrackIndexes = getExcludeTrackIndexes(subfield, remainingAreaMap, plannerParameters.bePreciseWithRemainingAreaMap);

    if( ( !initRefPoint || !initRefPoint->isValid() )
          && remainingAreaMap && remainingAreaMap->isAllocated()){//Get the initial reference point and inverse flags based on the worked area map (and, if necessary, the current locations of the machines)
        logger().printOut(LogLevel::INFO, __FUNCTION__, "Obtaining inverse flags based on the remaining area map...");
        inverseFlagsSet = getInverseFlagsBasedOnRemainingArea(subfield,
                                                              excludeTrackIndexes,
                                                              workinggroup,
                                                              plannerParameters,
                                                              *remainingAreaMap,
                                                              machineCurrentStates,
                                                              inverseTrackOrder,
                                                              inversePointsOrderStart);
    }

    if(( !initRefPoint || !initRefPoint->isValid() )
            && !inverseFlagsSet){//Get the initial reference point and inverse flags based on the the current locations of the machines
        logger().printOut(LogLevel::INFO, __FUNCTION__, "Obtaining inverse flags based on the machines current location...");
        inverseFlagsSet = getInverseFlagsBasedOnMachineLocation(subfield,
                                                                excludeTrackIndexes,
                                                                workinggroup,
                                                                plannerParameters,
                                                                machineCurrentStates,
                                                                inverseTrackOrder,
                                                                inversePointsOrderStart);
    }

    if(initRefPoint && initRefPoint->isValid() && !inverseFlagsSet){
        logger().printOut(LogLevel::INFO, __FUNCTION__, "Obtaining inverse flags based on the initial reference point...");
        getInverseFlagsBasedOnReferencePoint(subfield,
                                             excludeTrackIndexes,
                                             *initRefPoint,
                                             workinggroup,
                                             plannerParameters,
                                             inverseTrackOrder,
                                             inversePointsOrderStart);
    }

    auto aroResp = generateBaseRoutes(subfield,
                                      excludeTrackIndexes,
                                      workinggroup,
                                      plannerParameters,
                                      inverseTrackOrder,
                                      inversePointsOrderStart,
                                      initRefPoint,
                                      edgeMassCalculatorExtended,
                                      edgeSpeedCalculator,
                                      edgeSpeedCalculatorTransit,
                                      routes);

    if(!aroResp.isError())
        aroResp = adjustBaseRoutes(routes,
                                   subfield,
                                   workinggroup,
                                   edgeMassCalculatorExtended,
                                   remainingAreaMap,
                                   plannerParameters);

    return aroResp;

}

void InfieldBaseRoutesPlanner::setInfieldTrackSequencer(std::shared_ptr<ITrackSequencer> track_sequencer) {
    if(track_sequencer)
        m_tracksSequencer = track_sequencer;
}

void InfieldBaseRoutesPlanner::setInfieldTrackConnector(std::shared_ptr<IInfieldTracksConnector> connector)
{
    if(connector)
        m_tracksConnector = connector;
}

void InfieldBaseRoutesPlanner::setGridCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim)
{
    m_cim = cim;
}

std::set<size_t> InfieldBaseRoutesPlanner::getExcludeTrackIndexes(const Subfield &subfield, std::shared_ptr<const ArolibGrid_t> remainingAreaMap, bool bePrecise)
{
    std::set<size_t> ret;
    if(!remainingAreaMap || !remainingAreaMap->isAllocated())
        return ret;

    for(size_t i = 0 ; i < subfield.tracks.size() ; ++i){
        const auto& track = subfield.tracks.at(i);
        if(track.points.size() < 2){
            ret.insert(i);
            continue;
        }

        /* //If the map has values between 0 and 1, computing the values for the whole polygon might lead to wrong assumption
        Polygon trackPoly = track.boundary;
        if(trackPoly.points.empty()){
            if(track.width < 1e-5
                    || !geometry::offsetLinestring(track.points, trackPoly, 0.5*track.width, 0.5*track.width, true, 0)){
                ret.insert(i);
                continue;
            }
        }
        std::vector<Polygon> intersectionPolys;
        if(subfield.boundary_inner.points.empty())
            intersectionPolys.emplace_back(trackPoly);
        else
            intersectionPolys = geometry::get_intersection(subfield.boundary_inner, trackPoly);

        int excludeCount = 0;
        for(const auto& poly : intersectionPolys){
            bool errorTmp;
            double value = remainingArea_map.getPolygonComputedValue(poly,
                                                                     ArolibGrid_t::AVERAGE_TOTAL,
                                                                     bePrecise
                                                                     &errorTmp);

            if(errorTmp || value < 0.01)
                ++excludeCount;
        }

        if(excludeCount == intersectionPolys.size())
            ret.insert(i);
        */

        double resolution = geometry::calc_dist(track.points.front(), track.points.at(1));
        for(size_t j = 1 ; j+1 < track.points.size() ; ++j)
            resolution = std::max(resolution, geometry::calc_dist(track.points.at(j), track.points.at(j+1)) );

        bool worked = false;
        for(size_t j = 0 ; j+1 < track.points.size() ; ++j){
            if( (j == 0 || j+2 == track.points.size())
                    && arolib::geometry::calc_dist(track.points.at(j), track.points.at(j+1)) < 0.5 * resolution)
                continue;
            worked = isWorked(track.points.at(j), track.points.at(j+1), track.width, *remainingAreaMap, bePrecise);
            if(!worked)
                break;
        }
        if(worked)
            ret.insert(i);
    }

    return ret;
}

bool InfieldBaseRoutesPlanner::getInverseFlagsBasedOnRemainingArea(const Subfield &subfield,
                                                                   const std::set<size_t>& excludeTrackIndexes,
                                                                   const std::vector<Machine> workinggroup,
                                                                   const InfieldBaseRoutesPlanner::PlannerParameters &plannerParameters,
                                                                   const ArolibGrid_t &remainingArea_map,
                                                                   const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                                                   bool &inverseTrackOrder,
                                                                   bool &inversePointsOrderStart)
{
    if(!remainingArea_map.isAllocated())
        return false;

    double resolution = -1;
    for(auto &track : subfield.tracks)
        resolution = std::max( resolution, arolib::geometry::getMaxSampleDistance(track.points) );

    Polygon boundary;
    if(!geometry::offsetPolygon(subfield.boundary_inner, boundary, 0.1, true, 0))
        boundary = subfield.boundary_inner;

    //get the first track that has a non-worked segment (in forward and inverse track points' order)
    int indFirstTrackNotWorked_fwd = -1, indFirstTrackNotWorked_inv = -1;
    for(int side = 0 ; side < 2 ; side ++){
        int &indFirstTrackNotWorked = ( side == 0 ? indFirstTrackNotWorked_fwd : indFirstTrackNotWorked_inv );
        for(size_t i = 0 ; i < subfield.tracks.size() ; ++i){
            size_t ind_track = ( side == 0 ? i : subfield.tracks.size()-1-i );
            if( excludeTrackIndexes.find(ind_track) != excludeTrackIndexes.end() )
                continue;
            auto points = subfield.tracks.at(ind_track).points;
            if(points.size() < 2)
                continue;
            for(size_t j = 0 ; j+1 < points.size() ; ++j){
                int worked = isWorked(boundary, points.at(j), points.at(j+1), subfield.tracks.at(ind_track).width, remainingArea_map, plannerParameters.bePreciseWithRemainingAreaMap);
                if(worked == 0){
                    indFirstTrackNotWorked = i;
                    break;
                }
            }
            if(indFirstTrackNotWorked >= 0)
                break;
        }
    }
    if(indFirstTrackNotWorked_fwd < 0)// all worked
        return false;

    if(indFirstTrackNotWorked_fwd == 0 && indFirstTrackNotWorked_inv == 0)// all unworked
        return false;


    bool inFwdTrackOrder = (indFirstTrackNotWorked_fwd >= indFirstTrackNotWorked_inv);//was the field worked following the current tracks order?
    bool inFwdPointsOrder;

    int indFirstTrackNotWorked = inFwdTrackOrder ? indFirstTrackNotWorked_fwd : subfield.tracks.size()-1-indFirstTrackNotWorked_inv;

    const auto firstTrack = subfield.tracks.at(indFirstTrackNotWorked);
    std::vector<Point> points = firstTrack.points;//points of the FIRST unworked track (in the tracks order given by inFwdTrackOrder)


    int indFirstUnworkedSegment_fwd = -1, indFirstUnworkedSegment_inv = -1;
    bool somethingWorked = false;

    for(int side = 0 ; side < 2 ; side ++){
        int &indFirstUnworkedSegment = ( side == 0 ? indFirstUnworkedSegment_fwd : indFirstUnworkedSegment_inv );
        for(size_t i = 0 ; i+1 < points.size() ; ++i){
            size_t ind0 = ( side == 0 ? i : points.size()-1-i );
            size_t ind1 = ( side == 0 ? ind0+1 :ind0-1);

            Polygon segmentPoly = geometry::createRectangleFromLine( points.at(ind0), points.at(ind1), firstTrack.width );
            segmentPoly.points.pop_back();
            if( !geometry::in_polygon(segmentPoly.points, boundary, true) )//only check the segments that are completelly inside the inner boundary
                continue;

            if(isWorked(points.at(ind0), points.at(ind1), firstTrack.width, remainingArea_map, plannerParameters.bePreciseWithRemainingAreaMap))
                somethingWorked = true;
            else{
                indFirstUnworkedSegment = i;
                break;
            }
        }
    }

    if(indFirstUnworkedSegment_fwd < 0 || indFirstUnworkedSegment_inv < 0)
        return false;


    if(somethingWorked){
        double d_fwd = geometry::getGeometryLength(points, 0, indFirstUnworkedSegment_fwd);
        double d_inv = geometry::getGeometryLength(points, points.size()-1-indFirstUnworkedSegment_inv, points.size()-1);
        inFwdPointsOrder = d_inv < d_fwd;
    }
    else if(machineCurrentStates){//the complete track is not worked. make the decision based on machines' current position (i.e. the point closest to a machine)
        inFwdPointsOrder = true;
        double minDist = std::numeric_limits<double>::max();
        for(auto& m : workinggroup){
            if(m.isOfWorkingType(true)){
                auto it_m = machineCurrentStates->find(m.id);
                if(it_m != machineCurrentStates->end()){
                    auto machineCurrentPosition = it_m->second.position;
                    double distFwd = arolib::geometry::calc_dist(machineCurrentPosition, points.front());
                    double distInv = arolib::geometry::calc_dist(machineCurrentPosition, points.back());
                    if(minDist > distFwd){
                        inFwdPointsOrder = true;
                        minDist = distFwd;
                    }
                    if(minDist > distInv){
                        inFwdPointsOrder = false;
                        minDist = distInv;
                    }
                }
            }
        }
    }

    inverseTrackOrder = !inFwdTrackOrder;
    inversePointsOrderStart = !inFwdPointsOrder;

    return true;
}

bool InfieldBaseRoutesPlanner::getInverseFlagsBasedOnMachineLocation(const Subfield &subfield,
                                                                     const std::set<size_t>& excludeTrackIndexes,
                                                                     const std::vector<Machine> workinggroup,
                                                                     const InfieldBaseRoutesPlanner::PlannerParameters &plannerParameters,
                                                                     const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                                                     bool &inverseTrackOrder,
                                                                     bool &inversePointsOrderStart)
{
    if(subfield.tracks.empty())
        return false;

    int indFirstTrack = -1, indLastTrack = -1;
    for(size_t i = 0 ; i < subfield.tracks.size() ; ++i ){
        if(subfield.tracks.at(i).points.size() > 1
                && excludeTrackIndexes.find(i) == excludeTrackIndexes.end()){
            indFirstTrack = i;
            break;
        }
    }
    for(size_t i = 0 ; i < subfield.tracks.size() ; ++i ){
        const size_t j = subfield.tracks.size()-1-i;
        if(subfield.tracks.at(j).points.size() > 1
                && excludeTrackIndexes.find(j) == excludeTrackIndexes.end()){
            indLastTrack = j;
            break;
        }
    }
    if(indFirstTrack < 0 || indLastTrack < 0)
        return false;

    const auto& firstTrack = subfield.tracks.at(indFirstTrack);
    const auto& lastTrack = subfield.tracks.at(indLastTrack);

    bool locationOK = false;
    double minDist = std::numeric_limits<double>::max();

    Polygon boundary;
    if( !geometry::offsetPolygon(subfield.boundary_outer, boundary, 0.1, true, 0) )
        boundary.points.clear();

    if(machineCurrentStates){
        for(auto& m : workinggroup){

            if(m.isOfWorkingType(true)){
                auto it_m = machineCurrentStates->find(m.id);
                if(it_m == machineCurrentStates->end())
                    continue;
                auto machineCurrentPosition = it_m->second.position;

                if(!boundary.points.empty() && !geometry::in_polygon(machineCurrentPosition, boundary))
                    continue;

                locationOK = true;

                double distFwd = arolib::geometry::calc_dist(machineCurrentPosition, firstTrack.points.front());
                double distInv = arolib::geometry::calc_dist(machineCurrentPosition, firstTrack.points.back());
                if(minDist > distFwd){
                    inverseTrackOrder = false;
                    inversePointsOrderStart = false;
                    minDist = distFwd;
                }
                if(minDist > distInv){
                    inverseTrackOrder = false;
                    inversePointsOrderStart = true;
                    minDist = distInv;
                }

                distFwd = arolib::geometry::calc_dist(machineCurrentPosition, lastTrack.points.front());
                distInv = arolib::geometry::calc_dist(machineCurrentPosition, lastTrack.points.back());
                if(minDist > distFwd){
                    inverseTrackOrder = true;
                    inversePointsOrderStart = false;
                    minDist = distFwd;
                }
                if(minDist > distInv){
                    inverseTrackOrder = true;
                    inversePointsOrderStart = true;
                    minDist = distInv;
                }
            }

        }
    }

    return locationOK;
}

bool InfieldBaseRoutesPlanner::getInverseFlagsBasedOnReferencePoint(const Subfield &subfield,
                                                                    const std::set<size_t> &excludeTrackIndexes,
                                                                    const Pose2D &refPoint,
                                                                    const std::vector<Machine> workinggroup,
                                                                    const InfieldBaseRoutesPlanner::PlannerParameters &plannerParameters,
                                                                    bool &inverseTrackOrder,
                                                                    bool &inversePointsOrderStart)
{
    if(subfield.tracks.empty())
        return false;

    int indFirstTrack = -1, indLastTrack = -1;
    for(size_t i = 0 ; i < subfield.tracks.size() ; ++i ){
        if(subfield.tracks.at(i).points.size() > 1
                && excludeTrackIndexes.find(i) == excludeTrackIndexes.end()){
            indFirstTrack = i;
            break;
        }
    }
    for(size_t i = 0 ; i < subfield.tracks.size() ; ++i ){
        const size_t j = subfield.tracks.size()-1-i;
        if(subfield.tracks.at(j).points.size() > 1
                && excludeTrackIndexes.find(j) == excludeTrackIndexes.end()){
            indLastTrack = j;
            break;
        }
    }
    if(indFirstTrack < 0 || indLastTrack < 0)
        return false;

    const auto& firstTrack = subfield.tracks.at(indFirstTrack).points;
    const auto& lastTrack = subfield.tracks.at(indLastTrack).points;

    double turningRad = 0;
    Machine refMachine;
    for(auto& m : workinggroup){
        if(m.isOfWorkingType(true) && turningRad < m.getTurningRadius()){
            turningRad = m.getTurningRadius();
            refMachine = m;
        }
    }

    double turningRadConn = turningRad;
    //double turningRadConn = subfield.headlands.hasCompleteHeadland() ? turningRad : 0; // using no turning rad for the connections when there are partial headlandsbecause it can be time expensive

    bool foundConnection = false;
    double minDist = std::numeric_limits<double>::max();

//    //@note: at the moment, if the field has partial headlands, check for connections without an extended boundary
//    Polygon boundary = subfield.boundary_outer;
//    if(!subfield.headlands.hasCompleteHeadland())
//        geometry::offsetPolygon(subfield.boundary_outer, boundary, 1.5*turningRad, true, 0);
    Polygon boundary = m_tracksConnector->getExtendedLimitBoundary(subfield, turningRadConn);

    auto connection = m_tracksConnector->getConnection(refMachine,
                                                       refPoint,
                                                       Pose2D( firstTrack.front(), geometry::get_angle( firstTrack.front(), firstTrack.at(1) ) ),
                                                       turningRadConn,
                                                       std::make_pair(-1, -1),
                                                       boundary,
                                                       subfield.boundary_inner,
                                                       subfield.headlands);
    if(!connection.empty()){
        double connLength = geometry::getGeometryLength(connection);
        if(connLength < minDist){
            minDist = connLength;
            inverseTrackOrder = false;
            inversePointsOrderStart = false;
            foundConnection = true;
        }
    }

    connection = m_tracksConnector->getConnection(refMachine,
                                                  refPoint,
                                                  Pose2D( firstTrack.back(), geometry::get_angle( firstTrack.back(), r_at(firstTrack, 1) ) ),
                                                  turningRadConn,
                                                  std::make_pair(-1, -1),
                                                  boundary,
                                                  subfield.boundary_inner,
                                                  subfield.headlands);
    if(!connection.empty()){
        double connLength = geometry::getGeometryLength(connection);
        if(connLength < minDist){
            minDist = connLength;
            inverseTrackOrder = false;
            inversePointsOrderStart = true;
            foundConnection = true;
        }
    }


    connection = m_tracksConnector->getConnection(refMachine,
                                                  refPoint,
                                                  Pose2D( lastTrack.front(), geometry::get_angle( lastTrack.front(), lastTrack.at(1) ) ),
                                                  turningRadConn,
                                                  std::make_pair(-1, -1),
                                                  boundary,
                                                  subfield.boundary_inner,
                                                  subfield.headlands);
    if(!connection.empty()){
        double connLength = geometry::getGeometryLength(connection);
        if(connLength < minDist){
            minDist = connLength;
            inverseTrackOrder = true;
            inversePointsOrderStart = false;
            foundConnection = true;
        }
    }

    connection = m_tracksConnector->getConnection(refMachine,
                                                  refPoint,
                                                  Pose2D( lastTrack.back(), geometry::get_angle( lastTrack.back(), r_at(lastTrack, 1) ) ),
                                                  turningRadConn,
                                                  std::make_pair(-1, -1),
                                                  boundary,
                                                  subfield.boundary_inner,
                                                  subfield.headlands);
    if(!connection.empty()){
        double connLength = geometry::getGeometryLength(connection);
        if(connLength < minDist){
            minDist = connLength;
            inverseTrackOrder = true;
            inversePointsOrderStart = true;
            foundConnection = true;
        }
    }


    if(!foundConnection){

        minDist = arolib::geometry::calc_dist(refPoint, firstTrack.front());
        inverseTrackOrder = false;
        inversePointsOrderStart = false;

        double dist = arolib::geometry::calc_dist(refPoint, firstTrack.back());
        if(minDist > dist){
            minDist = dist;
            inverseTrackOrder = false;
            inversePointsOrderStart = true;
        }

        dist = arolib::geometry::calc_dist(refPoint, lastTrack.front());
        if(minDist > dist){
            minDist = dist;
            inverseTrackOrder = true;
            inversePointsOrderStart = false;
        }

        dist = arolib::geometry::calc_dist(refPoint, lastTrack.back());
        if(minDist > dist){
            minDist = dist;
            inverseTrackOrder = true;
            inversePointsOrderStart = true;
        }
    }

    return true;
}


bool InfieldBaseRoutesPlanner::isWorked(const Point &p0, const Point &p1, double width, const ArolibGrid_t &remainingAreaMap, bool bePrecise)
{
    if(!remainingAreaMap.isAllocated())
        return false;

    bePrecise &= width > 1e-5;

    bool errorTmp = true;
    double value = 1;

    if(bePrecise){
        value = remainingAreaMap.getLineComputedValue(p0,
                                                       p1,
                                                       width,
                                                       true,
                                                       ArolibGrid_t::AVERAGE_TOTAL,
                                                       &errorTmp);
        return (errorTmp || value < m_thresholdIsWorked);
    }

    arolib::Point p0_1;
    p0_1.x = 0.5*( p0.x + p1.x );
    p0_1.y = 0.5*( p0.y + p1.y );
    if(remainingAreaMap.hasValue(p0_1))
        value = remainingAreaMap.getValue(p0_1, &errorTmp);
    return (!errorTmp && value < m_thresholdIsWorked);

}

int InfieldBaseRoutesPlanner::isWorked(const Polygon &boundary, const Point &p0, const Point &p1, double width, const ArolibGrid_t &remainingAreaMap, bool bePrecise)
{
    if(width <= 0)
        return -1;

    if(!remainingAreaMap.isAllocated())
        return 0;

    if(boundary.points.empty())
        return isWorked(p0, p1, width, remainingAreaMap, bePrecise);


    Polygon segmentPoly = geometry::createRectangleFromLine( p0, p1, width );

    size_t countPointsInside = 0;
    for(size_t i = 0 ; i+1 < segmentPoly.points.size(); ++i)
        countPointsInside += geometry::in_polygon(segmentPoly.points.at(i), boundary);

    if(countPointsInside > 3 || (countPointsInside == 3 && !bePrecise))
        return isWorked(p0, p1, width, remainingAreaMap, bePrecise);

    if(!bePrecise)
        return -1;

    std::vector<Polygon> intersectionPolys = geometry::get_intersection(boundary, segmentPoly);

    double areaComplete = geometry::calc_area(p0, p1, width);
    double area = 0;
    std::vector<double> areas(intersectionPolys.size());
    for(size_t i = 0 ; i < intersectionPolys.size() ; ++i){
        areas.at(i) = geometry::calc_area( intersectionPolys.at(i) );
        area += areas.at(i);
    }

    if(areaComplete <= 0 || area / areaComplete < 0.5)
        return -1;

    double value = 0;
    for(size_t i = 0 ; i < intersectionPolys.size() ; ++i){
        if(areas.at(i) < 1e-3)
            continue;
        bool errorTmp;
        double valueTmp = remainingAreaMap.getPolygonComputedValue(intersectionPolys.at(i),
                                                                    ArolibGrid_t::AVERAGE_TOTAL,
                                                                    bePrecise
                                                                    &errorTmp);
        if(errorTmp)
            continue;
        value += valueTmp * areas.at(i) / area;
    }

    return (value < m_thresholdIsWorked);

}

bool InfieldBaseRoutesPlanner::hasBiomass(const Point &p0,
                                        const Point &p1,
                                        double workingWidth,
                                          std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator)
{
    return edgeMassCalculator->calcMass(p0, p1, workingWidth) > 1e-3;

}


AroResp InfieldBaseRoutesPlanner::generateBaseRoutes(const Subfield &subfield,
                                                     const std::set<size_t> &excludeTrackIndexes,
                                                     const std::vector<Machine> &workinggroup,
                                                     const PlannerParameters &plannerParameters,
                                                     bool inverseTrackOrder,
                                                     bool inversePointsOrderStart,
                                                     const Pose2D *initRefPose,
                                                     std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                                     std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                                     std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorTransit,
                                                     std::vector<Route> &routes)
{
    routes.clear();
    SimpleBaseRoutesPlanner sbrp(logger().logLevel());
    sbrp.logger().setParent(loggerPtr());

    auto aroResp = sbrp.setSubfield(subfield);
    if(aroResp.isError())
        return aroResp;

    sbrp.setInfieldTrackSequencer(m_tracksSequencer);
    sbrp.setInfieldTrackSequencerSettings(plannerParameters);
    sbrp.setInverseTrackOrder(inverseTrackOrder);
    sbrp.setFirstTrackInversePointOrder(inversePointsOrderStart);
    sbrp.setExcludeTrackIndexes(excludeTrackIndexes);
    if(initRefPose)
        sbrp.setInitRefPose(*initRefPose);

    for (auto &m : workinggroup){
        if(m.isOfWorkingType(true)){
            aroResp = sbrp.addMachine(m);
            if(aroResp.isError())
                return aroResp;
        }
    }

    for(auto &m : workinggroup){
        if (m.isOfWorkingType(true)) {
            logger().printOut(LogLevel::INFO, __FUNCTION__, "Added machine " + std::to_string(m.id) + " to the machine plan");
            routes.push_back(Route());
            aroResp = sbrp.getRoute(m.id, routes.back(), edgeMassCalculator, edgeSpeedCalculator, edgeSpeedCalculatorTransit);
            routes.back().route_id = routes.size()-1;
            if (aroResp.isError())
                return AroResp(1, "Error obtaining machine route: " + aroResp.msg);
        }
    }
    logger().printOut(LogLevel::INFO, __FUNCTION__, std::to_string(routes.size()) + " routes were generated");

    return AroResp(0,"OK");

}

AroResp InfieldBaseRoutesPlanner::adjustBaseRoutes(std::vector<Route> &routes,
                                                   const Subfield &subfield,
                                                   const std::vector<Machine> &workinggroup,
                                                   std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                                   std::shared_ptr<const ArolibGrid_t> remainingAreaMap,
                                                   const PlannerParameters &plannerParameters)
{
    std::map<MachineId_t, double> widths;//map containing the machine working widths (for easy access)
    for(auto &m : workinggroup)
        widths[m.id] = m.working_width;
    std::vector< std::pair<size_t, size_t>> firstGoodIndexes(routes.size(), {0,0});//vector containing the route point index of the first segment to be worked (i.e. has biomass and is not worked)
    size_t indFirstGood = -1;
    double deltaTime = -1;

    //search the route point index of the first segment to be worked (i.e. has biomass and is not worked) for each route
    for(size_t i = 0 ; i < routes.size() ; ++i){
        auto &route = routes.at(i);
        auto it_width = widths.find(route.machine_id);
        if(it_width == widths.end())
            continue;

        //get the first index where there is biomass to be worked in the route (excluding inter-track (headland) connections)
        for(size_t j = 0 ; j+1 < route.route_points.size() ; ++j){
            auto& p0 = route.route_points.at(j);
            auto& p1 = route.route_points.at(j+1);
            if(p0.type != RoutePoint::TRACK_START && !p0.isOfTypeWorking_InTrack(true))
                continue;
            if(!hasBiomass(p0, p1, it_width->second, edgeMassCalculator))
                continue;
            firstGoodIndexes.at(i).first = j;
            break;
        }

        if(remainingAreaMap && remainingAreaMap->isAllocated()){
            bool somethingWorked = false;

            Polygon boundary;
            if(!geometry::offsetPolygon(subfield.boundary_inner, boundary, 0.1, true, 0))
                boundary = subfield.boundary_inner;

            //get the first index in the route that is not worked (excluding inter-track (headland) connections)
            for(size_t j = firstGoodIndexes.at(i).first ; j+1 < route.route_points.size() ; ++j){
                auto& p0 = route.route_points.at(j);
                auto& p1 = route.route_points.at(j+1);
                if(p0.type != RoutePoint::TRACK_START && !p0.isOfTypeWorking_InTrack(true))
                    continue;
                int worked = isWorked(boundary, p0, p1, it_width->second, *remainingAreaMap, plannerParameters.bePreciseWithRemainingAreaMap);
                if( worked > 0 )
                    somethingWorked = true;
                if( worked != 0)
                    continue;
                if(somethingWorked)
                    firstGoodIndexes.at(i).second = j;
                else
                    firstGoodIndexes.at(i).second = firstGoodIndexes.at(i).first;
                break;
            }
        }
    }

    //if the first good index for all routes is 0, nothing has been worked
    bool continueChecking = false;
    for(auto & fgi : firstGoodIndexes){
        if(fgi.second > 0){
            continueChecking = true;
            break;
        }
    }
    if(!continueChecking)
        return AroResp(0,"OK");

    //obtain the timestamp of the first good index in all routes
    for(size_t i = 0 ; i < firstGoodIndexes.size() ; ++i){
        auto &route = routes.at(i);
        auto& rp = route.route_points.at(firstGoodIndexes.at(i).second);
        if(deltaTime < 0 || deltaTime > rp.time_stamp ){
            deltaTime = rp.time_stamp;
            indFirstGood = i;
        }

        if(plannerParameters.removeInitialWorkedSegments && firstGoodIndexes.at(i).second > 0)//remove initial points
            route.route_points.erase(route.route_points.begin(), route.route_points.begin()+firstGoodIndexes.at(i).second);
    }

    //adjust the timestamps with the obtained deltaTime, setting the timestamps of the points before that deltatime to -1 (i.e. these point swill be read as 'worked' by the following planners)
    if(deltaTime > 0){
        for(size_t i = 0 ; i < routes.size() ; ++i){
            auto &route = routes.at(i);
            for(size_t j = 0 ; j < route.route_points.size() ; ++j){
                auto& rp = route.route_points.at(j);
                rp.time_stamp -= deltaTime;
                if(rp.time_stamp < -1e-5)
                    rp.time_stamp = -1;
            }
        }
    }


    return AroResp(0,"OK");
}

std::vector<Point> InfieldBaseRoutesPlanner::getHeadlandPart(const std::vector<Point> &headland,
                                                            const Point &headlandPoint0,
                                                            const Point &headlandPoint1,
                                                            bool includeP0,
                                                            bool includeP1,
                                                            double sampleResolution,
                                                            bool longest)
{

    std::vector<arolib::Point> HLPoints;

    if(longest)
        HLPoints = arolib::geometry::getLongestGeometryPart(headland, headlandPoint0, headlandPoint1, true);
    else
        HLPoints = arolib::geometry::getShortestGeometryPart(headland, headlandPoint0, headlandPoint1, true);

    while(HLPoints.size() > 1 &&
          arolib::geometry::checkPointInLine(HLPoints.at(0),
                           HLPoints.at(1),
                           headlandPoint0,
                           true) &&
          arolib::geometry::getLocationInLine(HLPoints.at(0),
                            HLPoints.at(1),
                            headlandPoint0) >= 0){
        HLPoints.erase(HLPoints.begin());
    }
    if( HLPoints.size() == 1 &&
        arolib::geometry::checkPointInLine(headlandPoint0,
                         headlandPoint1,
                         HLPoints.front(),
                         true) &&
        arolib::geometry::getLocationInLine(headlandPoint0,
                          headlandPoint1,
                          HLPoints.front()) < 0 ){
        HLPoints.erase(HLPoints.begin());
    }

    while(HLPoints.size() > 1 &&
          arolib::geometry::checkPointInLine(HLPoints.back(),
                           HLPoints.at( HLPoints.size()-2 ),
                           headlandPoint1,
                           true) &&
          arolib::geometry::getLocationInLine(HLPoints.back(),
                            HLPoints.at( HLPoints.size()-2 ),
                            headlandPoint1) >= 0){
        HLPoints.pop_back();
    }
    if(HLPoints.size() == 1 &&
       arolib::geometry::checkPointInLine(headlandPoint1,
                        headlandPoint0,
                        HLPoints.back(),
                        true) &&
       arolib::geometry::getLocationInLine(headlandPoint1,
                         headlandPoint0,
                         HLPoints.back()) < 0){
        HLPoints.pop_back();
    }

    HLPoints.insert(HLPoints.begin(), headlandPoint0);
    HLPoints.push_back(headlandPoint1);

    arolib::geometry::unsample_linestring(HLPoints, m_unsamplingTolerance);

    HLPoints = arolib::geometry::sample_geometry(HLPoints, sampleResolution);

    if(!includeP0)
        HLPoints.erase(HLPoints.begin());
    if(!includeP1)
        HLPoints.pop_back();

    return HLPoints;
}


std::vector<Point> InfieldBaseRoutesPlanner::getHeadlandSidesConnection(const std::vector<Point> &headland,
                                                                       const Point &headlandPoint0,
                                                                       const Point &headlandPoint1,
                                                                       const Point &control_point,
                                                                       bool includeP0,
                                                                       bool includeP1,
                                                                       double sampleResolution)
{
    std::vector<Point> HLPoints;

    //calculate the shortest connection
    HLPoints = getHeadlandPart(headland,
                               headlandPoint0,
                               headlandPoint1,
                               true,
                               true,
                               sampleResolution,
                               false);

    if( arolib::geometry::calc_dist_to_linestring(HLPoints,control_point, false) < 1e-3 )//the control point is part of the calculated (shortest) connection --> calculate the longest connection
        HLPoints = getHeadlandPart(headland,
                                   headlandPoint0,
                                   headlandPoint1,
                                   true,
                                   true,
                                   sampleResolution,
                                   true);

    if(!includeP0)
        HLPoints.erase(HLPoints.begin());
    if(!includeP1)
        HLPoints.pop_back();

    return HLPoints;

}

InfieldBaseRoutesPlanner::InternalMassCalculator::InternalMassCalculator(std::shared_ptr<IEdgeMassCalculator> base,
                                                                         std::shared_ptr<const ArolibGrid_t> massFactorMap,
                                                                         std::shared_ptr<gridmap::GridCellsInfoManager> cim,
                                                                         gridmap::SharedGridsManager::PreciseCalculationOption precision)
    : m_base(base), m_precision(precision)
{
    if(!m_factorMap || !m_factorMap->isAllocated())
        return;

    m_gridsManager.setCellsInfoManager(cim);
    m_gridsManager.addGrid("factorMap", massFactorMap);
}

double InfieldBaseRoutesPlanner::InternalMassCalculator::calcMass(const Point &p0, const Point &p1, double width)
{
    if(!m_base)
        return 0;

    double mass = m_base->calcMass(p0, p1, width);
    if(!m_factorMap || !m_factorMap->isAllocated())
        return mass;

    double area = arolib::geometry::calc_dist(p0, p1) * width;
    if( area < 1e-9 )
        return mass;

    bool errorTmp;
    double value;
    std::vector<gridmap::GridmapLayout::GridCellOverlap> cellsInfo;

    m_gridsManager.getCellsInfoUnderLine("factorMap", p0, p1, width, m_precision, cellsInfo);

    value = m_gridsManager.getGrid("factorMap")->getCellsComputedValue( cellsInfo,
                                                                        ArolibGrid_t::AVERAGE_TOTAL,
                                                                        area,
                                                                        false,
                                                                        &errorTmp );
    if(!errorTmp)
        mass *= value;
    return mass;
}

}

