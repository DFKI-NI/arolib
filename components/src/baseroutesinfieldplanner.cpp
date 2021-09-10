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
 
#include "arolib/components/baseroutesinfieldplanner.h"

namespace arolib {


const double BaseRoutesInfieldPlanner::m_thresholdIsWorked = 0.5;

bool BaseRoutesInfieldPlanner::PlannerParameters::parseFromStringMap(BaseRoutesInfieldPlanner::PlannerParameters &params, const std::map<std::string, std::string> &map, bool strict)
{
    BaseRoutesInfieldPlanner::PlannerParameters tmp;

    try{

        if( !FieldGeneralParameters::parseFromStringMap(tmp, map, strict) )
            return false;
        if( !GridComputationSettings::parseFromStringMap(tmp, map, strict) )
            return false;

        int sequenceStrategy;
        std::map<std::string, double*> dMap = { {"sampleResolutionHeadland" , &tmp.sampleResolutionHeadland} };
        std::map<std::string, int*> iMap = { {"tracksPerMachine" , &tmp.tracksPerMachine} };
        std::map<std::string, bool*> bMap = { {"inverseTrackOrder" , &tmp.inverseTrackOrder},
                                              {"inversePointsOrder" , &tmp.inversePointsOrder}};
        std::map<std::string, int*> enumMap = { {"sequenceStrategy" , &sequenceStrategy} };



        if( !setValuesFromStringMap( map, dMap, strict)
                || !setValuesFromStringMap( map, iMap, strict)
                || !setValuesFromStringMap( map, bMap, strict)
                || !setValuesFromStringMap( map, enumMap, strict) )
            return false;

        tmp.sequenceStrategy = SimpleTrackSequencer::intToSequenceStrategy( sequenceStrategy );

    } catch(...){ return false; }

    params = tmp;

    return true;

}

std::map<std::string, std::string> BaseRoutesInfieldPlanner::PlannerParameters::parseToStringMap(const BaseRoutesInfieldPlanner::PlannerParameters &params)
{
    std::map<std::string, std::string> ret, subMap;
    subMap = FieldGeneralParameters::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );
    subMap = GridComputationSettings::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );

    ret["sampleResolutionHeadland"] = double2string( params.sampleResolutionHeadland );
    ret["tracksPerMachine"] = std::to_string( params.tracksPerMachine );
    ret["inverseTrackOrder"] = std::to_string( params.inverseTrackOrder );
    ret["inversePointsOrder"] = std::to_string( params.inversePointsOrder );
    ret["sequenceStrategy"] = std::to_string( params.sequenceStrategy );

    return ret;

}

BaseRoutesInfieldPlanner::BaseRoutesInfieldPlanner(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{

}

AroResp BaseRoutesInfieldPlanner::plan(Subfield &subfield,
                                       const std::vector<Machine> &workinggroup,
                                       const PlannerParameters &plannerParameters,
                                       const ArolibGrid_t &remainingArea_map,
                                       const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                       IEdgeMassCalculator &edgeMassCalculator,
                                       IEdgeSpeedCalculator &edgeSpeedCalculator,
                                       std::vector<Route> &routes,
                                       const std::vector<Pose2D> &_initRefPoint)
{
    LoggingComponent::LoggersHandler lh(true);//will be reset on destruction

    if(m_calcGridValueOption != CALC_PW)
        LoggingComponent::setTemporalLoggersParent(lh, *this, remainingArea_map);

    auto initRefPoint = _initRefPoint;
    bool inverseFlagsSet = false;
    bool inverseTrackOrder = plannerParameters.inverseTrackOrder;
    bool inversePointsOrderStart = plannerParameters.inverseTrackOrder;

    //Get the indexes of the tracks that are completelly worked
    std::set<size_t> excludeTrackIndexes = getExcludeTrackIndexes(subfield, remainingArea_map, plannerParameters.bePreciseWithRemainingAreaMap);

    if(initRefPoint.empty() && remainingArea_map.isAllocated()){//Get the initial reference point and inverse flags based on the worked area map (and, if necessary, the current locations of the machines)
        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Obtaining inverse flags based on the remaining area map...");
        inverseFlagsSet = getInverseFlagsBasedOnRemainingArea(subfield,
                                                              excludeTrackIndexes,
                                                              workinggroup,
                                                              plannerParameters,
                                                              remainingArea_map,
                                                              machineCurrentStates,
                                                              inverseTrackOrder,
                                                              inversePointsOrderStart);
    }

    if(initRefPoint.empty() && !inverseFlagsSet){//Get the initial reference point and inverse flags based on the the current locations of the machines
        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Obtaining inverse flags based on the machines current location...");
        inverseFlagsSet = getInverseFlagsBasedOnMachineLocation(subfield,
                                                                excludeTrackIndexes,
                                                                workinggroup,
                                                                plannerParameters,
                                                                machineCurrentStates,
                                                                inverseTrackOrder,
                                                                inversePointsOrderStart);
    }

    if(!initRefPoint.empty() && !inverseFlagsSet){
        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Obtaining inverse flags based on the initial reference point...");
        getInverseFlagsBasedOnReferencePoint(subfield,
                                             excludeTrackIndexes,
                                             initRefPoint.front(),
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
                                      edgeMassCalculator,
                                      edgeSpeedCalculator,
                                      routes);

    if(!aroResp.isError())
        aroResp = adjustBaseRoutes(routes,
                                   subfield,
                                   workinggroup,
                                   edgeMassCalculator,
                                   remainingArea_map,
                                   plannerParameters);

    return aroResp;

}

AroResp BaseRoutesInfieldPlanner::plan(PlanningWorkspace &pw,
                                             size_t subfieldIdx,
                                             const BaseRoutesInfieldPlanner::PlannerParameters &plannerParameters,
                                             IEdgeMassCalculator &edgeMassCalculator,
                                             IEdgeSpeedCalculator &edgeSpeedCalculator,
                                             const std::vector<Pose2D> &_initRefPoint)
{
    if(subfieldIdx >= getField(pw).subfields.size()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfield index");
        return AroResp(1, "Invalid subfield index");
    }
    Subfield &subfield = getField(pw).subfields.at(subfieldIdx);
    const auto &workinggroup = getMachines(pw);
    const auto &remainingArea_map = getMaps(pw).at( PlanningWorkspace::GridType::REMAINING_AREA );
    const auto &machineCurrentStates = getMachineCurrentStates(pw);
    auto &routes = getBaseRoutes_infield(pw)[subfieldIdx];

    m_calcGridValueOption = CALC_PW;
    m_planningWorkspace = &pw;

    LoggingComponent::LoggersHandler lh(true);//will be reset on destruction
    LoggingComponent::setTemporalLoggersParent(lh, *this, pw);

    auto compResp = plan(subfield,
                         workinggroup,
                         plannerParameters,
                         remainingArea_map,
                         machineCurrentStates,
                         edgeMassCalculator,
                         edgeSpeedCalculator,
                         routes,
                         _initRefPoint);

    m_planningWorkspace = nullptr;
    m_calcGridValueOption = CALC_DIRECT;

    return compResp;
}

std::set<size_t> BaseRoutesInfieldPlanner::getExcludeTrackIndexes(const Subfield &subfield, const ArolibGrid_t &remainingArea_map, bool bePrecise)
{
    std::set<size_t> ret;
    if(!remainingArea_map.isAllocated())
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
            worked = isWorked(track.points.at(j), track.points.at(j+1), track.width, remainingArea_map, bePrecise);
            if(!worked)
                break;
        }
        if(worked)
            ret.insert(i);
    }

    return ret;
}

bool BaseRoutesInfieldPlanner::getInverseFlagsBasedOnRemainingArea(Subfield &subfield,
                                                                   const std::set<size_t>& excludeTrackIndexes,
                                                                   const std::vector<Machine> workinggroup,
                                                                   const BaseRoutesInfieldPlanner::PlannerParameters &plannerParameters,
                                                                   const ArolibGrid_t &remainingArea_map,
                                                                   const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
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
    else{//the complete track is not worked. make the decision based on machines' current position (i.e. the point closest to a machine)
        inFwdPointsOrder = true;
        double minDist = std::numeric_limits<double>::max();
        for(auto& m : workinggroup){
            if(m.isOfWorkingType(true)){
                auto it_m = machineCurrentStates.find(m.id);
                if(it_m != machineCurrentStates.end()){
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

bool BaseRoutesInfieldPlanner::getInverseFlagsBasedOnMachineLocation(Subfield &subfield,
                                                                     const std::set<size_t>& excludeTrackIndexes,
                                                                     const std::vector<Machine> workinggroup,
                                                                     const BaseRoutesInfieldPlanner::PlannerParameters &plannerParameters,
                                                                     const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
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

    for(auto& m : workinggroup){

        if(m.isOfWorkingType(true)){
            auto it_m = machineCurrentStates.find(m.id);
            if(it_m == machineCurrentStates.end())
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

    return locationOK;
}

bool BaseRoutesInfieldPlanner::getInverseFlagsBasedOnReferencePoint(Subfield &subfield,
                                                                    const std::set<size_t> &excludeTrackIndexes,
                                                                    const Pose2D &refPoint,
                                                                    const std::vector<Machine> workinggroup,
                                                                    const BaseRoutesInfieldPlanner::PlannerParameters &plannerParameters,
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

    double turningRad = 0.001;
    Machine refMachine;
    for(auto& m : workinggroup){
        if(m.isOfWorkingType(true) && turningRad < m.getTurningRadius()){
            turningRad = m.getTurningRadius();
            refMachine = m;
        }
    }

    InfieldTracksConnectorDef connector(&logger());
    bool foundConnection = false;
    double minDist = std::numeric_limits<double>::max();

    auto connection = connector.getConnection(refMachine,
                                              refPoint,
                                              Pose2D( firstTrack.front(), geometry::get_angle( firstTrack.front(), firstTrack.at(1) ) ),
                                              turningRad,
                                              -1,
                                              subfield.boundary_outer,
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

    connection = connector.getConnection(refMachine,
                                         refPoint,
                                         Pose2D( firstTrack.back(), geometry::get_angle( firstTrack.back(), r_at(firstTrack, 1) ) ),
                                         turningRad,
                                         -1,
                                         subfield.boundary_outer,
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


    connection = connector.getConnection(refMachine,
                                         refPoint,
                                         Pose2D( lastTrack.front(), geometry::get_angle( lastTrack.front(), lastTrack.at(1) ) ),
                                         turningRad,
                                         -1,
                                         subfield.boundary_outer,
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

    connection = connector.getConnection(refMachine,
                                         refPoint,
                                         Pose2D( lastTrack.back(), geometry::get_angle( lastTrack.back(), r_at(lastTrack, 1) ) ),
                                         turningRad,
                                         -1,
                                         subfield.boundary_outer,
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


bool BaseRoutesInfieldPlanner::isWorked(const Point &p0, const Point &p1, double width, const ArolibGrid_t &remainingArea_map, bool bePrecise)
{
    if(!remainingArea_map.isAllocated())
        return false;

    bePrecise &= width > 1e-5;

    bool errorTmp;
    double value;

    if(bePrecise){

        if(m_calcGridValueOption == CALC_PW){
            const auto &map = getMaps(*m_planningWorkspace).at( PlanningWorkspace::GridType::REMAINING_AREA );
            PlanningWorkspace::Edge edge( p0,
                                          p1,
                                          width);

            auto it_edge = getEdgesInfield(*m_planningWorkspace).find(edge);
            auto &ep = getEdgesInfield(*m_planningWorkspace)[edge];

            if(it_edge == getEdgesInfield(*m_planningWorkspace).end() || !ep.gridCellsInfoIsSet.at(PlanningWorkspace::GridType::REMAINING_AREA))
                ep.updateGridCells( PlanningWorkspace::GridType::REMAINING_AREA, map, edge, true );

            value = map.getCellsComputedValue( ep.gridCellsInfo.at(PlanningWorkspace::GridType::REMAINING_AREA),
                                               ArolibGrid_t::AVERAGE_TOTAL,
                                               arolib::geometry::calc_dist(edge.p0, edge.p1) * edge.width,
                                               false,
                                               &errorTmp );
        }
        else
            value = remainingArea_map.getLineComputedValue(p0,
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

    value = remainingArea_map.getValue(p0_1, &errorTmp);
    return (!errorTmp && value < m_thresholdIsWorked);

}

int BaseRoutesInfieldPlanner::isWorked(const Polygon &boundary, const Point &p0, const Point &p1, double width, const ArolibGrid_t &remainingArea_map, bool bePrecise)
{
    if(width <= 0)
        return -1;

    if(!remainingArea_map.isAllocated())
        return 0;

    if(boundary.points.empty())
        return isWorked(p0, p1, width, remainingArea_map, bePrecise);


    Polygon segmentPoly = geometry::createRectangleFromLine( p0, p1, width );

    size_t countPointsInside = 0;
    for(size_t i = 0 ; i+1 < segmentPoly.points.size(); ++i)
        countPointsInside += geometry::in_polygon(segmentPoly.points.at(i), boundary);

    if(countPointsInside > 3 || (countPointsInside == 3 && !bePrecise))
        return isWorked(p0, p1, width, remainingArea_map, bePrecise);

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
        double valueTmp = remainingArea_map.getPolygonComputedValue(intersectionPolys.at(i),
                                                                    ArolibGrid_t::AVERAGE_TOTAL,
                                                                    bePrecise
                                                                    &errorTmp);
        if(errorTmp)
            continue;
        value += valueTmp * areas.at(i) / area;
    }

    return (value < m_thresholdIsWorked);

}

bool BaseRoutesInfieldPlanner::hasBiomass(const Point &p0,
                                        const Point &p1,
                                        double workingWidth,
                                        IEdgeMassCalculator &edgeMassCalculator)
{
    return edgeMassCalculator.calcMass(p0, p1, workingWidth) > 1e-3;

}


AroResp BaseRoutesInfieldPlanner::generateBaseRoutes(const Subfield &subfield,
                                                     const std::set<size_t> &excludeTrackIndexes,
                                                     const std::vector<Machine> &workinggroup,
                                                     const PlannerParameters &plannerParameters,
                                                     bool inverseTrackOrder,
                                                     bool inversePointsOrderStart,
                                                     IEdgeMassCalculator &edgeMassCalculator,
                                                     IEdgeSpeedCalculator &edgeSpeedCalculator,
                                                     std::vector<Route> &routes)
{
    routes.clear();
    SimpleBaseRoutesPlanner sbrp(m_logger.logLevel());
    sbrp.logger().setParent(&m_logger);

    if (plannerParameters.sequenceStrategy == SimpleTrackSequencer::SequenceStrategy::INNER_TO_OUTER ||
            plannerParameters.sequenceStrategy == SimpleTrackSequencer::SequenceStrategy::OUTER_TO_INNER) {
        sbrp.setTracksPerMachine(plannerParameters.tracksPerMachine);
    } else {
        sbrp.setTracksPerMachine(1);
    }
    sbrp.setSubfield(subfield);
    sbrp.setStrategy(plannerParameters.sequenceStrategy);
    sbrp.setInverseTrackOrder(inverseTrackOrder);
    sbrp.setFirstTrackInversePointOrder(inversePointsOrderStart);
    sbrp.setExcludeTrackIndexes(excludeTrackIndexes);
    for (auto &m : workinggroup){
        if(m.isOfWorkingType(true))
            sbrp.addMachine(m);
    }

    for(auto &m : workinggroup){
        if (m.isOfWorkingType(true)) {
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Added machine " + std::to_string(m.id) + " to the machine plan");
            routes.push_back(Route());
            bool ok;
            if(m_calcGridValueOption == CALC_PW)
                ok = sbrp.getRoute(m.id, *m_planningWorkspace, routes.back(), edgeMassCalculator, edgeSpeedCalculator);
            else
                ok = sbrp.getRoute(m.id, routes.back(), edgeMassCalculator, edgeSpeedCalculator);
            routes.back().route_id = routes.size()-1;
            if (!ok)
                return AroResp(1,"Error obtaining machine route.");
        }
    }
    m_logger.printOut(LogLevel::INFO, __FUNCTION__, std::to_string(routes.size()) + " routes were generated");

    return AroResp(0,"OK");

}

AroResp BaseRoutesInfieldPlanner::adjustBaseRoutes(std::vector<Route> &routes,
                                                   const Subfield &subfield,
                                                   const std::vector<Machine> &workinggroup,
                                                   IEdgeMassCalculator &edgeMassCalculator,
                                                   const ArolibGrid_t &remainingArea_map,
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

        if(remainingArea_map.isAllocated()){
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
                int worked = isWorked(boundary, p0, p1, it_width->second, remainingArea_map, plannerParameters.bePreciseWithRemainingAreaMap);
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

std::vector<Point> BaseRoutesInfieldPlanner::getHeadlandPart(const std::vector<Point> &headland,
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


std::vector<Point> BaseRoutesInfieldPlanner::getHeadlandSidesConnection(const std::vector<Point> &headland,
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

}

