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
 
#include "arolib/components/headlandbaseroutesplanner.h"

namespace arolib {

const double HeadlandBaseRoutesPlanner::ThresholdIsWorked = 0.5;
const std::string HeadlandBaseRoutesPlanner::RemainingAreaMapName = "REM_AREA";
const std::string HeadlandBaseRoutesPlanner::MassFactorMapName = "MASS_FACTOR";


HeadlandBaseRoutesPlanner::MachineOrderStrategy HeadlandBaseRoutesPlanner::intToMachineOrderStrategy(int value)
{
    if(value == MachineOrderStrategy::AUTO)
        return MachineOrderStrategy::AUTO;
    else if(value == MachineOrderStrategy::KEEP_ORDER_RELATIVE_TO_FIRST_TRACK__STRICT)
        return MachineOrderStrategy::KEEP_ORDER_RELATIVE_TO_FIRST_TRACK__STRICT;
    else if(value == MachineOrderStrategy::KEEP_ORDER_RELATIVE_TO_FIRST_WORKING_TRACK__STRICT)
        return MachineOrderStrategy::KEEP_ORDER_RELATIVE_TO_FIRST_WORKING_TRACK__STRICT;
    else if(value == MachineOrderStrategy::KEEP_ORDER_RELATIVE_TO_FIRST_WORKING_TRACK)
        return MachineOrderStrategy::KEEP_ORDER_RELATIVE_TO_FIRST_WORKING_TRACK;

    throw std::invalid_argument( "The given value does not correspond to any HeadlandBaseRoutesPlanner::MachineOrderStrategy" );

}

HeadlandBaseRoutesPlanner::WorkedAreaTransitRestriction HeadlandBaseRoutesPlanner::intToWorkedAreaTransitRestriction(int value)
{
    if(value == WorkedAreaTransitRestriction::NO_RESTRICTION)
        return WorkedAreaTransitRestriction::NO_RESTRICTION;
    else if(value == WorkedAreaTransitRestriction::TRANSIT_ONLY_OVER_UNWORKED_AREA)
        return WorkedAreaTransitRestriction::TRANSIT_ONLY_OVER_UNWORKED_AREA;
    else if(value == WorkedAreaTransitRestriction::TRANSIT_ONLY_OVER_WORKED_AREA)
        return WorkedAreaTransitRestriction::TRANSIT_ONLY_OVER_WORKED_AREA;
    else if(value == WorkedAreaTransitRestriction::FROM_MACHINE_TYPE)
        return WorkedAreaTransitRestriction::FROM_MACHINE_TYPE;

    throw std::invalid_argument( "The given value does not correspond to any HeadlandBaseRoutesPlanner::WorkedAreaTransitRestriction" );
}

bool HeadlandBaseRoutesPlanner::PlannerParameters::parseFromStringMap(HeadlandBaseRoutesPlanner::PlannerParameters &params, const std::map<std::string, std::string> &map, bool strict)
{
    try{
        HeadlandBaseRoutesPlanner::PlannerParameters tmp;

        if( !FieldGeneralParameters::parseFromStringMap(tmp, map, strict) )
            return false;
        if( !GridComputationSettings::parseFromStringMap(tmp, map, strict) )
            return false;

        int machineOrderStrategy = MachineOrderStrategy::AUTO;
        int workedAreaTransitRestriction = WorkedAreaTransitRestriction::FROM_MACHINE_TYPE;

        std::map<std::string, double*> dMap = { {"speedMultiplier" , &tmp.speedMultiplier} };
        std::map<std::string, bool*> bMap = { {"startFromOutermostTrack" , &tmp.startFromOutermostTrack},
                                              {"finishWithOutermostTrack" , &tmp.finishWithOutermostTrack},
                                              {"clockwise" , &tmp.clockwise},
                                              {"removeInitialWorkedSegments" , &tmp.removeInitialWorkedSegments},
                                              {"restrictToBoundary" , &tmp.restrictToBoundary},
                                              {"monitorPlannedAreas" , &tmp.monitorPlannedAreas}};
        std::map<std::string, int*> enumMap = { {"machineOrderStrategy" , &machineOrderStrategy},
                                                {"workedAreaTransitRestriction" , &workedAreaTransitRestriction}};


        if( !setValuesFromStringMap( map, dMap, strict)
                || !setValuesFromStringMap( map, bMap, strict)
                || !setValuesFromStringMap( map, enumMap, strict) )
            return false;

        tmp.machineOrderStrategy = intToMachineOrderStrategy( machineOrderStrategy );
        tmp.workedAreaTransitRestriction = intToWorkedAreaTransitRestriction( workedAreaTransitRestriction );

        params = tmp;
        return true;
    }
    catch(...){
        return false;
    }
}

std::map<std::string, std::string> HeadlandBaseRoutesPlanner::PlannerParameters::parseToStringMap(const HeadlandBaseRoutesPlanner::PlannerParameters &params)
{
    std::map<std::string, std::string> ret, subMap;
    subMap = FieldGeneralParameters::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );
    subMap = GridComputationSettings::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );

    ret["speedMultiplier"] = double2string( params.speedMultiplier );
    ret["startFromOutermostTrack"] = std::to_string( params.startFromOutermostTrack );
    ret["finishWithOutermostTrack"] = std::to_string( params.finishWithOutermostTrack );
    ret["clockwise"] = std::to_string( params.clockwise );
    ret["removeInitialWorkedSegments"] = std::to_string( params.removeInitialWorkedSegments );
    ret["restrictToBoundary"] = std::to_string( params.restrictToBoundary );
    ret["monitorPlannedAreas"] = std::to_string( params.monitorPlannedAreas );
    ret["machineOrderStrategy"] = std::to_string( params.machineOrderStrategy );
    ret["workedAreaTransitRestriction"] = std::to_string( params.workedAreaTransitRestriction );

    return ret;
}

HeadlandBaseRoutesPlanner::HeadlandBaseRoutesPlanner(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{
    if(logger().logLevel() == LogLevel::DEBUG)
        logger().includeElapsedTime(true);
}

AroResp HeadlandBaseRoutesPlanner::plan(const Subfield &subfield,
                                        const std::vector<Machine> &workinggroup,
                                        const PlannerParameters &plannerParameters,
                                        std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                        std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                        std::vector<Route> &routes,
                                        std::shared_ptr<ArolibGrid_t> massFactorMap,
                                        const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                        const Pose2D *initRefPose,
                                        const OutFieldInfo *outFieldInfo,
                                        std::shared_ptr<const ArolibGrid_t> remainingAreaMap)
{
    LoggingComponent::LoggersHandler lh(true);//will be reset on destruction

    if(!edgeMassCalculator)
        return AroResp::LoggingResp(1, "A mass calculator must be given", m_logger, LogLevel::ERROR, __FUNCTION__);

    if(!edgeSpeedCalculator)
        return AroResp::LoggingResp(1, "A speed calculator must be given", m_logger, LogLevel::ERROR, __FUNCTION__);

    routes.clear();

    auto machines = getWorkingGroup(workinggroup);
    if(machines.empty())
        return AroResp::LoggingResp(1, "No working machines were given", m_logger, LogLevel::ERROR, __FUNCTION__);



    if(!subfield.headlands.complete.tracks.empty()){
        InternalParametersForCompleteHL ip;
        auto aroResp = initInternalGeneralParameters(ip,
                                                     subfield,
                                                     machines,
                                                     plannerParameters,
                                                     massFactorMap,
                                                     remainingAreaMap,
                                                     lh);
        if(aroResp.isError())
            return aroResp;
        return planForCompleteHL(subfield,
                                 workinggroup,
                                 machines,
                                 plannerParameters,
                                 *edgeMassCalculator,
                                 *edgeSpeedCalculator,
                                 initRefPose,
                                 machineCurrentStates,
                                 outFieldInfo,
                                 ip,
                                 routes);
    }
    if(subfield.headlands.partial.empty()){
        return AroResp::LoggingResp(1, "No headland available", m_logger, LogLevel::ERROR, __FUNCTION__);
    }
    for(auto& hl : subfield.headlands.partial){
        if(hl.tracks.empty()){
            return AroResp::LoggingResp(1, "One or more partial headlands has no tracks", m_logger, LogLevel::ERROR, __FUNCTION__);
        }
    }
    if(machines.size() > 1)
        return AroResp::LoggingResp(1, "Planning for more than one working machine in fields with partial headlands is currently not supported", m_logger, LogLevel::ERROR, __FUNCTION__);

    InternalParametersForPartialHLs ip;
    auto aroResp = initInternalGeneralParameters(ip,
                                                 subfield,
                                                 machines,
                                                 plannerParameters,
                                                 massFactorMap,
                                                 remainingAreaMap,
                                                 lh);
    if(aroResp.isError())
        return aroResp;
    return planForPartialHLs(subfield,
                             machines.begin()->second,
                             plannerParameters,
                             *edgeMassCalculator,
                             *edgeSpeedCalculator,
                             initRefPose,
                             machineCurrentStates,
                             outFieldInfo,
                             ip,
                             routes);

}

void HeadlandBaseRoutesPlanner::setGridCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim)
{
    m_cim = cim;
}

HeadlandBaseRoutesPlanner::WorkedAreaTransitRestriction HeadlandBaseRoutesPlanner::getWorkedAreaTransitRestriction(const WorkedAreaTransitRestriction &war, const std::map<MachineId_t, Machine> &machines)
{

    if(war != WorkedAreaTransitRestriction::FROM_MACHINE_TYPE)
        return war;
    for(auto& it_m : machines){
        const Machine& m = it_m.second;
        if(m.isOfWorkingTypeForMaterialInput())
            return WorkedAreaTransitRestriction::TRANSIT_ONLY_OVER_UNWORKED_AREA;
        if(m.isOfWorkingTypeForMaterialOutput())
            return WorkedAreaTransitRestriction::TRANSIT_ONLY_OVER_WORKED_AREA;
    }
    return WorkedAreaTransitRestriction::NO_RESTRICTION;
}


void HeadlandBaseRoutesPlanner::InternalParametersForPartialHLs::HLParams::update(const HLParamsBase &base)
{
    *((HLParamsBase*)this) = base;
}

std::map<MachineId_t, Machine> HeadlandBaseRoutesPlanner::getWorkingGroup(const std::vector<Machine> &workinggroup)
{
    std::map<MachineId_t, Machine> ret;
    for(auto& m : workinggroup){
        if(m.isOfWorkingType(true))
            ret[m.id] = m;
    }
    return ret;
}

AroResp HeadlandBaseRoutesPlanner::planForCompleteHL(const Subfield &subfield,
                                                     const std::vector<Machine> &workinggroup,
                                                     const std::map<MachineId_t, Machine> &machines,
                                                     const PlannerParameters & plannerParameters,
                                                     IEdgeMassCalculator & edgeMassCalculator,
                                                     IEdgeSpeedCalculator & edgeSpeedCalculator,
                                                     const Pose2D * initRefPose,
                                                     const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                                     const OutFieldInfo* outFieldInfo,
                                                     InternalParametersForCompleteHL &ip,
                                                     std::vector<Route> & routes)
{
    auto& headland = subfield.headlands.complete;
    auto& tracks = headland.tracks;

    ip.reverseTracksOrder = { ( tracks.size() > 1 &&
                                  geometry::calc_dist_to_linestring(subfield.boundary_outer.points, tracks.front().points.front())
                                      > geometry::calc_dist_to_linestring(subfield.boundary_outer.points, tracks.back().points.front()) ) };

    auto aroResp = getStartingParameters_completeHL(subfield,
                                                    machines,
                                                    plannerParameters,
                                                    initRefPose,
                                                    machineCurrentStates,
                                                    outFieldInfo,
                                                    ip);
    if(aroResp.isError())
        return AroResp::LoggingResp(1, "Error obtaining the starting parameters", ": " + aroResp.msg, m_logger, LogLevel::ERROR, __FUNCTION__);

    std::vector<MachineId_t> sortedMachineIds = sortMachines_completeHL(subfield,
                                                                        workinggroup,
                                                                        machines,
                                                                        plannerParameters.machineOrderStrategy,
                                                                        ip,
                                                                        machineCurrentStates,
                                                                        outFieldInfo);

    aroResp = generateRoutes_completeHL(subfield,
                                        machines,
                                        sortedMachineIds,
                                        plannerParameters,
                                        edgeMassCalculator,
                                        edgeSpeedCalculator,
                                        machineCurrentStates,
                                        ip,
                                        routes);
    if(aroResp.isError())
        return AroResp::LoggingResp(1, "Error generating the routes", ": " + aroResp.msg, m_logger, LogLevel::ERROR, __FUNCTION__);

    return AroResp::ok();
}

AroResp HeadlandBaseRoutesPlanner::planForPartialHLs(const Subfield &subfield,
                                                     const Machine &machine,
                                                     const PlannerParameters & plannerParameters,
                                                     IEdgeMassCalculator & edgeMassCalculator,
                                                     IEdgeSpeedCalculator & edgeSpeedCalculator,
                                                     const Pose2D * initRefPose,
                                                     const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                                     const OutFieldInfo* outFieldInfo,
                                                     InternalParametersForPartialHLs &ip,
                                                     std::vector<Route> & routes)
{
    auto aroResp = initPartialHeadlandsParams(subfield, ip);
    if(aroResp.isError())
        return aroResp;

    bool allHlsWorked = true;
    for(auto& hlParams : ip.hlParams){
        if(hlParams.state != InternalParametersForPartialHLs::PHL_WORKED){
            allHlsWorked = false;
            break;
        }
    }
    if(allHlsWorked){
        if(plannerParameters.removeInitialWorkedSegments){
            routes.clear();
            return AroResp::ok();
        }
        for(auto& hlParams : ip.hlParams){//reset all states to not worked and let the planner plan as it all were unworked
            hlParams.state = InternalParametersForPartialHLs::PHL_NOT_WORKED;
            hlParams.trackIsPartiallyWorked = false;
            for(size_t i = 0 ; i < hlParams.trackStates.size() ; ++i)
                hlParams.trackStates.at(i) = InternalParametersForPartialHLs::TRACK_NOT_WORKED;
            ip.gm.removeGrid(RemainingAreaMapName);
        }
    }


    aroResp = presortPartialHeadlands(subfield, ip);
    if(aroResp.isError())
        return AroResp::LoggingResp(1, "Error pre-sorting partial headlands", ": " + aroResp.msg, m_logger, LogLevel::ERROR, __FUNCTION__);

    aroResp = getStartingParameters_partialHLs(subfield,
                                               machine,
                                               plannerParameters,
                                               initRefPose,
                                               machineCurrentStates,
                                               outFieldInfo,
                                               ip);
    if(aroResp.isError())
        return AroResp::LoggingResp(1, "Error obtaining the starting parameters", ": " + aroResp.msg, m_logger, LogLevel::ERROR, __FUNCTION__);

    if(ip.indFirstWorkingHeadland < 0)
        return AroResp::LoggingResp(-1, "Unable to obtain potential first working headlands; probably all headlands are considered to be completelly worked", m_logger, LogLevel::WARNING, __FUNCTION__);

    routes.clear();
    routes.emplace_back(Route());
    aroResp = generateRoute_partialHLs(subfield,
                                        machine,
                                        plannerParameters,
                                        edgeMassCalculator,
                                        edgeSpeedCalculator,
                                        machineCurrentStates,
                                        ip,
                                        routes.front());
    if(aroResp.isError())
        return AroResp::LoggingResp(1, "Error generating routes", ": " + aroResp.msg, m_logger, LogLevel::ERROR, __FUNCTION__);

    if(allHlsWorked){
        for(auto& route : routes){
            for(auto& rp : route.route_points)
                rp.time_stamp = -1;
        }
    }

    return AroResp::ok();
}

AroResp HeadlandBaseRoutesPlanner::initInternalGeneralParameters(InternalParametersGeneral& ip,
                                                                 const Subfield &subfield,
                                                                 const std::map<MachineId_t, Machine> &machines,
                                                                 const PlannerParameters & plannerParameters,
                                                                 std::shared_ptr<ArolibGrid_t> massFactorMap,
                                                                 std::shared_ptr<const ArolibGrid_t> remainingAreaMap,
                                                                 LoggersHandler& lh)
{
    ip.gm.logger().setParent(loggerPtr());

    ip.precision_wam = ( plannerParameters.bePreciseWithRemainingAreaMap ? gridmap::SharedGridsManager::PRECISE : gridmap::SharedGridsManager::PRECISE_ONLY_IF_AVAILABLE );
    ip.precision_massMap = ( plannerParameters.bePreciseWithMatterMap ? gridmap::SharedGridsManager::PRECISE : gridmap::SharedGridsManager::PRECISE_ONLY_IF_AVAILABLE );

    ip.workedAreaTransitRestriction = getWorkedAreaTransitRestriction(plannerParameters.workedAreaTransitRestriction, machines);

    if(remainingAreaMap)
        LoggingComponent::setTemporalLoggersParent(lh, *this, *remainingAreaMap);

    if(m_cim)
        ip.gm.setCellsInfoManager(m_cim);
    if(remainingAreaMap && remainingAreaMap->isAllocated())
        ip.gm.addGrid(RemainingAreaMapName, remainingAreaMap);
    if(massFactorMap && massFactorMap->isAllocated())
        ip.gm.addGrid(MassFactorMapName, massFactorMap);

    ip.boundary = ( plannerParameters.restrictToBoundary ? &subfield.boundary_outer : nullptr );

    return AroResp::ok();
}

AroResp HeadlandBaseRoutesPlanner::getStartingParameters_completeHL(const Subfield &subfield,
                                                                    const std::map<MachineId_t, Machine> &machines,
                                                                    const PlannerParameters & plannerParameters,
                                                                    const Pose2D * pInitRefPose,
                                                                    const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                                                    const OutFieldInfo* outFieldInfo,
                                                                    InternalParametersForCompleteHL & ip)
{
    auto& headland = subfield.headlands.complete;
    auto& tracks = headland.tracks;

    //initialize parameters for trivial cases

    ip.indFirstTrack = ( !ip.reverseTracksOrder ? 0 : tracks.size()-1 );
    ip.indFirstTrack = ( plannerParameters.startFromOutermostTrack ? ip.indFirstTrack : tracks.size()-1-ip.indFirstTrack );
    ip.indFirstWorkingTrack = ip.indFirstTrack;

    ip.indFirstTrackFirstPoint = ip.indFirstWorkingTrackPoint = ip.indFirstWorkingTrackFirstPoint = 0;
    ip.clockwise = plannerParameters.clockwise;
    ip.startingMachineId = Machine::AllMachineIds;

    for(auto& track : tracks){
        if(track.points.size() < 3)
            return AroResp::LoggingResp(1, "One or more headland tracks has not enought points", m_logger, LogLevel::ERROR, __FUNCTION__);
    }

    std::vector<size_t> potentialFirstWorkingTrackPointIdxs;
    ip.trackIsPartiallyWorked = false;

    //if there exists a remaining-area map, get the first track with unworked area and indexes corresponding to starts of non-worked segments (in both directions)
    if(ip.gm.hasGrid(RemainingAreaMapName)){
        auto aroResp = getStartingParameters_completeHL_workedArea(tracks,
                                                                   machines,
                                                                   plannerParameters,
                                                                   ip,
                                                                   potentialFirstWorkingTrackPointIdxs);
        if(aroResp.isError())
            return aroResp;
    }

    if(ip.indFirstWorkingTrack >= tracks.size())
        return AroResp::LoggingResp(-1, "All of the tracks have been considered completelly worked", m_logger, LogLevel::WARNING, __FUNCTION__);

    auto& firstWorkingTrack = tracks.at(ip.indFirstWorkingTrack);
    auto& firstWorkingTrackPts = firstWorkingTrack.points;

    ip.trackIsClockwise = geometry::isPolygonClockwise(firstWorkingTrackPts, true);

    if(potentialFirstWorkingTrackPointIdxs.size() > 1)//the track is partially worked
        ip.trackIsPartiallyWorked = true;
    else{//all track points are potential starting points
        potentialFirstWorkingTrackPointIdxs.resize( firstWorkingTrackPts.size() );
        for(size_t i = 0 ; i < firstWorkingTrackPts.size() ; ++i)
            potentialFirstWorkingTrackPointIdxs[i] = i;
    }


    int indStartingPoint = -1;
    const Pose2D * pRefPose = pInitRefPose;
    Pose2D refPoseFromMachine;

    if(machineCurrentStates){
        auto aroResp = getStartingParameters_completeHL_machineStates(subfield,
                                                                      firstWorkingTrack,
                                                                      machines,
                                                                      *machineCurrentStates,
                                                                      refPoseFromMachine,
                                                                      potentialFirstWorkingTrackPointIdxs,
                                                                      ip,
                                                                      indStartingPoint);
        if(aroResp.isError())
            return aroResp;

        if(refPoseFromMachine.isValid())
            pRefPose = &refPoseFromMachine;
    }

    if(indStartingPoint < 0 && pInitRefPose && pInitRefPose->isValid()){//the machine states could not be used to get the information --> use pInitRefPoint
        auto aroResp = getStartingParameters_completeHL_refPose(firstWorkingTrack,
                                                                machines,
                                                                *pInitRefPose,
                                                                potentialFirstWorkingTrackPointIdxs,
                                                                ip,
                                                                indStartingPoint);
        if(aroResp.isError())
            return aroResp;
    }

    if(indStartingPoint < 0 && outFieldInfo){//neither the machine states nor the pInitRefPose could be used to get the information --> use the outFieldInfo to search for the "best" access point and use it as reference
        auto aroResp = getStartingParameters_completeHL_outFieldInfo(subfield,
                                                                     firstWorkingTrack,
                                                                     machines,
                                                                     machineCurrentStates,
                                                                     *outFieldInfo,
                                                                     potentialFirstWorkingTrackPointIdxs,
                                                                     ip,
                                                                     indStartingPoint);
        if(aroResp.isError())
            return aroResp;
    }


    if(indStartingPoint >= 0){
        ip.indFirstWorkingTrackPoint = potentialFirstWorkingTrackPointIdxs.at(indStartingPoint);
        if(ip.trackIsPartiallyWorked){
            ip.clockwise = ip.trackIsClockwise;
            if(indStartingPoint % 2 == 1){
                ip.clockwise = !ip.clockwise;
            }
        }

        auto aroResp = getStartingTrackPointIdx_completeHL(subfield,
                                                           potentialFirstWorkingTrackPointIdxs,
                                                           indStartingPoint,
                                                           ip);
        if(aroResp.isError())
            return aroResp;
    }

    return AroResp::ok();
}

AroResp HeadlandBaseRoutesPlanner::getStartingParameters_completeHL_workedArea(const std::vector<Track> &tracks,
                                                                               const std::map<MachineId_t, Machine> &machines,
                                                                               const PlannerParameters & plannerParameters,
                                                                               InternalParametersForCompleteHL & ip,
                                                                               std::vector<size_t>& potentialFirstWorkingTrackPointIdxs)
{
    auto precision_wam = ( plannerParameters.bePreciseWithRemainingAreaMap ? gridmap::SharedGridsManager::PRECISE : gridmap::SharedGridsManager::PRECISE_ONLY_IF_AVAILABLE );
    potentialFirstWorkingTrackPointIdxs.clear();
    ip.indFirstWorkingTrack = tracks.size();
    for(size_t i = 0 ; i < tracks.size() ; ++i){
        size_t indTrack = ( plannerParameters.startFromOutermostTrack ? i : tracks.size()-i-1 );
        indTrack = ( ip.reverseTracksOrder ? tracks.size()-indTrack-1 : indTrack );
        auto& track = tracks.at(indTrack);
        auto& track_pts = track.points;
        for(size_t j = 0 ; j+1 < track_pts.size() ; ++j){
            if( !isSegmentWorked(track_pts.at(j), track_pts.at(j+1), track.width, ip) ){
                ip.indFirstWorkingTrack = indTrack;
                break;
            }
            else if(machines.size() > 1)
                return AroResp::LoggingResp(1, "Planning for partially worked fields for more than one working machine is currently not supported", m_logger, LogLevel::ERROR, __FUNCTION__);

        }
        if(ip.indFirstWorkingTrack < tracks.size())
            break;
    }
    if(ip.indFirstWorkingTrack < tracks.size()){
        //get index of a worked segment
        int indWorked = -1;
        auto& track = tracks.at(ip.indFirstWorkingTrack);
        auto& track_pts = track.points;
        for(size_t j = 0 ; j+1 < track_pts.size() ; ++j){
            if( isSegmentWorked(track_pts.at(j), track_pts.at(j+1), track.width, ip) ){
                indWorked = j;
                break;
            }
        }
        if(indWorked >= 0){//some part of the track has been worked
            bool checkForNotWorked = true;
            for(size_t j = indWorked+1 ; j+1 < indWorked+track_pts.size() ; ++j){
                auto ind = geometry::getIndexFromContinuousGeometry(track_pts, j);
                if(ind+1 == track_pts.size())//last point = first point in a closed geometry
                    continue;
                if( checkForNotWorked ){
                    if (!isSegmentWorked(track_pts.at(ind), track_pts.at(ind+1), track.width, ip) ){
                        potentialFirstWorkingTrackPointIdxs.push_back(ind);
                        checkForNotWorked = false;
                    }
                }
                else {
                    if( isSegmentWorked(track_pts.at(ind), track_pts.at(ind+1), track.width, ip) ){
                        potentialFirstWorkingTrackPointIdxs.push_back(ind);
                        checkForNotWorked = true;
                    }
                }
                //note: in potentialFirstWorkingTrackPointIdxs, if the track direction is in the desired direction, the even indexes correspond to starting points of UNWORKED segments and the odd ones to starting points of WORKED, and viceversa. We need to check only the potential starting points of WORKED segments
            }
            if(potentialFirstWorkingTrackPointIdxs.size()%2 != 0)
                potentialFirstWorkingTrackPointIdxs.push_back(indWorked);
        }
    }
    return AroResp::ok();
}

AroResp HeadlandBaseRoutesPlanner::getStartingParameters_completeHL_machineStates(const Subfield &subfield,
                                                                                  const Track& firstWorkingTrack,
                                                                                  const std::map<MachineId_t, Machine> &machines,
                                                                                  const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                                                  Pose2D& refPoseFromMachine,
                                                                                  const std::vector<size_t> &potentialFirstWorkingTrackPointIdxs,
                                                                                  InternalParametersForCompleteHL & ip,
                                                                                  int & indStartingPoint)
{
    int indStartingPoint2 = -1;
    refPoseFromMachine.point() = Point::invalidPoint();
    Pose2D refPoseFromMachine2;
    auto& trackPts = firstWorkingTrack.points;

    for(auto& it_m : machines){
        const Machine &machine = it_m.second;
        auto it_mdi = machineCurrentStates.find( machine.id );
        if(it_mdi != machineCurrentStates.end()){
            const MachineDynamicInfo& mdi = it_mdi->second;
            bool machineInField = geometry::in_polygon(it_mdi->second.position, subfield.boundary_outer);
            bool machineInHeadland = machineInField && !geometry::in_polygon(it_mdi->second.position, subfield.boundary_inner);
            double minDistInHeadland = std::numeric_limits<double>::max();
            double minDistNotInHeadland = std::numeric_limits<double>::max();
            if (machineInField){// if the machine is in the field, get the potential starting point that is closest to the machine
                size_t i_start = 0;
                size_t delta_i = 1;
                if(ip.keepGivenClockwise){
                    delta_i = 2;
                    if(ip.clockwise != ip.trackIsClockwise)
                        i_start = 1;
                }
                for(size_t i = i_start ; i < potentialFirstWorkingTrackPointIdxs.size() ; i+=delta_i){
                    double dist = geometry::calc_dist(trackPts.at( potentialFirstWorkingTrackPointIdxs.at(i) ), mdi.position);
                    if(machineInHeadland && minDistInHeadland > dist){
                        minDistInHeadland = dist;
                        indStartingPoint = i;
                        ip.startingMachineId = machine.id;
                        refPoseFromMachine = Pose2D(mdi.position, mdi.theta);
                    }
                    else if(minDistNotInHeadland > dist){
                        minDistNotInHeadland = dist;
                        indStartingPoint2 = i;
                        refPoseFromMachine2 = Pose2D(mdi.position, mdi.theta);
                    }
                }
            }
        }
    }
    if( (indStartingPoint < 0 ) && (indStartingPoint2 >= 0) ){//no machines were in the headland, but some were in the field
        indStartingPoint = indStartingPoint2;
        refPoseFromMachine = refPoseFromMachine2;
    }

    return AroResp::ok();
}

AroResp HeadlandBaseRoutesPlanner::getStartingParameters_completeHL_refPose(const Track& firstWorkingTrack,
                                                                            const std::map<MachineId_t, Machine> &machines,
                                                                            const Pose2D& initRefPose,
                                                                            const std::vector<size_t>& potentialFirstWorkingTrackPointIdxs,
                                                                            InternalParametersForCompleteHL & ip,
                                                                            int & indStartingPoint)
{
    auto& trackPts = firstWorkingTrack.points;
    double minDist = std::numeric_limits<double>::max();
    double turningRadTmp = -1;
    for(auto& it_m : machines)
        turningRadTmp = std::max(turningRadTmp, it_m.second.getTurningRadius());
    size_t i_start = 0;
    size_t delta_i = 1;
    if(ip.keepGivenClockwise){
        delta_i = 2;
        if(ip.clockwise != ip.trackIsClockwise)
            i_start = 1;
    }
    for(size_t i = i_start ; i < potentialFirstWorkingTrackPointIdxs.size() ; i+=delta_i){
        double dist;
        if(ip.trackIsPartiallyWorked || turningRadTmp < 1e-5)
            dist = geometry::calc_dist(trackPts.at( potentialFirstWorkingTrackPointIdxs.at(i) ), initRefPose);
        else{
            size_t ind = potentialFirstWorkingTrackPointIdxs.at(i);
            Pose2D poseTrack( trackPts.at( ind ) );
            if(ip.clockwise == ip.trackIsClockwise)
                poseTrack.angle = geometry::get_angle( trackPts.at( ind ), trackPts.at( ind+1 ) );
            else if (ind > 0)
                poseTrack.angle = geometry::get_angle( trackPts.at( ind ), trackPts.at( ind-1 ) );
            else
                poseTrack.angle = geometry::get_angle( trackPts.at( ind ), r_at( trackPts, 1 ) );
            dist = geometry::calcDubinsPathLength(initRefPose, poseTrack, turningRadTmp);
        }
        if(minDist > dist){
            minDist = dist;
            indStartingPoint = i;
        }
    }
    return AroResp::ok();
}

AroResp HeadlandBaseRoutesPlanner::getStartingParameters_completeHL_outFieldInfo(const Subfield &subfield,
                                                                                 const Track& firstWorkingTrack,
                                                                                 const std::map<MachineId_t, Machine> &machines,
                                                                                 const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                                                                 const OutFieldInfo& outFieldInfo,
                                                                                 const std::vector<size_t> &potentialFirstWorkingTrackPointIdxs,
                                                                                 InternalParametersForCompleteHL & ip,
                                                                                 int & indStartingPoint)
{
    //@note: this method assumes that all access ponts are at the same distance of the track, hence it does not include the check for transit times between the access points and the potential first points

    auto& trackPts = firstWorkingTrack.points;
    double trackLength = geometry::getGeometryLength(trackPts);
    double minTime = std::numeric_limits<double>::max();
    int fapIdx = -1;
    for(auto& it_m : machines){
        const Machine& machine = it_m.second;
        double bunkerMass = 0;
        OutFieldInfo::MachineBunkerState bunkerState1 = OutFieldInfo::MACHINE_EMPTY;
        OutFieldInfo::MachineBunkerState bunkerState2 = OutFieldInfo::MACHINE_LOADED;
        if(machineCurrentStates){
            auto it_mdi = machineCurrentStates->find(machine.id);
            if(it_mdi != machineCurrentStates->end()){
                const MachineDynamicInfo& mdi = it_mdi->second;
                bunkerMass = mdi.bunkerMass;
                if(mdi.bunkerMass > machine.bunker_mass * 0.5)
                    std::swap(bunkerState1, bunkerState2);
            }
        }

        double speed = (machine.def_working_speed > 1e-5 ? machine.def_working_speed : machine.calcSpeed(0.5 * machine.bunker_mass));
        speed = (speed > 1e-5 ? speed : 0.5);//default speed if no info is given in the machine (shouldn't happen)
        double trackDuration = trackLength / speed; //specific for the machine

        for(size_t i = 0; i < subfield.access_points.size() ; ++i){
            auto& fap = subfield.access_points.at(i);
            OutFieldInfo::TravelCosts tc;
            if( !outFieldInfo.getArrivalCost(fap.id, machine.id, bunkerState1, tc) ){
                if( !outFieldInfo.getArrivalCost(fap.id, machine.id, bunkerState2, tc) )
                    continue;
            }
            if(ip.trackIsPartiallyWorked){ //not all track points are considered potential starting points -> add the estimated duration to each potential starting point (we do not do this if all points are ponential starting points because we asume that all access points are more or less at the same distance to all track points)
                double speed_transit = getTransitSpeed(machine, bunkerMass);

                size_t j_start = 0;
                size_t delta_j = 1;
                if(ip.keepGivenClockwise){
                    delta_j = 2;
                    if(ip.clockwise == ip.trackIsClockwise)//note: we will search for the closest start to a WORKED! segment (i.e., estimating where did the machine started working)
                        j_start = 1;
                }

                for(size_t j = j_start ; j < potentialFirstWorkingTrackPointIdxs.size() ; j+=delta_j ){ //note: if the track if the track direction is in the desired direction, the even indexes correspond to starting points of UNWORKED segments and the odd ones to starting points of WORKED, and viceversa. We need to check only the potential starting points of WORKED segments
                    double dist = geometry::calc_dist( fap, trackPts.at( potentialFirstWorkingTrackPointIdxs.at(j) ) );
                    double time = trackDuration + tc.time + ( dist / speed_transit );
                    if(minTime > time){
                        minTime = time;
                        if(ip.clockwise != ip.trackIsClockwise)
                            indStartingPoint = ( j > 0 ? j-1 : potentialFirstWorkingTrackPointIdxs.size()-1 );
                        else
                            indStartingPoint = ( j+1 < potentialFirstWorkingTrackPointIdxs.size() ? j+1 : 0 );
                    }
                }
            }
            else {
                double time = trackDuration + tc.time;
                if(minTime > time){
                    minTime = time;
                    fapIdx = i;
                }
            }
        }
    }
    if(fapIdx >= 0){//this will be done only if sp.trackIsPartiallyWorked = false
        double minDist = std::numeric_limits<double>::max();
        for(size_t i = 0 ; i < potentialFirstWorkingTrackPointIdxs.size() ; ++i){
            double dist = geometry::calc_dist( trackPts.at( potentialFirstWorkingTrackPointIdxs.at(i) ), subfield.access_points.at(fapIdx) );
            if(minDist > dist){
                minDist = dist;
                indStartingPoint = i;
            }
        }
    }
    else if(indStartingPoint < 0){//nothing was obtained (no outfieldinfo) -> check the track point closest to an access point
        double minDist = std::numeric_limits<double>::max();
        int delta_i = 1 + ip.trackIsPartiallyWorked;
        int start_i = ( !ip.trackIsPartiallyWorked ? 0 : (ip.clockwise != ip.trackIsClockwise) );//if partially worked, only check even or odd indexes (depending on the track direction and desired direction)
        for(auto& fap : subfield.access_points){
            for(size_t i = start_i ; i < potentialFirstWorkingTrackPointIdxs.size() ; i+=delta_i){
                double dist = geometry::calc_dist( trackPts.at( potentialFirstWorkingTrackPointIdxs.at(i) ), fap );
                if(minDist > dist){
                    minDist = dist;
                    indStartingPoint = i;
                }
            }
        }

    }

    return AroResp::ok();
}

AroResp HeadlandBaseRoutesPlanner::getStartingTrackPointIdx_completeHL(const Subfield &subfield,
                                                                       const std::vector<size_t> &potentialFirstWorkingTrackPointIdxs,
                                                                       size_t indStartingPoint,
                                                                       InternalParametersForCompleteHL & ip)
{
    auto& tracks = subfield.headlands.complete.tracks;
    ip.indFirstWorkingTrackFirstPoint = potentialFirstWorkingTrackPointIdxs.at(indStartingPoint);
    if(ip.trackIsPartiallyWorked){
        size_t indTmp;
        if(ip.clockwise == ip.trackIsClockwise)
            indTmp = ( indStartingPoint > 0 ? indStartingPoint-1 : potentialFirstWorkingTrackPointIdxs.size()-1 );
        else
            indTmp = ( indStartingPoint+1 < potentialFirstWorkingTrackPointIdxs.size() ? indStartingPoint+1 : 0 );
        ip.indFirstWorkingTrackFirstPoint = potentialFirstWorkingTrackPointIdxs.at(indTmp);
    }
    if(ip.indFirstTrack == ip.indFirstWorkingTrack)
        ip.indFirstTrackFirstPoint = ip.indFirstWorkingTrackFirstPoint;
    else{
        std::multimap<double, size_t> distIdxsMap;
        auto& trackStartPts = tracks.at(ip.indFirstTrack).points;
        Point pt0WorkingTrack = tracks.at(ip.indFirstWorkingTrack).points.at(ip.indFirstWorkingTrackFirstPoint);
        for(size_t i = 0 ; i+1 < trackStartPts.size() ; ++i)
            distIdxsMap.insert( std::make_pair( geometry::calc_dist( pt0WorkingTrack, trackStartPts.at(i) ), i ) );
        ip.indFirstTrackFirstPoint = distIdxsMap.begin()->second;
        double minDistRef = distIdxsMap.begin()->first;
        double minDist = std::numeric_limits<double>::max();
        for(auto& fap : subfield.access_points){
            for(auto& it_idx : distIdxsMap){
                if(it_idx.first > minDistRef + 0.1)
                    break;
                double dist = geometry::calc_dist( fap, trackStartPts.at( it_idx.second ) );
                if(minDist > dist){
                    minDist = dist;
                    ip.indFirstTrackFirstPoint = it_idx.second;
                }
            }
        }

    }

    return AroResp::ok();
}

std::vector<MachineId_t> HeadlandBaseRoutesPlanner::sortMachines_completeHL(const Subfield &subfield,
                                                                            const std::vector<Machine> &workinggroup,
                                                                            const std::map<MachineId_t, Machine> &machines,
                                                                            MachineOrderStrategy machineOrderStrategy,
                                                                            const InternalParametersForCompleteHL & ip,
                                                                            const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                                                            const OutFieldInfo* outFieldInfo)
{
    auto getOriginalOrder = [&workinggroup, &machines]() -> std::vector<MachineId_t>{
        std::vector<MachineId_t> ids;
        for(auto& m : workinggroup){
            auto it_m = machines.find(m.id);
            if(it_m != machines.end())
                ids.push_back(m.id);
        }
        return ids;
    };

    std::vector<MachineId_t> sortedMachineIds;

    if( machineOrderStrategy == MachineOrderStrategy::KEEP_ORDER_RELATIVE_TO_FIRST_TRACK__STRICT
            || ip.indFirstWorkingTrack >= subfield.headlands.complete.tracks.size() ){
        sortedMachineIds = getOriginalOrder();
        return sortedMachineIds;
    }

    if( machineOrderStrategy == MachineOrderStrategy::KEEP_ORDER_RELATIVE_TO_FIRST_WORKING_TRACK__STRICT
            || ( machineOrderStrategy == MachineOrderStrategy::KEEP_ORDER_RELATIVE_TO_FIRST_WORKING_TRACK && ip.startingMachineId == Machine::AllMachineIds ) ){
        int deltaInd = (int)ip.indFirstWorkingTrackFirstPoint - ip.indFirstWorkingTrack;
        if(deltaInd == 0)
            return getOriginalOrder();
        if(deltaInd < 0) deltaInd *= -1;

        auto sortedMachineIdsTmp = getOriginalOrder();
        for(int i = 0 ; i < sortedMachineIdsTmp.size() ; ++i)
            sortedMachineIds.push_back( get_index_from_cyclic_container(sortedMachineIdsTmp, i-deltaInd) );
        return sortedMachineIds;
    }

    if(machineOrderStrategy == MachineOrderStrategy::KEEP_ORDER_RELATIVE_TO_FIRST_WORKING_TRACK){
        auto sortedMachineIdsTmp = getOriginalOrder();
        int deltaInd = 0;
        for(; deltaInd < sortedMachineIdsTmp.size(); ++deltaInd){
            if(sortedMachineIdsTmp.at(deltaInd) == ip.startingMachineId)
                break;
        }
        for(int i = 0 ; i < sortedMachineIdsTmp.size() ; ++i)
            sortedMachineIds.push_back( get_index_from_cyclic_container(sortedMachineIdsTmp, i-deltaInd) );
        return sortedMachineIds;
    }

    bool trackPtsInReverse;
    std::multimap<double, int> timeIdsMap;
    auto& track = subfield.headlands.complete.tracks.at( ip.indFirstWorkingTrack );
    auto& trackPts = track.points;
    Point refPt = trackPts.at( ip.indFirstWorkingTrackPoint );
    auto trackSegment = getUnworkedTrackSegment_completeHL(subfield, ip, trackPtsInReverse);
    auto trackSegmentLength = geometry::getGeometryLength(trackSegment);

    if(ip.startingMachineId != Machine::AllMachineIds)
        sortedMachineIds.push_back(ip.startingMachineId);

    for(auto& it_m : machines){
        const Machine& machine = it_m.second;
        if(machine.id == ip.startingMachineId)//already added
            continue;

        double speed = (machine.def_working_speed > 1e-5 ? machine.def_working_speed : machine.calcSpeed(0.5 * machine.bunker_mass));
        speed = (speed > 1e-5 ? speed : 0.5);//default speed if no info is given in the machine (shouldn't happen)
        double trackDuration = trackSegmentLength / speed;

        double speed_transit = getTransitSpeed(machine, 0);

        OutFieldInfo::MachineBunkerState bunkerState1 = OutFieldInfo::MACHINE_EMPTY;
        OutFieldInfo::MachineBunkerState bunkerState2 = OutFieldInfo::MACHINE_LOADED;

        if(machineCurrentStates){
            auto it_mdi = machineCurrentStates->find( machine.id );
            if(it_mdi != machineCurrentStates->end()){
                const MachineDynamicInfo& mdi = it_mdi->second;

                speed_transit = getTransitSpeed(machine, mdi.bunkerMass);

                if( geometry::in_polygon(mdi.position, subfield.boundary_outer) ){
                    Pose2D pose0(mdi.position, mdi.theta);
                    Pose2D pose1(refPt);
                    if(ip.clockwise == ip.trackIsClockwise)
                        pose1.angle = geometry::get_angle( trackPts.at( ip.indFirstWorkingTrackPoint ), trackPts.at( ip.indFirstWorkingTrackPoint+1 ) );
                    else if (ip.indFirstWorkingTrackPoint > 0)
                        pose1.angle = geometry::get_angle( trackPts.at( ip.indFirstWorkingTrackPoint ), trackPts.at( ip.indFirstWorkingTrackPoint-1 ) );
                    else
                        pose1.angle = geometry::get_angle( trackPts.at( ip.indFirstWorkingTrackPoint ), r_at( trackPts, 1 ) );
                    double dist = geometry::calcDubinsPathLength(pose0, pose1, machine.getTurningRadius());

                    timeIdsMap.insert( std::make_pair(trackDuration + dist/speed_transit, machine.id) );
                    continue;
                }

                if(mdi.bunkerMass > machine.bunker_mass * 0.5)
                    std::swap(bunkerState1, bunkerState2);
            }
        }

        double minTime = std::numeric_limits<double>::max();
        bool found = false;
        for(size_t i = 0; i < subfield.access_points.size() ; ++i){
            auto& fap = subfield.access_points.at(i);
            double distToPt = geometry::calc_dist(fap, refPt);
            double timeFapToPt = distToPt / speed_transit;
            OutFieldInfo::TravelCosts tc;
            tc.time = timeFapToPt;

            if(outFieldInfo){
                if( outFieldInfo->getArrivalCost(fap.id, machine.id, bunkerState1, tc)
                        || outFieldInfo->getArrivalCost(fap.id, machine.id, bunkerState2, tc) ){
                    tc.time += timeFapToPt;
                }
                else
                    tc.time = timeFapToPt;
            }

            if(minTime > tc.time){
                minTime = tc.time;
                found = true;
            }
        }
        if(found){
            timeIdsMap.insert( std::make_pair(trackDuration + minTime, machine.id) );
            continue;
        }

        timeIdsMap.insert( std::make_pair(trackDuration, machine.id) );

    }

    for(auto& it_mid : timeIdsMap)
        sortedMachineIds.push_back(it_mid.second);

    return sortedMachineIds;
}

AroResp HeadlandBaseRoutesPlanner::generateRoutes_completeHL(const Subfield &subfield,
                                                             const std::map<MachineId_t, Machine> &machines,
                                                             const std::vector<MachineId_t> &sortedMachineIds,
                                                             const PlannerParameters & plannerParameters,
                                                             IEdgeMassCalculator & edgeMassCalculator,
                                                             IEdgeSpeedCalculator & edgeSpeedCalculator,
                                                             const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                                             InternalParametersForCompleteHL & ip,
                                                             std::vector<Route> & routes)
{
    routes.clear();

    double minSpeed = 0.01;//@TODO hardcoded. should we get this as a parameter?
    int startTrackIdx = ( plannerParameters.removeInitialWorkedSegments ? ip.indFirstWorkingTrack : ip.indFirstTrack );
    int deltaTrackIdx = ( plannerParameters.startFromOutermostTrack ? 1 : -1 ) * ( ip.reverseTracksOrder ? -1 : 1 );
    int deltaMachineIdx = ( plannerParameters.removeInitialWorkedSegments ? (int)ip.indFirstWorkingTrack  - ip.indFirstTrack : 0);
    if(deltaMachineIdx < 0) deltaMachineIdx *= -1;
    size_t startMachineIdx = at_cyclic( sortedMachineIds, deltaMachineIdx );

    auto& tracks = subfield.headlands.complete.tracks;

    routes.resize( std::min(tracks.size(), sortedMachineIds.size()) );
    for(size_t i = 0 ; i < routes.size() ; ++i){
        routes[i].route_id = i;
        routes[i].machine_id = at_cyclic( sortedMachineIds, i + startMachineIdx );
    }
    if(routes.empty())
        return AroResp::LoggingResp(1, "No routes can be planned", m_logger, LogLevel::ERROR, __FUNCTION__);

    size_t routeIdx = 0;
    Point prevTrackLastPoint = Point::invalidPoint();

    //add non-working segments
    if(!plannerParameters.removeInitialWorkedSegments){
        for(int trackIdx = ip.indFirstTrack ; trackIdx >= 0 && trackIdx < tracks.size() ; trackIdx += deltaTrackIdx){
            auto& route = routes.at(routeIdx);
            auto& machine = machines.at(route.machine_id);
            auto& track = tracks.at(trackIdx);

            size_t indFrom = 0;
            if(trackIdx == ip.indFirstWorkingTrack){
                if(ip.indFirstWorkingTrackPoint == ip.indFirstWorkingTrackFirstPoint)
                    break;// the complete track will be added next
                indFrom = ip.indFirstWorkingTrackFirstPoint;
            }
            else if(trackIdx == startTrackIdx)
                indFrom = ip.indFirstTrackFirstPoint;
            else if (prevTrackLastPoint.isValid())
                indFrom = geometry::getGeomIndex(track.points, prevTrackLastPoint);

            size_t indTo = indFrom;
            if(trackIdx == ip.indFirstWorkingTrack) //for partially worked tracks, the first working point is not the same as the first track point
                indTo = ip.indFirstWorkingTrackFirstPoint;

            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Adding (worked) points from headland track (id) " + std::to_string(track.id) + " to route (id) " + std::to_string(route.route_id) + " of machine (id) " + std::to_string(machine.id));

            addTrackPointsToRoute_completeHL(route,
                                             machine,
                                             track,
                                             indFrom,
                                             indTo,
                                             false,
                                             edgeMassCalculator,
                                             edgeSpeedCalculator,
                                             ip,
                                             plannerParameters,
                                             machineCurrentStates);

            if(!route.route_points.empty())
                prevTrackLastPoint = route.route_points.back().point();

            if(trackIdx == ip.indFirstWorkingTrack)
                break; //do not change the route, it will continue in the next part

            ++routeIdx;
            if(routeIdx >= routes.size())
                routeIdx = 0;
        }
    }

    //add working segments
    for(int trackIdx = ip.indFirstWorkingTrack ; trackIdx >= 0 && trackIdx < tracks.size() ; trackIdx += deltaTrackIdx){
        auto& route = routes.at(routeIdx);
        auto& machine = machines.at(route.machine_id);
        auto& track = tracks.at(trackIdx);

        size_t indFrom = 0;
        if(trackIdx == ip.indFirstWorkingTrack)
            indFrom = ip.indFirstWorkingTrackPoint;
        else if (prevTrackLastPoint.isValid())
            indFrom = geometry::getGeomIndex(track.points, prevTrackLastPoint);

        size_t indTo = indFrom;
        if(trackIdx == ip.indFirstWorkingTrack) //for partially worked tracks, the first working point is not the same as the first track point
            indTo = ip.indFirstWorkingTrackFirstPoint;

        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Adding points from headland track (id) " + std::to_string(track.id) + " to route (id) " + std::to_string(route.route_id) + " of machine (id) " + std::to_string(machine.id));

        addTrackPointsToRoute_completeHL(route,
                                         machine,
                                         track,
                                         indFrom,
                                         indTo,
                                         true,
                                         edgeMassCalculator,
                                         edgeSpeedCalculator,
                                         ip,
                                         plannerParameters,
                                         machineCurrentStates);

        if(!route.route_points.empty())
            prevTrackLastPoint = route.route_points.back().point();

        ++routeIdx;
        if(routeIdx >= routes.size())
            routeIdx = 0;
    }

    for(size_t i = 0 ; i < routes.size() ; ++i){
        if(routes.at(i).route_points.size() < 2){
            routes.erase(routes.begin()+i);
            --i;
        }
    }

    if(routes.empty() && ip.indFirstWorkingTrack < tracks.size())
        return AroResp::LoggingResp(1, "No routes could be planned", m_logger, LogLevel::ERROR, __FUNCTION__);

    return AroResp::ok();
}

void HeadlandBaseRoutesPlanner::addTrackPointsToRoute_completeHL(Route& route,
                                                                 const Machine& machine,
                                                                 const Track& track,
                                                                 int indFrom, int indTo,
                                                                 bool isWorking,
                                                                 IEdgeMassCalculator &edgeMassCalculator,
                                                                 IEdgeSpeedCalculator &edgeSpeedCalculator,
                                                                 InternalParametersForCompleteHL & ip,
                                                                 const PlannerParameters & plannerParameters,
                                                                 const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates)
{
    const bool useSpeedCalculatorTurningTime = true;

    bool ptsInReverse;
    auto points = getTrackSegment_completeHL(track, ip, indFrom, indTo, true, ptsInReverse);

    if(points.size() < 2)
        return;

    const double minSpeed = 0.01;//@TODO hardcoded. should we get this as a parameter?
    double speedFactor = (plannerParameters.speedMultiplier > 0 ? plannerParameters.speedMultiplier : 1);

    double width = ( machine.working_width > 1e-5 ? machine.working_width :
                                                    machine.width > 1e-5 ? machine.width :
                                                                           track.width );
    auto getTurningTime = [&edgeSpeedCalculator, &machine](const Point& p1, const Point& p2, const Point& p3)->double{
        if(p1 == p2 || p2 == p3)
            return 0;
        double ang = geometry::get_angle(p1, p2, p3);
        return(edgeSpeedCalculator.calcTurningTime(ang, 0, machine));
    };

    route.route_points.reserve( route.route_points.size() + points.size() );

    for(size_t i = 0 ; i < points.size() ; ++i){
        RoutePoint rp(points.at(i));
        rp.track_id = track.id;
        rp.type = RoutePoint::getDefaultRPType(machine);

        if (route.route_points.empty()) {
            rp.type = RoutePoint::TRACK_START;
            rp.time_stamp = ( isWorking ? 0.0 : -1 );
            rp.bunker_mass = 0.0;
            rp.bunker_volume = 0.0;
            rp.worked_mass = 0.0;
            rp.worked_volume = 0.0;
            if(machineCurrentStates){
                auto it_m = machineCurrentStates->find(machine.id);
                if(it_m != machineCurrentStates->end()){
                    const MachineDynamicInfo& mdi = it_m->second;
                    rp.bunker_mass = mdi.bunkerMass;
                    rp.bunker_volume = mdi.bunkerVolume;
                }
            }
        }
        else{
            auto& rpPrev = route.route_points.back();

            rp.copyBasicWorkingValuesFrom(rpPrev);
            rp.point() = points.at(i);

            bool isSegmentWorked;

            if(i == points.size() - 1)
                rp.type = RoutePoint::TRACK_END;
            else if (i == 0 && rpPrev.track_id != track.id)
                rp.type = RoutePoint::TRACK_START;

            double speedFactor2 = 1;

            if(rpPrev.track_id  != track.id)
                speedFactor2 = 0.5;


            if(!useSpeedCalculatorTurningTime){
                if(route.route_points.size() > 2){
                    const double minFactor = 0.05;
                    auto& rpPrev2 = r_at(route.route_points, 1);
                    double angle = std::fabs( geometry::get_angle(rpPrev2, rpPrev, rp, true) );
                    speedFactor2 = minFactor + (1-minFactor) * angle / 180;
                    speedFactor2 = std::min(1.0, speedFactor2);
                }
            }

            if(!isWorking)
                rp.time_stamp = -1;
            else{
                double maxSpeed = machine.calcSpeed(rpPrev.bunker_mass);
                double calcSpeed = speedFactor * speedFactor2 * edgeSpeedCalculator.calcSpeed(rpPrev,
                                                                                              rp,
                                                                                              0,
                                                                                              machine);
                if (calcSpeed > maxSpeed)
                    calcSpeed = maxSpeed;
                if (calcSpeed < minSpeed){
                    logger().printOut(LogLevel::WARNING, __FUNCTION__, "Calculated harvester speed in track " + std::to_string(track.id)
                                       + " (index = " + std::to_string(i) + ") is lower than " + std::to_string(minSpeed) + " m/s");
                    calcSpeed = minSpeed;
                }

                rp.time_stamp = rpPrev.time_stamp + arolib::geometry::calc_dist(rpPrev, rp) / calcSpeed;


                if(useSpeedCalculatorTurningTime){
                    if(i == 0){// add the turning time corresponding to the change of tracks
                        double turningTime = 0;
                        if(i == 0 && route.route_points.size() > 2){
                            double turningTimeTmp = getTurningTime( r_at(route.route_points, 1), route.route_points.back(), points.front() );
                            if(turningTimeTmp > 1)
                                turningTime += turningTimeTmp;
                        }
                        if(i == 0 && points.size() > 2){
                            double turningTimeTmp = getTurningTime( route.route_points.back(), points.front(), points.at(1) );
                            if(turningTimeTmp > 1)
                                turningTime += turningTimeTmp;
                        }
                        if(turningTime > 1e-3)
                            rp.time_stamp += turningTime;
                    }
                }
            }

            double mass = 0;

            if(rp.type != RoutePoint::TRACK_START)
            //if(rpPrev.track_id != track.id)
                mass = getMass(rpPrev, rp, width, edgeMassCalculator, ip, isSegmentWorked);

            rp.worked_mass = rpPrev.worked_mass + mass;
            rp.worked_volume = rpPrev.worked_volume + 0.0;//@TODO
        }

        if(useSpeedCalculatorTurningTime){
            if(i > 0 && i+1 < points.size() && rp.time_stamp > -1e-9){
                double turningTime = getTurningTime( points.at(i-1), points.at(i), points.at(i+1) );
                if(turningTime > 1){
                    route.route_points.push_back(rp);
                    rp.time_stamp += turningTime;
                }
            }
        }

        route.route_points.push_back(rp);
    }

    if(plannerParameters.monitorPlannedAreas && isWorking)
        updateWorkedAreas(points, width, ip);
}

AroResp HeadlandBaseRoutesPlanner::initPartialHeadlandsParams(const Subfield &subfield, InternalParametersForPartialHLs &ip)
{
    ip.hlParams.resize(subfield.headlands.partial.size());
    for(size_t i = 0 ; i < subfield.headlands.partial.size() ; ++i){
        auto& hl = subfield.headlands.partial.at(i);
        auto& tracks = hl.tracks;
        auto& hlParams = ip.hlParams.at(i);
        hlParams.trackDirectionFlags.resize(hl.tracks.size(), false);
        hlParams.trackStates.resize(hl.tracks.size(), InternalParametersForPartialHLs::TRACK_NOT_WORKED );
        hlParams.state = InternalParametersForPartialHLs::PHL_NOT_WORKED;

        //check the tracks directions with respect to track[0]
        for(size_t j = 1 ; j < tracks.size() ; ++j){
            auto& track0 = tracks.at(j-1);
            auto& track = tracks.at(j);
            if(track0.points.empty() || track.points.empty())
                continue;

            double dP0P0 = geometry::calc_dist( track.points.front(), track0.points.front() );
            double dP0Pn = geometry::calc_dist( track.points.front(), track0.points.back() );
            double dPnP0 = geometry::calc_dist( track.points.back(), track0.points.front() );
            double dPnPn = geometry::calc_dist( track.points.back(), track0.points.back() );
            hlParams.trackDirectionFlags.at(j) = ( dP0P0 + dPnPn <= dP0Pn + dPnP0 ?
                                                             hlParams.trackDirectionFlags.at(j-1) :
                                                             !hlParams.trackDirectionFlags.at(j-1) );
        }

        //check if the tracks order is out-to-in or in-to-out
        hlParams.tracksInwards = true;
        if(tracks.size() > 1){
            double d0 = 0, dn = 0;
            for(auto& p : tracks.front().points)
                d0 += geometry::calc_dist_to_linestring(subfield.boundary_outer.points, p);
            d0 /= ( tracks.front().points.empty() ? 1 : tracks.front().points.size() );
            for(auto& p : tracks.back().points)
                dn += geometry::calc_dist_to_linestring(subfield.boundary_outer.points, p);
            d0 /= ( tracks.back().points.empty() ? 1 : tracks.back().points.size() );
            hlParams.tracksInwards = ( dn > d0 );
        }

        if(ip.gm.hasGrid(RemainingAreaMapName)){
            for(size_t j = 0 ; j < tracks.size() ; ++j){
                auto& track = tracks.at(j);
                int indWorked_fwd = -1, indUnworked_fwd = -1;
                for(size_t k = 0 ; k+1 < track.points.size() ; ++k){
                    if(isSegmentWorked(track.points.at(k), track.points.at(k+1), track.width, ip)){
                        if(indWorked_fwd < 0)
                            indWorked_fwd = k;
                    }
                    else if(indUnworked_fwd < 0)
                        indUnworked_fwd = k;
                    if(indWorked_fwd >= 0 && indUnworked_fwd >= 0)
                        break;
                }
                if(indWorked_fwd >= 0 && indUnworked_fwd >= 0){
                    hlParams.tracksToWork.push_back(j);
                    int indUnworked_rev = -1;
                    for(int k = track.points.size()-1 ; k-1 > indWorked_fwd && k-1 > indUnworked_fwd  ; --k){
                        if(!isSegmentWorked(track.points.at(k-1), track.points.at(k), track.width, ip)){
                            indUnworked_rev = k;
                            break;
                        }
                    }
                    if(indUnworked_rev < 0)
                        indUnworked_rev = indWorked_fwd;

                    if(indUnworked_fwd > 0 || indUnworked_rev < track.points.size()-1 ){
                        hlParams.trackStates.at(j) = InternalParametersForPartialHLs::TRACK_PARTIALLY_WORKED;
                        hlParams.partiallyWorkedTrackPointsInds[j] = std::make_pair(indUnworked_fwd, indUnworked_rev);
                    }
                    else
                        hlParams.trackStates.at(j) = InternalParametersForPartialHLs::TRACK_NOT_WORKED;
                }
                else if(indWorked_fwd >= 0)
                    hlParams.trackStates.at(j) = InternalParametersForPartialHLs::TRACK_WORKED;
                else{
                    hlParams.trackStates.at(j) = InternalParametersForPartialHLs::TRACK_NOT_WORKED;
                    hlParams.tracksToWork.push_back(j);
                }
            }
            if(tracks.empty())
                hlParams.state = InternalParametersForPartialHLs::PHL_WORKED;
            else if(hlParams.trackStates.front() == InternalParametersForPartialHLs::TRACK_NOT_WORKED
                     && hlParams.trackStates.back() == InternalParametersForPartialHLs::TRACK_NOT_WORKED)
                hlParams.state = InternalParametersForPartialHLs::PHL_NOT_WORKED;
            else{
                hlParams.state = InternalParametersForPartialHLs::PHL_WORKED;
                for(auto& state : hlParams.trackStates){
                    if(state != InternalParametersForPartialHLs::TRACK_WORKED){
                        hlParams.state = InternalParametersForPartialHLs::PHL_PARTIALLY_WORKED;
                        break;
                    }
                }
            }
        }
        else{//just in case
            for(size_t j = 0 ; j < tracks.size() ; ++j){
                auto& track = tracks.at(j);
                if(track.points.size() < 2)
                    hlParams.trackStates.at(j) = InternalParametersForPartialHLs::TRACK_WORKED;
                else
                    hlParams.tracksToWork.push_back(j);
            }
            if(tracks.empty())
                hlParams.state = InternalParametersForPartialHLs::PHL_WORKED;
        }
    }
    return AroResp::ok();

}

AroResp HeadlandBaseRoutesPlanner::presortPartialHeadlands(const Subfield &subfield, InternalParametersForPartialHLs &ip)
{
    ip.sortedPartialHLs.clear();
    auto& hls = subfield.headlands.partial;
    int indFirstMainHL = -1;
    //get the index of one main headland
    for(size_t i = 0 ; i < hls.size() ; ++i){
        if(!hls.at(i).isConnectingHeadland(true)){
            indFirstMainHL = i;
            break;
        }
    }
    if(indFirstMainHL < 0)
        return AroResp::LoggingResp(1, "No main headlands found", m_logger, LogLevel::ERROR, __FUNCTION__);

    std::set<size_t> visitedHLs = {(size_t)indFirstMainHL};
    ip.sortedPartialHLs.push_back(indFirstMainHL);
    bool currentHLisMain = true;

    while(visitedHLs.size() != hls.size()){
        bool found = false;
        for(size_t i = 1 ; i < hls.size() ; ++i){
            size_t ind = get_index_from_cyclic_container(hls, ip.sortedPartialHLs.back()+i);

            if(visitedHLs.find(ind) != visitedHLs.end())
                continue;

            auto& hl = hls.at(ind);

            if(currentHLisMain != hl.isConnectingHeadland(true))
                continue;

            if(currentHLisMain){
                if( hl.connectingHeadlandIds.first != hls.at( ip.sortedPartialHLs.back() ).id
                        && hl.connectingHeadlandIds.second != hls.at( ip.sortedPartialHLs.back() ).id )
                    continue;
            }
            else{
                auto& hlPrev = hls.at( ip.sortedPartialHLs.back() );
                if( hlPrev.connectingHeadlandIds.first != hl.id
                        && hlPrev.connectingHeadlandIds.second != hl.id )
                    continue;
            }

            found = true;
            visitedHLs.insert(ind);
            ip.sortedPartialHLs.push_back(ind);
            currentHLisMain = !currentHLisMain;
            break;
        }
        if(!found)
            break;
    }
    if(visitedHLs.size() != hls.size())
        return AroResp::LoggingResp(1, "Not possible to cover all headlands with the given connetions", m_logger, LogLevel::ERROR, __FUNCTION__);

    return AroResp::ok();
}

AroResp HeadlandBaseRoutesPlanner::getStartingParameters_partialHLs(const Subfield &subfield,
                                                                    const Machine &machine,
                                                                    const PlannerParameters &plannerParameters,
                                                                    const Pose2D *pInitRefPose,
                                                                    const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                                                    const OutFieldInfo *outFieldInfo,
                                                                    InternalParametersForPartialHLs &ip)
{
    ip.indFirstHeadland = ip.indFirstWorkingHeadland = -1;

    auto aroResp = getFirstWorkingHeadlandParameters(subfield,
                                                      machine,
                                                      plannerParameters,
                                                      pInitRefPose,
                                                      machineCurrentStates,
                                                      outFieldInfo,
                                                      ip);
    if(aroResp.isError())
        return AroResp::LoggingResp(1, "Error obtaining first working headland", ": " + aroResp.msg, m_logger, LogLevel::ERROR, __FUNCTION__);
    if(ip.indFirstWorkingHeadland < 0)
        return AroResp(-1, "Could not obtaining first working headland. Probably all headlands were considered to be worked");

    aroResp = getFirstHeadlandAndSortHeadlands(subfield,ip);
    if(aroResp.isError())
        return AroResp::LoggingResp(1, "Error sorting headlands based on first working headland", ": " + aroResp.msg, m_logger, LogLevel::ERROR, __FUNCTION__);

    return AroResp::ok();
}


AroResp HeadlandBaseRoutesPlanner::getFirstWorkingHeadlandParameters(const Subfield &subfield,
                                                                     const Machine &machine,
                                                                     const PlannerParameters & plannerParameters,
                                                                     const Pose2D * pInitRefPose,
                                                                     const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                                                     const OutFieldInfo* outFieldInfo,
                                                                     InternalParametersForPartialHLs &ip)
{
    ip.potentialFirstWorkingHeadlands = getIndexesOfPartiallyWorkedHeadlands(ip);
    std::set<size_t> indHLsToWork;
    for(auto ind : ip.sortedPartialHLs){
        if(ip.hlParams.at(ind).state != InternalParametersForPartialHLs::PHL_WORKED)
            indHLsToWork.insert(ind);
    }
    if(ip.potentialFirstWorkingHeadlands.empty()){//no headlands are considered partially worked
        if(indHLsToWork.size() == ip.sortedPartialHLs.size())// all headlands need to be workd -> all are potentialFirstWorkingHeadlands
            ip.potentialFirstWorkingHeadlands = ip.sortedPartialHLs;
        else{
            for(auto& ind : ip.sortedPartialHLs){
                if(ip.hlParams.at(ind).state == InternalParametersForPartialHLs::PHL_NOT_WORKED
                        && (isHeadlandWorked(ind, -1, ip) || isHeadlandWorked(ind, 1, ip)) ) //add only the HLS that have to be worked and have at least one worked adjacent headland
                    ip.potentialFirstWorkingHeadlands.push_back(ind);
            }
        }
    }

    if(ip.potentialFirstWorkingHeadlands.empty())
        return AroResp(-1, "Unable to obtain potential first working headlands; probably all headlands are considered to be completelly worked");

    if(ip.potentialFirstWorkingHeadlands.size() != ip.sortedPartialHLs.size()){//check if the order of worked headlands is supported
        bool isPrevWorked = false, isNextWorked = false;
        if(indHLsToWork.size() > 1){
            for(auto ind : indHLsToWork){
                if(isHeadlandWorked(ind, -1, ip)){
                    if(isPrevWorked)
                        return AroResp::LoggingResp(1, "The headlands were not worked in a supported order.", m_logger, LogLevel::ERROR, __FUNCTION__);
                    isPrevWorked = true;
                }
                if(isHeadlandWorked(ind, 1, ip)){
                    if(isNextWorked)
                        return AroResp::LoggingResp(1, "The headlands were not worked in a supported order.", m_logger, LogLevel::ERROR, __FUNCTION__);
                    isNextWorked = true;
                }
            }
        }
    }

    if(machineCurrentStates){
        auto aroResp = getFirstWorkingHeadlandParameters_machineState(subfield,
                                                                      machine,
                                                                      plannerParameters,
                                                                      pInitRefPose,
                                                                      *machineCurrentStates,
                                                                      ip);
        if(aroResp.isError())
            return aroResp;
    }

    if(ip.indFirstWorkingHeadland < 0 && pInitRefPose){//try next with the refPoint
        auto aroResp = getFirstWorkingHeadlandParameters_refPose(subfield,
                                                                 machine,
                                                                 plannerParameters,
                                                                 *pInitRefPose,
                                                                 ip);
        if(aroResp.isError())
            return aroResp;
    }

    if(ip.indFirstWorkingHeadland < 0){//try next with the access points and outfield info
        auto aroResp = getFirstWorkingHeadlandParameters_outFieldInfo(subfield,
                                                                      machine,
                                                                      plannerParameters,
                                                                      machineCurrentStates,
                                                                      ( outFieldInfo ? *outFieldInfo : OutFieldInfo() ),
                                                                      ip);
        if(aroResp.isError())
            return aroResp;
    }

    if(ip.indFirstWorkingHeadland < 0){//just select the 1st valid one
        for(int hlType = 0 ; hlType < 2 && ip.indFirstWorkingHeadland<0 ; ++hlType){//check first main headlands
            for(auto& ind : ip.potentialFirstWorkingHeadlands){
                auto& hl = subfield.headlands.partial.at(ind);
                if( hlType==0 && hl.isConnectingHeadland() )
                    continue;
                else if( hlType==1 && !hl.isConnectingHeadland() )
                    continue;

                int indFirstTrack = getFirstPartialHeadlandWorkingTrackInd(subfield, ind, ip, plannerParameters.startFromOutermostTrack, false);
                int indLastTrack = getLastPartialHeadlandWorkingTrackInd(ind, ip, plannerParameters.startFromOutermostTrack);
                if(indFirstTrack < 0 || indLastTrack < 0)
                    continue;
                ip.indFirstWorkingHeadland = ind;
                auto& hlParams = ip.hlParams.at(ip.indFirstWorkingHeadland);
                auto& lastTrack = hl.tracks.at(indLastTrack);
                bool finishesInSameSide = ( (hlParams.tracksToWork.size()%2) != 0 );
                size_t lastTrackPtInd = ( finishesInSameSide ? 0 : lastTrack.points.size()-1 );
                if(hlParams.trackDirectionFlags.at(indFirstTrack) != hlParams.trackDirectionFlags.at(indLastTrack))
                    lastTrackPtInd = lastTrack.points.size()-1-lastTrackPtInd;
                const Point& lastTrackPt = lastTrack.points.at(lastTrackPtInd);
                size_t indHLNext = getNextHLIndexClosestToPoint(subfield, ip.indFirstWorkingHeadland, lastTrackPt, ip);


                updateHLParamsForFirstWorkingHeadland(subfield, ip.indFirstWorkingHeadland, ip,
                                                      hlParams,
                                                      indFirstTrack,
                                                      indFirstTrack,
                                                      indFirstTrack,
                                                      indLastTrack,
                                                      false,
                                                      false,
                                                      0,
                                                      false,
                                                      indHLNext);

                break;
            }
        }
    }

    if(ip.indFirstWorkingHeadland >= 0)
        ip.hlParams.at(ip.indFirstWorkingHeadland).startFromOutermostTrack = plannerParameters.startFromOutermostTrack;

    return AroResp::ok();
}

AroResp HeadlandBaseRoutesPlanner::getFirstWorkingHeadlandParameters_machineState(const Subfield &subfield,
                                                                                  const Machine &machine, const PlannerParameters &plannerParameters,
                                                                                  const Pose2D *pInitRefPose,
                                                                                  const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                                                  InternalParametersForPartialHLs &ip)
{
    int indHLMachineInside = -1;

    auto isValid = [&ip, &subfield, pInitRefPose, &indHLMachineInside]( const Point& ptMachine, size_t indHL, double distHLfromMachineTH,
                                                                        double distPtFromMachine, double extraDist, double distPtFromMachineTH,
                                                                        bool hlNextIsWorked, int indHLPrev,
                                                                        double & minTotalDist ) -> bool{ // minTotalDist is updated if valid
        auto& hl = subfield.headlands.partial.at(indHL);
        bool insideHL = false, insideHLPrev = false;

        if(indHLMachineInside >= 0){
            insideHL = ( indHLMachineInside == indHL );
            insideHLPrev = ( indHLMachineInside == indHLPrev );
        }
        else{
            if(geometry::in_polygon(ptMachine, hl.boundary)){
                insideHL = true;
                indHLMachineInside = indHL;
            }
            else if(indHLPrev >= 0 && isHeadlandWorked(indHLPrev, 0, ip)){
                auto& hlPrev = subfield.headlands.partial.at(indHLPrev);
                if(geometry::in_polygon(ptMachine, hlPrev.boundary)){
                    insideHLPrev = true;
                    indHLMachineInside = indHLPrev;
                }
            }
        }


        if( !insideHL && !insideHLPrev ){
            if(pInitRefPose)//if a pInitRefPose was given, give priority to it
                return false;
            double distToHL = geometry::calc_dist_to_linestring(hl.boundary.points, ptMachine, false);
            if(distToHL > distHLfromMachineTH)//only valid if really close to headland
                return false;
        }
        double totalDist = distPtFromMachine + extraDist;
        if(minTotalDist < totalDist)
            return false;

//        if(insideHL){
//            if(distPtFromMachine < distPtFromMachineTH){
//                if(ip.potentialFirstWorkingHeadlands.size() == 1//only one HL left to work;
//                        || !hlNextIsWorked){
//                    minTotalDist = totalDist;
//                    return true;
//                }
//            }
//        }
//        return false;

        minTotalDist = totalDist;
        return true;

    };

    ip.indFirstWorkingHeadland = -1;
    InternalParametersForPartialHLs::HLParamsBase hlParams;

    auto it_m = machineCurrentStates.find(machine.id);
    if(it_m == machineCurrentStates.end())
        return AroResp::ok();
    auto& ptMachine = it_m->second.position;

    double minTotalDist = std::numeric_limits<double>::max();
    double distPtFromMachineTH = machine.getTurningRadius();
    double distHLfromMachineTH = 0.5 * ( machine.width > 1e-6 ? machine.width : machine.working_width );

    auto aroResp = checkPotentialFirstWorkingHLsOptions(subfield, plannerParameters, plannerParameters.startFromOutermostTrack, ip,
                                                        [&]
                                                        (InternalParametersForPartialHLs::FirstWorkingHeadlandOptionParams &op) -> void{

        auto& hl = subfield.headlands.partial.at(op.indHL);
        auto& firstTrack = hl.tracks.at(op.indFirstTrack);
        auto& lastTrack = hl.tracks.at(op.indLastTrack);
        auto& firstTrackToWork = hl.tracks.at(op.indFirstTrackToWork);

        double distPtFromMachine = geometry::calc_dist( firstTrackToWork.points.at(op.indFirstPointToWork), ptMachine );
        double extraDist = ( op.withStartTransitTrack ? geometry::getGeometryLength(firstTrack.points) : 0 );
        extraDist += ( op.withEndTransitTrack ? geometry::getGeometryLength(lastTrack.points) : 0 );

        if( isValid(ptMachine, op.indHL, distHLfromMachineTH,
                    distPtFromMachine, extraDist, distPtFromMachineTH,
                    op.hlNextIsWorked, op.indHLPrev,
                    minTotalDist) ){
            ip.indFirstWorkingHeadland = op.indHL;
            updateHLParamsForFirstWorkingHeadland(subfield, op.indHL, ip,
                                                  hlParams,
                                                  op.indFirstWorkingTrack,
                                                  op.indFirstTrackToWork,
                                                  op.indFirstTrack,
                                                  op.indLastTrack,
                                                  op.withStartTransitTrack,
                                                  op.withEndTransitTrack,
                                                  op.indFirstPointToWork,
                                                  op.firstTrackToWorkInReverse,
                                                  op.indHLNext);
        }
    });

    if(aroResp.isError())
        return aroResp;

    if(ip.indFirstWorkingHeadland >= 0)
        ip.hlParams.at( ip.indFirstWorkingHeadland ).update(hlParams);

    return AroResp::ok();
}

AroResp HeadlandBaseRoutesPlanner::getFirstWorkingHeadlandParameters_refPose(const Subfield &subfield,
                                                                             const Machine &machine,
                                                                             const PlannerParameters &plannerParameters,
                                                                             const Pose2D &initRefPose,
                                                                             InternalParametersForPartialHLs &ip)
{

    // @note: since this is called when no headland is partially worked, we only check track extrema

    auto isValid = [&ip](bool hlNextIsWorked) -> bool{
        return ip.potentialFirstWorkingHeadlands.size() == 1 //only one HL left to work
                || !hlNextIsWorked;
    };

    ip.indFirstWorkingHeadland = -1;
    InternalParametersForPartialHLs::HLParamsBase hlParams;

    auto& hls = subfield.headlands.partial;

    double rad = machine.getTurningRadius();

    double minDist = std::numeric_limits<double>::max();
    double minDistMain = std::numeric_limits<double>::max();
    int indMain = -1;
    auto hlParamsMain = hlParams;

    geometry::DubinsParams dp;
    dp.p1 = initRefPose.point();
    dp.rho1 = initRefPose.angle;


    auto aroResp = checkPotentialFirstWorkingHLsOptions(subfield, plannerParameters, plannerParameters.startFromOutermostTrack, ip,
                                                        [&](InternalParametersForPartialHLs::FirstWorkingHeadlandOptionParams& op) -> void{

        if(!isValid(op.hlNextIsWorked))
            return;

        auto& hl = subfield.headlands.partial.at(op.indHL);
        auto& firstTrack = hl.tracks.at(op.indFirstTrack);
        auto& firstTrackPt = firstTrack.points.at(op.indFirstTrackPt);

        double dist;
        if(rad > 1e-6){
            auto& p1 = ( op.side == 0 ? firstTrack.points.at(1) : r_at(firstTrack.points, 1) );
            dp.p2 = firstTrackPt;
            dp.rho2 = geometry::get_angle(firstTrackPt, p1, false);
            dist = geometry::calcDubinsPathLength(dp, rad);
        }
        else
            dist = geometry::calc_dist(initRefPose, firstTrackPt);

        if(dist < 1e-6)
            return;

        if(minDist > dist){
            minDist = dist;
            ip.indFirstWorkingHeadland = op.indHL;
            updateHLParamsForFirstWorkingHeadland(subfield, op.indHL, ip,
                                                  hlParams,
                                                  op.indFirstWorkingTrack,
                                                  op.indFirstTrackToWork,
                                                  op.indFirstTrack,
                                                  op.indLastTrack,
                                                  op.withStartTransitTrack,
                                                  op.withEndTransitTrack,
                                                  op.indFirstPointToWork,
                                                  op.firstTrackToWorkInReverse,
                                                  op.indHLNext);
        }

        if(minDistMain > dist && !hls.at(op.indHL).isConnectingHeadland()){
            minDistMain = dist;
            indMain = op.indHL;
            updateHLParamsForFirstWorkingHeadland(subfield, op.indHL, ip,
                                                  hlParamsMain,
                                                  op.indFirstWorkingTrack,
                                                  op.indFirstTrackToWork,
                                                  op.indFirstTrack,
                                                  op.indLastTrack,
                                                  op.withStartTransitTrack,
                                                  op.withEndTransitTrack,
                                                  op.indFirstPointToWork,
                                                  op.firstTrackToWorkInReverse,
                                                  op.indHLNext);
        }
    });

    if(aroResp.isError())
        return aroResp;


    bool givePriorityMainHeadlands = true;
    for(auto& hlParams : ip.hlParams){//give priority to main headlands only if no headlands have been worked
        if(hlParams.state == InternalParametersForPartialHLs::PHL_WORKED){
            givePriorityMainHeadlands = false;
            break;
        }
    }

    if(indMain >= 0 && (givePriorityMainHeadlands || minDistMain < rad) ){//prefer non connecting headlands
        ip.indFirstWorkingHeadland = indMain;
        ip.hlParams.at( ip.indFirstWorkingHeadland ).update(hlParamsMain);
    }
    else if(ip.indFirstWorkingHeadland >= 0)
        ip.hlParams.at( ip.indFirstWorkingHeadland ).update(hlParams);

    return AroResp::ok();

}

AroResp HeadlandBaseRoutesPlanner::getFirstWorkingHeadlandParameters_outFieldInfo(const Subfield &subfield,
                                                                                  const Machine &machine,
                                                                                  const PlannerParameters & plannerParameters,
                                                                                  const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                                                                  const OutFieldInfo& outFieldInfo,
                                                                                  InternalParametersForPartialHLs &ip)
{
    auto isValid = [&ip](bool hlNextIsWorked) -> bool{
        return ip.potentialFirstWorkingHeadlands.size() == 1 //only one HL left to work
                || !hlNextIsWorked;
    };
    auto isValid_main_close = [&machine, &plannerParameters](const FieldAccessPoint & fap, const Point & trackPt, const Track & track) -> bool{
        double dist = geometry::calc_dist(fap, trackPt);
        return dist < 2*machine.getTurningRadius()
                || dist < 2*track.width;
    };

    ip.indFirstWorkingHeadland = -1;
    InternalParametersForPartialHLs::HLParamsBase hlParams;

    double minTimeWithTC_main = std::numeric_limits<double>::max();
    double minTimeWithoutTC_main = std::numeric_limits<double>::max();
    double minTimeWithTC = std::numeric_limits<double>::max();
    double minTimeWithoutTC = std::numeric_limits<double>::max();
    int indWithTC = -1, indWithoutTC = -1;
    int indWithTC_main = -1, indWithoutTC_main = -1;
    InternalParametersForPartialHLs::HLParamsBase hlParamsWithTC, hlParamsWithoutTC, hlParamsWithTC_main, hlParamsWithoutTC_main;
    double bunkerMass = 0;
    OutFieldInfo::MachineBunkerState bunkerState1 = OutFieldInfo::MACHINE_EMPTY;
    OutFieldInfo::MachineBunkerState bunkerState2 = OutFieldInfo::MACHINE_LOADED;
    if(machineCurrentStates){
        auto it_mdi = machineCurrentStates->find(machine.id);
        if(it_mdi != machineCurrentStates->end()){
            const MachineDynamicInfo& mdi = it_mdi->second;
            bunkerMass = mdi.bunkerMass;
            if(mdi.bunkerMass > machine.bunker_mass * 0.5)
                std::swap(bunkerState1, bunkerState2);
        }
    }

    double speed = (machine.def_working_speed > 1e-5 ? machine.def_working_speed : machine.calcSpeed(0.5 * machine.bunker_mass));
    speed = (speed > 1e-5 ? speed : 0.5);//default speed if no info is given in the machine (shouldn't happen)

    double speed_transit = getTransitSpeed(machine, bunkerMass);

    auto boundary = subfield.boundary_outer.points;
    for(auto& fap : subfield.access_points)
        geometry::addSampleToGeometryClosestToPoint(boundary, fap);


    auto aroResp = checkPotentialFirstWorkingHLsOptions(subfield, plannerParameters, plannerParameters.startFromOutermostTrack, ip,
                                                        [&](InternalParametersForPartialHLs::FirstWorkingHeadlandOptionParams& op) -> void{

        if( !isValid(op.hlNextIsWorked) )
            return;

        for(size_t i = 0; i < subfield.access_points.size() ; ++i){
            auto& fap = subfield.access_points.at(i);
            auto& hl = subfield.headlands.partial.at(op.indHL);
            auto& firstTrack = hl.tracks.at(op.indFirstTrack);
            auto& lastTrack = hl.tracks.at(op.indLastTrack);
            auto& firstTrackPt = firstTrack.points.at(op.indFirstTrackPt);

            double distFromFap = geometry::getGeometryLength( geometry::getShortestGeometryPart( boundary, firstTrackPt, fap, true ) );
            double penaltyDist = 0;
            if(op.withStartTransitTrack){
                penaltyDist += geometry::getGeometryLength(firstTrack.points);
            }
            if(op.withEndTransitTrack){
                penaltyDist += geometry::getGeometryLength(lastTrack.points);
            }
            double time_transit_IF =  (distFromFap+penaltyDist)/speed_transit;

            OutFieldInfo::TravelCosts tc;
            if( outFieldInfo.getArrivalCost(fap.id, machine.id, bunkerState1, tc) ||
                    outFieldInfo.getArrivalCost(fap.id, machine.id, bunkerState2, tc) ){
                double time = time_transit_IF + tc.time;
                if(minTimeWithTC > time){
                    minTimeWithTC = time;
                    indWithTC = op.indHL;
                    updateHLParamsForFirstWorkingHeadland(subfield, op.indHL, ip,
                                                          hlParamsWithTC,
                                                          op.indFirstWorkingTrack,
                                                          op.indFirstTrackToWork,
                                                          op.indFirstTrack,
                                                          op.indLastTrack,
                                                          op.withStartTransitTrack,
                                                          op.withEndTransitTrack,
                                                          op.indFirstPointToWork,
                                                          op.firstTrackToWorkInReverse,
                                                          op.indHLNext);
                }
                if( !hl.isConnectingHeadland()
                        && minTimeWithTC_main > time
                        && isValid_main_close(fap, firstTrackPt, firstTrack) ){
                    minTimeWithTC_main = time;
                    indWithTC_main = op.indHL;
                    updateHLParamsForFirstWorkingHeadland(subfield, op.indHL, ip,
                                                          hlParamsWithTC_main,
                                                          op.indFirstWorkingTrack,
                                                          op.indFirstTrackToWork,
                                                          op.indFirstTrack,
                                                          op.indLastTrack,
                                                          op.withStartTransitTrack,
                                                          op.withEndTransitTrack,
                                                          op.indFirstPointToWork,
                                                          op.firstTrackToWorkInReverse,
                                                          op.indHLNext);
                }
                continue;
            }
            else{
                if(minTimeWithoutTC > time_transit_IF){
                    minTimeWithoutTC = time_transit_IF;
                    indWithoutTC = op.indHL;
                    updateHLParamsForFirstWorkingHeadland(subfield, op.indHL, ip,
                                                          hlParamsWithoutTC,
                                                          op.indFirstWorkingTrack,
                                                          op.indFirstTrackToWork,
                                                          op.indFirstTrack,
                                                          op.indLastTrack,
                                                          op.withStartTransitTrack,
                                                          op.withEndTransitTrack,
                                                          op.indFirstPointToWork,
                                                          op.firstTrackToWorkInReverse,
                                                          op.indHLNext);
                }
                if( !hl.isConnectingHeadland()
                        && minTimeWithoutTC_main > time_transit_IF
                        && isValid_main_close(fap, firstTrackPt, firstTrack) ){
                    minTimeWithoutTC_main = time_transit_IF;
                    indWithoutTC_main = op.indHL;
                    updateHLParamsForFirstWorkingHeadland(subfield, op.indHL, ip,
                                                          hlParamsWithoutTC_main,
                                                          op.indFirstWorkingTrack,
                                                          op.indFirstTrackToWork,
                                                          op.indFirstTrack,
                                                          op.indLastTrack,
                                                          op.withStartTransitTrack,
                                                          op.withEndTransitTrack,
                                                          op.indFirstPointToWork,
                                                          op.firstTrackToWorkInReverse,
                                                          op.indHLNext);
                }
            }
        }
    });



    if(aroResp.isError())
        return aroResp;

    bool givePriorityMainHeadlands = true;
    for(auto& hlParams : ip.hlParams){//give priority to main headlands only if no headlands have been worked
        if(hlParams.state == InternalParametersForPartialHLs::PHL_WORKED){
            givePriorityMainHeadlands = false;
            break;
        }
    }

    if(givePriorityMainHeadlands && indWithTC_main >= 0){//give priority to main headlands and access points where arrival costs were given
        ip.indFirstWorkingHeadland = indWithTC_main;
        hlParams = hlParamsWithTC_main;
    }
    else if(indWithTC >= 0 ){//give priority to access points where arrival costs were given
        ip.indFirstWorkingHeadland = indWithTC;
        hlParams = hlParamsWithTC;
    }
    else if(givePriorityMainHeadlands && indWithoutTC_main >= 0){//give priority to main headlands
        ip.indFirstWorkingHeadland = indWithoutTC_main;
        hlParams = hlParamsWithoutTC_main;
    }
    else if(indWithoutTC >= 0 ){
        ip.indFirstWorkingHeadland = indWithoutTC;
        hlParams = hlParamsWithoutTC;
    }

    if(ip.indFirstWorkingHeadland >= 0)
        ip.hlParams.at( ip.indFirstWorkingHeadland ).update(hlParams);
    return AroResp::ok();

}

AroResp HeadlandBaseRoutesPlanner::checkPotentialFirstWorkingHLsOptions(const Subfield &subfield,
                                                                        const PlannerParameters &plannerParameters,
                                                                        bool startHLFromOutermostTrack,
                                                                        InternalParametersForPartialHLs &ip,
                                                                        const std::function<void (InternalParametersForPartialHLs::FirstWorkingHeadlandOptionParams &)> &cb)
{
    InternalParametersForPartialHLs::FirstWorkingHeadlandOptionParams op;

    for(auto& indHL : ip.potentialFirstWorkingHeadlands){
        auto& hl = subfield.headlands.partial.at(indHL);
        auto& hlParams = ip.hlParams.at(indHL);
        op.indHL = indHL;
        op.indFirstTrackToWork = getFirstPartialHeadlandWorkingTrackInd(subfield, indHL, ip, startHLFromOutermostTrack, true);
        op.indFirstWorkingTrack = ( plannerParameters.removeInitialWorkedSegments ? op.indFirstTrackToWork :
                                                                                    getFirstPartialHeadlandWorkingTrackInd(subfield, indHL, ip, startHLFromOutermostTrack, false) );
        op.indLastWorkingTrack = getLastPartialHeadlandWorkingTrackInd(indHL, ip, startHLFromOutermostTrack);

        for(size_t withStartTransitTrack = 0 ; withStartTransitTrack < 2 ; ++withStartTransitTrack){
            op.withStartTransitTrack = withStartTransitTrack;
            if( op.withStartTransitTrack
                    && ( hl.tracks.size()<2
                         || ip.workedAreaTransitRestriction == WorkedAreaTransitRestriction::TRANSIT_ONLY_OVER_WORKED_AREA //cannot have first transit
                         || ip.workedAreaTransitRestriction != WorkedAreaTransitRestriction::TRANSIT_ONLY_OVER_UNWORKED_AREA //no need for starting transit track
                         || ip.potentialFirstWorkingHeadlands.size() == 1 ) )
                    continue;

            if(plannerParameters.removeInitialWorkedSegments)
                op.indFirstTrack = ( op.withStartTransitTrack ? getPartialHeadlandTransitTrackInd(subfield, ip, indHL, op.indFirstTrackToWork) : op.indFirstWorkingTrack );
            else
                op.indFirstTrack = ( op.withStartTransitTrack ? getPartialHeadlandTransitTrackInd(subfield, ip, indHL, op.indFirstWorkingTrack) : op.indFirstWorkingTrack );

            auto& firstTrack = hl.tracks.at(op.indFirstTrack);
            if(firstTrack.points.empty())
                return AroResp::LoggingResp(1, "Headland at index " + std::to_string(indHL) + " has an invalid track at index " + std::to_string(op.indFirstTrack), m_logger, LogLevel::ERROR, __FUNCTION__);

            for(size_t withEndTransitTrack = 0 ; withEndTransitTrack < 2 ; ++withEndTransitTrack){
                op.withEndTransitTrack = withEndTransitTrack;
                if( op.withEndTransitTrack
                        && (op.withStartTransitTrack//only allow one transit track
                            || hl.tracks.size()<2
                            || ip.workedAreaTransitRestriction == WorkedAreaTransitRestriction::TRANSIT_ONLY_OVER_UNWORKED_AREA //cannot have last transit track
                            || ip.potentialFirstWorkingHeadlands.size() == 1) )
                    continue;

                op.indLastTrack = ( op.withEndTransitTrack ? getPartialHeadlandTransitTrackInd(subfield, ip, indHL, op.indLastWorkingTrack) : op.indLastWorkingTrack );
                auto& lastTrack = hl.tracks.at(op.indLastTrack);
                if(lastTrack.points.empty())
                    return AroResp::LoggingResp(1, "Headland at index " + std::to_string(indHL) + " has an invalid track at index " + std::to_string(op.indLastTrack), m_logger, LogLevel::ERROR, __FUNCTION__);

                if(plannerParameters.removeInitialWorkedSegments)
                    op.finishesInSameSide = ( ( (hlParams.tracksToWork.size() + op.withStartTransitTrack + op.withEndTransitTrack) %2) == 0 );
                else
                    op.finishesInSameSide = ( ( (hl.tracks.size() + op.withStartTransitTrack + op.withEndTransitTrack) %2) == 0 );

                for(op.side = 0 ; op.side < 2 ; ++op.side){

                    op.indFirstTrackPt = ( op.side == 0 ? 0 : firstTrack.points.size()-1 );

                    auto& firstTrackToWork = hl.tracks.at(op.indFirstTrackToWork);
                    std::pair<size_t, size_t> ptIndexes = std::make_pair(0, firstTrackToWork.points.size()-1);
                    auto it_tp = hlParams.partiallyWorkedTrackPointsInds.find(op.indFirstTrackToWork);
                    if(it_tp != hlParams.partiallyWorkedTrackPointsInds.end())
                        ptIndexes = it_tp->second;

                    size_t countTracksBeforeStartWorkingTrack = op.withStartTransitTrack;
                    if(!plannerParameters.removeInitialWorkedSegments)
                        countTracksBeforeStartWorkingTrack += std::fabs(op.indFirstWorkingTrack - op.indFirstTrackToWork);
                    op.firstTrackToWorkInReverse = ( countTracksBeforeStartWorkingTrack%2 == op.side ?
                                                         hlParams.trackDirectionFlags.at(op.indFirstTrack) != hlParams.trackDirectionFlags.at(op.indFirstTrackToWork):
                                                         hlParams.trackDirectionFlags.at(op.indFirstTrack) == hlParams.trackDirectionFlags.at(op.indFirstTrackToWork) );
                    if(op.indFirstTrackToWork == op.indFirstWorkingTrack)
                        op.firstWorkingTrackInReverse = op.firstTrackToWorkInReverse;
                    else
                        op.firstWorkingTrackInReverse = ( withStartTransitTrack == op.side ?
                                                             hlParams.trackDirectionFlags.at(op.indFirstTrack) != hlParams.trackDirectionFlags.at(op.indFirstWorkingTrack):
                                                             hlParams.trackDirectionFlags.at(op.indFirstTrack) == hlParams.trackDirectionFlags.at(op.indFirstWorkingTrack) );

                    op.indFirstPointToWork = ( !op.firstTrackToWorkInReverse ? ptIndexes.first : ptIndexes.second );


                    op.indLastTrackPt = ( op.side == 0 ? 0 : lastTrack.points.size()-1 );
                    if(!op.finishesInSameSide)
                        op.indLastTrackPt = lastTrack.points.size()-1-op.indLastTrackPt;
                    if( hlParams.trackDirectionFlags.at(op.indFirstWorkingTrack) != hlParams.trackDirectionFlags.at(op.indLastTrack) )
                        op.indLastTrackPt = lastTrack.points.size()-1-op.indLastTrackPt;
                    auto& lastTrackPt = lastTrack.points.at(op.indLastTrackPt);

                    op.indHLNext = getNextHLIndexClosestToPoint(subfield, indHL, lastTrackPt, ip);
                    op.hlNextIsWorked = isHeadlandWorked(op.indHLNext, 0, ip);

                    op.indHLPrev = getNextHLIndexClosestToPoint(subfield, indHL, firstTrack.points.at(op.indFirstTrackPt), ip);

                    if(ip.sortedPartialHLs.size() > 2 && op.hlNextIsWorked){//disregard if the next headland is worked and there is another potential next headland that is not worked
                        int indAdjacentHL = getHeadlandIndexFromSortedList(indHL, 1, ip);
                        if(indAdjacentHL < 0 || indAdjacentHL == op.indHLNext){
                            indAdjacentHL = getHeadlandIndexFromSortedList(indHL, -1, ip);
                            if(indAdjacentHL < 0 || indAdjacentHL == op.indHLNext)
                                indAdjacentHL = -1;
                        }

                        if(indAdjacentHL > 0
                                && indAdjacentHL != op.indHLNext
                                && op.hlNextIsWorked
                                && !isHeadlandWorked(indAdjacentHL, 0, ip))
                            continue;
                    }

                    cb(op);
                }

            }
        }
    }
    return AroResp::ok();

}

void HeadlandBaseRoutesPlanner::updateHLParamsForFirstWorkingHeadland(const Subfield &subfield,
                                                                      size_t indHL,
                                                                      const InternalParametersForPartialHLs &ip,
                                                                      InternalParametersForPartialHLs::HLParamsBase &hlParams,
                                                                      size_t firstWorkingTrackInd,
                                                                      size_t firstTrackToWorkInd,
                                                                      size_t firstTrackInd,
                                                                      size_t lastTrackInd,
                                                                      bool withStartTransitTrack,
                                                                      bool withEndTransitTrack,
                                                                      size_t indFirstWorkingTrackPointToWork,
                                                                      bool firstTrackToWorkInReverse,
                                                                      size_t indHLNext)
{
    auto& hl = subfield.headlands.partial.at(indHL);
    auto& firstWorkingTrack = hl.tracks.at(firstWorkingTrackInd);

    hlParams.indFirstTrack = firstWorkingTrackInd;
    hlParams.indFirstWorkingTrack = firstTrackToWorkInd;
    hlParams.firstWorkingTrackInReverse = firstTrackToWorkInReverse;
    hlParams.indFirstWorkingTrackPoint = ( !hlParams.firstWorkingTrackInReverse ? 0 : firstWorkingTrack.points.size()-1 );
    hlParams.indFirstWorkingTrackPointToWork = indFirstWorkingTrackPointToWork;
    hlParams.nextWorkingHL = ( isHeadlandWorked(indHLNext, 0, ip) ? -1 : indHLNext);
    hlParams.indTransitTrackStart = ( withStartTransitTrack ? firstTrackInd : -1 );
    hlParams.indTransitTrackFinish = ( withEndTransitTrack ? lastTrackInd : -1 );

}

AroResp HeadlandBaseRoutesPlanner::updateHLParamsForNonFirstWorkingHeadlands(const Subfield &subfield,
                                                                             const PlannerParameters &plannerParameters,
                                                                             InternalParametersForPartialHLs &ip)
{
    int indSortedList = getSortedListIndexForHeadland(ip.indFirstWorkingHeadland, ip);
    if(indSortedList < 0)
        return AroResp::LoggingResp(1, "Invalid index for first working headland (not found in sorted list)", m_logger, LogLevel::ERROR, __FUNCTION__);

    for(int i = indSortedList-1 ; i >= 0 ; --i){
        auto& hl = subfield.headlands.partial.at( ip.sortedPartialHLs.at(i) );
        auto& hlRef = subfield.headlands.partial.at( ip.sortedPartialHLs.at(i+1) );
        auto& hlParams = ip.hlParams.at( ip.sortedPartialHLs.at(i) );
        auto& hlParamsRef = ip.hlParams.at( ip.sortedPartialHLs.at(i+1) );
        auto& trackDirectionFlagsRef = hlParamsRef.trackDirectionFlags;

        if(i == 0)
            hlParams.startFromOutermostTrack = plannerParameters.startFromOutermostTrack;
        else if (ip.workedAreaTransitRestriction == WorkedAreaTransitRestriction::NO_RESTRICTION){
            //hlParams.startFromOutermostTrack = !plannerParameters.finishWithOutermostTrack;
            hlParams.startFromOutermostTrack = true;
        }
        else{
            //hlParams.startFromOutermostTrack = (ip.workedAreaTransitRestriction == WorkedAreaTransitRestriction::TRANSIT_ONLY_OVER_UNWORKED_AREA);
            hlParams.startFromOutermostTrack = true;
        }

        hlParams.indFirstTrack = ( hlParams.startFromOutermostTrack ? 0 : hl.tracks.size()-1);
        if(!hlParams.tracksInwards)
            hlParams.indFirstTrack = hl.tracks.size()-1-hlParams.indFirstTrack;

        hlParams.indFirstWorkingTrack = hlParams.indFirstTrack;
        auto& firstTrack = hl.tracks.at( hlParams.indFirstTrack );

        bool refFirstTrackInReverse = ( hlParamsRef.indFirstWorkingTrack%2 == hlParamsRef.indFirstTrack%2 ?
                                        hlParamsRef.firstWorkingTrackInReverse :
                                        !hlParamsRef.firstWorkingTrackInReverse);
        if(trackDirectionFlagsRef.at( hlParamsRef.indFirstWorkingTrack ) != trackDirectionFlagsRef.at( hlParamsRef.indFirstTrack ))
            refFirstTrackInReverse = !refFirstTrackInReverse;

        Point ptRef;
        if(hlParamsRef.indTransitTrackStart >= 0){
            auto& transitTrack = hlRef.tracks.at( hlParamsRef.indTransitTrackStart );
            bool refTransitTrackInReverse = ( trackDirectionFlagsRef.at( hlParamsRef.indFirstTrack ) == trackDirectionFlagsRef.at( hlParamsRef.indTransitTrackStart ) ?
                                                !refFirstTrackInReverse :
                                                refFirstTrackInReverse );
            ptRef = ( refTransitTrackInReverse ? transitTrack.points.back() : transitTrack.points.front() );
        }
        else{
            auto& track = hlRef.tracks.at( hlParamsRef.indFirstTrack );
            ptRef = ( refFirstTrackInReverse ? track.points.back() : track.points.front() );
        }

        if(hl.tracks.size()%2 == 0){
            if( ip.workedAreaTransitRestriction == WorkedAreaTransitRestriction::TRANSIT_ONLY_OVER_UNWORKED_AREA )
                hlParams.indTransitTrackStart = ( hlParams.indFirstTrack == 0 ? 1 : hl.tracks.size()-2 );
            else
                hlParams.indTransitTrackFinish = ( hlParams.indFirstTrack == 0 ? hl.tracks.size()-2 : 1 );
        }

        if(hlParams.indTransitTrackFinish >= 0){
            auto& transitTrack = hl.tracks.at( hlParams.indTransitTrackFinish );
            hlParams.firstWorkingTrackInReverse = ( geometry::calc_dist(ptRef, transitTrack.points.front()) < geometry::calc_dist(ptRef, transitTrack.points.back()) );
        }
        else
            hlParams.firstWorkingTrackInReverse = ( geometry::calc_dist(ptRef, firstTrack.points.front()) > geometry::calc_dist(ptRef, firstTrack.points.back()) );

        hlParams.indFirstWorkingTrackPoint = ( hlParams.firstWorkingTrackInReverse ? firstTrack.points.size()-1 : 0 );
    }

    for(int i = indSortedList+1 ; i < ip.sortedPartialHLs.size() ; ++i){
        auto& hl = subfield.headlands.partial.at( ip.sortedPartialHLs.at(i) );
        auto& hlRef = subfield.headlands.partial.at( ip.sortedPartialHLs.at(i-1) );
        auto& hlParams = ip.hlParams.at( ip.sortedPartialHLs.at(i) );
        auto& hlParamsRef = ip.hlParams.at( ip.sortedPartialHLs.at(i-1) );
        auto& trackDirectionFlagsRef = hlParamsRef.trackDirectionFlags;

        if(i+1 == ip.sortedPartialHLs.size())
            hlParams.startFromOutermostTrack = !plannerParameters.finishWithOutermostTrack;
        else if (ip.workedAreaTransitRestriction == WorkedAreaTransitRestriction::NO_RESTRICTION){
            //hlParams.startFromOutermostTrack = !plannerParameters.finishWithOutermostTrack;
            hlParams.startFromOutermostTrack = true;
        }
        else{
            //hlParams.startFromOutermostTrack = (ip.workedAreaTransitRestriction == WorkedAreaTransitRestriction::TRANSIT_ONLY_OVER_UNWORKED_AREA);
            hlParams.startFromOutermostTrack = true;
        }

        hlParams.indFirstTrack = ( hlParams.startFromOutermostTrack ? 0 : hl.tracks.size()-1);
        if(!hlParams.tracksInwards)
            hlParams.indFirstTrack = hl.tracks.size()-1-hlParams.indFirstTrack;


        hlParams.indFirstWorkingTrack = hlParams.indFirstTrack;
        auto& firstTrack = hl.tracks.at( hlParams.indFirstTrack );

        size_t indLastTrackRef = hlRef.tracks.size() - 1 - hlParamsRef.indFirstTrack;

        bool refFirstTrackInReverse = ( hlParamsRef.indFirstWorkingTrack%2 == hlParamsRef.indFirstTrack%2 ?
                                        hlParamsRef.firstWorkingTrackInReverse :
                                        !hlParamsRef.firstWorkingTrackInReverse);
        if(trackDirectionFlagsRef.at( hlParamsRef.indFirstWorkingTrack ) != trackDirectionFlagsRef.at( hlParamsRef.indFirstTrack ))
            refFirstTrackInReverse = !refFirstTrackInReverse;

        bool refLastTrackInReverse = ( hlParamsRef.indFirstWorkingTrack%2 == indLastTrackRef%2 ?
                                        hlParamsRef.firstWorkingTrackInReverse :
                                        !hlParamsRef.firstWorkingTrackInReverse);
        if(trackDirectionFlagsRef.at( hlParamsRef.indFirstWorkingTrack ) != trackDirectionFlagsRef.at( indLastTrackRef ))
            refLastTrackInReverse = !refLastTrackInReverse;

        Point ptRef;
        if(hlParamsRef.indTransitTrackFinish >= 0){
            auto& transitTrack = hlRef.tracks.at( hlParamsRef.indTransitTrackFinish );
            bool refTransitTrackInReverse = ( trackDirectionFlagsRef.at( indLastTrackRef ) == trackDirectionFlagsRef.at( hlParamsRef.indTransitTrackFinish ) ?
                                                !refLastTrackInReverse :
                                                refLastTrackInReverse );
            ptRef = ( refTransitTrackInReverse ? transitTrack.points.front() : transitTrack.points.back() );
        }
        else{
            auto& track = hlRef.tracks.at( indLastTrackRef );
            ptRef = ( refLastTrackInReverse ? track.points.front() : track.points.back() );
        }

        if(hl.tracks.size()%2 == 0 && i+1 < ip.sortedPartialHLs.size()){
            if( ip.workedAreaTransitRestriction == WorkedAreaTransitRestriction::TRANSIT_ONLY_OVER_UNWORKED_AREA )
                hlParams.indTransitTrackStart = ( hlParams.indFirstTrack == 0 ? 1 : hl.tracks.size()-2 );
            else
                hlParams.indTransitTrackFinish = ( hlParams.indFirstTrack == 0 ? hl.tracks.size()-2 : 1 );
        }

        if(hlParams.indTransitTrackStart >= 0){
            auto& transitTrack = hl.tracks.at( hlParams.indTransitTrackStart );
            hlParams.firstWorkingTrackInReverse = ( geometry::calc_dist(ptRef, transitTrack.points.front()) < geometry::calc_dist(ptRef, transitTrack.points.back()) );
        }
        else
            hlParams.firstWorkingTrackInReverse = ( geometry::calc_dist(ptRef, firstTrack.points.front()) > geometry::calc_dist(ptRef, firstTrack.points.back()) );

        hlParams.indFirstWorkingTrackPoint = ( hlParams.firstWorkingTrackInReverse ? firstTrack.points.size()-1 : 0 );
    }
    return AroResp::ok();
}

int HeadlandBaseRoutesPlanner::getFirstPartialHeadlandWorkingTrackInd(const Subfield &subfield,
                                                                      size_t indHL, const InternalParametersForPartialHLs &ip,
                                                                      bool startHLFromOutermostTrack, bool onlyToBeWorked)
{
    if(onlyToBeWorked){
        auto& tracksToWork = ip.hlParams.at(indHL).tracksToWork;
        if(tracksToWork.empty())
            return -1;
        return( startHLFromOutermostTrack == ip.hlParams.at(indHL).tracksInwards ? tracksToWork.front() : tracksToWork.back() );
    }

    auto& tracks = subfield.headlands.partial.at(indHL).tracks;
    return ( startHLFromOutermostTrack == ip.hlParams.at(indHL).tracksInwards ? 0 : tracks.size()-1 );
}

int HeadlandBaseRoutesPlanner::getLastPartialHeadlandWorkingTrackInd(size_t indHL, const InternalParametersForPartialHLs &ip, bool startHLFromOutermostTrack)
{
    auto& tracksToWork = ip.hlParams.at(indHL).tracksToWork;
    if(tracksToWork.empty())
        return -1;
    return( startHLFromOutermostTrack == ip.hlParams.at(indHL).tracksInwards ? tracksToWork.back() : tracksToWork.front() );
}

size_t HeadlandBaseRoutesPlanner::getPartialHeadlandTransitTrackInd(const Subfield &subfield, const InternalParametersForPartialHLs &ip, size_t indHL, size_t indWorkingTrackRef)
{
    auto& tracks = subfield.headlands.partial.at(indHL).tracks;
    if( tracks.size() == 1 )
        return 0;
    else if( indWorkingTrackRef == 0 )
        return 1;
    else if( indWorkingTrackRef == tracks.size()-1 )
        return indWorkingTrackRef - 1;
    else if( ip.hlParams.at(indHL).tracksInwards )
        return indWorkingTrackRef - 1;
    else
        return indWorkingTrackRef + 1;
}

AroResp HeadlandBaseRoutesPlanner::getFirstHeadlandAndSortHeadlands(const Subfield &subfield, InternalParametersForPartialHLs &ip)
{
    //first, reverse the oder of the presorted HLs based on the params of the 1st working headland
    auto& firstWorkingHeadlandParams = ip.hlParams.at(ip.indFirstWorkingHeadland);
    if( firstWorkingHeadlandParams.nextWorkingHL >= 0
            && getHeadlandIndexFromSortedList(ip.indFirstWorkingHeadland, 1, ip) != firstWorkingHeadlandParams.nextWorkingHL )
        std::reverse( ip.sortedPartialHLs.begin(), ip.sortedPartialHLs.end() );

    auto& hls = subfield.headlands.partial;
    ip.indFirstHeadland = ip.indFirstWorkingHeadland;
    for(size_t i = 1 ; i < hls.size() ; ++i){
        size_t ind;
        if(!isHeadlandWorked(ip.indFirstWorkingHeadland, -i, ip, &ind)){
            break;
        }
        ip.indFirstHeadland = ind;
    }

    for(size_t i = 0 ; i < ip.sortedPartialHLs.size() ; ++i){
        if(ip.sortedPartialHLs.at(i) == ip.indFirstHeadland){
            if(i == 0)
                break;
            ip.sortedPartialHLs.insert( ip.sortedPartialHLs.end(), ip.sortedPartialHLs.begin(), ip.sortedPartialHLs.begin()+i );
            ip.sortedPartialHLs.erase( ip.sortedPartialHLs.begin(), ip.sortedPartialHLs.begin()+i );
            break;
        }
    }

    return AroResp::ok();

}

AroResp HeadlandBaseRoutesPlanner::generateRoute_partialHLs(const Subfield &subfield,
                                                             const Machine &machine,
                                                             const PlannerParameters & plannerParameters,
                                                             IEdgeMassCalculator & edgeMassCalculator,
                                                             IEdgeSpeedCalculator & edgeSpeedCalculator,
                                                             const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                                             InternalParametersForPartialHLs &ip,
                                                             Route & route){

    route.route_id = 0;
    route.machine_id = machine.id;
    route.route_points.clear();

    auto aroResp = updateHLParamsForNonFirstWorkingHeadlands(subfield,
                                                             plannerParameters,
                                                             ip);
    if(aroResp.isError())
        return AroResp::LoggingResp(1, "Error obtaining the parameters of partial headlands (non-first-working)", ": " + aroResp.msg, m_logger, LogLevel::ERROR, __FUNCTION__);

    for(size_t i = 0 ; i < ip.sortedPartialHLs.size() ; ++i){
        auto& indHL = ip.sortedPartialHLs.at(i);
        aroResp = addRouteSegments_partialHL(subfield,
                                             machine,
                                             indHL,
                                             plannerParameters,
                                             edgeMassCalculator,
                                             edgeSpeedCalculator,
                                             machineCurrentStates,
                                             ip,
                                             route);
        if(aroResp.isError())
           return AroResp::LoggingResp(1, "Error adding route segments for headland with index " + std::to_string( indHL ), ": " + aroResp.msg, m_logger, LogLevel::ERROR, __FUNCTION__);

        if(i+1 < ip.sortedPartialHLs.size()){
            addInterHeadlandConnection_partialHLs(route,
                                                  subfield,
                                                  machine,
                                                  indHL,
                                                  ip.sortedPartialHLs.at(i+1),
                                                  ip,
                                                  false,
                                                  plannerParameters,
                                                  edgeMassCalculator,
                                                  edgeSpeedCalculator,
                                                  machineCurrentStates);
        }
    }

    return AroResp::ok();
}

AroResp HeadlandBaseRoutesPlanner::addRouteSegments_partialHL(const Subfield &subfield,
                                                              const Machine &machine,
                                                              size_t indHL,
                                                              const PlannerParameters & plannerParameters,
                                                              IEdgeMassCalculator & edgeMassCalculator,
                                                              IEdgeSpeedCalculator & edgeSpeedCalculator,
                                                              const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                                              InternalParametersForPartialHLs &ip,
                                                              Route & route)
{
    bool hlIsWorked = isHeadlandWorked(indHL, 0, ip);
    if(plannerParameters.removeInitialWorkedSegments && hlIsWorked)
        return AroResp::ok();

    auto& hl = subfield.headlands.partial.at(indHL);
    auto& hlParams = ip.hlParams.at(indHL);
    auto& trackDirectionFlags = hlParams.trackDirectionFlags;

    bool isPartiallyWorked = isPartialHeadlandPartiallyWorked(subfield, indHL, ip);

    bool trackInReverse = ( hlParams.indFirstWorkingTrack%2 == hlParams.indFirstTrack%2 ? hlParams.firstWorkingTrackInReverse : !hlParams.firstWorkingTrackInReverse);//corresponding to the 1st track with respect to the 1st working track

    if( hlParams.indTransitTrackStart >= 0 &&
            (!isPartiallyWorked || !plannerParameters.removeInitialWorkedSegments) ){

        bool firstTrackInReverse = ( trackDirectionFlags.at( hlParams.indFirstWorkingTrack ) == trackDirectionFlags.at( hlParams.indFirstTrack ) ?
                                        trackInReverse : !trackInReverse );
        bool transitTrackInReverse = ( trackDirectionFlags.at( hlParams.indTransitTrackStart ) == trackDirectionFlags.at( hlParams.indFirstTrack ) ?
                                        !firstTrackInReverse : firstTrackInReverse );

        auto& transitTrack = hl.tracks.at(hlParams.indTransitTrackStart);
        size_t indFrom = ( transitTrackInReverse ? transitTrack.points.size()-1 : 0 );
        size_t indTo = transitTrack.points.size()-1-indFrom;
        Track transitTrackEd;
        if( machine.turning_radius > 0 && geometry::getGeometryLength(transitTrack.points) > machine.turning_radius ){//if possible, remove segment (with distance = turning_radius) from extrema
            transitTrackEd = transitTrack;
            if(indFrom > indTo){
                std::swap(indFrom, indTo);
                std::reverse(transitTrackEd.points.begin(), transitTrackEd.points.end());
            }
            auto pt_n_ind = geometry::getPointAtDist(transitTrackEd.points, -machine.turning_radius);
            if(pt_n_ind.second >= 0){
                transitTrackEd.points.at(pt_n_ind.second) = pt_n_ind.first;
                if(pt_n_ind.second+1 < transitTrackEd.points.size())
                    transitTrackEd.points.erase(transitTrackEd.points.begin()+pt_n_ind.second+1, transitTrackEd.points.end());
            }
            if( geometry::getGeometryLength(transitTrackEd.points) > machine.turning_radius ){
                pt_n_ind = geometry::getPointAtDist(transitTrackEd.points, machine.turning_radius);
                if(pt_n_ind.second >= 0){
                    transitTrackEd.points.at(pt_n_ind.second) = pt_n_ind.first;
                    if(pt_n_ind.second > 0)
                        transitTrackEd.points.erase(transitTrackEd.points.begin(), transitTrackEd.points.begin()+pt_n_ind.second);
                }

            }
            indTo = transitTrackEd.points.size();
        }

        addTrackPointsToRoute_partialHLs(route,
                                         machine,
                                         ( transitTrackEd.points.empty() ? transitTrack : transitTrackEd ),
                                         indFrom,
                                         indTo,
                                         !hlIsWorked && !isPartiallyWorked,
                                         true,
                                         edgeMassCalculator,
                                         edgeSpeedCalculator,
                                         ip,
                                         plannerParameters,
                                         machineCurrentStates);
        addInterTrackConnection_partialHLs(route,
                                           machine,
                                           hl.tracks.at( hlParams.indFirstTrack ),
                                           firstTrackInReverse,
                                           true,
                                           !hlIsWorked && !isPartiallyWorked,
                                           true,
                                           edgeMassCalculator,
                                           edgeSpeedCalculator,
                                           ip,
                                           plannerParameters,
                                           machineCurrentStates);
    }

    int deltaTrackIdx = ( hlParams.startFromOutermostTrack == hlParams.tracksInwards ? 1 : -1 );

    bool isWorking = false;
    int trackInd = -1;


    for(int isUnworkedSide = 0 ; isUnworkedSide+hlIsWorked < 2 ; ++isUnworkedSide ){
        int forLimit;
        if(isUnworkedSide == 0){
            if(hlIsWorked)
                forLimit = hl.tracks.size(); //@todo the hlParams have not been updated -> the worked tracks might be added in weird/random order (we need to add 1st the not-worked headlands and then the worked ones in rev (sortedHL) order? what about the worked mass in the route points?)
            else
                forLimit = std::abs((int)hlParams.indFirstTrack - (int)hlParams.indFirstWorkingTrack);
        }
        else
            forLimit = hlParams.tracksToWork.size();

        for(int i = 0 ; i < forLimit ; ++i, trackInReverse = !trackInReverse){

            if(isUnworkedSide == 0){
                if(plannerParameters.removeInitialWorkedSegments)
                    continue;
                trackInd = hlParams.indFirstTrack + i*deltaTrackIdx;
            }
            else if(deltaTrackIdx > 0)
                trackInd = hlParams.tracksToWork.at(i);
            else
                trackInd = r_at(hlParams.tracksToWork, i);

            isWorking |= (trackInd == hlParams.indFirstWorkingTrack && !hlIsWorked);

            bool currentTrackInReverse = ( trackDirectionFlags.at( hlParams.indFirstWorkingTrack ) == trackDirectionFlags.at( trackInd ) ?
                                            trackInReverse : !trackInReverse );

            auto& track = hl.tracks.at(trackInd);
            size_t indFrom = ( currentTrackInReverse ? track.points.size()-1 : 0);
            size_t indTo = track.points.size()-1-indFrom;
            if(!isWorking
                    || trackInd != hlParams.indFirstWorkingTrack
                    || hlParams.indFirstWorkingTrackPointToWork == 0
                    || hlParams.indFirstWorkingTrackPointToWork == track.points.size()-1){
                addTrackPointsToRoute_partialHLs(route,
                                                 machine,
                                                 track,
                                                 indFrom,
                                                 indTo,
                                                 isWorking,
                                                 false,
                                                 edgeMassCalculator,
                                                 edgeSpeedCalculator,
                                                 ip,
                                                 plannerParameters,
                                                 machineCurrentStates);
            }
            else{
                //add the indFirstWorkingTrack separatelly for worked and unworked based on the indexes

                size_t indFromWorked = indFrom;
                size_t indToWorked = hlParams.indFirstWorkingTrackPointToWork;
                size_t indFromUnworked = hlParams.indFirstWorkingTrackPointToWork;
                size_t indToUnworked = indTo;
                addTrackPointsToRoute_partialHLs(route,
                                                 machine,
                                                 track,
                                                 indFromWorked,
                                                 indToWorked,
                                                 false,
                                                 false,
                                                 edgeMassCalculator,
                                                 edgeSpeedCalculator,
                                                 ip,
                                                 plannerParameters,
                                                 machineCurrentStates);
                addTrackPointsToRoute_partialHLs(route,
                                                 machine,
                                                 track,
                                                 indFromUnworked,
                                                 indToUnworked,
                                                 true,
                                                 false,
                                                 edgeMassCalculator,
                                                 edgeSpeedCalculator,
                                                 ip,
                                                 plannerParameters,
                                                 machineCurrentStates);
            }
            if(trackInd+deltaTrackIdx < hl.tracks.size() && trackInd+deltaTrackIdx > 0){
                bool nextTrackInReverse = ( trackDirectionFlags.at( hlParams.indFirstWorkingTrack ) == trackDirectionFlags.at( trackInd+deltaTrackIdx ) ?
                                                !trackInReverse : trackInReverse );
                addInterTrackConnection_partialHLs(route,
                                                   machine,
                                                   hl.tracks.at( trackInd+deltaTrackIdx ),
                                                   nextTrackInReverse,
                                                   true,
                                                   isWorking,
                                                   false,
                                                   edgeMassCalculator,
                                                   edgeSpeedCalculator,
                                                   ip,
                                                   plannerParameters,
                                                   machineCurrentStates);
            }
        }
    }


//    //old
//    for(int i = 0; i < hl.tracks.size() ; ++i, trackInReverse = !trackInReverse){

//        trackInd = hlParams.indFirstTrack + i*deltaTrackIdx;

//        isWorking |= (trackInd == hlParams.indFirstWorkingTrack);
//        if(!isWorking && plannerParameters.removeInitialWorkedSegments)
//            continue;

//        bool currentTrackInReverse = ( trackDirectionFlags.at( hlParams.indFirstWorkingTrack ) == trackDirectionFlags.at( trackInd ) ?
//                                        trackInReverse : !trackInReverse );

//        auto& track = hl.tracks.at(trackInd);
//        size_t indFrom = ( currentTrackInReverse ? track.points.size()-1 : 0);
//        size_t indTo = track.points.size()-1-indFrom;
//        if(!isWorking
//                || trackInd != hlParams.indFirstWorkingTrack
//                || hlParams.indFirstWorkingTrackPoint == 0
//                || hlParams.indFirstWorkingTrackPoint == track.points.size()-1){
//            addTrackPointsToRoute_partialHLs(route,
//                                             machine,
//                                             track,
//                                             indFrom,
//                                             indTo,
//                                             isWorking,
//                                             false,
//                                             edgeMassCalculator,
//                                             edgeSpeedCalculator,
//                                             ip,
//                                             plannerParameters,
//                                             machineCurrentStates);
//        }
//        else{
//            //add the indFirstWorkingTrack separatelly for worked and unworked based on the indexes

//            size_t indFromWorked = indFrom;
//            size_t indToWorked = hlParams.indFirstWorkingTrackPoint-1;
//            size_t indFromUnworked = hlParams.indFirstWorkingTrackPoint;
//            size_t indToUnworked = indTo;
//            addTrackPointsToRoute_partialHLs(route,
//                                             machine,
//                                             track,
//                                             indFromWorked,
//                                             indToWorked,
//                                             isWorking,
//                                             false,
//                                             edgeMassCalculator,
//                                             edgeSpeedCalculator,
//                                             ip,
//                                             plannerParameters,
//                                             machineCurrentStates);
//            addTrackPointsToRoute_partialHLs(route,
//                                             machine,
//                                             track,
//                                             indFromUnworked,
//                                             indToUnworked,
//                                             isWorking,
//                                             false,
//                                             edgeMassCalculator,
//                                             edgeSpeedCalculator,
//                                             ip,
//                                             plannerParameters,
//                                             machineCurrentStates);
//        }
//        if(trackInd+deltaTrackIdx < hl.tracks.size() && trackInd+deltaTrackIdx > 0){
//            bool nextTrackInReverse = ( trackDirectionFlags.at( hlParams.indFirstWorkingTrack ) == trackDirectionFlags.at( trackInd+deltaTrackIdx ) ?
//                                            !trackInReverse : trackInReverse );
//            addInterTrackConnection_partialHLs(route,
//                                               machine,
//                                               hl.tracks.at( trackInd+deltaTrackIdx ),
//                                               nextTrackInReverse,
//                                               true,
//                                               isWorking,
//                                               false,
//                                               edgeMassCalculator,
//                                               edgeSpeedCalculator,
//                                               ip,
//                                               plannerParameters,
//                                               machineCurrentStates);
//        }
//    }


    if(trackInd >= 0 && hlParams.indTransitTrackFinish >= 0){

        bool prevTrackInReverse = ( trackDirectionFlags.at( hlParams.indFirstWorkingTrack ) == trackDirectionFlags.at( trackInd ) ?
                                        !trackInReverse : trackInReverse );//in the last iteration trackInReverse was updated
        bool transitTrackInReverse = ( trackDirectionFlags.at( hlParams.indTransitTrackFinish ) == trackDirectionFlags.at( trackInd ) ?
                                        !prevTrackInReverse : prevTrackInReverse );

        auto& transitTrack = hl.tracks.at(hlParams.indTransitTrackFinish);
        size_t indFrom = ( transitTrackInReverse ? transitTrack.points.size()-1 : 0 );
        size_t indTo = transitTrack.points.size()-1-indFrom;

        Track transitTrackEd;
        if( machine.turning_radius > 0 && geometry::getGeometryLength(transitTrack.points) > machine.turning_radius ){//if possible, remove segment (with distance = turning_radius) from extrema
            transitTrackEd = transitTrack;
            transitTrackInReverse = false;
            if(indFrom > indTo){
                std::swap(indFrom, indTo);
                std::reverse(transitTrackEd.points.begin(), transitTrackEd.points.end());
            }

            auto pt_n_ind = geometry::getPointAtDist(transitTrackEd.points, machine.turning_radius);
            if(pt_n_ind.second >= 0){
                transitTrackEd.points.at(pt_n_ind.second) = pt_n_ind.first;
                if(pt_n_ind.second > 0)
                    transitTrackEd.points.erase(transitTrackEd.points.begin(), transitTrackEd.points.begin()+pt_n_ind.second);
            }

            if( geometry::getGeometryLength(transitTrackEd.points) > machine.turning_radius ){
                pt_n_ind = geometry::getPointAtDist(transitTrackEd.points, -machine.turning_radius);
                if(pt_n_ind.second >= 0){
                    transitTrackEd.points.at(pt_n_ind.second) = pt_n_ind.first;
                    if(pt_n_ind.second+1 < transitTrackEd.points.size())
                        transitTrackEd.points.erase(transitTrackEd.points.begin()+pt_n_ind.second+1, transitTrackEd.points.end());
                }
            }
            indTo = transitTrackEd.points.size();
        }



        addInterTrackConnection_partialHLs(route,
                                           machine,
                                           ( transitTrackEd.points.empty() ? transitTrack : transitTrackEd ),
                                           transitTrackInReverse,
                                           true,
                                           true,
                                           true,
                                           edgeMassCalculator,
                                           edgeSpeedCalculator,
                                           ip,
                                           plannerParameters,
                                           machineCurrentStates);

        addTrackPointsToRoute_partialHLs(route,
                                         machine,
                                         ( transitTrackEd.points.empty() ? transitTrack : transitTrackEd ),
                                         indFrom,
                                         indTo,
                                         true,
                                         true,
                                         edgeMassCalculator,
                                         edgeSpeedCalculator,
                                         ip,
                                         plannerParameters,
                                         machineCurrentStates);
    }

    return AroResp::ok();

}

void HeadlandBaseRoutesPlanner::addTrackPointsToRoute_partialHLs(Route& route,
                                                                 const Machine& machine,
                                                                 const Track& track,
                                                                 size_t indFrom,
                                                                 size_t indTo,
                                                                 bool isWorking,
                                                                 bool isTransit,
                                                                 IEdgeMassCalculator &edgeMassCalculator,
                                                                 IEdgeSpeedCalculator &edgeSpeedCalculator,
                                                                 InternalParametersForPartialHLs &ip,
                                                                 const PlannerParameters &plannerParameters,
                                                                 const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                                                 double extraSpeedFactor)
{
    const bool useSpeedCalculatorTurningTime = true;

    bool ptsInReverse;
    auto points = getTrackSegment_partialHL(track, ip, indFrom, indTo, ptsInReverse);

    if(points.empty())
        return;

    const double minSpeed = 0.01;//@TODO hardcoded. should we get this as a parameter?

    double width = ( machine.working_width > 1e-5 ? machine.working_width :
                                                    machine.width > 1e-5 ? machine.width :
                                                                           track.width );

    double speedFactor = (plannerParameters.speedMultiplier > 0 ? plannerParameters.speedMultiplier : 1);

    if(extraSpeedFactor > 1e-6)
        speedFactor *= extraSpeedFactor;

    auto getTurningTime = [&edgeSpeedCalculator, &machine](const Point& p1, const Point& p2, const Point& p3)->double{
        if(p1 == p2 || p2 == p3)
            return 0;
        double ang = geometry::get_angle(p1, p2, p3);
        return(edgeSpeedCalculator.calcTurningTime(ang, 0, machine));
    };

    route.route_points.reserve( route.route_points.size() + points.size() );
    for(size_t i = 0 ; i < points.size() ; ++i){
        RoutePoint rp(points.at(i));
        rp.track_id = track.id;
        rp.type = ( isTransit ? RoutePoint::TRANSIT : RoutePoint::getDefaultRPType(machine) );

        if (route.route_points.empty()) {
            rp.type = RoutePoint::TRACK_START;
            rp.time_stamp = ( isWorking ? 0.0 : -1 );
            rp.bunker_mass = 0.0;
            rp.bunker_volume = 0.0;
            rp.worked_mass = 0.0;
            rp.worked_volume = 0.0;
            if(machineCurrentStates){
                auto it_m = machineCurrentStates->find(machine.id);
                if(it_m != machineCurrentStates->end()){
                    const MachineDynamicInfo& mdi = it_m->second;
                    rp.bunker_mass = mdi.bunkerMass;
                    rp.bunker_volume = mdi.bunkerVolume;
                }
            }
        }
        else{
            auto& rpPrev = route.route_points.back();
            rp.copyBasicWorkingValuesFrom(rpPrev);

            bool isSegmentWorked;

            if(!isTransit){
                if( i+1 == points.size() && track.id >= 0
                        && (indTo == 0 || indTo == track.points.size()-1) )
                    rp.type = RoutePoint::TRACK_END;
                else if ( i == 0 && track.id >= 0
                          && (indFrom == 0 || indFrom == track.points.size()-1) )
                    rp.type = RoutePoint::TRACK_START;
            }

            if(!isWorking)
                rp.time_stamp = -1;
            else if(isTransit)
                rp.time_stamp = rpPrev.time_stamp + arolib::geometry::calc_dist(rpPrev, rp) / getTransitSpeed(machine, rpPrev.bunker_mass);
            else{

                double speedFactor2 = 1;

                if(!useSpeedCalculatorTurningTime){
                    if(route.route_points.size() > 2){
                        const double minFactor = 0.05;
                        auto& rpPrev2 = r_at(route.route_points, 1);
                        double angle = std::fabs( geometry::get_angle(rpPrev2, rpPrev, rp, true) );
                        speedFactor2 = minFactor + (1-minFactor) * angle / 180;
                        speedFactor2 = std::min(1.0, speedFactor2);
                    }
                }

                double maxSpeed = machine.calcSpeed(rpPrev.bunker_mass);
                double calcSpeed = speedFactor * speedFactor2 * edgeSpeedCalculator.calcSpeed(rpPrev,
                                                                                              rp,
                                                                                              0,
                                                                                              machine);
                if (calcSpeed > maxSpeed)
                    calcSpeed = maxSpeed;
                if (calcSpeed < minSpeed){
                    logger().printOut(LogLevel::WARNING, __FUNCTION__, "Calculated harvester speed in track " + std::to_string(track.id)
                                       + " (index = " + std::to_string(i) + ") is lower than " + std::to_string(minSpeed) + " m/s");
                    calcSpeed = minSpeed;
                }

                rp.time_stamp = ( rpPrev.time_stamp < 1e-9 ? 0.0 : rpPrev.time_stamp)
                                    + arolib::geometry::calc_dist(rpPrev, rp) / calcSpeed;
            }

            double mass = 0;
            if(rp.type != RoutePoint::TRACK_START)
            //if(rpPrev.track_id != track.id)
                mass = getMass(rpPrev, rp, width, edgeMassCalculator, ip, isSegmentWorked);

            rp.worked_mass = rpPrev.worked_mass + mass;
            rp.worked_volume = rpPrev.worked_volume + 0.0;//@TODO

            if( geometry::calc_dist(rpPrev, rp) < 1e-3 )
                route.route_points.pop_back();

        }

        if(useSpeedCalculatorTurningTime){
            if(i > 0 && i+1 < points.size() && rp.time_stamp > -1e-9){
                double turningTime = getTurningTime( points.at(i-1), points.at(i), points.at(i+1) );
                if(turningTime > 1){
                    route.route_points.push_back(rp);
                    rp.time_stamp += turningTime;
                }
            }
        }

        route.route_points.push_back(rp);
    }

    if(plannerParameters.monitorPlannedAreas && isWorking && !isTransit)
        updateWorkedAreas(points, width, ip);

}

void HeadlandBaseRoutesPlanner::addInterTrackConnection_partialHLs(Route& route,
                                                                   const Machine& machine,
                                                                   const Track& nextTrack,
                                                                   bool nextTrackInReverse,
                                                                   bool removeLastPoint,
                                                                   bool isWorking,
                                                                   bool isTransit,
                                                                   IEdgeMassCalculator &edgeMassCalculator,
                                                                   IEdgeSpeedCalculator &edgeSpeedCalculator,
                                                                   InternalParametersForPartialHLs &ip,
                                                                   const PlannerParameters &plannerParameters,
                                                                   const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates)
{
    if(route.route_points.empty())
        return;


    Track connTrack;
    connTrack.id = -1;
    connTrack.points.push_back( nextTrackInReverse ? nextTrack.points.back() : nextTrack.points.front() );

    double extraSpeedFactor = 1;
    if(route.route_points.size() > 1){
        auto& lastRP = route.route_points.back();
        Pose2D pose0( lastRP );
        pose0.angle = geometry::get_angle( r_at(route.route_points, 1), lastRP );
        Pose2D pose1;
        pose1.point() = ( nextTrackInReverse ? nextTrack.points.back() : nextTrack.points.front() );
        const Point& pt2Next = ( nextTrackInReverse ? r_at(nextTrack.points, 1) : nextTrack.points.at(1) );
        pose1.angle = geometry::get_angle( pose1, pt2Next );
        auto lengthDubbins = geometry::calcDubinsPathLength(pose0, pose1, machine.getTurningRadius());
        if(lengthDubbins > 1e-6){
            auto dist = geometry::calc_dist(route.route_points.back(), connTrack.points.back());
            extraSpeedFactor = dist / lengthDubbins;
        }

    }

    addTrackPointsToRoute_partialHLs(route, machine, connTrack, 0, 0, isWorking, isTransit,
                                     edgeMassCalculator, edgeSpeedCalculator, ip, plannerParameters, machineCurrentStates, extraSpeedFactor);



//    if(route.route_points.size() < 2)
//        return;

//    auto& lastRP = route.route_points.back();
//    Pose2D pose0( lastRP );
//    pose0.angle = geometry::get_angle( r_at(route.route_points, 1), lastRP );
//    Pose2D pose1;
//    pose1.point() = ( nextTrackInReverse ? nextTrack.points.back() : nextTrack.points.front() );
//    const Point& pt2Next = ( nextTrackInReverse ? r_at(nextTrack.points, 1) : nextTrack.points.at(1) );
//    pose1.angle = geometry::get_angle( pose1, pt2Next );
//    //auto points = geometry::calcDubinsPath(10, pose0, pose1, machine.getTurningRadius());
//    auto points = geometry::calcDubinsPath(pose0, pose1, machine.getTurningRadius(), 15, true);

//    if(machine.length > 1e-6)
//        points = geometry::sample_geometry(points, 2*machine.length);

//    double speed = 0.7 * getTransitSpeed(machine, lastRP.bunker_mass);

//    //for now, connect with a straigh line but use the length of the path to compute the duration of the turn
//    double length = geometry::getGeometryLength(points);
//    double dTime = length/speed;
//    RoutePoint rp;
//    rp.copyBasicWorkingValuesFrom(lastRP);
//    rp.type = RoutePoint::TRANSIT;
//    rp.point() = geometry::getCentroid(pose0, pose1);
//    rp.time_stamp = lastRP.time_stamp + 0.5 * dTime;
//    route.route_points.push_back(rp);
//    rp.point() = pose1.point();
//    rp.time_stamp += 0.5 * dTime;
//    route.route_points.push_back(rp);




//    route.route_points.reserve( route.route_points.size() + points.size() );
//    for(size_t i = 1 ; i + removeLastPoint < points.size() ; ++i){
//        const auto& rpPrev = route.route_points.back();
//        RoutePoint rp( points.at(i) );
//        rp.copyBasicWorkingValuesFrom(rpPrev);
//        rp.type = RoutePoint::TRANSIT;
//        rp.time_stamp = rpPrev.time_stamp + geometry::calc_dist(rp, rpPrev) / speed;
//        route.route_points.push_back(rp);
//    }

}

void HeadlandBaseRoutesPlanner::addInterHeadlandConnection_partialHLs(Route& route,
                                                                      const Subfield &subfield,
                                                                      const Machine& machine,
                                                                      size_t indHLFrom,
                                                                      size_t indHLTo,
                                                                      InternalParametersForPartialHLs &ip,
                                                                      bool removeLastPoint,
                                                                      const PlannerParameters & plannerParameters,
                                                                      IEdgeMassCalculator &edgeMassCalculator,
                                                                      IEdgeSpeedCalculator &edgeSpeedCalculator,
                                                                      const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates)
{

    auto& hlFrom = subfield.headlands.partial.at(indHLFrom);
    auto& hlTo = subfield.headlands.partial.at(indHLTo);


    bool isNextWorked = isHeadlandWorked(indHLTo, 0, ip);
    bool isNextPartiallyWorked = isPartialHeadlandPartiallyWorked(subfield, indHLTo, ip);

    auto& hlParamsNext = ip.hlParams.at(indHLTo);

    size_t nextTrackInd = ( hlParamsNext.indTransitTrackStart < 0 ? hlParamsNext.indFirstTrack : hlParamsNext.indTransitTrackStart );
    bool nextTrackInReverse = ( hlParamsNext.indFirstWorkingTrack%2 == nextTrackInd%2 ?
                                    hlParamsNext.firstWorkingTrackInReverse :
                                    !hlParamsNext.firstWorkingTrackInReverse);
    auto& nextTrack = hlTo.tracks.at( nextTrackInd );

    if( hlFrom.isConnectingHeadland() && !hlTo.isConnectingHeadland() && !route.route_points.empty()
            && nextTrackInd != ( hlParamsNext.tracksInwards ? hlTo.tracks.size()-1 : 0 ) ){
        Track connTrack;
        std::map<size_t, int> trackIdsMap;
        Point pRef = route.route_points.back().point();
        if(hlParamsNext.tracksInwards){
            for(size_t i = hlTo.tracks.size()-1 ; i > nextTrackInd ; --i){
                trackIdsMap[connTrack.points.size()] = hlTo.tracks.at(i).id;
                pRef = ( geometry::calc_dist(pRef, hlTo.tracks.at(i).points.back()) > geometry::calc_dist(pRef, hlTo.tracks.at(i).points.front()) ? hlTo.tracks.at(i).points.front() : hlTo.tracks.at(i).points.back() );
                connTrack.points.push_back( pRef );
            }
        }
        else{
            for(size_t i = 0 ; i < nextTrackInd ; ++i){
                trackIdsMap[connTrack.points.size()] = hlTo.tracks.at(i).id;
                pRef = ( geometry::calc_dist(pRef, hlTo.tracks.at(i).points.back()) > geometry::calc_dist(pRef, hlTo.tracks.at(i).points.front()) ? hlTo.tracks.at(i).points.front() : hlTo.tracks.at(i).points.back() );
                connTrack.points.push_back( pRef );
            }
        }
        if(!removeLastPoint)
            connTrack.points.push_back( nextTrackInReverse ? nextTrack.points.back() : nextTrack.points.front() );

        bool workIt =  ( nextTrackInd == hlParamsNext.indFirstTrack
                            && ip.workedAreaTransitRestriction != WorkedAreaTransitRestriction::TRANSIT_ONLY_OVER_UNWORKED_AREA );
        connTrack.id = -1;
        addTrackPointsToRoute_partialHLs(route, machine, connTrack, 0, connTrack.points.size()-1,
                                         !isNextWorked && !isNextPartiallyWorked,
                                         !workIt,
                                         edgeMassCalculator, edgeSpeedCalculator, ip, plannerParameters, machineCurrentStates);

        //adjust route point track ids
        for(auto& it : trackIdsMap){
            RoutePoint& rp = r_at(route.route_points, connTrack.points.size() - 1 - it.first);
            rp.track_id = it.second;
        }
    }
    else{//for now, do the same as with the inter-tracks connections
        bool workIt = hlTo.isConnectingHeadland() ||
                        ( nextTrackInd == hlParamsNext.indFirstTrack
                            && ip.workedAreaTransitRestriction != WorkedAreaTransitRestriction::TRANSIT_ONLY_OVER_UNWORKED_AREA );

        addInterTrackConnection_partialHLs(route, machine, nextTrack, nextTrackInReverse, removeLastPoint,
                                           !isNextWorked && !isNextPartiallyWorked,
                                           !workIt,
                                           edgeMassCalculator, edgeSpeedCalculator, ip, plannerParameters, machineCurrentStates);
    }
}

int HeadlandBaseRoutesPlanner::getHeadlandIndexFromSortedList(size_t indHLRef, int deltaInd, const InternalParametersForPartialHLs &ip)
{
    int ind = getSortedListIndexForHeadland(indHLRef, ip);
    if(ind < 0)
        return -1;
    return at_cyclic(ip.sortedPartialHLs, ind+deltaInd);
}

int HeadlandBaseRoutesPlanner::getSortedListIndexForHeadland(size_t indHL, const InternalParametersForPartialHLs &ip)
{
    for(size_t i = 0 ; i < ip.sortedPartialHLs.size() ; ++i ){
        if(ip.sortedPartialHLs.at(i) == indHL)
            return i;
    }
    return -1;
}

bool HeadlandBaseRoutesPlanner::isSegmentWorked(const Point &p0, const Point &p1, double width,
                                                InternalParametersGeneral &ip){
    if(!ip.gm.hasGrid(RemainingAreaMapName))
        return false;

    bool errorTmp = true;
    double value = 1;
    double area = arolib::geometry::calc_dist(p0, p1) * width;

    if( area > 1e-9 ){
        std::vector<gridmap::GridmapLayout::GridCellOverlap> cellsInfo;
        ip.gm.getCellsInfoUnderLine(RemainingAreaMapName, p0, p1, width, ip.precision_wam, cellsInfo);

        if(!cellsInfo.empty())
            value = ip.gm.getGrid(RemainingAreaMapName)->getCellsComputedValue( cellsInfo,
                                                                                ArolibGrid_t::AVERAGE_TOTAL,
                                                                                area,
                                                                                false,
                                                                                &errorTmp );
    }
    else{
        arolib::Point p0_1;
        p0_1.x = 0.5*( p0.x + p1.x );
        p0_1.y = 0.5*( p0.y + p1.y );
        if(ip.gm.getGrid(RemainingAreaMapName)->hasValue(p0_1))
            value = ip.gm.getGrid(RemainingAreaMapName)->getValue(p0_1, &errorTmp);
    }

    return (!errorTmp && value < ThresholdIsWorked);

}

bool HeadlandBaseRoutesPlanner::isHeadlandWorked(size_t indHLRef, int deltaInd, const InternalParametersForPartialHLs &ip, size_t* indHL)
{
    size_t ind = getHeadlandIndexFromSortedList(indHLRef, deltaInd, ip);
    if(indHL) *indHL = ind;
    return ip.hlParams.at(ind).state == InternalParametersForPartialHLs::PHL_WORKED;
}

bool HeadlandBaseRoutesPlanner::isPartialHeadlandPartiallyWorked(const Subfield &subfield, size_t indHL, const InternalParametersForPartialHLs &ip){
    auto& hlParams = ip.hlParams.at(indHL);
    const auto& firstWorkingTrack = subfield.headlands.partial.at(indHL).tracks.at( hlParams.indFirstWorkingTrack );

    return ( indHL == ip.indFirstWorkingHeadland
             && ( hlParams.indFirstTrack != hlParams.indFirstWorkingTrack
                || ( hlParams.indFirstWorkingTrackPoint != 0
                        && hlParams.indFirstWorkingTrackPoint != firstWorkingTrack.points.size()-1 )
                    )
             );
}

float HeadlandBaseRoutesPlanner::getMass(const Point &p0,
                                         const Point &p1,
                                         double width,
                                         IEdgeMassCalculator &edgeMassCalculator,
                                         InternalParametersGeneral &ip,
                                         bool & isWorked)
{
    double area = arolib::geometry::calc_dist(p0, p1) * width;
    isWorked = false;

    if( area < 1e-9 )
        return 0;

    double mass = edgeMassCalculator.calcMass(p0, p1, width);

    if(ip.gm.hasGrid(MassFactorMapName)){
        bool errorTmp;
        double value;
        std::vector<gridmap::GridmapLayout::GridCellOverlap> cellsInfo;

        ip.gm.getCellsInfoUnderLine(MassFactorMapName, p0, p1, width, ip.precision_massMap, cellsInfo);

        value = ip.gm.getGrid(MassFactorMapName)->getCellsComputedValue( cellsInfo,
                                                                         ArolibGrid_t::AVERAGE_TOTAL,
                                                                         area,
                                                                         false,
                                                                         &errorTmp );
        if(!errorTmp)
            mass *= value;
    }

    Polygon segmentPoly = geometry::createRectangleFromLine(p0, p1, width);

    if(ip.boundary){
        auto intersections = geometry::get_intersection(*ip.boundary, segmentPoly);
        double area_intersection = 0;
        for(auto& intersection : intersections)
            area_intersection += geometry::calc_area(intersection);
        mass *= std::min(1.0, std::max(0.0, area_intersection / area));
    }

    for(auto& workedPoly : ip.workedPolys){
        auto intersections = geometry::get_intersection(workedPoly.outer, segmentPoly);
        double area_intersection = 0;
        for(auto& intersection : intersections)
            area_intersection += geometry::calc_area(intersection);
        for(auto& hole : workedPoly.holes){
            intersections = geometry::get_intersection(hole, segmentPoly);
            for(auto& intersection : intersections)
                area_intersection -= geometry::calc_area(intersection);
        }
        mass *= std::min(1.0, std::max(0.0, (area - area_intersection) / area));
    }

    return mass;

}

void HeadlandBaseRoutesPlanner::updateWorkedAreas(const std::vector<Point> &points,
                                                  double width,
                                                  InternalParametersGeneral &ip)
{
    ip.workedPolys.push_back(PolygonWithHoles());
    if(!geometry::offsetLinestring(points, ip.workedPolys.back(), 0.5*width, 0.5*width, true, 0)){
        ip.workedPolys.pop_back();
    }
}

std::vector<Point> HeadlandBaseRoutesPlanner::getTrackSegment_completeHL(const Track &track,
                                                                         const InternalParametersForCompleteHL & ip,
                                                                         size_t from, int to,
                                                                         bool closedTrack,
                                                                         bool &inReverse)
{
    if(from >= track.points.size())
        return {};
    if(to < 0)
        to = track.points.size() + ( closedTrack ? from : -1 );

    bool trackIsClockwise = geometry::isPolygonClockwise(track.points, true);
    if(ip.clockwise == trackIsClockwise){
        inReverse = false;
        return geometry::getGeometryPart(track.points, from, to, false, closedTrack);
    }
    inReverse = true;
    return geometry::getGeometryPart(track.points, to, from, true, closedTrack);
}

std::vector<Point> HeadlandBaseRoutesPlanner::getTrackSegment_partialHL(const Track &track,
                                                                        const InternalParametersForPartialHLs & ip,
                                                                        size_t from, size_t to,
                                                                        bool &inReverse)
{
    inReverse = false;
    if(from >= track.points.size())
        return {};
    if(from > to){
        std::swap(from, to);
        inReverse = true;
    }
    return geometry::getGeometryPart(track.points, from, to, inReverse, false);
}

std::vector<Point> HeadlandBaseRoutesPlanner::getUnworkedTrackSegment_completeHL(const Subfield &subfield, const InternalParametersForCompleteHL &ip, bool& inReverse)
{
    auto& tracks = subfield.headlands.complete.tracks;
    return getTrackSegment_completeHL( tracks.at(ip.indFirstWorkingTrack),
                                       ip, ip.indFirstWorkingTrackPoint, ip.indFirstWorkingTrackFirstPoint, true, inReverse );
}

std::vector<size_t> HeadlandBaseRoutesPlanner::getIndexesOfPartiallyWorkedHeadlands(InternalParametersForPartialHLs &ip)
{
    const bool restrictToDirection = true;
    std::vector<size_t> ret;
    if(ip.gm.hasGrid(RemainingAreaMapName)){//check if there are partially worked headlands
        for(auto ind : ip.sortedPartialHLs){
            auto& hlParams = ip.hlParams.at(ind);
            if( hlParams.state == InternalParametersForPartialHLs::PHL_WORKED
                    || hlParams.state == InternalParametersForPartialHLs::PHL_NOT_WORKED )
                continue;
            if(restrictToDirection){
                auto& state = ( hlParams.startFromOutermostTrack == hlParams.tracksInwards ? hlParams.trackStates.front() : hlParams.trackStates.back() );
                if(state != InternalParametersForPartialHLs::TRACK_WORKED)
                    ret.push_back(ind);
                continue;
            }
            else if( hlParams.trackStates.front() != InternalParametersForPartialHLs::TRACK_WORKED
                        || hlParams.trackStates.back() != InternalParametersForPartialHLs::TRACK_WORKED )
                ret.push_back(ind);
        }
    }
    return ret;
}

size_t HeadlandBaseRoutesPlanner::getNextHLIndexClosestToPoint(const Subfield &subfield, size_t indHL, const Point &pt, const InternalParametersForPartialHLs &ip)
{
    auto& hls = subfield.headlands.partial;
    size_t indNext = getHeadlandIndexFromSortedList(indHL, 1, ip);
    size_t indPrev = getHeadlandIndexFromSortedList(indHL, -1, ip);

    double dNext = geometry::calc_dist_to_linestring(hls.at(indNext).boundary.points, pt, false);
    double dPrev = geometry::calc_dist_to_linestring(hls.at(indPrev).boundary.points, pt, false);

    return ( dNext < dPrev ? indNext : indPrev );
}

double HeadlandBaseRoutesPlanner::getTransitSpeed(const Machine &machine, double bunkerMass)
{
    double speed_transit = machine.calcSpeed(bunkerMass);
    return (speed_transit > 1e-5 ? speed_transit : 0.5);//default speed if no info is given in the machine (shouldn't happen)

}



}

