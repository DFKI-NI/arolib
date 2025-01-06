/*
 * Copyright 2021-2025 DFKI GmbH
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

#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/curves_helper.hpp"
#include "arolib/planning/track_sequencing/simpletracksequencer.hpp"
#include "arolib/planning/track_connectors/infieldtracksconnectordef.hpp"
#include "arolib/planning/simpleBaseRoutesPlanner.hpp"

namespace arolib {


const double InfieldBaseRoutesPlanner::m_unsamplingTolerance = 0.1;
//const double InfieldBaseRoutesPlanner::m_thresholdIsWorkedLB = 0.4;
//const double InfieldBaseRoutesPlanner::m_thresholdIsWorkedUB = 0.6;
const double InfieldBaseRoutesPlanner::m_thresholdIsWorkedLB = 0.3; //0.4;
const double InfieldBaseRoutesPlanner::m_thresholdIsWorkedUB = 0.7; //0.6;

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
                                       const std::vector<Machine> &_workinggroup,
                                       const PlannerParameters &plannerParameters,
                                       std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                       std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                       std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorTransit,
                                       std::vector<Route> &routes,
                                       const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                       const std::map<MachineId_t, Pose2D> *_initRefPoses,
                                       std::shared_ptr<const ArolibGrid_t> massFactorMap,
                                       std::shared_ptr<const ArolibGrid_t> remainingAreaMap)
{

    if(!edgeMassCalculator)
        return AroResp::LoggingResp(1, "A mass calculator must be given", m_logger, LogLevel::ERROR, __FUNCTION__);

    std::vector<Machine> workinggroup;
    for(auto &m : _workinggroup){
        if (m.isOfWorkingType(true))
            workinggroup.emplace_back(m);
    }

    if(workinggroup.empty())
        return AroResp::LoggingResp(1, "No valid machines given", m_logger, LogLevel::ERROR, __FUNCTION__);

    LoggingComponent::LoggersHandler lh(true);//will be reset on destruction

    if(remainingAreaMap)
        LoggingComponent::setTemporalLoggersParent(lh, *this, *remainingAreaMap);

    std::map<MachineId_t, Pose2D> initRefPoses;
    if(_initRefPoses){
        for(auto& m : workinggroup){
            auto it_m = _initRefPoses->find( m.id );
            if(it_m != _initRefPoses->end() && it_m->second.isValid())
                initRefPoses[m.id] = it_m->second;
        }
    }

    std::shared_ptr<InternalMassCalculator> edgeMassCalculatorExtended = std::make_shared<InternalMassCalculator>( edgeMassCalculator,
                                                                                                                   massFactorMap,
                                                                                                                   m_cim,
                                                                                                                   plannerParameters.bePreciseWithMatterMap ? gridmap::SharedGridsManager::PRECISE : gridmap::SharedGridsManager::PRECISE_ONLY_IF_AVAILABLE );


    WorkedAreaAnalyst waa;
    waa.setWorkedAreaMap(remainingAreaMap, false);
    waa.setThresholds( m_thresholdIsWorkedLB, m_thresholdIsWorkedUB );

    TracksInfo tracksInfo = initTracksInfo(subfield, waa, plannerParameters.bePreciseWithRemainingAreaMap);

    bool allAssigned = initRefPoses.size() == workinggroup.size();

    auto updateRefPosesFromFirstTracks = [& subfield, &tracksInfo, &workinggroup, &initRefPoses](){
        for(auto& it_m1 : tracksInfo.indFirstTrack){
            auto it_m2 = initRefPoses.find(it_m1.first);
            if(it_m2 != initRefPoses.end()) //already set
                continue;

            auto& trackPts = subfield.tracks.at(it_m1.second).points;
            if( tracksInfo.tracksInfo.at(it_m1.second).workingDirection > 0 )
                initRefPoses[it_m1.first] = Pose2D(trackPts.front(), geometry::get_angle(trackPts.front(), trackPts.at(1)));
            else
                initRefPoses[it_m1.first] = Pose2D(trackPts.back(), geometry::get_angle(trackPts.back(), r_at(trackPts, 1)));
        }
    };

    if(!allAssigned){
        allAssigned = updateFirstTrackInfoFromMachinesNearPartiallyWorkedTracks(subfield, tracksInfo, workinggroup,
                                                                                machineCurrentStates ? *machineCurrentStates : std::map<MachineId_t, MachineDynamicInfo>{},
                                                                                initRefPoses);
        updateRefPosesFromFirstTracks();
    }

    if(!allAssigned)
        allAssigned = completeInitRefPosesFromMachinesLocations(subfield, tracksInfo, workinggroup,
                                                                machineCurrentStates ? *machineCurrentStates : std::map<MachineId_t, MachineDynamicInfo>{},
                                                                initRefPoses);

    if(!allAssigned){
        allAssigned = updateFirstTrackFromPartiallyWorkedTracks(subfield, tracksInfo, workinggroup, initRefPoses);
        updateRefPosesFromFirstTracks();
    }

    auto aroResp = generateBaseRoutes(subfield,
                                      tracksInfo.excludeTrackIndexes,
                                      workinggroup,
                                      plannerParameters,
                                      initRefPoses,
                                      edgeMassCalculatorExtended,
                                      edgeSpeedCalculator,
                                      edgeSpeedCalculatorTransit,
                                      routes);

    if(!aroResp.isError())
        aroResp = adjustBaseRoutes(routes,
                                   subfield,
                                   workinggroup,
                                   edgeMassCalculatorExtended,
                                   waa,
                                   plannerParameters);

    return aroResp;

}

AroResp InfieldBaseRoutesPlanner::plan(const Subfield &subfield,
                                       const std::vector<Machine> &workinggroup,
                                       const PlannerParameters &plannerParameters,
                                       std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                       std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                       std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorTransit,
                                       std::vector<Route> &routes,
                                       const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                       const Pose2D &initRefPose,
                                       std::shared_ptr<const ArolibGrid_t> massFactorMap,
                                       std::shared_ptr<const ArolibGrid_t> remainingAreaMap)
{
    std::map<MachineId_t, Pose2D> initRefPoses;
    std::map<MachineId_t, Pose2D>* pInitRefPoses = nullptr;

    if(initRefPose.isValid()){
        for(auto& m : workinggroup)
            initRefPoses[m.id] = initRefPose;
        pInitRefPoses = &initRefPoses;
    }

    return plan(subfield,
                workinggroup,
                plannerParameters,
                edgeMassCalculator,
                edgeSpeedCalculator,
                edgeSpeedCalculatorTransit,
                routes,
                machineCurrentStates,
                pInitRefPoses,
                massFactorMap,
                remainingAreaMap);
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

InfieldBaseRoutesPlanner::TracksInfo InfieldBaseRoutesPlanner::initTracksInfo(const Subfield &subfield, WorkedAreaAnalyst& waa, bool bePrecise)
{
    TracksInfo info(subfield.tracks.size());

    auto remainingArea_map = waa.getWorkedAreaMap();
    if(!remainingArea_map || !remainingArea_map->isAllocated())
        return info;

    const Polygon* boundary = subfield.boundary_inner.points.size() > 3 ? &subfield.boundary_inner : &subfield.boundary_outer;

    for(size_t i = 0 ; i < subfield.tracks.size() ; ++i){

        const auto& track = subfield.tracks.at(i);
        if(track.points.size() < 2){
            info.tracksInfo.at(i).workedState = TRACK_WORKED;
            info.excludeTrackIndexes.insert(i);
            continue;
        }

        double resolution = geometry::calc_dist(track.points.front(), track.points.at(1));
        for(size_t j = 1 ; j+1 < track.points.size() ; ++j)
            resolution = std::max(resolution, geometry::calc_dist(track.points.at(j), track.points.at(j+1)) );

        //get the indexes of the first worked and not-worked segment in forward and reverse order
        int indFirstSegmentWorked = -1, indLastSegmentWorked = -1;
        int indFirstSegmentNotWorked = -1, indLastSegmentNotWorked = -1;
        std::vector<std::pair<int, float>> workedPairs(track.points.size()-1);
        float unknownValuesSum = 0;
        size_t unknownValuesCount = 0;
        for(int side = 0 ; side < 2 ; side ++){

            if(side > 0){
                if( (indFirstSegmentWorked < 0 && indFirstSegmentNotWorked < 0)
                        /*|| (indFirstSegmentWorked >= 0 && indFirstSegmentNotWorked < 0)
                        || (indFirstSegmentNotWorked >= 0 && indFirstSegmentWorked < 0)*/ )
                    break;
            }

            int &indSegmentWorked = ( side == 0 ? indFirstSegmentWorked : indLastSegmentWorked );
            int &indSegmentNotWorked = ( side == 0 ? indFirstSegmentNotWorked : indLastSegmentNotWorked );
            for(size_t j = 0 ; j+1 < track.points.size() ; ++j){
                size_t indP0 = ( side == 0 ? j : track.points.size()-j-1 );
                size_t indP1 = ( side == 0 ? j+1 : track.points.size()-j-2 );

                const auto& p0 = track.points.at(indP0);
                const auto& p1 =  track.points.at(indP1);

                auto workedPair = isSegmentWorked(*boundary, p0, p1, track.width, waa, bePrecise);
                auto& workedState = workedPair.first;
                if(workedState == WorkedAreaAnalyst::NOT_WORKED && indSegmentNotWorked < 0)
                    indSegmentNotWorked = indP0;
                else if(workedState == WorkedAreaAnalyst::WORKED && indSegmentWorked < 0)
                    indSegmentWorked = indP0;
                else if(side == 0 && workedState == WorkedAreaAnalyst::UNKNOWN && !std::isnan(workedPair.second)){
                    unknownValuesSum += std::min(1.0f, std::max(0.0f, workedPair.second));
                    ++unknownValuesCount;
                }

                if(side == 0){
                    workedPairs.at(j) = workedPair;
                }

                if(indSegmentNotWorked >= 0 && indSegmentWorked >= 0)
                    break;
            }
        }

        auto getWorkedStateFromUnknowns = [&workedPairs](size_t indFrom, size_t indTo) -> WorkedAreaAnalyst::WorkedState{
            size_t count = 0;
            float sum = 0;
            if(indFrom <= indTo){
                for(size_t i = indFrom; i < indTo; ++i){
                    auto& workedPair = workedPairs.at(i);
                    if( workedPair.first == WorkedAreaAnalyst::UNKNOWN && !std::isnan( workedPair.second ) ){
                        sum += workedPairs.at(i).second;
                        ++count;
                    }
                }
            }
            else{
                for(size_t i = indFrom; i > indTo; --i){
                    auto& workedPair = workedPairs.at(i-1);
                    if( workedPair.first == WorkedAreaAnalyst::UNKNOWN && !std::isnan( workedPair.second ) ){
                        sum += workedPairs.at(i-1).second;
                        ++count;
                    }
                }
            }
            if(count == 0)
                return WorkedAreaAnalyst::UNKNOWN;
            float avg = sum / count;
            return ( avg > 0.5 ? WorkedAreaAnalyst::WORKED : WorkedAreaAnalyst::NOT_WORKED );
        };

        if(indFirstSegmentWorked < 0 && indFirstSegmentNotWorked < 0){
            if(unknownValuesCount == 0 || unknownValuesSum / unknownValuesCount > 0.5)
                info.tracksInfo.at(i).workedState = TRACK_NOT_WORKED;
            else
                info.tracksInfo.at(i).workedState = TRACK_WORKED;
            continue;
        }
        else if(indFirstSegmentWorked >= 0 && indFirstSegmentNotWorked < 0){

            WorkedAreaAnalyst::WorkedState workedBefore = WorkedAreaAnalyst::WORKED, workedAfter = WorkedAreaAnalyst::WORKED;

            if(indLastSegmentWorked < 0)
                indLastSegmentWorked = indLastSegmentWorked+1;

            if(indFirstSegmentWorked > 0)
                workedBefore = getWorkedStateFromUnknowns(0, indFirstSegmentWorked);
            if(indFirstSegmentWorked >= indLastSegmentWorked-1){
                if(indFirstSegmentWorked+2 < track.points.size())
                    workedAfter = getWorkedStateFromUnknowns(track.points.size()-1, indFirstSegmentWorked+1);
            }
            else{
                if(indLastSegmentWorked+1 < track.points.size())
                    workedAfter = getWorkedStateFromUnknowns(track.points.size()-1, indLastSegmentWorked);
            }

            //default for unknown: worked
            if(workedBefore == WorkedAreaAnalyst::UNKNOWN)
                workedBefore = WorkedAreaAnalyst::WORKED;
            if(workedAfter == WorkedAreaAnalyst::UNKNOWN)
                workedAfter = WorkedAreaAnalyst::WORKED;

            if(workedBefore == WorkedAreaAnalyst::WORKED && workedAfter == WorkedAreaAnalyst::WORKED){
                info.tracksInfo.at(i).workedState = TRACK_WORKED;
                info.excludeTrackIndexes.insert(i);
                continue;
            }
            else if(workedBefore == WorkedAreaAnalyst::WORKED && workedAfter == WorkedAreaAnalyst::NOT_WORKED){// it is partially worked from the track start
                info.tracksInfo.at(i).workedState = TRACK_PARTIALLY_WORKED;
                info.tracksInfo.at(i).workingDirection = 1;
                info.tracksInfo.at(i).indFirstWorkingPointFwd = indLastSegmentWorked;
                info.partiallyWorkedTrackIndexes.insert(i);
                continue;
            }
            else if(workedBefore == WorkedAreaAnalyst::NOT_WORKED && workedAfter == WorkedAreaAnalyst::WORKED){// it is partially worked from the track end
                info.tracksInfo.at(i).workedState = TRACK_PARTIALLY_WORKED;
                info.tracksInfo.at(i).workingDirection = -1;
                info.tracksInfo.at(i).indFirstWorkingPointRev = indFirstSegmentWorked;
                info.partiallyWorkedTrackIndexes.insert(i);
                continue;
            }
            else{
                info.tracksInfo.at(i).workedState = TRACK_PARTIALLY_WORKED; // it is partially worked from both sides
                info.partiallyWorkedTrackIndexes.insert(i);
                info.tracksInfo.at(i).indFirstWorkingPointFwd = 0;
                info.tracksInfo.at(i).indFirstWorkingPointRev = track.points.size() - 1;
                if( geometry::getGeometryLength(track.points, 0, indFirstSegmentWorked) > geometry::getGeometryLength(track.points, indLastSegmentWorked) )
                    info.tracksInfo.at(i).workingDirection = 2;
                else
                    info.tracksInfo.at(i).workingDirection = -2;
                continue;
            }

        }
        else if(indFirstSegmentNotWorked >= 0 && indFirstSegmentWorked < 0){

            WorkedAreaAnalyst::WorkedState workedBefore = WorkedAreaAnalyst::NOT_WORKED, workedAfter = WorkedAreaAnalyst::NOT_WORKED;

            if(indLastSegmentNotWorked < 0)
                indLastSegmentNotWorked = indLastSegmentNotWorked+1;

            if(indFirstSegmentNotWorked > 0)
                workedBefore = getWorkedStateFromUnknowns(0, indFirstSegmentNotWorked);
            if(indFirstSegmentNotWorked >= indLastSegmentNotWorked-1){
                if(indFirstSegmentNotWorked+2 < track.points.size())
                    workedAfter = getWorkedStateFromUnknowns(track.points.size()-1, indFirstSegmentNotWorked+1);
            }
            else{
                if(indLastSegmentNotWorked+1 < track.points.size())
                    workedAfter = getWorkedStateFromUnknowns(track.points.size()-1, indLastSegmentNotWorked);
            }

            //default for unknown: NOT worked
            if(workedBefore == WorkedAreaAnalyst::UNKNOWN)
                workedBefore = WorkedAreaAnalyst::NOT_WORKED;
            if(workedAfter  == WorkedAreaAnalyst::UNKNOWN)
                workedAfter = WorkedAreaAnalyst::NOT_WORKED;

            if(workedBefore == 0 && workedAfter == 0){
                info.tracksInfo.at(i).workedState = TRACK_NOT_WORKED;
                continue;
            }
            else if(workedBefore == WorkedAreaAnalyst::WORKED && workedAfter == WorkedAreaAnalyst::NOT_WORKED){// it is partially worked from the track start
                info.tracksInfo.at(i).workedState = TRACK_PARTIALLY_WORKED;
                info.tracksInfo.at(i).workingDirection = 1;
                info.tracksInfo.at(i).indFirstWorkingPointFwd = indFirstSegmentNotWorked;
                info.partiallyWorkedTrackIndexes.insert(i);
                continue;
            }
            else if(workedBefore == WorkedAreaAnalyst::NOT_WORKED && workedAfter == WorkedAreaAnalyst::WORKED){// it is partially worked from the track end
                info.tracksInfo.at(i).workedState = TRACK_PARTIALLY_WORKED;
                info.tracksInfo.at(i).workingDirection = -1;
                info.tracksInfo.at(i).indFirstWorkingPointRev = indLastSegmentNotWorked;
                info.partiallyWorkedTrackIndexes.insert(i);
                continue;
            }
            else{
                info.tracksInfo.at(i).workedState = TRACK_PARTIALLY_WORKED; // it is partially worked from both sides
                info.partiallyWorkedTrackIndexes.insert(i);
                info.tracksInfo.at(i).indFirstWorkingPointFwd = indFirstSegmentNotWorked;
                info.tracksInfo.at(i).indFirstWorkingPointRev = indLastSegmentNotWorked;
                if( geometry::getGeometryLength(track.points, 0, indFirstSegmentNotWorked) > geometry::getGeometryLength(track.points, indLastSegmentNotWorked) )
                    info.tracksInfo.at(i).workingDirection = 2;
                else
                    info.tracksInfo.at(i).workingDirection = -2;
                continue;
            }
        }
        else if(indFirstSegmentNotWorked < indFirstSegmentWorked && indLastSegmentNotWorked > indLastSegmentWorked ){
            info.tracksInfo.at(i).workedState = TRACK_NOT_WORKED;
            continue;
        }
        else if(indFirstSegmentNotWorked > indFirstSegmentWorked && indLastSegmentNotWorked > indLastSegmentWorked ){ // it is partially worked from the track start
            info.tracksInfo.at(i).workedState = TRACK_PARTIALLY_WORKED;
            info.tracksInfo.at(i).workingDirection = 1;
            info.tracksInfo.at(i).indFirstWorkingPointFwd = indFirstSegmentNotWorked;
            info.partiallyWorkedTrackIndexes.insert(i);
            continue;
        }
        else if(indFirstSegmentNotWorked < indFirstSegmentWorked && indLastSegmentNotWorked < indLastSegmentWorked ){ // it is partially worked from the track end
            info.tracksInfo.at(i).workedState = TRACK_PARTIALLY_WORKED;
            info.tracksInfo.at(i).workingDirection = -1;
            info.tracksInfo.at(i).indFirstWorkingPointRev = indLastSegmentNotWorked;
            info.partiallyWorkedTrackIndexes.insert(i);
            continue;
        }
        else{
            info.tracksInfo.at(i).workedState = TRACK_PARTIALLY_WORKED; // it is partially worked from both sides
            info.partiallyWorkedTrackIndexes.insert(i);
            info.tracksInfo.at(i).indFirstWorkingPointFwd = indFirstSegmentNotWorked;
            info.tracksInfo.at(i).indFirstWorkingPointRev = indLastSegmentNotWorked;
            if( geometry::getGeometryLength(track.points, 0, indFirstSegmentNotWorked) > geometry::getGeometryLength(track.points, indLastSegmentNotWorked) )
                info.tracksInfo.at(i).workingDirection = 2;
            else
                info.tracksInfo.at(i).workingDirection = -2;
        }
    }

    return info;
}

bool InfieldBaseRoutesPlanner::updateFirstTrackFromPartiallyWorkedTracks(const Subfield &subfield, TracksInfo &tracksInfo, const std::vector<Machine> workinggroup, const std::map<MachineId_t, Pose2D> &initRefPoses)
{
    if(tracksInfo.partiallyWorkedTrackIndexes.empty() || workinggroup.empty())
        return false;

    std::set<size_t> assignedTracks;
    for(auto& it_t : tracksInfo.indFirstTrack)
        assignedTracks.insert(it_t.second);

    std::multimap<double, size_t, std::greater<double>> lengthsMap;
    std::multimap<double, size_t> tracksWithoutDir;

    for(auto ind : tracksInfo.partiallyWorkedTrackIndexes){
        if(assignedTracks.find(ind) != assignedTracks.end())
            continue;

        auto& info = tracksInfo.tracksInfo.at(ind);
        double length = -1;
        if(info.indFirstWorkingPointFwd >= 0)
            length = std::max(length, geometry::getGeometryLength(subfield.tracks.at(ind).points, 0, info.indFirstWorkingPointFwd));
        if(info.indFirstWorkingPointRev >= 0)
            length = std::max(length, geometry::getGeometryLength(subfield.tracks.at(ind).points, info.indFirstWorkingPointRev, -1));
        if(length < -1e-6){
            tracksWithoutDir.insert( std::make_pair(geometry::getGeometryLength(subfield.tracks.at(ind).points), ind) );
            continue;
        }
        lengthsMap.insert( std::make_pair(length, ind) );
    }

    for(auto& m : workinggroup){
        if( initRefPoses.find(m.id) == initRefPoses.end() )
            continue;
        if(!lengthsMap.empty()){
            tracksInfo.indFirstTrack[m.id] = lengthsMap.begin()->second;
            lengthsMap.erase( lengthsMap.begin() );
            continue;
        }
        if(!tracksWithoutDir.empty()){
            tracksInfo.indFirstTrack[m.id] = tracksWithoutDir.begin()->second;
            tracksWithoutDir.erase( tracksWithoutDir.begin() );
            continue;
        }
        return false;
    }

    return true;
}



bool InfieldBaseRoutesPlanner::updateFirstTrackInfoFromMachinesNearPartiallyWorkedTracks(const Subfield &subfield,
                                                                                         TracksInfo &tracksInfo,
                                                                                         const std::vector<Machine> workinggroup,
                                                                                         const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                                                         const std::map<MachineId_t, Pose2D> &initRefPoses)
{

    std::set<size_t> assignedTracks;
    for(auto& it_t : tracksInfo.indFirstTrack)
        assignedTracks.insert(it_t.second);

    std::map<size_t, std::map<int, std::vector<Point>>> trackWorkedSegments;
    for(auto trackInd : tracksInfo.partiallyWorkedTrackIndexes){
        if( assignedTracks.find(trackInd) != assignedTracks.end() )
            continue;

        auto& track = subfield.tracks.at(trackInd);
        auto& info = tracksInfo.tracksInfo.at(trackInd);
        if(info.indFirstWorkingPointFwd >= 0){
            auto& seg = trackWorkedSegments[trackInd][1];
            seg.insert(seg.end(), track.points.begin(), track.points.begin()+info.indFirstWorkingPointFwd+1);
        }
        if(info.indFirstWorkingPointRev >= 0){
            auto& seg = trackWorkedSegments[trackInd][-1];
            size_t revInd = track.points.size() - 1 - info.indFirstWorkingPointRev;
            seg.insert(seg.end(), track.points.rbegin(), track.points.rbegin()+revInd+1);
        }
    }

    std::multimap<double, MachineId_t> machineMinDistances;
    std::map<MachineId_t, std::multimap<double, std::pair<size_t, int>>> machineDistances; // < machineId, < distance, < trackInd, direction > > >
    std::set<MachineId_t> machinesToAssign;

    for(auto& m : workinggroup){
        if( tracksInfo.indFirstTrack.find(m.id) != tracksInfo.indFirstTrack.end() )
            continue;

        auto it_mdi = machineCurrentStates.find(m.id);
        auto it_m = initRefPoses.find(m.id);

        if(it_m == initRefPoses.end())
            machinesToAssign.insert(m.id);

        if(it_mdi == machineCurrentStates.end() && it_m == initRefPoses.end())
            continue;

        const Point* posRef = it_m != initRefPoses.end() ? &(it_m->second) : nullptr;
        const Point* pos = it_mdi != machineCurrentStates.end() ? &(it_mdi->second.position) : nullptr;

        auto& distances = machineDistances[m.id];

        for(auto& track_it : trackWorkedSegments){
            auto& track = subfield.tracks.at(track_it.first);
            for(auto& seg_it : track_it.second){
                auto& seg = seg_it.second;
                double distToWorkedSegment = std::numeric_limits<double>::max(); //distance to the worked segment (to check if a point is close enough to the worked part of the track)
                if(pos)
                    distToWorkedSegment = geometry::calc_dist_to_linestring(seg, *pos);
                if(posRef)
                    distToWorkedSegment = std::min(distToWorkedSegment, geometry::calc_dist_to_linestring(seg, *posRef));
                if(distToWorkedSegment > 0.6 * track.width)
                    continue;
                double distToFirstWorkingPoint = std::numeric_limits<double>::max(); //distance to first working point (i.e., last point of the worked segment - seg.back())
                if(pos)
                    distToFirstWorkingPoint = geometry::calc_dist(seg.back(), *pos);
                if(posRef)
                    distToFirstWorkingPoint = std::min(distToFirstWorkingPoint, geometry::calc_dist(seg.back(), *posRef));
                if(distances.size() < machineCurrentStates.size() || distToFirstWorkingPoint <= distances.rbegin()->first)
                    distances.insert( std::make_pair(distToFirstWorkingPoint, std::make_pair(track_it.first, seg_it.first)) );
            }
        }

        if(!distances.empty())
            machineMinDistances.insert( std::make_pair(distances.begin()->first, m.id ) );
    }

    while(!machineMinDistances.empty()){
        auto machineId = machineMinDistances.begin()->second;
        auto& distances = machineDistances.at(machineId);
        auto trackInd = distances.begin()->second.first;
        if(assignedTracks.find(trackInd) != assignedTracks.end()){
            machineMinDistances.erase( machineMinDistances.begin() );
            distances.erase( distances.begin() );
            if(!distances.empty())
                machineMinDistances.insert( std::make_pair(distances.begin()->first, machineId ) );
            continue;
        }

        tracksInfo.indFirstTrack[machineId] = trackInd;
        tracksInfo.tracksInfo.at(trackInd).workingDirection = distances.begin()->second.second;
        assignedTracks.insert(trackInd);
    }

    for(auto mid : machinesToAssign){
        if(tracksInfo.indFirstTrack.find(mid) == tracksInfo.indFirstTrack.end())
            return false;
    }

    return true;

}

bool InfieldBaseRoutesPlanner::completeInitRefPosesFromMachinesLocations(const Subfield &subfield,
                                                                         TracksInfo &tracksInfo,
                                                                         const std::vector<Machine> workinggroup,
                                                                         const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                                         std::map<MachineId_t, Pose2D> &initRefPoses)
{
    std::set<size_t> assignedTracks;
    for(auto& it_t : tracksInfo.indFirstTrack)
        assignedTracks.insert(it_t.second);

    Polygon boundary;
    if( !geometry::offsetPolygon(subfield.boundary_outer, boundary, 0.1, true, 0) )
        boundary.points.clear();

    std::multimap<double, MachineId_t> machineMinDistances;
    std::map<MachineId_t, std::multimap<double, std::pair<size_t, Pose2D>>> machineDistances; // < machineId, < distance, < trackInd, pose > > >
    std::set<MachineId_t> machinesToAssign;

    for(auto& m : workinggroup){
        if( tracksInfo.indFirstTrack.find(m.id) != tracksInfo.indFirstTrack.end() )
            continue;

        auto it_mdi = machineCurrentStates.find(m.id);
        auto it_m = initRefPoses.find(m.id);

        if(it_m == initRefPoses.end())
            machinesToAssign.insert(m.id);

        if(it_mdi == machineCurrentStates.end() && it_m == initRefPoses.end())
            continue;

        Pose2D poseRef = ( it_m == initRefPoses.end() ? Pose2D(Point::invalidPoint()) : Pose2D(it_mdi->second.position, it_mdi->second.theta) );
        Pose2D pose = ( it_mdi == machineCurrentStates.end() ? Pose2D(Point::invalidPoint()) : Pose2D(it_mdi->second.position, it_mdi->second.theta) );

        if(!boundary.points.empty() && !geometry::in_polygon(pose, boundary))
            continue;

        double turningRad = std::max(0.0, m.getTurningRadius());
        auto& distances = machineDistances[m.id];

        for(size_t i = 0 ; i < subfield.tracks.size() ; ++i ){
            if(tracksInfo.excludeTrackIndexes.find(i) != tracksInfo.excludeTrackIndexes.end())
                continue;

            if(assignedTracks.find(i) != assignedTracks.end())
                continue;

            auto& trackPts = subfield.tracks.at(i).points;
            Pose2D poseFwd(trackPts.front(), geometry::get_angle(trackPts.front(), trackPts.at(1)));
            Pose2D poseRev(trackPts.back(), geometry::get_angle(trackPts.back(), r_at(trackPts, 1)));

            double distFwd = std::numeric_limits<double>::max();
            double distRev = std::numeric_limits<double>::max();

            if(pose.isValid()){
                auto dTmp = geometry::calcDubinsPathLength(pose, poseFwd, turningRad);
                if(dTmp > -1e-6)
                    distFwd = std::min(distFwd, dTmp);
                dTmp = geometry::calcDubinsPathLength(pose, poseRev, turningRad);
                if(dTmp > -1e-6)
                    distRev = std::min(distRev, dTmp);
            }

            if(poseRef.isValid()){
                auto dTmp = geometry::calcDubinsPathLength(poseRef, poseFwd, turningRad);
                if(dTmp > -1e-6)
                    distFwd = std::min(distFwd, dTmp);
                dTmp = geometry::calcDubinsPathLength(poseRef, poseRev, turningRad);
                if(dTmp > -1e-6)
                    distRev = std::min(distRev, dTmp);
            }

            auto dist = std::min(distFwd, distRev);
            if(dist > 0.6 * subfield.tracks.at(i).width)
                continue;

            if(distances.size() < workinggroup.size() || dist <= distances.rbegin()->first)
                distances.insert( std::make_pair(dist, std::make_pair(i, distFwd > distRev ? poseRev : poseFwd )) );
        }

        if(!distances.empty())
            machineMinDistances.insert( std::make_pair(distances.begin()->first, m.id ) );
    }

    while(!machineMinDistances.empty()){
        auto machineId = machineMinDistances.begin()->second;
        auto& distances = machineDistances.at(machineId);
        auto trackInd = distances.begin()->second.first;
        if(assignedTracks.find(trackInd) != assignedTracks.end()){
            machineMinDistances.erase( machineMinDistances.begin() );
            distances.erase( distances.begin() );
            if(!distances.empty())
                machineMinDistances.insert( std::make_pair(distances.begin()->first, machineId) );
            continue;
        }

        initRefPoses[machineId] = distances.begin()->second.second;
        assignedTracks.insert(trackInd);
    }

    for(auto& m : workinggroup){
        if(initRefPoses.find(m.id) == initRefPoses.end())
            return false;
    }

    return true;
}

std::pair<WorkedAreaAnalyst::WorkedState, float> InfieldBaseRoutesPlanner::isSegmentWorked(const Point &p0, const Point &p1, double width, WorkedAreaAnalyst &waa, bool bePrecise)
{
    auto remainingAreaMap = waa.getWorkedAreaMap();
    if(!remainingAreaMap || !remainingAreaMap->isAllocated())
        return std::make_pair(WorkedAreaAnalyst::NOT_WORKED, 0.0);

    return waa.isSegmentWorked(p0, p1, width, bePrecise ? gridmap::SharedGridsManager::PRECISE : gridmap::SharedGridsManager::PRECISE_ONLY_IF_AVAILABLE);

}

std::pair<WorkedAreaAnalyst::WorkedState, float> InfieldBaseRoutesPlanner::isSegmentWorked(const Polygon &boundary, const Point &p0, const Point &p1, double width, WorkedAreaAnalyst &waa, bool bePrecise)
{
    if(width <= 0)
        return std::make_pair(WorkedAreaAnalyst::UNKNOWN, std::nan("1"));

    auto remainingAreaMap = waa.getWorkedAreaMap();
    if(!remainingAreaMap || !remainingAreaMap->isAllocated())
        return std::make_pair(WorkedAreaAnalyst::NOT_WORKED, 0.0);

    if(boundary.points.empty())
        return isSegmentWorked(p0, p1, width, waa, bePrecise);

    return waa.isSegmentWorked(boundary, p0, p1, width, bePrecise ? gridmap::SharedGridsManager::PRECISE : gridmap::SharedGridsManager::PRECISE_ONLY_IF_AVAILABLE);

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
                                                     const std::map<MachineId_t, Pose2D>& initRefPoses,
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
    sbrp.setExcludeTrackIndexes(excludeTrackIndexes);

    if(excludeTrackIndexes.empty() || !plannerParameters.limitStartToExtremaTracks)
        sbrp.setInfieldTrackSequencerSettings(plannerParameters);
    else{ // one or more tracks are workd -> remove condition to start from extrema tracks
        ITrackSequencer::TrackSequencerSettings tmpSettings = plannerParameters;
        tmpSettings.limitStartToExtremaTracks = false;
        sbrp.setInfieldTrackSequencerSettings(tmpSettings);
    }


    if(!initRefPoses.empty())
        sbrp.setInitRefPoses(initRefPoses);
    else{
        const std::vector<Point>* firstTrack = nullptr;
        for(size_t i = 0 ; i < subfield.tracks.size() ; ++i ){
            size_t track_ind = plannerParameters.inverseTrackOrder ? subfield.tracks.size()-1-i : i;
            if(subfield.tracks.at(track_ind).points.size() > 1
                    && excludeTrackIndexes.find(track_ind) == excludeTrackIndexes.end()){
                firstTrack = &subfield.tracks.at(track_ind).points;
                break;
            }
        }
        if (!firstTrack)
            return AroResp(1, "Error obtaining default first working track");

        Pose2D refPose = plannerParameters.inversePointsOrder ?
                           Pose2D(firstTrack->back(), geometry::get_angle(firstTrack->back(), r_at(*firstTrack, 1)))
                         : Pose2D(firstTrack->front(), geometry::get_angle(firstTrack->front(), firstTrack->at(1)));

        auto initRefPosesTmp = initRefPoses;
        for (auto &m : workinggroup)
            initRefPosesTmp[m.id] = refPose;

        sbrp.setInitRefPoses(initRefPosesTmp);
    }

    for (auto &m : workinggroup){
        aroResp = sbrp.addMachine(m);
        if(aroResp.isError())
            return aroResp;
    }

    for(auto &m : workinggroup){
        logger().printOut(LogLevel::INFO, __FUNCTION__, "Added machine " + std::to_string(m.id) + " to the machine plan");
        routes.push_back(Route());
        aroResp = sbrp.getRoute(m.id, routes.back(), edgeMassCalculator, edgeSpeedCalculator, edgeSpeedCalculatorTransit);
        routes.back().route_id = routes.size()-1;
        if (aroResp.isError())
            return AroResp(1, "Error obtaining machine route: " + aroResp.msg);
    }
    logger().printOut(LogLevel::INFO, __FUNCTION__, std::to_string(routes.size()) + " routes were generated");

    return AroResp(0,"OK");

}

AroResp InfieldBaseRoutesPlanner::adjustBaseRoutes(std::vector<Route> &routes,
                                                   const Subfield &subfield,
                                                   const std::vector<Machine> &workinggroup,
                                                   std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                                   WorkedAreaAnalyst &waa,
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

        auto remainingAreaMap = waa.getWorkedAreaMap();
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
                auto workedState = isSegmentWorked(boundary, p0, p1, it_width->second, waa, plannerParameters.bePreciseWithRemainingAreaMap).first;
                if( workedState == WorkedAreaAnalyst::WORKED )
                    somethingWorked = true;
                if( workedState != WorkedAreaAnalyst::NOT_WORKED)
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

