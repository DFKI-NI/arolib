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
 
#include "arolib/planning/track_sequencing/simulated_annealing/tracksequencersimannealing.hpp"

#include "arolib/misc/randomgeneration.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/field_geometry_processing.hpp"
#include "arolib/planning/track_sequencing/tracksequenceradjacentnext.hpp"
#include "arolib/planning/track_sequencing/tracksequencerclosestnext.hpp"
#include "arolib/planning/track_connectors/infieldtracksconnectordef.hpp"
#include "arolib/planning/track_sequencing/simulated_annealing/stategenerator_swaptracks.hpp"


namespace{

using namespace arolib;

inline Pose2D getEntryExitPose (const Subfield& subfield, size_t trackIndex, bool trackInPointsDirection, bool entry){
    return geometry::getLinestringEntryExitPose(subfield.tracks.at(trackIndex).points, entry, trackInPointsDirection);
}

PointVec getSaveStandardPath(const Pose2D& poseStart, const Pose2D& poseEnd,
                             const ITrackSequencer& sequencer, const TrackSequencerSimAnnealing::ITracksConnector& connector,
                             const Subfield& subfield, const Machine& machine, bool checkBidirectional,
                             TrackSequencerSimAnnealing::SavePathFunction* savePathFct){
    auto path = sequencer.getPathsMapManager()->getPathFromMap(poseStart, poseEnd, machine.turning_radius, checkBidirectional);
    if(path.empty()){
        path = connector.getStandardConnection(poseStart, poseEnd, subfield, machine);
        if(!path.empty() && savePathFct){
            (*savePathFct)(poseStart, poseEnd, machine.turning_radius, path);
        }
    }
    return path;
}

void repairTracksDirections(std::vector<ITrackSequencer::TrackInfo>& sequences, const std::vector<bool>& tracksRelativeDirections,
                            const TrackSequencerSimAnnealing::PlanningParameters& params, const Machine & machine, const Pose2D* initRefPose,
                            TrackSequencerSimAnnealing::SavePathFunction &savePathFct){
    assert(sequences.size() == tracksRelativeDirections.size());

    auto getDirection = [&](size_t ind, size_t indRef) -> ITrackSequencer::TrackPointsDirection{
        auto& tiRef = sequences.at(indRef);
        if(tracksRelativeDirections.at(ind) == tracksRelativeDirections.at(indRef))
            return ( tiRef.trackPointsDirection ==  ITrackSequencer::TrackPointsDirection::FORWARD ? ITrackSequencer::TrackPointsDirection::REVERSE : ITrackSequencer::TrackPointsDirection::FORWARD );
        return tiRef.trackPointsDirection;

    };

    auto repairFromNext = [&](auto&& repairFromNext, size_t ind) -> bool{
        if(ind + 1 >= sequences.size())
            return false;
        auto& ti = sequences.at(ind);
        if(sequences.at(ind+1).trackPointsDirection == ITrackSequencer::TrackPointsDirection::UNDEF){
            if(!repairFromNext(repairFromNext, ind+1))
                return false;
        }
        ti.trackPointsDirection = getDirection(ind, ind+1);
        return true;
    };

    for(size_t i = 0 ; i < sequences.size() ; ++i){
        auto& ti = sequences.at(i);
        if(ti.trackPointsDirection != ITrackSequencer::TrackPointsDirection::UNDEF)
            continue;
        if(repairFromNext(repairFromNext, i))
            break;
        if(i > 0)
            ti.trackPointsDirection = getDirection(i, i-1);
        else if(!initRefPose)
            ti.trackPointsDirection = ITrackSequencer::TrackPointsDirection::FORWARD;
        else{
            Pose2D poseRev = getEntryExitPose(*params.subfield, ti.trackIndex, false, true);
            auto pathRev = getSaveStandardPath(*initRefPose, poseRev, *params.sequencer, *params.connector, *params.subfield, machine, true, &savePathFct);
            if(pathRev.empty())
                ti.trackPointsDirection = ITrackSequencer::TrackPointsDirection::FORWARD;
            else{
                Pose2D poseFwd = getEntryExitPose(*params.subfield, ti.trackIndex, true, true);
                auto pathFwd = getSaveStandardPath(*initRefPose, poseFwd, *params.sequencer, *params.connector, *params.subfield, machine, true, &savePathFct);
                auto connCostFwd = params.connectionCostCalculator ? params.connectionCostCalculator->calc(pathFwd, machine) : geometry::getGeometryLength(pathFwd);
                auto connCostRev = params.connectionCostCalculator ? params.connectionCostCalculator->calc(pathRev, machine) : geometry::getGeometryLength(pathRev);
                ti.trackPointsDirection = ( connCostRev < connCostFwd ? ITrackSequencer::TrackPointsDirection::REVERSE : ITrackSequencer::TrackPointsDirection::FORWARD );
            }

        }
        for(size_t j = i+1 ; j < sequences.size() ; ++j)
            sequences.at(j).trackPointsDirection = getDirection(j, j-1);
        break;
    }
}

bool initStateFromSequence(const TrackSequencerSimAnnealing::PlanningParameters& params,
                           std::map<MachineId_t, std::vector<ITrackSequencer::TrackInfo>>& sequences,
                           TrackSequencerSimAnnealing::State& state,
                           TrackSequencerSimAnnealing::SavePathFunction& savePathFct){

    auto& subfield = *params.subfield;
    auto& machines = *params.machines;
    auto& sequencer = *params.sequencer;
    auto& connector = *params.connector;

    state.clear();

    int indMaxLength = -1;
    double maxLength = -1e9;
    for(auto& it_seqs : sequences){
        for(const ITrackSequencer::TrackInfo &ti : it_seqs.second){
            auto l = geometry::getGeometryLength( subfield.tracks.at(ti.trackIndex).points );
            if(maxLength < l){
                maxLength = l;
                indMaxLength = ti.trackIndex;
            }
        }
    }
    if(indMaxLength < 0)
        return false;


    for(auto& it_seqs : sequences){
        auto& seqs_m = it_seqs.second;
        if(seqs_m.empty())
            continue;

        auto it_m = machines.find(it_seqs.first);
        if(it_m == machines.end())
            return false;
        const Machine& machine = it_m->second;

        auto &state_m = state[it_seqs.first];
        state_m.reserve(seqs_m.size() + 1);

        std::vector<bool> tracksRelativeDirections(seqs_m.size());
        auto& pt0Ref = subfield.tracks.at(indMaxLength).points.front();
        auto& ptnRef = subfield.tracks.at(indMaxLength).points.back();
        tracksRelativeDirections.front() = true;
        for(size_t i = 1 ; i < seqs_m.size() ; ++i){
            ITrackSequencer::TrackInfo &ti = seqs_m.at(i);
            auto& pt0 = subfield.tracks.at(ti.trackIndex).points.front();
            auto& ptn = subfield.tracks.at(ti.trackIndex).points.back();
            auto d0_0 = geometry::calc_dist( pt0Ref, pt0 );
            auto dn_0 = geometry::calc_dist( ptnRef, pt0 );
            auto d0_n = geometry::calc_dist( pt0Ref, ptn );
            auto dn_n = geometry::calc_dist( ptnRef, ptn );
            tracksRelativeDirections.at(i) = ( (d0_0 + dn_n) <= (d0_n + dn_0) );
        }

        const Pose2D* initRefPose = nullptr;
        {
            auto it_m = params.initRefPoses->find(machine.id);
            if(it_m != params.initRefPoses->end() && it_m->second.isValid())
                initRefPose = &( it_m->second );
        }

        repairTracksDirections(seqs_m, tracksRelativeDirections, params, machine, initRefPose, savePathFct );

        {
            ITrackSequencer::TrackInfo& ti = seqs_m.front();
            state_m.emplace_back( TrackSequencerSimAnnealing::StateTrackInfo() );
            TrackSequencerSimAnnealing::StateTrackInfo& si = state_m.back();
            si.trackInd = -1;
            si.trackInPointsDirection = ti.trackPointsDirection == ITrackSequencer::TrackPointsDirection::REVERSE;
            si.connPoseEnd = getEntryExitPose(subfield, ti.trackIndex, ti.trackPointsDirection == ITrackSequencer::TrackPointsDirection::FORWARD, true);
            si.trackCost = 0;
            si.connectionCost = 0;
            if(initRefPose)
                si.connPoseStart = *initRefPose;
        }


        for(size_t i = 0 ; i < seqs_m.size() ; ++i){

            ITrackSequencer::TrackInfo &ti = seqs_m.at(i);

            state_m.emplace_back( TrackSequencerSimAnnealing::StateTrackInfo() );
            auto& si = state_m.back();

            si.trackInd = ti.trackIndex;

            si.trackInPointsDirection = ti.trackPointsDirection == ITrackSequencer::TrackPointsDirection::FORWARD;
            si.trackInRelativeDirection = tracksRelativeDirections.at(i);

            si.connPoseStart = getEntryExitPose(subfield, ti.trackIndex, ti.trackPointsDirection == ITrackSequencer::TrackPointsDirection::FORWARD, false);
            si.connectionCost = 0;
            if(i+1 < seqs_m.size()){
                const ITrackSequencer::TrackInfo &tiNext = seqs_m.at(i+1);
                si.connPoseEnd = getEntryExitPose(subfield, tiNext.trackIndex, tiNext.trackPointsDirection == ITrackSequencer::TrackPointsDirection::FORWARD, true);
            }

            if( !params.stateCostCalculator && params.trackCostCalculator)
                si.trackCost = params.trackCostCalculator->calc(subfield.tracks.at(si.trackInd), si.trackInPointsDirection, machine);
        }

    }

    auto getPathFromMap = [&sequencer](const Pose2D& pose1, const Pose2D& pose2, const Machine& machine, bool checkBidirectional){
        return sequencer.getPathsMapManager()->getPathFromMap(pose1, pose2, machine, checkBidirectional);
    };

    //update connection paths and costs
    if( ( params.stateCostCalculator && params.stateCostCalculator->dependsOnConnectingPaths() )
            || ( params.connectionCostCalculator && params.connectionCostCalculator->dependsOnPath() )){
        for(auto& it_m : state){
            const Machine& machine = machines.at(it_m.first);
            std::vector<TrackSequencerSimAnnealing::StateTrackInfo>& state_m = it_m.second;

            std::vector< TrackSequencerSimAnnealing::ITracksConnector::ConnectionInfo > connectionsInfo;
            connectionsInfo.reserve( state_m.size()-1 );
            size_t startInd = state_m.front().connPoseStart.isValid() ? 0 : 1;
            for(size_t i = startInd ; i+1 < state_m.size() ; ++i) // the fieldExit connection (last track state) is computed afterwards if needed
                connectionsInfo.emplace_back( TrackSequencerSimAnnealing::ITracksConnector::ConnectionInfo(machine, i) );
            std::vector<TrackSequencerSimAnnealing::ITracksConnector::Connection> connections = connector.getTrackStateConnections( state,
                                                                                                                                    nullptr,
                                                                                                                                    subfield,
                                                                                                                                    connectionsInfo,
                                                                                                                                    getPathFromMap );
            if(connections.size() != connectionsInfo.size())
                return false;

            bool gotFieldExitPath = false;
            for(size_t i = 0 ; i < connectionsInfo.size() ; ++i){
                auto& ci = connectionsInfo.at(i);
                auto& conn = connections.at(i);
                auto& si = state_m.at(ci.trackStateInd);
                si.connectionPath = std::move(conn.path);
                if(conn.savePath)
                    savePathFct(si.connPoseStart, si.connPoseEnd, machine.turning_radius, si.connectionPath);
                if(ci.trackStateInd == state_m.size()-1 && !si.connectionPath.empty()) // at the moment redundant because the connectionInfo of the last track state is not added
                    gotFieldExitPath = true;
            }

            //if( !params.stateCostCalculator ){
            if( params.connectionCostCalculator ){
                for(size_t i = 0 ; i+1 < state_m.size() ; ++i) // the cost of the fieldExit connection (last track state) is computed afterwards if needed
                    state_m.at(i).connectionCost = params.connectionCostCalculator->calc(state_m.at(i).connectionPath, machine);
            }

            if(params.settings->considerFieldExit && !gotFieldExitPath){

                std::vector<TrackSequencerSimAnnealing::ITracksConnector::ExitConnection> exitConnections = connector.getExitConnections( state,
                                                                                                                                          nullptr,
                                                                                                                                          subfield,
                                                                                                                                          machine,
                                                                                                                                          params.fieldExitPoses,
                                                                                                                                          getPathFromMap );
                auto& si = state_m.back();
                if( params.connectionCostCalculator ){ //select best connection
                    int indBest = -1;
                    double minExitCost = std::numeric_limits<double>::max();
                    for(size_t i = 0 ; i < exitConnections.size() ; ++i){
                        auto& conn = exitConnections.at(i);
                        auto cost = params.connectionCostCalculator->calc(conn.path, machine);
                        if(cost < minExitCost){
                            minExitCost = cost;
                            indBest = i;
                        }
                        if(conn.savePath)
                            savePathFct(si.connPoseStart, si.connPoseEnd, machine.turning_radius, si.connectionPath);
                    }
                    if(indBest >= 0){
                        si.connectionPath = std::move(exitConnections.at(indBest).path);
                        si.connPoseEnd = std::move(exitConnections.at(indBest).endPose);
                        si.connectionCost = minExitCost;
                    }
                }
                else if(exitConnections.size() == 1){// only one option
                    si.connectionPath = std::move(exitConnections.front().path);
                    si.connPoseEnd = std::move(exitConnections.front().endPose);
                    if(exitConnections.front().savePath)
                        savePathFct(si.connPoseStart, exitConnections.front().endPose, machine.turning_radius, si.connectionPath);
                }
            }
        }
    }

    return true;
}


double getCost(const TrackSequencerSimAnnealing::State& state, const std::shared_ptr<const TrackSequencerSimAnnealing::StateCostCalculator>& stateCostCalculator){
    if(stateCostCalculator)
        return stateCostCalculator->calc(state, nullptr);
    double cost = 0;
    for(auto& it_m : state){
        for(const TrackSequencerSimAnnealing::StateTrackInfo& si : it_m.second)
            cost += ( si.trackCost + si.connectionCost );
    }
    return cost;
}



}


namespace arolib{

using namespace std::placeholders;

double TrackSequencerSimAnnealing::CCCConnectionLength::calc(const PointVec &path, const Machine &) const
{
    return geometry::getGeometryLength(path);
}

TrackSequencerSimAnnealing::TracksConnectorDef::TracksConnectorDef(std::shared_ptr<const IInfieldTracksConnector> connector){
    if(connector)
        m_connector = connector;
    else
        m_connector = std::make_shared<InfieldTracksConnectorDef>();
}

std::vector<TrackSequencerSimAnnealing::ITracksConnector::Connection> TrackSequencerSimAnnealing::TracksConnectorDef::getTrackStateConnections(const State &state,
                                                                                                                                               const State */*statePrev*/,
                                                                                                                                               const Subfield& subfield,
                                                                                                                                               const std::vector<ConnectionInfo> &connectionsInfo,
                                                                                                                                               const GetPrecomputedPathFtn &getPrecomputedPath) const
{
    auto getPath = [this, &getPrecomputedPath, &subfield](const Machine& machine, const Pose2D& pose1, const Pose2D& pose2, PointVec& path)->bool{
        bool savePath = false;
        path = getPrecomputedPath(pose1, pose2, machine, true);
        if(path.empty()){
            path = m_connector->getConnection( subfield, machine,
                                               pose1, pose2,
                                               machine.turning_radius,
                                               std::make_pair(0.0, 0.0) );
            savePath = !path.empty();
        }
        return savePath;
    };

    std::vector<TrackSequencerSimAnnealing::ITracksConnector::Connection> ret;
    if( connectionsInfo.empty() )
        return ret;

    ret.reserve(connectionsInfo.size());
    for(auto& ci : connectionsInfo){

        auto it_m = state.find( ci.machine.id );
        if(it_m == state.end()){
            ret.clear();
            return ret;
        }

        const std::vector<StateTrackInfo>& trackStates = it_m->second;

        auto& stateFrom = trackStates.at(ci.trackStateInd);

        ret.emplace_back(Connection(true));
        auto& conn = ret.back();

        if(stateFrom.connPoseStart.isValid() && stateFrom.connPoseEnd.isValid())
            conn.savePath = getPath(ci.machine, stateFrom.connPoseStart, stateFrom.connPoseEnd, conn.path);
    }
    return ret;
}

std::vector<Point> TrackSequencerSimAnnealing::TracksConnectorDef::getStandardConnection(const Pose2D &poseFrom,
                                                                                         const Pose2D &poseTo,
                                                                                         const Subfield &subfield,
                                                                                         const Machine &machine) const
{
    return m_connector->getConnection( subfield, machine,
                                       poseFrom, poseTo,
                                       -1, // use machine turning rad
                                       std::make_pair(0.0, 0.0) );

}

std::vector<TrackSequencerSimAnnealing::ITracksConnector::ExitConnection> TrackSequencerSimAnnealing::TracksConnectorDef::getExitConnections(const State &state,
                                                                                                                                             const State */*statePrev*/,
                                                                                                                                             const Subfield &subfield,
                                                                                                                                             const Machine& machine,
                                                                                                                                             const std::map<size_t, Pose2D>& fieldExitPoses,
                                                                                                                                             const GetPrecomputedPathFtn &getPrecomputedPath) const
{
    auto getPath = [this, &getPrecomputedPath, &subfield, &machine](const Pose2D& pose1, const Pose2D& pose2)->PointVec{
        auto path = getPrecomputedPath(pose1, pose2, machine, false);
        if(path.empty())
            path = m_connector->getConnection( subfield, machine,
                                               pose1, pose2,
                                               machine.turning_radius,
                                               std::make_pair(0.0, 0.0) );
        return path;
    };

    std::vector<TrackSequencerSimAnnealing::ITracksConnector::ExitConnection> ret;

    if(fieldExitPoses.empty())
        return ret;

    auto it_m = state.find(machine.id);
    if(it_m == state.end())
        return ret;

    const std::vector<StateTrackInfo>& trackStates = it_m->second;

    if(trackStates.empty())
        return ret;

    auto& stateFrom = trackStates.back();
    if(!stateFrom.connPoseStart.isValid())
        return ret;

    ret.reserve(fieldExitPoses.size());
    for(auto& it : fieldExitPoses){
        auto path = getPath(stateFrom.connPoseStart, it.second);
        if(!path.empty()){
            ret.emplace_back(ExitConnection(true));
            ret.back().path = std::move(path);
            ret.back().endPose = it.second;
        }
    }

    return ret;

}

TrackSequencerSimAnnealing::PlanningParameters::PlanningParameters(const Subfield &_subfield,
                                                                   const ITrackSequencer &_sequencer,
                                                                   //std::shared_ptr<const IInfieldTracksConnector> _connector,
                                                                   std::shared_ptr<const ITracksConnector> _connector,
                                                                   const std::map<MachineId_t, Machine> &_machines,
                                                                   const TrackSequencerSettings &_settings,
                                                                   const std::map<MachineId_t, Pose2D> &_initRefPoses,
                                                                   const std::set<size_t> &_fixedTracks,
                                                                   std::shared_ptr<const StateCostCalculator> _stateCostCalculator,
                                                                   std::shared_ptr<const ConnectionCostCalculator> _connectionCostCalculator,
                                                                   std::shared_ptr<const TrackCostCalculator> _trackCostCalculator,
                                                                   std::shared_ptr<const std::set<size_t>> _firstTrackSet,
                                                                   const std::map<size_t, Pose2D> &_fieldExitPoses):
    subfield(&_subfield),
    sequencer(&_sequencer),
    connector(_connector),
    machines(&_machines),
    settings(&_settings),
    initRefPoses(&_initRefPoses),
    fixedTracks(&_fixedTracks),
    stateCostCalculator(_stateCostCalculator),
    connectionCostCalculator(_connectionCostCalculator),
    trackCostCalculator(_trackCostCalculator),
    firstTrackSet(_firstTrackSet),
    fieldExitPoses(_fieldExitPoses)
{
}


TrackSequencerSimAnnealing::TrackSequencerSimAnnealing(std::unique_ptr<ITrackSequencer> base,
                                                       std::shared_ptr<StateGenerator> stateGenerator,
                                                       std::shared_ptr<ITracksConnector> connector,
                                                       std::shared_ptr<StateCostCalculator> stateCostCalculator,
                                                       std::shared_ptr<ConnectionCostCalculator> connectionCostCalculator,
                                                       std::shared_ptr<TrackCostCalculator> trackCostCalculator,
                                                       LogLevel logLevel) :
    ITrackSequencer(__FUNCTION__, logLevel),
    m_stateGenerator( stateGenerator ? stateGenerator : std::make_shared<StateGeneratorSingleSwap>() ),
    m_stateCostCalculator(stateCostCalculator),
    m_connectionCostCalculator(connectionCostCalculator),
    m_trackCostCalculator(trackCostCalculator)
{

    if(base)
        m_base = std::move(base);
    else{
        m_base = std::make_unique<TrackSequencerClosestNext>(false);
        m_base->logger().setParent( loggerPtr() );
    }

    if(connector)
        m_connector = connector;
    else
        //m_connector = std::make_shared<InfieldTracksConnectorDef>( loggerPtr() );
        m_connector = std::make_shared<TracksConnectorDef>( std::make_shared<InfieldTracksConnectorDef>( loggerPtr() ) );

    if(!m_stateCostCalculator){
        if(!m_connectionCostCalculator)
            m_connectionCostCalculator = std::make_shared<CCCConnectionLength>();

        if(!m_trackCostCalculator)
            m_trackCostCalculator = std::make_shared<NoTrackCostCalculator>();
    }
}

void TrackSequencerSimAnnealing::StateGenerator::init(PlanningParameters &params, bool computeConnectionPath, SavePathFunction savePathFct, size_t maxAttemptsNewState)
{
    m_params = &params;
    m_computeConnectionPath = computeConnectionPath;
    m_savePathFct = savePathFct;
    m_maxAttemptsNewState = std::min(maxAttemptsNewState, std::numeric_limits<size_t>::max() - 10);
}

bool TrackSequencerSimAnnealing::StateGenerator::isStateVisited(const State &state) const
{
    return m_visitedStates.find( toStateStr(state) ) != m_visitedStates.end();
}


bool TrackSequencerSimAnnealing::StateGenerator::addStateToVisited(const State &state)
{
    return m_visitedStates.insert( toStateStr(state) ).second;
}

const std::set<std::string> TrackSequencerSimAnnealing::StateGenerator::visitedStates() const
{
    return m_visitedStates;
}
void TrackSequencerSimAnnealing::StateGenerator::clearVisitedStates(){
    m_visitedStates.clear();
}

std::string TrackSequencerSimAnnealing::StateGenerator::toStateStr(const State &state){
    std::string stateStr;
    for(auto& it_m : state){
        stateStr += std::to_string(it_m.first) + ":";
        for(const StateTrackInfo& si : it_m.second)
            stateStr += std::to_string(si.trackInd) + "-" + std::to_string(si.trackInPointsDirection) + ",";
        stateStr += ";";
    }
    return stateStr;
}

const TrackSequencerSimAnnealing::PlanningParameters *TrackSequencerSimAnnealing::StateGenerator::params() const
{
    return m_params;
}

AroResp TrackSequencerSimAnnealing::computeSequences(const Subfield &subfield,
                                                     const std::vector<Machine> &machines,
                                                     const TrackSequencerSettings &settings,
                                                     Sequences_t &sequences,
                                                     const std::map<MachineId_t, Pose2D>& initRefPoses,
                                                     const std::set<size_t> &excludeTrackIndexes)
{
    std::map<MachineId_t, std::vector<TrackInfo> > base_seqs;

    auto originalPathsMapPtr = m_pathsMapManager;
    m_pathsMapManager = std::make_shared<geometry::PathsMapManager>(*originalPathsMapPtr);

    m_pathsMapManager->updatePaths(m_base->getPathsMapManager(), false);
    if(m_useBaseConnections){
        m_base->setPathsMapManager( m_pathsMapManager );
        m_base->setSaveAllComputedPaths( true );
    }
    else if(m_pathsMapManager.get() == m_base->getPathsMapManager().get()){
        m_base->setSaveAllComputedPaths( false );
        m_base->setSaveConnectingPaths( false );
    }


    std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

    logger().printDebug(__FUNCTION__, "Using base track sequencer to obtain base sequences...");
    auto aroResp = m_base->computeSequences(subfield, machines, settings, base_seqs, initRefPoses, excludeTrackIndexes);
    if(aroResp.isError()){
        if(!m_saveAllComputedPaths)
            m_pathsMapManager->clearPathsMap();
        m_pathsMapManager = originalPathsMapPtr;
        return AroResp(1, "Error obtaining the base sequences: " + aroResp.msg);
    }

    if(settings.maxSequencePlanningTime > 1e-6){
        double duration = 0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count();
        if(duration > settings.maxSequencePlanningTime){
            sequences = std::move(base_seqs);
            originalPathsMapPtr->movePaths( m_pathsMapManager, true ); //here all computed paths will be saved, even if m_saveAllComputedPaths == false
            m_pathsMapManager = originalPathsMapPtr;
            return AroResp::LoggingResp(-1, "Planning timeout reached before running SimAnnealing", "", logger(), LogLevel::WARNING, __FUNCTION__);
        }
    }

    State initialState;

    auto machinesMap = Machine::toMachineIdMap(machines);
    for(auto &it_m : machinesMap)
        it_m.second.turning_radius = IInfieldTracksConnector::getTurningRad(it_m.second, settings.useMachineTurningRad ? -1 : 0);

    std::set<size_t> fixedTracks{};

    std::shared_ptr<std::set<size_t>> firstTrackSet = std::make_shared<std::set<size_t>>();
    if(settings.limitStartToExtremaTracks){
        auto extremaTrackInds = geometry::getInfieldExtremaTracksIndexes(subfield, excludeTrackIndexes);
        if(extremaTrackInds.empty()){
            m_pathsMapManager = originalPathsMapPtr;
            return AroResp(1, "Error obtaining extrema tracks: no extrema tracks found");
        }
        firstTrackSet->insert(extremaTrackInds.begin(), extremaTrackInds.end());
    }

    PlanningParameters params(subfield, *this, m_connector, machinesMap, settings, initRefPoses, fixedTracks,
                              m_stateCostCalculator, m_connectionCostCalculator, m_trackCostCalculator,
                              firstTrackSet,
                              getFieldExitPoses(subfield));

    SavePathFunction savePathFct = std::bind(&TrackSequencerSimAnnealing::addPathToMapInternal, this, _1, _2, _3, _4);

    logger().printDebug(__FUNCTION__, "Initializing state from base sequences...");
    if(!initStateFromSequence(params, base_seqs, initialState, savePathFct)){
        m_pathsMapManager = originalPathsMapPtr;
        return AroResp(1, "Error initializing state");
    }

    m_stateGenerator->init(params, m_connectionCostCalculator->dependsOnPath(), savePathFct);
    m_stateGenerator->addStateToVisited(initialState);

    double initialCost = getCost(initialState, params.stateCostCalculator);
    logger().printDebug(__FUNCTION__, "Base cost = " + double2string(initialCost));

    if(m_tryWithSimpleSequencer){
        logger().printDebug(__FUNCTION__, "Using basic track sequencer to obtain base sequences...");
        TrackSequencerAdjacentNext ts0;
        ts0.logger().setParent(loggerPtr());

        if(true){ // if the simple track sequencer is too basic and does not use propper connections
            m_base->setSaveAllComputedPaths( false );
            m_base->setSaveConnectingPaths( false );
        }
        else{ // if the simple track sequencer does use propper connections
            ts0.setPathsMapManager(m_pathsMapManager);
            ts0.setSaveAllComputedPaths(true);
        }

        std::map<MachineId_t, std::vector<TrackInfo> > base_seqs_2;
        auto aroResp = ts0.computeSequences(subfield, machines, settings, base_seqs_2, initRefPoses, excludeTrackIndexes);
        if(aroResp.isError())
            logger().printWarning(__FUNCTION__, "Error computing sequences using basic sequencer: " +aroResp.msg);
        else{
            State initialStateBasic;
            logger().printDebug(__FUNCTION__, "Initializing test state from base sequences (basic track sequencer)...");
            if(!initStateFromSequence(params, base_seqs_2, initialStateBasic, savePathFct))
                logger().printWarning(__FUNCTION__, "Error initializing state with sequences obtained from the basic sequencer: " +aroResp.msg);
            else{
                m_stateGenerator->addStateToVisited(initialStateBasic);
                double initialCost2 = getCost(initialStateBasic, params.stateCostCalculator);

                if(initialCost2 < initialCost){
                    logger().printDebug(__FUNCTION__, "Basic sequencer resulted in a lower cost (" + double2string(initialCost) + " -> " + double2string(initialCost2) + "). Using its results...");
                    std::swap(initialState, initialStateBasic);
                    std::swap(initialCost, initialCost2);
                    std::swap(base_seqs, base_seqs_2);
                }
            }
        }
    }

    logger().printDebug(__FUNCTION__, "Base tracks sequences obtained with a cost of " + double2string(initialCost));

    auto temperatureFunction = [this](size_t x){
        return m_startTemp * std::exp( -1 * m_kTemp * x );
    };

    SimulatedAnnealing<State> sa;

    SimulatedAnnealing<State>::BreakFunction* pBreakFunction = nullptr;

    double maxPlanningTime = settings.maxSequencePlanningTime;

    auto updatePathsMap = [this, &originalPathsMapPtr, &machinesMap](const State& state){
        if(m_saveAllComputedPaths){
            originalPathsMapPtr->movePathsMap(m_pathsMapManager);
            m_pathsMapManager = originalPathsMapPtr;
        }
        else{
            m_pathsMapManager->clearPathsMap();
            m_pathsMapManager = originalPathsMapPtr;
            if(m_saveConnectingPaths){
                for(auto& it_m : state){
                    double turningRad = machinesMap[it_m.first].getTurningRadius();
                    for(const StateTrackInfo& si : it_m.second)
                        m_pathsMapManager->addPathToMap(si.connPoseStart, si.connPoseEnd, turningRad, si.connectionPath);
                }
            }
        }
    };

    if(maxPlanningTime > 1e-6){
        double duration = 0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count();
        maxPlanningTime -= duration;
        if(maxPlanningTime < 1e-3){
            sequences = std::move(base_seqs);
            updatePathsMap(initialState);
            return AroResp::LoggingResp(-1, "Planning timeout reached before running SimAnnealing", "", logger(), LogLevel::WARNING, __FUNCTION__);
        }
    }

    StateGenerator::ResultState result = sa.compute(initialState,
                                                    initialCost,
                                                    m_iterations,
                                                    *m_stateGenerator,
                                                    temperatureFunction,
                                                    true,
                                                    m_subIterations,
                                                    pBreakFunction,
                                                    maxPlanningTime);

    if(!result){
        sequences = std::move(base_seqs);
        updatePathsMap(initialState);
        return AroResp::LoggingResp(-1, "Sim. annealing failed! Returning base sequences", "", logger(), LogLevel::WARNING, __FUNCTION__);
    }
    logger().printDebug(__FUNCTION__, "Sim. annealing improved cost: " + double2string(initialCost) + " -> " + double2string(result->second));

    const State& finalState = result->first;


    sequences.clear();
    logger().printDebug(__FUNCTION__, "Resulting IF tracks sequences ...");
    for(auto& it_m : finalState){
        auto& states = it_m.second;
        if(states.size() < 2)
            continue;
        auto& seq = sequences[it_m.first];
        seq.reserve( states.size()-1 );
        logger().printDebug("", "\t Machine id : " + std::to_string(it_m.first) + ":");
        for(size_t i = 1 ; i < states.size() ; ++i){
            const StateTrackInfo& si = states.at(i);
            seq.emplace_back( TrackInfo(si.trackInd, si.trackInPointsDirection ? TrackPointsDirection::FORWARD : TrackPointsDirection::REVERSE) );
            logger().printDebug("", "\t\t" + std::to_string(si.trackInd) + (seq.back().trackPointsDirection == TrackPointsDirection::FORWARD ? "(FW)" : "(RV)"));
        }
    }

    updatePathsMap(finalState);

    return AroResp::ok();

}

bool TrackSequencerSimAnnealing::setStartTemperature(float startTemp) {
    if(startTemp <= 0)
        return false;
    m_startTemp = startTemp;
    return true;
}

bool TrackSequencerSimAnnealing::setTemperatureCoefficient(float k) {
    if(k <= 0)
        return false;
    m_kTemp = k;
    return true;
}

bool TrackSequencerSimAnnealing::setIterations(size_t n) {
    if(n == 0)
        return false;
    m_iterations = n;
    return true;
}

bool TrackSequencerSimAnnealing::setSubIterations(size_t n)
{
    if(n == 0)
        return false;
    m_subIterations = n;
    return true;
}

void TrackSequencerSimAnnealing::setUseBaseConnections(bool use)
{
    m_useBaseConnections = use;
}

std::map<size_t, Pose2D> TrackSequencerSimAnnealing::getFieldExitPoses(const Subfield &subfield)
{
    std::map<size_t, Pose2D> ret;
    if(subfield.boundary_outer.points.size() < 3)
        return ret;

    auto boundary = subfield.boundary_outer;
    geometry::closePolygon(boundary);

    for(size_t i = 0 ; i < subfield.access_points.size() ; ++i){
        int ind = geometry::addSampleToGeometryClosestToPoint(boundary.points, subfield.access_points.at(i), 1);
        if(ind < 0)
            continue;
        size_t indPrev = ind > 0 ? ind-1 : boundary.points.size()-2;
        size_t indNext = ind+1 < boundary.points.size() ? ind+1 : 1;
        Pose2D pose (boundary.points.at(ind));
        auto ang = geometry::get_angle( boundary.points.at(indPrev), pose, boundary.points.at(indNext) );
        if(std::fabs(std::fabs(ang) - M_PI) < 1e-6){//straight segment
            pose.angle = geometry::get_angle( pose, boundary.points.at(indPrev) ) - ( geometry::isPolygonClockwise(boundary) ? M_PI_2 : -M_PI_2);
            geometry::correct_angle(pose.angle);
            ret[i] = pose;
            continue;
        }
        pose = geometry::getAngleBisectionPose( boundary.points.at(indPrev), pose, boundary.points.at(indNext), false, true );
        if( geometry::isPolygonClockwise(boundary) ){
            pose.angle += M_PI;
            geometry::correct_angle(pose.angle);
        }
        ret[i] = pose;

    }
    return ret;
}


} // namespace arolib
