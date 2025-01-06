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
 
#include "arolib/planning/track_sequencing/simulated_annealing/stategenerator_swaptracks.hpp"

#include "arolib/misc/randomgeneration.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/field_geometry_processing.hpp"

namespace{

using namespace arolib;

inline Pose2D getEntryExitPose (const Subfield& subfield, size_t trackIndex, bool trackInPointsDirection, bool entry){
    return geometry::getLinestringEntryExitPose(subfield.tracks.at(trackIndex).points, entry, trackInPointsDirection);
}

}

namespace arolib{
TrackSequencerSimAnnealing::StateGenerator::ResultState StateGeneratorSingleSwap::_getState(const TrackSequencerSimAnnealing::State &currentState, double currentEnergy)
{
    return getStateSingleSwap(currentState, currentEnergy);
}

TrackSequencerSimAnnealing::StateGenerator::ResultState StateGeneratorSingleSwap::getStateSingleSwap(const TrackSequencerSimAnnealing::State &currentState, double currentEnergy)
{

    auto getNonFixedTrackInds = [](const std::set<size_t>& fixedTracks, const std::vector<TrackSequencerSimAnnealing::StateTrackInfo>& states)->std::vector<size_t>{
        std::vector<size_t> inds;
        inds.reserve( states.size()-1 );
        for(size_t i = 1 ; i < states.size() ; ++i){
            if( fixedTracks.find( states.at(i).trackInd ) != fixedTracks.end())
                continue;
            inds.push_back(i);
        }
        return inds;
    };


    auto replaceConnectionsAndSavePath = [this](TrackSequencerSimAnnealing::ITracksConnector::Connection& conn, TrackSequencerSimAnnealing::StateTrackInfo& sti, double turningRad){
        std::swap(conn.path, sti.connectionPath);
        if(conn.savePath && !sti.connectionPath.empty())
            m_savePathFct(sti.connPoseStart, sti.connPoseEnd, turningRad, sti.connectionPath);
    };

    std::vector<MachineId_t> machineIds;
    for(auto& it_m : currentState){
        if(it_m.second.size() > 2) //Note: >2 because the first corresponds to the initial connection (it must have at least 2 tracks plus the initial connection)
            machineIds.push_back(it_m.first);
    }

    ResultState result = nullptr;

    bool checkForFixedTracks = params()->fixedTracks && !params()->fixedTracks->empty();

    bool checkFirstTrack = !checkForFixedTracks
                            && params()->settings->limitStartToExtremaTracks
                            && params()->firstTrackSet
                            && !params()->firstTrackSet->empty(); //the first track is not fixed and it must be in the given firstTrackSet

    size_t countAttemptsNewState = 0;

    std::map<MachineId_t, std::set< std::pair<int, int> > > failedSwapsAll;

    while(!machineIds.empty() && !result && countAttemptsNewState < m_maxAttemptsNewState){

        int ind1 = -1, ind2 = -1;
        size_t countValidTracks = 0;

        size_t machineInd = gen_random_int(0, machineIds.size()-1);
        MachineId_t machineId = machineIds.at( machineInd );

        auto it_machine = params()->machines->find(machineId);
        if(it_machine == params()->machines->end()){
            machineIds.erase(machineIds.begin() + machineInd);
            continue;
        }
        auto& machine = it_machine->second;

        auto& currentStates = currentState.at(machineId);

        auto& failedSwaps = failedSwapsAll[machineId];
        std::vector<size_t> validTracks;
        bool checkForValidTracks = false;
        if(checkForFixedTracks){
            validTracks = getNonFixedTrackInds( *params()->fixedTracks, currentStates );
            countValidTracks = validTracks.size();
            checkForValidTracks = true;
        }
        else
            countValidTracks = currentStates.size()-1; //the states[0] is not a real track, it is the starting pose

        if(countValidTracks < 2){
            machineIds.erase(machineIds.begin() + machineInd);
            continue;
        }

        bool okMachine = true;


        //@note: the permutations only contain single swapping (i.e. does not differenciate between track directions)

        size_t countSwapCombinations = std::tgamma(countValidTracks+1) / (2 * std::tgamma(countValidTracks-1) )  // n!/(2*(n−2)!)
                + 1e-3; // +1e-3 just in case because tgamma returns float/double

        long double countPermutationsValid;

        if(checkFirstTrack){
            size_t countFirstTracks = 0;
            if(!checkForValidTracks)
                countFirstTracks = params()->firstTrackSet->size();
            else{
                for(auto& ind : validTracks){
                    if(params()->firstTrackSet->find(ind) != params()->firstTrackSet->end())
                        ++countFirstTracks;
                }
            }
            if(countFirstTracks == 0)
                okMachine = false;
            else{
                countPermutationsValid = std::tgamma(countFirstTracks+1) // (n_validFirst!)
                        * std::tgamma(countValidTracks) // ( (n-1)! )
                        + 1e-3; // +1e-3 just in case because tgamma returns float/double
            }

        }
        else
            countPermutationsValid = std::tgamma(countValidTracks+1)  // n!
                    + 1e-3; // +1e-3 just in case because tgamma returns float/double

        while(okMachine){
            ind1 = -1, ind2 = -1;
            if(checkForValidTracks){
                int randInd = gen_random_int(0, validTracks.size()-1);
                ind1 = validTracks.at( randInd );
                ind2 = at_cyclic(validTracks, randInd + gen_random_int(1, validTracks.size()-1) );
            }
            else{
                ind1 = gen_random_int(1, currentStates.size()-1);
                ind2 = get_index_from_cyclic_container(currentStates, ind1 + gen_random_int(1, currentStates.size()-2));
                if(ind2 < ind1 )//add one to compensate for state[0]
                    ++ind2;
            }

            if(ind2 < ind1 )
                std::swap(ind2, ind1);

            if(!failedSwaps.insert( std::make_pair(ind1, ind2) ).second){
                if(failedSwaps.size() >= countSwapCombinations){

                    okMachine = false;
                    break;
                }
                continue;
            }

            if( ind1 == 1
                    && checkFirstTrack
                    && params()->firstTrackSet->find( currentStates.at(ind2).trackInd ) == params()->firstTrackSet->end() )
                continue;

            break;
        }
        ++countAttemptsNewState;

        if(!okMachine){
            machineIds.erase(machineIds.begin() + machineInd);
            continue;
        }

        result = SimulatedAnnealing<TrackSequencerSimAnnealing::State>::initResultState(currentState, currentEnergy);
        TrackSequencerSimAnnealing::State &newState = result->first;
        double &newEnergy = result->second;

        auto& states = newState.at(machineId);

        std::swap(states.at(ind1), states.at(ind2));

        //Note: after the swap, ind1 and ind2 are pointing to the other state. we need to get these references after the swap in case one of the prev states is part of the swap
        auto& state1Prev = states.at(ind1-1);
        size_t indState2Prev = std::abs(ind1 - ind2) != 1 ? ind2-1 : ind2;
        auto& state2Prev = states.at(indState2Prev);

        auto& state1Swapped = states.at(ind1);
        auto& state2Swapped = states.at(ind2);

        double costPrev = state1Prev.connectionCost + state1Swapped.connectionCost + state1Swapped.trackCost
                + state2Prev.connectionCost + state2Swapped.connectionCost + state2Swapped.trackCost; // used only if params()->stateCostCalculator == nullptr)


        if( !(state1Swapped.trackInPointsDirection ^ state1Swapped.trackInRelativeDirection) ^ !(state2Swapped.trackInPointsDirection ^ state2Swapped.trackInRelativeDirection) ){ //invert directions of tracks
            state1Swapped.trackInPointsDirection = !state1Swapped.trackInPointsDirection;
            state2Swapped.trackInPointsDirection = !state2Swapped.trackInPointsDirection;
            state1Prev.connPoseEnd = getEntryExitPose(*params()->subfield, state1Swapped.trackInd, state1Swapped.trackInPointsDirection, true);
            if(ind2 != ind1+1)//state2Prev == state2Swapped == state1(original)
                state2Prev.connPoseEnd = getEntryExitPose(*params()->subfield, state2Swapped.trackInd, state2Swapped.trackInPointsDirection, true);

            state1Swapped.connPoseStart = getEntryExitPose(*params()->subfield, state1Swapped.trackInd, state1Swapped.trackInPointsDirection, false);
            state2Swapped.connPoseStart = getEntryExitPose(*params()->subfield, state2Swapped.trackInd, state2Swapped.trackInPointsDirection, false);

            if( !params()->stateCostCalculator && params()->trackCostCalculator && params()->trackCostCalculator->dependsOnTrackDirection() ){
                state1Swapped.trackCost = params()->trackCostCalculator->calc( params()->subfield->tracks.at(state1Swapped.trackInd), state1Swapped.trackInPointsDirection, machine );
                state2Swapped.trackCost = params()->trackCostCalculator->calc( params()->subfield->tracks.at(state2Swapped.trackInd), state2Swapped.trackInPointsDirection, machine );
            }

            std::swap(state1Swapped.connPoseEnd, state2Swapped.connPoseEnd);
            if(ind2 == ind1+1)//state2Prev == state2Swapped == state1(original)
                state1Swapped.connPoseEnd = getEntryExitPose(*params()->subfield, state2Swapped.trackInd, state2Swapped.trackInPointsDirection, true);
        }
        else if(ind2 != ind1+1){
            std::swap(state1Prev.connPoseEnd, state2Prev.connPoseEnd);
            std::swap(state1Swapped.connPoseEnd, state2Swapped.connPoseEnd);
        }
        else{//state2Prev == state2Swapped == state1(original)
            state1Prev.connPoseEnd = state2Prev.connPoseEnd;
            std::swap(state1Swapped.connPoseEnd, state2Swapped.connPoseEnd);
            state1Swapped.connPoseEnd = getEntryExitPose(*params()->subfield, state2Swapped.trackInd, state2Swapped.trackInPointsDirection, true);
        }

        if(!addStateToVisited(newState)){
            result = nullptr;
            if(visitedStates().size() >= countPermutationsValid)
                machineIds.erase(machineIds.begin() + machineInd);
            continue;
        }


        if(m_computeConnectionPath){

            auto& sequencer = params()->sequencer;
            auto getPrecomputedPath = [&sequencer] (const Pose2D& pose1, const Pose2D& pose2, const Machine& machine, bool checkBidirectional)->PointVec{
                return sequencer->getPathsMapManager()->getPathFromMap(pose1, pose2, machine.turning_radius, checkBidirectional);
            };

            bool incState2Prev = false, incState2 = false;
            std::vector<TrackSequencerSimAnnealing::ITracksConnector::ConnectionInfo> connectionsInfo;
            connectionsInfo.emplace_back( TrackSequencerSimAnnealing::ITracksConnector::ConnectionInfo( machine, ind1-1 ) );
            connectionsInfo.emplace_back( TrackSequencerSimAnnealing::ITracksConnector::ConnectionInfo( machine, ind1 ) );
            if(indState2Prev != ind2){
                connectionsInfo.emplace_back( TrackSequencerSimAnnealing::ITracksConnector::ConnectionInfo( machine, indState2Prev ) );
                incState2Prev = true;
            }
            if(ind2 != states.size()-1){
                connectionsInfo.emplace_back( TrackSequencerSimAnnealing::ITracksConnector::ConnectionInfo( machine, ind2 ) );
                incState2 = true;
            }

            auto connections = params()->connector->getTrackStateConnections( newState, &currentState, *params()->subfield, connectionsInfo, getPrecomputedPath);

            if(connections.size() != connectionsInfo.size()){
                result = nullptr;
                continue;
            }

            bool connsOK = true;
            for(size_t i = 0 ; i < connections.size() ; ++i){
                const auto& conn = connections.at(i);
                const auto& ti = connectionsInfo.at(i);
                const auto& ts = states.at( ti.trackStateInd );
                if( ts.connPoseStart.isValid() && ts.connPoseEnd.isValid() && conn.path.empty() ){
                    connsOK = false;
                    break;
                }
            }
            if(!connsOK){
                result = nullptr;
                continue;
            }


            bool gotFieldExitPath = !connections.back().path.empty(); //checked before swapping the paths

            replaceConnectionsAndSavePath(connections.at(0), state1Prev, machine.turning_radius);
            replaceConnectionsAndSavePath(connections.at(1), state1Swapped, machine.turning_radius);
            if(incState2Prev)
                replaceConnectionsAndSavePath(connections.at(2), state2Prev, machine.turning_radius);
            if(incState2)
                replaceConnectionsAndSavePath(connections.back(), state2Swapped, machine.turning_radius);

            if(!gotFieldExitPath
                    && params()->settings->considerFieldExit
                    && !params()->fieldExitPoses.empty()
                    && ind2 == states.size()-1){

                std::vector<TrackSequencerSimAnnealing::ITracksConnector::ExitConnection> exitConnections = params()->connector->getExitConnections( newState,
                                                                                                                                                     &currentState,
                                                                                                                                                     *params()->subfield,
                                                                                                                                                     machine,
                                                                                                                                                     params()->fieldExitPoses,
                                                                                                                                                     getPrecomputedPath );

                if(exitConnections.empty()){
                    result = nullptr;
                    continue;
                }

                incState2 = true;
                if( params()->connectionCostCalculator ){ //select best connection
                    int indBest = -1;
                    double minExitCost = std::numeric_limits<double>::max();
                    for(size_t i = 0 ; i < exitConnections.size() ; ++i){
                        auto& conn = exitConnections.at(i);
                        auto cost = params()->connectionCostCalculator->calc(conn.path, machine);
                        if(cost < minExitCost){
                            minExitCost = cost;
                            indBest = i;
                        }
                        if(conn.savePath)
                            m_savePathFct(state2Swapped.connPoseStart, state2Swapped.connPoseEnd, machine.turning_radius, state2Swapped.connectionPath);
                    }

                    if(indBest < 0){
                        result = nullptr;
                        continue;
                    }
                    state2Swapped.connectionPath = std::move(exitConnections.at(indBest).path);
                    state2Swapped.connPoseEnd = std::move(exitConnections.at(indBest).endPose);
                    state2Swapped.connectionCost = minExitCost;
                }
                else if(exitConnections.size() == 1){// only one option
                    state2Swapped.connectionPath = std::move(exitConnections.front().path);
                    state2Swapped.connPoseEnd = std::move(exitConnections.front().endPose);
                    if(exitConnections.front().savePath)
                        m_savePathFct(state2Swapped.connPoseStart, exitConnections.front().endPose, machine.turning_radius, state2Swapped.connectionPath);
                }
                else{ // no way to decide for best exit connection -> remove path
                    state2Swapped.connectionPath.clear();
                    state2Swapped.connPoseEnd.setInvalid();
                }
            }


            if(/*!params()->stateCostCalculator && */params()->connectionCostCalculator){
                state1Prev.connectionCost = params()->connectionCostCalculator->calc(state1Prev.connectionPath, machine);
                state1Swapped.connectionCost = params()->connectionCostCalculator->calc(state1Swapped.connectionPath, machine);
                if(incState2Prev)
                    state2Prev.connectionCost = params()->connectionCostCalculator->calc(state2Prev.connectionPath, machine);
                if(incState2)
                    state2Swapped.connectionCost = params()->connectionCostCalculator->calc(state2Swapped.connectionPath, machine);
            }

        }

        if(params()->stateCostCalculator)
            newEnergy = params()->stateCostCalculator->calc(newState, &currentState);
        else{
            double costNew = state1Prev.connectionCost + state1Swapped.connectionCost + state1Swapped.trackCost
                    + state2Prev.connectionCost + state2Swapped.connectionCost + state2Swapped.trackCost;

            newEnergy += (costNew - costPrev);
        }

    }

    return result;
}


} // namespace arolib

