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
 
#include "arolib/planning/track_sequencing/tracksequencer.hpp"

#include "arolib/misc/basicconversions.hpp"

namespace arolib{

bool ITrackSequencer::TrackSequencerSettings::parseFromStringMap(TrackSequencerSettings &params, const std::map<std::string, std::string> &map, bool strict)
{
    TrackSequencerSettings tmp;

    std::map<std::string, double*> dMap = { {"maxSequencePlanningTime" , &tmp.maxSequencePlanningTime} };
    std::map<std::string, bool*> bMap = { {"limitStartToExtremaTracks" , &tmp.limitStartToExtremaTracks},
                                          {"useMachineTurningRad" , &tmp.useMachineTurningRad},
                                          {"considerFieldExit" , &tmp.considerFieldExit} };



    if( !setValuesFromStringMap( map, dMap, strict)
            || !setValuesFromStringMap( map, bMap, strict) )
        return false;

    params = tmp;
    return true;

}

std::map<std::string, std::string> ITrackSequencer::TrackSequencerSettings::parseToStringMap(const TrackSequencerSettings &params)
{
    std::map<std::string, std::string> ret;
    ret["maxSequencePlanningTime"] = double2string( params.maxSequencePlanningTime );
    ret["limitStartToExtremaTracks"] = std::to_string( params.limitStartToExtremaTracks );
    ret["useMachineTurningRad"] = std::to_string( params.useMachineTurningRad );
    ret["considerFieldExit"] = std::to_string( params.considerFieldExit );

    return ret;
}


ITrackSequencer::ITrackSequencer(const std::string & childName, const LogLevel &logLevel):
    LoggingComponent(logLevel, childName)
{

}

ITrackSequencer::TrackInfo::TrackInfo(size_t ind, TrackPointsDirection dir)
    : trackIndex(ind), trackPointsDirection(dir)
{

}

void ITrackSequencer::setInfieldTrackConnector(std::shared_ptr<IInfieldTracksConnector> connector) {
    if(connector)
        m_tracksConnector = connector;
}

void ITrackSequencer::setPathsMapManager(geometry::PathsMapManagerPtr_t pmm)
{
    if(pmm)
        m_pathsMapManager = pmm;
    else
        m_pathsMapManager = std::make_shared<geometry::PathsMapManager>();
}

geometry::PathsMapManagerPtr_t ITrackSequencer::getPathsMapManager()
{
    return m_pathsMapManager;
}

geometry::PathsMapManagerConstPtr_t ITrackSequencer::getPathsMapManager() const
{
    return m_pathsMapManager;
}

void ITrackSequencer::setSaveAllComputedPaths(bool saveThem)
{
    m_saveAllComputedPaths = saveThem;
}

bool ITrackSequencer::getSaveAllComputedPaths() const
{
    return m_saveAllComputedPaths;
}

void ITrackSequencer::setSaveConnectingPaths(bool saveThem)
{
    m_saveConnectingPaths = saveThem;
}

bool ITrackSequencer::getSaveConnectingPaths() const
{
    return m_saveConnectingPaths;
}

ITrackSequencer::Sequences_t ITrackSequencer::removeTracksFromSequence(const Sequences_t &sequences,
                                                                       const std::set<size_t> &trackInds,
                                                                       bool removeGivenTracks)
{
    Sequences_t ret;
    for(auto& it_m : sequences){
        ret[it_m.first] = {};
        auto& seq = ret[it_m.first];
        seq.reserve( it_m.second.size() );
        if(removeGivenTracks){
            for(const TrackInfo & ti : it_m.second){
                if( trackInds.find(ti.trackIndex) == trackInds.end() )
                    seq.emplace_back(ti);
            }
        }
        else{
            for(const TrackInfo & ti : it_m.second){
                if( trackInds.find(ti.trackIndex) != trackInds.end() )
                    seq.emplace_back(ti);
            }
        }
    }
    return ret;
}



} // namespace arolib
