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
 
#include "arolib/planning/track_sequencing/tracksequencer.hpp"

namespace arolib{

bool ITrackSequencer::TrackSequencerSettings::parseFromStringMap(TrackSequencerSettings &params, const std::map<std::string, std::string> &map, bool strict)
{
    TrackSequencerSettings tmp;

    std::map<std::string, bool*> bMap = { {"limitStartToExtremaTracks" , &tmp.limitStartToExtremaTracks},
                                          {"useMachineTurningRad" , &tmp.useMachineTurningRad} };



    if( !setValuesFromStringMap( map, bMap, strict) )
        return false;

    params = tmp;
    return true;

}

std::map<std::string, std::string> ITrackSequencer::TrackSequencerSettings::parseToStringMap(const TrackSequencerSettings &params)
{
    std::map<std::string, std::string> ret;
    ret["limitStartToExtremaTracks"] = std::to_string( params.limitStartToExtremaTracks );
    ret["useMachineTurningRad"] = std::to_string( params.useMachineTurningRad );

    return ret;
}

ITrackSequencer::ITrackSequencer(const std::string childName, const LogLevel &logLevel):
    LoggingComponent(logLevel, childName)
{

}

void ITrackSequencer::addPathToMap(const Pose2D &pose1, const Pose2D &pose2, double turningRad, const PointVec& path)
{
    std::lock_guard<std::mutex> guard(m_mutex_pathsMap);
    int iRad = ( turningRad < 1e-9 ? -1 : turningRad * 100);
    (*m_pathsMap)[pose1][pose2][iRad] = path;
}

ITrackSequencer::TrackInfo::TrackInfo(size_t ind, TrackPointsDirection dir)
    : trackIndex(ind), trackPointsDirection(dir)
{

}

void ITrackSequencer::setInfieldTrackConnector(std::shared_ptr<IInfieldTracksConnector> connector) {
    if(connector)
        m_tracksConnector = connector;
}

ITrackSequencer::PathsMapConstPtr_t ITrackSequencer::getPathsMap() const { return m_pathsMap; }

PointVec ITrackSequencer::getPathFromMap(PathsMapConstPtr_t map, const Pose2D &pose1, const Pose2D &pose2, double turningRad, bool checkBidirectional)
{
    if(!map)
        return {};
    int iRad = ( turningRad < 1e-9 ? -1 : turningRad * 100);
    auto it1 = map->find(pose1);
    if(it1 != map->end()){
        auto it2 = it1->second.find(pose2);
        if(it2 != it1->second.end()){
            auto it3 = it2->second.find(iRad);
            if(it3 != it2->second.end()){
                return it3->second;
            }
        }
    }
    if(checkBidirectional){
        auto it1 = map->find(pose2);
        if(it1 != map->end()){
            auto it2 = it1->second.find(pose1);
            if(it2 != it1->second.end()){
                auto it3 = it2->second.find(iRad);
                if(it3 != it2->second.end()){
                    return it3->second;
                }
            }
        }
    }
    return {};
}

PointVec ITrackSequencer::getPathFromMap(const Pose2D &pose1, const Pose2D &pose2, double turningRad, bool checkBidirectional)
{
    std::lock_guard<std::mutex> guard(m_mutex_pathsMap);
    return getPathFromMap(m_pathsMap, pose1, pose2, turningRad, checkBidirectional);
}


} // namespace arolib
