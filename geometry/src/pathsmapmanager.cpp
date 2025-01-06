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

#include "arolib/geometry/pathsmapmanager.hpp"

#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/curves_helper.hpp"

namespace arolib{

namespace geometry{

void PathsMapManager::addPathToMap(const Pose2D &pose1, const Pose2D &pose2, double turningRad, const PointVec &path)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    int iRad = ( turningRad < 1e-9 ? -1 : turningRad * 100);
    m_pathsMap[pose1][pose2][iRad] = path;
}

void PathsMapManager::clearPathsMap()
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_pathsMap.clear();
}

PointVec PathsMapManager::getPathFromMap(const PathsMapManager &map, const Pose2D &pose1, const Pose2D &pose2, double turningRad, bool checkBidirectional)
{
    return map.getPathFromMap(pose1, pose2, turningRad, checkBidirectional);
}

PointVec PathsMapManager::getPathFromMap(const PathsMapManager &map, const Pose2D &pose1, const Pose2D &pose2, const Machine &machine, bool checkBidirectional)
{
    return map.getPathFromMap(pose1, pose2, machine.turning_radius, checkBidirectional);
}

PointVec PathsMapManager::getPathFromMap(PathsMapManagerConstPtr_t map, const Pose2D &pose1, const Pose2D &pose2, double turningRad, bool checkBidirectional)
{
    if(!map)
        return {};
    return map->getPathFromMap(pose1, pose2, turningRad, checkBidirectional);
}

PointVec PathsMapManager::getPathFromMap(PathsMapManagerConstPtr_t map, const Pose2D &pose1, const Pose2D &pose2, const Machine &machine, bool checkBidirectional)
{
    if(!map)
        return {};
    return map->getPathFromMap(pose1, pose2, machine.turning_radius, checkBidirectional);
}

PointVec PathsMapManager::getPathFromMap(const Pose2D &pose1, const Pose2D &pose2, double turningRad, bool checkBidirectional) const
{
    std::lock_guard<std::mutex> guard(m_mutex);

    int iRad = ( turningRad < 1e-9 ? -1 : turningRad * 100);
    auto it1 = m_pathsMap.find(pose1);
    if(it1 != m_pathsMap.end()){
        auto it2 = it1->second.find(pose2);
        if(it2 != it1->second.end()){
            auto it3 = it2->second.find(iRad);
            if(it3 != it2->second.end()){
                return it3->second;
            }
        }
    }
    if(checkBidirectional){
        auto it1 = m_pathsMap.find(pose2);
        if(it1 != m_pathsMap.end()){
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

PointVec PathsMapManager::getPathFromMap(const Pose2D &pose1, const Pose2D &pose2, const Machine& machine, bool checkBidirectional) const
{
    return getPathFromMap(pose1, pose2, machine.turning_radius, checkBidirectional);
}

void PathsMapManager::updatePaths(const PathsMapManager &from, bool replace)
{
    if(this == &from)
        return;

    std::scoped_lock<std::mutex, std::mutex> guard1(m_mutex, from.m_mutex);
    for(auto& it1 : from.m_pathsMap){
        m_pathsMap.insert_or_assign(it1.first, it1.second);
        for(auto& it2 : it1.second){
            for(auto& it3 : it2.second){
                if(replace)
                    m_pathsMap[it1.first][it2.first][it3.first] = it3.second;
                else
                    m_pathsMap[it1.first][it2.first].insert( std::make_pair(it3.first, it3.second) );
            }
        }
    }
}

void PathsMapManager::updatePaths(const PathsMapManagerConstPtr_t &from, bool replace)
{
    if(!from)
        return;
    updatePaths(*from, replace);
}

void PathsMapManager::movePaths(PathsMapManager &from, bool replace)
{
    if(this == &from)
        return;
    std::scoped_lock<std::mutex, std::mutex> guard1(m_mutex, from.m_mutex);
    for(auto& it1 : from.m_pathsMap){
        m_pathsMap.insert_or_assign(it1.first, it1.second);
        for(auto& it2 : it1.second){
            for(auto& it3 : it2.second){
                if(replace)
                    m_pathsMap[it1.first][it2.first][it3.first] = std::move(it3.second);
                else
                    m_pathsMap[it1.first][it2.first].insert( std::make_pair(it3.first, std::move(it3.second)) );
            }
        }
    }
    from.m_pathsMap.clear();
}

void PathsMapManager::movePaths(PathsMapManagerPtr_t &from, bool replace)
{
    if(!from)
        return;
    movePaths(*from, replace);
}

void PathsMapManager::movePathsMap(const PathsMapManager &from)
{
    if(this == &from)
        return;

    std::scoped_lock<std::mutex, std::mutex> guard1(m_mutex, from.m_mutex);
    m_pathsMap = std::move( from.m_pathsMap );
}

void PathsMapManager::movePathsMap(PathsMapManagerPtr_t &from)
{
    if(!from)
        return;
    return movePathsMap(*from);
}

}

}
