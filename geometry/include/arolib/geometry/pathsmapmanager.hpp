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
 
#ifndef AROLIB_PATHS_MAP_MANAGER_HPP
#define AROLIB_PATHS_MAP_MANAGER_HPP

#include <mutex>

#include "arolib/types/pose2D.hpp"
#include "arolib/types/machine.hpp"

namespace arolib{

namespace geometry{

class PathsMapManager;
using PathsMapManagerPtr_t = std::shared_ptr< PathsMapManager >;
using PathsMapManagerConstPtr_t = std::shared_ptr< const PathsMapManager >;

/**
 * @brief Class used to save and reuse computed paths
 */
class PathsMapManager{

public:

    /**
     * @brief Default constructor
     */
    PathsMapManager() = default;


    /**
     * @brief Copy constructor
     */
    PathsMapManager(const PathsMapManager& other){
        //the mutex must be a brand new one
        std::lock_guard<std::mutex> guard(other.m_mutex);
        m_pathsMap = other.m_pathsMap;
    }

    /**
     * @brief Move constructor
     */
    PathsMapManager(PathsMapManager&& other){
        std::lock_guard<std::mutex> guard(other.m_mutex);
        m_pathsMap = other.m_pathsMap;
    }

    /**
     * @brief Copy assignment
     */
    PathsMapManager& operator=(const PathsMapManager& other){
        if(this == &other)
            return *this;
        //the mutex must be a brand new one
        std::lock_guard<std::mutex> guard(other.m_mutex);
        m_pathsMap = other.m_pathsMap;
        return *this;
    }


    /**
     * @brief Move assignment
     */
    PathsMapManager& operator=(PathsMapManager&& other){
        if(this == &other)
            return *this;
        std::lock_guard<std::mutex> guard(other.m_mutex);
        m_pathsMap = std::move( other.m_pathsMap );
        return *this;
    }

    /**
     * @brief Add a computed path between two poses and a specific turning radius to the local paths' map.
     * @param pose1 Start pose.
     * @param pose2 End pose.
     * @param turningRad Turning radius.
     * @param Path
     */
    void addPathToMap(const Pose2D &pose1, const Pose2D &pose2, double turningRad, const PointVec& path);


    /**
     * @brief Clear the map containing the computed paths' between two poses and a specific turning radius.
     */
    void clearPathsMap();

    /**
     * @brief Get a computed path between two poses and a specific turning radius from the given paths' map.
     * @param map Paths' map.
     * @param pose1 Start pose.
     * @param pose2 End pose.
     * @param turningRad Turning radius.
     * @param checkBidirectional If true, it will also search for paths from pose2 to pose1 if no path from pose1 to pose2 was found.
     * @return Path (empty if not found)
     */
    static PointVec getPathFromMap(const PathsMapManager& map, const Pose2D& pose1, const Pose2D& pose2, double turningRad, bool checkBidirectional = true);

    /**
     * @brief Get a computed path between two poses and the turning radius of the given machine from the given paths' map.
     * @param map Paths' map.
     * @param pose1 Start pose.
     * @param pose2 End pose.
     * @param machine Machine (with the turning radius to be used for the search)
     * @param checkBidirectional If true, it will also search for paths from pose2 to pose1 if no path from pose1 to pose2 was found.
     * @return Path (empty if not found)
     */
    static PointVec getPathFromMap(const PathsMapManager& map, const Pose2D& pose1, const Pose2D& pose2, const Machine& machine, bool checkBidirectional = true);

    /**
     * @brief Get a computed path between two poses and a specific turning radius from the given paths' map.
     * @param map Paths' map.
     * @param pose1 Start pose.
     * @param pose2 End pose.
     * @param turningRad Turning radius.
     * @param checkBidirectional If true, it will also search for paths from pose2 to pose1 if no path from pose1 to pose2 was found.
     * @return Path (empty if not found)
     */
    static PointVec getPathFromMap(PathsMapManagerConstPtr_t map, const Pose2D& pose1, const Pose2D& pose2, double turningRad, bool checkBidirectional = true);

    /**
     * @brief Get a computed path between two poses and the turning radius of the given machine from the given paths' map.
     * @param map Paths' map.
     * @param pose1 Start pose.
     * @param pose2 End pose.
     * @param machine Machine (with the turning radius to be used for the search)
     * @param checkBidirectional If true, it will also search for paths from pose2 to pose1 if no path from pose1 to pose2 was found.
     * @return Path (empty if not found)
     */
    static PointVec getPathFromMap(PathsMapManagerConstPtr_t map, const Pose2D& pose1, const Pose2D& pose2, const Machine& machine, bool checkBidirectional = true);

    /**
     * @brief Get a computed path between two poses and a specific turning radius from the local paths' map.
     * @param pose1 Start pose.
     * @param pose2 End pose.
     * @param turningRad Turning radius.
     * @param checkBidirectional If true, it will also search for paths from pose2 to pose1 if no path from pose1 to pose2 was found.
     * @return Path (empty if not found)
     */
    PointVec getPathFromMap(const Pose2D& pose1, const Pose2D& pose2, double turningRad, bool checkBidirectional = true) const;

    /**
     * @brief Get a computed path between two poses and the turning radius of the given machine from the local paths' map.
     * @param pose1 Start pose.
     * @param pose2 End pose.
     * @param machine Machine (with the turning radius to be used for the search)
     * @param checkBidirectional If true, it will also search for paths from pose2 to pose1 if no path from pose1 to pose2 was found.
     * @return Path (empty if not found)
     */
    PointVec getPathFromMap(const Pose2D& pose1, const Pose2D& pose2, const Machine& machine, bool checkBidirectional = true) const;

    /**
     * @brief Updates the paths map adding/replacing the paths from the given manager.
     * @param from Source.
     * @param replace Replace If true, if there already exists a path, it will be replaced with the one from the source.
     */
    void updatePaths(const PathsMapManager& from, bool replace = true);

    /**
     * @brief Updates the paths map adding/replacing the paths from the given manager.
     * @param from Source.
     * @param replace Replace If true, if there already exists a path, it will be replaced with the one from the source.
     */
    void updatePaths(const PathsMapManagerConstPtr_t& from, bool replace = true);

    /**
     * @brief Updates the paths map adding/replacing the paths from the given manager. The paths are removed from the source.
     * @param from Source.
     */
    void movePaths(PathsMapManager& from, bool replace = true);

    /**
     * @brief Updates the paths map adding/replacing the paths from the given manager. The paths are removed from the source.
     * @param from Source.
     */
    void movePaths(PathsMapManagerPtr_t &from, bool replace = true);

    /**
     * @brief Makes a move operation on the (complete) paths map.
     * @param from Source.
     */
    void movePathsMap(const PathsMapManager& from);


    /**
     * @brief Makes a move operation on the (complete) paths map.
     * @param from Source.
     */
    void movePathsMap(PathsMapManagerPtr_t &from);


private:
    mutable std::mutex m_mutex; /**< Mutex for operations in the local paths' map */
    std::map< Pose2D /*startPose*/, std::map< Pose2D /*endPose*/, std::map< int /*adjustedTurningRad*/, PointVec /*path*/ > > > m_pathsMap;
};


}
}
#endif //AROLIB_PATHS_MAP_MANAGER_HPP
