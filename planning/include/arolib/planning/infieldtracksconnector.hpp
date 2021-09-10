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
 
#ifndef ARO_INFIELDTRACKSCONNECTOR_HPP
#define ARO_INFIELDTRACKSCONNECTOR_HPP

#include <ctime>

#include "arolib/misc/loggingcomponent.h"
#include "arolib/types/pose2D.hpp"
#include "arolib/types/subfield.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/curves_helper.hpp"
#include "arolib/geometry/pathsmoother.hpp"

namespace arolib{

/**
 * @brief Interface class for infield-tracks connectors.
 */

class IInfieldTracksConnector : public LoggingComponent
{
protected:

    /**
     * @brief Constructor.
     * @param childName Name (class name) of the child
     * @param parentLogger Parent logger
     */
    explicit IInfieldTracksConnector(const std::string& childName, Logger* parentLogger = nullptr);

public:

    /**
     * @brief Get the connection path between to poses.
     * @param machine machine
     * @param pose_start Start pose
     * @param pose_end End pose
     * @param turningRad (if <0 uses turning rad of machine)
     * @param extraDist Distance to drive in direction of pose start(end before making the turning manouvers)
     * @param limitBoundary If valid, the path must lie inside this boundary
     * @param infieldBoundary Boundary if the inner field
     * @param headlands Available headlands
     * @param speed_start Speed of the machine on pose_start (disregarded if <= 0)
     * @param maxConnectionLength Limit of the connection length (disregarded if <= 0)
     * @return Connection path (empty on error)
     */
    virtual std::vector<Point> getConnection( const Machine& machine,
                                              const Pose2D& pose_start,
                                              const Pose2D& pose_end,
                                              double turningRad,
                                              double extraDist,
                                              const Polygon& limitBoundary,
                                              const Polygon& infieldBoundary,
                                              const Headlands& headlands,
                                              double speed_start,
                                              double maxConnectionLength) const = 0;

    /**
     * @brief Get the connection path between to poses.
     * @param sf Subfield
     * @param machine machine
     * @param pose_start Start pose
     * @param pose_end End pose
     * @param turningRad (if <0 uses turning rad of machine)
     * @param extraDist Distance to drive in direction of pose start(end before making the turning manouvers)
     * @param speed_start Speed of the machine on pose_start (disregarded if <= 0)
     * @param maxConnectionLength Limit of the connection length (disregarded if <= 0)
     * @return Connection path (empty on error)
     */
    virtual std::vector<Point> getConnection( const Subfield& sf,
                                              const Machine& machine,
                                              const Pose2D& pose_start,
                                              const Pose2D& pose_end,
                                              double turningRad = -1,
                                              double extraDist = -1,
                                              double speed_start = -1,
                                              double maxConnectionLength = -1) const;


    /**
     * @brief Get the connection path between a start pose and a track (linestring).
     * @param machine machine
     * @param pose_start Start pose
     * @param track track points
     * @param checkBothSides If true, the connections from the start pose to each side (extrema) of the track will be computed and the best one returned. If false, the connection will be done to the first point of the track.
     * @param turningRad (if <0 uses turning rad of machine)
     * @param extraDist Distance to drive in direction of pose start(end before making the turning manouvers)
     * @param limitBoundary If valid, the path must lie inside this boundary
     * @param infieldBoundary Boundary if the inner field
     * @param headlands Available headlands
     * @param speed_start Speed of the machine on pose_start (disregarded if <= 0)
     * @param maxConnectionLength Limit of the connection length (disregarded if <= 0)
     * @return Connection path (empty on error)
     */
    virtual std::vector<Point> getConnection( const Machine& machine,
                                              const Pose2D& pose_start,
                                              const std::vector<Point>& track,
                                              bool checkBothSides,
                                              double turningRad,
                                              double extraDist,
                                              const Polygon& limitBoundary,
                                              const Polygon& infieldBoundary,
                                              const Headlands& headlands,
                                              double speed_start = -1,
                                              double maxConnectionLength = -1) const;
};

/**
 * @brief Default Infield-tracks connector
 */
class InfieldTracksConnectorDef : public IInfieldTracksConnector
{
public:
    /**
     * @brief Constructor.
     * @param parentLogger Parent logger
     */
    explicit InfieldTracksConnectorDef(Logger* parentLogger = nullptr);

    /**
     * @brief Get the connection path between to poses.
     * @sa IInfieldTracksConnector::getConnection
     */
    virtual std::vector<Point> getConnection(const Machine& machine,
                                             const Pose2D& _pose_start,
                                             const Pose2D& _pose_end,
                                             double turningRad = -1,
                                             double extraDist = -1,
                                             const Polygon& _limitBoundary = Polygon(),
                                             const Polygon& infieldBoundary = Polygon(),
                                             const Headlands& headlands = Headlands(),
                                             double speed_start = -1,
                                             double maxConnectionLength = -1) const override;

protected:

    /**
     * @brief Check if a path is valid.
     * @param path Path
     * @param limitBoundary Overall (limit) boundary
     * @param infieldBoundary Inner-field boundary
     * @param maxConnectionLength Connection length limit
     * @return True if valid.
     */
    virtual bool isPathValid(const std::vector<Point>& path,
                             const Polygon& limitBoundary,
                             const Polygon& infieldBoundary,
                             double maxConnectionLength) const;

    /**
     * @brief Get a connection path between 2 poses (location + orientation) through a given set of tracks (linestrings).
     * @param pose_start Start pose
     * @param pose_end Target pose
     * @param turningRad turning radius
     * @param tracks Input tracks/linestrings
     * @param limitBoundary Overall (limit) boundary
     * @param infieldBoundary Inner-field boundary
     * @param maxConnectionLength Connection length limit
     * @return Connection path (empty on error)
     */
    virtual std::vector<Point> getConnectionOverTracks(const Pose2D& pose_start,
                                                        const Pose2D& pose_end,
                                                        double turningRad,
                                                        const std::vector<std::vector<Point> > &tracks,
                                                        const Polygon& limitBoundary,
                                                        const Polygon& infieldBoundary,
                                                        double maxConnectionLength) const;

    /**
     * @brief Get a connection path between 2 poses (location + orientation) through a complete (surrouding) headland.
     * @param hl Headland
     * @param pose_start Start pose
     * @param pose_end Target pose
     * @param turningRad turning radius
     * @param limitBoundary Overall (limit) boundary
     * @param infieldBoundary Inner-field boundary
     * @param maxConnectionLength Connection length limit
     * @return Connection path (empty on error)
     */
    virtual std::vector<Point> getConnectionOverHeadland( const CompleteHeadland& hl,
                                                          const Pose2D& pose_start,
                                                          const Pose2D& pose_end,
                                                          double turningRad,
                                                          const Polygon& limitBoundary,
                                                          const Polygon& infieldBoundary,
                                                          double maxConnectionLength) const;

    /**
     * @brief Get a connection path between 2 poses (location + orientation) over the area outside a boundary.
     * @param boundary Boundary
     * @param pose_start Start pose
     * @param pose_end Target pose
     * @param turningRad turning radius
     * @param limitBoundary Overall (limit) boundary
     * @param infieldBoundary Inner-field boundary
     * @param maxConnectionLength Connection length limit
     * @return Connection path (empty on error)
     */
    virtual std::vector<Point> getConnectionOutsideBoundary(Polygon boundary,
                                                             const Pose2D& pose_start,
                                                             const Pose2D& pose_end,
                                                             double turningRad,
                                                             const Polygon& limitBoundary,
                                                             const Polygon& infieldBoundary,
                                                             double maxConnectionLength) const;


};

}

#endif // ARO_INFIELDTRACKSCONNECTOR_HPP
