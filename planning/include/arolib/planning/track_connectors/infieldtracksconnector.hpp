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
    explicit IInfieldTracksConnector(const std::string& childName, std::shared_ptr<Logger> parentLogger = nullptr);

public:

    /**
     * @brief Get the connection path between two poses.
     * @param machine machine
     * @param pose_start Start pose
     * @param pose_end End pose
     * @param turningRad Turning radius (if <0 uses turning rad of machine)
     * @param extraDist Distances to drive in direction of pose start/end before making the turning manouvers
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
                                              const std::pair<double, double>& extraDist,
                                              const Polygon& limitBoundary,
                                              const Polygon& infieldBoundary,
                                              const Headlands& headlands,
                                              double speed_start = -1,
                                              double maxConnectionLength = -1) const = 0;

    /**
     * @brief Get the connection path between two poses.
     * @param sf Subfield
     * @param machine machine
     * @param pose_start Start pose
     * @param pose_end End pose
     * @param turningRad Turning radius (if <0 uses turning rad of machine)
     * @param extraDist Distances to drive in direction of pose start/end before making the turning manouvers
     * @param speed_start Speed of the machine on pose_start (disregarded if <= 0)
     * @param maxConnectionLength Limit of the connection length (disregarded if <= 0)
     * @return Connection path (empty on error)
     */
    virtual std::vector<Point> getConnection(const Subfield& sf,
                                             const Machine& machine,
                                             const Pose2D& pose_start,
                                             const Pose2D& pose_end,
                                             double turningRad = -1,
                                             const std::pair<double, double>& extraDist = std::make_pair(-1, -1),
                                             double speed_start = -1,
                                             double maxConnectionLength = -1) const;


    /**
     * @brief Get the connection path between a start pose and a track (linestring).
     * @param machine machine
     * @param pose_start Start pose
     * @param track track points
     * @param checkBothSides If true, the connections from the start pose to each side (extrema) of the track will be computed and the best one returned. If false, the connection will be done to the first point of the track.
     * @param turningRad Turning radius (if <0 uses turning rad of machine)
     * @param extraDist Distances to drive in direction of pose start/end before making the turning manouvers
     * @param limitBoundary If valid, the path must lie inside this boundary
     * @param infieldBoundary Boundary if the inner field
     * @param headlands Available headlands
     * @param speed_start Speed of the machine on pose_start (disregarded if <= 0)
     * @param maxConnectionLength Limit of the connection length (disregarded if <= 0)
     * @return Connection path (empty on error)
     */
    virtual std::vector<Point> getConnection(const Machine& machine,
                                             const Pose2D& pose_start,
                                             const std::vector<Point>& track,
                                             bool checkBothSides,
                                             double turningRad,
                                             const std::pair<double, double>& extraDist,
                                             const Polygon& limitBoundary,
                                             const Polygon& infieldBoundary,
                                             const Headlands& headlands,
                                             double speed_start = -1,
                                             double maxConnectionLength = -1) const;

    /**
     * @brief Get an appropiate limit boundary for the subfield and a given turning radius based on the subfield's geometries.
     * @param sf Subfield
     * @param turningRad Turning radius
     * @return Boundary
     */
    virtual Polygon getExtendedLimitBoundary(const Subfield& sf,
                                             double turningRad) const;

    /**
     * @brief Get the turning radius used for computations.
     * @param machine Machine
     * @param turningRad Turning radius (user given) (if <0 uses turning rad of machine)
     * @return Turning radius used for computations (if <0, no turning rad is used)
     */
    static double getTurningRad(const Machine& machine, double turningRad);



protected:


    /**
     * @brief Check if a path is valid.
     * @param path Path
     * @param limitBoundary Overall (limit) boundary
     * @param infieldBoundary Inner-field boundary
     * @param maxConnectionLength Connection length limit
     * @return True if valid.
     */
    static bool isPathValid(const std::vector<Point> &path, const Polygon &limitBoundary, const Polygon &infieldBoundary, double maxConnectionLength);

    /**
     * @brief Check if a path is valid.
     * @param path Path
     * @param limitBoundaries Limit boundaries
     * @param infieldBoundary Inner-field boundary
     * @param maxConnectionLength Connection length limit
     * @return True if valid.
     */
    static bool isPathValid(const std::vector<Point> &path, const std::vector<const Polygon *> &limitBoundaries, const Polygon &infieldBoundary, double maxConnectionLength);


    /**
     * @brief Get the infield boundary used for validation
     * @param infieldBoundary Inner-field boundary
     * @param p0 First path point
     * @param pn Last path point
     * @return infield boundary used for validation
     */
    static Polygon getInfieldBoundaryForValidation(const Polygon &infieldBoundary, const Point &p0, const Point &pn);


    /**
     * @brief Get the poses that will be used to compute the connection after applying the desired start- and end- straight driving distance
     * @param pose_start Start pose
     * @param pose_end End pose
     * @param extraDist Distances to drive in direction of pose start/end before making the turning manouvers
     * @param [out] pose_start_ed Resulting start pose
     * @param [out] pose_end Resulting end pose
     */
    virtual void getExtendedPoses(const Pose2D& pose_start, const Pose2D& pose_end, const std::pair<double, double>& extraDist,
                                  Pose2D& pose_start_ed, Pose2D& pose_end_ed) const;


    /**
     * @brief Check called at the beginning to know if the minimum possible length for the given parameters is higher than the given maximum connection length limit.
     *
     * This allows to disregard connections that are known to go over the maximum connection length limit before spending time computing the path.
     *
     * @param pose_start Start pose
     * @param pose_end End pose
     * @param turningRad Turning radius (if <0 uses turning rad of machine)
     * @param limitBoundary Overall (limit) boundary
     * @param infieldBoundary Inner-field boundary
     * @param maxConnectionLength Limit of the connection length (disregarded if <= 0)
     * @return False if the minimum possible length for the given parameters is higher than the given maximum connection length limit
     */
    virtual bool checkForMinPossibleLength(const Pose2D& pose_start,
                                           const Pose2D& pose_end,
                                           double turningRad,
                                           const Polygon& limitBoundary,
                                           const Polygon& infieldBoundary,
                                           double maxConnectionLength ) const;

};


}

#endif // ARO_INFIELDTRACKSCONNECTOR_HPP
