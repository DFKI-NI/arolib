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
 
#ifndef _AROLIB_CURVES_HELPER_HPP_
#define _AROLIB_CURVES_HELPER_HPP_

#include <algorithm>
#include <set>
#include <functional>

#include "3rdParty/dubins/dubins.h"
#include "3rdParty/bezier/bezier.h"
#include "3rdParty/spline/splines.hpp"
#include "geometry_helper.hpp"
#include "arolib/types/pose2D.hpp"

namespace arolib{

namespace geometry{

/**
 * @brief Parameters to compute Dubins paths
 */
struct DubinsParams{
    /**
     * @brief Dubins path type
     *
     * L: left turn. R: right turn. S: straight
     */
    enum PathType {
        LSL, /**< left-straight-left */
        LSR, /**< left-straight-right */
        RSL, /**< right-straight-left */
        RSR, /**< right-straight-right */
        RLR, /**< right-left-right */
        LRL, /**< left-right-left */
        SHORTEST /**< Shortest path */
    };

    /**
     * @brief Get the PathType from its int value
     * @param value Its int value
     * @return PathType
     */
    static PathType intToPathType(int value);

    Point p1; /**< Source point */
    Point p2; /**< Target point */
    double rho1; /**< Sourse orientation/angle */
    double rho2;/**< Target orientation/angle */
    PathType type; /**< Path type */
};

/**
 * @brief Compute dubins path with given number of samples/points.
 * @param dp Input parameters
 * @param radius radius
 * @param resolution Distance between samples
 * @param [out] length Real length of the path (optional)
 * @return Path
 */
std::vector<Point> calcDubinsPath(const DubinsParams& dp, double radius, double resolution, double *length = nullptr);

/**
 * @brief Compute dubins path with given number of samples/points.
 * @param start start pose
 * @param finish end pose
 * @param radius radius
 * @param resolution Distance between samples
 * @param pathType Dubins path type
 * @param [out] length Real length of the path (optional)
 * @return Path
 */
std::vector<Point> calcDubinsPath(const Pose2D& start, const Pose2D& finish, double radius, double resolution, DubinsParams::PathType pathType = DubinsParams::SHORTEST, double *length = nullptr);

/**
 * @brief Compute dubins (unsampled) path with given angular resolution for the curves.
 * @param dp Input parameters
 * @param radius radius
 * @param angResolution Angular resolution to sample curves
 * @param inDeg Is the angular resolution in degrees
 * @param [out] length Real length of the path (optional)
 * @return Path
 */
std::vector<Point> calcDubinsPath(const DubinsParams& dp, double radius, double angResolution, bool inDeg, double *length = nullptr);

/**
 * @brief Compute dubins (unsampled) path with given angular resolution for the curves.
 * @param dp Input parameters
 * @param radius radius
 * @param angResolution Angular resolution to sample curves
 * @param inDeg Is the angular resolution in degrees
 * @param pathType Dubins path type
 * @param [out] length Real length of the path (optional)
 * @return Path
 */
std::vector<Point> calcDubinsPath(const Pose2D& start, const Pose2D& finish, double radius, double angResolution, bool inDeg, DubinsParams::PathType pathType = DubinsParams::SHORTEST, double *length = nullptr);

/**
 * @brief Compute dubins path with given number of samples/points.
 * @param numSamples number of samples
 * @param dp Input parameters
 * @param radius radius
 * @param [out] length Real length of the path (optional)
 * @return Path
 */
std::vector<Point> calcDubinsPath(size_t numSamples, const DubinsParams& dp, double radius, double *length = nullptr);

/**
 * @brief Compute dubins path with given number of samples/points.
 * @param numSamples number of samples
 * @param start start pose
 * @param finish end pose
 * @param radius radius
 * @param pathType Dubins path type
 * @param [out] length Real length of the path (optional)
 * @return Path
 */
std::vector<Point> calcDubinsPath(size_t numSamples, const Pose2D& start, const Pose2D& finish, double radius, DubinsParams::PathType pathType = DubinsParams::SHORTEST, double *length = nullptr);


/**
 * @brief Compute dubins path length.
 * @param dp Input parameters
 * @param radius radius
 * @return Real length of the path (<0 in error)
 */
double calcDubinsPathLength(const DubinsParams& dp, double radius);

/**
 * @brief Compute dubins path length.
 * @param start start pose
 * @param finish end pose
 * @param radius radius
 * @param pathType Dubins path type
 * @return Real length of the path (<0 in error)
 */
double calcDubinsPathLength(const Pose2D& start, const Pose2D& finish, double radius, DubinsParams::PathType pathType = DubinsParams::SHORTEST);


/**
 * @brief Get bezier-curve points for a set of nodes.
 * @param nodes Nodes
 * @param num_samples Number of samples for the output curve
 * @return Bezier-curve
 */
std::vector<Point> getBezierPoints(const std::vector<Point>& nodes, size_t num_samples);

/**
 * @brief Get bezier-curve points for a set of nodes.
 * @param nodes Nodes
 * @param samplesRef Set holding the (relative) location of the samples for the output curve [0,1]
 * @return Bezier-curve
 */
std::vector<Point> getBezierPoints(const std::vector<Point>& nodes, const std::multiset<double> &samplesRef);

/**
 * @brief Get BS-spline points for a set of nodes.
 * @param nodes Nodes
 * @param num_samples Number of samples for the output curve
 * @param order Order of the spline
 * @return BS-spline points
 */
std::vector<Point> getBSplinePoints(const std::vector<Point>& nodes, size_t num_samples, size_t order);

/**
 * @brief Get BS-spline points for a set of nodes.
 * @param nodes Nodes
 * @param samplesRef Set holding the (relative) location of the samples for the output curve [0,1]
 * @param order Order of the spline
 * @return BS-spline points
 */
std::vector<Point> getBSplinePoints(const std::vector<Point>& nodes, const std::multiset<double> &samplesRef, size_t order);


const size_t BezierMaxSampleSize = 20; /**< Maximum number of samples for Bezier curves */
}
}
#endif //_AROLIB_CURVES_HELPER_HPP_
