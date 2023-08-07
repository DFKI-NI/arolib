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
 
#ifndef _AROLIB_ROUTE_HPP
#define _AROLIB_ROUTE_HPP

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <ctime>
#include "arolib/types/linestring.hpp"
#include "arolib/types/route_point.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/types/track.hpp"
#include "arolib/types/coordtransformer.hpp"
#include "arolib/misc/datetime.hpp"

namespace arolib{

/**
 * @brief Route class
 */
class Route{

public:

    /**
     * @brief Obtain the (interpolated) route-point at a given time
     *
     * Assumes that the route timestamps are monotonically increasing
     * @param time Desired time(-stamp)
     * @param refIndex (out, optional) Pointer to a variable where the reference index (the index of the route point immediatly before the given time (ar at the same time) ) will be saved. Set to -1 if invalid index
     * @return (interpolated) Route-point at a given time
     */
    RoutePoint calcPoint(double time, int *refIndex = nullptr) const;

    /**
     * @brief Obtain the (interpolated) route-point at a given time
     *
     * Assumes that the route timestamps are monotonically increasing
     * @param route_points Route points
     * @param time Desired time(-stamp)
     * @param refIndex (out, optional) Pointer to a variable where the reference index (the index of the route point immediatly before the given time (ar at the same time) ) will be saved. Set to -1 if invalid index
     * @return (interpolated) Route-point at a given time
     */
    static RoutePoint calcPoint(const std::vector<RoutePoint>& route_points, double time, int *refIndex = nullptr);

    /**
     * @brief Obtain the (interpolated) point at a given time based on a given reference index
     *
     * Assumes that the route timestamps are monotonically increasing
     * @param time Desired time(-stamp)
     * @param [in/out] refIndex Reference index from which the search will start. Updated after the search with the index of the route point immediatly before the given time (ar at the same time).
     * @return (interpolated) Route-point at a given time
     */
    RoutePoint calcPoint2(double time, int &refIndex) const;

    /**
     * @brief Obtain the (interpolated) point at a given time based on a given reference index
     *
     * Assumes that the route timestamps are monotonically increasing
     * @param route_points Route points
     * @param time Desired time(-stamp)
     * @param [in/out] refIndex Reference index from which the search will start. Updated after the search with the index of the route point immediatly before the given time (ar at the same time).
     * @return (interpolated) Route-point at a given time
     */
    static RoutePoint calcPoint2(const std::vector<RoutePoint>& route_points, double time, int &refIndex);

    // TODO: Use polynomial interpolation in Route.calcSpeed instead of the current approach so there are no instant velocity changes
    /**
     * @brief Computes the (interpolated) speed at a given time
     *
     * Assumes that the route timestamps are monotonically increasing
     * @param time Desired time(-stamp)
     * @return (interpolated) speed at a given time [m/s]
     */
    double calcSpeed(double time) const;

    /**
     * @brief Computes the (interpolated) speed at a given time
     *
     * Assumes that the route timestamps are monotonically increasing
     * @param route_points Route points
     * @param time Desired time(-stamp)
     * @return (interpolated) speed at a given time [m/s]
     */
    static double calcSpeed(const std::vector<RoutePoint>& route_points, double time);

    /**
     * @brief Computes the (interpolated) speed at a given time
     *
     * Assumes that the route timestamps are monotonically increasing
     * @param time Desired time(-stamp)
     * @param [in/out] refIndex Reference index from which the search will start. Updated after the search with the index of the route point immediatly before the given time (ar at the same time).
     * @return (interpolated) speed at a given time [m/s]
     */
    double calcSpeed(double time, int &refIndex) const;

    /**
     * @brief Computes the (interpolated) speed at a given time
     *
     * Assumes that the route timestamps are monotonically increasing
     * @param route_points Route points
     * @param time Desired time(-stamp)
     * @param [in/out] refIndex Reference index from which the search will start. Updated after the search with the index of the route point immediatly before the given time (ar at the same time).
     * @return (interpolated) speed at a given time [m/s]
     */
    static double calcSpeed(const std::vector<RoutePoint>& route_points, double time, int &refIndex);

    /**
     * @brief Computes the (interpolated) bearing at a given time (with respect to the true north)
     *
     * Assumes that the route timestamps are monotonically increasing
     * @param time Desired time(-stamp)
     * @return (interpolated) bearing at a given time [degrees]
     */
    double calcBearing(double time) const;

    /**
     * @brief Computes the (interpolated) bearing at a given time (with respect to the true north)
     *
     * Assumes that the route timestamps are monotonically increasing
     * @param route_points Route points
     * @param time Desired time(-stamp)
     * @return (interpolated) bearing at a given time [degrees]
     */
    static double calcBearing(const std::vector<RoutePoint>& route_points, double time);

    /**
     * @brief Computes the (interpolated) bearing at a given time (with respect to the true north)
     *
     * Assumes that the route timestamps are monotonically increasing
     * @param time Desired time(-stamp)
     * @param [in/out] refIndex Reference index from which the search will start. Updated after the search with the index of the route point immediatly before the given time (ar at the same time).
     * @return (interpolated) bearing at a given time [degrees]
     */
    double calcBearing(double time, int &refIndex) const;

    /**
     * @brief Computes the (interpolated) bearing at a given time (with respect to the true north)
     *
     * Assumes that the route timestamps are monotonically increasing
     * @param route_points Route points
     * @param time Desired time(-stamp)
     * @param [in/out] refIndex Reference index from which the search will start. Updated after the search with the index of the route point immediatly before the given time (ar at the same time).
     * @return (interpolated) bearing at a given time [degrees]
     */
    static double calcBearing(const std::vector<RoutePoint>& route_points, double time, int &refIndex);

    /**
     * @brief Get the route-points as a vector of 'basic' points
     * @return Vector of 'basic' points
     */
    std::vector<Point> getPoints() const;

    /**
     * @brief Copy the route parameters (except for the route points) to another route
     * @param [out] other Route where the parameters will be copied
     * @param clearPoints If true, the route-points of the other route will be removed; otherwise, they will remain unchanged
     */
    void copyToWithoutPoints(Route& other, bool clearPoints = false) const;

    /**
     * @brief Synchronizes the routes based on their base date-times.
     *
     * If one or more routes have no base date-time, no synch is done.
     * The new base time-date of all routes will be the lowest base date-time, and the timestamps will be adjusted accordingly.
     * @param [in/out] routes Vector of routes
     * @return True if synchronization was done.
     */
    static bool syncRoutes( std::vector<Route>& routes );

    /**
     * @brief Synchronizes the routes based on their base date-times.
     *
     * If one or more routes have no base date-time, no synch is done.
     * The new base time-date of all routes will be the lowest base date-time, and the timestamps will be adjusted accordingly.
     * @param [in/out] routes Vector of routes' pointers
     * @return True if synchronization was done.
     */
    static bool syncRoutes( const std::vector<Route *> &routes );

    /**
     * @brief Get the index of the next route point that fullfills a rule.
     * @param startIdx Index from which to start the search
     * @param rule Rule function which returns true if the rule is fullfilled
     * @return Index of the next route point that fullfills a rule. -1 if none found
     */
    int getNextIndex( size_t startIdx, const std::function<bool(const RoutePoint& rp)> rule);

    /**
     * @brief Get the index of the next route point that fullfills a rule.
     * @param startIdx Index from which to start the search
     * @param rule Rule function which returns true if the rule is fullfilled. Paramater 1 = route Point to be checked; Paramater 2 = route point after.
     * @return Index of the next route point that fullfills a rule. -1 if none found
     */
    int getNextIndex( size_t startIdx, const std::function<bool(const RoutePoint& rp, const RoutePoint& rp_next)> rule);

    /**
     * @brief Get the index of the next route point that has type that matches any of the given types.
     * @param startIdx Index from which to start the search
     * @param RPType Route point types to be match
     * @return Index of the next route point that has type that matches any of the given types. -1 if none found
     */
    int getNextIndex(size_t startIdx, const std::set<RoutePoint::RoutePointType> &RPTypes);


    /**
     * @brief Obtain the (interpolated) type of an intermediate route point (i.e. what is the type of the route-point based on the types of its (immediatly) previous and next route-points)
     * @param rp0 (immediatly) previous route-point
     * @param rp1 (immediatly) next route-point
     * @param p Pointer to the point corresponding to the location of the intermediate point (disregarded if = NULL)
     * @return Route-point type
     */
    static RoutePoint::RoutePointType getIntermediateRPType(const RoutePoint& rp0, const RoutePoint& rp1, RoutePoint::RoutePointType defaultWorkingType = RoutePoint::DEFAULT, const Point *p = nullptr);


public:
    MachineId_t machine_id; /**< Id of the machine assigned to the route */
    int route_id; /**< Unique route id */
    std::vector<RoutePoint> route_points; /**< Route points */
    std::string baseDateTime; /**< Base date-time in ISO_8601 */

private:

    const static std::set<RoutePoint::RoutePointType> m_nonConsecutiveTypes; /**< Set of 'non-consecutive' route point types */
};

inline bool operator==(const Route& lhs, const Route& rhs) {
  return (lhs.machine_id == rhs.machine_id) &&\
         (lhs.route_points == rhs.route_points) &&\
        (lhs.baseDateTime == rhs.baseDateTime);
}

inline std::ostream& operator<< (std::ostream &out, const arolib::Route& route) {
  out << "Route with " << route.route_points.size() << " points";
  return out;
}

inline std::ostream& operator<< (std::ostream &out, const std::vector<arolib::Route>& routes) {
  out << routes.size() << " Routes: [ ";
  for(const auto& route: routes)
    out << route;
  out << " ]";
  return out;
}

}

#endif // _AROLIB_ROUTE_HPP
