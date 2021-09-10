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
 
#ifndef _AROLIB_HEADLANDROUTE_HPP
#define _AROLIB_HEADLANDROUTE_HPP

#include <algorithm>
#include "route_point.hpp"
#include "arolib/misc/datetime.hpp"
//#include "arolib/geometry/geometry_helper.hpp"

namespace arolib{

/**
  * @brief Route class for harvesting machines specifically for headland harvesting
  */
class HeadlandRoute{

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
     * @brief Obtain the (interpolated) point at a given time based on a given reference index
     *
     * Assumes that the route timestamps are monotonically increasing
     * @param time Desired time(-stamp)
     * @param [in/out] refIndex Reference index from which the search will start. Updated after the search with the index of the route point immediatly before the given time (ar at the same time).
     * @return (interpolated) Route-point at a given time
     */
    RoutePoint calcPoint2(double time, int &refIndex) const;

    /**
     * @brief Computes the (interpolated) speed at a given time
     *
     * Assumes that the route timestamps are monotonically increasing
     * @param time Desired time(-stamp)
     * @param refIndex (out, optional) Pointer to a variable where the reference index (the index of the route point immediatly before the given time (ar at the same time) ) will be saved. Set to -1 if invalid index
     * @return (interpolated) speed at a given time [m/s]
     */
    double calcSpeed(double time) const;

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
     * @brief Get the route-points as a vector of 'basic' points
     * @return Vector of 'basic' points
     */
    std::vector<Point> getPoints() const;


    /**
     * @brief Synchronizes the routes based on their base date-times.
     *
     * If one or more routes have no base date-time, no synch is done.
     * The new base time-date of all routes will be the lowest base date-time, and the timestamps will be adjusted accordingly.
     * @param [in/out] routes Vector of routes
     * @return True if synchronization was done.
     */
    static bool syncRoutes( std::vector<HeadlandRoute>& routes );

    /**
     * @brief Synchronizes the routes based on their base date-times.
     *
     * If one or more routes have no base date-time, no synch is done.
     * The new base time-date of all routes will be the lowest base date-time, and the timestamps will be adjusted accordingly.
     * @param [in/out] routes Vector of routes' pointers
     * @return True if synchronization was done.
     */
    static bool syncRoutes( const std::vector<HeadlandRoute *> &routes );

public:

    int subfield_id; /**< Id of the subfield related to the headland route */
    int machine_id; /**< Id of the machine assigned to the route */
    int route_id; /**< Unique route id */
    std::vector<RoutePoint> route_points; /**< Route points */
    std::string baseDateTime; /**< Base date-time in ISO_8601 */
};


inline std::ostream& operator<< (std::ostream &out, const arolib::HeadlandRoute& route) {
  out << "HeadlandRoute with " << route.route_points.size() << " points: [ ";
  
  int i = 0;
  for(const auto& point: route.getPoints())
  {
    if(i) out << ", ";
    out << point;
    i++;
  }

  out << " ]";
  return out;
}

inline std::ostream& operator<< (std::ostream &out, const std::vector<arolib::HeadlandRoute>& routes) {
  out << routes.size() << " routes: [ ";

  int i = 0;
  for(const auto& route: routes)
  {
    if(i) out << ", ";
    out << i << ": " << route.getPoints().size() << " points";
    i++;
  }

  out << " ]";
  return out;
}

}

#endif // _AROLIB_HEADLANDROUTE_HPP
