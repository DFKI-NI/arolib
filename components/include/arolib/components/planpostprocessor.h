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
 
#ifndef AROLIB_PLANPOSTPROCESSOR_H
#define AROLIB_PLANPOSTPROCESSOR_H

#include <unistd.h>
#include <iostream>
#include <math.h>
#include <string>

#include "arolib/misc/basic_responses.h"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/container_helper.h"
#include "arolib/types/field.hpp"
#include "arolib/types/route.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/field_geometry_processing.hpp"
#include "arolib/geometry/curves_helper.hpp"


namespace arolib {

/**
 * @brief Class used to split given routes into segments
 */
class PlanPostProcessor : public LoggingComponent
{
public:
    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    PlanPostProcessor(const LogLevel &logLevel = LogLevel::INFO);

    /**
     * @brief Get the route segments splitted by route-point types.
     * @param route Route
     * @param stamps Route-point types used to split/segment the route
     * @param [out] segments Output route segments
     * @param includeRelatedMachines If set to true, it will also check the route-point types of the related machines of each route point
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp getSegmentedRoute( const Route& route,
                                     std::set<RoutePoint::RoutePointType> stamps,
                                     std::vector<Route>& segments,
                                     bool includeRelatedMachines = false);



    /**
     * @brief Adds routepoints between field entries/exits and resource points following the given external routes.
     *
     * The timestamps are of the route points to be connected are respected
     * @param routes [in/out] Routes to be edited
     * @param external_roads External roads
     * @param res Connection resolution [m]
     * @param distanceLimit If the distance between the points to be connected and the roads is higher than this value, no connection is added (disregarded if < 0)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp addOutfieldSegments(std::vector<Route>& routes,
                                const std::vector<Linestring>& external_roads,
                                double distanceLimit = -1,
                                double res = -1);

protected:
    /**
     * @brief Adds routepoints between the points at ind and ind+1.
     *
     * The timestamps are of the route points to be connected are respected
     * @param route [in/out] Route to be edited
     * @param ind RP index
     * @param connetion Connection points
     * @param RPType Type of the connection routepooints to be inserted
     * @return AroResp with error id (0:=OK) and message
     */
    bool addConnectionToRoute(Route& route,
                                 size_t ind,
                                 std::vector<Point> connection,
                                 RoutePoint::RoutePointType RPType);

protected:

};

}

#endif // AROLIB_PLANPOSTPROCESSOR_H
