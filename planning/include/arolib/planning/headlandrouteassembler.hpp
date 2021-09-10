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
 
#ifndef AROLIB_HEADLANDROUTEASSEMBLER_HPP
#define AROLIB_HEADLANDROUTEASSEMBLER_HPP

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <ctime>

#include "arolib/types/linestring.hpp"
#include "arolib/types/route_point.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/cartography/common.hpp"
#include "arolib/planning/edgeMassCalculator.hpp"
#include "arolib/planning/edgeSpeedCalculator.hpp"
#include "planningworkspace.h"
#include "aro_functions.hpp"
#include "arolib/types/headlandroute.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/logger.h"

namespace arolib{

/**
 * @brief Class used to assemble/create primary-machine routes for headland working
 * @sa RouteAssembler
 */
class HeadlandRouteAssembler : public LoggingComponent, protected PlanningWorkspaceAccessor{

public:

    /**
     * @brief Constructor.
     * @param workingRPType Route-point working type
     * @param logLevel Log level
     */
    explicit HeadlandRouteAssembler(RoutePoint::RoutePointType workingRPType, const LogLevel &logLevel = LogLevel::INFO);

    /**
     * @brief Constructor (loading an existent route as base).
     *
     * The methods will be applied to an existent route, instead of to a new empty route
     * @param route Initial route
     * @param logLevel Log level
     */
    explicit HeadlandRouteAssembler(const HeadlandRoute& route, RoutePoint::RoutePointType workingRPType, const LogLevel &logLevel = LogLevel::INFO);

    /**
     * @brief Constructor.
     * @param machine_id Id of the machine for the created route
     * @param route_id Id for the created route
     * @param logLevel Log level
     */
    explicit HeadlandRouteAssembler(const int &machine_id, const int &route_id, RoutePoint::RoutePointType workingRPType, const LogLevel &logLevel = LogLevel::INFO);

    /**
     * @brief Adds a track to the current route using user define functions to compute the yield and speed of the edges
     * @param _points Track points
     * @param track_id Id of the track to be added
     * @param reverse Flag that indicates that the points should be added in reversed order
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param edgeSpeedCalculator (in/out*) Speed calculator
     */
    void addTrack(const std::vector<Point>& _points,
                  int track_id,
                  bool reverse,
                  const Machine& machine,
                  IEdgeMassCalculator & edgeMassCalculator,
                  IEdgeSpeedCalculator & edgeSpeedCalculator);

    /**
     * @brief Adds a track to the current route
     * @param speed Speed of the harvester [m/s]
     * @param default_yield Default yield proportion (t/ha)
     * @param _points Track points
     * @param track_id Id of the track to be added
     * @param reverse Flag that indicates that the points should be added in reversed order
     */
    void addTrack(double speed,
                  double default_yield,
                  const std::vector<Point>& _points,
                  int track_id,
                  bool reverse);

    /**
     * @brief Adds a track to the current route
     * @param speed Speed of the harvester [m/s]
     * @param default_yield Default yield proportion (t/ha)
     * @param l Linestring containing the track points
     * @param track_id Id of the track to be added
     * @param reverse Flag that indicates that the points should be added in reversed order
     */
    void addTrack(double speed,
                  double default_yield,
                  const Linestring& l,
                  int track_id,
                  bool reverse);


    /**
     * @brief Clears all data (route, machine id, route id, etc.)
     */
    void clear();

    /**
     * @brief Remove the current route-points
     */
    void clearRoutePoints(){ m_route.route_points.clear(); }

    /**
     * @brief Set the current route
     * @param route Route to be set
     */
    void setRoute(const HeadlandRoute& route){ m_route = route; }

    /**
     * @brief Get the current route
     * @return Current route
     */
    HeadlandRoute getRoute(){ return m_route; }

    /**
     * @brief Set the machine and route ids
     * @param machine_id Machine id
     * @param route_id Route id
     */
    void setIDs(int machine_id, int route_id);

    /**
     * @brief Set the working width (used for calculations, e.g. on the grids/maps)
     * @param workingWidth Working width [m]
     * @return True on success
     */
    bool setWorkingWidth(double workingWidth);


protected:

    /**
     * @brief Flag used internally to know whether the plannning is being done using a Planning Workspace or not
     */
    enum CalcGridValueOption{
        CALC_DIRECT,
        CALC_PW
    };

    HeadlandRoute m_route; /**< Current route */
    double m_workingWidth = 0; /**< Working width [m] */
    RoutePoint::RoutePointType m_workingRPType = RoutePoint::DEFAULT; /**< Route-point working type */
    CalcGridValueOption m_calcGridValueOption = CALC_DIRECT; /**< By default = CALC_DIRECT. Change to CALC_PW (and back) done by corresponding methods */
    PlanningWorkspace* m_planningWorkspace = nullptr; /**< Pointer to the planning workspace (if NULL, no methods with a planning workspace as parameter were called) */
};

}

#endif // AROLIB_HEADLANDROUTEASSEMBLER_HPP
