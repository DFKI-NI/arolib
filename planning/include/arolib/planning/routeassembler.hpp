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
 
#ifndef AROLIB_ROUTEASSEMBLER_HPP
#define AROLIB_ROUTEASSEMBLER_HPP

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <ctime>

#include "arolib/planning/planningworkspace.h"
#include "arolib/planning/aro_functions.hpp"
#include "arolib/planning/edgeMassCalculator.hpp"
#include "arolib/planning/edgeSpeedCalculator.hpp"
#include "arolib/planning/infieldtracksconnector.hpp"
#include "arolib/types/route.hpp"
#include "arolib/types/linestring.hpp"
#include "arolib/types/route_point.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/field_geometry_processing.hpp"
#include "arolib/cartography/common.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/logger.h"
#include "arolib/misc/container_helper.h"
#include "arolib/geometry/curves_helper.hpp"

namespace arolib{

/**
 * @brief Class used to assemble/create primary-machine routes for inner-field working
 * @sa HeadlandRouteAssembler
 */
class RouteAssembler : public LoggingComponent, protected PlanningWorkspaceAccessor
{

public:

    /**
     * @brief Constructor.
     * @param workingRPType Route-point working type
     * @param logLevel Log level
     */
    explicit RouteAssembler(RoutePoint::RoutePointType workingRPType, LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Constructor (loading an existent route as base).
     *
     * The methods will be applied to an existent route, instead of to a new empty route
     * @param route Initial route
     * @param logLevel Log level
     */
    explicit RouteAssembler(const Route& route, RoutePoint::RoutePointType workingRPType, LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Constructor.
     * @param machine_id Id of the machine for the created route
     * @param route_id Id for the created route
     * @param logLevel Log level
     */
    explicit RouteAssembler(MachineId_t machine_id, int route_id, RoutePoint::RoutePointType workingRPType, LogLevel logLevel = LogLevel::INFO);

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
     * @brief Adds a track to the current route using user define functions to compute the yield and speed of the edges, taking into account the subfield boundary
     * @param sf Subfield
     * @param _points Track points
     * @param track_id Id of the track to be added
     * @param reverse Flag that indicates that the points should be added in reversed order
     * @param [in/out*] edgeMassCalculator Mass calculator
     * @param edgeSpeedCalculator (in/out*) Speed calculator
     */
    void addTrack(const Subfield &sf,
                  const std::vector<Point>& _points,
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
     * @param boundary (optional) Subfield's inner boundary
     */
    void addTrack(double speed,
                  double default_yield,
                  const std::vector<Point>& _points,
                  int track_id,
                  bool reverse,
                  const Polygon &boundary = Polygon());

    /**
     * @brief Adds a track to the current route
     * @param speed Speed of the harvester [m/s]
     * @param default_yield Default yield proportion (t/ha)
     * @param l Linestring containing the track points
     * @param track_id Id of the track to be added
     * @param reverse Flag that indicates that the points should be added in reversed order
     * @param boundary (optional) Subfield's inner boundary
     */
    void addTrack(double speed,
                  double default_yield,
                  const Track& l,
                  int track_id,
                  bool reverse,
                  const Polygon &boundary = Polygon());

    /**
     * @brief Adds the connection between the last route point and a given next track
     * @param nextTrack Track to be added to the route after this headland connection
     * @param sf Subfield
     * @param speed speed
     * @param checkBothSides If false, only the connection to the first point of the track will be checked and generated
     * @param limitBoundary Limit outter boundary
     * @return True on success
     */
    bool addConnectionToTrack(const Track &nextTrack,
                               const Subfield& sf,
                               double speed,
                               bool checkBothSides,
                               const Polygon &limitBoundary);

    /**
     * @brief Adds the headland connection between two tracks (the last track added to the route, and a given next track)
     *
     * The headland-points in the subfield should have been generated already following the given strategy, and must have the right track ids
     *    For KEEP_TRACK_SEGMENT_DIRECTION, a KEEP_TRACK_SEGMENT_DIRECTION strategy must have been followed to generate the subfield's headland points
     *    For all other strategies, a MIN_DIST_TO_HEADLAND_SEGMENT must have been followed to generate the subfield's headland points
     * @param speed Speed of the harvester [m/s]
     * @param headland_part Headland-connection points
     * @param nextTrack Track to be added to the route after this headland connection
     * @param headland_size Number of points from headland_part to be used for the headland-connection
     * @param start_index Index of the fisrst point from headland_part to be used for the headland-connection
     * @param reverseIndexes If true, the points are obtained from headland_part (starting at start_index) in reverse order
     * @return True on success
     */
    bool addHeadlandPart(double speed,
                         std::vector<Point> headland_part,
                         const Track &nextTrack,
                         size_t headland_size,
                         size_t start_index,
                         bool reverseIndexes);


    /**
     * @brief Obtain the point (either first or last) in a given track whose connection path through the headland to the last (TRACK_END) route point of the current route is the shortest
     *
     * The headland-points in the subfield should have been generated already following the given strategy, and must have the right track ids
     *    - For KEEP_TRACK_SEGMENT_DIRECTION, a KEEP_TRACK_SEGMENT_DIRECTION strategy must have been followed to generate the subfield's headland points
     *    - For all other strategies, a MIN_DIST_TO_HEADLAND_SEGMENT must have been followed to generate the subfield's headland points
     * The last route point in the current trck must be a TRACK_END (i.e. the last operation was addTrack and not addHeadlandPart)
     * @param sf Processed subfield
     * @param nextTrack Track to be added to the route after this headland connection
     * @param [out] p_out Point (either first or last) in the given track whose connection path through the headland to the last route point is the shortest
     * @return True on success
     */
    bool getClosestPointInNextTrack(const Subfield& sf,
                                    const Track &nextTrack,
                                    Point &p_out);

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
    void setRoute(const Route& route){ m_route = route; }

    /**
     * @brief Get the current route
     * @return Current route
     */
    const Route& getRoute() const { return m_route; }

    /**
     * @brief Get the number of route-points in the current route
     * @return Number of route-points in the current route
     */
    size_t getRouteSize() const { return m_route.route_points.size(); }

    /**
     * @brief Get a route-point from the current route
     * @param index Index of the desired route-point
     * @return Desired route-point
     */
    RoutePoint getRoutePoint(size_t index) const;

    /**
     * @brief Set the machine and route ids
     * @param machine_id Machine id
     * @param route_id Route id
     */
    void setIDs(int machine_id, int route_id);

    /**
     * @brief Set the machine (must have a positive working width)
     * @return True on success
     */
    bool setMachine(const Machine& m);

    /**
     * @brief Set the tracks connector
     * @return True on success
     */
    void setTracksConnector(std::shared_ptr<IInfieldTracksConnector> tc);

protected:

    /**
     * @brief Adjust the headland-connection points (obtained following a MIN_DIST_TO_HEADLAND_SEGMENT) to obtain the shortest headland-connection possible
     *
     * This method assumes that the hl_points were obtained from headland-points that were created following a MIN_DIST_TO_HEADLAND_SEGMENT strategy
     * @param hl_points Headland-connection points (obtained following a MIN_DIST_TO_HEADLAND_SEGMENT)
     * @param prevPoints First segment (2 points) to be connected (i.e. last two points of the last added track). If less than 2 points are given, no adjustment on that side of the connection will be done.
     * @param nextPoints Second segment (2 points) to be connected (i.e. first two points of the next track to be added). If less than 2 points are given, no adjustment on that side of the connection will be done.
     * @return Resulting shortest headland-connection
     */
    std::vector<Point> adjustHeadlandPoints(const std::vector<Point> &hl_points,
                                            const std::vector<Point> &prevPoints,
                                            const std::vector<Point> &nextPoints);

    /**
     * @brief Adjust the track points making internal connections when the track is partially outside of the boundary
     * @param sf Subfield
     * @param [in/out] points Track points
     * @param [out] pointsOutside Points corresponding to the in-treck connections outside of the IF-boundary
     * @return True on success
     */
    bool adjustTracksPointsBasedOnBoundaries(const Subfield& sf, std::vector<Point> &points , const Machine &machine, std::set<Point> &pointsOutside);

protected:

    /**
     * @brief Flag used internally to know whether the plannning is being done using a Planning Workspace or not
     */
    enum CalcGridValueOption{
        CALC_DIRECT,
        CALC_PW
    };

    Route m_route; /**< Current route */
    Machine m_machine; /**< Machine */
    RoutePoint::RoutePointType m_workingRPType = RoutePoint::DEFAULT; /**< Route-point working type */

    std::shared_ptr<IInfieldTracksConnector> m_tracksConnector = std::make_shared<InfieldTracksConnectorDef>(); /**< IInfieldTracksConnector used to connect the tracks */

    CalcGridValueOption m_calcGridValueOption = CALC_DIRECT; /**< By default = CALC_DIRECT. Change to CALC_PW (and back) done by corresponding methods */
    PlanningWorkspace* m_planningWorkspace = nullptr; /**< Pointer to the planning workspace (if NULL, no methods with a planning workspace as parameter were called) */
};

}

#endif // AROLIB_ROUTEASSEMBLER_HPP
