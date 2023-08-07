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
 
#ifndef AROLIB_ROUTEASSEMBLER_HPP
#define AROLIB_ROUTEASSEMBLER_HPP

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <ctime>

#include "arolib/planning/planningworkspace.h"
#include "arolib/planning/aro_functions.hpp"
#include "arolib/planning/edge_calculators/edgeMassCalculator.hpp"
#include "arolib/planning/edge_calculators/edgeSpeedCalculator.hpp"
#include "arolib/planning/track_connectors/infieldtracksconnectordef.hpp"
#include "arolib/planning/track_sequencing/tracksequencer.hpp"
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
     * @param machine Machine
     * @param route_id Id for the created route
     * @param logLevel Log level
     */
    explicit RouteAssembler(const Machine &machine, int route_id, RoutePoint::RoutePointType workingRPType, LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Adds a track to the current route using user define functions to compute the yield and speed of the edges
     *
     * If the current route has points, a connection to the new track will be computed
     * @param _points Track points
     * @param track_id Id of the track to be added
     * @param checkBothSides If true, the connections to th 1st and last points o the track will be checked and the best one selected (inverting the track points accordingly)
     * @param limitBoundary Limit outter boundary for inter-track connection
     * @return True on success
     */
    bool addTrack(const std::vector<Point>& _points,
                  int track_id,
                  bool checkBothSides,
                  const Polygon &limitBoundary);

    /**
     * @brief Adds a track to the current route using user define functions to compute the yield and speed of the edges, taking into account the subfield boundary
     *
     * If the current route has points, a connection to the new track will be computed
     * @param sf Subfield
     * @param _points Track points
     * @param track_id Id of the track to be added
     * @param checkBothSides If true, the connections to th 1st and last points o the track will be checked and the best one selected (inverting the track points accordingly)
     * @param limitBoundary Limit outter boundary for inter-track connection
     * @return True on success
     */
    bool addTrack(const Subfield &sf,
                  const std::vector<Point>& _points,
                  int track_id,
                  bool checkBothSides,
                  const Polygon &limitBoundary);


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
     * @brief Set the tracks connector
     * @param tc tracks connector
     */
    void setTracksConnector(std::shared_ptr<IInfieldTracksConnector> tc);

    /**
     * @brief Set mass calculator
     * @param mc speed calculator
     */
    void setMassCalculator(std::shared_ptr<IEdgeMassCalculator> mc);

    /**
     * @brief Set the working-speed calculator
     * @param sc speed calculator
     */
    void setWorkingSpeedCalculator(std::shared_ptr<IEdgeSpeedCalculator> sc);

    /**
     * @brief Set the transit-speed calculator
     * @param sc speed calculator
     */
    void setTransitSpeedCalculator(std::shared_ptr<IEdgeSpeedCalculator> sc);

    /**
     * @brief Get the tracks connector
     * @return Tracks connector
     */
    std::shared_ptr<const IInfieldTracksConnector> getTracksConnector() const ;

    /**
     * @brief Set the tracks connections map
     * @param conns tracks connections map
     */
    void setTracksConnectionsMap(ITrackSequencer::PathsMapConstPtr_t conns);

protected:

    /**
     * @brief Adds the connection between the last route point and a given next track
     * @param nextTrack Track points to be added to the route after this headland connection
     * @param sf Subfield
     * @param checkBothSides If false, only the connection to the first point of the track will be checked and generated
     * @param limitBoundary Limit outter boundary
     * @return True on success
     */
    bool addConnectionToTrack(const std::vector<Point> &nextTrack,
                               const Subfield& sf,
                               bool checkBothSides,
                               const Polygon &limitBoundary);

    /**
     * @brief Adjust the track points making internal connections when the track is partially outside of the boundary
     * @param sf Subfield
     * @param [in/out] points Track points
     * @param [out] pointsOutside Points corresponding to the in-treck connections outside of the IF-boundary
     * @return True on success
     */
    bool adjustTracksPointsBasedOnBoundaries(const Subfield& sf, std::vector<Point> &points , const Machine &machine, std::set<Point> &pointsOutside);

protected:
    Route m_route; /**< Current route */
    Machine m_machine; /**< Machine */
    RoutePoint::RoutePointType m_workingRPType = RoutePoint::DEFAULT; /**< Route-point working type */

    std::shared_ptr<IInfieldTracksConnector> m_tracksConnector = std::make_shared<InfieldTracksConnectorDef>(); /**< IInfieldTracksConnector used to connect the tracks */
    std::shared_ptr<IEdgeMassCalculator> m_massCalculator = std::make_shared<EdgeMassCalculatorDef>(); /**< Mass calculator */
    std::shared_ptr<IEdgeSpeedCalculator> m_workingSpeedCalculator = std::make_shared<EdgeWorkingSpeedCalculatorDef>(); /**< Speed calculator for the tracks */
    std::shared_ptr<IEdgeSpeedCalculator> m_transitSpeedCalculator = std::make_shared<EdgeTransitSpeedCalculatorDef>(); /**< Speed calculator for the tracks' connection */

    ITrackSequencer::PathsMapConstPtr_t m_tracksConnectionsMap = nullptr;
};

}

#endif // AROLIB_ROUTEASSEMBLER_HPP
