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
 
#ifndef AROLIB_ROUTE_SMOOTHER_HPP
#define AROLIB_ROUTE_SMOOTHER_HPP

#include <vector>
#include <map>
#include <fstream>

#include "arolib/types/machine.hpp"
#include "arolib/types/route.hpp"
#include "arolib/geometry/pathsmoother.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/container_helper.h"
#include "arolib/misc/basicconversions.hpp"

namespace arolib{

/**
 * @brief Route used to smoothen the routes (i.e. remove sharp corners and adjust inner-field inter-track (headland) connections to look earlike )
 */
class RouteSmoother : public LoggingComponent{

public:

    /**
     * @brief Strategy to deal with timestamps in the smoothen segments (and their subsequent route points)
     */
    enum TimeHandlingStrategy {
        KEEP_ELAPSED_TIME, /**< The smoothen segment will last the same time as the original segment */
        RECALC_PROP_TO_LENGTH /**< The timestamps of the route segment (and following route points) will be recalculated based on the length of the smoothen segment (the smoothen segment can be longer or shorter than the original one) */
    };
    static TimeHandlingStrategy intToTimeHandlingStrategy(int value);

    /**
     * @brief Constructor
     * @param machines Vector containing the machines that are assigned to the routes to be smoothen (machines with same id will be overwritten)
     * @param logLevel Log level
     */
    explicit RouteSmoother (LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Set the machines that are assigned to the routes to be smoothen. If a machine with the same id is already saved, it is replaced.
     * @param machines Vector containing the machines that are assigned to the routes to be smoothen (machines with same id will be overwritten)
     * @param removeCurrent If true, all currently saved machines will be removed
     */
    void setWorkingGroup(const std::vector<Machine> &machines, bool removeCurrent = true);

    /**
     * @brief Smoothen an OLV route.
     * @param [in/out] route Route to be smoothen.
     * @param THStrategy Strategy to deal with timestamps in the smoothen segments (and their subsequent route points)
     * @param onlyNonOverloadingSegments If true, the segments corresponding to overloading points will remain unchanged.
     * @return True on success.
     */
    bool smoothenOLVRoute(Route& route,
                          TimeHandlingStrategy THStrategy = RECALC_PROP_TO_LENGTH,
                          bool onlyNonOverloadingSegments = true) const;

    /**
     * @brief Smoothen an OLV route.
     * @param machine Machine.
     * @param [in/out] route Route to be smoothen.
     * @param THStrategy Strategy to deal with timestamps in the smoothen segments (and their subsequent route points)
     * @param onlyNonOverloadingSegments If true, the segments corresponding to overloading points will remain unchanged.
     * @return True on success.
     */
    bool smoothenOLVRoute(const Machine& machine,
                          Route& route,
                          TimeHandlingStrategy THStrategy = RECALC_PROP_TO_LENGTH,
                          bool onlyNonOverloadingSegments = true) const;

    /**
     * @brief Smoothen a harvester route.
     * @note The method to smooth the segments is selected automatically (Bezier or BSpline). In the case of to inner-field inter-track (headland) connection segments (i.e. these connection segments will not be smoothen to have a earlike-like shape)
     * @warning If smoothenHeadlandSegments was called on this route previously and smoothenHLSegments=true, the ear-like shape of the inner-field inter-track (headland) connection segments will be altered (it will be done twice, which is not desired).
     * @sa smoothenHeadlandSegments
     * @param [in/out] route Route to be smoothen.
     * @param boundary Boundary.
     * @param THStrategy Strategy to deal with timestamps in the smoothen segments (and their subsequent route points)
     * @param smoothenHLSegments If tre, the segments corresponding to inner-field inter-track (headland) connection points will be initially adjusted to have earlike-shapes (using smoothenHeadlandSegments).
     * @return True on success.
     */
    bool smoothenHarvesterRoute(Route& route,
                                const Polygon& boundary = Polygon(),
                                TimeHandlingStrategy THStrategy = RECALC_PROP_TO_LENGTH,
                                bool smoothenHLSegments = false) const;

    /**
     * @brief Smoothen a harvester route.
     * @note The method to smooth the segments is selected automatically (Bezier or BSpline). In the case of to inner-field inter-track (headland) connection segments (i.e. these connection segments will not be smoothen to have a earlike-like shape)
     * @warning If smoothenHeadlandSegments was called on this route previously and smoothenHLSegments=true, the ear-like shape of the inner-field inter-track (headland) connection segments will be altered (it will be done twice, which is not desired).
     * @sa smoothenHeadlandSegments
     * @param machine Machine.
     * @param [in/out] route Route to be smoothen.
     * @param boundary Boundary.
     * @param THStrategy Strategy to deal with timestamps in the smoothen segments (and their subsequent route points)
     * @param smoothenHLSegments If tre, the segments corresponding to inner-field inter-track (headland) connection points will be initially adjusted to have earlike-shapes (using smoothenHeadlandSegments).
     * @return True on success.
     */
    bool smoothenHarvesterRoute(const Machine& machine,
                                Route& route,
                                const Polygon& boundary = Polygon(),
                                TimeHandlingStrategy THStrategy = RECALC_PROP_TO_LENGTH,
                                bool smoothenHLSegments = false) const;


    /**
     * @brief Smoothen the inner-field inter-track (headland) connection segments of a 'working-type machine' route so that the connections have an ear-like shape (if applicable).
     * @warning This method was design for 'working-type machine' routes that were generated following the TrackConnectionStrategy=OPTIMAL_INTERTRACK_CONNECTION. The usage of other TrackConnectionStrategy may result in undesired route segments
     * @sa RouteAssembler::TrackConnectionStrategy
     * @param [in/out] route Route to be smoothen.
     * @param boundary Boundary.
     * @param THStrategy Strategy to deal with timestamps in the smoothen segments (and their subsequent route points)
     * @return True on success.
     */
    bool smoothenHeadlandSegments(Route& route,
                                  const Polygon& boundary = Polygon(),
                                  TimeHandlingStrategy THStrategy = RECALC_PROP_TO_LENGTH) const;


    /**
     * @brief Smoothen the inner-field inter-track (headland) connection segments of a 'working-type machine' route so that the connections have an ear-like shape (if applicable).
     * @warning This method was design for 'working-type machine' routes that were generated following the TrackConnectionStrategy=OPTIMAL_INTERTRACK_CONNECTION. The usage of other TrackConnectionStrategy may result in undesired route segments
     * @sa RouteAssembler::TrackConnectionStrategy
     * @param machine Machine.
     * @param [in/out] route Route to be smoothen.
     * @param boundary Boundary.
     * @param THStrategy Strategy to deal with timestamps in the smoothen segments (and their subsequent route points)
     * @return True on success.
     */
    bool smoothenHeadlandSegments(const Machine& machine,
                                  Route& route,
                                  const Polygon& boundary = Polygon(),
                                  TimeHandlingStrategy THStrategy = RECALC_PROP_TO_LENGTH) const;

    /**
     * @brief Smoothen a route filtering out segments with certain route-point types.
     * @param [in/out] route Route to be smoothen.
     * @param filter Route-point types to be filtered in/out from the smoothening.
     * @param filterOut if true, the types in filter will be taken as types to be excluded; if false, they are taken as the only types to be taken into account for the smoothening.
     * @param indFrom Index of the first route point.
     * @param indTo Index of the last route point (if <0 --> last route point in the route).
     * @param boundary Boundary.
     * @param THStrategy Strategy to deal with timestamps in the smoothen segments (and their subsequent route points)
     * @return True on success.
     */
    bool smoothenRoute(Route& route,
                       const std::set<RoutePoint::RoutePointType>& filter,
                       bool filterOut,
                       size_t indFrom = 0,
                       int indTo = -1,
                       const Polygon& boundary = Polygon(),
                       TimeHandlingStrategy THStrategy = RECALC_PROP_TO_LENGTH) const;

    /**
     * @brief Smoothen a route filtering out segments with certain route-point types.
     * @param machine Machine
     * @param [in/out] route Route to be smoothen.
     * @param filter Route-point types to be filtered in/out from the smoothening.
     * @param filterOut if true, the types in filter will be taken as types to be excluded; if false, they are taken as the only types to be taken into account for the smoothening.
     * @param indFrom Index of the first route point.
     * @param indTo Index of the last route point (if <0 --> last route point in the route).
     * @param boundary Boundary.
     * @param THStrategy Strategy to deal with timestamps in the smoothen segments (and their subsequent route points)
     * @return True on success.
     */
    bool smoothenRoute(const Machine& machine,
                       Route& route,
                       const std::set<RoutePoint::RoutePointType>& filter,
                       bool filterOut,
                       size_t indFrom = 0,
                       int indTo = -1,
                       const Polygon& boundary = Polygon(),
                       TimeHandlingStrategy THStrategy = RECALC_PROP_TO_LENGTH) const;


    /**
     * @brief Smoothen a segment of a route from a given point until a cut point.
     * @param machine Machine
     * @param [in/out] route Route to be smoothen.
     * @param cutFunct function returning if a given route point is the final point of the segment to be smoothen
     * @param indFrom Index of the first route point.
     * @param indTo Index of the last route point (if <0 --> last route point in the route).
     * @param boundary Boundary.
     * @param THStrategy Strategy to deal with timestamps in the smoothen segments (and their subsequent route points)
     * @return True on success.
     */
    bool smoothenRoute(const Machine& machine,
                       Route& route,
                       const std::function<bool(const RoutePoint&)>& cutFunct,
                       size_t indFrom = 0,
                       int indTo = -1,
                       const Polygon& boundary = Polygon(),
                       TimeHandlingStrategy THStrategy = RECALC_PROP_TO_LENGTH) const;


    /**
     * @brief Smoothen a segment of a route from a given point until a cut point.
     * @param machine Machine
     * @param [in/out] route Route to be smoothen.
     * @param cutFunct function returning if a given point of the route route is the final point of the segment to be smoothen (Note: the route send as parameter to cutFunct might have already smoothen segments, hence might not be the same input route of smoothenRoute)
     * @param indFrom Index of the first route point.
     * @param indTo Index of the last route point (if <0 --> last route point in the route).
     * @param boundary Boundary.
     * @param THStrategy Strategy to deal with timestamps in the smoothen segments (and their subsequent route points)
     * @return True on success.
     */
    bool smoothenRoute(const Machine& machine,
                       Route& route,
                       const std::function<bool (const Route&, size_t)> &cutFunct,
                       size_t indFrom = 0,
                       int indTo = -1,
                       const Polygon& boundary = Polygon(),
                       TimeHandlingStrategy THStrategy = RECALC_PROP_TO_LENGTH) const;



    /**
     * @brief For debugging purposes
     */
    void setDebugFile(const std::string& filename);

protected:

    /**
     * @brief Smoothen a route between two given indexes (i.e. smoothen ALL the 'sharp segments' between the two indexes)
     * @note The method to smooth the segments is selected automatically (Bezier or BSpline). This applies also to inner-field inter-track (headland) connection segments (i.e. these connection segments will not have a earlike-like shape)
     * @param [in/out] route Route to be smoothen.
     * @param route_ind0 Index corresponding to the first point
     * @param route_ind1 Index corresponding to the last point (inclusive)
     * @param machine Machine assigned to the route.
     * @param THStrategy Strategy to deal with timestamps in the smoothen segments (and their subsequent route points)
     * @param onlyNonOverloadingSegments If true, the segments corresponding to overloading points will remain unchanged.
     * @return True on success.
     */
    bool smoothenRoutePart(Route& route,
                           size_t route_ind0,
                           size_t route_indn,
                           const Machine& machine,
                           TimeHandlingStrategy THStrategy,
                           bool onlyNonOverloadingSegments = true) const;

    /**
     * @brief Smoothen a route between two given indexes (i.e. smoothen ALL the 'sharp segments' between the two indexes)
     * @note The method to smooth the segments is selected automatically (Bezier or BSpline). This applies also to inner-field inter-track (headland) connection segments (i.e. these connection segments will not have a earlike-like shape)
     * @param [in/out] route Route to be smoothen.
     * @param machine Machine assigned to the route.
     * @param route_ind0 Index corresponding to the first point
     * @param route_ind1 Index corresponding to the last point (inclusive)
     * @param THStrategy Strategy to deal with timestamps in the smoothen segments (and their subsequent route points)
     * @param onlyNonOverloadingSegments If true, the segments corresponding to overloading points will remain unchanged.
     * @return True on success.
     */
    bool smoothenRoutePart(Route& route,
                           const Machine& machine,
                           size_t route_ind0,
                           size_t route_indn,
                           TimeHandlingStrategy THStrategy) const;//new method

    /**
     * @brief Smoothen a complete route.
     * @note The method to smooth the segments is selected automatically (Bezier or BSpline). This applies also to inner-field inter-track (headland) connection segments (i.e. these connection segments will not have a earlike-like shape)
     * @param [in/out] route Route to be smoothen.
     * @param machine Machine assigned to the route.
     * @param THStrategy Strategy to deal with timestamps in the smoothen segments (and their subsequent route points)
     * @param onlyNonOverloadingSegments If true, the segments corresponding to overloading points will remain unchanged.
     * @return True on success.
     */
    bool smoothenRoute(Route& route,
                       const Machine& machine,
                       TimeHandlingStrategy THStrategy,
                       bool onlyNonOverloadingSegments = true) const;

    /**
     * @brief Smoothen a single route segment.
     * @note The method to smooth the segments is selected automatically (Bezier or BSpline). This applies also to inner-field inter-track (headland) connection segments (i.e. these connection segments will not have a earlike-like shape)
     * @param route Route to be smoothen.
     * @param ind0 Index corresponding to the first point of the segment
     * @param ind1 Index corresponding to the last point of the segment (inclusive)
     * @param machine Machine assigned to the route.
     * @param THStrategy Strategy to deal with timestamps in the smoothen segments (and their subsequent route points)
     * @param [out] smoothSegment Smoothen segment.
     * @param [out] delta_time Timestamp difference between the last route points of the original and the smoothe segment (if THStrategy=KEEP_ELAPSED_TIME, delta_time=0).
     * @return True on success.
     */
    bool smoothenSegment(const Route &route,
                         size_t ind_0,
                         size_t ind_n,
                         const Machine& machine,
                         TimeHandlingStrategy THStrategy,
                         std::vector<RoutePoint>& smoothSegment,
                         double& delta_time) const;

    /**
     * @brief Smoothen a single route segment.
     * @note The method to smooth the segments is selected automatically (Bezier or BSpline). This applies also to inner-field inter-track (headland) connection segments (i.e. these connection segments will not have a earlike-like shape)
     * @param route Route to be smoothen.
     * @param machine Machine
     * @param ind0 Index corresponding to the first point of the segment
     * @param ind1 Index corresponding to the last point of the segment (inclusive)
     * @param machine Machine assigned to the route.
     * @param THStrategy Strategy to deal with timestamps in the smoothen segments (and their subsequent route points)
     * @param [out] smoothSegment Smoothen segment.
     * @param [out] delta_time Timestamp difference between the last route points of the original and the smoothe segment (if THStrategy=KEEP_ELAPSED_TIME, delta_time=0).
     * @return True on success.
     */
    bool smoothenSegment(const Route &route,
                         const Machine& machine,
                         size_t ind_0,
                         size_t ind_n,
                         TimeHandlingStrategy THStrategy,
                         std::vector<RoutePoint>& smoothSegment,
                         double& delta_time) const;//new method

    /**
     * @brief Smoothen a segment segment with 4 points with only right angles (in the same direction).
     * @param points_in Input points (size must be 4).
     * @param radius Turning radius of the machine
     * @param [out] points_out Resulting points
     * @return True on success.
     */
    bool getEarFromRightAnglesSegment(const std::vector<Point> &points_in, const Machine &machine, std::vector<Point> &points_out) const;

    /**
     * @brief (deprecated) Smoothen a segment segment with 4 points with only right angles (in the same direction).
     * @param points_in Input points (size must be 4).
     * @param radius Turning radius of the machine
     * @param [out] points_out Resulting points
     * @return True on success.
     */
    bool getEarFromRightAnglesSegment_old(const std::vector<Point> &points_in, const Machine &machine, std::vector<Point> &points_out) const;

    /**
     * @brief Get the indexes of the two 'sharp' corners containing the first (starting from a given index) 'sharp' segment to be smoothen (i.e. the segment will be delimited by these 2 indexes).
     *
     * The last corner (ind1_s) is selected so that the (not short) segment of the route following this corner has no 'sharp' corners, hence the segment delimited by the corners might include one or more sharp corners (sesides the limits).
     * @param route_points Route-points.
     * @param machine Machine assigned to the route.
     * @param ind0 Index corresponding to the first route point of the (original) route segment/part to be checked.
     * @param ind1 Index corresponding to the last route point (inclusive) of the (original) route segment to be checked.
     * @param onlyNonOverloadingSegments If true, the segments corresponding to overloading points will be disregarded.
     * @param ind0_s Index corresponding to the first 'sharp' corner (i.e. lower limit of the next 'sharp' segment). If >= route_points.size() --> no corners found
     * @param ind1_s Index corresponding to the last 'sharp' corner (i.e. upper limit of the next 'sharp' segment). If >= route_points.size() --> no corners found
     * @return True on success.
     */
    bool getNextCornersIndexes( const std::vector<RoutePoint> &route_points,
                                const Machine &machine,
                                size_t ind0, size_t ind1,
                                bool onlyNonOverloadingSegments,
                                size_t &ind0_s, size_t &ind1_s, size_t &indn,
                                double &ang0_s, double &ang1_s) const;

    /**
     * @brief Get the indexes of the two 'sharp' corners containing the first (starting from a given index) 'sharp' segment to be smoothen (i.e. the segment will be delimited by these 2 indexes).
     *
     * The last corner (ind1_s) is selected so that the (not short) segment of the route following this corner has no 'sharp' corners, hence the segment delimited by the corners might include one or more sharp corners (sesides the limits).
     * @param route_points Route-points.
     * @param machine Machine assigned to the route.
     * @param ind0 Index corresponding to the first route point of the (original) route segment/part to be checked.
     * @param ind1 Index corresponding to the last route point (inclusive) of the (original) route segment to be checked.
     * @param ind0_s Index corresponding to the first 'sharp' corner (i.e. lower limit of the next 'sharp' segment). If >= route_points.size() --> no corners found
     * @param ind1_s Index corresponding to the last 'sharp' corner (i.e. upper limit of the next 'sharp' segment). If >= route_points.size() --> no corners found
     * @return True on success.
     */
    bool getNextCornersIndexes( const std::vector<RoutePoint> &route_points,
                                const Machine &machine,
                                size_t ind0, size_t ind1,
                                size_t &ind0_s, size_t &ind1_s, size_t &indn,
                                double &ang0_s, double &ang1_s ) const;//new method

    /**
     * @brief Smoothens a segment based on the turning radius of a machine
     *
     * @param points_us Unsamples input segment
     * @param [out] smoothPoints Smooth segment
     * @param machine Machine
     * @return True on success.
     */
    bool smoothenSegmentBasedOnRadius(const std::vector<Point> &points_us, std::vector<Point> &smoothPoints, const Machine &machine) const;

    /**
     * @brief Obtain the BSpline nodes to be used to generate the earlike-shaped inner-field inter-track (headland) connection segment
     * @param nodes Points corresponding to the original inner-field inter-track (headland) connection segment.
     * @param workingwidth Working width
     * @return BSpline nodes.
     */
    std::vector<Point> getBSplineAdjustedNodes(std::vector<Point> nodes, double workingwidth) const;

    /**
     * @brief Obtain the (interpolated) type of an intermediate route point (i.e. what is the type of the route-point based on the types of its (immediatly) previous and next route-points)
     * @param rp0 (immediatly) previous route-point
     * @param rp1 (immediatly) next route-point
     * @param p Pointer to the point corresponding to the location of the intermediate point (disregarded if = NULL)
     * @return Route-point type
     */
    static RoutePoint::RoutePointType getIntermediateRPType(const RoutePoint& rp0, const RoutePoint& rp1, RoutePoint::RoutePointType defaultWorkingType, const Point *p = nullptr);


    /**
     * @brief Projects the route point properties of a route segment into a smooth segment
     * @param segment Route segment
     * @param newPoints Smooth segment
     * @param THStrategy Strategy to deal with timestamps in the smoothen segments (and their subsequent route points)
     * @return Projected route segment
     */
    static std::vector<RoutePoint> projectRouteSegment(const std::vector<RoutePoint>& segment, const std::vector<Point>& newPoints , TimeHandlingStrategy THStrategy);

protected:
    std::map<MachineId_t, Machine> m_machines; /**< Machines assigned to the routes to be smoothen */

    static const double m_extraDistMult; /**< Multiplier/factor applied to the machine workingwidth to add extra routepoints during smoothing */
    static const double m_curveDistMult; /**< Multiplier/factor applied to the machine workingwidth to obtain the last (valid) corner of a 'sharp' segment (i.e. segments that must be smoothen). */

    std::string m_debugFilename = ""; /**< File name for debugging */
    mutable std::ofstream m_debugFile; /**< File for debugging */
};

}

#endif // AROLIB_ROUTE_SMOOTHER_HPP
