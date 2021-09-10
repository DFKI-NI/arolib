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
 
#ifndef AROLIB_ROUNDTRIPPLANNER_HPP
#define AROLIB_ROUNDTRIPPLANNER_HPP

#include <ctime>

#include "directedgraph.hpp"
#include "astar.hpp"
#include "planningException.hpp"
#include "graphhelper.hpp"

#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/filesystem_helper.h"


namespace arolib{

/**
 * @brief Class to compute plans to go to a destination and back to the same route point
 */
class RoundtripPlanner : public LoggingComponent{

public:

    /**
     * @brief Planner parameters/settings
     *
     * Inherits from Astar::AStarSettings
     * @sa Astar::AStarSettings
     */
    struct PlannerSettings : public virtual Astar::AStarSettings{

        /**
         * @brief Parse the parameters from a string map, starting from a default PlannerSettings
         * @param [out] param Parameters
         * @param map String map containing the parameter values
         * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the default values will remain)
         * @return True on success
         */
        static bool parseFromStringMap( PlannerSettings& params, const std::map<std::string, std::string>& map, bool strict = false);

        /**
         * @brief Parse the parameters to a string map
         * @param param Parameters
         * @return String map containing the parameter values
         */
        static std::map<std::string, std::string> parseToStringMap( const PlannerSettings& params);
    };

    /**
     * @brief Constructor
     * @param machine Machine
     * @param settings Planner parameters/settings
     * @param edgeCostCalculator Edge Cost Calculator. Temporary: if = nullptr, uses internal astar functions
     * @param outputFolder Folder where the planning (search) information will be stored (if empty-string, no data will be saved)
     * @param logLevel Log level
     */
    explicit RoundtripPlanner(const Machine &machine,
                              const PlannerSettings &settings,
                              std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                              const std::string& outputFolder = "",
                              LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Plan next roundtrip from a given route point to the best (intermediate) destination, and back to the route point
     *
     * If the given route point is the last, the machine remains at the destination (one way trip)
     * @param graph Current graph
     * @param routeBase Base route
     * @param rp_index Route point index from which to plan the trip
     * @param rp_ret_index Index of the route point where the machine has to return (i.e. where the trip ends). If >= route_points size, it stays at the destination
     * @param destinationVertices Possible destination vertices
     * @param timeAtDestination Time the machine will spend at the (intermediate) destination
     * @param maxVisitTime_toDest Time constrain to visit vertices when planning to the destination (@sa AStar::PlanParameters max_time_visit)
     * @param maxVisitTime_toRoute Time constrain to visit vertices when planning back to the route (@sa AStar::PlanParameters max_time_visit)
     * @param restrictedMachineIds Ids of the machines for which the max_time_visit applies (@sa AStar::PlanParameters restrictedMachineIds)
     * @param functAtDest this function will be called to the last route point of the 'to the destination' route segment to get the first route point of the 'back to the route' segment
     * @return True on success
     */
    bool planTrip(const DirectedGraph::Graph &graph,
                  const Route &routeBase,
                  size_t rp_index,
                  size_t rp_ret_index,
                  const std::vector<DirectedGraph::vertex_t> & destinationVertices,
                  const std::set<DirectedGraph::vertex_t> & excludeVts_toDest,
                  const std::set<DirectedGraph::vertex_t> & excludeVts_toRoute,
                  double maxVisitTime_toDest,
                  double maxVisitTime_toRoute,
                  const std::set<MachineId_t> &restrictedMachineIds,
                  bool allowReverseDriving,
                  RoutePoint functAtDest(const RoutePoint& ));

    /**
     * @brief Get the updated graph
     * @return Updated graph
     */
    const DirectedGraph::Graph& getGraph() const;

    /**
     * @brief Get the (updated) route including the planned trip to the (intermediate) destination
     * @return (updated) route including the planned trip to the (intermediate) destination
     */
    const Route& getPlannedRoute() const;

    /**
     * @brief Get the index ranges of the (sub) trips from the (updated) route including the planned trip to the (intermediate) destination
     * @param indStart_toDest
     * @param indEnd_toDest
     * @param indStart_toRoute
     * @param indEnd_toRoute
     * @return True on success
     */
    bool getPlannedRouteIndexRanges(size_t& indStart_toDest,
                                    size_t& indEnd_toDest,
                                    size_t& indStart_toRoute,
                                    size_t& indEnd_toRoute ) const;

    /**
     * @brief Get the (updated) base route, excluding the planned trip to the (intermediate) destination
     *
     * The route will be the base route with the updated timestamp of the route points following the trip
     * @return (updated) base route, excluding the planned trip to the (intermediate) destination
     */
    Route getUpdatedBaseRoute();

    /**
     * @brief Get the total cost of the planned trip
     * @return Total cost of the planned trip
     */
    double getCost() const;

    /**
     * @brief Get the duration of the planned trip
     * @return Duration of the planned trip [s]
     */
    double getTavelDuration() const;

    /**
     * @brief Get the total amount of CROSS edges
     * @param [out] numCrossings Total amount of inner-field-harvesting CROSS edges
     * @param [out] numCrossings_HL Total amount of headland-harvesting CROSS edges
     * @return Total amount of CROSS edges (numCrossings + numCrossings_HL)
     */
    size_t getNumCrossings(size_t& numCrossings, size_t& numCrossings_HL) const;

    /**
     * @brief Check if a plan was found
     * @return True if a plan was found
     */
    bool foundPlan() const;

private:

    /**
     * @brief Class holding the relevant information of a planning state
     */
    struct PlanState{
        DirectedGraph::Graph graph; /**< Graph */
        size_t rp_index; /**< Route-point index (trip start) */
        size_t rp_ret_index; /**< Return route-point index */
        size_t rp_ret_index_new; /**< Return route-point index (new) */
        RoutePoint routePt; /**< Route-point */
        DirectedGraph::vertex_t routePtVt = -1; /**< Route-point vertex */
        RoutePoint routePt_ret; /**< Return route-point */
        DirectedGraph::vertex_t routePtVt_ret = -1; /**< Return route-point vertex */
        DirectedGraph::vertex_t destVt = -1; /**< Destination vertex */
        Route routeBase; /**< Base route */
        Route routeTrip; /**< (complete) planned route for the trip */
        Route routeUpdated; /**< Updated route with the planned trip */
        std::vector<Route> subRoutes;  /**< Planned route segments */
        double plan_cost = 0.0; /**< Total plan cost */
        double travel_time = 0.0; /**< Total duration [s] of the harvester planned travel routes */
        double planningDuration_toRoute = 0; /**< Time [s] needed to plan the route segment to arrive to the route point to continue the route */
        double planningDuration_toDest = 0; /**< Time [s] needed to plan the route segment to arrive to the (intermediate) destination */
        double planCost_toRoute = 0; /**< Cost of the planned route segment to arrive to the route point to continue harvesting */
        double planCost_toDest = 0; /**< Cost of the planned route segment to arrive to the (intermediate) destination */
        double planCost_atDest = 0; /**< Cost at the (intermediate) destination */
        size_t numCrossings = 0; /**< Total amount of inner-field-harvesting CROSS edges used */
        size_t numCrossings_HL = 0; /**< Total amount of headland-harvesting CROSS edges used */
        std::map<DirectedGraph::vertex_t, std::vector<DirectedGraph::VisitPeriod> > removedVisitPeriods; /**< Removed visit periods */


        /**
         * @brief Update the current cost
         */
        void updateCost();

        /**
         * @brief Add a route segment
         * @brief routePoints Route points
         */
        void addSubroute(const std::vector<RoutePoint>& routePoints);
    };

    PlanState m_state; /**< Current working planning state */
    PlannerSettings m_settings; /**< Planner parameters/settings */
    Machine m_machine; /**< Harvester machine */
    bool m_found_plan = false; /**< Flag indicating if a plan was found */
    std::shared_ptr<IEdgeCostCalculator> m_edgeCostCalculator = nullptr; /**< Edge cost calculator */

    std::string m_outputFolder = ""; /**< Folder where the planning (search) information will be stored (if empty-string, no data will be saved) */
    size_t m_countPlans = 0; /**< Counter of route-segment plans */

    /**
     * @brief Initializes the current planning state (m_state)
     * @param routeBase Base route
     * @param rp_index Index of the route point where the trip starts
     * @param rp_ret_index Index of the route point where the machine has to return (i.e. where the trip ends).
     */
    void initState(const DirectedGraph::Graph &graph, const Route &routeBase, size_t rp_index, size_t rp_ret_index);

    /**
     * @brief Plan the route-segment to the best destination vertex
     * @param [out] plan Plan results
     * @param destinationVertices Possible destination vertices
     * @param maxVisitTime Time constrain to visit vertices when planning to the destination (@sa AStar::PlanParameters max_time_visit)
     * @return True on success
     */
    bool planPathToDestination(AstarPlan &plan,
                               const std::vector<DirectedGraph::vertex_t> & destinationVertices,
                               double maxVisitTime,
                               const std::set<MachineId_t> &restrictedMachineIds,
                               const std::set<DirectedGraph::vertex_t> & excludeVts, bool allowReverseDriving);

    /**
     * @brief Plan the route-segment from the current vertex to the last route-point
     * @param [out] plan Plan results
     * @param timeAtDestination Time spent by the machine at the destination
     * @param maxVisitTime Time constrain to visit vertices when planning back to the route (@sa AStar::PlanParameters max_time_visit)
     * @return True on success
     */
    bool planPathToRoutePoint(AstarPlan &plan,
                              double maxVisitTime,
                              const std::set<MachineId_t> &restrictedMachineIds,
                              const std::set<DirectedGraph::vertex_t> & excludeVts);

    /**
     * @brief Updates the base routein m_state with the planned trip
     * @return True on success
     */
    bool updateRoute();

    /**
     * @brief Update the set of vertices to be excluded during the search, adding more necessary vertices when necessary
     * @param graph Current graph
     * @param [in/out] exclude Set of exclude vertices to be updated
     */
    void updateExcludeSet_toRP(std::set<DirectedGraph::vertex_t>& exclude,
                            bool allowReverseDriving) const;


    /**
     * @brief Restores the previously removed visiting periods adding to them the planned travel time.
     */
    void restoreVisitPeriods();


    /**
     * @brief Get the folder where the planning (search) information will be stored depending on the plan type
     * @param planType Plan type corresponding to the search within a overload plan (e.g. "to_adj", "to_harv", etc)
     */
    std::string getOutputFolder(const std::string& planType);

};

}

#endif // AROLIB_ROUNDTRIPPLANNER_HPP
