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
 
#ifndef AROLIB_ASTAR_PLAN_HPP
#define AROLIB_ASTAR_PLAN_HPP

#include <vector>

#include "arolib/types/route_point.hpp"
#include "arolib/planning/directedgraph.hpp"

namespace arolib{

/**
 * @brief Class holding the plan data yielded by the A* search
 */
class AstarPlan{

public:
    /**
     * @brief Constructor.
     */
    explicit AstarPlan() = default;

    /**
     * @brief Adjust the FIELD_ENTRY/FIELD_EXIT route-point types
     *
     * The planned route might use interchangeably the entry/exit types (i.e. a FIELD_ENTRY might be actually a FIELD_EXIT and viceversa), so we assume that the last entry/exit route point is an EXIT or ENTRY point, and from then they must be ENTRY/EXIT/ENTRY/EXIT/etc or EXIT/ENTRY/EXIT/ENTRY/etc...
     * @param lastAccessIsExist Treat last access point as exit (if true) or entry (if false)
    */
    void adjustAccessPoints(bool lastAccessIsExit);

    /**
     * @brief Update the graph data (edge overruns, vertex visiting periods, ...) with the current plan.
     * @param [in/out] graph Graph to be updated
     */
    void insertIntoGraph(DirectedGraph::Graph &graph) const;

    bool isOK = false;/**< Is the plan valid? */
    double plan_cost_ = 0;/**< Total plan costs (i.e. the costs used for planning)*/
    double plan_cost_total = 0;/**< Total costs for the plan including waiting costs (not necessarily the ones used for planning, i.e. might be different from plan_cost_)*/
    double waitTime = 0;  /**< Total amount of waiting time in the plan */
    size_t numCrossings = 0; /**< Total number of inner-field cross-edges in the plan */
    size_t numCrossings_HL = 0; /**< Total number of headland-harvesting cross-edges in the plan */
    std::vector<RoutePoint> route_points_; /**< Plan route points */
    std::vector<DirectedGraph::EdgeOverrun> edge_overruns_; /**< Plan edge overruns */
    std::vector< std::pair< DirectedGraph::vertex_t, DirectedGraph::VisitPeriod > > visit_periods; /**< Plan vertex visit periods */


};


}

#endif // AROLIB_ASTAR_PLAN_HPP
